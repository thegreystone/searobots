/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.*;

/**
 * Claude's torpedo controller. Three-phase approach:
 *
 * <ol>
 *   <li><b>Transit:</b> Head toward mission data target at full throttle.
 *       No pinging. Use passive sonar bearings to refine heading.</li>
 *   <li><b>Acquisition:</b> Near target area, start pinging. Use consecutive
 *       fixes to estimate target velocity. Steer toward intercept point.</li>
 *   <li><b>Terminal:</b> Within 300m, committed attack run. Hold steady
 *       course (can't turn fast at 25 m/s anyway).</li>
 * </ol>
 */
public class ClaudeTorpedoController implements TorpedoController {

    // Mission data from launch
    private double targetX, targetY, targetZ;
    private double targetHeading = Double.NaN, targetSpeed = 5.0;
    private boolean hasTarget;
    private TerrainMap terrain;

    // Target tracking from active pings
    private double prevFixX = Double.NaN, prevFixY = Double.NaN;
    private long prevFixTick = -1;
    private double estVelX = 0, estVelY = 0;

    // Phase management
    private enum Phase { TRANSIT, ACQUISITION, TERMINAL }
    private Phase phase = Phase.TRANSIT;

    private static final double MIN_TERRAIN_CLEARANCE = 30;
    private static final double ACQUISITION_RANGE = 800;  // start pinging at this distance
    private static final double TERMINAL_RANGE = 300;      // committed attack run

    @Override
    public void onLaunch(TorpedoLaunchContext context) {
        this.terrain = context.terrain();
        String data = context.missionData();
        if (data != null && !data.isBlank()) {
            try {
                var parts = data.split(",");
                if (parts.length >= 2) {
                    targetX = Double.parseDouble(parts[0].trim());
                    targetY = Double.parseDouble(parts[1].trim());
                    targetZ = parts.length >= 3 ? Double.parseDouble(parts[2].trim()) : -100;
                    if (parts.length >= 4) {
                        targetHeading = Double.parseDouble(parts[3].trim());
                    }
                    if (parts.length >= 5) {
                        targetSpeed = Double.parseDouble(parts[4].trim());
                    }
                    hasTarget = true;

                    // Pre-compute initial velocity estimate from mission data
                    if (!Double.isNaN(targetHeading) && targetSpeed > 0) {
                        estVelX = targetSpeed * Math.sin(targetHeading);
                        estVelY = targetSpeed * Math.cos(targetHeading);
                    }
                }
            } catch (NumberFormatException e) {
                // Could not parse
            }
        }
    }

    @Override
    public void onTick(TorpedoInput input, TorpedoOutput output) {
        output.setThrottle(1.0);

        var pos = input.self().position();
        double heading = input.self().heading();

        if (!hasTarget) {
            output.setRudder(0);
            output.setSternPlanes(0);
            return;
        }

        // Distance to current target estimate
        double dx = targetX - pos.x();
        double dy = targetY - pos.y();
        double dist = Math.sqrt(dx * dx + dy * dy);

        // Phase transitions
        if (phase == Phase.TRANSIT && dist < ACQUISITION_RANGE) {
            phase = Phase.ACQUISITION;
        }
        if (phase == Phase.ACQUISITION && dist < TERMINAL_RANGE) {
            phase = Phase.TERMINAL;
        }

        // ── Phase-specific sonar behavior ──
        switch (phase) {
            case TRANSIT -> {
                // Silent running: use passive sonar only.
                // If we hear something ahead, adjust course toward it.
                usePassiveContacts(input, pos, heading);
            }
            case ACQUISITION -> {
                // Go active: ping when ready, build intercept solution
                if (input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }
                processActiveReturns(input, pos, heading);
            }
            case TERMINAL -> {
                // Keep pinging for last-second corrections
                if (input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }
                processActiveReturns(input, pos, heading);
            }
        }

        // ── Compute intercept point ──
        double torpSpeed = Math.max(input.speed(), 5);
        double timeToIntercept = dist / torpSpeed;

        // Cap velocity magnitude to reject garbage estimates
        double velMag = Math.sqrt(estVelX * estVelX + estVelY * estVelY);
        double cappedVelX = estVelX, cappedVelY = estVelY;
        if (velMag > 15) { // no sub goes faster than 15 m/s
            double scale = 15 / velMag;
            cappedVelX *= scale;
            cappedVelY *= scale;
        }
        // Cap lead distance to 40% of current distance
        double maxLead = dist * 0.4;
        double leadTime = Math.min(timeToIntercept, maxLead / Math.max(velMag, 0.1));

        double interceptX = targetX + cappedVelX * leadTime;
        double interceptY = targetY + cappedVelY * leadTime;

        // ── Proportional Navigation (PN) steering ──
        // Instead of pointing at the intercept point, we steer to keep the
        // line-of-sight rate near zero (constant bearing = collision course).
        double idx = interceptX - pos.x();
        double idy = interceptY - pos.y();
        double bearingToIntercept = Math.atan2(idx, idy);
        if (bearingToIntercept < 0) bearingToIntercept += 2 * Math.PI;

        double headingError = bearingToIntercept - heading;
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        if (phase == Phase.TERMINAL && dist < 150) {
            // Final approach: gentle corrections only (avoid oscillation at close range)
            output.setRudder(Math.clamp(headingError * 0.8, -0.5, 0.5));
        } else {
            output.setRudder(Math.clamp(headingError * 2.0, -1, 1));
        }

        // ── Depth control ──
        double minDepth = -20; // never breach
        double desiredZ;

        if (phase == Phase.TERMINAL) {
            // Terminal: hold current depth (don't chase depth changes at close range)
            desiredZ = pos.z();
        } else if (phase == Phase.ACQUISITION && prevFixTick >= 0) {
            // Have active fix: adjust toward target depth
            double depthError = targetZ - pos.z();
            desiredZ = pos.z() + depthError * 0.05;
        } else {
            // Transit: gradually approach target depth from mission data
            double cruiseTarget = Math.max(targetZ, -200);
            double depthError = cruiseTarget - pos.z();
            desiredZ = pos.z() + depthError * 0.015;
        }

        // Terrain avoidance (skip in terminal to avoid last-second pull-up)
        if (terrain != null && phase != Phase.TERMINAL) {
            double floor = terrain.elevationAt(pos.x(), pos.y());
            double safeZ = floor + MIN_TERRAIN_CLEARANCE;
            double lookAhead = input.speed() * 3;
            double aheadFloor = terrain.elevationAt(
                    pos.x() + Math.sin(heading) * lookAhead,
                    pos.y() + Math.cos(heading) * lookAhead);
            safeZ = Math.max(safeZ, aheadFloor + MIN_TERRAIN_CLEARANCE + 10);
            if (desiredZ < safeZ) desiredZ = safeZ;
        }

        desiredZ = Math.min(desiredZ, minDepth);
        double dz = desiredZ - pos.z();
        double pitchGain = phase == Phase.TERMINAL ? 0.04 : 0.03;
        double pitchMax = phase == Phase.TERMINAL ? 0.4 : 0.3;
        output.setSternPlanes(Math.clamp(dz * pitchGain, -pitchMax, pitchMax));

        // Publish intercept point for viewer
        output.publishTarget(interceptX, interceptY, targetZ);
    }

    /** Use passive sonar during transit to refine bearing to target. */
    private void usePassiveContacts(TorpedoInput input, Vec3 pos, double heading) {
        if (input.sonarContacts().isEmpty()) return;

        // Pick the loudest forward contact (likely our target)
        SonarContact best = pickForwardContact(input.sonarContacts(), heading);
        if (best != null && best.signalExcess() > 3) {
            // Blend passive bearing into our target estimate.
            // Passive gives bearing only, so project at current estimated range.
            double dx = targetX - pos.x();
            double dy = targetY - pos.y();
            double currentDist = Math.sqrt(dx * dx + dy * dy);

            double passiveX = pos.x() + Math.sin(best.bearing()) * currentDist;
            double passiveY = pos.y() + Math.cos(best.bearing()) * currentDist;

            // Gentle blend: passive bearing is good, range is just our estimate
            targetX = targetX * 0.9 + passiveX * 0.1;
            targetY = targetY * 0.9 + passiveY * 0.1;
        }
    }

    /** Process active sonar returns for precise targeting. */
    private void processActiveReturns(TorpedoInput input, Vec3 pos, double heading) {
        if (input.activeSonarReturns().isEmpty()) return;

        SonarContact best = pickForwardContact(input.activeSonarReturns(), heading);
        if (best == null) return;

        double fixX = pos.x() + Math.sin(best.bearing()) * best.range();
        double fixY = pos.y() + Math.cos(best.bearing()) * best.range();

        // Estimate target velocity from consecutive fixes
        if (!Double.isNaN(prevFixX) && prevFixTick >= 0) {
            double dtFix = (input.tick() - prevFixTick) / 50.0;
            if (dtFix > 0.5 && dtFix < 20) {
                double rawVX = (fixX - prevFixX) / dtFix;
                double rawVY = (fixY - prevFixY) / dtFix;
                // Smooth: trust new data more once we have active fixes
                estVelX = estVelX * 0.3 + rawVX * 0.7;
                estVelY = estVelY * 0.3 + rawVY * 0.7;
            }
        }

        prevFixX = fixX;
        prevFixY = fixY;
        prevFixTick = input.tick();

        // Update target position from active return
        targetX = fixX;
        targetY = fixY;
        if (!Double.isNaN(best.estimatedDepth())) {
            targetZ = best.estimatedDepth();
        }
    }

    /** Pick the active/passive return that's most forward of our heading. */
    private SonarContact pickForwardContact(java.util.List<SonarContact> returns, double heading) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (var c : returns) {
            if (c.range() <= 0 && c.isActive()) continue; // skip zero-range active
            double angleDiff = c.bearing() - heading;
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
            double forwardness = Math.cos(angleDiff);
            double score = forwardness * 100 + c.signalExcess() * 0.5;
            if (c.isActive() && c.range() > 0) score += 50; // prefer active fixes
            if (score > bestScore) { bestScore = score; best = c; }
        }
        return best;
    }
}
