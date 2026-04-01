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
 *   <li><b>Terminal:</b> Within 800m, slow down for tight turning.
 *       True proportional navigation using line-of-sight rate.</li>
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
    private static final double ACQUISITION_RANGE = 2500;
    private static final double TERMINAL_RANGE = 800;

    // Proportional navigation: track bearing rate to target
    private double prevBearingToTarget = Double.NaN;
    private long prevBearingTick = -1;

    // PID depth controller
    private double depthIntegral;
    private double prevDepthError;
    private double launchDepth = Double.NaN; // remember where we started

    @Override
    public void onLaunch(TorpedoLaunchContext context) {
        this.terrain = context.terrain();
        this.launchDepth = context.launchPosition().z();
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
        var pos = input.self().position();
        double heading = input.self().heading();

        if (!hasTarget) {
            output.setThrottle(1.0);
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

        // ── Throttle management ──
        // Adaptive speed: stay fast when heading is aligned (tail chase),
        // slow down when heading error is large (need to turn for crossing shots).
        // This maximizes closing speed in tail chases while enabling tight
        // turns in beam/crossing engagements.
        double headingToTarget = Math.atan2(dx, dy);
        if (headingToTarget < 0) headingToTarget += 2 * Math.PI;
        double headingErr = headingToTarget - heading;
        while (headingErr > Math.PI) headingErr -= 2 * Math.PI;
        while (headingErr < -Math.PI) headingErr += 2 * Math.PI;
        double absErr = Math.abs(headingErr);

        // turnNeed: 0 when on course, 1 when 45+ degrees off
        double turnNeed = Math.clamp(absErr / Math.toRadians(45), 0, 1);

        // Adaptive speed: two competing needs.
        //
        // 1. Turn authority: slow down when heading error is large (turn radius ~ v^2)
        double turnSpeed = 23.0 - turnNeed * 11.0; // 23 on course, 12 turning hard
        //
        // 2. Closing speed: must be faster than the target's escape speed.
        //    Target's speed along our bearing (how fast it's running from us):
        double bearingX = dx / Math.max(dist, 1), bearingY = dy / Math.max(dist, 1);
        double targetAwaySpeed = Math.max(estVelX * bearingX + estVelY * bearingY, 0);
        double closingFloor = targetAwaySpeed + 8.0; // always 8 m/s faster than target retreat
        //
        // Take the max: never sacrifice closing speed, but slow for turns when we can.
        double goalSpeed = Math.clamp(Math.max(turnSpeed, closingFloor), 12.0, 23.0);
        if (phase == Phase.TRANSIT) goalSpeed = 23.0;
        double speedError = goalSpeed - input.speed();
        double throttle = Math.clamp(speedError * 0.15 + 0.5, 0.1, 1.0);
        output.setThrottle(throttle);

        // ── Sonar: passive every tick, active when cooldown allows ──
        // Passive bearings update the target bearing continuously (every tick).
        // Active pings give range + precise position (every 5 seconds).
        // Between pings, passive tracking keeps the PN guidance fed with
        // fresh bearing data instead of flying on stale predictions.
        usePassiveContacts(input, pos, heading);
        processActiveReturns(input, pos, heading);

        // Active ping scheduling
        switch (phase) {
            case TRANSIT -> {
                // Start pinging when within 3000m of estimated target
                if (dist < 3000 && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }
            }
            case ACQUISITION, TERMINAL -> {
                if (input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }
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
        // Lead distance: full time-to-intercept lead, capped at 60% of range
        double maxLead = dist * 0.6;
        double leadTime = Math.min(timeToIntercept, maxLead / Math.max(velMag, 0.1));

        double interceptX = targetX + cappedVelX * leadTime;
        double interceptY = targetY + cappedVelY * leadTime;

        // ── Steering ──
        // Bearing to intercept point
        double idx = interceptX - pos.x();
        double idy = interceptY - pos.y();
        double bearingToIntercept = Math.atan2(idx, idy);
        if (bearingToIntercept < 0) bearingToIntercept += 2 * Math.PI;

        // Bearing directly to target (for PN guidance)
        double bearingToTarget = Math.atan2(dx, dy);
        if (bearingToTarget < 0) bearingToTarget += 2 * Math.PI;

        if (phase == Phase.TERMINAL) {
            // True Proportional Navigation: steer proportional to LOS rate.
            // a_cmd = N * V_closing * (dLOS/dt)
            // This naturally leads the target without explicit intercept geometry.
            double dt = input.deltaTimeSeconds();
            double rudderCmd;

            if (!Double.isNaN(prevBearingToTarget) && prevBearingTick >= 0) {
                double dtBearing = (input.tick() - prevBearingTick) / 50.0;
                if (dtBearing > 0.01) {
                    double dBearing = bearingToTarget - prevBearingToTarget;
                    while (dBearing > Math.PI) dBearing -= 2 * Math.PI;
                    while (dBearing < -Math.PI) dBearing += 2 * Math.PI;
                    double losRate = dBearing / dtBearing; // rad/s

                    // PN gain N=4 (aggressive: typical range 3-5)
                    double closingSpeed = Math.max(torpSpeed - velMag * 0.5, 5);
                    double pnAccel = 4.0 * closingSpeed * losRate;
                    // Convert lateral acceleration to rudder command
                    // At speed v, rudder 1.0 gives ~v/turnRadius lateral accel
                    rudderCmd = Math.clamp(pnAccel * 0.08, -1, 1);

                    // Blend with pursuit guidance at very close range
                    // (PN can oscillate when LOS rate is noisy at close range)
                    if (dist < 150) {
                        double headErr = bearingToIntercept - heading;
                        while (headErr > Math.PI) headErr -= 2 * Math.PI;
                        while (headErr < -Math.PI) headErr += 2 * Math.PI;
                        double pursuit = Math.clamp(headErr * 3.0, -1, 1);
                        double blend = Math.clamp((dist - 50) / 100.0, 0, 1);
                        rudderCmd = rudderCmd * blend + pursuit * (1 - blend);
                    }
                } else {
                    // No valid dt, fall back to pursuit
                    double headErr = bearingToIntercept - heading;
                    while (headErr > Math.PI) headErr -= 2 * Math.PI;
                    while (headErr < -Math.PI) headErr += 2 * Math.PI;
                    rudderCmd = Math.clamp(headErr * 3.0, -1, 1);
                }
            } else {
                // First tick in terminal, use pursuit
                double headErr = bearingToIntercept - heading;
                while (headErr > Math.PI) headErr -= 2 * Math.PI;
                while (headErr < -Math.PI) headErr += 2 * Math.PI;
                rudderCmd = Math.clamp(headErr * 3.0, -1, 1);
            }

            prevBearingToTarget = bearingToTarget;
            prevBearingTick = input.tick();
            output.setRudder(rudderCmd);
        } else {
            // Transit/Acquisition: aim at intercept point
            double headingError = bearingToIntercept - heading;
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            output.setRudder(Math.clamp(headingError * 2.5, -1, 1));
        }

        // ── Depth control (PID) ──
        // Depth profile: start shallow (terrain clearance), then smoothly
        // converge to target depth as we close in. This naturally avoids
        // terrain early and reaches the target's depth for the kill.
        double minDepth = -20; // never breach

        // Depth profile depends on range:
        // Close range (<800m): go straight for target depth (no time to be strategic)
        // Long range (>800m): cruise slightly above target for terrain clearance,
        //   then blend down as we close.
        double goalZ;
        if (dist < 800) {
            // Close: direct approach to target depth
            goalZ = targetZ;
        } else {
            // Far: cruise above target depth for terrain clearance.
            // Limit the climb: never more than 150m above launch depth
            // (prevents extreme climbs that breach the surface).
            double cruiseZ = Math.min(targetZ + 30, -60);
            if (!Double.isNaN(launchDepth)) {
                double maxClimb = launchDepth + 150;
                cruiseZ = Math.min(cruiseZ, maxClimb);
            }
            // Blend from cruise to target over the 800-2000m range
            double t = Math.clamp((dist - 800) / 1200.0, 0, 1); // 1 at 2000m, 0 at 800m
            goalZ = targetZ + (cruiseZ - targetZ) * t;
        }

        // Terrain avoidance
        if (terrain != null) {
            double floor = terrain.elevationAt(pos.x(), pos.y());
            double safeZ = floor + MIN_TERRAIN_CLEARANCE;
            double lookAhead = input.speed() * 3;
            double aheadFloor = terrain.elevationAt(
                    pos.x() + Math.sin(heading) * lookAhead,
                    pos.y() + Math.cos(heading) * lookAhead);
            safeZ = Math.max(safeZ, aheadFloor + MIN_TERRAIN_CLEARANCE + 10);
            if (goalZ < safeZ) goalZ = safeZ;
        }
        goalZ = Math.min(goalZ, minDepth);
        // Depth floor ramps based on distance:
        // Far (>800m): floor at -60m (terrain clearance, lob over ridges)
        // Close (<200m): floor at targetZ (must reach the target, even if shallow)
        // Between: smooth blend
        if (dist > 800) {
            goalZ = Math.min(goalZ, -60);
        } else if (dist > 200) {
            double t = (dist - 200) / 600.0; // 1 at 800m, 0 at 200m
            double floor = targetZ + (-60 - targetZ) * t;
            goalZ = Math.min(goalZ, floor);
        }
        // Below 200m: no floor, goalZ = targetZ from the close-range path

        // PID controller with distance-dependent gains.
        // Far away: soft P, strong D (stay smooth, go shallow for clearance).
        // Close in: aggressive P, less D (commit to target depth for the kill).
        double dt = input.deltaTimeSeconds();
        double depthError = goalZ - pos.z();
        depthIntegral += depthError * dt;
        depthIntegral = Math.clamp(depthIntegral, -20, 20); // tight anti-windup
        double depthDerivative = (depthError - prevDepthError) / Math.max(dt, 0.001);
        prevDepthError = depthError;

        // Urgency ramps from 0 (far) to 1 (close)
        double urgency = Math.clamp(1.0 - dist / 1500.0, 0, 1);
        double kP = 0.0015 + urgency * 0.008;   // 0.0015 far -> 0.0095 close
        double kI = 0.0002 + urgency * 0.0008; // very gentle integral
        // D term: extra damping when ascending (prevents overshoot past target depth)
        double basekD = 0.05 - urgency * 0.02;
        double kD = depthDerivative > 0 ? basekD * 2.0 : basekD; // double D when rising
        double maxPlanes = 0.10 + urgency * 0.15;

        double planes = kP * depthError + kI * depthIntegral + kD * depthDerivative;

        // Surface avoidance: predictive + hard limit.
        // At speed s and pitch p, vertical rate is s*sin(p).
        double vRate = input.speed() * Math.sin(input.self().pitch());
        double predictedZ = pos.z() + vRate * 4; // where we'll be in 4 seconds

        // If predicted to be above -40m, start braking now
        if (predictedZ > -40 && vRate > 0) {
            double brakeUrgency = Math.max(0, (predictedZ + 40) * 0.03 + vRate * 0.05);
            planes = Math.min(planes, -brakeUrgency);
            depthIntegral = Math.min(depthIntegral, 0);
        }
        // Hard floor at -45m
        if (pos.z() > -45) {
            double surfaceUrgency = (pos.z() + 45) * 0.08;
            planes = Math.min(planes, -surfaceUrgency - 0.10);
            depthIntegral = Math.min(depthIntegral, 0);
        }

        output.setSternPlanes(Math.clamp(planes, -maxPlanes, maxPlanes));

        // Publish intercept point for viewer
        output.publishTarget(interceptX, interceptY, targetZ);
    }

    /** Use passive sonar during transit to refine bearing to target. */
    private void usePassiveContacts(TorpedoInput input, Vec3 pos, double heading) {
        if (input.sonarContacts().isEmpty()) return;

        // Pick the loudest forward contact (likely our target)
        SonarContact best = pickForwardContact(input.sonarContacts(), heading);
        if (best != null && best.signalExcess() > 0) {
            // Passive gives bearing only (no range). Use it two ways:
            // 1. Gently nudge target position estimate (bearing is good, range is our guess)
            // 2. Feed the PN bearing tracker for continuous LOS rate computation
            double pdx = targetX - pos.x();
            double pdy = targetY - pos.y();
            double currentDist = Math.sqrt(pdx * pdx + pdy * pdy);

            double passiveX = pos.x() + Math.sin(best.bearing()) * currentDist;
            double passiveY = pos.y() + Math.cos(best.bearing()) * currentDist;

            // Gentle position blend (noisy bearings + estimated range = don't trust too much)
            targetX = targetX * 0.95 + passiveX * 0.05;
            targetY = targetY * 0.95 + passiveY * 0.05;

            // Update PN bearing tracker from passive (much faster than waiting for active pings)
            if (phase == Phase.TERMINAL || phase == Phase.ACQUISITION) {
                double passiveBearing = best.bearing();
                if (passiveBearing < 0) passiveBearing += 2 * Math.PI;
                prevBearingToTarget = passiveBearing;
                prevBearingTick = input.tick();
            }
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

    /** Pick the best non-torpedo contact forward of our heading. */
    private SonarContact pickForwardContact(java.util.List<SonarContact> returns, double heading) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (var c : returns) {
            if (c.range() <= 0 && c.isActive()) continue; // skip zero-range active
            if (c.classification() == se.hirt.searobots.api.SonarContact.Classification.TORPEDO) continue;
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
