/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * This software is free:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
    private double lastActiveRange = Double.MAX_VALUE; // closest active return range

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
        // Terminal slow-down: reduce speed inside 100m when turning hard (crossing shots).
        // Slower speed = tighter turn radius. Only apply for large heading errors
        // to avoid slowing down in tail chases where speed is needed.
        if (phase == Phase.TERMINAL && dist < 100 && absErr > Math.toRadians(25)) {
            goalSpeed = Math.min(goalSpeed, 14.0);
        }
        double speedError = goalSpeed - input.speed();
        double throttle = Math.clamp(speedError * 0.15 + 0.5, 0.1, 1.0);
        output.setThrottle(throttle);

        // ── Sonar: passive every tick, active when cooldown allows ──
        // Passive bearings update the target bearing continuously (every tick).
        // Active pings give range + precise position (every 5 seconds).
        // Between pings, passive tracking keeps the PN guidance fed with
        // fresh bearing data instead of flying on stale predictions.
        usePassiveContacts(input, pos, heading);
        double activeRange = processActiveReturns(input, pos, heading);

        // Manual detonation: if active sonar shows target within 30m, detonate.
        // The proximity fuse may miss in crossing shots; manual detonation ensures
        // the warhead goes off when we're close enough.
        if (activeRange < 30 && phase == Phase.TERMINAL) {
            output.detonate();
            return;
        }

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
                    // Closing speed: torpedo speed minus target's speed along our bearing
                    double closingSpeed = Math.max(
                            torpSpeed - (cappedVelX * bearingX + cappedVelY * bearingY), 5);
                    double pnAccel = 4.0 * closingSpeed * losRate;
                    // Convert lateral acceleration to rudder command
                    // At speed v, rudder 1.0 gives ~v/turnRadius lateral accel
                    rudderCmd = Math.clamp(pnAccel * 0.08, -1, 1);

                    // Blend with pursuit guidance at very close range.
                    // Use direct bearing to target (not intercept point) for pure pursuit.
                    if (dist < 150) {
                        double headErr = bearingToTarget - heading;
                        while (headErr > Math.PI) headErr -= 2 * Math.PI;
                        while (headErr < -Math.PI) headErr += 2 * Math.PI;
                        double pursuit = Math.clamp(headErr * 4.0, -1, 1);
                        double blend = Math.clamp((dist - 50) / 100.0, 0, 1);
                        rudderCmd = rudderCmd * blend + pursuit * (1 - blend);
                    }
                } else {
                    // No valid dt, fall back to pursuit on target
                    double headErr = bearingToTarget - heading;
                    while (headErr > Math.PI) headErr -= 2 * Math.PI;
                    while (headErr < -Math.PI) headErr += 2 * Math.PI;
                    rudderCmd = Math.clamp(headErr * 4.0, -1, 1);
                }
            } else {
                // First tick in terminal, use pursuit on target
                double headErr = bearingToTarget - heading;
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

        // Depth profile: blend from cruise altitude to target depth over a long
        // range so the torpedo arrives at the right depth. Starting the descent
        // early (from 1500m) gives the PID enough distance to converge, even
        // with gentle pitch rates.
        double goalZ;
        // Cruise altitude: slightly above target depth for terrain clearance.
        // Never climb above launch depth: if we're already deep enough, stay deep.
        double cruiseZ = Math.min(targetZ + 30, -60);
        if (!Double.isNaN(launchDepth)) {
            cruiseZ = Math.min(cruiseZ, launchDepth);
        }
        if (dist > 1500) {
            goalZ = cruiseZ;
        } else {
            // Smooth blend from cruise to target depth over 1500m to 0m
            double t = dist / 1500.0; // 1 at 1500m, 0 at target
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
        // Depth floor: prevent going too shallow during far approach.
        // Relaxes as we get closer so the torpedo can match shallow targets.
        if (dist > 500) {
            goalZ = Math.min(goalZ, -60);
        } else if (dist > 100) {
            double t = (dist - 100) / 400.0;
            goalZ = Math.min(goalZ, targetZ + (-60 - targetZ) * t);
        }

        // PID controller with distance-dependent gains.
        // Far away: soft P, strong D (stay smooth, go shallow for clearance).
        // Close in: aggressive P, extra planes for last-second depth corrections.
        double dt = input.deltaTimeSeconds();
        double depthError = goalZ - pos.z();
        depthIntegral += depthError * dt;
        depthIntegral = Math.clamp(depthIntegral, -20, 20); // tight anti-windup
        double depthDerivative = (depthError - prevDepthError) / Math.max(dt, 0.001);
        prevDepthError = depthError;

        // Urgency ramps from 0 (far) to 1 (close)
        double urgency = Math.clamp(1.0 - dist / 1500.0, 0, 1);
        // Vertical rate for overshoot detection and surface avoidance
        double vRate = input.speed() * Math.sin(input.self().pitch());

        // Terminal boost: scale with both range (close = more) and depth error
        // (large error = more). This avoids boosting when already at the right
        // depth (prevents oscillation) while aggressively correcting large gaps.
        double termRange = Math.clamp(1.0 - dist / 400.0, 0, 1);
        double depthNeed = Math.clamp(Math.abs(depthError) / 30.0, 0, 1);
        double termBoost = termRange * depthNeed;
        double kP = 0.0015 + urgency * 0.008 + termBoost * 0.012;
        double kI = 0.0002 + urgency * 0.0008;
        // D term: extra damping when vertical rate opposes the correction
        double basekD = 0.05 - urgency * 0.02 + termBoost * 0.02;
        boolean overshooting = (vRate > 0 && depthError < -2) || (vRate < 0 && depthError > 2);
        double kD = overshooting ? basekD * 3.0 : (depthDerivative > 0 ? basekD * 2.0 : basekD);
        // More planes authority when depth error is large at close range
        double maxPlanes = 0.10 + urgency * 0.15 + termBoost * 0.25;

        double planes = kP * depthError + kI * depthIntegral + kD * depthDerivative;
        double predictedZ = pos.z() + vRate * 4; // where we'll be in 4 seconds

        // Surface avoidance: predictive + hard limit.
        if (predictedZ > -40 && vRate > 0) {
            double brakeUrgency = Math.max(0, (predictedZ + 40) * 0.03 + vRate * 0.05);
            planes = Math.min(planes, -brakeUrgency);
            depthIntegral = Math.min(depthIntegral, 0);
        }
        if (pos.z() > -45) {
            double surfaceUrgency = (pos.z() + 45) * 0.08;
            planes = Math.min(planes, -surfaceUrgency - 0.10);
            depthIntegral = Math.min(depthIntegral, 0);
        }

        output.setSternPlanes(Math.clamp(planes, -maxPlanes, maxPlanes));

        // Publish intercept point for viewer
        output.publishTarget(interceptX, interceptY, targetZ);

        // Publish full diagnostics for analysis
        double estHdg = Math.atan2(estVelX, estVelY);
        if (estHdg < 0) estHdg += 2 * Math.PI;
        double estSpd = Math.sqrt(estVelX * estVelX + estVelY * estVelY);
        output.publishDiagnostics(targetX, targetY, targetZ, estHdg, estSpd,
                interceptX, interceptY, targetZ, phase.name());
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

            // DO NOT overwrite the PN bearing tracker (prevBearingToTarget) with
            // passive bearings. The PN guidance computes LOS rate from consecutive
            // bearing samples; injecting noisy passive data every tick destroys the
            // rate signal and causes erratic steering. Let the PN tracker update
            // naturally from the recomputed bearingToTarget in the main loop.
        }
    }

    /** Process active sonar returns for precise targeting. Returns closest active range. */
    private double processActiveReturns(TorpedoInput input, Vec3 pos, double heading) {
        if (input.activeSonarReturns().isEmpty()) return Double.MAX_VALUE;

        SonarContact best = pickForwardContact(input.activeSonarReturns(), heading);
        if (best == null) return Double.MAX_VALUE;

        double closestRange = best.range();

        double fixX = pos.x() + Math.sin(best.bearing()) * best.range();
        double fixY = pos.y() + Math.cos(best.bearing()) * best.range();

        // Estimate target velocity from consecutive fixes
        if (!Double.isNaN(prevFixX) && prevFixTick >= 0) {
            double dtFix = (input.tick() - prevFixTick) / 50.0;
            if (dtFix > 0.2 && dtFix < 20) {
                double rawVX = (fixX - prevFixX) / dtFix;
                double rawVY = (fixY - prevFixY) / dtFix;
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
        lastActiveRange = closestRange;
        return closestRange;
    }

    /** Pick the best non-torpedo contact forward of our heading. */
    private SonarContact pickForwardContact(java.util.List<SonarContact> returns,
                                             double heading) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (var c : returns) {
            if (c.range() <= 0 && c.isActive()) continue; // skip zero-range active
            if (c.classification() == SonarContact.Classification.TORPEDO) continue;
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
