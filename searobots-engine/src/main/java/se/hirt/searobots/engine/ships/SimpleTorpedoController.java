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
package se.hirt.searobots.engine.ships;

import se.hirt.searobots.api.*;

/**
 * Torpedo controller that uses consecutive active pings to estimate
 * target position and velocity, then steers to an intercept point.
 */
public class SimpleTorpedoController implements TorpedoController {

    private double targetX, targetY, targetZ;
    private boolean hasTarget;
    private TerrainMap terrain;

    // Target tracking from consecutive pings
    private double prevFixX = Double.NaN, prevFixY = Double.NaN;
    private long prevFixTick = -1;
    private double estVelX = 0, estVelY = 0; // estimated target velocity

    private static final double MIN_TERRAIN_CLEARANCE = 30;

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
                    hasTarget = true;
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

        // ── Always ping when ready ──
        if (input.activeSonarCooldownTicks() == 0) {
            output.activeSonarPing();
        }

        // ── Process active sonar returns ──
        if (!input.activeSonarReturns().isEmpty()) {
            // Pick the forward-most contact (ignore launcher behind us)
            SonarContact best = pickForwardContact(input.activeSonarReturns(), heading);
            if (best != null) {
                double fixX = pos.x() + Math.sin(best.bearing()) * best.range();
                double fixY = pos.y() + Math.cos(best.bearing()) * best.range();

                // Estimate target velocity from consecutive fixes
                if (!Double.isNaN(prevFixX) && prevFixTick >= 0) {
                    double dtFix = (input.tick() - prevFixTick) / 50.0; // seconds between fixes
                    if (dtFix > 0.5 && dtFix < 20) { // reasonable fix interval
                        double rawVX = (fixX - prevFixX) / dtFix;
                        double rawVY = (fixY - prevFixY) / dtFix;
                        // Smooth velocity estimate (don't trust a single noisy measurement)
                        estVelX = estVelX * 0.5 + rawVX * 0.5;
                        estVelY = estVelY * 0.5 + rawVY * 0.5;
                    }
                }

                prevFixX = fixX;
                prevFixY = fixY;
                prevFixTick = input.tick();

                // Update target position and depth from active return
                targetX = fixX;
                targetY = fixY;
                if (!Double.isNaN(best.estimatedDepth())) {
                    targetZ = best.estimatedDepth();
                }
            }
        }

        // ── Compute intercept point ──
        double dx = targetX - pos.x();
        double dy = targetY - pos.y();
        double dist = Math.sqrt(dx * dx + dy * dy);

        // Time to intercept at current closing rate
        double torpSpeed = Math.max(input.speed(), 5);
        double timeToIntercept = dist / torpSpeed;

        // Predict where target will be when we arrive
        double interceptX = targetX + estVelX * timeToIntercept;
        double interceptY = targetY + estVelY * timeToIntercept;

        // ── Steer toward intercept point ──
        double idx = interceptX - pos.x();
        double idy = interceptY - pos.y();
        double bearingToIntercept = Math.atan2(idx, idy);
        if (bearingToIntercept < 0) bearingToIntercept += 2 * Math.PI;

        double headingError = bearingToIntercept - heading;
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        output.setRudder(Math.clamp(headingError * 1.5, -1, 1));

        // ── Depth control ──
        // Cruise at launch depth. Only adjust toward target depth when close
        // and we have active sonar data to confirm where the target is.
        double minDepth = -30;
        double desiredZ;
        if (dist < 200) {
            desiredZ = pos.z(); // hold depth in terminal phase
        } else if (dist < 500 && prevFixTick >= 0) {
            // Close with active fix: adjust toward target depth
            double depthError = targetZ - pos.z();
            desiredZ = pos.z() + depthError * 0.03;
        } else {
            // Cruise: approach target depth gradually (don't stay at launch depth
            // if target is at a different depth)
            double cruiseTarget = Math.max(targetZ, -150); // don't go deeper than -150m
            double depthError = cruiseTarget - pos.z();
            desiredZ = pos.z() + depthError * 0.01; // very gentle blend
        }

        // Terrain avoidance (only when not terminal)
        if (terrain != null && dist > 100) {
            double floor = terrain.elevationAt(pos.x(), pos.y());
            double safeZ = floor + MIN_TERRAIN_CLEARANCE;
            double lookAhead = input.speed() * 3;
            double aheadFloor = terrain.elevationAt(
                    pos.x() + Math.sin(heading) * lookAhead,
                    pos.y() + Math.cos(heading) * lookAhead);
            safeZ = Math.max(safeZ, aheadFloor + MIN_TERRAIN_CLEARANCE + 10);
            if (desiredZ < safeZ) desiredZ = safeZ;
        }

        desiredZ = Math.min(desiredZ, minDepth); // never breach near surface
        double dz = desiredZ - pos.z();
        double pitchGain = dist < 300 ? 0.06 : 0.03;
        double pitchMax = dist < 300 ? 0.6 : 0.4;
        output.setSternPlanes(Math.clamp(dz * pitchGain, -pitchMax, pitchMax));

        // Publish intercept point for viewer visualization
        output.publishTarget(interceptX, interceptY, targetZ);
    }

    /** Pick the active return that's most forward of our heading. */
    private SonarContact pickForwardContact(java.util.List<SonarContact> returns, double heading) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (var c : returns) {
            if (c.range() <= 0) continue;
            double angleDiff = c.bearing() - heading;
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
            double forwardness = Math.cos(angleDiff);
            double score = forwardness * 100 - c.range() * 0.01;
            if (score > bestScore) { bestScore = score; best = c; }
        }
        return best;
    }
}
