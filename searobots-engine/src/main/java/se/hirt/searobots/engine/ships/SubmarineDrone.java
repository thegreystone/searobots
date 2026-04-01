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
 * A simple submarine drone that cruises at a fixed depth (~40m below
 * surface), making moderate noise. Never attacks, never tracks contacts.
 * Just drives around avoiding islands, terrain, and the arena boundary.
 * Quieter than the surface ship drone, harder to detect and track.
 * Useful for testing submarine-vs-submarine engagement scenarios.
 */
public final class SubmarineDrone implements SubmarineController {

    @Override
    public String name() { return "Sub Drone"; }

    private static final double CRUISE_THROTTLE = 0.2;  // quiet patrol, ~6 m/s
    private static final double TARGET_DEPTH = -40;      // fixed operating depth
    private static final double BOUNDARY_MARGIN = 1500;
    private static final double TERRAIN_SCAN_DIST = 1200;
    private static final double FLOOR_CLEARANCE = 80;
    private static final double MIN_DEPTH = -20;

    private BattleArea battleArea;
    private TerrainMap terrain;

    @Override
    public void onMatchStart(MatchContext context) {
        this.battleArea = context.config().battleArea();
        this.terrain = context.terrain();
    }

    @Override
    public void onTick(SubmarineInput input, SubmarineOutput output) {
        var pos = input.self().pose().position();
        double heading = input.self().pose().heading();
        double depth = pos.z();
        double rudder = 0;

        output.setThrottle(CRUISE_THROTTLE);

        // Depth control: maintain target depth using ballast and stern planes
        double floor = terrain.elevationAt(pos.x(), pos.y());
        double safeDepth = Math.max(floor + FLOOR_CLEARANCE, TARGET_DEPTH);
        safeDepth = Math.min(safeDepth, MIN_DEPTH);

        double depthError = depth - safeDepth;
        // Simple PD depth controller
        double verticalSpeed = input.self().velocity().linear().z();
        double ballast = Math.clamp(0.5 - 0.01 * depthError - 0.1 * verticalSpeed, 0.0, 1.0);
        double sternPlanes = Math.clamp(-0.02 * depthError, -0.5, 0.5);
        output.setBallast(ballast);
        output.setSternPlanes(sternPlanes);

        // Avoid arena boundary: steer toward center
        // Lift model: rudder authority scales with v^2, avoid stall at full deflection
        double distToBoundary = battleArea.distanceToBoundary(pos.x(), pos.y());
        if (distToBoundary < BOUNDARY_MARGIN) {
            double toCenter = Math.atan2(-pos.x(), -pos.y());
            if (toCenter < 0) toCenter += 2 * Math.PI;
            double diff = toCenter - heading;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;
            double urgency = 1.0 - distToBoundary / BOUNDARY_MARGIN;
            rudder = Math.clamp(diff * 2, -1, 1) * Math.max(urgency, 0.5);
            // More speed for turning authority when near boundary
            output.setThrottle(Math.max(CRUISE_THROTTLE, 0.3 + urgency * 0.2));
        }

        // Avoid terrain: scan ahead and to the sides at multiple distances.
        // Check the hull width too (the sub is 65m long, 8m wide).
        boolean terrainThreat = false;
        double worstFloor = -9999;
        for (double dist : new double[]{200, 500, 800, 1200}) {
            for (double angleOff : new double[]{0, -0.3, 0.3}) {
                double scanH = heading + angleOff;
                double sx = pos.x() + Math.sin(scanH) * dist;
                double sy = pos.y() + Math.cos(scanH) * dist;
                double scanFloor = terrain.elevationAt(sx, sy);
                if (scanFloor + FLOOR_CLEARANCE > depth) {
                    terrainThreat = true;
                }
                if (scanFloor > worstFloor) worstFloor = scanFloor;
            }
        }
        // Also check immediately around the hull (close-range safety)
        for (double angleOff : new double[]{-0.8, -0.4, 0, 0.4, 0.8}) {
            double sx = pos.x() + Math.sin(heading + angleOff) * 100;
            double sy = pos.y() + Math.cos(heading + angleOff) * 100;
            double hullFloor = terrain.elevationAt(sx, sy);
            if (hullFloor + FLOOR_CLEARANCE > depth) {
                terrainThreat = true;
            }
        }

        if (terrainThreat) {
            // Turn toward deeper side
            double leftH = heading - 0.6;
            double rightH = heading + 0.6;
            double floorLeft = terrain.elevationAt(
                    pos.x() + Math.sin(leftH) * TERRAIN_SCAN_DIST,
                    pos.y() + Math.cos(leftH) * TERRAIN_SCAN_DIST);
            double floorRight = terrain.elevationAt(
                    pos.x() + Math.sin(rightH) * TERRAIN_SCAN_DIST,
                    pos.y() + Math.cos(rightH) * TERRAIN_SCAN_DIST);
            rudder = floorLeft < floorRight ? -0.8 : 0.8;
            // Slow down when terrain is close
            output.setThrottle(Math.min(CRUISE_THROTTLE, 0.15));
        }

        output.setRudder(rudder);
        output.setStatus("DRONE");
    }

    @Override
    public void onMatchEnd(MatchResult result) {}
}
