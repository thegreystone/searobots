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
package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;

/**
 * A simple target drone that cruises near the surface at high speed,
 * making lots of noise. Never attacks, never tracks contacts. Just
 * drives around avoiding islands and the arena boundary.
 * Useful for testing sonar, tracking, and torpedo mechanics.
 */
public final class TargetDrone implements SubmarineController {

    @Override
    public String name() { return "Ship Drone"; }

    // Full throttle: the surface ship VehicleConfig limits terminal speed to ~8 m/s.
    private static final double CRUISE_THROTTLE = 1.0;
    private static final double BOUNDARY_MARGIN = 1200;
    private static final double ISLAND_SCAN_DIST = 1200;

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
        double rudder = 0;

        // surfaceLocked config handles staying at z=0, no need for stern planes or ballast
        output.setThrottle(CRUISE_THROTTLE);

        // Avoid arena boundary: steer toward center
        double distToBoundary = battleArea.distanceToBoundary(pos.x(), pos.y());
        if (distToBoundary < BOUNDARY_MARGIN) {
            double toCenter = Math.atan2(-pos.x(), -pos.y());
            if (toCenter < 0) toCenter += 2 * Math.PI;
            double diff = toCenter - heading;
            while (diff > Math.PI) diff -= 2 * Math.PI;
            while (diff < -Math.PI) diff += 2 * Math.PI;
            double urgency = 1.0 - distToBoundary / BOUNDARY_MARGIN;
            rudder = Math.clamp(diff * 2, -1, 1) * Math.max(urgency, 0.5);
        }

        // Avoid islands: check terrain ahead
        double aheadX = pos.x() + Math.sin(heading) * ISLAND_SCAN_DIST;
        double aheadY = pos.y() + Math.cos(heading) * ISLAND_SCAN_DIST;
        double floorAhead = terrain.elevationAt(aheadX, aheadY);
        if (floorAhead > -40) {
            // Shallow water or island ahead: turn toward deeper side
            double leftH = heading - 0.6;
            double rightH = heading + 0.6;
            double floorLeft = terrain.elevationAt(
                    pos.x() + Math.sin(leftH) * ISLAND_SCAN_DIST,
                    pos.y() + Math.cos(leftH) * ISLAND_SCAN_DIST);
            double floorRight = terrain.elevationAt(
                    pos.x() + Math.sin(rightH) * ISLAND_SCAN_DIST,
                    pos.y() + Math.cos(rightH) * ISLAND_SCAN_DIST);
            rudder = floorLeft < floorRight ? -0.8 : 0.8;
        }

        output.setRudder(rudder);
        output.setStatus("DRONE");
    }

    @Override
    public void onMatchEnd(MatchResult result) {}
}
