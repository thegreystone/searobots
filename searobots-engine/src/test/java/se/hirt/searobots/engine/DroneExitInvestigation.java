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

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.SubmarineDrone;

import java.util.List;

class DroneExitInvestigation {

    @Test
    void investigateDroneExit() {
        long seed = 4291328891004373985L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        var listener = new SimulationListener() {
            boolean droneForfeited = false;

            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (submarines.size() < 2) return;
                var drone = submarines.get(1); // sub drone is index 1

                // Log every 500 ticks when near boundary
                var pos = drone.pose().position();
                double distToBoundary = config.battleArea().distanceToBoundary(pos.x(), pos.y());

                if (tick % 2500 == 0 || (distToBoundary < 500 && tick % 50 == 0)) {
                    System.out.printf("  t=%-6d drone pos=[%7.0f,%7.0f,%6.0f] spd=%5.1f hdg=%5.1f bndry=%5.0f hp=%d %s %s%n",
                            tick, pos.x(), pos.y(), pos.z(), drone.speed(),
                            Math.toDegrees(drone.pose().heading()),
                            distToBoundary, drone.hp(),
                            drone.forfeited() ? "FORFEIT" : "",
                            drone.status() != null ? drone.status() : "");
                }

                if (drone.forfeited() && !droneForfeited) {
                    droneForfeited = true;
                    System.out.printf("%n=== DRONE FORFEITED at t=%d (%.0fs) ===%n", tick, tick / 50.0);
                    System.out.printf("  pos=[%.0f, %.0f, %.0f] speed=%.1f heading=%.1f%n",
                            pos.x(), pos.y(), pos.z(), drone.speed(),
                            Math.toDegrees(drone.pose().heading()));
                    System.out.printf("  distToBoundary=%.0f%n", distToBoundary);
                    System.out.printf("  battleArea: %s%n", config.battleArea());
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}
    }
}
