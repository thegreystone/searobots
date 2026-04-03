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

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.PathPlanner;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.SubmarineDrone;

import java.util.List;

class SingleSeedTrace {

    @Test void traceSeed55555() { trace(55555, 50000, 70000); }
    @Test void traceSeed8888()  { trace(8888,  35000, 41000); }
    @Test void traceSeed7777()  { trace(7777,  5000, 12000); }

    private void trace(long seed, long startTick, long endTick) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);
        var planner = new PathPlanner(world.terrain(), -80, 200, 75);

        List<SubmarineController> controllers = List.of(new ClaudeAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        System.out.printf("%n=== SEED %d TRACE [%d-%d] ===%n", seed, startTick, endTick);

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (submarines.isEmpty()) return;
                var s0 = submarines.get(0);
                if (tick < startTick || (tick > endTick && s0.hp() > 0)) return;

                var pos = s0.pose().position();
                double floor = world.terrain().elevationAt(pos.x(), pos.y());
                boolean safe = planner.isSafe(pos.x(), pos.y());

                if (tick % 500 == 0 || !safe || s0.hp() <= 0) {
                    var wps = s0.waypoints();
                    String wpStr = wps.isEmpty() ? "NO_WP" : "";
                    for (var wp : wps) {
                        if (wp.active()) {
                            wpStr = String.format("-> [%5.0f,%5.0f]", wp.x(), wp.y());
                            break;
                        }
                    }
                    if (wpStr.isEmpty() && !wps.isEmpty()) {
                        var wp = wps.getFirst();
                        wpStr = String.format("1st[%5.0f,%5.0f]", wp.x(), wp.y());
                    }
                    System.out.printf("  t=%-6d [%5.0f,%5.0f,%4.0f] fl=%4.0f %s spd=%5.1f hdg=%5.1f hp=%4d %s %s%n",
                            tick, pos.x(), pos.y(), pos.z(), floor,
                            safe ? "OK" : "!!",
                            s0.speed(), Math.toDegrees(s0.pose().heading()),
                            s0.hp(), s0.status(), wpStr);
                    if (s0.hp() <= 0) return;
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(30_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}
    }
}
