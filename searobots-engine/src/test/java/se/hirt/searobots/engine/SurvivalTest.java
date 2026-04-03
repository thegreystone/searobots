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
import se.hirt.searobots.engine.ships.SubmarineDrone;

import java.util.List;

/**
 * Tests that any submarine controller survives terrain across multiple seeds.
 * Extend and provide your controller via {@link #createController()}.
 */
public abstract class SurvivalTest extends AbstractControllerTest {

    @Test
    void subSurvivesMultipleSeeds() {
        long[] seeds = {42, 123, 999, 2024, 7777, 314, 55555, 8888, 1001, 6543};
        int survived = 0;
        int total = seeds.length;

        System.out.printf("=== Survival Test: %s ===%n", controllerName());
        for (long seed : seeds) {
            var config = MatchConfig.withDefaults(seed);
            var world = new WorldGenerator().generate(config);
            var sim = new SimulationLoop();
            sim.setSpeedMultiplier(1_000_000);

            List<SubmarineController> controllers = List.of(createController(), new SubmarineDrone());
            List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

            boolean[] subAlive = {true};
            long[] deathTick = {-1};
            double[] deathZ = {0};
            String[] deathStatus = {""};

            var listener = new SimulationListener() {
                @Override
                public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                    if (submarines.size() < 2) return;
                    var s0 = submarines.get(0);
                    if (s0.hp() <= 0 && subAlive[0]) {
                        subAlive[0] = false;
                        deathTick[0] = tick;
                        deathZ[0] = s0.pose().position().z();
                        deathStatus[0] = s0.status() != null ? s0.status() : "";
                    }
                }
                @Override public void onMatchEnd() {}
            };

            var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
            thread.start();
            try { thread.join(60_000); } catch (InterruptedException e) {}
            sim.stop();
            try { thread.join(5000); } catch (InterruptedException e) {}

            if (subAlive[0]) {
                survived++;
                System.out.printf("  Seed %6d: SURVIVED%n", seed);
            } else {
                System.out.printf("  Seed %6d: DIED at tick %d (%.0fs) z=%.0f %s%n",
                        seed, deathTick[0], deathTick[0] / 50.0,
                        deathZ[0], deathStatus[0]);
            }
        }

        System.out.printf("Survival: %d/%d seeds%n", survived, total);
    }
}
