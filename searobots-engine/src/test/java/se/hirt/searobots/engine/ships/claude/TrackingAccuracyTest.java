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
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.TargetDrone;

import java.util.List;

class TrackingAccuracyTest {

    @Test
    void measureEstimationAccuracy() {
        long seed = 42;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new ClaudeAttackSub(), new TargetDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip());

        System.out.printf("%-8s %-8s %-8s %-8s %-8s %-8s %-10s%n",
                "Time", "ActDist", "EstDist", "PosErr", "Hdg", "Spd", "Status");
        System.out.println("-".repeat(70));

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (tick % 250 != 0 || submarines.size() < 2) return;
                var s0 = submarines.get(0);
                var s1 = submarines.get(1);
                var p0 = s0.pose().position();
                var p1 = s1.pose().position();
                double actualDist = Math.sqrt(Math.pow(p0.x()-p1.x(),2) + Math.pow(p0.y()-p1.y(),2));

                var contacts = s0.contactEstimates();
                if (contacts == null || contacts.isEmpty()) return;
                var ce = contacts.getFirst();

                double posErr = Math.sqrt(Math.pow(ce.x()-p1.x(),2) + Math.pow(ce.y()-p1.y(),2));
                double estDist = Math.sqrt(Math.pow(ce.x()-p0.x(),2) + Math.pow(ce.y()-p0.y(),2));

                String hdg = Double.isNaN(ce.estimatedHeading()) ? "n/a" :
                        String.format("%.0f", Math.toDegrees(ce.estimatedHeading()));
                String spd = ce.estimatedSpeed() < 0 ? "n/a" :
                        String.format("%.1f", ce.estimatedSpeed());
                String status = s0.status() != null ?
                        s0.status().substring(0, Math.min(10, s0.status().length())) : "";

                if (actualDist < 3000) {
                    System.out.printf("%-8.0f %-8.0f %-8.0f %-8.0f %-8s %-8s %-10s%n",
                            tick/50.0, actualDist, estDist, posErr, hdg, spd, status);
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
