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

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.SubmarineDrone;

import java.util.List;

public class HeadlessRun {
    public static void main(String[] args) {
        long seed = args.length > 0 ? Long.parseLong(args[0]) : -7475901583965440089L;
        var world = new WorldGenerator().generate(MatchConfig.withDefaults(seed));
        var sim = new SimulationLoop();

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        sim.run(world, controllers, configs, new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
                for (var s : subs) {
                    double yawDeg = Math.toDegrees(s.pose().heading());
                    double pitchDeg = Math.toDegrees(s.pose().pitch());
                    // Check for crazy values
                    if (Math.abs(s.speed()) > 30 || Math.abs(s.pose().position().x()) > 20000
                            || Math.abs(s.pose().position().y()) > 20000
                            || Math.abs(pitchDeg) > 60) {
                        System.out.printf("ALERT tick=%d id=%d name=%s x=%.0f y=%.0f z=%.1f hdg=%.1f pitch=%.1f spd=%.1f rudder=%.2f planes=%.2f throttle=%.2f status=%s%n",
                                tick, s.id(), s.name(), s.pose().position().x(), s.pose().position().y(),
                                s.pose().position().z(), yawDeg, pitchDeg, s.speed(),
                                s.rudder(), s.sternPlanes(), s.throttle(), s.status());
                        sim.stop();
                        return;
                    }
                    if (tick % 500 == 0 && s.id() == 0) {
                        System.out.printf("tick=%d id=%d x=%.0f y=%.0f z=%.1f hdg=%.1f pitch=%.1f spd=%.1f rudder=%.2f planes=%.2f status=%s%n",
                                tick, s.id(), s.pose().position().x(), s.pose().position().y(),
                                s.pose().position().z(), yawDeg, pitchDeg, s.speed(),
                                s.rudder(), s.sternPlanes(), s.status());
                    }
                }
            }

            @Override
            public void onMatchEnd() {
                System.out.println("Match ended.");
            }
        });
    }
}
