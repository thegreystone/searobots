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
package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

public final class CodexClaudeBattleTrace {
    private static final int MATCH_TICKS = 600 * 50;

    private CodexClaudeBattleTrace() {}

    public static void main(String[] args) {
        long seed = args.length > 0 ? Long.parseLong(args[0]) : -499828820137885826L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());
        Set<Integer> seenTorpedoes = new HashSet<>();
        int[] lastCodexHp = {config.startingHp()};
        int[] lastClaudeHp = {config.startingHp()};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() < 2) {
                    return;
                }

                var codex = subs.get(0);
                var claude = subs.get(1);
                for (TorpedoSnapshot torp : torps) {
                    if (seenTorpedoes.add(torp.id())) {
                        System.out.printf(Locale.US,
                                "LAUNCH tick=%d owner=%d pos=(%.0f,%.0f,%.0f) target=(%.0f,%.0f,%.0f)%n",
                                tick, torp.ownerId(),
                                torp.pose().position().x(), torp.pose().position().y(), torp.pose().position().z(),
                                torp.targetX(), torp.targetY(), torp.targetZ());
                    }
                    if (torp.detonated()) {
                        System.out.printf(Locale.US,
                                "DETONATE tick=%d owner=%d pos=(%.0f,%.0f,%.0f)%n",
                                tick, torp.ownerId(),
                                torp.pose().position().x(), torp.pose().position().y(), torp.pose().position().z());
                    }
                }

                if (codex.hp() != lastCodexHp[0] || claude.hp() != lastClaudeHp[0]) {
                    System.out.printf(Locale.US,
                            "DAMAGE tick=%d codexHp=%d claudeHp=%d%n",
                            tick, codex.hp(), claude.hp());
                    lastCodexHp[0] = codex.hp();
                    lastClaudeHp[0] = claude.hp();
                }

                if (tick % 500 == 0) {
                    double range = codex.pose().position().distanceTo(claude.pose().position());
                    String codexTrack = codex.contactEstimates().isEmpty()
                            ? "-"
                            : String.format(Locale.US, "%s c=%.2f u=%.0f",
                            codex.contactEstimates().getFirst().label(),
                            codex.contactEstimates().getFirst().confidence(),
                            codex.contactEstimates().getFirst().uncertaintyRadius());
                    String claudeTrack = claude.contactEstimates().isEmpty()
                            ? "-"
                            : String.format(Locale.US, "%s c=%.2f u=%.0f",
                            claude.contactEstimates().getFirst().label(),
                            claude.contactEstimates().getFirst().confidence(),
                            claude.contactEstimates().getFirst().uncertaintyRadius());
                    System.out.printf(Locale.US,
                            "t=%5d rng=%5.0f codex[hp=%d spd=%.1f thr=%.2f ping=%s torps=%d fs=%s %s %s] "
                                    + "claude[hp=%d spd=%.1f thr=%.2f ping=%s fs=%s %s %s]%n",
                            tick, range,
                            codex.hp(), codex.speed(), codex.throttle(), codex.pingRequested(),
                            codex.torpedoesRemaining(), codex.firingSolution() != null, codexTrack, codex.status(),
                            claude.hp(), claude.speed(), claude.throttle(), claude.pingRequested(),
                            claude.firingSolution() != null, claudeTrack, claude.status());
                }

                if (tick >= MATCH_TICKS || codex.hp() <= 0 || claude.hp() <= 0) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {
                System.out.println("TRACE END");
            }
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try {
            thread.join(60_000);
        } catch (InterruptedException ignored) {
        }
        sim.stop();
        try {
            thread.join(5_000);
        } catch (InterruptedException ignored) {
        }
    }
}
