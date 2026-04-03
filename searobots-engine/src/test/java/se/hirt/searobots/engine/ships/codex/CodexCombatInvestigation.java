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

import se.hirt.searobots.api.ContactEstimate;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import java.util.List;
import java.util.Locale;

public final class CodexCombatInvestigation {
    private static final long[] DEFAULT_SEEDS = {
            -562490107179808566L,
            -4924880284512257837L,
            6303822463256846936L,
            -499828820137885826L
    };
    private static final int MATCH_TICKS = 600 * 50;

    private CodexCombatInvestigation() {}

    public static void main(String[] args) {
        long[] seeds = args.length == 0 ? DEFAULT_SEEDS : parseSeeds(args);
        for (long seed : seeds) {
            runSeed(seed);
        }
    }

    private static long[] parseSeeds(String[] args) {
        long[] seeds = new long[args.length];
        for (int i = 0; i < args.length; i++) {
            seeds[i] = Long.parseLong(args[i]);
        }
        return seeds;
    }

    private static void runSeed(long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        double[] minBoundaryCodex = {Double.POSITIVE_INFINITY};
        double[] minBoundaryClaude = {Double.POSITIVE_INFINITY};
        int[] lastCodexHp = {config.startingHp()};
        int[] lastClaudeHp = {config.startingHp()};
        long[] endTick = {MATCH_TICKS};
        String[] endReason = {"TIMEOUT"};

        System.out.printf("%n=== Combat Seed %d ===%n", seed);

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (submarines.size() < 2) return;

                var codex = submarines.get(0);
                var claude = submarines.get(1);

                double codexBoundary = config.battleArea().distanceToBoundary(
                        codex.pose().position().x(), codex.pose().position().y());
                double claudeBoundary = config.battleArea().distanceToBoundary(
                        claude.pose().position().x(), claude.pose().position().y());
                minBoundaryCodex[0] = Math.min(minBoundaryCodex[0], codexBoundary);
                minBoundaryClaude[0] = Math.min(minBoundaryClaude[0], claudeBoundary);

                boolean event = tick % 250 == 0
                        || codex.pingRequested()
                        || claude.pingRequested()
                        || codex.firingSolution() != null
                        || claude.firingSolution() != null
                        || codex.hp() != lastCodexHp[0]
                        || claude.hp() != lastClaudeHp[0]
                        || codex.forfeited()
                        || claude.forfeited();

                if (event) {
                    System.out.printf(Locale.US,
                            "t=%5d rng=%5.0f  C[%4d hp %5.0f b %4.1f spd %4.1f %s %s]  "
                                    + "L[%4d hp %5.0f b %4.1f spd %4.1f %s %s]%n",
                            tick,
                            codex.pose().position().distanceTo(claude.pose().position()),
                            codex.hp(), codexBoundary, codex.speed(), codex.throttle(),
                            statusString(codex),
                            estimateString(codex.contactEstimates()),
                            claude.hp(), claudeBoundary, claude.speed(), claude.throttle(),
                            statusString(claude),
                            estimateString(claude.contactEstimates()));
                }

                lastCodexHp[0] = codex.hp();
                lastClaudeHp[0] = claude.hp();

                if (codex.firingSolution() != null) {
                    endTick[0] = tick;
                    endReason[0] = "CODEX FIRING SOLUTION";
                    sim.stop();
                } else if (claude.firingSolution() != null) {
                    endTick[0] = tick;
                    endReason[0] = "CLAUDE FIRING SOLUTION";
                    sim.stop();
                } else if (codex.hp() <= 0 || codex.forfeited()) {
                    endTick[0] = tick;
                    endReason[0] = codex.forfeited() ? "CODEX FORFEIT" : "CODEX DIED";
                    sim.stop();
                } else if (claude.hp() <= 0 || claude.forfeited()) {
                    endTick[0] = tick;
                    endReason[0] = claude.forfeited() ? "CLAUDE FORFEIT" : "CLAUDE DIED";
                    sim.stop();
                } else if (tick >= MATCH_TICKS) {
                    endTick[0] = tick;
                    endReason[0] = "TIMEOUT";
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {
                System.out.printf(Locale.US,
                        "END seed=%d tick=%d reason=%s minBoundaryCodex=%.0f minBoundaryClaude=%.0f%n",
                        seed, endTick[0], endReason[0], minBoundaryCodex[0], minBoundaryClaude[0]);
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

    private static String statusString(SubmarineSnapshot snapshot) {
        String status = snapshot.status();
        if (status == null || status.isBlank()) {
            status = "-";
        }
        if (snapshot.pingRequested()) {
            status += " PING";
        }
        return status;
    }

    private static String estimateString(List<ContactEstimate> estimates) {
        if (estimates == null || estimates.isEmpty()) {
            return "no-track";
        }
        ContactEstimate best = estimates.getFirst();
        return String.format(Locale.US, "%s c=%.2f u=%.0f",
                best.label(), best.confidence(), best.uncertaintyRadius());
    }
}
