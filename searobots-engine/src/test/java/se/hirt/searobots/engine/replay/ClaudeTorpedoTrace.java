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
package se.hirt.searobots.engine.replay;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Deep study of why Claude's torpedoes fail terminal acquisition. For each
 * torpedo Claude (index 1) launches, it watches — every tick, purely from the
 * published snapshot diagnostics, no controller change — three quantities:
 * <ul>
 *   <li>distance from the torpedo to the TRUE enemy hull,</li>
 *   <li>distance from the torpedo's own target ESTIMATE (diagEst) to the TRUE
 *       enemy — i.e. how wrong the torpedo's solution is, and whether an active
 *       ping ever corrects it,</li>
 *   <li>the guidance phase (TRANSIT/ACQUISITION/TERMINAL).</li>
 * </ul>
 * The decisive number is the minimum estimate-to-truth error over the run: if
 * it stays large, the torpedo NEVER acquired (active sonar never illuminated
 * the moved target); if it goes small but the torpedo still misses, the failure
 * is terminal geometry, not acquisition.
 * <p>
 * Run on demand: {@code mvn test -Dsearobots.diag=true -Dtest=ClaudeTorpedoTrace}.
 */
@EnabledIfSystemProperty(named = "searobots.diag", matches = "true")
class ClaudeTorpedoTrace {

    private static final long[] SEEDS = {1, 3, 5, 8, 10, 13, 16};
    private static final int MAX_TICKS = 30_000;
    private static final int CLAUDE = 1;

    private static final class Torp {
        long launchTick = -1;
        double launchDistTrue;
        double minDistTrue = Double.MAX_VALUE; // torpedo -> true enemy
        long minDistTrueTick;
        double minEstError = Double.MAX_VALUE;  // estimate -> true enemy (did it ever acquire?)
        long minEstErrorTick;
        double estErrorAtLaunch = Double.MAX_VALUE;
        boolean reachedAcq, reachedTerm;
        int pingTicks; // ticks the torpedo requested a ping
        // last sample
        long lastTick;
        String lastPhase = "";
        double lastDistTrue, lastEstError, lastFuel, lastSpeed;
        boolean detonated;
    }

    private final Map<String, Integer> buckets = new LinkedHashMap<>();
    private final Map<String, double[]> bucketStats = new LinkedHashMap<>(); // {sumClosest, sumFuelLeft, sumLaunchTick}

    private void bucket(String b, Torp t) {
        buckets.merge(b, 1, Integer::sum);
        double[] s = bucketStats.computeIfAbsent(b, k -> new double[3]);
        s[0] += t.minDistTrue;
        s[1] += t.lastFuel;
        s[2] += t.launchTick;
    }

    @Test
    void traceClaudeTorpedoes() {
        int total = 0;
        for (long seed : SEEDS) {
            Map<Integer, Torp> torps = runSeed(seed);
            System.out.printf(Locale.US, "%n===== seed %d : %d Claude torpedoes =====%n", seed, torps.size());
            for (var e : torps.entrySet()) {
                Torp t = e.getValue();
                total++;
                bucket(classify(t), t);
                System.out.printf(Locale.US,
                        "  torp %d: launch t=%d distTrue=%.0fm estErr@launch=%.0fm%n",
                        e.getKey(), t.launchTick, t.launchDistTrue, t.estErrorAtLaunch);
                System.out.printf(Locale.US,
                        "    BEST acquisition: estimate came within %.0fm of true enemy (t=%d)%n",
                        t.minEstError, t.minEstErrorTick);
                System.out.printf(Locale.US,
                        "    closest the torpedo itself got to true enemy: %.0fm (t=%d)%n",
                        t.minDistTrue, t.minDistTrueTick);
                System.out.printf(Locale.US,
                        "    phases: acq=%s term=%s | pings=%d | end: phase=%s distTrue=%.0fm estErr=%.0fm fuel=%.0f det=%s%n",
                        t.reachedAcq, t.reachedTerm, t.pingTicks, t.lastPhase, t.lastDistTrue, t.lastEstError,
                        t.lastFuel, t.detonated);
                System.out.println("    => " + classify(t) + " :: " + diagnose(t));
            }
        }
        final int totalTorps = total;
        System.out.printf(Locale.US, "%n===== FAILURE BUCKETS over %d Claude torpedoes =====%n", totalTorps);
        buckets.forEach((b, n) -> {
            double[] s = bucketStats.get(b);
            System.out.printf(Locale.US,
                    "  %-22s %2d  (%2.0f%%)  avgClosest=%.0fm  avgFuelLeft=%.0fs  avgLaunchTick=%.0f%n",
                    b, n, 100.0 * n / totalTorps, s[0] / n, s[1] / n, s[2] / n);
        });
    }

    /** Coarse bucket for aggregate counting. */
    private static String classify(Torp t) {
        boolean acquired = t.minEstError < 150;       // active illuminated the true target
        if (t.detonated && t.minDistTrue < 35)
            return "HIT";
        if (t.minDistTrue < 180)
            return "TERMINAL_OVERSHOOT";               // got close, couldn't close last ~50m
        if (acquired && t.lastEstError > 400)
            return "LOST_TRACK_AFTER_ACQ";             // acquired then estimate diverged
        if (acquired)
            return "ACQUIRED_NEVER_CLOSED";            // good solution but never arrived (time/closure)
        return "NEVER_ACQUIRED";
    }

    private static String diagnose(Torp t) {
        if (t.detonated && t.minDistTrue < 35) {
            return "HIT/near-hit";
        }
        if (t.minEstError > 250) {
            return String.format(Locale.US,
                    "NEVER ACQUIRED: best solution was %.0fm off the true target - active ping never illuminated "
                            + "the moved target; flew blind on the stale launch lead", t.minEstError);
        }
        if (t.minEstError < 150 && t.minDistTrue > 80) {
            return String.format(Locale.US,
                    "ACQUIRED (within %.0fm) but still missed by %.0fm - terminal geometry/turn failure, not acquisition",
                    t.minEstError, t.minDistTrue);
        }
        return String.format(Locale.US, "partial: best estErr %.0fm, closest %.0fm", t.minEstError, t.minDistTrue);
    }

    private Map<Integer, Torp> runSeed(long seed) {
        MatchConfig base = MatchConfig.withDefaults(seed);
        MatchConfig config = new MatchConfig(base.worldSeed(), base.tickRateHz(), MAX_TICKS, base.submarineCount(),
                base.torpedoCount(), base.startingHp(), base.blastRadius(), base.minFuseRadius(), base.maxFuseRadius(),
                base.ratedDepth(), base.crushDepth(), base.battleArea(), base.terrainMarginMeters(),
                base.gridCellMeters(), base.minSeaFloorZ(), base.maxSeaFloorZ(), base.maxSubSpeed(), base.startTime());

        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1e9);

        List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
        List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        Map<Integer, Torp> torps = new LinkedHashMap<>();
        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> ts) {
                SubmarineSnapshot enemy = null;
                for (var s : subs)
                    if (s.id() != CLAUDE)
                        enemy = s;
                if (enemy == null)
                    return;
                double ex = enemy.pose().position().x(), ey = enemy.pose().position().y();
                for (var torp : ts) {
                    if (torp.ownerId() != CLAUDE)
                        continue;
                    Torp t = torps.computeIfAbsent(torp.id(), k -> new Torp());
                    double px = torp.pose().position().x(), py = torp.pose().position().y();
                    double distTrue = Math.hypot(px - ex, py - ey);
                    double estErr = Math.hypot(torp.diagEstX() - ex, torp.diagEstY() - ey);
                    if (t.launchTick < 0) {
                        t.launchTick = tick;
                        t.launchDistTrue = distTrue;
                        t.estErrorAtLaunch = estErr;
                    }
                    if (distTrue < t.minDistTrue) { t.minDistTrue = distTrue; t.minDistTrueTick = tick; }
                    if (estErr < t.minEstError) { t.minEstError = estErr; t.minEstErrorTick = tick; }
                    String phase = torp.diagPhase() == null ? "" : torp.diagPhase();
                    if ("ACQUISITION".equals(phase)) t.reachedAcq = true;
                    if ("TERMINAL".equals(phase)) t.reachedTerm = true;
                    if (torp.pingRequested()) t.pingTicks++;
                    if (torp.detonated()) t.detonated = true;
                    t.lastTick = tick;
                    t.lastPhase = phase;
                    t.lastDistTrue = distTrue;
                    t.lastEstError = estErr;
                    t.lastFuel = torp.fuelRemaining();
                    t.lastSpeed = torp.speed();
                }
            }

            @Override
            public void onMatchEnd() {}
        };
        sim.run(world, controllers, vehicles, listener);
        return torps;
    }
}
