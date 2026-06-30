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

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Fine-grained endgame trace of overshooting Claude torpedoes, to study whether
 * the continuously-calculated impact point is doing its job. For each torpedo,
 * every tick inside 800m of the TRUE target it records the geometry, then prints
 * the window around closest point of approach (CPA) for the torpedoes that
 * overshoot (CPA in 30..200m). Columns:
 * <pre>
 *   t       tick
 *   ph      guidance phase (TRA/ACQ/TER)
 *   spd     torpedo speed (m/s)
 *   rTrue   range torpedo->TRUE target (m)
 *   rEst    range torpedo->its own estimate (m)
 *   estErr  |estimate - TRUE target| (m)   how wrong the solution is
 *   intErr  |impact point - TRUE target| (m)  is the calculated impact point good?
 *   los     bearing to TRUE target (deg)
 *   hdgErr  angle between torpedo heading and bearing-to-TRUE (deg)
 *   missStr rTrue*sin(hdgErr): perpendicular miss if it flies straight (m)
 *   losRate d(los)/dt (deg/s): crossing rate the guidance must null
 * </pre>
 * A growing missStr / saturating losRate near CPA means the geometry outran the
 * torpedo's turn rate (control limit). A large intErr means the impact-point
 * prediction itself was bad (lead/prediction limit).
 */
@EnabledIfSystemProperty(named = "searobots.diag", matches = "true")
class ClaudeTorpedoOvershootTrace {

    private static final long[] SEEDS = {8, 13};
    private static final int MAX_TICKS = 30_000;
    private static final int CLAUDE = 1;
    private static final double ENDGAME_RANGE = 800.0;

    private record Sample(long t, String phase, double spd, double rTrue, double rEst, double estErr,
                          double intErr, double losDeg, double hdgErrDeg, double missStraight, double losRateDeg) {}

    private static final class Torp {
        final List<Sample> samples = new ArrayList<>();
        double prevLos = Double.NaN;
        long prevTick = -1;
        boolean detonated;
    }

    @Test
    void traceOvershoots() {
        for (long seed : SEEDS) {
            Map<Integer, Torp> torps = runSeed(seed);
            for (var e : torps.entrySet()) {
                Torp t = e.getValue();
                if (t.samples.isEmpty())
                    continue;
                // CPA = sample with min rTrue
                int cpa = 0;
                for (int i = 1; i < t.samples.size(); i++)
                    if (t.samples.get(i).rTrue() < t.samples.get(cpa).rTrue())
                        cpa = i;
                double cpaRange = t.samples.get(cpa).rTrue();
                boolean overshoot = cpaRange >= 30 && cpaRange <= 200 && !(t.detonated && cpaRange < 35);
                if (!overshoot)
                    continue;
                System.out.printf(Locale.US,
                        "%n===== seed %d torp %d : OVERSHOOT, CPA=%.0fm at t=%d (detonated=%s) =====%n",
                        seed, e.getKey(), cpaRange, t.samples.get(cpa).t(), t.detonated);
                System.out.println(
                        "      t   ph   spd  rTrue  rEst estErr intErr   los hdgErr missStr losRate");
                // Full endgame: coarse (every 15 ticks) until the last ~120 ticks, then every tick.
                int to = Math.min(t.samples.size() - 1, cpa + 12);
                for (int i = 0; i <= to; i++) {
                    boolean fine = i >= cpa - 24;
                    if (!fine && (i % 15 != 0))
                        continue;
                    Sample s = t.samples.get(i);
                    String mark = i == cpa ? " <CPA" : "";
                    System.out.printf(Locale.US,
                            "  %7d  %3s %5.1f %6.0f %5.0f %5.0f %6.0f %5.0f %6.1f %7.1f %7.1f%s%n",
                            s.t(), s.phase(), s.spd(), s.rTrue(), s.rEst(), s.estErr(), s.intErr(),
                            s.losDeg(), s.hdgErrDeg(), s.missStraight(), s.losRateDeg(), mark);
                }
            }
        }
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
                    var px = torp.pose().position().x();
                    var py = torp.pose().position().y();
                    double rTrue = Math.hypot(px - ex, py - ey);
                    Torp t = torps.computeIfAbsent(torp.id(), k -> new Torp());
                    if (torp.detonated())
                        t.detonated = true;
                    double los = norm(Math.atan2(ex - px, ey - py));
                    double losRateDeg = 0;
                    if (!Double.isNaN(t.prevLos) && t.prevTick >= 0 && tick > t.prevTick) {
                        double d = los - t.prevLos;
                        while (d > Math.PI) d -= 2 * Math.PI;
                        while (d < -Math.PI) d += 2 * Math.PI;
                        losRateDeg = Math.toDegrees(d / ((tick - t.prevTick) / 50.0));
                    }
                    t.prevLos = los;
                    t.prevTick = tick;
                    if (rTrue > ENDGAME_RANGE)
                        continue;
                    double hdg = torp.pose().heading();
                    double hdgErr = los - hdg;
                    while (hdgErr > Math.PI) hdgErr -= 2 * Math.PI;
                    while (hdgErr < -Math.PI) hdgErr += 2 * Math.PI;
                    double estErr = Math.hypot(torp.diagEstX() - ex, torp.diagEstY() - ey);
                    double intErr = Math.hypot(torp.diagIntX() - ex, torp.diagIntY() - ey);
                    double rEst = Math.hypot(px - torp.diagEstX(), py - torp.diagEstY());
                    String ph = switch (torp.diagPhase() == null ? "" : torp.diagPhase()) {
                        case "TRANSIT" -> "TRA";
                        case "ACQUISITION" -> "ACQ";
                        case "TERMINAL" -> "TER";
                        default -> "?";
                    };
                    t.samples.add(new Sample(tick, ph, torp.speed(), rTrue, rEst, estErr, intErr,
                            Math.toDegrees(los), Math.toDegrees(hdgErr), rTrue * Math.sin(hdgErr), losRateDeg));
                }
            }

            @Override
            public void onMatchEnd() {}
        };
        sim.run(world, controllers, vehicles, listener);
        return torps;
    }

    private static double norm(double a) {
        while (a < 0) a += 2 * Math.PI;
        while (a >= 2 * Math.PI) a -= 2 * Math.PI;
        return a;
    }
}
