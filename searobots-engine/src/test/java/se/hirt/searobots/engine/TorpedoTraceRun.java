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
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.List;

/**
 * Traces torpedo guidance diagnostics for analysis.
 * Compares: estimated target position/heading vs actual, vs intercept point.
 *
 * Usage: java TorpedoTraceRun [torpedoId] [seed]
 *   defaults: torpedoId=1003, seed=daf0549d9456a568
 */
public class TorpedoTraceRun {
    public static void main(String[] args) {
        int traceTorpId = args.length > 0 ? Integer.parseInt(args[0]) : 1003;
        long seed = args.length > 1
                ? Long.parseUnsignedLong(args[1], 16)
                : Long.parseUnsignedLong("daf0549d9456a568", 16);

        var world = new WorldGenerator().generate(MatchConfig.withDefaults(seed));
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.of(new ClaudeAttackSub(), new CodexAttackSub());
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        // TSV header
        System.out.println(String.join("\t",
                "tick",
                // torpedo state
                "torpX", "torpY", "torpZ", "torpHdg", "torpSpd", "fuel",
                // estimated target (from controller diagnostics)
                "estX", "estY", "estZ", "estHdg", "estSpd",
                // intercept point (where torpedo steers toward)
                "intX", "intY", "intZ",
                // actual target
                "actX", "actY", "actZ", "actHdg", "actSpd",
                // derived
                "tgtDist2D", "tgtDist3D", "estErr2D", "estErrZ", "intErr2D",
                "estHdgErr",
                // phase and ping
                "phase", "ping"
        ));

        sim.run(world, controllers, configs, new SimulationListener() {
            boolean headerPrinted = false;

            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
                for (var t : torpedoes) {
                    if (t.id() != traceTorpId) continue;

                    // Find target sub
                    SubmarineSnapshot target = null;
                    for (var s : subs) {
                        if (s.id() != t.ownerId()) target = s;
                    }
                    if (target == null) continue;

                    var tp = t.pose().position();
                    var ap = target.pose().position();

                    // 2D and 3D distance to actual target
                    double dx = ap.x() - tp.x(), dy = ap.y() - tp.y(), dz = ap.z() - tp.z();
                    double dist2D = Math.sqrt(dx * dx + dy * dy);
                    double dist3D = Math.sqrt(dx * dx + dy * dy + dz * dz);

                    // Estimation error
                    double estErr2D = Double.NaN, estErrZ = Double.NaN, intErr2D = Double.NaN;
                    double estHdgErr = Double.NaN;
                    if (!Double.isNaN(t.diagEstX())) {
                        double ex = t.diagEstX() - ap.x(), ey = t.diagEstY() - ap.y();
                        estErr2D = Math.sqrt(ex * ex + ey * ey);
                        estErrZ = t.diagEstZ() - ap.z();
                    }
                    if (!Double.isNaN(t.diagIntX())) {
                        double ix = t.diagIntX() - ap.x(), iy = t.diagIntY() - ap.y();
                        intErr2D = Math.sqrt(ix * ix + iy * iy);
                    }
                    if (!Double.isNaN(t.diagEstHeading())) {
                        estHdgErr = t.diagEstHeading() - target.pose().heading();
                        while (estHdgErr > Math.PI) estHdgErr -= 2 * Math.PI;
                        while (estHdgErr < -Math.PI) estHdgErr += 2 * Math.PI;
                    }

                    // Print every tick when close (<800m), every 25 ticks otherwise
                    boolean close = dist2D < 800;
                    if (!close && tick % 25 != 0) continue;

                    System.out.printf("%d\t" +
                                    "%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.0f\t" +
                                    "%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t" +
                                    "%.1f\t%.1f\t%.1f\t" +
                                    "%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t" +
                                    "%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t" +
                                    "%.1f\t" +
                                    "%s\t%b%n",
                            tick,
                            tp.x(), tp.y(), tp.z(), Math.toDegrees(t.pose().heading()), t.speed(), t.fuelRemaining(),
                            t.diagEstX(), t.diagEstY(), t.diagEstZ(),
                            Double.isNaN(t.diagEstHeading()) ? Double.NaN : Math.toDegrees(t.diagEstHeading()),
                            t.diagEstSpeed(),
                            t.diagIntX(), t.diagIntY(), t.diagIntZ(),
                            ap.x(), ap.y(), ap.z(), Math.toDegrees(target.pose().heading()), target.speed(),
                            dist2D, dist3D, estErr2D, estErrZ, intErr2D,
                            Double.isNaN(estHdgErr) ? Double.NaN : Math.toDegrees(estHdgErr),
                            t.diagPhase(), t.pingRequested()
                    );

                    if (!t.alive()) {
                        System.err.printf("TORPEDO %d DEAD at tick %d, detonated=%b, range=%.0fm%n",
                                t.id(), tick, t.detonated(), dist3D);
                        sim.stop();
                    }
                }
            }

            @Override public void onMatchEnd() {
                System.err.println("Match ended.");
            }
        });
    }
}
