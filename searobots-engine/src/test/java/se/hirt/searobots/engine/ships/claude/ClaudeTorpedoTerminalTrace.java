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
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Traces Claude torpedo terminal approach to diagnose miss patterns.
 * Tracks target error, speed, ping timing, and closing geometry.
 */
public class ClaudeTorpedoTerminalTrace {

    static final double TRACE_RADIUS = 500.0;

    @Test
    void traceTerminal() {
        // Run multiple seeds and trace all torpedoes within 500m of target
        long[] seeds = {0x1e001, 0x3000, 0xa000, 0xdeadbeefL, 0x77777777L};

        for (long seed : seeds) {
            System.out.printf("%n=== Seed %s ===%n", Long.toHexString(seed));
            runTrace(seed);
        }
    }

    private void runTrace(long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(new ClaudeAttackSub(), new CodexAttackSub());
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());
        Map<Integer, TorpState> states = new HashMap<>();

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() < 2) return;
                var claude = subs.get(0);
                var codex = subs.get(1);

                for (var torp : torps) {
                    if (torp.ownerId() != claude.id()) continue;
                    // Log all torpedoes regardless of alive state

                    var st = states.computeIfAbsent(torp.id(), k -> new TorpState());
                    double distToTarget = torp.pose().position().distanceTo(codex.pose().position());

                    // Target error: how far is the torpedo's intercept point from the actual target?
                    double targetErr = Double.NaN;
                    double targetJump = Double.NaN;
                    if (!Double.isNaN(torp.targetX()) && !Double.isNaN(torp.targetY())) {
                        targetErr = Math.hypot(
                                torp.targetX() - codex.pose().position().x(),
                                torp.targetY() - codex.pose().position().y());
                        if (!Double.isNaN(st.lastTX)) {
                            targetJump = Math.hypot(torp.targetX() - st.lastTX, torp.targetY() - st.lastTY);
                        }
                        st.lastTX = torp.targetX();
                        st.lastTY = torp.targetY();
                    }

                    // Depth error
                    double depthErr = Double.NaN;
                    if (!Double.isNaN(torp.targetZ())) {
                        depthErr = torp.targetZ() - codex.pose().position().z();
                    }

                    // Closing speed
                    double closingSpeed = Double.NaN;
                    if (!Double.isNaN(st.lastDist)) {
                        closingSpeed = (st.lastDist - distToTarget) / 0.02; // per tick
                    }
                    st.lastDist = distToTarget;

                    // Track minimum distance
                    st.minDist = Math.min(st.minDist, distToTarget);

                    // Start tracing when within range
                    if (!st.tracing && distToTarget <= TRACE_RADIUS) {
                        st.tracing = true;
                        System.out.printf("ENTER tick=%d torp=%d dist=%.0f speed=%.1f tgtErr=%.0f depthErr=%.0f%n",
                                tick, torp.id(), distToTarget, torp.speed(), targetErr, depthErr);
                    }

                    if (st.tracing) {
                        System.out.printf("  t=%d d=%.0f spd=%.1f close=%.1f tgtE=%.0f dE=%.0f jump=%.0f ping=%s sl=%.0f fuel=%.0f%n",
                                tick, distToTarget, torp.speed(), closingSpeed,
                                targetErr, depthErr, targetJump, torp.pingRequested(),
                                torp.sourceLevelDb(), torp.fuelRemaining());
                    }

                    // Log any torpedo death (even outside trace radius)
                    if (!st.done && (torp.detonated() || !torp.alive())) {
                        st.done = true;
                        System.out.printf("END tick=%d torp=%d det=%s dist=%.0f minDist=%.0f tgtErr=%.0f hp=%d%n",
                                tick, torp.id(), torp.detonated(), distToTarget, st.minDist,
                                targetErr, codex.hp());
                    }
                }

                if (tick >= 90_000 || claude.hp() <= 0 || codex.hp() <= 0) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}
    }

    static class TorpState {
        double lastTX = Double.NaN, lastTY = Double.NaN;
        double lastDist = Double.NaN;
        double minDist = Double.MAX_VALUE;
        boolean tracing, done;
    }
}
