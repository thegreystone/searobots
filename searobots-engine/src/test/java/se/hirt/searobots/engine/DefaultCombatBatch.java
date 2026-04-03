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
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.List;
import java.util.function.Supplier;

public class DefaultCombatBatch {

    private static final int TICKS_30MIN = 90_000;

    record Result(int hpA, int hpB) {
        boolean aWins() { return hpB <= 0 && hpA > 0; }
        boolean bWins() { return hpA <= 0 && hpB > 0; }
        boolean draw() { return !aWins() && !bWins(); }
    }

    private Result runCombat(Supplier<SubmarineController> a, Supplier<SubmarineController> b, long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);
        var controllers = List.<SubmarineController>of(a.get(), b.get());
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());
        int[] hp = {1000, 1000};
        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() >= 2) { hp[0] = subs.get(0).hp(); hp[1] = subs.get(1).hp(); }
                if (hp[0] <= 0 || hp[1] <= 0 || tick >= TICKS_30MIN) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };
        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}
        return new Result(hp[0], hp[1]);
    }

    @Test
    void defaultVsClaude50() {
        int dWins = 0, cWins = 0, draws = 0;
        for (int i = 0; i < 50; i++) {
            long seed = 0x1000 + i * 4937L;
            var r = runCombat(DefaultAttackSub::new, ClaudeAttackSub::new, seed);
            String outcome = r.aWins() ? "DEFAULT" : r.bWins() ? "CLAUDE" : "draw";
            if (r.aWins()) dWins++; else if (r.bWins()) cWins++; else draws++;
            System.out.printf("seed=%04x  Default hp=%d  Claude hp=%d  %s%n", seed, r.hpA, r.hpB, outcome);
        }
        System.out.printf("%n=== Default %d wins, Claude %d wins, %d draws ===%n", dWins, cWins, draws);
    }

    @Test
    void defaultVsCodex50() {
        int dWins = 0, xWins = 0, draws = 0;
        for (int i = 0; i < 50; i++) {
            long seed = 0x1000 + i * 4937L;
            var r = runCombat(DefaultAttackSub::new, CodexAttackSub::new, seed);
            String outcome = r.aWins() ? "DEFAULT" : r.bWins() ? "CODEX" : "draw";
            if (r.aWins()) dWins++; else if (r.bWins()) xWins++; else draws++;
            System.out.printf("seed=%04x  Default hp=%d  Codex hp=%d  %s%n", seed, r.hpA, r.hpB, outcome);
        }
        System.out.printf("%n=== Default %d wins, Codex %d wins, %d draws ===%n", dWins, xWins, draws);
    }
}
