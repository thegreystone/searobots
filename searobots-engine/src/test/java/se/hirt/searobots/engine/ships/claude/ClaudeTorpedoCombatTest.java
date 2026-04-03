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
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.SubmarineDrone;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.List;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Combat tests for Claude's torpedo system (sub + torpedo controller).
 * Tests against drones first (easy targets), then against active opponents.
 */
public class ClaudeTorpedoCombatTest {

    private static final int TICKS_10MIN = 30_000;
    private static final int TICKS_15MIN = 45_000;
    private static final int TICKS_30MIN = 90_000;

    record CombatOutcome(int hpA, int hpB, int torpsAFired, int torpsBFired,
                         int detonations, long endTick) {
        boolean aHitB() { return hpB < 1000; }
        boolean bHitA() { return hpA < 1000; }
        boolean aKilledB() { return hpB <= 0; }
    }

    private CombatOutcome runCombat(Supplier<SubmarineController> a, Supplier<SubmarineController> b,
                                     long seed, int maxTicks) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(a.get(), b.get());
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        int[] hpA = {1000}, hpB = {1000};
        int[] torpsA = {0}, torpsB = {0};
        int[] detonations = {0};
        long[] endTick = {maxTicks};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() >= 2) {
                    hpA[0] = subs.get(0).hp();
                    hpB[0] = subs.get(1).hp();
                    torpsA[0] = config.torpedoCount() - subs.get(0).torpedoesRemaining();
                    torpsB[0] = config.torpedoCount() - subs.get(1).torpedoesRemaining();
                    for (var t : torps) {
                        if (t.detonated()) detonations[0]++;
                    }
                }
                if (hpA[0] <= 0 || hpB[0] <= 0) { endTick[0] = tick; sim.stop(); }
                if (tick >= maxTicks) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}

        return new CombatOutcome(hpA[0], hpB[0], torpsA[0], torpsB[0], detonations[0], endTick[0]);
    }

    // ── Drone scenarios (torpedo should reliably hit) ──

    @Test
    void claudeVsDrone_10seeds() {
        long[] seeds = {0x1000, 0x2000, 0x3000, 0x4000, 0x5000,
                        0x6000, 0x7000, 0x8000, 0x9000, 0xA000};
        int hits = 0, kills = 0, torpsFired = 0;

        for (long seed : seeds) {
            var result = runCombat(ClaudeAttackSub::new, SubmarineDrone::new, seed, TICKS_15MIN);
            torpsFired += result.torpsAFired;
            if (result.aHitB()) hits++;
            if (result.aKilledB()) kills++;
            System.out.printf("seed=%04x  Claude hp=%d  Drone hp=%d  torps=%d  det=%d  %s%n",
                    seed, result.hpA, result.hpB, result.torpsAFired, result.detonations,
                    result.aKilledB() ? "KILL" : result.aHitB() ? "HIT" : "miss");
        }

        System.out.printf("%nDrone results: %d/%d hits, %d/%d kills, %d torps fired%n",
                hits, seeds.length, kills, seeds.length, torpsFired);
        assertTrue(torpsFired > 0, "Claude should fire at least some torpedoes");
        assertTrue(hits >= 3, "Should hit drone on at least 3/10 seeds, got " + hits);
    }

    @Test
    void claudeVsDefault_10seeds() {
        long[] seeds = {0x1000, 0x2000, 0x3000, 0x4000, 0x5000,
                        0x6000, 0x7000, 0x8000, 0x9000, 0xA000};
        int hits = 0, kills = 0, torpsFired = 0;
        int gotHit = 0;

        for (long seed : seeds) {
            var result = runCombat(ClaudeAttackSub::new, DefaultAttackSub::new, seed, TICKS_30MIN);
            torpsFired += result.torpsAFired;
            if (result.aHitB()) hits++;
            if (result.aKilledB()) kills++;
            if (result.bHitA()) gotHit++;
            System.out.printf("seed=%04x  Claude hp=%d  Default hp=%d  torpsC=%d torpsD=%d  %s%n",
                    seed, result.hpA, result.hpB, result.torpsAFired, result.torpsBFired,
                    result.aKilledB() ? "KILL" : result.aHitB() ? "HIT" : "miss");
        }

        System.out.printf("%nDefault results: %d/%d hits, %d/%d kills, got hit %d times, %d torps fired%n",
                hits, seeds.length, kills, seeds.length, gotHit, torpsFired);
    }

    @Test
    void claudeVsCodex_10seeds() {
        long[] seeds = {0x1000, 0x2000, 0x3000, 0x4000, 0x5000,
                        0x6000, 0x7000, 0x8000, 0x9000, 0xA000};
        int hits = 0, kills = 0, torpsFired = 0;
        int gotHit = 0;

        for (long seed : seeds) {
            var result = runCombat(ClaudeAttackSub::new, CodexAttackSub::new, seed, TICKS_10MIN);
            torpsFired += result.torpsAFired;
            if (result.aHitB()) hits++;
            if (result.aKilledB()) kills++;
            if (result.bHitA()) gotHit++;
            System.out.printf("seed=%04x  Claude hp=%d  Codex hp=%d  torpsC=%d torpsX=%d  %s%n",
                    seed, result.hpA, result.hpB, result.torpsAFired, result.torpsBFired,
                    result.aKilledB() ? "KILL" : result.aHitB() ? "HIT" : "miss");
        }

        System.out.printf("%nCodex results: %d/%d hits, %d/%d kills, got hit %d times, %d torps fired%n",
                hits, seeds.length, kills, seeds.length, gotHit, torpsFired);
    }

    @Test
    void claudeVsCodexBatch() {
        long[] seeds = {0x1000, 0x2000, 0x3000, 0x4000, 0x5000,
                        0x6000, 0x7000, 0x8000, 0x9000, 0xA000,
                        0xB000, 0xC000, 0xD000, 0xE000, 0xF000,
                        0x1100, 0x2200, 0x3300, 0x4400, 0x5500};
        int claudeWins = 0, codexWins = 0, draws = 0;
        int claudeHits = 0, codexHits = 0;
        int claudeKills = 0, codexKills = 0;
        int claudeTorps = 0, codexTorps = 0;
        int totalClaudeDmg = 0, totalCodexDmg = 0;

        System.out.println("=== Claude vs Codex: 20-Match Combat Batch ===");
        System.out.println("──────────────────────────────────────────────────────────────────────────────");
        System.out.printf("%-6s  %10s  %10s  %7s  %7s  %6s  %s%n",
                "Seed", "Claude HP", "Codex HP", "C Torps", "X Torps", "Tick", "Result");
        System.out.println("──────────────────────────────────────────────────────────────────────────────");

        for (long seed : seeds) {
            var r = runCombat(ClaudeAttackSub::new, CodexAttackSub::new, seed, TICKS_10MIN);
            int claudeDmg = 1000 - r.hpB;  // damage Claude dealt to Codex
            int codexDmg = 1000 - r.hpA;   // damage Codex dealt to Claude
            totalClaudeDmg += claudeDmg;
            totalCodexDmg += codexDmg;
            claudeTorps += r.torpsAFired;
            codexTorps += r.torpsBFired;
            if (r.aHitB()) claudeHits++;
            if (r.bHitA()) codexHits++;
            if (r.aKilledB()) claudeKills++;
            if (r.hpA <= 0) codexKills++;

            String outcome;
            if (r.hpB <= 0 && r.hpA > 0) { outcome = "CLAUDE WIN"; claudeWins++; }
            else if (r.hpA <= 0 && r.hpB > 0) { outcome = "CODEX WIN"; codexWins++; }
            else if (r.hpA <= 0 && r.hpB <= 0) { outcome = "MUTUAL KILL"; draws++; }
            else if (claudeDmg > codexDmg) { outcome = "Claude dmg+" + (claudeDmg - codexDmg); claudeWins++; }
            else if (codexDmg > claudeDmg) { outcome = "Codex dmg+" + (codexDmg - claudeDmg); codexWins++; }
            else { outcome = "DRAW"; draws++; }

            System.out.printf("0x%04x  %10d  %10d  %7d  %7d  %6d  %s%n",
                    seed, r.hpA, r.hpB, r.torpsAFired, r.torpsBFired, r.endTick, outcome);
        }

        System.out.println("──────────────────────────────────────────────────────────────────────────────");
        System.out.println("\n=== SUMMARY ===");
        System.out.printf("Claude wins: %d / %d%n", claudeWins, seeds.length);
        System.out.printf("Codex  wins: %d / %d%n", codexWins, seeds.length);
        System.out.printf("Draws:       %d / %d%n", draws, seeds.length);
        System.out.printf("Claude kills: %d,  Codex kills: %d%n", claudeKills, codexKills);
        System.out.printf("Claude hits:  %d,  Codex hits:  %d%n", claudeHits, codexHits);
        System.out.printf("Claude torps fired: %d,  Codex torps fired: %d%n", claudeTorps, codexTorps);
        System.out.printf("Total dmg Claude dealt: %d,  Total dmg Codex dealt: %d%n", totalClaudeDmg, totalCodexDmg);
        System.out.printf("Avg dmg Claude dealt: %.1f,  Avg dmg Codex dealt: %.1f%n",
                (double) totalClaudeDmg / seeds.length, (double) totalCodexDmg / seeds.length);
    }

    @Test
    void claudeVsCodex50() {
        // 50 seeds: i * 0x1111 for i = 1..50
        long[] seeds = new long[50];
        for (int i = 0; i < 50; i++) {
            seeds[i] = (long)(i + 1) * 0x1111;
        }

        int claudeWins = 0, codexWins = 0, draws = 0;
        int claudeHits = 0, codexHits = 0;
        int claudeKills = 0, codexKills = 0;
        int claudeTorps = 0, codexTorps = 0;
        int totalClaudeDmg = 0, totalCodexDmg = 0;

        System.out.println("=== Claude vs Codex: 50-Match Combat Batch ===");
        System.out.println("──────────────────────────────────────────────────────────────────────────────");
        System.out.printf("%-8s  %10s  %10s  %7s  %7s  %6s  %s%n",
                "Seed", "Claude HP", "Codex HP", "C Torps", "X Torps", "Tick", "Result");
        System.out.println("──────────────────────────────────────────────────────────────────────────────");

        for (long seed : seeds) {
            var r = runCombat(ClaudeAttackSub::new, CodexAttackSub::new, seed, TICKS_10MIN);
            int claudeDmg = 1000 - r.hpB;  // damage Claude dealt to Codex
            int codexDmg = 1000 - r.hpA;   // damage Codex dealt to Claude
            totalClaudeDmg += claudeDmg;
            totalCodexDmg += codexDmg;
            claudeTorps += r.torpsAFired;
            codexTorps += r.torpsBFired;
            if (r.aHitB()) claudeHits++;
            if (r.bHitA()) codexHits++;
            if (r.aKilledB()) claudeKills++;
            if (r.hpA <= 0) codexKills++;

            String outcome;
            if (r.hpB <= 0 && r.hpA > 0) { outcome = "CLAUDE WIN"; claudeWins++; }
            else if (r.hpA <= 0 && r.hpB > 0) { outcome = "CODEX WIN"; codexWins++; }
            else if (r.hpA <= 0 && r.hpB <= 0) { outcome = "MUTUAL KILL"; draws++; }
            else if (claudeDmg > codexDmg) { outcome = "Claude dmg+" + (claudeDmg - codexDmg); claudeWins++; }
            else if (codexDmg > claudeDmg) { outcome = "Codex dmg+" + (codexDmg - claudeDmg); codexWins++; }
            else { outcome = "DRAW"; draws++; }

            System.out.printf("0x%05x  %10d  %10d  %7d  %7d  %6d  %s%n",
                    seed, r.hpA, r.hpB, r.torpsAFired, r.torpsBFired, r.endTick, outcome);
        }

        System.out.println("──────────────────────────────────────────────────────────────────────────────");
        System.out.println("\n=== SUMMARY (50 seeds) ===");
        System.out.printf("Claude wins: %d / %d%n", claudeWins, seeds.length);
        System.out.printf("Codex  wins: %d / %d%n", codexWins, seeds.length);
        System.out.printf("Draws:       %d / %d%n", draws, seeds.length);
        System.out.printf("Claude kills: %d,  Codex kills: %d%n", claudeKills, codexKills);
        System.out.printf("Claude hits:  %d,  Codex hits:  %d%n", claudeHits, codexHits);
        System.out.printf("Claude torps fired: %d,  Codex torps fired: %d%n", claudeTorps, codexTorps);
        System.out.printf("Total dmg Claude dealt: %d,  Total dmg Codex dealt: %d%n", totalClaudeDmg, totalCodexDmg);
        System.out.printf("Avg dmg Claude dealt: %.1f,  Avg dmg Codex dealt: %.1f%n",
                (double) totalClaudeDmg / seeds.length, (double) totalCodexDmg / seeds.length);
        System.out.printf("Win rate: %.1f%%%n", 100.0 * claudeWins / seeds.length);
    }

    @Test
    void claudeVsCodex50b() {
        // Different seed set: 0xDEAD0000 + i * 0x7331
        long[] seeds = new long[50];
        for (int i = 0; i < 50; i++) {
            seeds[i] = 0xDEAD0000L + (long)(i + 1) * 0x7331;
        }

        int claudeWins = 0, codexWins = 0, draws = 0;
        int claudeKills = 0, codexKills = 0;
        int claudeTorps = 0, codexTorps = 0;
        int totalClaudeDmg = 0, totalCodexDmg = 0;

        System.out.println("=== Claude vs Codex: 50-Match Batch B ===");
        System.out.printf("%-12s  %10s  %10s  %7s  %7s  %6s  %s%n",
                "Seed", "Claude HP", "Codex HP", "C Torps", "X Torps", "Tick", "Result");
        System.out.println("-".repeat(80));

        for (long seed : seeds) {
            var r = runCombat(ClaudeAttackSub::new, CodexAttackSub::new, seed, TICKS_10MIN);
            int claudeDmg = 1000 - r.hpB;
            int codexDmg = 1000 - r.hpA;
            totalClaudeDmg += claudeDmg;
            totalCodexDmg += codexDmg;
            claudeTorps += r.torpsAFired;
            codexTorps += r.torpsBFired;
            if (r.aKilledB()) claudeKills++;
            if (r.hpA <= 0) codexKills++;

            String outcome;
            if (r.hpB <= 0 && r.hpA > 0) { outcome = "CLAUDE WIN"; claudeWins++; }
            else if (r.hpA <= 0 && r.hpB > 0) { outcome = "CODEX WIN"; codexWins++; }
            else if (r.hpA <= 0 && r.hpB <= 0) { outcome = "MUTUAL KILL"; draws++; }
            else if (claudeDmg > codexDmg) { outcome = "Claude dmg+" + (claudeDmg - codexDmg); claudeWins++; }
            else if (codexDmg > claudeDmg) { outcome = "Codex dmg+" + (codexDmg - claudeDmg); codexWins++; }
            else { outcome = "DRAW"; draws++; }

            System.out.printf("0x%08x  %10d  %10d  %7d  %7d  %6d  %s%n",
                    seed, r.hpA, r.hpB, r.torpsAFired, r.torpsBFired, r.endTick, outcome);
        }

        System.out.println("-".repeat(80));
        System.out.printf("Claude wins: %d / %d (%.0f%%)%n", claudeWins, seeds.length, 100.0 * claudeWins / seeds.length);
        System.out.printf("Codex  wins: %d / %d%n", codexWins, seeds.length);
        System.out.printf("Draws:       %d / %d%n", draws, seeds.length);
        System.out.printf("Claude kills: %d,  Codex kills: %d%n", claudeKills, codexKills);
        System.out.printf("Claude torps: %d,  Codex torps: %d%n", claudeTorps, codexTorps);
        System.out.printf("Dmg dealt: Claude %d,  Codex %d%n", totalClaudeDmg, totalCodexDmg);
    }
}
