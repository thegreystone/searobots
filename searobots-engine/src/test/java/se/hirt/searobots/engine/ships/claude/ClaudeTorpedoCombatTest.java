package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.SubmarineDrone;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.ArrayList;
import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

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
}
