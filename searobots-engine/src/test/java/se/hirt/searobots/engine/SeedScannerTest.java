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

/**
 * Scans seeds to verify DefaultAttackSub fires torpedoes.
 */
public class SeedScannerTest {

    @Test
    void defaultSubFiresTorpedoes() {
        System.out.println("=== Default vs Claude ===");
        runMatchups("Default", DefaultAttackSub::new, "Claude", ClaudeAttackSub::new);
        System.out.println("\n=== Default vs Codex ===");
        runMatchups("Default", DefaultAttackSub::new, "Codex", CodexAttackSub::new);
    }

    private void runMatchups(String nameA, Supplier<SubmarineController> factoryA,
                             String nameB, Supplier<SubmarineController> factoryB) {
        int combat10min = 30_000;
        long[] seeds = new long[10];
        for (int i = 0; i < seeds.length; i++) {
            seeds[i] = 0x2000 + i * 6311L;
        }

        int totalTorpsFired = 0;
        for (long seed : seeds) {
            String hex = Long.toHexString(seed);
            var config = MatchConfig.withDefaults(seed);
            var world = new WorldGenerator().generate(config);

            var sim = new SimulationLoop();
            sim.setSpeedMultiplier(1_000_000);

            var controllers = List.<SubmarineController>of(factoryA.get(), factoryB.get());
            var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

            int[] hpA = {1000}, hpB = {1000};
            int[] torpsA = {0}, torpsB = {0};

            var listener = new SimulationListener() {
                @Override
                public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                    if (subs.size() >= 2) {
                        hpA[0] = subs.get(0).hp();
                        hpB[0] = subs.get(1).hp();
                        torpsA[0] = config.torpedoCount() - subs.get(0).torpedoesRemaining();
                        torpsB[0] = config.torpedoCount() - subs.get(1).torpedoesRemaining();
                    }
                    if (tick >= combat10min) sim.stop();
                }
                @Override public void onMatchEnd() {}
            };

            var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
            thread.start();
            try { thread.join(30_000); } catch (InterruptedException e) {}
            sim.stop();
            try { thread.join(3000); } catch (InterruptedException e) {}

            totalTorpsFired += torpsA[0];
            System.out.printf("seed=%s  %s hp=%d torps=%d  |  %s hp=%d torps=%d%n",
                    hex, nameA, hpA[0], torpsA[0], nameB, hpB[0], torpsB[0]);
        }
        System.out.printf("Total %s torpedoes fired: %d / %d seeds%n", nameA, totalTorpsFired, seeds.length);
    }
}
