package se.hirt.searobots.engine;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import static se.hirt.searobots.api.VehicleConfig.submarine;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests that the autopilot can safely navigate generated terrain
 * without collisions or getting stuck.
 */
class WorldNavigationTest {

    // -- Shared result record and helper ------------------------------------

    record PatrolResult(long seed, int tickLimit, int[] hp, double[] distanceMoved, boolean timedOut) {
        boolean survived(int subIndex) {
            return hp[subIndex] > 0;
        }

        boolean noTerrainDamage(int subIndex) {
            return hp[subIndex] == 1000;
        }
    }

    private PatrolResult runPatrolTest(long seed, int tickLimit) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new DefaultAttackSub());
        List<VehicleConfig> configs = List.of(submarine(), submarine());

        int[] finalHp = {1000, 1000};
        double[][] spawnPos = new double[2][3];
        double[][] lastPos = new double[2][3];
        boolean[] posRecorded = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.size() < 2) return;
                for (int i = 0; i < 2; i++) {
                    var s = submarines.get(i);
                    var pos = s.pose().position();
                    if (!posRecorded[0]) {
                        spawnPos[i][0] = pos.x();
                        spawnPos[i][1] = pos.y();
                        spawnPos[i][2] = pos.z();
                    }
                    lastPos[i][0] = pos.x();
                    lastPos[i][1] = pos.y();
                    lastPos[i][2] = pos.z();
                    finalHp[i] = s.hp();
                }
                posRecorded[0] = true;
                if (tick >= tickLimit) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        long timeoutMs = (tickLimit <= 3000) ? 60_000 : 120_000;
        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        boolean timedOut = false;
        try {
            thread.join(timeoutMs);
            if (thread.isAlive()) timedOut = true;
        } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        double[] dist = new double[2];
        for (int i = 0; i < 2; i++) {
            double dx = lastPos[i][0] - spawnPos[i][0];
            double dy = lastPos[i][1] - spawnPos[i][1];
            double dz = lastPos[i][2] - spawnPos[i][2];
            dist[i] = Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        return new PatrolResult(seed, tickLimit, finalHp, dist, timedOut);
    }

    // -- 4a. Patrol Survival -----------------------------------------------

    @Test
    void patrolSurvivalMultipleSeeds() {
        long[] seeds = {42, 123, 999, 2024, 7777};
        int tickLimit = 6000; // 120s at 50Hz
        int passed = 0;

        System.out.println("=== Patrol Survival (120s, 5 seeds) ===");
        for (long seed : seeds) {
            var r = runPatrolTest(seed, tickLimit);
            boolean ok = r.noTerrainDamage(0) && r.noTerrainDamage(1);
            if (ok) passed++;
            System.out.printf("  Seed %5d: sub0 hp=%4d dist=%.0fm | sub1 hp=%4d dist=%.0fm  %s%n",
                    seed, r.hp[0], r.distanceMoved[0], r.hp[1], r.distanceMoved[1],
                    ok ? "PASS" : "FAIL");
        }
        System.out.printf("Result: %d/%d seeds passed%n", passed, seeds.length);
        assertTrue(passed >= 3,
                "At least 3/5 seeds should survive with no terrain damage, got " + passed + "/5");
    }

    // -- 4b. Cross-Map Navigation ------------------------------------------

    @Test
    void crossMapNavigation() {
        int tickLimit = 3000; // 60s at 50Hz
        var r = runPatrolTest(42, tickLimit);

        System.out.println("=== Cross-Map Navigation (60s, seed 42) ===");
        System.out.printf("  Sub0: hp=%d dist=%.0fm | Sub1: hp=%d dist=%.0fm%n",
                r.hp[0], r.distanceMoved[0], r.hp[1], r.distanceMoved[1]);

        assertTrue(r.survived(0), "Sub 0 should survive (hp=" + r.hp[0] + ")");
        assertTrue(r.survived(1), "Sub 1 should survive (hp=" + r.hp[1] + ")");
        assertTrue(r.distanceMoved[0] > 100,
                "Sub 0 should move > 100m from spawn, moved " + r.distanceMoved[0] + "m");
        assertTrue(r.distanceMoved[1] > 100,
                "Sub 1 should move > 100m from spawn, moved " + r.distanceMoved[1] + "m");
    }

    // -- 4c. Full Patrol Circuit -------------------------------------------

    @Test
    void fullPatrolCircuit() {
        int tickLimit = 6000; // 120s at 50Hz
        var r = runPatrolTest(42, tickLimit);

        System.out.println("=== Full Patrol Circuit (120s, seed 42) ===");
        System.out.printf("  Sub0: hp=%d dist=%.0fm | Sub1: hp=%d dist=%.0fm%n",
                r.hp[0], r.distanceMoved[0], r.hp[1], r.distanceMoved[1]);

        boolean atLeastOneFullHp = r.noTerrainDamage(0) || r.noTerrainDamage(1);
        assertTrue(atLeastOneFullHp,
                "At least one sub should survive with full HP (1000). "
                        + "Sub0 hp=" + r.hp[0] + ", Sub1 hp=" + r.hp[1]);

        // The full-HP sub should also have moved > 200m
        double bestDist = r.noTerrainDamage(0) ? r.distanceMoved[0] : r.distanceMoved[1];
        assertTrue(bestDist > 200,
                "Full-HP sub should move > 200m, moved " + bestDist + "m");
    }

    // -- 4d. Stress Test ---------------------------------------------------

    @Tag("slow")
    @Test
    void stressTestMultipleSeeds() {
        long[] seeds = {42, 100, 200, 300, 400, 500, 600, 700, 800, 900};
        int tickLimit = 3000; // 60s at 50Hz
        int passed = 0;

        System.out.println("=== Stress Test (60s, 10 seeds) ===");
        for (long seed : seeds) {
            var r = runPatrolTest(seed, tickLimit);
            boolean ok = r.noTerrainDamage(0) && r.noTerrainDamage(1);
            if (ok) passed++;
            System.out.printf("  Seed %4d: sub0 hp=%4d dist=%.0fm | sub1 hp=%4d dist=%.0fm  %s%s%n",
                    seed, r.hp[0], r.distanceMoved[0], r.hp[1], r.distanceMoved[1],
                    ok ? "PASS" : "FAIL",
                    r.timedOut ? " (TIMEOUT)" : "");
        }
        System.out.printf("Survival: %d/%d seeds with no terrain damage%n", passed, seeds.length);
        assertTrue(passed >= 8,
                "At least 8/10 seeds should survive with no terrain damage, got " + passed + "/10");
    }
}
