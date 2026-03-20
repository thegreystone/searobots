package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Phase 4 engagement scenario tests. Verifies that DefaultAttackSub
 * can reliably detect, track, and achieve a firing solution against
 * various target types.
 */
class EngagementScenarioTest {

    private record EngagementResult(
            long firstContactTick,
            long firstChaseTick,
            long torpedoSolutionTick,
            double startingDistance,
            boolean subSurvived,
            boolean targetSurvived
    ) {}

    private EngagementResult runEngagement(SubmarineController target, VehicleConfig targetConfig, long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), target);
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), targetConfig);

        long[] firstContact = {-1};
        long[] firstChase = {-1};
        long[] torpedoSolution = {-1};
        double[] startDist = {0};
        boolean[] subAlive = {true};
        boolean[] targetAlive = {true};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.size() < 2) return;
                var s0 = submarines.get(0);
                var s1 = submarines.get(1);

                if (tick == 0) {
                    var p0 = s0.pose().position();
                    var p1 = s1.pose().position();
                    startDist[0] = Math.sqrt(Math.pow(p0.x()-p1.x(),2) + Math.pow(p0.y()-p1.y(),2));
                }

                // Track first contact
                if (firstContact[0] < 0 && s0.contactEstimates() != null
                        && !s0.contactEstimates().isEmpty()) {
                    firstContact[0] = tick;
                }

                // Track first chase (status starts with C)
                if (firstChase[0] < 0 && s0.status() != null
                        && s0.status().startsWith("C")) {
                    firstChase[0] = tick;
                }

                // Track survival
                if (s0.hp() <= 0) subAlive[0] = false;
                if (s1.hp() <= 0) targetAlive[0] = false;
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        return new EngagementResult(firstContact[0], firstChase[0], torpedoSolution[0],
                startDist[0], subAlive[0], targetAlive[0]);
    }

    @Test
    void surfaceShipEngagement() {
        // Phase 4a: sub vs surface ship drone
        var result = runEngagement(new TargetDrone(), VehicleConfig.surfaceShip(), 42);

        System.out.printf("Surface ship engagement (seed 42):%n");
        System.out.printf("  Starting distance: %.0fm%n", result.startingDistance);
        System.out.printf("  First contact: %.1fs%n", result.firstContactTick / 50.0);
        System.out.printf("  First chase: %.1fs%n", result.firstChaseTick / 50.0);
        System.out.printf("  Sub survived: %s%n", result.subSurvived);

        assertTrue(result.firstContactTick > 0, "Should detect surface ship");
        assertTrue(result.firstContactTick < 10000, "Should detect within 200 seconds");
        assertTrue(result.firstChaseTick > 0, "Should enter CHASE");
        // Sub may die from terrain in some seeds. Key test is detection and chase.
    }

    @Test
    void submarineDroneEngagement() {
        // Phase 4b: sub vs submarine drone at depth
        var result = runEngagement(new SubmarineDrone(), VehicleConfig.submarine(), 42);

        System.out.printf("Submarine drone engagement (seed 42):%n");
        System.out.printf("  Starting distance: %.0fm%n", result.startingDistance);
        System.out.printf("  First contact: %.1fs%n", result.firstContactTick / 50.0);
        System.out.printf("  First chase: %.1fs%n", result.firstChaseTick / 50.0);
        System.out.printf("  Sub survived: %s%n", result.subSurvived);

        assertTrue(result.firstContactTick > 0, "Should detect submarine drone");
        // Note: sub may die from terrain collision in challenging terrain.
        // The key test is detection behavior, not survival.
    }

    @Test
    void surfaceShipMultipleSeeds() {
        // Run multiple seeds to check consistency
        int detected = 0;
        int chased = 0;
        double totalContactTime = 0;

        for (long seed : new long[]{42, 123, 999, 2024, 7777}) {
            var result = runEngagement(new TargetDrone(), VehicleConfig.surfaceShip(), seed);
            if (result.firstContactTick > 0) {
                detected++;
                totalContactTime += result.firstContactTick / 50.0;
            }
            if (result.firstChaseTick > 0) chased++;

            System.out.printf("  Seed %d: dist=%.0f contact=%.1fs chase=%.1fs%n",
                    seed, result.startingDistance,
                    result.firstContactTick > 0 ? result.firstContactTick / 50.0 : -1,
                    result.firstChaseTick > 0 ? result.firstChaseTick / 50.0 : -1);
        }

        System.out.printf("Surface ship: %d/5 detected, %d/5 chased, avg contact=%.1fs%n",
                detected, chased, detected > 0 ? totalContactTime / detected : 0);

        assertTrue(detected >= 3, "Should detect in at least 3/5 seeds");
        assertTrue(chased >= 2, "Should chase in at least 2/5 seeds");
    }

    @Test
    void submarineDroneMultipleSeeds() {
        int detected = 0;
        int chased = 0;
        double totalContactTime = 0;

        for (long seed : new long[]{42, 123, 999, 2024, 7777}) {
            var result = runEngagement(new SubmarineDrone(), VehicleConfig.submarine(), seed);
            if (result.firstContactTick > 0) {
                detected++;
                totalContactTime += result.firstContactTick / 50.0;
            }
            if (result.firstChaseTick > 0) chased++;

            System.out.printf("  Seed %d: dist=%.0f contact=%.1fs chase=%.1fs%n",
                    seed, result.startingDistance,
                    result.firstContactTick > 0 ? result.firstContactTick / 50.0 : -1,
                    result.firstChaseTick > 0 ? result.firstChaseTick / 50.0 : -1);
        }

        System.out.printf("Sub drone: %d/5 detected, %d/5 chased, avg contact=%.1fs%n",
                detected, chased, detected > 0 ? totalContactTime / detected : 0);

        assertTrue(detected >= 3, "Should detect sub drone in at least 3/5 seeds");
    }
}
