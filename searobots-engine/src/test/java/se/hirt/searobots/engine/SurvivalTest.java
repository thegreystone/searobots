package se.hirt.searobots.engine;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

/**
 * Tests that the DefaultAttackSub survives terrain across multiple seeds.
 */
class SurvivalTest {

    @Test
    void subSurvivesMultipleSeeds() {
        long[] seeds = {42, 123, 999, 2024, 7777, 314, 55555, 8888, 1001, 6543};
        int survived = 0;
        int total = seeds.length;

        for (long seed : seeds) {
            var config = MatchConfig.withDefaults(seed);
            var world = new WorldGenerator().generate(config);
            var sim = new SimulationLoop();
            sim.setSpeedMultiplier(1_000_000);

            List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new SubmarineDrone());
            List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

            boolean[] subAlive = {true};
            long[] deathTick = {-1};
            double[] deathZ = {0};
            double[] deathFloor = {0};
            String[] deathStatus = {""};

            var listener = new SimulationListener() {
                @Override
                public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                    if (submarines.size() < 2) return;
                    var s0 = submarines.get(0);
                    if (s0.hp() <= 0 && subAlive[0]) {
                        subAlive[0] = false;
                        deathTick[0] = tick;
                        deathZ[0] = s0.pose().position().z();
                        deathStatus[0] = s0.status() != null ? s0.status() : "";
                        // Estimate floor from status (f:XXX)
                        var st = s0.status();
                        if (st != null && st.contains("f:")) {
                            try {
                                int fi = st.indexOf("f:") + 2;
                                int fe = st.indexOf(" ", fi);
                                if (fe < 0) fe = st.length();
                                deathFloor[0] = -Double.parseDouble(st.substring(fi, fe));
                            } catch (Exception ex) {}
                        }
                    }
                }
                @Override public void onMatchEnd() {}
            };

            var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
            thread.start();
            try { thread.join(60_000); } catch (InterruptedException e) {}
            sim.stop();
            try { thread.join(5000); } catch (InterruptedException e) {}

            if (subAlive[0]) {
                survived++;
                System.out.printf("  Seed %6d: SURVIVED%n", seed);
            } else {
                System.out.printf("  Seed %6d: DIED at tick %d (%.0fs) z=%.0f floor~%.0f %s%n",
                        seed, deathTick[0], deathTick[0] / 50.0,
                        deathZ[0], deathFloor[0], deathStatus[0]);
            }
        }

        System.out.printf("Survival: %d/%d seeds%n", survived, total);
    }
}
