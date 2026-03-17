package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

class ShallowDeathInvestigation {

    @Test
    void investigateDeaths() {
        long[] seeds = {7777, 55555, 999, 2024, 8888, 1001};
        for (long seed : seeds) {
            investigate(seed);
        }
    }

    private void investigate(long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        long[] deathTick = {-1};
        double[] deathX = {0}, deathY = {0};

        // Circular buffer: last 100 ticks before death
        int BUF = 200;
        String[] history = new String[BUF];
        int[] histIdx = {0};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.isEmpty()) return;
                var s0 = submarines.get(0);
                var pos = s0.pose().position();
                var status = s0.status() != null ? s0.status() : "";

                // Record every 5 ticks (0.1s) into circular buffer
                if (tick % 5 == 0 && deathTick[0] < 0) {
                    history[histIdx[0] % BUF] = String.format(
                            "  t=%-6d pos=[%7.0f,%7.0f,%6.0f] spd=%5.1f hdg=%5.1f thr=%5.2f rud=%5.2f hp=%d %s",
                            tick, pos.x(), pos.y(), pos.z(), s0.speed(),
                            Math.toDegrees(s0.pose().heading()), s0.throttle(),
                            0.0, // rudder not in snapshot, but status has state info
                            s0.hp(), status);
                    histIdx[0]++;
                }

                if (s0.hp() <= 0 && deathTick[0] < 0) {
                    deathTick[0] = tick;
                    deathX[0] = pos.x();
                    deathY[0] = pos.y();
                    System.out.printf("%n=== SEED %d: DEATH at t=%d (%.0fs) ===%n", seed, tick, tick / 50.0);
                    System.out.printf("  pos=[%.0f, %.0f, %.0f] speed=%.1f %s%n",
                            pos.x(), pos.y(), pos.z(), s0.speed(), status);

                    // Print last N entries from circular buffer
                    int count = Math.min(histIdx[0], BUF);
                    int start = (histIdx[0] - count) % BUF;
                    if (start < 0) start += BUF;
                    System.out.println("  --- Last " + count + " snapshots (every 0.1s) ---");
                    for (int i = 0; i < Math.min(count, 40); i++) {
                        System.out.println(history[(start + count - 40 + i) % BUF]);
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

        if (deathTick[0] > 0) {
            System.out.printf("  Terrain around death [%.0f, %.0f]:%n", deathX[0], deathY[0]);
            var terrain = world.terrain();
            for (int dy = -3; dy <= 3; dy++) {
                StringBuilder sb = new StringBuilder("    ");
                for (int dx = -3; dx <= 3; dx++) {
                    double elev = terrain.elevationAt(deathX[0] + dx * 100, deathY[0] + dy * 100);
                    sb.append(String.format("%6.0f", elev));
                }
                System.out.println(sb);
            }
        } else {
            System.out.printf("%n=== SEED %d: SURVIVED ===%n", seed);
        }
    }
}
