package se.hirt.searobots.engine.ships.claude;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

class SingleSeedTrace {

    @Test void traceSeed55555() { trace(55555, 50000, 70000); }
    @Test void traceSeed8888()  { trace(8888,  35000, 41000); }
    @Test void traceSeed7777()  { trace(7777,  5000, 12000); }

    private void trace(long seed, long startTick, long endTick) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);
        var planner = new PathPlanner(world.terrain(), -80, 200, 75);

        List<SubmarineController> controllers = List.of(new ClaudeAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        System.out.printf("%n=== SEED %d TRACE [%d-%d] ===%n", seed, startTick, endTick);

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.isEmpty()) return;
                var s0 = submarines.get(0);
                if (tick < startTick || (tick > endTick && s0.hp() > 0)) return;

                var pos = s0.pose().position();
                double floor = world.terrain().elevationAt(pos.x(), pos.y());
                boolean safe = planner.isSafe(pos.x(), pos.y());

                if (tick % 500 == 0 || !safe || s0.hp() <= 0) {
                    var wps = s0.waypoints();
                    String wpStr = wps.isEmpty() ? "NO_WP" : "";
                    for (var wp : wps) {
                        if (wp.active()) {
                            wpStr = String.format("-> [%5.0f,%5.0f]", wp.x(), wp.y());
                            break;
                        }
                    }
                    if (wpStr.isEmpty() && !wps.isEmpty()) {
                        var wp = wps.getFirst();
                        wpStr = String.format("1st[%5.0f,%5.0f]", wp.x(), wp.y());
                    }
                    System.out.printf("  t=%-6d [%5.0f,%5.0f,%4.0f] fl=%4.0f %s spd=%5.1f hdg=%5.1f hp=%4d %s %s%n",
                            tick, pos.x(), pos.y(), pos.z(), floor,
                            safe ? "OK" : "!!",
                            s0.speed(), Math.toDegrees(s0.pose().heading()),
                            s0.hp(), s0.status(), wpStr);
                    if (s0.hp() <= 0) return;
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(30_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}
    }
}
