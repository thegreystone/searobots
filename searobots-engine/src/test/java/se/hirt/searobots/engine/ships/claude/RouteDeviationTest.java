package se.hirt.searobots.engine.ships.claude;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

/**
 * Diagnostic test: tracks how far the sub deviates from its planned
 * waypoints and what the floor looks like along the actual path.
 */
class RouteDeviationTest {

    @Test
    void trackDeviationForDyingSeeds() {
        for (long seed : new long[]{7777, 55555, 8888}) {
            trackSeed(seed);
        }
    }

    private void trackSeed(long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new ClaudeAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        System.out.printf("%n=== SEED %d ROUTE DEVIATION ===%n", seed);

        var listener = new SimulationListener() {
            boolean dead = false;
            double maxDeviation = 0;
            double maxFloorOnPath = -9999;
            long maxDeviationTick = 0;
            String maxDeviationStatus = "";

            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (submarines.isEmpty() || dead) return;
                var s0 = submarines.get(0);
                if (s0.hp() <= 0) {
                    dead = true;
                    var pos = s0.pose().position();
                    System.out.printf("  DEAD at t=%d pos=[%.0f,%.0f,%.0f] %s%n",
                            tick, pos.x(), pos.y(), pos.z(), s0.status());
                    System.out.printf("  Max deviation: %.0fm at t=%d (%s)%n",
                            maxDeviation, maxDeviationTick, maxDeviationStatus);
                    System.out.printf("  Worst floor on actual path: %.0fm%n", maxFloorOnPath);
                    return;
                }

                var pos = s0.pose().position();
                var waypoints = s0.waypoints();
                double floor = world.terrain().elevationAt(pos.x(), pos.y());
                if (floor > maxFloorOnPath) maxFloorOnPath = floor;

                // Find distance to nearest waypoint (the active one)
                if (!waypoints.isEmpty()) {
                    // Find the active waypoint (marked active)
                    double minDist = Double.MAX_VALUE;
                    Waypoint activeWp = null;
                    for (var wp : waypoints) {
                        if (wp.active()) {
                            activeWp = wp;
                            break;
                        }
                    }
                    if (activeWp == null) activeWp = waypoints.getFirst();

                    // Distance to line segment between previous and active waypoint
                    double dx = activeWp.x() - pos.x();
                    double dy = activeWp.y() - pos.y();
                    double dist = Math.sqrt(dx * dx + dy * dy);

                    // Also compute cross-track error (perpendicular distance to route)
                    // For simplicity, just use distance to active waypoint
                    if (dist > maxDeviation) {
                        maxDeviation = dist;
                        maxDeviationTick = tick;
                        maxDeviationStatus = s0.status() != null ? s0.status() : "";
                    }
                }

                // Log when floor is dangerously shallow
                if (tick % 250 == 0 && floor > -80) {
                    var wpInfo = waypoints.isEmpty() ? "NO WAYPOINTS" :
                            String.format("wp=[%.0f,%.0f] #%d/%d",
                                    waypoints.getFirst().x(), waypoints.getFirst().y(),
                                    1, waypoints.size());
                    boolean wpSafe = true;
                    if (!waypoints.isEmpty()) {
                        for (var wp : waypoints) {
                            double wpFloor = world.terrain().elevationAt(wp.x(), wp.y());
                            if (wpFloor > -80) {
                                wpSafe = false;
                                wpInfo += String.format(" UNSAFE(floor=%.0f@[%.0f,%.0f])",
                                        wpFloor, wp.x(), wp.y());
                                break;
                            }
                        }
                    }
                    System.out.printf("  t=%-6d pos=[%5.0f,%5.0f,%4.0f] floor=%4.0f spd=%4.1f %s | %s%n",
                            tick, pos.x(), pos.y(), pos.z(), floor, s0.speed(),
                            s0.status(), wpInfo);
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
