package se.hirt.searobots.engine;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

class DroneExitInvestigation {

    @Test
    void investigateDroneExit() {
        long seed = 4291328891004373985L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        var listener = new SimulationListener() {
            boolean droneForfeited = false;

            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (submarines.size() < 2) return;
                var drone = submarines.get(1); // sub drone is index 1

                // Log every 500 ticks when near boundary
                var pos = drone.pose().position();
                double distToBoundary = config.battleArea().distanceToBoundary(pos.x(), pos.y());

                if (tick % 2500 == 0 || (distToBoundary < 500 && tick % 50 == 0)) {
                    System.out.printf("  t=%-6d drone pos=[%7.0f,%7.0f,%6.0f] spd=%5.1f hdg=%5.1f bndry=%5.0f hp=%d %s %s%n",
                            tick, pos.x(), pos.y(), pos.z(), drone.speed(),
                            Math.toDegrees(drone.pose().heading()),
                            distToBoundary, drone.hp(),
                            drone.forfeited() ? "FORFEIT" : "",
                            drone.status() != null ? drone.status() : "");
                }

                if (drone.forfeited() && !droneForfeited) {
                    droneForfeited = true;
                    System.out.printf("%n=== DRONE FORFEITED at t=%d (%.0fs) ===%n", tick, tick / 50.0);
                    System.out.printf("  pos=[%.0f, %.0f, %.0f] speed=%.1f heading=%.1f%n",
                            pos.x(), pos.y(), pos.z(), drone.speed(),
                            Math.toDegrees(drone.pose().heading()));
                    System.out.printf("  distToBoundary=%.0f%n", distToBoundary);
                    System.out.printf("  battleArea: %s%n", config.battleArea());
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}
    }
}
