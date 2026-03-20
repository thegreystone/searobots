package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.List;
import java.util.ArrayList;

class SeedAnalysis {
    @Test
    void analyzeSeed() {
        long seed = -5810025624933101181L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();

        var spawn = world.spawnPoints().get(0);
        double sx = spawn.x(), sy = spawn.y();
        double heading = 2.4155;
        System.out.printf("Spawn: (%.1f, %.1f, %.1f)%n", sx, sy, spawn.z());

        var context = new MatchContext(config, terrain, world.thermalLayers(), world.currentField());
        var autopilot = new SubmarineAutopilot(context);
        var sub = new DefaultAttackSub();
        sub.onMatchStart(context);
        var strategicWps = sub.generatePatrolWaypoints(sx, sy, heading, config.battleArea());

        autopilot.setWaypoints(strategicWps, sx, sy, spawn.z(), heading, 0);

        System.out.println("\n=== A* Nav Waypoints ===");
        var navWps = autopilot.navWaypoints();
        for (int i = 0; i < navWps.size(); i++) {
            var wp = navWps.get(i);
            double dist = Math.sqrt(Math.pow(wp.x() - sx, 2) + Math.pow(wp.y() - sy, 2));
            double bearing = Math.toDegrees(Math.atan2(wp.x() - sx, wp.y() - sy));
            if (bearing < 0) bearing += 360;
            double floor = terrain.elevationAt(wp.x(), wp.y());
            boolean active = i == autopilot.currentNavIndex();
            System.out.printf("  NAV[%d]%s: (%.0f, %.0f, %.0f) dist=%.0f bearing=%.1f floor=%.1f%n",
                    i, active ? " *ACTIVE*" : "", wp.x(), wp.y(), wp.z(), dist, bearing, floor);
        }
    }

    @Test
    void runSimAndTrace() {
        long seed = -5810025624933101181L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new DefaultAttackSub());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        var log = new ArrayList<String>();

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.isEmpty()) return;
                var s = submarines.get(0);
                var pos = s.pose().position();
                double floor = world.terrain().elevationAt(pos.x(), pos.y());
                double gap = pos.z() - floor;

                // Log every 100 ticks, more detail around 8000-12000
                boolean detailed = tick >= 7000 && tick <= 13000;
                if (tick % (detailed ? 50 : 500) == 0) {
                    // Find active waypoint
                    int activeIdx = -1;
                    double activeX = 0, activeY = 0, activeZ = 0;
                    if (s.waypoints() != null) {
                        for (int i = 0; i < s.waypoints().size(); i++) {
                            if (s.waypoints().get(i).active()) {
                                activeIdx = i;
                                activeX = s.waypoints().get(i).x();
                                activeY = s.waypoints().get(i).y();
                                activeZ = s.waypoints().get(i).z();
                                break;
                            }
                        }
                    }
                    double wpDist = Math.sqrt(Math.pow(activeX - pos.x(), 2) + Math.pow(activeY - pos.y(), 2));

                    log.add(String.format(
                        "t=%5d pos=(%.0f,%.0f,%.0f) floor=%.0f gap=%.0f hdg=%.1f spd=%.1f " +
                        "thr=%.2f rud=%.2f hp=%d status=%s " +
                        "wp[%d]=(%.0f,%.0f,%.0f) wpDist=%.0f",
                        tick, pos.x(), pos.y(), pos.z(), floor, gap,
                        Math.toDegrees(s.pose().heading()), s.speed(),
                        s.throttle(), s.rudder(), s.hp(), s.status(),
                        activeIdx, activeX, activeY, activeZ, wpDist));
                }

                if (s.hp() <= 0 || tick >= 15000) {
                    sim.stop();
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        System.out.println("\n=== Simulation Trace (sub 0) ===");
        for (var line : log) {
            System.out.println(line);
        }
    }
}
