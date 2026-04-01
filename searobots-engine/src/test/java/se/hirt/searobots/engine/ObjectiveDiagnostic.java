package se.hirt.searobots.engine;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.PathPlanner;

public class ObjectiveDiagnostic {
    public static void main(String[] args) {
        long seed = -2259460372447562033L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();
        var objectives = SubmarineCompetition.generateObjectives(seed, world);

        System.out.printf("Seed: %d%n", seed);
        System.out.printf("Spawn 0: (%.0f, %.0f, %.0f) floor=%.0f%n",
                world.spawnPoints().get(0).x(), world.spawnPoints().get(0).y(),
                world.spawnPoints().get(0).z(), terrain.elevationAt(
                        world.spawnPoints().get(0).x(), world.spawnPoints().get(0).y()));

        System.out.printf("Objective 1: (%.0f, %.0f) floor=%.0f%n",
                objectives.x1(), objectives.y1(),
                terrain.elevationAt(objectives.x1(), objectives.y1()));
        System.out.printf("Objective 2: (%.0f, %.0f) floor=%.0f%n",
                objectives.x2(), objectives.y2(),
                terrain.elevationAt(objectives.x2(), objectives.y2()));

        // Check if objectives are in safe water
        var planner = new PathPlanner(terrain, -90, 200, 75);
        System.out.printf("Obj1 safe: %s cost: %.2f%n",
                planner.isSafe(objectives.x1(), objectives.y1()),
                planner.costAt(objectives.x1(), objectives.y1()));
        System.out.printf("Obj2 safe: %s cost: %.2f%n",
                planner.isSafe(objectives.x2(), objectives.y2()),
                planner.costAt(objectives.x2(), objectives.y2()));

        // Check path exists from spawn to each objective
        var path1 = planner.findPath(
                world.spawnPoints().get(0).x(), world.spawnPoints().get(0).y(),
                objectives.x1(), objectives.y1(), -200);
        var path2 = planner.findPath(
                objectives.x1(), objectives.y1(),
                objectives.x2(), objectives.y2(), -200);

        System.out.printf("Path to Obj1: %s (%d waypoints)%n",
                path1.isEmpty() ? "NO PATH" : "OK", path1.size());
        System.out.printf("Path Obj1->Obj2: %s (%d waypoints)%n",
                path2.isEmpty() ? "NO PATH" : "OK", path2.size());

        // Check distances
        double d1 = Math.hypot(objectives.x1() - world.spawnPoints().get(0).x(),
                objectives.y1() - world.spawnPoints().get(0).y());
        double d2 = Math.hypot(objectives.x2() - objectives.x1(),
                objectives.y2() - objectives.y1());
        System.out.printf("Distance spawn->Obj1: %.0fm%n", d1);
        System.out.printf("Distance Obj1->Obj2: %.0fm%n", d2);
        System.out.printf("Total distance: %.0fm (at 8m/s = %.0fs)%n", d1 + d2, (d1 + d2) / 8);

        // Check terrain along direct path
        System.out.println("\nTerrain profile spawn->Obj1:");
        for (int i = 0; i <= 10; i++) {
            double t = i / 10.0;
            double x = world.spawnPoints().get(0).x() + t * (objectives.x1() - world.spawnPoints().get(0).x());
            double y = world.spawnPoints().get(0).y() + t * (objectives.y1() - world.spawnPoints().get(0).y());
            double floor = terrain.elevationAt(x, y);
            System.out.printf("  %.0f%%: (%.0f, %.0f) floor=%.0f%n", t * 100, x, y, floor);
        }
    }
}
