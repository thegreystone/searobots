package se.hirt.searobots.engine;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
class CheckGrid {
    @Test void check() {
        var world = new WorldGenerator().generate(MatchConfig.withDefaults(55555));
        var planner = new PathPlanner(world.terrain(), -80, 200, 75);
        // Check death location and surroundings
        double x = 873, y = 1063;
        System.out.printf("Death [%.0f,%.0f]: safe=%s cost=%.1f floor=%.0f%n",
                x, y, planner.isSafe(x, y), planner.costAt(x, y), world.terrain().elevationAt(x, y));
        for (int d = 0; d < 360; d += 45) {
            double brg = Math.toRadians(d);
            double px = x + Math.sin(brg) * 500;
            double py = y + Math.cos(brg) * 500;
            System.out.printf("  %3d deg +500m: safe=%s cost=%.1f floor=%.0f%n",
                    d, planner.isSafe(px, py), planner.costAt(px, py), world.terrain().elevationAt(px, py));
        }
    }
}
