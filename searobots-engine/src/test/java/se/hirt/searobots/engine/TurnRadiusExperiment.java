package se.hirt.searobots.engine;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.awt.Color;
import java.util.*;

class TurnRadiusExperiment {

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
    private static final double DT = 1.0 / CONFIG.tickRateHz();

    private static TerrainMap deepFlat() {
        int size = 201; double cellSize = 100; double origin = -(size/2)*cellSize;
        double[] data = new double[size*size];
        Arrays.fill(data, -500);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    @Test
    void measureTurnRadii() {
        var terrain = deepFlat();
        var noCurrent = new CurrentField(List.of());
        var physics = new SubmarinePhysics();

        System.out.printf("%-8s %-8s %-10s %-10s %-10s%n",
                "Speed", "Rudder", "YawRate", "TurnRad", "180degTime");
        System.out.println("-".repeat(50));

        for (double speed : new double[]{3, 5, 7, 10, 13}) {
            for (double rudder : new double[]{0.35, 0.5, 0.7, 1.0}) {
                var sub = new SubmarineEntity(VehicleConfig.submarine(), 0,
                        new DefaultAttackSub(), new Vec3(0, 0, -200), 0, Color.GREEN, 1000);
                sub.setSpeed(speed);
                // Let thrust lag settle to maintain speed
                sub.setThrottle(speed / 15.0);  // approximate throttle for speed
                sub.setActualThrottle(speed / 15.0);
                sub.setRudder(rudder);

                // Warm up for 10 seconds to let sway build up
                for (int i = 0; i < 500; i++) {
                    sub.setRudder(rudder);
                    sub.setThrottle(speed / 15.0);
                    physics.step(sub, DT, terrain, noCurrent, CONFIG.battleArea());
                }
                double h0 = sub.heading();
                double s0 = sub.speed();
                // Measure over 2 seconds
                for (int i = 0; i < 100; i++) {
                    sub.setRudder(rudder);
                    sub.setThrottle(speed / 15.0);
                    physics.step(sub, DT, terrain, noCurrent, CONFIG.battleArea());
                }
                double measuredSeconds = 2.0;
                double h1 = sub.heading();
                double yawRate = (h1 - h0) / measuredSeconds; // radians per second
                if (yawRate < 0) yawRate += 2 * Math.PI; // handle wrap
                if (yawRate > Math.PI) yawRate -= 2 * Math.PI;

                double turnRadius = yawRate != 0 ? Math.abs(s0 / yawRate) : Double.POSITIVE_INFINITY;
                double time180 = yawRate != 0 ? Math.PI / Math.abs(yawRate) : Double.POSITIVE_INFINITY;

                System.out.printf("%-8.1f %-8.2f %-10.4f %-10.0f %-10.1f%n",
                        s0, rudder, yawRate, turnRadius, time180);
            }
        }
    }
}
