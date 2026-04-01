package se.hirt.searobots.engine;

import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.*;

/**
 * Comprehensive turn circle matrix at various depths.
 */
public class TurnCircleMatrix {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    public static void main(String[] args) {
        double[] depths = {0, -60, -200};
        double[] throttles = {0.1, 0.25, 0.4, 0.5, 0.75, 1.0};
        double[] rudders = {0.1, 0.3, 0.5, 0.6, 0.7, 1.0};

        for (double depth : depths) {
            System.out.printf("=== DEPTH: %.0fm ===%n%n", -depth);

            // Get steady-state speeds
            double[] speeds = new double[throttles.length];
            for (int i = 0; i < throttles.length; i++) {
                var sub = makeSub(depth);
                sub.setThrottle(throttles[i]);
                runTicks(sub, 50 * 120);
                speeds[i] = sub.speed();
            }

            // Turn Radius
            System.out.println("--- Turn Radius (meters) ---");
            System.out.printf("%-20s", "Speed \\ Rudder");
            for (double r : rudders) System.out.printf("%8.0f%%", r * 100);
            System.out.println();

            for (int i = 0; i < throttles.length; i++) {
                System.out.printf("%4.1f m/s (%4.1f kn)  ", speeds[i], speeds[i] * 1.944);
                for (double rudder : rudders) {
                    double[] result = measureTurn(throttles[i], rudder, depth);
                    System.out.printf("%8.0f", result[0]);
                }
                System.out.println();
            }

            // Yaw Rate
            System.out.println("\n--- Yaw Rate (deg/s) ---");
            System.out.printf("%-20s", "Speed \\ Rudder");
            for (double r : rudders) System.out.printf("%8.0f%%", r * 100);
            System.out.println();

            for (int i = 0; i < throttles.length; i++) {
                System.out.printf("%4.1f m/s (%4.1f kn)  ", speeds[i], speeds[i] * 1.944);
                for (double rudder : rudders) {
                    double[] result = measureTurn(throttles[i], rudder, depth);
                    System.out.printf("%8.2f", result[1]);
                }
                System.out.println();
            }

            // 360 time
            System.out.println("\n--- Time for 360 deg turn (seconds) ---");
            System.out.printf("%-20s", "Speed \\ Rudder");
            for (double r : rudders) System.out.printf("%8.0f%%", r * 100);
            System.out.println();

            for (int i = 0; i < throttles.length; i++) {
                System.out.printf("%4.1f m/s (%4.1f kn)  ", speeds[i], speeds[i] * 1.944);
                for (double rudder : rudders) {
                    double[] result = measureTurn(throttles[i], rudder, depth);
                    double time360 = Math.abs(result[1]) > 0.01 ? 360.0 / Math.abs(result[1]) : 9999;
                    System.out.printf("%8.0f", time360);
                }
                System.out.println();
            }
            System.out.println();
        }
    }

    static double[] measureTurn(double throttle, double rudder, double depth) {
        var sub = makeSub(depth);
        sub.setThrottle(throttle);
        // Reach steady speed (longer to let first-order filter settle)
        runTicks(sub, 50 * 90);
        double speed = sub.speed();

        // Apply rudder and measure over 90 seconds (longer for filter to reach steady state)
        sub.setRudder(rudder);
        double startHdg = sub.heading();
        runTicks(sub, 50 * 90);

        double hdgChange = sub.heading() - startHdg;
        // Unwrap for multiple revolutions
        // Use total accumulated change instead of mod
        // Since heading wraps, count full revolutions
        double totalHdgChange = 0;
        var sub2 = makeSub(depth);
        sub2.setThrottle(throttle);
        runTicks(sub2, 50 * 90);
        sub2.setRudder(rudder);
        double prevHdg = sub2.heading();
        for (int t = 0; t < 50 * 90; t++) {
            physics.step(sub2, DT, world.terrain(), world.currentField(), world.config().battleArea());
            double curHdg = sub2.heading();
            double delta = curHdg - prevHdg;
            if (delta > Math.PI) delta -= 2 * Math.PI;
            if (delta < -Math.PI) delta += 2 * Math.PI;
            totalHdgChange += delta;
            prevHdg = curHdg;
        }

        double yawDegPerSec = Math.toDegrees(totalHdgChange) / 90.0;
        double yawRadPerSec = totalHdgChange / 90.0;
        double turnRad = Math.abs(yawRadPerSec) > 0.001 ? Math.abs(speed / yawRadPerSec) : 9999;

        return new double[]{turnRad, yawDegPerSec};
    }

    static SubmarineEntity makeSub(double depth) {
        return new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(0, 0, depth), 0, Color.RED, 1000);
    }

    static void runTicks(SubmarineEntity sub, int ticks) {
        for (int t = 0; t < ticks; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
        }
    }
}
