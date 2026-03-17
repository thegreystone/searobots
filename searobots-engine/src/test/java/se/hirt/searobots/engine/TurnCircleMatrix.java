package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;

import java.awt.Color;

/**
 * Comprehensive turn circle matrix: turn radius at every combination
 * of speed and rudder setting.
 */
public class TurnCircleMatrix {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    public static void main(String[] args) {
        double[] throttles = {0.1, 0.2, 0.25, 0.4, 0.5, 0.75, 1.0};
        double[] rudders = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

        // First get steady-state speeds
        double[] speeds = new double[throttles.length];
        for (int i = 0; i < throttles.length; i++) {
            var sub = makeSub();
            sub.setThrottle(throttles[i]);
            for (int t = 0; t < 50 * 120; t++) step(sub);
            speeds[i] = sub.speed();
        }

        // === Turn Radius Matrix ===
        System.out.println("=== TURN RADIUS (meters) ===");
        System.out.printf("%-18s", "Speed \\ Rudder");
        for (double r : rudders) System.out.printf("%7.0f%%", r * 100);
        System.out.println();

        for (int i = 0; i < throttles.length; i++) {
            System.out.printf("%.1f m/s (%4.1f kn) ", speeds[i], speeds[i] * 1.944);
            for (double rudder : rudders) {
                double[] result = measureTurn(throttles[i], rudder);
                System.out.printf("%7.0f", result[0]); // turn radius
            }
            System.out.println();
        }

        // === Yaw Rate Matrix ===
        System.out.println("\n=== YAW RATE (deg/s) ===");
        System.out.printf("%-18s", "Speed \\ Rudder");
        for (double r : rudders) System.out.printf("%7.0f%%", r * 100);
        System.out.println();

        for (int i = 0; i < throttles.length; i++) {
            System.out.printf("%.1f m/s (%4.1f kn) ", speeds[i], speeds[i] * 1.944);
            for (double rudder : rudders) {
                double[] result = measureTurn(throttles[i], rudder);
                System.out.printf("%7.2f", result[1]); // yaw rate
            }
            System.out.println();
        }

        // === Time for 360 degree turn ===
        System.out.println("\n=== TIME FOR 360 DEG TURN (seconds) ===");
        System.out.printf("%-18s", "Speed \\ Rudder");
        for (double r : rudders) System.out.printf("%7.0f%%", r * 100);
        System.out.println();

        for (int i = 0; i < throttles.length; i++) {
            System.out.printf("%.1f m/s (%4.1f kn) ", speeds[i], speeds[i] * 1.944);
            for (double rudder : rudders) {
                double[] result = measureTurn(throttles[i], rudder);
                double time360 = Math.abs(result[1]) > 0.01 ? 360.0 / Math.abs(result[1]) : 9999;
                System.out.printf("%7.0f", time360);
            }
            System.out.println();
        }
    }

    static double[] measureTurn(double throttle, double rudder) {
        var sub = makeSub();
        sub.setThrottle(throttle);
        // Reach steady speed
        for (int t = 0; t < 50 * 60; t++) step(sub);
        double speed = sub.speed();

        // Apply rudder and measure over 30 seconds
        sub.setRudder(rudder);
        double startHdg = sub.heading();
        for (int t = 0; t < 50 * 30; t++) step(sub);

        double hdgChange = sub.heading() - startHdg;
        while (hdgChange > Math.PI) hdgChange -= 2 * Math.PI;
        while (hdgChange < -Math.PI) hdgChange += 2 * Math.PI;

        double yawDegPerSec = Math.toDegrees(hdgChange) / 30.0;
        double yawRadPerSec = hdgChange / 30.0;
        double turnRad = Math.abs(yawRadPerSec) > 0.001 ? Math.abs(speed / yawRadPerSec) : 9999;

        return new double[]{turnRad, yawDegPerSec};
    }

    static SubmarineEntity makeSub() {
        return new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(0, 0, -100), 0, Color.RED, 1000);
    }

    static void step(SubmarineEntity sub) {
        physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
    }
}
