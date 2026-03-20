package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.awt.Color;

/**
 * Measures steady-state turn radius at 1 m/s increments from 1 to 15 m/s,
 * at optimal rudder (~0.35) and at full rudder (1.0).
 * Results feed into the autopilot's turn rate table.
 */
class TurnRadiusTableTest {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    @Test
    void measureTurnRadiusTable() {
        double[] rudders = {0.35, 1.0};
        System.out.println("=== Turn Radius Table (depth -200m) ===");
        System.out.printf("%-8s %-12s", "Speed", "Throttle");
        for (double r : rudders) System.out.printf("  Rud=%.2f (radius/yaw)", r);
        System.out.println();
        System.out.println("-".repeat(80));

        // For each target speed, find the throttle that achieves it,
        // then measure the turn radius at that speed.
        for (int targetSpeed = 1; targetSpeed <= 15; targetSpeed++) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.printf("%4d m/s  (unreachable)%n", targetSpeed);
                continue;
            }

            System.out.printf("%4d m/s  thr=%-8.3f", targetSpeed, throttle);
            for (double rudder : rudders) {
                double[] result = measureTurn(throttle, rudder, -200);
                System.out.printf("  %6.0fm  %5.2f°/s       ", result[0], result[1]);
            }
            System.out.println();
        }

        // Also print a Java array for direct encoding
        System.out.println("\n=== Java array (optimal rudder 0.35) ===");
        System.out.println("// turnRadiusAtSpeed[speed_ms] = radius_m");
        System.out.print("static final double[] TURN_RADIUS = {0");
        for (int targetSpeed = 1; targetSpeed <= 15; targetSpeed++) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.print(", 9999");
            } else {
                double[] result = measureTurn(throttle, 0.35, -200);
                System.out.printf(", %.0f", result[0]);
            }
        }
        System.out.println("};");

        System.out.println("\n=== Java array (full rudder 1.0) ===");
        System.out.print("static final double[] TURN_RADIUS_FULL = {0");
        for (int targetSpeed = 1; targetSpeed <= 15; targetSpeed++) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.print(", 9999");
            } else {
                double[] result = measureTurn(throttle, 1.0, -200);
                System.out.printf(", %.0f", result[0]);
            }
        }
        System.out.println("};");
    }

    double findThrottleForSpeed(double targetSpeed, double depth) {
        // Binary search for throttle that produces the target steady-state speed
        double lo = 0, hi = 1.0;
        for (int iter = 0; iter < 30; iter++) {
            double mid = (lo + hi) / 2;
            double speed = steadyStateSpeed(mid, depth);
            if (speed < targetSpeed) lo = mid;
            else hi = mid;
        }
        double result = (lo + hi) / 2;
        double achieved = steadyStateSpeed(result, depth);
        return Math.abs(achieved - targetSpeed) < 0.5 ? result : -1;
    }

    double steadyStateSpeed(double throttle, double depth) {
        var sub = makeSub(depth);
        sub.setThrottle(throttle);
        runTicks(sub, 50 * 120);
        return sub.speed();
    }

    double[] measureTurn(double throttle, double rudder, double depth) {
        var sub = makeSub(depth);
        sub.setThrottle(throttle);
        runTicks(sub, 50 * 90); // reach steady speed

        double speed = sub.speed();
        sub.setRudder(rudder);
        double prevHdg = sub.heading();
        double totalHdgChange = 0;
        int turnTicks = 50 * 90;
        for (int t = 0; t < turnTicks; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
            double curHdg = sub.heading();
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

    SubmarineEntity makeSub(double depth) {
        return new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(0, 0, depth), 0, Color.RED, 1000);
    }

    void runTicks(SubmarineEntity sub, int ticks) {
        for (int t = 0; t < ticks; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
        }
    }

    @Test
    void stallCurve() {
        double[] speeds = {3, 5, 7, 10, 15};
        double[] rudders = {0.1, 0.2, 0.3, 0.35, 0.4, 0.5, 0.556, 0.6, 0.7, 0.8, 0.9, 1.0};

        var cfg = VehicleConfig.submarine();
        System.out.printf("Stall angle: %.1f deg (rudder setting %.2f)%n%n",
                Math.toDegrees(cfg.stallAngle()), cfg.stallAngle() / (Math.PI / 4));

        System.out.printf("%-8s", "Rudder");
        for (double r : rudders) System.out.printf("%7.0f%%", r * 100);
        System.out.printf("%n%-8s", "Angle");
        for (double r : rudders) System.out.printf("%7.1f\u00b0", r * 45);
        System.out.println("\n" + "-".repeat(100));

        for (double targetSpeed : speeds) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            System.out.printf("%2.0f m/s  ", targetSpeed);
            double bestRadius = Double.MAX_VALUE;
            double bestRudder = 0;
            for (double rudder : rudders) {
                double[] r = measureTurn(throttle, rudder, -200);
                System.out.printf("%6.0fm", r[0]);
                if (r[0] < bestRadius) { bestRadius = r[0]; bestRudder = rudder; }
            }
            System.out.printf("  best=%.0f%% (%.0fm)%n", bestRudder * 100, bestRadius);
        }
    }
}
