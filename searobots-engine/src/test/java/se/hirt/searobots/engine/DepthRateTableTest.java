package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.*;

/**
 * Measures steady-state depth change rates at 1 m/s increments from 1 to 15 m/s,
 * using stern planes, ballast, and combined. Results feed into the autopilot's
 * depth control tables.
 *
 * Conventions: z is negative underwater. Positive depth rate = ascending (toward surface).
 * Stern planes: +1 = full rise (nose up), -1 = full dive (nose down).
 * Ballast: 0.0 = full flood (heavy, sink), 0.5 = neutral, 1.0 = full blow (light, rise).
 */
class DepthRateTableTest {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    // ── Stern planes depth rate ──

    @Test
    void measureDepthRateWithPlanes() {
        double[] planeSettings = {-1.0, -0.55, -0.35, 0.35, 0.55, 1.0};
        System.out.println("=== Depth Rate with Stern Planes (start depth -200m, ballast neutral) ===");
        System.out.printf("%-8s %-10s", "Speed", "Throttle");
        for (double p : planeSettings) System.out.printf("  Pln=%+5.2f         ", p);
        System.out.println();
        System.out.printf("%-8s %-10s", "", "");
        for (double ignored : planeSettings) System.out.printf("  dz(m/s) pitch(deg) ");
        System.out.println();
        System.out.println("-".repeat(30 + planeSettings.length * 21));

        for (int targetSpeed = 1; targetSpeed <= 15; targetSpeed++) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.printf("%4d m/s  (unreachable)%n", targetSpeed);
                continue;
            }

            System.out.printf("%4d m/s  thr=%-6.3f", targetSpeed, throttle);
            for (double planes : planeSettings) {
                double[] result = measureDepthRate(throttle, planes, 0.5, -200);
                System.out.printf("  %+6.2f  %+6.1f\u00b0    ", result[0], result[1]);
            }
            System.out.println();
        }

        // Java arrays for autopilot
        System.out.println("\n=== Java arrays: depth rate (m/s) by speed, planes at -0.55 (optimal dive) ===");
        System.out.print("static final double[] DIVE_RATE = {0");
        for (int s = 1; s <= 15; s++) {
            double throttle = findThrottleForSpeed(s, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.print(", 0");
            } else {
                double[] r = measureDepthRate(throttle, -0.55, 0.5, -200);
                System.out.printf(", %.2f", r[0]);
            }
        }
        System.out.println("};");

        System.out.print("static final double[] RISE_RATE = {0");
        for (int s = 1; s <= 15; s++) {
            double throttle = findThrottleForSpeed(s, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.print(", 0");
            } else {
                double[] r = measureDepthRate(throttle, 0.55, 0.5, -200);
                System.out.printf(", %.2f", r[0]);
            }
        }
        System.out.println("};");

        System.out.println("\n=== Java arrays: depth rate (m/s) by speed, full planes ===");
        System.out.print("static final double[] DIVE_RATE_FULL = {0");
        for (int s = 1; s <= 15; s++) {
            double throttle = findThrottleForSpeed(s, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.print(", 0");
            } else {
                double[] r = measureDepthRate(throttle, -1.0, 0.5, -200);
                System.out.printf(", %.2f", r[0]);
            }
        }
        System.out.println("};");

        System.out.print("static final double[] RISE_RATE_FULL = {0");
        for (int s = 1; s <= 15; s++) {
            double throttle = findThrottleForSpeed(s, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.print(", 0");
            } else {
                double[] r = measureDepthRate(throttle, 1.0, 0.5, -200);
                System.out.printf(", %.2f", r[0]);
            }
        }
        System.out.println("};");
    }

    // ── Ballast-only depth rate ──

    @Test
    void measureDepthRateWithBallast() {
        double[] ballastSettings = {0.0, 0.25, 0.5, 0.75, 1.0};
        int[] speeds = {0, 3, 5, 7, 10, 15};

        System.out.println("=== Depth Rate with Ballast Only (start depth -200m, planes neutral) ===");
        System.out.printf("%-8s %-10s", "Speed", "Throttle");
        for (double b : ballastSettings) System.out.printf("  Bal=%.2f         ", b);
        System.out.println();
        System.out.printf("%-8s %-10s", "", "");
        for (double ignored : ballastSettings) System.out.printf("  dz(m/s) vSpd(m/s) ");
        System.out.println();
        System.out.println("-".repeat(30 + ballastSettings.length * 21));

        for (int targetSpeed : speeds) {
            double throttle = targetSpeed == 0 ? 0 : findThrottleForSpeed(targetSpeed, -200);
            if (targetSpeed > 0 && (throttle < 0 || throttle > 1.0)) {
                System.out.printf("%4d m/s  (unreachable)%n", targetSpeed);
                continue;
            }

            System.out.printf("%4d m/s  thr=%-6.3f", targetSpeed, throttle);
            for (double ballast : ballastSettings) {
                double[] result = measureBallastRate(throttle, ballast, -200);
                System.out.printf("  %+6.2f  %+6.2f     ", result[0], result[1]);
            }
            System.out.println();
        }
    }

    // ── Combined planes + ballast ──

    @Test
    void measureDepthRateCombined() {
        System.out.println("=== Combined Depth Rate: Planes + Ballast (start depth -200m) ===");
        System.out.println("Scenario: maximum dive and maximum rise at each speed");
        System.out.printf("%-8s %-10s  %-28s  %-28s%n",
                "Speed", "Throttle", "Max Dive (pln=-0.55,bal=0.0)", "Max Rise (pln=+0.55,bal=1.0)");
        System.out.printf("%-8s %-10s  %-10s %-8s %-8s  %-10s %-8s %-8s%n",
                "", "", "dz(m/s)", "pitch", "vSpd", "dz(m/s)", "pitch", "vSpd");
        System.out.println("-".repeat(90));

        for (int targetSpeed = 1; targetSpeed <= 15; targetSpeed++) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            if (throttle < 0 || throttle > 1.0) {
                System.out.printf("%4d m/s  (unreachable)%n", targetSpeed);
                continue;
            }

            double[] dive = measureDepthRate(throttle, -0.55, 0.0, -200);
            double[] rise = measureDepthRate(throttle, 0.55, 1.0, -200);
            System.out.printf("%4d m/s  thr=%-6.3f  %+7.2f   %+6.1f\u00b0 %+6.2f   %+7.2f   %+6.1f\u00b0 %+6.2f%n",
                    targetSpeed, throttle,
                    dive[0], dive[1], dive[2],
                    rise[0], rise[1], rise[2]);
        }
    }

    // ── Emergency ballast blow (time to surface) ──

    @Test
    void measureEmergencyBlow() {
        double[] startDepths = {-50, -100, -200, -300, -500};
        int[] speeds = {0, 5, 10};

        System.out.println("=== Emergency Ballast Blow: Time to Surface ===");
        System.out.println("Full blow (ballast=1.0), planes=+0.55 (rise)");
        System.out.printf("%-12s", "StartDepth");
        for (int s : speeds) System.out.printf("  %d m/s (sec)    ", s);
        System.out.println();
        System.out.println("-".repeat(60));

        for (double startDepth : startDepths) {
            System.out.printf("%6.0f m    ", startDepth);
            for (int targetSpeed : speeds) {
                double throttle = targetSpeed == 0 ? 0 : findThrottleForSpeed(targetSpeed, startDepth);
                double seconds = measureTimeToSurface(throttle, startDepth);
                System.out.printf("  %6.1f           ", seconds);
            }
            System.out.println();
        }
    }

    // ── Plane angle vs depth rate (stall curve equivalent for pitch) ──

    @Test
    void planeStallCurve() {
        double[] speeds = {3, 5, 7, 10, 15};
        double[] planeSettings = {0.1, 0.2, 0.3, 0.35, 0.4, 0.5, 0.556, 0.6, 0.7, 0.8, 0.9, 1.0};

        var cfg = VehicleConfig.submarine();
        System.out.printf("Stall angle: %.1f\u00b0 (plane setting %.2f)%n%n",
                Math.toDegrees(cfg.stallAngle()), cfg.stallAngle() / (Math.PI / 4));

        System.out.printf("%-8s", "Planes");
        for (double p : planeSettings) System.out.printf("%7.0f%%", p * 100);
        System.out.printf("%n%-8s", "Angle");
        for (double p : planeSettings) System.out.printf("%7.1f\u00b0", p * 45);
        System.out.println("\n" + "-".repeat(100));

        System.out.println("Dive rate (m/s) with negative planes:");
        for (double targetSpeed : speeds) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            System.out.printf("%2.0f m/s  ", targetSpeed);
            double bestRate = 0;
            double bestPlane = 0;
            for (double plane : planeSettings) {
                double[] r = measureDepthRate(throttle, -plane, 0.5, -200);
                System.out.printf("%+6.2f ", r[0]);
                if (r[0] < bestRate) { bestRate = r[0]; bestPlane = plane; }
            }
            System.out.printf("  best=%.0f%% (%.2f m/s)%n", bestPlane * 100, bestRate);
        }

        System.out.println("\nRise rate (m/s) with positive planes:");
        for (double targetSpeed : speeds) {
            double throttle = findThrottleForSpeed(targetSpeed, -200);
            System.out.printf("%2.0f m/s  ", targetSpeed);
            double bestRate = 0;
            double bestPlane = 0;
            for (double plane : planeSettings) {
                double[] r = measureDepthRate(throttle, plane, 0.5, -200);
                System.out.printf("%+6.2f ", r[0]);
                if (r[0] > bestRate) { bestRate = r[0]; bestPlane = plane; }
            }
            System.out.printf("  best=%.0f%% (%.2f m/s)%n", bestPlane * 100, bestRate);
        }
    }

    // ── Helpers ──

    /**
     * Measures steady-state depth change rate with given planes and ballast.
     * Starts deep enough that the sub won't hit surface or floor during measurement.
     * Returns {depthRate_m_per_s, steadyPitch_deg, verticalSpeed_m_per_s}.
     */
    double[] measureDepthRate(double throttle, double planes, double ballast, double nominalDepth) {
        // For ascending tests, start deep so we don't hit the surface during stabilization.
        // For descending tests, nominalDepth (-200m) is fine.
        // Deep flat ocean floor is at -1000m (clearance at -988m), so -800m is safe for both.
        boolean ascending = planes > 0 || ballast > 0.5;
        double startDepth = ascending ? -800 : nominalDepth;

        var sub = makeSub(startDepth);
        sub.setThrottle(throttle);
        sub.setBallast(0.5); // neutral ballast during speed-up
        runTicks(sub, 50 * 120); // reach steady speed

        // Apply planes and ballast, let pitch and vertical speed stabilize
        sub.setSternPlanes(planes);
        sub.setBallast(ballast);
        runTicks(sub, 50 * 60); // 60s to reach steady state

        // Measure over 30 seconds
        double zStart = sub.z();
        runTicks(sub, 50 * 30);
        double zEnd = sub.z();

        double depthRate = (zEnd - zStart) / 30.0;
        double pitch = Math.toDegrees(sub.pitch());
        double vSpeed = sub.verticalSpeed();
        return new double[]{depthRate, pitch, vSpeed};
    }

    /**
     * Measures ballast-only depth rate (planes neutral).
     * Returns {depthRate_m_per_s, verticalSpeed_m_per_s}.
     */
    double[] measureBallastRate(double throttle, double ballast, double startDepth) {
        boolean ascending = ballast > 0.5;
        double depth = ascending ? -800 : startDepth;

        var sub = makeSub(depth);
        sub.setThrottle(throttle);
        sub.setBallast(0.5);
        runTicks(sub, 50 * 120); // reach steady speed

        sub.setBallast(ballast);
        runTicks(sub, 50 * 60); // let ballast settle and vertical speed stabilize

        double zStart = sub.z();
        runTicks(sub, 50 * 30);
        double zEnd = sub.z();

        double depthRate = (zEnd - zStart) / 30.0;
        return new double[]{depthRate, sub.verticalSpeed()};
    }

    /**
     * Measures time from startDepth to surface (z >= -1) with full blow + rise planes.
     */
    double measureTimeToSurface(double throttle, double startDepth) {
        var sub = makeSub(startDepth);
        sub.setThrottle(throttle);
        sub.setBallast(0.5);
        runTicks(sub, 50 * 90); // reach steady speed

        sub.setBallast(1.0);       // full blow
        sub.setSternPlanes(0.55);  // optimal rise

        int maxTicks = 50 * 600; // 10 minute timeout
        for (int t = 0; t < maxTicks; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
            if (sub.z() >= -1.0) {
                return t * DT;
            }
        }
        return -1; // did not surface
    }

    double findThrottleForSpeed(double targetSpeed, double depth) {
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
}
