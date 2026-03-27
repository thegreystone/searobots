package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.awt.Color;

/**
 * Measures torpedo turn radius and pitch rate at different speeds.
 * Used to calibrate the torpedo controller's throttle management
 * and understand maneuverability at different speeds.
 */
class TorpedoTurnRadiusTest {

    static final double DT = 1.0 / 50;
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    @Test
    void measureTurnRadiusAtSpeeds() {
        System.out.println("=== Torpedo Turn Radius / Pitch Rate Table ===");
        System.out.printf("%-8s %-12s %-12s %-12s %-12s%n",
                "Speed", "Yaw Radius", "Yaw Rate", "Pitch Rate", "Pitch Radius");
        System.out.println("-".repeat(60));

        for (int speed = 5; speed <= 25; speed += 5) {
            double[] yawResult = measureYawTurn(speed);
            double[] pitchResult = measurePitchTurn(speed);
            System.out.printf("%4d m/s  %8.0fm     %6.2f°/s    %6.2f°/s     %8.0fm%n",
                    speed, yawResult[0], yawResult[1], pitchResult[1], pitchResult[0]);
        }

        // Also test intermediate speeds for the controller
        System.out.println("\n=== Fine-grained yaw turn radius ===");
        System.out.printf("%-8s %-12s %-12s%n", "Speed", "Yaw Radius", "Yaw Rate");
        for (int speed = 3; speed <= 25; speed++) {
            double[] r = measureYawTurn(speed);
            System.out.printf("%4d m/s  %8.0fm     %6.2f°/s%n", speed, r[0], r[1]);
        }
    }

    /** Measure steady-state yaw turn radius at full rudder. */
    private double[] measureYawTurn(int targetSpeed) {
        var cfg = VehicleConfig.torpedo();
        var torp = createTorpedo(cfg, targetSpeed);

        // Apply full rudder and run until yaw rate stabilizes
        TorpedoPhysics physics = new TorpedoPhysics();
        double startHeading = 0;
        double totalAngle = 0;
        double steadyYawRate = 0;
        int settleTime = 500; // 10 seconds to reach steady state

        for (int tick = 0; tick < settleTime + 500; tick++) {
            torp.applyCommands(); // no-op, we set directly
            // Full rudder, no stern planes, throttle to maintain speed
            double speedError = targetSpeed - torp.speed();
            double throttle = Math.clamp(speedError * 0.5 + 0.5, 0, 1);
            torp.setActualRudder(1.0);
            torp.setActualSternPlanes(0);
            torp.setActualThrottle(throttle);

            double prevHeading = torp.heading();
            physics.step(torp, DT, world.terrain(), null, null);

            if (tick >= settleTime) {
                double dh = torp.heading() - prevHeading;
                while (dh > Math.PI) dh -= 2 * Math.PI;
                while (dh < -Math.PI) dh += 2 * Math.PI;
                totalAngle += Math.abs(dh);
                steadyYawRate = Math.abs(dh) / DT;
            }
        }

        double avgYawRate = totalAngle / (500 * DT); // rad/s over measurement window
        double speed = torp.speed();
        double radius = speed > 0.1 && avgYawRate > 0.001 ? speed / avgYawRate : Double.POSITIVE_INFINITY;

        return new double[]{radius, Math.toDegrees(avgYawRate)};
    }

    /** Measure steady-state pitch rate at full stern planes. */
    private double[] measurePitchTurn(int targetSpeed) {
        var cfg = VehicleConfig.torpedo();
        var torp = createTorpedo(cfg, targetSpeed);

        TorpedoPhysics physics = new TorpedoPhysics();
        double totalAngle = 0;
        int settleTime = 500;

        for (int tick = 0; tick < settleTime + 500; tick++) {
            double speedError = targetSpeed - torp.speed();
            double throttle = Math.clamp(speedError * 0.5 + 0.5, 0, 1);
            torp.setActualRudder(0);
            torp.setActualSternPlanes(1.0); // full dive
            torp.setActualThrottle(throttle);

            double prevPitch = torp.pitch();
            physics.step(torp, DT, world.terrain(), null, null);

            if (tick >= settleTime) {
                double dp = torp.pitch() - prevPitch;
                totalAngle += Math.abs(dp);
            }
        }

        double avgPitchRate = totalAngle / (500 * DT);
        double speed = torp.speed();
        double radius = speed > 0.1 && avgPitchRate > 0.001 ? speed / avgPitchRate : Double.POSITIVE_INFINITY;

        return new double[]{radius, Math.toDegrees(avgPitchRate)};
    }

    private TorpedoEntity createTorpedo(VehicleConfig cfg, double initialSpeed) {
        var torp = new TorpedoEntity(9000, 0, cfg, null,
                new Vec3(0, 0, -200), 0, 0, 20.0, Color.RED);
        torp.setSpeed(initialSpeed);
        torp.setActualThrottle(0.5);
        return torp;
    }
}
