package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;

import java.awt.*;

/**
 * Physics characterization: systematic verification of submarine behavior
 * under controlled inputs. No AI, no navigation - just raw physics.
 */
public class PhysicsCharacterization {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    public static void main(String[] args) {
        System.out.println("=== SUBMARINE PHYSICS CHARACTERIZATION ===\n");

        test_SteadyStateSpeed();
        test_TurnRadiusVsSpeed();
        test_TurnRadiusVsRudder();
        test_RudderResponseOverTime();
        test_PitchResponseOverTime();
        test_PitchEquilibrium();
        test_ReverseThrottle();
        test_ReverseRudder();
        test_EmergencyBallastBlow();
        test_CoastingAfterEngineOff();
        test_DepthChangeWithPlanes();
        test_ZigZagManeuver();
        test_RudderStepResponse();
        test_StraightLineStability();
        test_SpeedLossDuringTurn();

        System.out.println("=== ALL EXPERIMENTS COMPLETE ===");
    }

    // --- Experiment: Steady-state speed at various throttles ---
    static void test_SteadyStateSpeed() {
        System.out.println("--- Steady-State Speed vs Throttle ---");
        System.out.printf("%-12s %-12s %-12s %-12s%n", "Throttle%", "Speed m/s", "Speed kn", "Time to SS");

        for (double throttle : new double[]{0.1, 0.2, 0.25, 0.4, 0.5, 0.75, 1.0}) {
            var sub = makeSub(0, 0);
            sub.setThrottle(throttle);
            int ticks = runUntilSpeedStable(sub, 50 * 120);
            System.out.printf("%-12.0f %-12.2f %-12.2f %-12.1fs%n",
                    throttle * 100, sub.speed(), sub.speed() * 1.944, ticks / 50.0);
        }
        System.out.println();
    }

    // --- Experiment: Turn radius at various speeds with full rudder ---
    static void test_TurnRadiusVsSpeed() {
        System.out.println("--- Turn Radius vs Speed (Full Rudder=1.0) ---");
        System.out.printf("%-12s %-12s %-14s %-14s %-14s%n",
                "Speed m/s", "Speed kn", "YawRate d/s", "TurnRad m", "360deg time");

        for (double throttle : new double[]{0.1, 0.25, 0.4, 0.5, 0.75, 1.0}) {
            var sub = makeSub(0, 0);
            sub.setThrottle(throttle);
            runUntilSpeedStable(sub, 50 * 60);
            double speed = sub.speed();

            sub.setRudder(1.0);
            double startHdg = sub.heading();
            runTicks(sub, 50 * 30);

            double hdgChange = sub.heading() - startHdg;
            while (hdgChange > Math.PI) hdgChange -= 2 * Math.PI;
            while (hdgChange < -Math.PI) hdgChange += 2 * Math.PI;
            double yawDegPerSec = Math.toDegrees(hdgChange) / 30.0;
            double yawRadPerSec = hdgChange / 30.0;
            double turnRad = Math.abs(yawRadPerSec) > 0.001 ? Math.abs(speed / yawRadPerSec) : 9999;
            double time360 = Math.abs(yawRadPerSec) > 0.001 ? 360.0 / Math.abs(yawDegPerSec) : 9999;

            System.out.printf("%-12.1f %-12.1f %-14.2f %-14.0f %-14.0fs%n",
                    speed, speed * 1.944, yawDegPerSec, turnRad, time360);
        }
        System.out.println();
    }

    // --- Experiment: Turn radius vs rudder deflection at patrol speed ---
    static void test_TurnRadiusVsRudder() {
        System.out.println("--- Turn Radius vs Rudder (Patrol Speed, Throttle=0.4) ---");
        System.out.printf("%-12s %-14s %-14s %-14s%n",
                "Rudder%", "YawRate d/s", "TurnRad m", "360deg time");

        for (double rudder : new double[]{0.1, 0.2, 0.3, 0.5, 0.7, 0.9, 1.0}) {
            var sub = makeSub(0, 0);
            sub.setThrottle(0.4);
            runUntilSpeedStable(sub, 50 * 60);

            sub.setRudder(rudder);
            double startHdg = sub.heading();
            runTicks(sub, 50 * 30);

            double hdgChange = sub.heading() - startHdg;
            while (hdgChange > Math.PI) hdgChange -= 2 * Math.PI;
            while (hdgChange < -Math.PI) hdgChange += 2 * Math.PI;
            double yawDegPerSec = Math.toDegrees(hdgChange) / 30.0;
            double yawRadPerSec = hdgChange / 30.0;
            double turnRad = Math.abs(yawRadPerSec) > 0.001 ? Math.abs(sub.speed() / yawRadPerSec) : 9999;
            double time360 = Math.abs(yawDegPerSec) > 0.01 ? 360.0 / Math.abs(yawDegPerSec) : 9999;

            System.out.printf("%-12.0f %-14.2f %-14.0f %-14.0fs%n",
                    rudder * 100, yawDegPerSec, turnRad, time360);
        }
        System.out.println();
    }

    // --- Experiment: Heading change over time at patrol speed ---
    static void test_RudderResponseOverTime() {
        System.out.println("--- Heading Change Over Time (Patrol Speed, Rudder=0.5) ---");
        System.out.printf("%-10s %-14s %-14s %-12s%n", "Time s", "Heading deg", "YawRate d/s", "Speed m/s");

        var sub = makeSub(0, 0);
        sub.setThrottle(0.4);
        runUntilSpeedStable(sub, 50 * 60);
        sub.setRudder(0.5);

        for (int sec = 0; sec <= 60; sec += 5) {
            if (sec > 0) runTicks(sub, 50 * 5);
            System.out.printf("%-10d %-14.1f %-14.4f %-12.2f%n",
                    sec, Math.toDegrees(sub.heading()),
                    Math.toDegrees(sub.yawRate()), sub.speed());
        }
        System.out.println();
    }

    // --- Experiment: Pitch response over time ---
    static void test_PitchResponseOverTime() {
        System.out.println("--- Pitch Over Time (Patrol Speed, Various Planes) ---");
        System.out.printf("%-8s", "Time s");
        double[] planes = {0.1, 0.25, 0.5, 1.0, -0.5};
        for (double p : planes) System.out.printf("  P=%-5.0f%%", p * 100);
        System.out.println();

        var subs = new SubmarineEntity[planes.length];
        for (int i = 0; i < planes.length; i++) {
            subs[i] = makeSub(0, 0);
            subs[i].setThrottle(0.4);
            runUntilSpeedStable(subs[i], 50 * 60);
            subs[i].setSternPlanes(planes[i]);
        }

        for (int sec = 0; sec <= 30; sec += 2) {
            if (sec > 0) for (var s : subs) runTicks(s, 50 * 2);
            System.out.printf("%-8d", sec);
            for (var s : subs) System.out.printf("  %+7.1f\u00b0", Math.toDegrees(s.pitch()));
            System.out.println();
        }
        System.out.println();
    }

    // --- Experiment: Does pitch reach equilibrium (not hit clamp)? ---
    static void test_PitchEquilibrium() {
        System.out.println("--- Pitch Equilibrium (Does restoring moment balance planes?) ---");
        System.out.printf("%-12s %-14s %-14s%n", "Planes%", "Final Pitch", "At clamp?");

        for (double planes : new double[]{0.1, 0.25, 0.5, 0.75, 1.0}) {
            var sub = makeSub(0, 0);
            sub.setThrottle(0.4);
            runUntilSpeedStable(sub, 50 * 60);
            sub.setSternPlanes(planes);
            runTicks(sub, 50 * 60);
            double pitchDeg = Math.toDegrees(sub.pitch());
            boolean atClamp = Math.abs(pitchDeg) > 44.5;
            System.out.printf("%-12.0f %-14.1f\u00b0 %-14s%n", planes * 100, pitchDeg, atClamp ? "YES" : "no");
        }
        System.out.println();
    }

    // --- Experiment: Reverse throttle behavior ---
    static void test_ReverseThrottle() {
        System.out.println("--- Reverse Throttle ---");
        System.out.printf("%-10s %-12s %-12s%n", "Time s", "Speed m/s", "Throttle");

        var sub = makeSub(0, 0);
        sub.setThrottle(0.4);
        runUntilSpeedStable(sub, 50 * 60);
        System.out.printf("%-10s %-12.2f %-12s%n", "cruise", sub.speed(), "0.4");

        sub.setThrottle(-0.5);
        for (int sec = 0; sec <= 30; sec += 3) {
            if (sec > 0) runTicks(sub, 50 * 3);
            System.out.printf("%-10d %-12.2f %-12.2f%n", sec, sub.speed(), sub.actualThrottle());
        }
        System.out.println();
    }

    // --- Experiment: Rudder in reverse ---
    static void test_ReverseRudder() {
        System.out.println("--- Rudder In Reverse (Throttle=-0.5, Rudder=1.0) ---");
        System.out.printf("%-10s %-14s %-14s %-12s%n", "Time s", "YawRate d/s", "Heading deg", "Speed m/s");

        var sub = makeSub(0, 0);
        sub.setThrottle(-0.5);
        runTicks(sub, 50 * 30); // reach reverse speed
        sub.setRudder(1.0);

        for (int sec = 0; sec <= 30; sec += 5) {
            if (sec > 0) runTicks(sub, 50 * 5);
            System.out.printf("%-10d %-14.4f %-14.1f %-12.2f%n",
                    sec, Math.toDegrees(sub.yawRate()), Math.toDegrees(sub.heading()), sub.speed());
        }
        System.out.println();
    }

    // --- Experiment: Emergency ballast blow (deep -> surface) ---
    static void test_EmergencyBallastBlow() {
        System.out.println("--- Emergency Ballast Blow (from -200m) ---");
        System.out.printf("%-10s %-12s %-12s %-12s%n", "Time s", "Depth m", "Vert Spd", "Ballast");

        var sub = makeSub(0, -200);
        sub.setThrottle(0.25);
        sub.setBallast(0.5); // neutral
        runTicks(sub, 50 * 10);

        // Emergency blow: full buoyancy + nose up
        sub.setBallast(1.0);
        sub.setSternPlanes(0.5); // positive = nose up

        for (int sec = 0; sec <= 60; sec += 5) {
            if (sec > 0) runTicks(sub, 50 * 5);
            System.out.printf("%-10d %-12.1f %-12.2f %-12.2f%n",
                    sec, -sub.z(), sub.verticalSpeed(), sub.actualBallast());
        }
        System.out.println();
    }

    // --- Experiment: Coasting after engine off ---
    static void test_CoastingAfterEngineOff() {
        System.out.println("--- Coasting (Engine Off from Patrol Speed) ---");
        System.out.printf("%-10s %-12s %-12s%n", "Time s", "Speed m/s", "Speed kn");

        var sub = makeSub(0, 0);
        sub.setThrottle(0.4);
        runUntilSpeedStable(sub, 50 * 60);
        System.out.printf("%-10s %-12.2f %-12.2f%n", "start", sub.speed(), sub.speed() * 1.944);

        sub.setThrottle(0);
        for (int sec = 5; sec <= 60; sec += 5) {
            runTicks(sub, 50 * 5);
            System.out.printf("%-10d %-12.2f %-12.2f%n", sec, sub.speed(), sub.speed() * 1.944);
        }
        System.out.println();
    }

    // --- Experiment: Depth change with planes at various speeds ---
    static void test_DepthChangeWithPlanes() {
        System.out.println("--- Depth Change (Planes=0.5 at Various Speeds) ---");
        System.out.printf("%-12s %-14s %-14s %-14s%n", "Speed m/s", "Pitch deg", "Depth after 30s", "Vert rate m/s");

        for (double throttle : new double[]{0.1, 0.25, 0.4, 0.75, 1.0}) {
            var sub = makeSub(0, -200);
            sub.setThrottle(throttle);
            runUntilSpeedStable(sub, 50 * 60);
            double speed = sub.speed();
            double startZ = sub.z();

            sub.setSternPlanes(0.5);
            runTicks(sub, 50 * 30);

            double pitchDeg = Math.toDegrees(sub.pitch());
            double depthChange = sub.z() - startZ;
            double vertRate = depthChange / 30.0;

            System.out.printf("%-12.1f %-14.1f\u00b0 %-14.1fm %-14.2f%n",
                    speed, pitchDeg, depthChange, vertRate);
        }
        System.out.println();
    }

    // --- Experiment: 10/10 Zigzag maneuver (standard IMO test) ---
    // Apply rudder until heading changes by 10 deg, reverse rudder, repeat.
    // Measures overshoot and response time.
    static void test_ZigZagManeuver() {
        System.out.println("--- 10/10 Zigzag Maneuver (Patrol Speed) ---");
        System.out.printf("%-8s %-12s %-12s %-10s%n", "Time s", "Heading deg", "Rudder", "Speed m/s");

        var sub = makeSub(0, -100);
        sub.setThrottle(0.4);
        runUntilSpeedStable(sub, 50 * 60);

        double targetAngle = 10.0; // degrees
        double rudderValue = 0.5;  // use 50% rudder (near optimal)
        sub.setRudder(rudderValue);
        int direction = 1; // 1 = turning right, -1 = turning left
        double baseHeading = Math.toDegrees(sub.heading());
        int reversals = 0;

        for (int tick = 0; tick < 50 * 120 && reversals < 6; tick++) {
            step(sub);
            double hdg = Math.toDegrees(sub.heading());
            double hdgDiff = hdg - baseHeading;
            while (hdgDiff > 180) hdgDiff -= 360;
            while (hdgDiff < -180) hdgDiff += 360;

            if (tick % 50 == 0) {
                System.out.printf("%-8.0f %-12.1f %-12.2f %-10.2f%n",
                        tick / 50.0, hdgDiff, sub.rudder(), sub.speed());
            }

            // Check for reversal
            if (direction > 0 && hdgDiff >= targetAngle) {
                sub.setRudder(-rudderValue);
                direction = -1;
                reversals++;
                System.out.printf(">>> REVERSE at %.1fs, heading=%+.1f deg (overshoot from here)%n",
                        tick / 50.0, hdgDiff);
            } else if (direction < 0 && hdgDiff <= -targetAngle) {
                sub.setRudder(rudderValue);
                direction = 1;
                reversals++;
                System.out.printf(">>> REVERSE at %.1fs, heading=%+.1f deg (overshoot from here)%n",
                        tick / 50.0, hdgDiff);
            }
        }
        System.out.println();
    }

    // --- Experiment: Rudder step response (time constant) ---
    // Measures how quickly yaw rate builds after a step rudder input.
    static void test_RudderStepResponse() {
        System.out.println("--- Rudder Step Response (Patrol Speed, Rudder=0.5 Step) ---");
        System.out.printf("%-8s %-14s %-14s %-14s%n", "Time s", "YawRate d/s", "% of Steady", "Heading deg");

        var sub = makeSub(0, -100);
        sub.setThrottle(0.4);
        runUntilSpeedStable(sub, 50 * 60);

        // Measure steady-state yaw rate first
        var ref = makeSub(0, -100);
        ref.setThrottle(0.4);
        runUntilSpeedStable(ref, 50 * 60);
        ref.setRudder(0.5);
        runTicks(ref, 50 * 30);
        double steadyYawRate = Math.toDegrees(ref.yawRate());

        // Now do the actual step response
        sub.setRudder(0.5);
        double time63 = -1;
        double time95 = -1;

        for (int tick = 0; tick <= 50 * 30; tick++) {
            if (tick > 0) step(sub);
            double yawRate = Math.toDegrees(sub.yawRate());
            double pct = (steadyYawRate != 0) ? yawRate / steadyYawRate * 100 : 0;

            if (tick % 25 == 0) { // every 0.5s
                System.out.printf("%-8.1f %-14.4f %-14.1f %-14.1f%n",
                        tick / 50.0, yawRate, pct, Math.toDegrees(sub.heading()));
            }

            if (time63 < 0 && pct >= 63.0) time63 = tick / 50.0;
            if (time95 < 0 && pct >= 95.0) time95 = tick / 50.0;
        }
        System.out.printf("Steady-state yaw rate: %.4f deg/s%n", steadyYawRate);
        System.out.printf("63%% response time (tau): %.1fs%n", time63);
        System.out.printf("95%% response time: %.1fs%n", time95);
        System.out.println();
    }

    // --- Experiment: Straight line stability ---
    // Start at patrol speed with a small heading perturbation. Does heading converge?
    static void test_StraightLineStability() {
        System.out.println("--- Straight Line Stability (Patrol Speed, 2 deg perturbation) ---");
        System.out.printf("%-8s %-14s %-14s %-12s%n", "Time s", "Heading deg", "YawRate d/s", "Speed m/s");

        // Start with a small heading offset, rudder centered
        var sub = makeSub(Math.toRadians(2.0), -100);
        sub.setThrottle(0.4);
        // Don't wait for steady state, just go
        sub.setRudder(0.0);
        sub.setSternPlanes(0.0);

        for (int sec = 0; sec <= 60; sec += 2) {
            if (sec > 0) runTicks(sub, 50 * 2);
            System.out.printf("%-8d %-14.4f %-14.6f %-12.2f%n",
                    sec, Math.toDegrees(sub.heading()),
                    Math.toDegrees(sub.yawRate()), sub.speed());
        }
        System.out.println();
    }

    // --- Experiment: Speed loss during a turn ---
    // Full rudder turn and measure how speed changes.
    static void test_SpeedLossDuringTurn() {
        System.out.println("--- Speed Loss During Turn (Various Rudder at Patrol Speed) ---");
        System.out.printf("%-10s %-14s %-14s %-14s%n",
                "Rudder%", "Initial m/s", "After 90deg", "Speed Loss%");

        for (double rudder : new double[]{0.3, 0.5, 0.7, 1.0}) {
            var sub = makeSub(0, -100);
            sub.setThrottle(0.4);
            runUntilSpeedStable(sub, 50 * 60);
            double initialSpeed = sub.speed();
            double startHdg = sub.heading();

            sub.setRudder(rudder);

            // Run until 90 degrees of heading change
            for (int tick = 0; tick < 50 * 300; tick++) {
                step(sub);
                double hdgChange = sub.heading() - startHdg;
                while (hdgChange > Math.PI) hdgChange -= 2 * Math.PI;
                while (hdgChange < -Math.PI) hdgChange += 2 * Math.PI;
                if (Math.abs(hdgChange) >= Math.PI / 2) break;
            }
            double finalSpeed = sub.speed();
            double loss = (1.0 - finalSpeed / initialSpeed) * 100;
            System.out.printf("%-10.0f %-14.2f %-14.2f %-14.1f%n",
                    rudder * 100, initialSpeed, finalSpeed, loss);
        }
        System.out.println();
    }

    // --- Helpers ---

    static SubmarineEntity makeSub(double heading, double depth) {
        return new SubmarineEntity(
                VehicleConfig.submarine(), 0, new DummyController(),
                new Vec3(0, 0, depth), heading, Color.RED, 1000);
    }

    static void step(SubmarineEntity sub) {
        physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
    }

    static void runTicks(SubmarineEntity sub, int ticks) {
        for (int t = 0; t < ticks; t++) step(sub);
    }

    static int runUntilSpeedStable(SubmarineEntity sub, int maxTicks) {
        double prev = 0;
        for (int t = 0; t < maxTicks; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
            if (t > 100 && Math.abs(sub.speed() - prev) < 1e-6) return t;
            prev = sub.speed();
        }
        return maxTicks;
    }

    static class DummyController implements SubmarineController {
        @Override public String name() { return "Test"; }
        @Override public void onTick(SubmarineInput input, SubmarineOutput output) {}
    }
}
