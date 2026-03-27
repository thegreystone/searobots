package se.hirt.searobots.engine.ships.codex;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.Pose;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoLaunchContext;
import se.hirt.searobots.api.TorpedoOutput;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.api.Velocity;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.TorpedoEntity;
import se.hirt.searobots.engine.TorpedoPhysics;

import java.awt.Color;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertTrue;

class CodexTorpedoGuidanceTest {
    private static final double DT = 1.0 / 50.0;

    @Test
    void closeRangeActiveFixCommandsDeepDive() {
        var controller = new CodexTorpedoController();
        var world = GeneratedWorld.deepFlat();
        controller.onLaunch(new TorpedoLaunchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                new Vec3(0.0, -600.0, -120.0),
                0.0,
                0.0,
                "0,1200,-120,0,0"));

        controller.onTick(new FixedInput(
                        0L,
                        new Pose(new Vec3(0.0, 0.0, -120.0), 0.0, 0.0, 0.0),
                        Velocity.ZERO,
                        25.0,
                        120.0,
                        List.of(),
                        List.of(),
                        0),
                new CapturingOutput());

        var output = new CapturingOutput();
        controller.onTick(new FixedInput(
                600L,
                new Pose(new Vec3(0.0, 0.0, -120.0), 0.0, 0.0, 0.0),
                Velocity.ZERO,
                25.0,
                120.0,
                List.of(),
                List.of(activeContact(0.0, 240.0, -280.0)),
                0),
                output);

        assertTrue(output.publishedTargetZ < -220.0,
                "A close deep active fix should move the target depth deep, got " + output.publishedTargetZ);
        assertTrue(output.sternPlanes < -0.8,
                "A close deep active fix should command a strong dive, got " + output.sternPlanes);
        assertTrue(output.throttle <= 0.55,
                "A close deep active fix with a large vertical intercept error should slow down, got " + output.throttle);
    }

    @Test
    void deepMissionTargetBuildsDepthBeforeTerminal() {
        var outcome = runTerminalApproach(
                new Vec3(0.0, 700.0, -280.0),
                -280.0,
                0.0,
                2_500,
                false);

        assertTrue(outcome.depthAt300m < -180.0,
                "Codex should already be deep before terminal. " + outcome);
        assertTrue(outcome.bestDistance < 120.0,
                "Codex should get close to a deep mission target. " + outcome);
    }

    @Test
    void lateDeepActiveFixStillClosesDepthGap() {
        var outcome = runTerminalApproach(
                new Vec3(0.0, 1_000.0, -280.0),
                -120.0,
                450.0,
                3_500,
                true);

        assertTrue(outcome.depthAt300m < -120.0,
                "A late accurate active depth fix should still produce a meaningful pre-terminal dive. " + outcome);
        assertTrue(outcome.bestDistance < 60.0,
                "Accurate active terminal fixes should produce a near-lethal close approach. " + outcome);
    }

    private TerminalOutcome runTerminalApproach(Vec3 target,
                                                double missionDepth,
                                                double activeStartRange,
                                                int maxTicks,
                                                boolean gateActiveByRange) {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexTorpedoController();
        var torpedo = new TorpedoEntity(
                1,
                0,
                VehicleConfig.torpedo(),
                controller,
                new Vec3(0.0, 0.0, -100.0),
                0.0,
                0.0,
                30.0,
                Color.GREEN);
        torpedo.setSpeed(25.0);
        controller.onLaunch(new TorpedoLaunchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                new Vec3(0.0, 0.0, -100.0),
                0.0,
                0.0,
                String.format(Locale.US, "0,%.1f,%.1f,0,0", target.y(), missionDepth)));

        var physics = new TorpedoPhysics();
        double bestDistance = Double.POSITIVE_INFINITY;
        double depthAt300m = Double.NaN;
        double bestDepthGap = Double.POSITIVE_INFINITY;

        for (int tick = 0; tick < maxTicks && torpedo.alive(); tick++) {
            var pos = torpedo.pose().position();
            double dx = target.x() - pos.x();
            double dy = target.y() - pos.y();
            double dz = target.z() - pos.z();
            double horizontalRange = Math.hypot(dx, dy);
            double slantRange = Math.sqrt(dx * dx + dy * dy + dz * dz);
            boolean activeAvailable = !gateActiveByRange || horizontalRange <= activeStartRange;
            List<SonarContact> activeReturns = activeAvailable
                    ? List.of(activeContact(bearingTo(pos, target), slantRange, target.z()))
                    : List.of();

            var input = new FixedInput(
                    tick,
                    torpedo.pose(),
                    torpedo.velocity(),
                    torpedo.speed(),
                    torpedo.fuelRemaining(),
                    List.of(),
                    activeReturns,
                    0);

            controller.onTick(input, torpedo.createOutput());
            physics.step(torpedo, DT, world.terrain(), null, world.config().battleArea());
            torpedo.decrementArmingDelay();

            var steppedPos = torpedo.pose().position();
            double currentDistance = steppedPos.distanceTo(target);
            double steppedHorizontalRange = Math.hypot(target.x() - steppedPos.x(), target.y() - steppedPos.y());
            double depthGap = Math.abs(torpedo.z() - target.z());
            bestDistance = Math.min(bestDistance, currentDistance);
            bestDepthGap = Math.min(bestDepthGap, depthGap);
            if (Double.isNaN(depthAt300m) && steppedHorizontalRange <= 300.0) {
                depthAt300m = torpedo.z();
            }
        }

        return new TerminalOutcome(bestDistance, bestDepthGap, depthAt300m, torpedo.z());
    }

    private static SonarContact activeContact(double bearing, double range, double depth) {
        return new SonarContact(
                bearing,
                35.0,
                range,
                true,
                0.0,
                Math.toRadians(0.3),
                Math.max(2.0, range * 0.02),
                220.0,
                0.95,
                Double.NaN,
                depth);
    }

    private static double bearingTo(Vec3 from, Vec3 to) {
        double bearing = Math.atan2(to.x() - from.x(), to.y() - from.y());
        return bearing < 0.0 ? bearing + 2.0 * Math.PI : bearing;
    }

    private record TerminalOutcome(double bestDistance,
                                   double bestDepthGap,
                                   double depthAt300m,
                                   double finalDepth) {
        @Override
        public String toString() {
            return String.format(Locale.US,
                    "bestDistance=%.1f bestDepthGap=%.1f depthAt300m=%.1f finalDepth=%.1f",
                    bestDistance, bestDepthGap, depthAt300m, finalDepth);
        }
    }

    private record FixedInput(long tick,
                              Pose self,
                              Velocity velocity,
                              double speed,
                              double fuelRemaining,
                              List<SonarContact> sonarContacts,
                              List<SonarContact> activeSonarReturns,
                              int activeSonarCooldownTicks) implements TorpedoInput {
        @Override
        public double deltaTimeSeconds() {
            return DT;
        }
    }

    private static final class CapturingOutput implements TorpedoOutput {
        private double sternPlanes;
        private double throttle = Double.NaN;
        private double publishedTargetZ = Double.NaN;

        @Override
        public void setRudder(double value) {
        }

        @Override
        public void setSternPlanes(double value) {
            sternPlanes = value;
        }

        @Override
        public void setThrottle(double value) {
            throttle = value;
        }

        @Override
        public void publishTarget(double x, double y, double z) {
            publishedTargetZ = z;
        }
    }
}
