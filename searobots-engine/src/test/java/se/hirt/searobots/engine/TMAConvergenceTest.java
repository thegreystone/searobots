package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.SubmarineDrone;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests that passive TMA (Target Motion Analysis) behaves realistically:
 * <ul>
 *   <li>Straight-line running gives bearing but poor range</li>
 *   <li>Cross-track maneuvering converges range within ~90 seconds</li>
 *   <li>Multiple legs give a good firing solution without active pings</li>
 *   <li>Active ping gives instant accurate range</li>
 * </ul>
 */
public class TMAConvergenceTest {

    // Tests use GeneratedWorld.deepFlat(): 2000m apart, flat ocean, no thermocline

    /**
     * A submarine that follows scripted legs and records sonar contact data.
     */
    static class ScriptedSub implements SubmarineController {
        record Leg(double heading, int durationTicks) {}

        private final List<Leg> legs;
        private final double depth;
        private final double speed;
        private final boolean pingOnce;
        private int currentLeg = 0;
        private int ticksOnLeg = 0;
        private boolean pinged = false;

        // Collected TMA data from the controller's perspective
        volatile double lastRange, lastQuality, lastUncertainty, lastEstHeading = Double.NaN;
        volatile boolean hasContact;

        ScriptedSub(List<Leg> legs, double depth, double speed, boolean pingOnce) {
            this.legs = legs;
            this.depth = depth;
            this.speed = speed;
            this.pingOnce = pingOnce;
        }

        @Override public String name() { return "ScriptedSub"; }
        @Override public void onMatchStart(MatchContext context) {}

        @Override
        public void onTick(SubmarineInput input, SubmarineOutput output) {
            if (currentLeg >= legs.size()) {
                output.setThrottle(0);
                return;
            }
            var leg = legs.get(currentLeg);
            var pose = input.self().pose();
            double heading = pose.heading();
            double error = leg.heading - heading;
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            output.setRudder(Math.clamp(error * 2, -1, 1));

            double speedError = speed - input.self().surgeSpeed();
            output.setThrottle(Math.clamp(speedError * 0.5 + 0.3, 0, 0.7));

            double depthError = depth - pose.position().z();
            output.setBallast(Math.clamp(-depthError * 0.01, -1, 1));
            output.setSternPlanes(Math.clamp(depthError * 0.05, -1, 1));

            ticksOnLeg++;
            if (ticksOnLeg >= leg.durationTicks) {
                currentLeg++;
                ticksOnLeg = 0;
            }

            // Optional single ping (before reading contacts, so we see the return)
            if (pingOnce && !pinged && input.activeSonarCooldownTicks() == 0
                    && !input.sonarContacts().isEmpty()) {
                output.activeSonarPing();
                pinged = true;
            }

            // Collect sonar contact data (prefer active returns over passive)
            var activeReturns = input.activeSonarReturns();
            var passiveContacts = input.sonarContacts();
            if (!activeReturns.isEmpty()) {
                var c = activeReturns.getFirst();
                lastRange = c.range();
                lastQuality = c.solutionQuality();
                lastUncertainty = c.rangeUncertainty();
                lastEstHeading = c.estimatedHeading();
                hasContact = true;
            } else if (!passiveContacts.isEmpty()) {
                var c = passiveContacts.getFirst();
                lastRange = c.range();
                lastQuality = c.solutionQuality();
                lastUncertainty = c.rangeUncertainty();
                lastEstHeading = c.estimatedHeading();
                hasContact = true;
            }
        }
    }

    private double actualDistance(List<SubmarineSnapshot> subs) {
        if (subs.size() < 2) return 0;
        var p0 = subs.get(0).pose().position();
        var p1 = subs.get(1).pose().position();
        double dx = p1.x() - p0.x(), dy = p1.y() - p0.y();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * A target that cruises north at moderate speed (detectable at 2km).
     */
    static class NoisyTarget implements SubmarineController {
        @Override public String name() { return "NoisyTarget"; }
        @Override public void onMatchStart(MatchContext context) {}
        @Override
        public void onTick(SubmarineInput input, SubmarineOutput output) {
            double heading = input.self().pose().heading();
            double error = 0 - heading; // north
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            output.setRudder(Math.clamp(error * 2, -1, 1));
            output.setThrottle(0.5); // ~8 m/s, SL ~96 dB
            double depthError = -100 - input.self().pose().position().z();
            output.setBallast(Math.clamp(-depthError * 0.01, -1, 1));
            output.setSternPlanes(Math.clamp(depthError * 0.05, -1, 1));
        }
    }

    // ── Tests ──

    @Test
    void straightLineGivesPoorRange() {
        // Slow speed to minimize self-noise, drive east (toward target).
        // Along-bearing motion creates minimal cross-track = poor TMA geometry.
        var sub = new ScriptedSub(
                List.of(new ScriptedSub.Leg(Math.toRadians(90), 3000)), // east, 60s
                -100, 3, false);
        var target = new NoisyTarget();

        double actual = runSim(sub, target, 3000);

        System.out.printf("Straight line: range=%.0f actual=%.0f error=%.0f%% quality=%.2f uncertainty=%.0f%n",
                sub.lastRange, actual, errorPct(sub.lastRange, actual), sub.lastQuality, sub.lastUncertainty);

        // Range error should be large: driving toward the target gives
        // bearing but very poor range resolution
        assertTrue(errorPct(sub.lastRange, actual) > 30 || sub.lastUncertainty > actual * 0.5,
                "Straight line should give poor range. Error: " + errorPct(sub.lastRange, actual)
                        + "%, uncertainty: " + sub.lastUncertainty);
        // Quality may build slightly from the initial turn and small bearing
        // changes, but should remain below what a proper zig-zag achieves
        assertTrue(sub.lastQuality < 0.50,
                "Quality should stay moderate-low without deliberate maneuvering. Got: " + sub.lastQuality);
    }

    @Test
    void twoLegManeuverConvergesRange() {
        var sub = new ScriptedSub(List.of(
                new ScriptedSub.Leg(Math.toRadians(60), 1500),   // NE, 30s
                new ScriptedSub.Leg(Math.toRadians(120), 1500),  // SE, 30s
                new ScriptedSub.Leg(Math.toRadians(60), 1500)    // NE again, 30s
        ), -100, 7, false);
        var target = new NoisyTarget();

        double actual = runSim(sub, target, 4500);

        System.out.printf("Two-leg: range=%.0f actual=%.0f error=%.0f%% quality=%.2f uncertainty=%.0f%n",
                sub.lastRange, actual, errorPct(sub.lastRange, actual), sub.lastQuality, sub.lastUncertainty);

        assertTrue(errorPct(sub.lastRange, actual) < 35,
                "Two-leg maneuver should give decent range. Error: " + errorPct(sub.lastRange, actual) + "%");
        assertTrue(sub.lastQuality > 0.35,
                "Quality should improve with legs. Got: " + sub.lastQuality);
    }

    @Test
    void threeLegGivesFiringSolution() {
        var sub = new ScriptedSub(List.of(
                new ScriptedSub.Leg(Math.toRadians(45), 1500),
                new ScriptedSub.Leg(Math.toRadians(135), 1500),
                new ScriptedSub.Leg(Math.toRadians(45), 1500),
                new ScriptedSub.Leg(Math.toRadians(315), 1500)
        ), -100, 7, false);
        var target = new NoisyTarget();

        double actual = runSim(sub, target, 6000);

        System.out.printf("Three-leg: range=%.0f actual=%.0f error=%.0f%% quality=%.2f uncertainty=%.0f heading=%s%n",
                sub.lastRange, actual, errorPct(sub.lastRange, actual), sub.lastQuality, sub.lastUncertainty,
                Double.isNaN(sub.lastEstHeading) ? "none" : String.format("%.0f°", Math.toDegrees(sub.lastEstHeading)));

        assertTrue(errorPct(sub.lastRange, actual) < 15,
                "Three legs should give good range. Error: " + errorPct(sub.lastRange, actual) + "%");
        assertTrue(sub.lastQuality > 0.4,
                "Quality should be meaningful with three legs. Got: " + sub.lastQuality);
    }

    @Test
    void activePingGivesInstantRange() {
        var sub = new ScriptedSub(
                List.of(new ScriptedSub.Leg(Math.toRadians(90), 3000)),
                -100, 7, true); // pings once
        var target = new NoisyTarget();

        double actual = runSim(sub, target, 3000); // needs enough time for ping + return

        System.out.printf("Active ping: range=%.0f actual=%.0f error=%.0f%% quality=%.2f%n",
                sub.lastRange, actual, errorPct(sub.lastRange, actual), sub.lastQuality);

        assertTrue(sub.lastRange > 0,
                "Active ping should produce a range estimate");
        assertTrue(errorPct(sub.lastRange, actual) < 5,
                "Active ping should give accurate range. Error: " + errorPct(sub.lastRange, actual) + "%");
    }

    @Test
    void convergenceTimeline() {
        // Sub starts west of target (target is at +1000, sub at -1000).
        // Slow speed (3 m/s) keeps self-noise low for detection.
        // North/south legs give maximum cross-track relative to bearing line.
        var sub = new ScriptedSub(List.of(
                new ScriptedSub.Leg(Math.toRadians(0), 1500),    // north, 30s
                new ScriptedSub.Leg(Math.toRadians(180), 1500),  // south, 30s
                new ScriptedSub.Leg(Math.toRadians(0), 1500),    // north, 30s
                new ScriptedSub.Leg(Math.toRadians(180), 1500),  // south, 30s
                new ScriptedSub.Leg(Math.toRadians(0), 1500),    // north, 30s
                new ScriptedSub.Leg(Math.toRadians(180), 1500),  // south, 30s
                new ScriptedSub.Leg(Math.toRadians(0), 1500)     // north, 30s
        ), -100, 3, false);
        var target = new NoisyTarget();

        System.out.println("=== TMA Convergence Timeline (zig-zag) ===");
        System.out.printf("%-8s %-10s %-10s %-8s %-8s %-10s%n",
                "Time(s)", "Range", "Actual", "Error%", "Quality", "Uncert");

        var world = GeneratedWorld.deepFlat();
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(sub, target);
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() >= 2 && tick % 500 == 0 && tick > 0) {
                    double actual = actualDistance(subs);
                    if (sub.hasContact) {
                        System.out.printf("%-8.0f %-10.0f %-10.0f %-8.0f %-8.2f %-10.0f%n",
                                tick / 50.0, sub.lastRange, actual,
                                errorPct(sub.lastRange, actual),
                                sub.lastQuality, sub.lastUncertainty);
                    } else {
                        System.out.printf("%-8.0f %-10s %-10.0f %-8s %-8s %-10s%n",
                                tick / 50.0, "---", actual, "---", "---", "no contact");
                    }
                }
                if (tick >= 10000) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(30_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}
    }

    // ── Helpers ──

    private double errorPct(double estimated, double actual) {
        return actual > 0 ? Math.abs(estimated - actual) / actual * 100 : 100;
    }

    private double runSim(ScriptedSub sub, SubmarineController target, int maxTicks) {
        var world = GeneratedWorld.deepFlat();
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(sub, target);
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        double[] lastActual = {0};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                lastActual[0] = actualDistance(subs);
                if (tick >= maxTicks) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(30_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}

        return lastActual[0];
    }
}
