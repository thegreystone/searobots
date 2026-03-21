package se.hirt.searobots.engine.ships.claude;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies that ClaudeAttackSub follows mandatory objectives in order
 * without replanning, and reaches both waypoints sequentially.
 */
class ClaudeObjectiveTest {

    private static final int TICKS_PER_SECOND = 50;

    record ObjectiveResult(long seed, int objectivesHit, double closestToObj1, double closestToObj2,
                           long obj1HitTick, long obj2HitTick, boolean reachedInOrder, boolean alive,
                           List<double[]> positionLog) {}

    private ObjectiveResult runObjectiveTest(long seed, int durationSeconds) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();
        var objectives = SubmarineCompetition.generateObjectives(seed, world);

        var controller = new ClaudeAttackSub();
        var sp = world.spawnPoints().get(0);
        double heading = WorldGenerator.findSafeHeading(terrain, sp.x(), sp.y());
        if (Double.isNaN(heading)) {
            heading = Math.atan2(-sp.x(), -sp.y());
            if (heading < 0) heading += 2 * Math.PI;
        }

        var physics = new SubmarinePhysics();
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, controller,
                sp, heading, java.awt.Color.BLUE, 1000);

        var context = new MatchContext(config, terrain, world.thermalLayers(),
                world.currentField());
        controller.onMatchStart(context);

        // Inject objectives BEFORE first tick (same as CompetitionRunner fix)
        double depth1 = Math.max(-300, terrain.elevationAt(objectives.x1(), objectives.y1()) + 90);
        double depth2 = Math.max(-300, terrain.elevationAt(objectives.x2(), objectives.y2()) + 90);
        controller.setObjectives(List.of(
                new StrategicWaypoint(objectives.x1(), objectives.y1(), depth1,
                        Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1),
                new StrategicWaypoint(objectives.x2(), objectives.y2(), depth2,
                        Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1)
        ));

        double closestToObj1 = Double.MAX_VALUE;
        double closestToObj2 = Double.MAX_VALUE;
        long obj1HitTick = -1;
        long obj2HitTick = -1;
        double OBJ_RADIUS = 400.0;
        var posLog = new ArrayList<double[]>();

        int durationTicks = durationSeconds * TICKS_PER_SECOND;
        for (int t = 0; t < durationTicks; t++) {
            var pose = new Pose(new Vec3(entity.x(), entity.y(), entity.z()),
                    entity.heading(), entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(entity.speed() * Math.sin(entity.heading()),
                            entity.speed() * Math.cos(entity.heading()),
                            entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, entity.hp(), 0);
            var env = new EnvironmentSnapshot(terrain, List.of(), world.currentField());
            var input = new TestHelpers.TestInput(t, 1.0 / TICKS_PER_SECOND, state, env,
                    List.of(), List.of(), 0);
            var output = new TestHelpers.CapturedOutput();
            controller.onTick(input, output);

            entity.setThrottle(output.throttle);
            entity.setRudder(output.rudder);
            entity.setSternPlanes(output.sternPlanes);
            entity.setBallast(output.ballast);
            physics.step(entity, 1.0 / TICKS_PER_SECOND, terrain, world.currentField(),
                    config.battleArea());

            if (entity.hp() <= 0 || entity.forfeited()) break;

            // Track objective proximity
            double d1 = Math.hypot(entity.x() - objectives.x1(), entity.y() - objectives.y1());
            if (d1 < closestToObj1) closestToObj1 = d1;
            if (obj1HitTick < 0 && d1 < OBJ_RADIUS) obj1HitTick = t;

            double d2 = Math.hypot(entity.x() - objectives.x2(), entity.y() - objectives.y2());
            if (d2 < closestToObj2) closestToObj2 = d2;
            if (obj1HitTick >= 0 && obj2HitTick < 0 && d2 < OBJ_RADIUS) obj2HitTick = t;

            // Stop early if both reached
            if (obj1HitTick >= 0 && obj2HitTick >= 0) break;

            // Log position every 10 seconds for diagnostics
            if (t % 500 == 0) {
                posLog.add(new double[]{entity.x(), entity.y(), entity.z()});
            }
        }

        int hits = 0;
        if (obj1HitTick >= 0) hits++;
        if (obj2HitTick >= 0) hits++;
        boolean inOrder = obj1HitTick < 0 || obj2HitTick < 0 || obj1HitTick < obj2HitTick;

        return new ObjectiveResult(seed, hits, closestToObj1, closestToObj2,
                obj1HitTick, obj2HitTick, inOrder, entity.hp() > 0, posLog);
    }

    @Test
    void objectivesFollowedInOrder() {
        long[] seeds = {42, 123, 999, 2024, 7777};
        int passed = 0;

        System.out.println("=== Objective Following Test: ClaudeAttackSub ===");
        for (long seed : seeds) {
            var r = runObjectiveTest(seed, 1800); // 30 minutes per seed
            boolean ok = r.objectivesHit >= 2 && r.reachedInOrder && r.alive;
            if (ok) passed++;
            System.out.printf("  Seed %5d: %d/2 obj, closest1=%.0fm closest2=%.0fm, " +
                            "wp1@t=%d wp2@t=%d, order=%s, alive=%s  %s%n",
                    seed, r.objectivesHit, r.closestToObj1, r.closestToObj2,
                    r.obj1HitTick, r.obj2HitTick,
                    r.reachedInOrder ? "OK" : "WRONG",
                    r.alive ? "yes" : "no",
                    ok ? "PASS" : "FAIL");
        }
        System.out.printf("Result: %d/%d seeds passed (2/2 objectives, in order, alive)%n", passed, seeds.length);
        assertTrue(passed >= 4,
                "At least 4/5 seeds should hit both objectives in order, got " + passed + "/5");
    }

    @Test
    void objectivesNotReplanned() {
        // Run one seed and verify the controller's published strategic waypoints
        // always show objectives (not patrol points) while objectives are active
        long seed = 42;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();
        var objectives = SubmarineCompetition.generateObjectives(seed, world);

        var controller = new ClaudeAttackSub();
        var sp = world.spawnPoints().get(0);
        double heading = WorldGenerator.findSafeHeading(terrain, sp.x(), sp.y());
        if (Double.isNaN(heading)) {
            heading = Math.atan2(-sp.x(), -sp.y());
            if (heading < 0) heading += 2 * Math.PI;
        }

        var physics = new SubmarinePhysics();
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, controller,
                sp, heading, java.awt.Color.BLUE, 1000);

        var context = new MatchContext(config, terrain, world.thermalLayers(),
                world.currentField());
        controller.onMatchStart(context);

        double depth1 = Math.max(-300, terrain.elevationAt(objectives.x1(), objectives.y1()) + 90);
        double depth2 = Math.max(-300, terrain.elevationAt(objectives.x2(), objectives.y2()) + 90);
        controller.setObjectives(List.of(
                new StrategicWaypoint(objectives.x1(), objectives.y1(), depth1,
                        Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1),
                new StrategicWaypoint(objectives.x2(), objectives.y2(), depth2,
                        Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1)
        ));

        int nonObjectiveTicks = 0;
        int totalTicks = 0;
        int durationTicks = 300 * TICKS_PER_SECOND; // 5 minutes

        for (int t = 0; t < durationTicks; t++) {
            var pose = new Pose(new Vec3(entity.x(), entity.y(), entity.z()),
                    entity.heading(), entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(entity.speed() * Math.sin(entity.heading()),
                            entity.speed() * Math.cos(entity.heading()),
                            entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, entity.hp(), 0);
            var env = new EnvironmentSnapshot(terrain, List.of(), world.currentField());
            var input = new TestHelpers.TestInput(t, 1.0 / TICKS_PER_SECOND, state, env,
                    List.of(), List.of(), 0);
            var output = new TestHelpers.CapturedOutput();
            controller.onTick(input, output);

            entity.setThrottle(output.throttle);
            entity.setRudder(output.rudder);
            entity.setSternPlanes(output.sternPlanes);
            entity.setBallast(output.ballast);
            physics.step(entity, 1.0 / TICKS_PER_SECOND, terrain, world.currentField(),
                    config.battleArea());

            if (entity.hp() <= 0 || entity.forfeited()) break;

            // Check published strategic waypoints: while objectives are active,
            // the active waypoint should be near one of the objectives
            if (!output.strategicWaypoints.isEmpty()) {
                totalTicks++;
                boolean nearObjective = false;
                for (var wp : output.strategicWaypoints) {
                    if (wp.active()) {
                        double dObj1 = Math.hypot(wp.x() - objectives.x1(), wp.y() - objectives.y1());
                        double dObj2 = Math.hypot(wp.x() - objectives.x2(), wp.y() - objectives.y2());
                        if (dObj1 < 50 || dObj2 < 50) nearObjective = true;
                        break;
                    }
                }
                if (!nearObjective) nonObjectiveTicks++;
            }
        }

        double nonObjPct = totalTicks > 0 ? 100.0 * nonObjectiveTicks / totalTicks : 0;
        System.out.printf("Non-objective active waypoint ticks: %d/%d (%.1f%%)%n",
                nonObjectiveTicks, totalTicks, nonObjPct);

        // During objective navigation, the controller should NEVER have non-objective waypoints active.
        // After objectives are done it switches to patrol, which is fine.
        // Allow some tolerance for the transition period.
        assertTrue(nonObjPct < 5.0,
                "Controller should not replan away from objectives. Non-objective ticks: " + nonObjPct + "%");
    }
}
