package se.hirt.searobots.engine;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Basic autopilot routing exercises in flat deep water (no terrain).
 * Verifies fundamental behavior: straight-line following, turns,
 * depth changes, and waypoint transitions.
 */
class BasicRoutingTest {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    /**
     * Creates a sub at the given position/heading with a direct autopilot
     * (bypassing DefaultAttackSub's patrol waypoint generation).
     */
    record SimSub(SubmarineEntity entity, SubmarineAutopilot autopilot) {}

    SimSub createSub(double x, double y, double z, double heading) {
        var config = MatchConfig.withDefaults(0);
        var context = new MatchContext(config, world.terrain(), List.of(),
                world.currentField());
        var autopilot = new SubmarineAutopilot(context);
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(x, y, z), heading, Color.RED, 1000);
        return new SimSub(entity, autopilot);
    }

    /** Run the simulation for a number of ticks. */
    void runTicks(SubmarineEntity sub, int ticks) {
        for (int t = 0; t < ticks; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
        }
    }

    /** Set strategic waypoints on the autopilot and run the physics loop. */
    void setWaypointAndRun(SimSub sim, double wpX, double wpY, double wpZ,
                            int ticks) {
        var wp = new StrategicWaypoint(wpX, wpY, wpZ, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 200, -1);
        sim.autopilot.setWaypoints(List.of(wp),
                sim.entity.x(), sim.entity.y(), sim.entity.z(),
                sim.entity.heading(), sim.entity.speed());

        for (int t = 0; t < ticks; t++) {
            var pose = new Pose(new Vec3(sim.entity.x(), sim.entity.y(), sim.entity.z()),
                    sim.entity.heading(), sim.entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(sim.entity.speed() * Math.sin(sim.entity.heading()),
                             sim.entity.speed() * Math.cos(sim.entity.heading()),
                             sim.entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, sim.entity.hp(), 0);
            var env = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            sim.autopilot.tick(input, output);

            sim.entity.setThrottle(output.throttle);
            sim.entity.setRudder(output.rudder);
            sim.entity.setSternPlanes(output.sternPlanes);
            sim.entity.setBallast(output.ballast);

            physics.step(sim.entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
        }
    }

    double distanceTo(SubmarineEntity sub, double x, double y) {
        return Math.sqrt(Math.pow(sub.x() - x, 2) + Math.pow(sub.y() - y, 2));
    }

    // ── Test 1: Straight ahead ──────────────────────────────────────

    @Test
    void straightAhead() {
        // Sub at origin heading north, waypoint 2km north. Should go straight.
        var sim = createSub(0, 0, -200, 0);
        setWaypointAndRun(sim, 0, 2000, -200, 50 * 120); // 120s to accelerate and travel

        double dist = distanceTo(sim.entity, 0, 2000);
        System.out.printf("Straight ahead: final pos=(%.0f, %.0f) dist to goal=%.0fm speed=%.1f%n",
                sim.entity.x(), sim.entity.y(), dist, sim.entity.speed());
        assertTrue(dist < 1200, "Should make good progress toward goal, was " + dist + "m away");
        assertTrue(Math.abs(sim.entity.x()) < 200, "Should stay roughly on course, x=" + sim.entity.x());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 2: 45-degree turn ──────────────────────────────────────

    @Test
    void turn45degrees() {
        // Sub heading north, waypoint northeast (45 degrees off)
        var sim = createSub(0, 0, -200, 0);
        setWaypointAndRun(sim, 1414, 1414, -200, 50 * 120);

        double dist = distanceTo(sim.entity, 1414, 1414);
        System.out.printf("45-degree turn: final pos=(%.0f, %.0f) dist to goal=%.0fm%n",
                sim.entity.x(), sim.entity.y(), dist);
        assertTrue(dist < 1200, "Should make progress toward goal, was " + dist + "m away");
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 3: 90-degree turn ──────────────────────────────────────

    @Test
    void turn90degrees() {
        // Sub heading north, waypoint due east
        var sim = createSub(0, 0, -200, 0);
        setWaypointAndRun(sim, 2000, 0, -200, 50 * 150); // 150s for wider turn + travel

        double dist = distanceTo(sim.entity, 2000, 0);
        double bearing = Math.toDegrees(Math.atan2(sim.entity.x(), sim.entity.y()));
        System.out.printf("90-degree turn: final pos=(%.0f, %.0f) dist to goal=%.0fm hdg=%.0f bearing_from_origin=%.0f%n",
                sim.entity.x(), sim.entity.y(), dist, Math.toDegrees(sim.entity.heading()), bearing);
        // Sub should have turned east (positive X). Check that it's east of origin.
        assertTrue(sim.entity.x() > 100, "Should have turned east, x=" + sim.entity.x());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 4: 180-degree turn ─────────────────────────────────────

    @Test
    void turn180degrees() {
        // Sub heading north, waypoint behind (south). Sub must arc around.
        var sim = createSub(0, 0, -200, 0);
        setWaypointAndRun(sim, 0, -2000, -200, 50 * 180); // 180s for U-turn + travel

        double dist = distanceTo(sim.entity, 0, -2000);
        System.out.printf("180-degree turn: final pos=(%.0f, %.0f) dist to goal=%.0fm heading=%.0f%n",
                sim.entity.x(), sim.entity.y(), dist, Math.toDegrees(sim.entity.heading()));
        assertTrue(dist < 2000, "Should make progress toward goal, was " + dist + "m away");
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 5: Depth change (dive) ─────────────────────────────────

    @Test
    void diveToDepth() {
        // Sub at -100m, waypoint ahead and deeper at -300m
        var sim = createSub(0, 0, -100, 0);
        setWaypointAndRun(sim, 0, 2000, -300, 50 * 90);

        System.out.printf("Dive: final pos=(%.0f, %.0f, %.0f) target depth=-300%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z());
        assertTrue(sim.entity.z() < -200, "Should have dived significantly, at z=" + sim.entity.z());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 6: Depth change (rise) ─────────────────────────────────

    @Test
    void riseToDepth() {
        // Sub at -400m, waypoint ahead and shallower at -100m
        var sim = createSub(0, 0, -400, 0);
        setWaypointAndRun(sim, 0, 2000, -100, 50 * 90);

        System.out.printf("Rise: final pos=(%.0f, %.0f, %.0f) target depth=-100%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z());
        assertTrue(sim.entity.z() > -300, "Should have risen significantly, at z=" + sim.entity.z());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 7: Turn + depth change combined ────────────────────────

    @Test
    void turnAndDive() {
        // Sub heading north at -100m, waypoint east and deep
        var sim = createSub(0, 0, -100, 0);
        setWaypointAndRun(sim, 2000, 0, -300, 50 * 180);

        double dist = distanceTo(sim.entity, 2000, 0);
        System.out.printf("Turn+dive: final pos=(%.0f, %.0f, %.0f) dist=%.0fm%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z(), dist);
        assertTrue(dist < 1800, "Should make progress toward goal, was " + dist + "m away");
        assertTrue(sim.entity.z() < -150, "Should have dived, at z=" + sim.entity.z());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 8: Sequential waypoints (S-turn) ─────────────────────

    @Test
    void sequentialWaypoints() {
        // Sub heading north, first waypoint east, then northeast.
        // Tests that the sub follows multiple waypoints in sequence.
        var sim = createSub(0, 0, -200, 0);
        var wp1 = new StrategicWaypoint(1000, 500, -200, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1);
        var wp2 = new StrategicWaypoint(2000, 2000, -200, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1);
        sim.autopilot.setWaypoints(List.of(wp1, wp2),
                0, 0, -200, 0, 0);

        // Run until first waypoint reached, then advance
        boolean advanced = false;
        for (int t = 0; t < 50 * 180; t++) {
            var pose = new Pose(new Vec3(sim.entity.x(), sim.entity.y(), sim.entity.z()),
                    sim.entity.heading(), sim.entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(sim.entity.speed() * Math.sin(sim.entity.heading()),
                             sim.entity.speed() * Math.cos(sim.entity.heading()),
                             sim.entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, sim.entity.hp(), 0);
            var env = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            sim.autopilot.tick(input, output);

            if (!advanced && sim.autopilot.hasArrived()) {
                sim.autopilot.advanceWaypoint(sim.entity.x(), sim.entity.y(),
                        sim.entity.z(), sim.entity.heading(), sim.entity.speed());
                advanced = true;
            }

            sim.entity.setThrottle(output.throttle);
            sim.entity.setRudder(output.rudder);
            sim.entity.setSternPlanes(output.sternPlanes);
            sim.entity.setBallast(output.ballast);
            physics.step(sim.entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
        }

        double dist = distanceTo(sim.entity, 2000, 2000);
        System.out.printf("Sequential WPs: final pos=(%.0f, %.0f) dist to WP2=%.0fm advanced=%s%n",
                sim.entity.x(), sim.entity.y(), dist, advanced);
        assertTrue(sim.entity.x() > 300, "Should have turned east, x=" + sim.entity.x());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 9: Depth staircase ─────────────────────────────────────

    @Test
    void depthStaircase() {
        // Sub at -100m heading north. Dive to -200, then -300, then -400.
        var sim = createSub(0, 0, -100, 0);
        double[] depths = {-200, -300, -400};
        double y = 1000;
        for (double targetZ : depths) {
            setWaypointAndRun(sim, 0, y, targetZ, 50 * 60);
            System.out.printf("Depth staircase: at y=%.0f depth=%.0f (target=%.0f)%n",
                    sim.entity.y(), sim.entity.z(), targetZ);
            y += 1000;
        }
        assertTrue(sim.entity.z() < -300, "Should have reached near -400, at z=" + sim.entity.z());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 10: Slow speed turn ────────────────────────────────────

    @Test
    void slowSpeedTurn() {
        // Sub at very slow speed (just started), 90-degree waypoint.
        // Should handle gracefully without jerking.
        var sim = createSub(0, 0, -200, 0);
        // Set throttle low for slow approach
        var wp = new StrategicWaypoint(1000, 0, -200, Purpose.PATROL,
                NoisePolicy.QUIET, MovementPattern.DIRECT, 200, -1);
        sim.autopilot.setWaypoints(List.of(wp), 0, 0, -200, 0, 0);

        for (int t = 0; t < 50 * 120; t++) {
            var pose = new Pose(new Vec3(sim.entity.x(), sim.entity.y(), sim.entity.z()),
                    sim.entity.heading(), sim.entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(sim.entity.speed() * Math.sin(sim.entity.heading()),
                             sim.entity.speed() * Math.cos(sim.entity.heading()),
                             sim.entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, sim.entity.hp(), 0);
            var env = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            sim.autopilot.tick(input, output);

            sim.entity.setThrottle(output.throttle);
            sim.entity.setRudder(output.rudder);
            sim.entity.setSternPlanes(output.sternPlanes);
            sim.entity.setBallast(output.ballast);
            physics.step(sim.entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
        }

        System.out.printf("Slow speed turn: final pos=(%.0f, %.0f) speed=%.1f hdg=%.0f%n",
                sim.entity.x(), sim.entity.y(), sim.entity.speed(),
                Math.toDegrees(sim.entity.heading()));
        assertTrue(sim.entity.x() > 50, "Should have turned east, x=" + sim.entity.x());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 11: Opposite heading, close waypoint ───────────────────

    @Test
    void closeWaypointBehind() {
        // Sub heading north, waypoint 500m behind (south). Short U-turn.
        var sim = createSub(0, 0, -200, 0);
        setWaypointAndRun(sim, 0, -500, -200, 50 * 120);

        double dist = distanceTo(sim.entity, 0, -500);
        System.out.printf("Close behind: final pos=(%.0f, %.0f) dist=%.0fm hdg=%.0f%n",
                sim.entity.x(), sim.entity.y(), dist,
                Math.toDegrees(sim.entity.heading()));
        // Sub should have turned around and be heading south
        double heading = sim.entity.heading();
        boolean headingSouth = heading > Math.toRadians(135) && heading < Math.toRadians(225);
        assertTrue(headingSouth || dist < 500,
                "Should be heading south or near goal, hdg=" + Math.toDegrees(heading) + " dist=" + dist);
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Terrain scenarios ──────────────────────────────────────────────

    // Helper: create terrain with a ridge across the path.
    // Deep water (-500m) everywhere except a ridge from (ridgeMinX..ridgeMaxX)
    // at y in (ridgeMinY..ridgeMaxY) with floor at ridgeDepth.
    static GeneratedWorld worldWithRidge(double ridgeMinX, double ridgeMaxX,
                                          double ridgeMinY, double ridgeMaxY,
                                          double ridgeDepth) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                if (wx >= ridgeMinX && wx <= ridgeMaxX
                        && wy >= ridgeMinY && wy <= ridgeMaxY) {
                    data[row * size + col] = ridgeDepth;
                } else {
                    data[row * size + col] = -500;
                }
            }
        }
        var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);
        var config = MatchConfig.withDefaults(0);
        return new GeneratedWorld(config, terrain, java.util.List.of(),
                new CurrentField(java.util.List.of()),
                java.util.List.of(new Vec3(0, -2000, -200), new Vec3(0, 2000, -200)));
    }

    SimSub createSubInWorld(GeneratedWorld w, double x, double y, double z, double heading) {
        var config = w.config();
        var context = new MatchContext(config, w.terrain(), java.util.List.of(), w.currentField());
        var autopilot = new SubmarineAutopilot(context);
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(x, y, z), heading, Color.RED, 1000);
        return new SimSub(entity, autopilot);
    }

    void setWaypointAndRunInWorld(SimSub sim, GeneratedWorld w,
                                   double wpX, double wpY, double wpZ, int ticks) {
        var wp = new StrategicWaypoint(wpX, wpY, wpZ, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 200, -1);
        sim.autopilot.setWaypoints(java.util.List.of(wp),
                sim.entity.x(), sim.entity.y(), sim.entity.z(),
                sim.entity.heading(), sim.entity.speed());

        var statusCounts = new java.util.LinkedHashMap<String, Integer>();
        for (int t = 0; t < ticks; t++) {
            var pose = new Pose(new Vec3(sim.entity.x(), sim.entity.y(), sim.entity.z()),
                    sim.entity.heading(), sim.entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(sim.entity.speed() * Math.sin(sim.entity.heading()),
                             sim.entity.speed() * Math.cos(sim.entity.heading()),
                             sim.entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, sim.entity.hp(), 0);
            var env = new EnvironmentSnapshot(w.terrain(), java.util.List.of(), w.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, java.util.List.of(), java.util.List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            sim.autopilot.tick(input, output);

            // Track status
            String status = sim.autopilot.lastStatus();
            if (status == null || status.isEmpty()) status = "(normal)";
            int fIdx = status.indexOf(" f:");
            String key = fIdx >= 0 ? status.substring(0, fIdx) : status;
            statusCounts.merge(key, 1, Integer::sum);

            sim.entity.setThrottle(output.throttle);
            sim.entity.setRudder(output.rudder);
            sim.entity.setSternPlanes(output.sternPlanes);
            sim.entity.setBallast(output.ballast);
            physics.step(sim.entity, DT, w.terrain(), w.currentField(),
                    w.config().battleArea());
        }

        // Print status breakdown
        int total = statusCounts.values().stream().mapToInt(Integer::intValue).sum();
        System.out.print("  Status: ");
        statusCounts.entrySet().stream()
                .sorted((a, b) -> b.getValue() - a.getValue())
                .limit(5)
                .forEach(e -> System.out.printf("%s=%.0f%% ", e.getKey(),
                        100.0 * e.getValue() / total));
        System.out.println();
    }

    // ── Test 13: Ridge across path (impassable, must route around) ──

    @Test
    void ridgeAcrossPath_impassable() {
        // Deep water everywhere except an island (-50m) blocking the direct path.
        // Sub heading north, waypoint on the other side. Ridge at y=-500..500.
        // NOTE: The A* routes around it, but reactive terrain avoidance may
        // interfere near the ridge. This test verifies the sub at least makes
        // progress and takes no damage.
        var w = worldWithRidge(-5000, 5000, -500, 500, -50);
        var sim = createSubInWorld(w, 0, -1500, -200, 0);

        setWaypointAndRunInWorld(sim, w, 0, 1500, -200, 50 * 300);

        double dist = distanceTo(sim.entity, 0, 1500);
        System.out.printf("Impassable ridge: final pos=(%.0f, %.0f, %.0f) dist=%.0fm hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z(), dist, sim.entity.hp());
        assertTrue(sim.entity.y() > -1200, "Should have made progress north, y=" + sim.entity.y());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 14: Shallow ridge (sub must go over, staying above it) ─

    @Test
    void ridgeAcrossPath_shallow() {
        // Ridge at -200m. Sub starts at -300m. The ridge is navigable but the sub
        // must rise to clear it, then dive back down on the other side.
        var w = worldWithRidge(-5000, 5000, -300, 300, -200);
        var sim = createSubInWorld(w, 0, -1500, -300, 0);

        setWaypointAndRunInWorld(sim, w, 0, 1500, -300, 50 * 300);

        System.out.printf("Shallow ridge: final pos=(%.0f, %.0f, %.0f) hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z(), sim.entity.hp());
        assertTrue(sim.entity.y() > -500, "Should have crossed the ridge, y=" + sim.entity.y());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 15: Narrow gap in ridge ────────────────────────────────

    @Test
    void narrowGapInRidge() {
        // Impassable ridge across the path, but with a 600m gap centered at x=0.
        // Sub should find and route through the gap.
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                boolean inRidge = wy >= -300 && wy <= 300
                        && (wx < -300 || wx > 300); // gap at x=-300..300
                data[row * size + col] = inRidge ? -50 : -500;
            }
        }
        var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);
        var config = MatchConfig.withDefaults(0);
        var w = new GeneratedWorld(config, terrain, java.util.List.of(),
                new CurrentField(java.util.List.of()),
                java.util.List.of(new Vec3(0, -1500, -200), new Vec3(0, 1500, -200)));

        var sim = createSubInWorld(w, 0, -1500, -200, 0);
        setWaypointAndRunInWorld(sim, w, 0, 1500, -200, 50 * 300);

        System.out.printf("Narrow gap: final pos=(%.0f, %.0f, %.0f) hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z(), sim.entity.hp());
        assertTrue(sim.entity.y() > 0, "Should have gone through the gap, y=" + sim.entity.y());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 16: Ridge offset to one side (route should go other way) ─

    @Test
    void ridgeOffsetRight() {
        // Ridge blocks the right (east) side. Sub should route left (west).
        // Ridge from x=-200..5000, y=-500..500. Open water to the west (x < -200).
        var w = worldWithRidge(-200, 5000, -500, 500, -50);
        var sim = createSubInWorld(w, 0, -2000, -200, 0);

        setWaypointAndRunInWorld(sim, w, 0, 2000, -200, 50 * 180);

        System.out.printf("Ridge offset right: final pos=(%.0f, %.0f) hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.hp());
        assertTrue(sim.entity.y() > -1500, "Should have made progress, y=" + sim.entity.y());
        // Sub should have gone west to avoid the ridge
        assertTrue(sim.entity.x() < 0 || sim.entity.y() > 0,
                "Should have routed west around the ridge or crossed it");
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 17: Deep valley between two ridges ─────────────────────

    @Test
    void deepValleyPreferred() {
        // Two ridges (-150m) with a deep valley (-500m) between them.
        // The sub should prefer the deep valley path.
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                // Two ridges at x=-1500..-500 and x=500..1500
                boolean leftRidge = wx >= -1500 && wx <= -500 && wy >= -500 && wy <= 500;
                boolean rightRidge = wx >= 500 && wx <= 1500 && wy >= -500 && wy <= 500;
                if (leftRidge || rightRidge) {
                    data[row * size + col] = -150;
                } else {
                    data[row * size + col] = -500;
                }
            }
        }
        var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);
        var config = MatchConfig.withDefaults(0);
        var w = new GeneratedWorld(config, terrain, java.util.List.of(),
                new CurrentField(java.util.List.of()),
                java.util.List.of(new Vec3(0, -3000, -300), new Vec3(0, 3000, -300)));

        var sim = createSubInWorld(w, 0, -3000, -300, 0);
        setWaypointAndRunInWorld(sim, w, 0, 3000, -300, 50 * 240);

        System.out.printf("Deep valley: final pos=(%.0f, %.0f, %.0f) hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z(), sim.entity.hp());
        assertTrue(sim.entity.y() > -1000, "Should have made progress through valley, y=" + sim.entity.y());
        // The sub should have stayed in the valley (near x=0), not gone over the ridges
        assertTrue(Math.abs(sim.entity.x()) < 800,
                "Should have stayed in the deep valley, x=" + sim.entity.x());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 18: Simple island, waypoint on other side ────────────────

    @Test
    void routeAroundIsland() {
        // Deep water (-500m) with an island (above sea level) blocking the direct path.
        // Sub starts south of the island, waypoint north of the island.
        // The A* planner should route around it.
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                // Island: circle of radius 500m centered at (0, 0)
                boolean onIsland = wx * wx + wy * wy < 500 * 500;
                data[row * size + col] = onIsland ? 5.0 : -500;
            }
        }
        var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);
        var config = MatchConfig.withDefaults(0);
        var w = new GeneratedWorld(config, terrain, java.util.List.of(),
                new CurrentField(java.util.List.of()),
                java.util.List.of(new Vec3(0, -3000, -200), new Vec3(0, 3000, -200)));

        var sim = createSubInWorld(w, 0, -2000, -200, 0);
        setWaypointAndRunInWorld(sim, w, 0, 2000, -200, 50 * 300); // 5 minutes

        double dist = distanceTo(sim.entity, 0, 2000);
        System.out.printf("Route around island: final pos=(%.0f, %.0f, %.0f) dist=%.0fm hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.z(), dist, sim.entity.hp());

        // Sub should have gone around the island and be approaching the north side
        assertTrue(sim.entity.y() > 0,
                "Should have navigated past the island center, y=" + sim.entity.y());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 19: Island offset to side, should route through open water ─

    @Test
    void routeAroundOffsetIsland() {
        // Island offset to the east. Sub heading north, waypoint north.
        // The direct path is clear to the west of the island.
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                // Island at (800, 0), radius 500m
                boolean onIsland = Math.pow(wx - 800, 2) + wy * wy < 500 * 500;
                data[row * size + col] = onIsland ? 5.0 : -500;
            }
        }
        var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);
        var config = MatchConfig.withDefaults(0);
        var w = new GeneratedWorld(config, terrain, java.util.List.of(),
                new CurrentField(java.util.List.of()),
                java.util.List.of(new Vec3(0, -3000, -200), new Vec3(0, 3000, -200)));

        var sim = createSubInWorld(w, 0, -2000, -200, 0);
        setWaypointAndRunInWorld(sim, w, 0, 2000, -200, 50 * 300);

        System.out.printf("Offset island: final pos=(%.0f, %.0f) hp=%d%n",
                sim.entity.x(), sim.entity.y(), sim.entity.hp());
        // Should go straight north (island is to the east, not blocking)
        assertTrue(sim.entity.y() > 500,
                "Should have made good progress north, y=" + sim.entity.y());
        assertEquals(1000, sim.entity.hp(), "Should take no damage");
    }

    // ── Test 12: No overshoot/U-turn ─────────────────────────────────

    @Test
    void noOscillationNearWaypoint() {
        // Sub heading north, waypoint 1km ahead. Track heading over time.
        // Heading should be roughly constant (no oscillation from overshoot).
        var sim = createSub(0, 0, -200, 0);

        var wp = new StrategicWaypoint(0, 1000, -200, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 200, -1);
        sim.autopilot.setWaypoints(List.of(wp),
                0, 0, -200, 0, 0);

        int reversals = 0;
        double lastRudder = 0;
        for (int t = 0; t < 50 * 60; t++) {
            var pose = new Pose(new Vec3(sim.entity.x(), sim.entity.y(), sim.entity.z()),
                    sim.entity.heading(), sim.entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(sim.entity.speed() * Math.sin(sim.entity.heading()),
                             sim.entity.speed() * Math.cos(sim.entity.heading()),
                             sim.entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, sim.entity.hp(), 0);
            var env = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            sim.autopilot.tick(input, output);

            // Count rudder reversals (sign changes) as proxy for oscillation
            if (t > 50 * 10 && Math.abs(output.rudder) > 0.05) {
                if (lastRudder != 0 && Math.signum(output.rudder) != Math.signum(lastRudder)) {
                    reversals++;
                }
                lastRudder = output.rudder;
            }

            sim.entity.setThrottle(output.throttle);
            sim.entity.setRudder(output.rudder);
            sim.entity.setSternPlanes(output.sternPlanes);
            sim.entity.setBallast(output.ballast);
            physics.step(sim.entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
        }

        System.out.printf("Oscillation test: %d rudder reversals%n", reversals);
        assertTrue(reversals < 20,
                "Should not oscillate excessively, had " + reversals + " rudder reversals");
    }
}
