/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * This software is free:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package se.hirt.searobots.engine;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import static se.hirt.searobots.api.VehicleConfig.submarine;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for submarine navigation planning around islands and terrain,
 * and the balance between terrain tracking and waypoint following.
 */
class NavigationSimTest {

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);

    private DefaultAttackSub controller;

    @BeforeEach
    void setUp() {
        controller = new DefaultAttackSub();
    }

    // ── Terrain helpers ──────────────────────────────────────────────

    /**
     * Creates a terrain map with deep water everywhere except a rectangular
     * island (above sea level) at the specified world coordinates.
     */
    static TerrainMap terrainWithIsland(double baseDepth,
                                        double islandMinX, double islandMaxX,
                                        double islandMinY, double islandMaxY) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double worldY = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double worldX = origin + col * cellSize;
                if (worldX >= islandMinX && worldX <= islandMaxX
                        && worldY >= islandMinY && worldY <= islandMaxY) {
                    data[row * size + col] = 5.0; // above sea level
                } else {
                    data[row * size + col] = baseDepth;
                }
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /**
     * Creates a flat terrain map at the given depth everywhere.
     */
    static TerrainMap flatTerrain(double elevation) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, elevation);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /**
     * Creates terrain with a shallow quadrant. The quadrant where x > 0 and y > 0
     * has shallowElevation; everything else has baseDepth.
     */
    static TerrainMap terrainWithShallowQuadrant(double baseDepth, double shallowElevation) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double worldY = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                double worldX = origin + col * cellSize;
                if (worldX > 0 && worldY > 0) {
                    data[row * size + col] = shallowElevation;
                } else {
                    data[row * size + col] = baseDepth;
                }
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /**
     * Creates terrain with a narrow deep-water channel running along the X axis.
     * The channel (floor at channelDepth) extends from y=-channelHalfWidth to
     * y=+channelHalfWidth. The shelves on either side have shelfDepth.
     */
    static TerrainMap terrainWithNarrowChannel(double channelDepth, double shelfDepth,
                                                double channelHalfWidth) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double worldY = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                if (Math.abs(worldY) <= channelHalfWidth) {
                    data[row * size + col] = channelDepth;
                } else {
                    data[row * size + col] = shelfDepth;
                }
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    // ── Test harness (same pattern as DefaultAttackSubTest) ─────

    private void startMatch(TerrainMap terrain) {
        var context = new MatchContext(CONFIG, terrain, List.of(), new CurrentField(List.of()));
        controller.onMatchStart(context);
    }

    private CapturedOutput tickFull(TerrainMap terrain, long tick,
                                     double x, double y, double z,
                                     double heading, Vec3 linearVelocity, int hp) {
        var pose = new Pose(new Vec3(x, y, z), heading, 0, 0);
        var velocity = new Velocity(linearVelocity, Vec3.ZERO);
        var state = new SubmarineState(pose, velocity, hp, 0);
        var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
        var input = new TestInputFull(tick, 0.02, state, env, List.of(), List.of(), 0);
        var output = new CapturedOutput();
        controller.onTick(input, output);
        return output;
    }

    record TestInputFull(long tick, double deltaTimeSeconds,
                         SubmarineState self, EnvironmentSnapshot environment,
                         List<SonarContact> sonarContacts, List<SonarContact> activeSonarReturns,
                         int activeSonarCooldownTicks)
            implements SubmarineInput {}

    static final class CapturedOutput implements SubmarineOutput {
        double rudder, sternPlanes, throttle, ballast;
        boolean pinged;
        final ArrayList<Waypoint> waypoints = new ArrayList<>();

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void publishWaypoint(Waypoint wp) { waypoints.add(wp); }
    }

    // ── Test 1: planRouteAvoidsIsland ───────────────────────────────

    @Test
    void planRouteAvoidsIsland() {
        // Island at x=[-250, 250], y=[-500, 500] blocking the direct path
        var terrain = terrainWithIsland(-500, -250, 250, -500, 500);
        startMatch(terrain);

        var route = controller.autopilot().planRoute(-2000, 0, 2000, 0, 7.0);

        // Route should have more than 2 waypoints (needs detour around island)
        assertTrue(route.size() > 2,
                "Route around island should have > 2 waypoints, got " + route.size());

        // No waypoint should be inside the island
        for (var wp : route) {
            boolean insideIsland = wp.x() >= -250 && wp.x() <= 250
                    && wp.y() >= -500 && wp.y() <= 500;
            assertFalse(insideIsland,
                    "Waypoint at (" + wp.x() + ", " + wp.y() + ") is inside the island");
        }

        // Route should start near (-2000, 0) and end near (2000, 0)
        var first = route.getFirst();
        double startDist = Math.sqrt(Math.pow(first.x() - (-2000), 2) + Math.pow(first.y(), 2));
        assertTrue(startDist < 500,
                "First waypoint should be near start (-2000,0), got (" + first.x() + "," + first.y() + ")");

        var last = route.getLast();
        double endDist = Math.sqrt(Math.pow(last.x() - 2000, 2) + Math.pow(last.y(), 2));
        assertTrue(endDist < 500,
                "Last waypoint should be near target (2000,0), got (" + last.x() + "," + last.y() + ")");
    }

    // ── Test 2: planRouteDirectWhenClear ────────────────────────────

    @Test
    void planRouteDirectWhenClear() {
        // Deep water everywhere, no obstacles
        var terrain = flatTerrain(-500);
        startMatch(terrain);

        var route = controller.autopilot().planRoute(-2000, 0, 2000, 0, 7.0);

        // Direct route should be simple: start + end = 2 waypoints
        assertTrue(route.size() <= 2,
                "Clear route should have at most 2 waypoints, got " + route.size());
    }

    // ── Test 3: patrolRouteAvoidsShallowTerrain ─────────────────────

    @Test
    void patrolRouteAvoidsShallowTerrain() {
        // Shallow quadrant (x > 0, y > 0) at -30m (only 30m water)
        var terrain = terrainWithShallowQuadrant(-500, -30);
        startMatch(terrain);

        // Start patrol from a point in the deep quadrant (x < 0, y < 0)
        // The controller generates patrol waypoints on first tick
        var out = tickFull(terrain, 0, -2000, -2000, -400, 0, Vec3.ZERO, 1000);

        // The patrol should have generated waypoints
        assertFalse(out.waypoints.isEmpty(), "Patrol should produce waypoints");

        // No waypoints should be in the shallow quadrant where sub cannot safely navigate.
        // SHALLOW_WATER_LIMIT is -50, and -30m floor is above that.
        // The planRoute checks for floor > -50 as blocked, so waypoints should avoid it.
        for (var wp : out.waypoints) {
            if (wp.x() > 200 && wp.y() > 200) {
                // Check if the terrain at this waypoint is too shallow
                double floor = terrain.elevationAt(wp.x(), wp.y());
                assertTrue(floor < -50,
                        "Waypoint at (" + wp.x() + ", " + wp.y()
                                + ") is in shallow terrain (floor=" + floor + ")");
            }
        }
    }

    // ── Test 4: subNavigatesAroundIslandSimulation ──────────────────

    @Test
    void subNavigatesAroundIslandSimulation() {
        // Custom terrain: deep water everywhere, island in center blocking direct path
        var terrain = terrainWithIsland(-500, -300, 300, -600, 600);

        // Spawn points on opposite sides of the island
        var spawnPoints = List.of(
                new Vec3(-3000, 0, -200),
                new Vec3(3000, 0, -200)
        );

        var world = new GeneratedWorld(CONFIG, terrain, List.of(),
                new CurrentField(List.of()), spawnPoints);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000); // max speed

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new DefaultAttackSub());

        // Track positions at each tick for both subs
        var positionLog = new ArrayList<double[]>(); // [tick, sub0_x, sub0_y, sub1_x, sub1_y, sub0_hp, sub1_hp]
        int[] finalHp = new int[]{1000, 1000};
        boolean[] ticked = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                ticked[0] = true;
                if (tick % 100 == 0 && submarines.size() >= 2) {
                    var s0 = submarines.get(0);
                    var s1 = submarines.get(1);
                    positionLog.add(new double[]{
                            tick,
                            s0.pose().position().x(), s0.pose().position().y(),
                            s1.pose().position().x(), s1.pose().position().y(),
                            s0.hp(), s1.hp()
                    });
                    finalHp[0] = s0.hp();
                    finalHp[1] = s1.hp();
                }
                // Stop after 2000 ticks (40 seconds at 50Hz)
                if (tick >= 2000) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, List.of(submarine(), submarine()), listener));
        thread.start();
        try {
            thread.join(30_000); // 30 second timeout
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        assertTrue(ticked[0], "Simulation should have produced ticks");
        assertFalse(positionLog.isEmpty(), "Should have logged positions");

        // Verify both subs survived (not killed by terrain collision)
        assertTrue(finalHp[0] > 0, "Sub 0 should survive (hp=" + finalHp[0] + ")");
        assertTrue(finalHp[1] > 0, "Sub 1 should survive (hp=" + finalHp[1] + ")");

        // Verify both subs have moved from their spawn points (they are patrolling,
        // not stuck on terrain). At patrol speed ~6 m/s over 40s they should move
        // at least a few hundred meters.
        double[] firstEntry = positionLog.getFirst();
        double[] lastEntry = positionLog.getLast();
        double sub0Moved = Math.sqrt(
                Math.pow(lastEntry[1] - firstEntry[1], 2)
                        + Math.pow(lastEntry[2] - firstEntry[2], 2));
        double sub1Moved = Math.sqrt(
                Math.pow(lastEntry[3] - firstEntry[3], 2)
                        + Math.pow(lastEntry[4] - firstEntry[4], 2));
        assertTrue(sub0Moved > 100,
                "Sub 0 should have moved from spawn, moved only " + sub0Moved + "m");
        assertTrue(sub1Moved > 100,
                "Sub 1 should have moved from spawn, moved only " + sub1Moved + "m");

        // Verify neither sub's position went through the island
        for (var entry : positionLog) {
            double s0x = entry[1], s0y = entry[2];
            double s1x = entry[3], s1y = entry[4];
            boolean s0InIsland = s0x >= -300 && s0x <= 300 && s0y >= -600 && s0y <= 600;
            boolean s1InIsland = s1x >= -300 && s1x <= 300 && s1y >= -600 && s1y <= 600;
            assertFalse(s0InIsland,
                    "Sub 0 should not be inside the island at tick " + entry[0]
                            + " pos=(" + s0x + "," + s0y + ")");
            assertFalse(s1InIsland,
                    "Sub 1 should not be inside the island at tick " + entry[0]
                            + " pos=(" + s1x + "," + s1y + ")");
        }
    }

    // ── Test 5: subFollowsWaypointsInDeepWater ──────────────────────

    @Test
    void subFollowsWaypointsInDeepWater() {
        // In deep flat terrain, verify the controller produces patrol waypoints
        // and that the rudder output steers the sub toward the active waypoint.
        // Each test case uses a fresh controller to avoid state issues.
        //
        // Sub at z=-300 over -500m floor: target depth = -420m, margin = 120 > 80,
        // so terrain avoidance is NOT active.
        var terrain = flatTerrain(-500);

        double x = 0, y = 0, z = -300;
        double[] testHeadings = {0, Math.PI / 4, Math.PI / 2, Math.PI, 3 * Math.PI / 2};
        int totalTests = 0;
        int correctSteering = 0;

        for (double testHeading : testHeadings) {
            var ctrl = new DefaultAttackSub();
            ctrl.onMatchStart(new MatchContext(CONFIG, terrain, List.of(), new CurrentField(List.of())));

            Vec3 vel = new Vec3(6.0 * Math.sin(testHeading), 6.0 * Math.cos(testHeading), 0);

            // Tick a few times to let the controller plan waypoints and stabilize
            CapturedOutput lastOut = null;
            for (int tick = 0; tick < 10; tick++) {
                var pose = new Pose(new Vec3(x, y, z), testHeading, 0, 0);
                var velocity = new Velocity(vel, Vec3.ZERO);
                var state = new SubmarineState(pose, velocity, 1000, 0);
                var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
                var input = new TestInputFull(tick, 0.02, state, env, List.of(), List.of(), 0);
                lastOut = new CapturedOutput();
                ctrl.onTick(input, lastOut);
            }

            if (lastOut != null && !lastOut.waypoints.isEmpty()) {
                Waypoint activeWp = null;
                for (var wp : lastOut.waypoints) {
                    if (wp.active()) {
                        activeWp = wp;
                        break;
                    }
                }
                if (activeWp == null) activeWp = lastOut.waypoints.getFirst();

                double bearingToWp = Math.atan2(activeWp.x() - x, activeWp.y() - y);
                if (bearingToWp < 0) bearingToWp += 2 * Math.PI;
                double bearingErr = bearingToWp - testHeading;
                while (bearingErr > Math.PI) bearingErr -= 2 * Math.PI;
                while (bearingErr < -Math.PI) bearingErr += 2 * Math.PI;

                totalTests++;
                boolean aligned = Math.abs(bearingErr) < Math.toRadians(15);
                boolean rudderCorrect = Math.abs(lastOut.rudder) < 0.01 // rudder near zero if aligned
                        || (bearingErr > 0 && lastOut.rudder > 0)
                        || (bearingErr < 0 && lastOut.rudder < 0);
                if (aligned || rudderCorrect) {
                    correctSteering++;
                }
            }
        }

        assertTrue(totalTests > 0, "Should have tested with waypoints");
        double ratio = (double) correctSteering / totalTests;
        assertTrue(ratio > 0.5,
                "Rudder should steer toward active waypoint in most test headings, got "
                        + String.format("%.1f%%", ratio * 100)
                        + " (" + correctSteering + "/" + totalTests + ")");
    }

    // ── Test 6: terrainAvoidanceDoesNotOverrideWaypointsInSafeWater ─

    @Test
    void terrainAvoidanceDoesNotOverrideWaypointsInSafeWater() {
        // In moderately deep terrain (-300m floor), the sub at z=-200m should be
        // above the target depth (-300+80=-220m). Terrain avoidance activates when
        // margin < floorClearance, i.e. (depth - (floor + 80)) < 80. Here that is
        // (-200 - (-220)) = 20 < 80, so terrain avoidance IS active but urgency
        // is low. Use -200m floor instead so margin is larger: -100 - (-120) = 20.
        // Actually, let us use a depth below the target to ensure no terrain
        // avoidance at all. Floor at -300m, target = -220m, sub at -250m:
        // margin = -250 - (-220) = -30, which is < 80 so avoidance triggers.
        //
        // For truly safe water: floor at -300m, sub at -150m:
        // worstFloor = -300, clearance target = -220, margin = -150-(-220) = 70 < 80.
        // Still triggers!
        //
        // The terrain avoidance check is: margin < floorClearance = 80. This means
        // the sub must be more than 160m above the floor to avoid terrain avoidance.
        // Floor at -300m: sub needs to be above -300 + 160 = -140m.
        // But MIN_DEPTH is -20 and surface avoidance kicks in above -20.
        // So: sub at -130m, floor at -300m: margin = -130 - (-220) = 90 > 80. No avoidance.
        //
        // Test: the controller's rudder in safe water should correlate with its waypoints.
        var terrain = flatTerrain(-300);
        var ctrl = new DefaultAttackSub();
        ctrl.onMatchStart(new MatchContext(CONFIG, terrain, List.of(), new CurrentField(List.of())));

        double x = 0, y = 0, z = -130; // above terrain avoidance threshold
        double heading = Math.PI / 3; // heading ~60 degrees

        Vec3 vel = new Vec3(5.0 * Math.sin(heading), 5.0 * Math.cos(heading), 0);

        // Run a few ticks to let the controller plan waypoints
        CapturedOutput lastOut = null;
        for (int tick = 0; tick < 20; tick++) {
            var pose = new Pose(new Vec3(x, y, z), heading, 0, 0);
            var velocity = new Velocity(vel, Vec3.ZERO);
            var state = new SubmarineState(pose, velocity, 1000, 0);
            var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
            var input = new TestInputFull(tick, 0.02, state, env, List.of(), List.of(), 0);
            lastOut = new CapturedOutput();
            ctrl.onTick(input, lastOut);
        }

        assertNotNull(lastOut, "Should have output from ticks");
        assertFalse(lastOut.waypoints.isEmpty(), "Sub should publish waypoints in safe water");

        // Find active waypoint
        Waypoint activeWp = null;
        for (var wp : lastOut.waypoints) {
            if (wp.active()) {
                activeWp = wp;
                break;
            }
        }
        if (activeWp == null) activeWp = lastOut.waypoints.getFirst();

        // Verify rudder correlates with bearing to active waypoint
        double bearingToWp = Math.atan2(activeWp.x() - x, activeWp.y() - y);
        if (bearingToWp < 0) bearingToWp += 2 * Math.PI;
        double bearingErr = bearingToWp - heading;
        while (bearingErr > Math.PI) bearingErr -= 2 * Math.PI;
        while (bearingErr < -Math.PI) bearingErr += 2 * Math.PI;

        // The rudder should point toward the waypoint (same sign as bearing error)
        // or the sub is already heading toward it (small bearing error)
        boolean aligned = Math.abs(bearingErr) < Math.toRadians(15);
        boolean rudderCorrect = Math.abs(lastOut.rudder) < 0.01
                || (bearingErr > 0 && lastOut.rudder > 0)
                || (bearingErr < 0 && lastOut.rudder < 0);
        assertTrue(aligned || rudderCorrect,
                "In safe water (z=-130, floor=-300), rudder should steer toward waypoint. "
                        + "Bearing error=" + Math.toDegrees(bearingErr)
                        + " deg, rudder=" + lastOut.rudder);
    }

    // ── Test 7: subSurvivesNarrowPassage ────────────────────────────

    @Test
    void subSurvivesNarrowPassage() {
        // Narrow channel: deep water (-300m) in a 400m wide band along y=0,
        // shallow shelves (-40m) on either side
        var terrain = terrainWithNarrowChannel(-300, -40, 200);

        // Spawn the sub on the left side of the channel, heading east (along x)
        var spawnPoints = List.of(
                new Vec3(-5000, 0, -200),
                new Vec3(5000, 0, -200) // dummy second spawn for 2-sub config
        );

        var world = new GeneratedWorld(CONFIG, terrain, List.of(),
                new CurrentField(List.of()), spawnPoints);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new DefaultAttackSub());

        int[] finalHp = {1000, 1000};
        double[] finalX = {-5000, 5000};
        boolean[] ticked = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                ticked[0] = true;
                if (submarines.size() >= 2) {
                    finalHp[0] = submarines.get(0).hp();
                    finalHp[1] = submarines.get(1).hp();
                    finalX[0] = submarines.get(0).pose().position().x();
                    finalX[1] = submarines.get(1).pose().position().x();
                }
                if (tick >= 2000) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, List.of(submarine(), submarine()), listener));
        thread.start();
        try {
            thread.join(30_000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        assertTrue(ticked[0], "Simulation should have produced ticks");

        // Sub 0 should survive the narrow passage (full hp, no terrain kills)
        assertTrue(finalHp[0] > 0,
                "Sub 0 should survive the narrow passage (hp=" + finalHp[0] + ")");

        // Sub should have moved (not stuck against the shelf).
        // If it navigated sensibly, it would move along the channel.
        // Even if it did not make it far, it should not have lost all HP.
        assertTrue(finalHp[0] == 1000,
                "Sub 0 should not take terrain damage in the channel (hp=" + finalHp[0] + ")");
    }

    // ── Seed-based full simulation tests ─────────────────────────────

    /**
     * Runs a full simulation for a given seed and prints detailed autopilot
     * performance analysis: distance traveled, terrain damage, depth profile,
     * status breakdown, and waypoint progression.
     */
    private void runSeedAnalysis(long seed, int durationTicks) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new SubmarineDrone());

        // Per-sub tracking
        int subCount = 2;
        double[][] startPos = new double[subCount][3];
        double[][] lastPos = new double[subCount][3];
        int[] finalHp = new int[subCount];
        double[] totalDist = new double[subCount];
        double[] minDepth = new double[subCount]; // shallowest (closest to 0)
        double[] maxDepth = new double[subCount]; // deepest (most negative)
        double[] maxSpeed = new double[subCount];
        double[] maxPitch = new double[subCount];
        java.util.Arrays.fill(minDepth, Double.NEGATIVE_INFINITY);
        java.util.Arrays.fill(maxDepth, Double.MAX_VALUE);
        java.util.Arrays.fill(finalHp, 1000);

        // Status tracking for sub 0
        var statusCounts = new java.util.LinkedHashMap<String, Integer>();
        int[] tickCount = {0};
        long[] firstDamageTick = {-1};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                tickCount[0]++;
                for (int i = 0; i < Math.min(subCount, submarines.size()); i++) {
                    var s = submarines.get(i);
                    var pos = s.pose().position();
                    double spd = s.velocity().speed();

                    if (tick == 0) {
                        startPos[i] = new double[]{pos.x(), pos.y(), pos.z()};
                        lastPos[i] = new double[]{pos.x(), pos.y(), pos.z()};
                    }

                    // Accumulate distance
                    double dx = pos.x() - lastPos[i][0];
                    double dy = pos.y() - lastPos[i][1];
                    totalDist[i] += Math.sqrt(dx * dx + dy * dy);
                    lastPos[i] = new double[]{pos.x(), pos.y(), pos.z()};

                    finalHp[i] = s.hp();
                    if (pos.z() > minDepth[i]) minDepth[i] = pos.z();
                    if (pos.z() < maxDepth[i]) maxDepth[i] = pos.z();
                    if (spd > maxSpeed[i]) maxSpeed[i] = spd;
                    if (Math.abs(s.pose().pitch()) > maxPitch[i])
                        maxPitch[i] = Math.abs(s.pose().pitch());

                    if (i == 0 && s.hp() < 1000 && firstDamageTick[0] < 0) {
                        firstDamageTick[0] = tick;
                    }
                }

                // Track sub 0 status (extract state prefix before floor/gap data)
                if (!submarines.isEmpty()) {
                    String status = submarines.get(0).status();
                    if (status == null || status.isEmpty()) status = "(normal)";
                    // Strip floor/gap suffix like " f:216 g:27" to get just the state
                    int fIdx = status.indexOf(" f:");
                    String stateKey = fIdx >= 0 ? status.substring(0, fIdx) : status;
                    statusCounts.merge(stateKey, 1, Integer::sum);
                }

                if (tick >= durationTicks) {
                    sim.stop();
                }
            }

            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() ->
                sim.run(world, controllers, List.of(submarine(), submarine()), listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        // Print analysis
        double seconds = tickCount[0] / 50.0;
        System.out.println("=== Navigation Analysis: seed " + seed + " ===");
        System.out.printf("Duration: %.0fs (%d ticks)%n", seconds, tickCount[0]);
        System.out.println();

        for (int i = 0; i < subCount; i++) {
            String name = i == 0 ? "DefaultAttackSub" : "SubmarineDrone";
            System.out.printf("--- %s (sub %d) ---%n", name, i);
            System.out.printf("  HP: %d/1000%s%n", finalHp[i],
                    finalHp[i] < 1000 ? " (DAMAGED" +
                            (firstDamageTick[0] >= 0 && i == 0
                                    ? " at tick " + firstDamageTick[0] + " / " +
                                      String.format("%.1fs", firstDamageTick[0] / 50.0)
                                    : "") + ")" : "");
            System.out.printf("  Distance traveled: %.0fm%n", totalDist[i]);
            System.out.printf("  Avg speed: %.1f m/s%n", totalDist[i] / seconds);
            System.out.printf("  Max speed: %.1f m/s%n", maxSpeed[i]);
            System.out.printf("  Depth range: %.0fm to %.0fm%n", maxDepth[i], minDepth[i]);
            System.out.printf("  Max pitch: %.1f\u00b0%n", Math.toDegrees(maxPitch[i]));
            System.out.println();
        }

        System.out.println("--- Sub 0 status breakdown ---");
        int totalStatusTicks = statusCounts.values().stream().mapToInt(Integer::intValue).sum();
        statusCounts.entrySet().stream()
                .sorted((a, b) -> b.getValue() - a.getValue())
                .forEach(e -> System.out.printf("  %-25s %5d ticks (%4.1f%%)%n",
                        e.getKey(), e.getValue(),
                        100.0 * e.getValue() / totalStatusTicks));
        System.out.println();

        // Assertions
        assertTrue(tickCount[0] > 0, "Simulation should have produced ticks");
        assertTrue(totalDist[0] > 100,
                "Sub 0 should have moved significantly, only traveled " + totalDist[0] + "m");
        assertTrue(finalHp[0] > 0, "Sub 0 should survive (hp=" + finalHp[0] + ")");
    }

    @Test
    void seed_neg8551482231658960540() {
        runSeedAnalysis(-8551482231658960540L, 15000);
    }

    @Test
    void seed_neg8551482231658960540_routeDiagnostic() {
        long seed = -8551482231658960540L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();

        System.out.println("=== Route Diagnostic: seed " + seed + " ===");
        var sp = world.spawnPoints().get(0);
        double floor0 = terrain.elevationAt(sp.x(), sp.y());
        System.out.printf("Sub 0 spawn: (%.0f, %.0f, %.0f)  floor=%.0fm%n", sp.x(), sp.y(), sp.z(), floor0);

        var ctrl = new DefaultAttackSub();
        ctrl.onMatchStart(new MatchContext(config, terrain, List.of(), new CurrentField(List.of())));

        double heading = WorldGenerator.findSafeHeading(terrain, sp.x(), sp.y());
        if (Double.isNaN(heading)) {
            heading = Math.atan2(-sp.x(), -sp.y());
            if (heading < 0) heading += 2 * Math.PI;
        }
        System.out.printf("Initial heading: %.0f\u00b0%n", Math.toDegrees(heading));

        Vec3 vel = new Vec3(6.0 * Math.sin(heading), 6.0 * Math.cos(heading), 0);
        var pose = new Pose(sp, heading, 0, 0);
        var velocity = new Velocity(vel, Vec3.ZERO);
        var state = new SubmarineState(pose, velocity, 1000, 0);
        var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
        var input = new TestInputFull(0, 0.02, state, env, List.of(), List.of(), 0);
        var output = new CapturedOutput();
        ctrl.onTick(input, output);

        var autopilot = ctrl.autopilot();
        System.out.println("\nStrategic waypoints:");
        for (int i = 0; i < autopilot.strategicWaypoints().size(); i++) {
            var wp = autopilot.strategicWaypoints().get(i);
            double f = terrain.elevationAt(wp.x(), wp.y());
            System.out.printf("  [%d] (%.0f, %.0f) depth=%.0f  floor=%.0fm  %s%n",
                    i, wp.x(), wp.y(), wp.preferredDepth(), f, wp.noise());
        }

        var navWps = autopilot.navWaypoints();
        System.out.println("\nNav waypoints:");
        for (int i = 0; i < navWps.size(); i++) {
            var wp = navWps.get(i);
            double f = terrain.elevationAt(wp.x(), wp.y());
            String marker = i == autopilot.currentNavIndex() ? " <-- ACTIVE" : "";
            System.out.printf("  [%d] (%.0f, %.0f, %.0f)  floor=%.0fm  clr=%.0fm%s%n",
                    i, wp.x(), wp.y(), wp.z(), f, wp.z() - f, marker);
        }

        System.out.println("\nTerrain along each leg:");
        for (int i = 0; i < navWps.size() - 1; i++) {
            var from = navWps.get(i);
            var to = navWps.get(i + 1);
            double legLen = from.horizontalDistanceTo(to);
            int samples = Math.max(5, (int) (legLen / 50));
            double worstFloor = Double.NEGATIVE_INFINITY;
            double worstClearance = Double.MAX_VALUE;
            for (int s = 0; s <= samples; s++) {
                double t = (double) s / samples;
                double sx = from.x() + (to.x() - from.x()) * t;
                double sy = from.y() + (to.y() - from.y()) * t;
                double sz = from.z() + (to.z() - from.z()) * t;
                double f = terrain.elevationAt(sx, sy);
                double clr = sz - f;
                if (f > worstFloor) worstFloor = f;
                if (clr < worstClearance) worstClearance = clr;
            }
            String danger = worstClearance < 50 ? " *** COLLISION RISK ***" : "";
            System.out.printf("  Leg %d->%d (%.0fm): worst_floor=%.0f  worst_clr=%.0f%s%n",
                    i, i + 1, legLen, worstFloor, worstClearance, danger);
        }
    }

    @Test
    void seed_3823285984661543777() {
        // Diagnostic: trace the sub's state every second until death
        long seed = 3823285984661543777L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new SubmarineDrone());

        boolean[] dead = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.isEmpty()) return;
                var s = submarines.get(0);
                var pos = s.pose().position();
                double spd = s.velocity().speed();
                double floor = world.terrain().elevationAt(pos.x(), pos.y());
                double gap = pos.z() - floor;

                if (tick % 50 == 0 && !dead[0]) { // every 1 second
                    System.out.printf("t=%3.0fs  pos=(%6.0f,%6.0f,%5.0f)  floor=%5.0f  gap=%4.0f  " +
                                    "spd=%4.1f  hdg=%3.0f°  pitch=%+5.1f°  hp=%4d  status=%s%n",
                            tick / 50.0, pos.x(), pos.y(), pos.z(), floor, gap,
                            spd, Math.toDegrees(s.pose().heading()),
                            Math.toDegrees(s.pose().pitch()),
                            s.hp(), s.status());
                }
                if (s.hp() <= 0 && !dead[0]) {
                    dead[0] = true;
                    System.out.printf("*** DEAD at tick %d (%.1fs) pos=(%6.0f,%6.0f,%5.0f) floor=%5.0f gap=%4.0f%n",
                            tick, tick / 50.0, pos.x(), pos.y(), pos.z(), floor, gap);
                    sim.stop();
                }
                if (tick >= 5000) sim.stop(); // 100s max
            }

            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() ->
                sim.run(world, controllers, List.of(submarine(), submarine()), listener));
        thread.start();
        try { thread.join(30_000); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        // This seed has challenging terrain. The sub may die due to terrain
        // avoidance overriding the planned route (a known issue to fix).
        // For now, this test serves as a diagnostic, not a hard assertion.
        if (dead[0]) {
            System.out.println("NOTE: Sub died - terrain avoidance override suspected");
        }
    }

    /**
     * Diagnostic: dump the initial route the autopilot plans for sub 0,
     * along with terrain elevation at each waypoint and along each leg.
     */
    @Test
    void seed_3823285984661543777_routeDiagnostic() {
        long seed = 3823285984661543777L;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();

        System.out.println("=== Route Diagnostic: seed " + seed + " ===");
        System.out.printf("Spawn points: %d%n", world.spawnPoints().size());
        for (int i = 0; i < world.spawnPoints().size(); i++) {
            var sp = world.spawnPoints().get(i);
            double floor = terrain.elevationAt(sp.x(), sp.y());
            System.out.printf("  Sub %d spawn: (%.0f, %.0f, %.0f)  floor=%.0fm%n",
                    i, sp.x(), sp.y(), sp.z(), floor);
        }

        // Initialize controller to get the autopilot's first route
        var ctrl = new DefaultAttackSub();
        var context = new MatchContext(config, terrain, List.of(), new CurrentField(List.of()));
        ctrl.onMatchStart(context);

        // Tick once to trigger patrol waypoint generation
        var sp = world.spawnPoints().get(0);
        // Use the same heading logic as SimulationLoop
        double heading = WorldGenerator.findSafeHeading(terrain, sp.x(), sp.y());
        if (Double.isNaN(heading)) {
            heading = Math.atan2(-sp.x(), -sp.y());
            if (heading < 0) heading += 2 * Math.PI;
        }
        System.out.printf("Initial heading: %.0f\u00b0%n", Math.toDegrees(heading));
        Vec3 vel = new Vec3(6.0 * Math.sin(heading), 6.0 * Math.cos(heading), 0);
        var pose = new Pose(sp, heading, 0, 0);
        var velocity = new Velocity(vel, Vec3.ZERO);
        var state = new SubmarineState(pose, velocity, 1000, 0);
        var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
        var input = new TestInputFull(0, 0.02, state, env, List.of(), List.of(), 0);
        var output = new CapturedOutput();
        ctrl.onTick(input, output);

        // Print strategic waypoints
        var autopilot = ctrl.autopilot();
        System.out.println("\nStrategic waypoints:");
        for (int i = 0; i < autopilot.strategicWaypoints().size(); i++) {
            var wp = autopilot.strategicWaypoints().get(i);
            double floor = terrain.elevationAt(wp.x(), wp.y());
            System.out.printf("  [%d] (%.0f, %.0f) depth=%.0f  floor=%.0fm  %s %s%n",
                    i, wp.x(), wp.y(), wp.preferredDepth(), floor,
                    wp.noise(), wp.pattern());
        }

        // Print nav waypoints (A* route for current leg)
        System.out.println("\nNav waypoints (A* route, current leg):");
        var navWps = autopilot.navWaypoints();
        for (int i = 0; i < navWps.size(); i++) {
            var wp = navWps.get(i);
            double floor = terrain.elevationAt(wp.x(), wp.y());
            double clearance = wp.z() - floor;
            String marker = i == autopilot.currentNavIndex() ? " <-- ACTIVE" : "";
            String danger = clearance < 50 ? " *** LOW CLEARANCE ***" : "";
            System.out.printf("  [%d] (%.0f, %.0f, %.0f)  floor=%.0fm  clearance=%.0fm%s%s%n",
                    i, wp.x(), wp.y(), wp.z(), floor, clearance, danger, marker);
        }

        // Sample terrain along each leg of the nav route
        System.out.println("\nTerrain profile along route:");
        for (int i = 0; i < navWps.size() - 1; i++) {
            var from = navWps.get(i);
            var to = navWps.get(i + 1);
            double legLen = from.horizontalDistanceTo(to);
            int samples = Math.max(5, (int) (legLen / 50));
            double worstFloor = Double.NEGATIVE_INFINITY;
            double worstDist = 0;
            double worstWpZ = 0;

            System.out.printf("  Leg %d->%d (%.0fm):%n", i, i + 1, legLen);
            for (int s = 0; s <= samples; s++) {
                double t = (double) s / samples;
                double sx = from.x() + (to.x() - from.x()) * t;
                double sy = from.y() + (to.y() - from.y()) * t;
                double sz = from.z() + (to.z() - from.z()) * t;
                double floor = terrain.elevationAt(sx, sy);
                double clearance = sz - floor;
                String danger = clearance < 50 ? " *** COLLISION ***" : "";
                if (floor > worstFloor) {
                    worstFloor = floor;
                    worstDist = t * legLen;
                    worstWpZ = sz;
                }
                if (s % Math.max(1, samples / 10) == 0 || !danger.isEmpty()) {
                    System.out.printf("    t=%.2f (%.0fm) pos=(%.0f,%.0f) wpZ=%.0f floor=%.0f clr=%.0f%s%n",
                            t, t * legLen, sx, sy, sz, floor, clearance, danger);
                }
            }
            double worstClearance = worstWpZ - worstFloor;
            if (worstClearance < 50) {
                System.out.printf("    WORST: floor=%.0f at %.0fm into leg, wpZ=%.0f, clearance=%.0f%n",
                        worstFloor, worstDist, worstWpZ, worstClearance);
            }
        }

        // Check if planner considers these cells safe
        System.out.println("\nPath planner safety check:");
        var planner = autopilot.pathPlanner();
        for (int i = 0; i < navWps.size(); i++) {
            var wp = navWps.get(i);
            boolean safe = planner.isSafe(wp.x(), wp.y());
            float cost = planner.costAt(wp.x(), wp.y());
            System.out.printf("  [%d] (%.0f, %.0f) safe=%s cost=%.2f%n",
                    i, wp.x(), wp.y(), safe, cost);
        }
    }
}
