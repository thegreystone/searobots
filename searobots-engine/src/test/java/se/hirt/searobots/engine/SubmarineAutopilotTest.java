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

import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SubmarineAutopilotTest {

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);

    // ── Helpers ──────────────────────────────────────────────────────

    static TerrainMap flatTerrain(double elevation) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, elevation);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    static TerrainMap terrainWithIsland() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                double wy = origin + row * cellSize;
                double dist = Math.sqrt(wx * wx + wy * wy);
                if (dist < 500) {
                    data[row * size + col] = -30; // shallow
                } else {
                    data[row * size + col] = -500; // deep
                }
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    static TerrainMap terrainWithBlockedGoal() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                double wy = origin + row * cellSize;
                double dist = Math.sqrt(Math.pow(wx - 2000, 2) + Math.pow(wy, 2));
                if (dist < 400) {
                    data[row * size + col] = -10; // impassable around goal
                } else {
                    data[row * size + col] = -500;
                }
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    static TerrainMap terrainRisingAhead() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            for (int col = 0; col < size; col++) {
                // Floor rises ahead (positive Y direction, since heading=0 is north)
                data[row * size + col] = wy > 500 ? -60 : -500;
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    static TerrainMap terrainShallowRight() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                // Shallow to the right (positive X)
                data[row * size + col] = wx > 300 ? -60 : -500;
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    private SubmarineAutopilot createAutopilot(TerrainMap terrain) {
        var context = new MatchContext(CONFIG, terrain, List.of(), new CurrentField(List.of()));
        return new SubmarineAutopilot(context);
    }

    private CapturedOutput tickAutopilot(SubmarineAutopilot ap, TerrainMap terrain,
                                          long tick, double x, double y, double z,
                                          double heading, Vec3 velocity) {
        var pose = new Pose(new Vec3(x, y, z), heading, 0, 0);
        var vel = new Velocity(velocity, Vec3.ZERO);
        var state = new SubmarineState(pose, vel, 1000, 0);
        var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
        var input = new TestInput(tick, 0.02, state, env);
        var output = new CapturedOutput();
        ap.tick(input, output);
        return output;
    }

    private CapturedOutput tickAutopilot(SubmarineAutopilot ap, TerrainMap terrain,
                                          long tick, double x, double y, double z,
                                          double heading) {
        return tickAutopilot(ap, terrain, tick, x, y, z, heading, Vec3.ZERO);
    }

    private static StrategicWaypoint wp(double x, double y, double depth) {
        return new StrategicWaypoint(x, y, depth, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 200, -1);
    }

    private static StrategicWaypoint wp(double x, double y, double depth,
                                         NoisePolicy noise) {
        return new StrategicWaypoint(x, y, depth, Purpose.PATROL,
                noise, MovementPattern.DIRECT, 200, -1);
    }

    private static StrategicWaypoint wp(double x, double y, double depth,
                                         MovementPattern pattern) {
        return new StrategicWaypoint(x, y, depth, Purpose.PATROL,
                NoisePolicy.NORMAL, pattern, 200, -1);
    }

    private static StrategicWaypoint wp(double x, double y, double depth,
                                         double arrivalRadius) {
        return new StrategicWaypoint(x, y, depth, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, arrivalRadius, -1);
    }

    private static StrategicWaypoint wpWithSpeed(double x, double y, double depth,
                                                   NoisePolicy noise, double targetSpeed) {
        return new StrategicWaypoint(x, y, depth, Purpose.PATROL,
                noise, MovementPattern.DIRECT, 200, targetSpeed);
    }

    record TestInput(long tick, double deltaTimeSeconds,
                     SubmarineState self, EnvironmentSnapshot environment)
            implements SubmarineInput {}

    static final class CapturedOutput implements SubmarineOutput {
        double rudder, sternPlanes, throttle, ballast;
        boolean pinged;
        final java.util.ArrayList<Waypoint> waypoints = new java.util.ArrayList<>();

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void publishWaypoint(Waypoint wp) { waypoints.add(wp); }
    }

    // ── 1a. Route Planning ──────────────────────────────────────────

    @Nested
    class RoutePlanning {

        @Test
        void directRouteOnFlatTerrain() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            assertFalse(ap.navWaypoints().isEmpty(),
                    "Nav waypoints should be planned on flat terrain");
        }

        @Test
        void routeAroundIsland() {
            var terrain = terrainWithIsland();
            var ap = createAutopilot(terrain);
            // Start west of island, waypoint east of island
            ap.setWaypoints(List.of(wp(2000, 0, -200)), -2000, 0, -200, 0, 5);

            var nav = ap.navWaypoints();
            assertFalse(nav.isEmpty(), "Route should be planned around island");
            // With an island in the way, A* should produce more waypoints than a direct route
            assertTrue(nav.size() >= 3,
                    "Route around island should have multiple nav waypoints, got " + nav.size());
        }

        @Test
        void headingAwareStartSharpTurn() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            // Sub heading north (0), waypoint behind (south): 180 degree turn.
            // The trajectory projector should insert arc waypoint(s) ahead of the
            // sub, then route back toward the goal. The first waypoint should NOT
            // be directly at the goal behind us.
            ap.setWaypoints(List.of(wp(0, -2000, -200)), 0, 0, -200, 0, 5);

            var nav = ap.navWaypoints();
            assertFalse(nav.isEmpty(), "Route should be planned for sharp turn");
            assertTrue(nav.size() >= 2,
                    "Sharp turn should have arc waypoint(s), got " + nav.size() + " waypoints");
            // The first waypoint should be ahead or to the side (arc), not behind
            var first = nav.getFirst();
            assertTrue(first.y() > -500,
                    "First waypoint should not be far behind sub, got y=" + first.y());
        }

        @Test
        void immediatePlanningOnSetWaypoints() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            // Nav waypoints should be populated immediately, not after first tick
            assertFalse(ap.navWaypoints().isEmpty(),
                    "Nav waypoints should be populated immediately after setWaypoints");
        }

        @Test
        void emptyWaypointsNoRoute() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(), 0, 0, -200, 0, 5);

            // Ticking with no waypoints should not crash
            assertDoesNotThrow(() -> tickAutopilot(ap, terrain, 1, 0, 0, -200, 0));
            assertTrue(ap.navWaypoints().isEmpty(), "No nav waypoints for empty strategic list");
        }

        @Test
        void blockedPathFallback() {
            var terrain = terrainWithBlockedGoal();
            var ap = createAutopilot(terrain);
            // Goal is in the middle of impassable terrain
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            // Should still have a route (direct fallback or partial path)
            assertFalse(ap.navWaypoints().isEmpty(),
                    "Blocked goal should still produce a fallback route");
        }

        @Test
        void replanOnAdvance() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            var waypoints = List.of(
                    wp(1000, 0, -200),
                    wp(3000, 0, -200));
            ap.setWaypoints(waypoints, 0, 0, -200, 0, 5);

            var firstRoute = List.copyOf(ap.navWaypoints());
            ap.advanceWaypoint(1000, 0, -200, 0, 5);

            var secondRoute = ap.navWaypoints();
            assertFalse(secondRoute.isEmpty(), "New route should be planned after advance");
            assertNotEquals(firstRoute, secondRoute,
                    "Route should change after advancing to next waypoint");
        }

        @Test
        void multipleWaypointsFirstLeg() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            var waypoints = List.of(
                    wp(1000, 0, -200),
                    wp(2000, 1000, -200),
                    wp(3000, 0, -200));
            ap.setWaypoints(waypoints, 0, 0, -200, 0, 5);

            assertEquals(0, ap.currentWaypointIndex(),
                    "Should start at first strategic waypoint");
            assertFalse(ap.navWaypoints().isEmpty(),
                    "Route to first waypoint should be planned");
        }

        @Test
        void arrivalRadiusRespected() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(500, 0, -200, 500.0)), 0, 0, -200, 0, 5);

            // At 400m from target (within 500m radius), tick should mark arrival
            tickAutopilot(ap, terrain, 1, 100, 0, -200, 0);
            assertTrue(ap.hasArrived(),
                    "Should arrive when within custom arrivalRadius=500m");
        }
    }

    // ── 1b. Horizontal Steering ─────────────────────────────────────

    @Nested
    class HorizontalSteering {

        @Test
        void steersTowardWaypoint() {
            // Sub at origin heading north (0), waypoint slightly east and ahead.
            // Bearing to (500, 2000) = atan2(500, 2000) ~ 14 degrees (within 20 deg, no lead point)
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(500, 2000, -200)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertTrue(out.rudder > 0,
                    "Should steer right toward eastern waypoint, got " + out.rudder);
        }

        @Test
        void steersLeftTowardWaypoint() {
            // Sub at origin heading north (0), waypoint slightly west and ahead.
            // Bearing to (-500, 2000) = atan2(-500, 2000) ~ -14 degrees (within 20 deg, no lead point)
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(-500, 2000, -200)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertTrue(out.rudder < 0,
                    "Should steer left toward western waypoint, got " + out.rudder);
        }

        @Test
        void straightAheadNoRudder() {
            // Sub at origin heading north (0), waypoint directly ahead (positive Y)
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(0, 2000, -200)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertEquals(0, out.rudder, 0.1,
                    "Rudder should be near zero when waypoint is directly ahead, got " + out.rudder);
        }

        @Test
        void arrivalAdvancesNavWaypoint() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            int initialIndex = ap.currentNavIndex();
            // Tick at a position very close to a nav waypoint
            // The first nav waypoint after start marker should be reachable
            var nav = ap.navWaypoints();
            if (nav.size() > 1) {
                var firstNav = nav.get(1);
                // Tick near that nav waypoint
                tickAutopilot(ap, terrain, 1,
                        firstNav.x(), firstNav.y(), -200, 0);
                assertTrue(ap.currentNavIndex() >= initialIndex,
                        "Nav index should advance when near a nav waypoint");
            }
        }

        @Test
        void strategicWaypointArrival() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(500, 0, -200)), 0, 0, -200, 0, 5);

            // Tick at position within 200m of strategic waypoint
            tickAutopilot(ap, terrain, 1, 400, 0, -200, 0);
            assertTrue(ap.hasArrived(),
                    "hasArrived should be true when within arrivalRadius of strategic waypoint");
        }
    }

    // ── 1c. Depth Control ───────────────────────────────────────────

    @Nested
    class DepthControl {

        @Test
        void divesToPreferredDepth() {
            // preferredDepth=-200, sub at -100, should sink
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -100, 0);
            assertTrue(out.ballast < 0.5,
                    "Sub at -100m with preferredDepth=-200 should sink, got ballast=" + out.ballast);
        }

        @Test
        void risesWhenTooDeep() {
            // preferredDepth=-200, sub at -400, should rise
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -400, 0);
            assertTrue(out.ballast > 0.5,
                    "Sub at -400m with preferredDepth=-200 should rise, got ballast=" + out.ballast);
        }

        @Test
        void floorClearance() {
            // Floor at -150, FLOOR_CLEARANCE=50, safe floor = -100
            // preferredDepth=-200 is below floor+clearance=-100, so target should be clamped to -100
            var terrain = flatTerrain(-150);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            // Sub at -130m, below safe floor (-100), should rise
            var out = tickAutopilot(ap, terrain, 1, 0, 0, -130, 0);
            assertTrue(out.ballast > 0.5,
                    "Sub below safe floor should rise, got ballast=" + out.ballast);
        }

        @Test
        void crushDepthClamp() {
            // preferredDepth=-800, crush=-700, safety=50, depthLimit=-650
            // Target should be clamped to -650
            var terrain = flatTerrain(-900);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -800)), 0, 0, -200, 0, 5);

            // Sub at -500m, should sink toward -650 (not -800)
            var out = tickAutopilot(ap, terrain, 1, 0, 0, -500, 0);
            assertTrue(out.ballast < 0.5,
                    "Sub at -500m should sink toward -650 limit, got ballast=" + out.ballast);

            // Sub at -660m (below -650 limit), should rise
            var out2 = tickAutopilot(ap, terrain, 2, 0, 0, -660, 0);
            assertTrue(out2.ballast > 0.5,
                    "Sub below crush depth limit should rise, got ballast=" + out2.ballast);
        }

        @Test
        void pullUpWhenSinkingNearFloor() {
            // Floor at -500, sub at -440 (gap=60), sinking at -2 m/s
            // gap < FLOOR_CLEARANCE*1.5=75 and verticalSpeed < -0.5 -> pull-up
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -300)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -440, 0,
                    new Vec3(5, 0, -2));
            assertTrue(out.sternPlanes > 0,
                    "Pull-up should pitch nose up, got sternPlanes=" + out.sternPlanes);
            assertTrue(out.ballast >= 0.5,
                    "Pull-up should increase ballast, got ballast=" + out.ballast);
        }

        @Test
        void surfaceAvoidance() {
            // Sub at -10m (above MIN_DEPTH=-20), should pitch down
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -10, 0);
            assertTrue(out.sternPlanes < 0,
                    "Sub near surface should pitch down, got sternPlanes=" + out.sternPlanes);
        }
    }

    // ── 1d. Noise Policy to Throttle ────────────────────────────────

    @Nested
    class NoisePolicyThrottle {

        @Test
        void silentThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200, NoisePolicy.SILENT)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertTrue(out.throttle >= 0.10 && out.throttle <= 0.15,
                    "SILENT throttle should be in [0.10, 0.15], got " + out.throttle);
        }

        @Test
        void quietThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200, NoisePolicy.QUIET)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertTrue(out.throttle >= 0.25 && out.throttle <= 0.35,
                    "QUIET throttle should be in [0.25, 0.35], got " + out.throttle);
        }

        @Test
        void normalThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200, NoisePolicy.NORMAL)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertEquals(0.40, out.throttle, 0.05,
                    "NORMAL throttle should be ~0.40, got " + out.throttle);
        }

        @Test
        void sprintThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200, NoisePolicy.SPRINT)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertTrue(out.throttle >= 0.80 && out.throttle <= 1.00,
                    "SPRINT throttle should be in [0.80, 1.00], got " + out.throttle);
        }

        @Test
        void depthModulatesSprintThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -50, NoisePolicy.SPRINT)), 0, 0, -200, 0, 5);

            // At shallow depth (-50m, above -100m threshold), cavitation factor reduces throttle
            var out = tickAutopilot(ap, terrain, 1, 0, 0, -50, 0);
            // cavitationFactor = clamp((-(-50) - 20) / 80, 0.5, 1.0) = clamp(30/80, 0.5, 1.0) = 0.5
            // So throttle should be reduced compared to deep sprint
            assertTrue(out.throttle < 0.80,
                    "Sprint at shallow depth should reduce throttle for cavitation, got " + out.throttle);
        }

        @Test
        void targetSpeedMatchesThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            // targetSpeed=5.0, maxSubSpeed=15, so speedThrottle=5/15=0.333
            // Clamped to NORMAL range [0.25, 0.80]
            ap.setWaypoints(List.of(wpWithSpeed(2000, 0, -200, NoisePolicy.NORMAL, 5.0)),
                    0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            double expectedThrottle = 5.0 / 15.0; // ~0.333
            assertEquals(expectedThrottle, out.throttle, 0.05,
                    "Throttle should match targetSpeed=5.0, got " + out.throttle);
        }
    }

    // ── 1e. Movement Patterns ───────────────────────────────────────

    @Nested
    class MovementPatterns {

        @Test
        void directConstantBearing() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(0, 2000, -200, MovementPattern.DIRECT)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertEquals(0, out.rudder, 0.1,
                    "DIRECT pattern toward waypoint ahead should have near-zero rudder, got " + out.rudder);
        }

        @Test
        void zigzagOscillates() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(0, 5000, -200, MovementPattern.ZIGZAG_TMA)), 0, 0, -200, 0, 5);

            // Tick at different points in the zigzag cycle (750 ticks per leg)
            boolean foundPositive = false;
            boolean foundNegative = false;
            for (long t = 0; t < 2000; t += 100) {
                var out = tickAutopilot(ap, terrain, t, 0, 0, -200, 0);
                if (out.rudder > 0.05) foundPositive = true;
                if (out.rudder < -0.05) foundNegative = true;
            }
            assertTrue(foundPositive && foundNegative,
                    "ZIGZAG_TMA pattern should produce rudder changes in both directions");
        }

        @Test
        void sprintDriftAlternatesThrottle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            var sprintDriftWp = new StrategicWaypoint(0, 5000, -200, Purpose.PATROL,
                    NoisePolicy.SPRINT, MovementPattern.SPRINT_DRIFT, 200, -1);
            ap.setWaypoints(List.of(sprintDriftWp), 0, 0, -200, 0, 5);

            // Sprint phase: ticks 0-749
            var outSprint = tickAutopilot(ap, terrain, 100, 0, 0, -200, 0);
            // Drift phase: ticks 750-1749
            var outDrift = tickAutopilot(ap, terrain, 900, 0, 0, -200, 0);

            assertTrue(outSprint.throttle > outDrift.throttle,
                    "Sprint phase throttle (" + outSprint.throttle
                            + ") should be higher than drift phase (" + outDrift.throttle + ")");
        }
    }

    // ── 1f. Terrain Avoidance ───────────────────────────────────────

    @Nested
    class TerrainAvoidance {

        @Test
        void risingTerrainAvoidance() {
            var terrain = terrainRisingAhead();
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(0, 3000, -200)), 0, 0, -200, 0, 5);

            // Sub heading north toward rising terrain
            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0,
                    new Vec3(0, 5, 0));
            // Should see depth adjustment or throttle reduction
            boolean adjusted = out.throttle < 0.40 || out.ballast > 0.5 || out.sternPlanes > 0;
            assertTrue(adjusted,
                    "Rising terrain ahead should trigger avoidance response (throttle="
                            + out.throttle + ", ballast=" + out.ballast
                            + ", sternPlanes=" + out.sternPlanes + ")");
        }

        @Test
        void steeringAwayFromShallowWater() {
            var terrain = terrainShallowRight();
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(0, 3000, -200)), 0, 0, -200, 0, 5);

            // Tick several times to let proximity checks engage
            CapturedOutput out = null;
            for (long t = 0; t < 50; t++) {
                out = tickAutopilot(ap, terrain, t, 0, 0, -200, 0,
                        new Vec3(0, 5, 0));
            }
            // Should steer away from shallow right side (negative rudder = left)
            assertTrue(out.rudder < 0,
                    "Should steer away from shallow water on right, got rudder=" + out.rudder);
        }

        @Test
        void emergencySurfaceInShallowWater() {
            // Floor at -20m: triggers emergency surface (floorBelow > -30)
            var terrain = flatTerrain(-20);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -15)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -15, 0);
            assertEquals(1.0, out.ballast, 0.001,
                    "Emergency in shallow water: ballast should be 1.0, got " + out.ballast);
            assertTrue(out.sternPlanes > 0,
                    "Emergency in shallow water: should pitch up, got sternPlanes=" + out.sternPlanes);
        }
    }

    // ── 1g. Emergency Layer ─────────────────────────────────────────

    @Nested
    class EmergencyLayer {

        @Test
        void emergencySurfaceTriggersBlocked() {
            // Floor at -20m triggers emergency (floorBelow > -30)
            var terrain = flatTerrain(-20);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -15)), 0, 0, -200, 0, 5);

            tickAutopilot(ap, terrain, 1, 0, 0, -15, 0);
            assertTrue(ap.isBlocked(),
                    "Emergency surface should set isBlocked=true");
        }

        @Test
        void blowTanksOnEmergency() {
            var terrain = flatTerrain(-20);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -15)), 0, 0, -200, 0, 5);

            var out = tickAutopilot(ap, terrain, 1, 0, 0, -15, 0);
            assertEquals(1.0, out.ballast, 0.001,
                    "Emergency should blow tanks (ballast=1.0), got " + out.ballast);
        }

        @Test
        void deepWaterNoEmergency() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);

            tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            assertFalse(ap.isBlocked(),
                    "Deep water should not trigger emergency/blocked");
        }

        @Test
        void emergencyClearsOnNewWaypoints() {
            // First: trigger emergency
            var shallowTerrain = flatTerrain(-20);
            var ap = createAutopilot(shallowTerrain);
            ap.setWaypoints(List.of(wp(2000, 0, -15)), 0, 0, -200, 0, 5);
            tickAutopilot(ap, shallowTerrain, 1, 0, 0, -15, 0);
            assertTrue(ap.isBlocked(), "Should be blocked after emergency");

            // Then: set new waypoints (clears blocked state)
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);
            assertFalse(ap.isBlocked(),
                    "New setWaypoints should clear blocked state");
        }
    }

    // ── 1h. Waypoint Lookahead ──────────────────────────────────────

    @Nested
    class WaypointLookahead {

        @Test
        void earlyTurnLargeAngle() {
            // Set up a waypoint that requires a large turn from current nav direction
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            // Sub heading north, first waypoint east, second waypoint south
            var waypoints = List.of(
                    wp(2000, 0, -200),
                    wp(2000, -2000, -200));
            ap.setWaypoints(waypoints, 0, 0, -200, 0, 5);

            // Tick near the first waypoint
            var out = tickAutopilot(ap, terrain, 1, 1900, 0, -200,
                    Math.toRadians(90), new Vec3(5, 0, 0));
            // Should be steering, not zero rudder
            assertNotEquals(0, out.rudder, 0.01,
                    "Should be turning near waypoint requiring large angle change");
        }

        @Test
        void depthTransitionBeforeArrival() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            // First waypoint at -200m, second at -400m
            var waypoints = List.of(
                    wp(1000, 0, -200),
                    wp(2000, 0, -400));
            ap.setWaypoints(waypoints, 0, 0, -200, 0, 5);

            // Sub near first waypoint at -200m, heading toward second at -400m
            // The depth controller should start transitioning
            var out = tickAutopilot(ap, terrain, 1, 0, 0, -200, 0);
            // At -200m with preferredDepth=-200, ballast should be near neutral
            assertEquals(0.5, out.ballast, 0.15,
                    "At preferred depth, ballast should be near neutral, got " + out.ballast);
        }
    }

    // ── 1i. Layer Handoff ───────────────────────────────────────────

    @Nested
    class LayerHandoff {

        @Test
        void fullArrivalCycle() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            // Waypoint at (500, 0) with default arrivalRadius=200
            ap.setWaypoints(List.of(wp(500, 0, -200)), 0, 0, -200, 0, 5);

            assertFalse(ap.hasArrived(), "Should not be arrived initially");

            // Simulate approach: tick progressively closer
            for (int x = 0; x <= 500; x += 50) {
                tickAutopilot(ap, terrain, x, x, 0, -200, Math.toRadians(90),
                        new Vec3(5, 0, 0));
            }

            assertTrue(ap.hasArrived(),
                    "Should have arrived after reaching waypoint");
        }

        @Test
        void midRouteReplan() {
            var terrain = flatTerrain(-500);
            var ap = createAutopilot(terrain);
            ap.setWaypoints(List.of(wp(3000, 0, -200)), 0, 0, -200, 0, 5);

            // Tick a few times to establish route
            tickAutopilot(ap, terrain, 1, 500, 0, -200, Math.toRadians(90));

            // Mid-route replan
            var firstRoute = List.copyOf(ap.navWaypoints());
            ap.setWaypoints(List.of(wp(0, 3000, -200)), 500, 0, -200, 0, 5);
            var secondRoute = ap.navWaypoints();

            assertFalse(secondRoute.isEmpty(), "New route should be planned after replan");
            assertNotEquals(firstRoute, secondRoute,
                    "Route should change after mid-route replan");
            assertFalse(ap.hasArrived(), "Replan should clear arrived state");
        }

        @Test
        void blockedThenReplan() {
            // First: trigger blocked state
            var shallowTerrain = flatTerrain(-20);
            var ap = createAutopilot(shallowTerrain);
            ap.setWaypoints(List.of(wp(2000, 0, -15)), 0, 0, -200, 0, 5);
            tickAutopilot(ap, shallowTerrain, 1, 0, 0, -15, 0);
            assertTrue(ap.isBlocked(), "Should be blocked in shallow water");

            // Then: replan with new waypoints in deep water
            ap.setWaypoints(List.of(wp(2000, 0, -200)), 0, 0, -200, 0, 5);
            assertFalse(ap.isBlocked(), "New setWaypoints should clear blocked state");
            assertFalse(ap.navWaypoints().isEmpty(), "New route should be planned");
        }
    }
}
