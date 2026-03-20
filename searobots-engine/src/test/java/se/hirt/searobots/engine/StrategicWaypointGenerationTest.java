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

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class StrategicWaypointGenerationTest {

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
    private static final BattleArea AREA = CONFIG.battleArea(); // Circular(7000)

    // crush depth = -700, safety margin = 50, depth limit = -650
    private static final double DEPTH_LIMIT = CONFIG.crushDepth() + 50;

    private DefaultAttackSub controller;

    @BeforeEach
    void setUp() {
        controller = new DefaultAttackSub();
    }

    // ── helpers ──────────────────────────────────────────────────────

    static TerrainMap flatTerrain(double elevation) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, elevation);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    private void startMatch(TerrainMap terrain) {
        var context = new MatchContext(CONFIG, terrain, List.of(), new CurrentField(List.of()));
        controller.onMatchStart(context);
    }

    private void startMatch(TerrainMap terrain, List<ThermalLayer> layers) {
        var context = new MatchContext(CONFIG, terrain, layers, new CurrentField(List.of()));
        controller.onMatchStart(context);
    }

    // ── 2a. Patrol ──────────────────────────────────────────────────

    @Nested
    class Patrol {

        @Test
        void generatesMultipleWaypoints() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            assertTrue(wps.size() >= 3,
                    "Patrol should generate at least 3 waypoints, got " + wps.size());
        }

        @Test
        void waypointsBiasedTowardCenter() {
            // Sub at the edge of the arena, facing outward (heading 0 = north,
            // but position is far east at x=5000). First waypoint should pull
            // toward center (0,0) because distFromCenter > 0.3*extent.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(5000, 0, 0, AREA);

            assertFalse(wps.isEmpty(), "Should generate waypoints");
            var first = wps.getFirst();
            double distFromCenter = Math.sqrt(first.x() * first.x() + first.y() * first.y());
            assertTrue(distFromCenter < 5000,
                    "First patrol waypoint should be closer to center than sub position (5000), "
                            + "got distance " + distFromCenter);
        }

        @Test
        void waypointsBiasedByHeading() {
            // Sub at center facing east (heading = PI/2). With no last contact
            // and distFromCenter < 0.3*extent, patrol should project forward
            // along heading. The first waypoint should have positive X.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(0, 0, Math.PI / 2, AREA);

            assertFalse(wps.isEmpty());
            var first = wps.getFirst();
            assertTrue(first.x() > 0,
                    "Sub facing east, first waypoint should have positive X, got " + first.x());
        }

        @Test
        void pingWaypointAboveThermocline() {
            // With thermal layers, waypoint at index 2 should be PING_POSITION
            // and its depth should be above (shallower than) the thermocline.
            var layers = List.of(new ThermalLayer(-100, 15.0, 8.0));
            var terrain = flatTerrain(-500);
            startMatch(terrain, layers);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            assertTrue(wps.size() > 2, "Need at least 3 waypoints for ping");
            var pingWp = wps.get(2);
            assertEquals(Purpose.PING_POSITION, pingWp.purpose(),
                    "Third waypoint should be PING_POSITION");
            // Depth should be above thermocline: thermocline at -100,
            // ping depth = max(-100 + 10, -20) = -90
            assertTrue(pingWp.preferredDepth() >= -100,
                    "Ping waypoint depth should be above thermocline (-100), got "
                            + pingWp.preferredDepth());
        }

        @Test
        void diveBackAfterPing() {
            // The waypoint after the ping should be deeper than the ping waypoint.
            var layers = List.of(new ThermalLayer(-100, 15.0, 8.0));
            var terrain = flatTerrain(-500);
            startMatch(terrain, layers);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            assertTrue(wps.size() > 3, "Need at least 4 waypoints");
            var pingWp = wps.get(2);
            var afterPing = wps.get(3);
            assertTrue(afterPing.preferredDepth() < pingWp.preferredDepth(),
                    "Waypoint after ping should be deeper: ping=" + pingWp.preferredDepth()
                            + " after=" + afterPing.preferredDepth());
        }

        @Test
        void safeTerrainOnly() {
            // All waypoints should have depth above the floor + clearance
            // (i.e., depth >= floor + clearance, where both are negative).
            var terrain = flatTerrain(-300);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            for (var wp : wps) {
                double floor = terrain.elevationAt(wp.x(), wp.y());
                // Floor clearance is 50 * 0.6 = 30 for stealth depth, but
                // depth must not exceed depthLimit either.
                assertTrue(wp.preferredDepth() >= DEPTH_LIMIT,
                        "Waypoint depth " + wp.preferredDepth()
                                + " should be >= depth limit " + DEPTH_LIMIT);
                assertTrue(wp.preferredDepth() >= floor,
                        "Waypoint depth " + wp.preferredDepth()
                                + " should be above floor " + floor);
            }
        }

        @Test
        void withinBattleArea() {
            // All waypoints should be within the arena bounds.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            for (var wp : wps) {
                double dist = AREA.distanceToBoundary(wp.x(), wp.y());
                assertTrue(dist > 0,
                        "Waypoint (" + wp.x() + "," + wp.y()
                                + ") should be within arena, dist to boundary=" + dist);
            }
        }

        @Test
        void normalNoise() {
            // Most patrol waypoints should have NORMAL noise policy for coverage.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            long normalCount = wps.stream()
                    .filter(wp -> wp.noise() == NoisePolicy.NORMAL).count();
            assertTrue(normalCount >= wps.size() / 2,
                    "Most patrol waypoints should be NORMAL, got " + normalCount
                            + " out of " + wps.size());
        }

        @Test
        void directPattern() {
            // All patrol waypoints should use DIRECT movement pattern.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generatePatrolWaypoints(0, 0, 0, AREA);

            for (var wp : wps) {
                assertEquals(MovementPattern.DIRECT, wp.pattern(),
                        "Patrol waypoints should use DIRECT pattern");
            }
        }
    }

    // ── 2b. Tracking ────────────────────────────────────────────────

    @Nested
    class Tracking {

        @Test
        void perpendicularToBearing() {
            // First waypoint should be approximately perpendicular to contact bearing.
            // Contact bearing = 0 (north), so perpendicular would be ~PI/2 or ~-PI/2.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double contactBearing = 0; // north
            var contact = new TrackedContact(0, 3000, 0, 5.0, 3000, false, 0, false);

            var wps = controller.generateTrackingWaypoints(0, 0, 0, contactBearing, contact);

            assertFalse(wps.isEmpty());
            var first = wps.getFirst();
            // The cross waypoint should be offset sideways (in X), not straight ahead.
            double bearingToWp = Math.atan2(first.x(), first.y());
            double angleToBearing = Math.abs(DefaultAttackSub.angleDiff(bearingToWp, contactBearing));
            // Should be roughly PI/2 (perpendicular), allow some tolerance
            assertTrue(angleToBearing > Math.toRadians(30),
                    "First waypoint should be approximately perpendicular to contact bearing, "
                            + "angle offset was " + Math.toDegrees(angleToBearing) + " degrees");
        }

        @Test
        void towardTrackedPosition() {
            // Second waypoint should be near the contact's position.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double contactX = 2000, contactY = 2000;
            double contactBearing = Math.atan2(contactX, contactY);
            double dist = Math.sqrt(contactX * contactX + contactY * contactY);
            var contact = new TrackedContact(contactX, contactY, 0, 5.0, dist, false, 0, false);

            var wps = controller.generateTrackingWaypoints(0, 0, 0, contactBearing, contact);

            assertTrue(wps.size() >= 2, "Should have at least 2 waypoints");
            var second = wps.get(1);
            assertEquals(contactX, second.x(), 1.0,
                    "Second waypoint X should be near contact X");
            assertEquals(contactY, second.y(), 1.0,
                    "Second waypoint Y should be near contact Y");
        }

        @Test
        void quietNoise() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var contact = new TrackedContact(2000, 2000, 0, 5.0, 2828, false, 0, false);
            double bearing = Math.atan2(2000, 2000);

            var wps = controller.generateTrackingWaypoints(0, 0, 0, bearing, contact);

            for (var wp : wps) {
                assertEquals(NoisePolicy.QUIET, wp.noise(),
                        "Tracking waypoints should be QUIET");
            }
        }

        @Test
        void largerArrivalRadius() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var contact = new TrackedContact(2000, 2000, 0, 5.0, 2828, false, 0, false);
            double bearing = Math.atan2(2000, 2000);

            var wps = controller.generateTrackingWaypoints(0, 0, 0, bearing, contact);

            for (var wp : wps) {
                assertTrue(wp.arrivalRadius() >= 300,
                        "Tracking arrival radius should be >= 300, got " + wp.arrivalRadius());
            }
        }

        @Test
        void targetSpeedHint() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double contactSpeed = 5.0;
            var contact = new TrackedContact(2000, 2000, 0, contactSpeed, 2828, false, 0, false);
            double bearing = Math.atan2(2000, 2000);

            var wps = controller.generateTrackingWaypoints(0, 0, 0, bearing, contact);

            for (var wp : wps) {
                assertTrue(wp.targetSpeed() > 0,
                        "Target speed should be positive, got " + wp.targetSpeed());
                assertEquals(contactSpeed + 1, wp.targetSpeed(), 0.01,
                        "Target speed should be contact speed + 1");
            }
        }
    }

    // ── 2c. Chase ───────────────────────────────────────────────────

    @Nested
    class Chase {

        @Test
        void waypointTowardTarget() {
            // Chase waypoint should be near the target position.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double targetX = 3000, targetY = 3000;
            double dist = Math.sqrt(targetX * targetX + targetY * targetY);
            var contact = new TrackedContact(targetX, targetY, Double.NaN, 5.0, dist,
                    false, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, false);

            assertFalse(wps.isEmpty());
            var first = wps.getFirst();
            // When heading is unknown (NaN), no stern offset applied, so should match
            assertEquals(targetX, first.x(), 1.0, "Chase waypoint X near target");
            assertEquals(targetY, first.y(), 1.0, "Chase waypoint Y near target");
        }

        @Test
        void zigzagAtLongRange() {
            // Distance > 4000 should use ZIGZAG_TMA pattern.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double dist = 5000;
            var contact = new TrackedContact(0, dist, 0, 5.0, dist, false, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, false);

            assertFalse(wps.isEmpty());
            assertEquals(MovementPattern.ZIGZAG_TMA, wps.getFirst().pattern(),
                    "Long range (>4000) should use ZIGZAG_TMA");
        }

        @Test
        void sprintDriftAtMedium() {
            // Distance 2000-4000 should use SPRINT_DRIFT pattern.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double dist = 3000;
            var contact = new TrackedContact(0, dist, 0, 5.0, dist, false, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, false);

            assertFalse(wps.isEmpty());
            assertEquals(MovementPattern.SPRINT_DRIFT, wps.getFirst().pattern(),
                    "Medium range (2000-4000) should use SPRINT_DRIFT");
        }

        @Test
        void directAtClose() {
            // Distance < 2000 should use DIRECT pattern.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double dist = 1800;
            var contact = new TrackedContact(0, dist, 0, 5.0, dist, false, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, false);

            assertFalse(wps.isEmpty());
            assertEquals(MovementPattern.DIRECT, wps.getFirst().pattern(),
                    "Close range (<2000) should use DIRECT");
        }

        @Test
        void sternOffsetWhenCloseAndBehind() {
            // Distance < 1500 with known heading: waypoint should be offset
            // behind the target (stern offset applied).
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double targetHeading = 0; // target heading north
            double dist = 1200;
            // Target at (0, dist), heading north. RAM_RANGE=500, so
            // dist > RAM_RANGE*2 = 1000 is true.
            var contact = new TrackedContact(0, dist, targetHeading, 5.0, dist,
                    true, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, false);

            assertFalse(wps.isEmpty());
            var wp = wps.getFirst();
            // Stern offset should shift waypoint behind the target (lower Y
            // since target heading is north/0). The target is at (0, 1200)
            // heading north, so stern is south of target.
            assertTrue(wp.y() < dist,
                    "Stern offset should place waypoint behind target, expected Y < "
                            + dist + ", got " + wp.y());
        }

        @Test
        void pingWhenStale() {
            // Stale contact with cooldown ready should add a PING_POSITION waypoint.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double dist = 3000;
            var contact = new TrackedContact(0, dist, 0, 5.0, dist, false, 100, true);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, true);

            assertTrue(wps.stream().anyMatch(wp -> wp.purpose() == Purpose.PING_POSITION),
                    "Stale contact with cooldown ready should have a PING_POSITION waypoint");
        }

        @Test
        void noPingWhenFresh() {
            // Fresh contact should not add a PING_POSITION waypoint.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double dist = 3000;
            var contact = new TrackedContact(0, dist, 0, 5.0, dist, false, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, true);

            assertTrue(wps.stream().noneMatch(wp -> wp.purpose() == Purpose.PING_POSITION),
                    "Fresh contact should not produce a PING_POSITION waypoint");
        }

        @Test
        void targetSpeedHint() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double contactSpeed = 7.0;
            double dist = 3000;
            var contact = new TrackedContact(0, dist, 0, contactSpeed, dist, false, 0, false);

            var wps = controller.generateChaseWaypoints(0, 0, 0, contact, false);

            assertFalse(wps.isEmpty());
            var first = wps.getFirst();
            assertEquals(contactSpeed + 1, first.targetSpeed(), 0.01,
                    "Chase target speed should be contact speed + 1");
        }
    }

    // ── 2d. Evade ───────────────────────────────────────────────────

    @Nested
    class Evade {

        @Test
        void perpendicularToThreat() {
            // Evade waypoint should be approximately perpendicular to the threat bearing.
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            double threatBearing = 0; // threat from north

            var wps = controller.generateEvadeWaypoints(0, 0, threatBearing, terrain);

            assertFalse(wps.isEmpty());
            var wp = wps.getFirst();
            double bearingToWp = Math.atan2(wp.x(), wp.y());
            double angleOffset = Math.abs(DefaultAttackSub.angleDiff(bearingToWp, threatBearing));
            // Should be roughly PI/2 (perpendicular)
            assertTrue(angleOffset > Math.toRadians(45),
                    "Evade waypoint should be roughly perpendicular to threat, "
                            + "angle offset was " + Math.toDegrees(angleOffset) + " degrees");
            assertTrue(angleOffset < Math.toRadians(135),
                    "Evade waypoint should be roughly perpendicular to threat (not parallel), "
                            + "angle offset was " + Math.toDegrees(angleOffset) + " degrees");
        }

        @Test
        void deepWaterPreference() {
            // With terrain that is deeper on the positive-X side, evade should
            // prefer the deeper side when both are safe. Threat from north,
            // so perpendicular options are east (+X) and west (-X).
            int size = 201;
            double cellSize = 100;
            double origin = -(size / 2) * cellSize;
            double[] data = new double[size * size];
            for (int row = 0; row < size; row++) {
                for (int col = 0; col < size; col++) {
                    double worldX = origin + col * cellSize;
                    // East side (-800m deep), west side (-200m deep)
                    data[row * size + col] = worldX > 0 ? -800 : -200;
                }
            }
            var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);
            startMatch(terrain);

            double threatBearing = 0; // threat from north

            var wps = controller.generateEvadeWaypoints(0, 0, threatBearing, terrain);

            assertFalse(wps.isEmpty());
            var wp = wps.getFirst();
            assertTrue(wp.x() > 0,
                    "Evade should prefer deeper water side (positive X), got x=" + wp.x());
        }

        @Test
        void silentNoise() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generateEvadeWaypoints(0, 0, 0, terrain);

            for (var wp : wps) {
                assertEquals(NoisePolicy.SILENT, wp.noise(),
                        "Evade waypoints should be SILENT");
            }
        }

        @Test
        void maxSafeDepth() {
            // Evade depth should be near the crush limit (depthLimit + 20).
            var terrain = flatTerrain(-800);
            startMatch(terrain);

            var wps = controller.generateEvadeWaypoints(0, 0, 0, terrain);

            assertFalse(wps.isEmpty());
            var wp = wps.getFirst();
            // depthLimit = crushDepth + 50 = -650, evadeDepth = depthLimit + 20 = -630
            double expectedEvadeDepth = DEPTH_LIMIT + 20;
            assertEquals(expectedEvadeDepth, wp.preferredDepth(), 1.0,
                    "Evade depth should be near depth limit + 20");
        }

        @Test
        void largeArrivalRadius() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);

            var wps = controller.generateEvadeWaypoints(0, 0, 0, terrain);

            for (var wp : wps) {
                assertTrue(wp.arrivalRadius() >= 500,
                        "Evade arrival radius should be >= 500, got " + wp.arrivalRadius());
            }
        }
    }
}
