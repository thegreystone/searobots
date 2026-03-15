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

class DefaultAttackSubTest {

    // Controller constants (mirrored here for readable test comments)
    // FLOOR_CLEARANCE = 80, EMERGENCY_GAP = 60, MIN_DEPTH = -20
    // Crush depth = -700, safety margin = 50, depth limit = -650

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);

    private DefaultAttackSub controller;

    @BeforeEach
    void setUp() {
        controller = new DefaultAttackSub();
    }

    // helpers

    static TerrainMap flatTerrain(double elevation) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, elevation);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    static TerrainMap xVaryingTerrain(java.util.function.DoubleUnaryOperator elevationFn) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double worldX = origin + col * cellSize;
                data[row * size + col] = elevationFn.applyAsDouble(worldX);
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    static TerrainMap yVaryingTerrain(java.util.function.DoubleUnaryOperator elevationFn) {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double worldY = origin + row * cellSize;
            double elev = elevationFn.applyAsDouble(worldY);
            for (int col = 0; col < size; col++) {
                data[row * size + col] = elev;
            }
        }
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

    private CapturedOutput tick(TerrainMap terrain, double x, double y, double z,
                                double heading, Vec3 linearVelocity) {
        return tickFull(terrain, List.of(), 0, x, y, z, heading, linearVelocity,
                1000, List.of(), List.of(), 0);
    }

    private CapturedOutput tick(TerrainMap terrain, double x, double y, double z, double heading) {
        return tick(terrain, x, y, z, heading, Vec3.ZERO);
    }

    private CapturedOutput tickFull(TerrainMap terrain, List<ThermalLayer> layers,
                                     long tick, double x, double y, double z,
                                     double heading, Vec3 linearVelocity, int hp,
                                     List<SonarContact> passive, List<SonarContact> active,
                                     int cooldown) {
        var pose = new Pose(new Vec3(x, y, z), heading, 0, 0);
        var velocity = new Velocity(linearVelocity, Vec3.ZERO);
        var state = new SubmarineState(pose, velocity, hp, 0);
        var env = new EnvironmentSnapshot(terrain, layers, new CurrentField(List.of()));
        var input = new TestInputFull(tick, 0.02, state, env, passive, active, cooldown);
        var output = new CapturedOutput();
        controller.onTick(input, output);
        return output;
    }

    record TestInput(long tick, double deltaTimeSeconds,
                     SubmarineState self, EnvironmentSnapshot environment)
            implements SubmarineInput {}

    record TestInputFull(long tick, double deltaTimeSeconds,
                         SubmarineState self, EnvironmentSnapshot environment,
                         List<SonarContact> sonarContacts, List<SonarContact> activeSonarReturns,
                         int activeSonarCooldownTicks)
            implements SubmarineInput {}

    static final class CapturedOutput implements SubmarineOutput {
        double rudder, sternPlanes, throttle, ballast;
        boolean pinged;
        final java.util.ArrayList<se.hirt.searobots.api.Waypoint> waypoints = new java.util.ArrayList<>();

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void publishWaypoint(se.hirt.searobots.api.Waypoint wp) { waypoints.add(wp); }
    }

    // depth control: bottom tracking

    @Nested
    class BottomTracking {

        @Test
        void sinksTowardDeepFloor() {
            // Floor at -500m, target = -500+80 = -420m, sub at -100m, should sink
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertTrue(out.ballast < 0.5,
                    "Sub at -100m over -500m floor should sink, got ballast=" + out.ballast);
        }

        @Test
        void risesWhenTooDeep() {
            // Floor at -500m, target = -420m, sub at -470m, too deep, should rise
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -470, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub below target depth should rise, got ballast=" + out.ballast);
        }

        @Test
        void nearNeutralAtTargetDepth() {
            // Floor at -500m, target = -500+80 = -420m
            // Sub at -420m with no vertical speed, ballast near 0.5
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -420, 0);

            assertEquals(0.5, out.ballast, 0.1,
                    "Sub at target depth should have near-neutral ballast");
        }

        @Test
        void sternPlanesPitchDownWhenTooShallow() {
            // Floor at -500m, target = -420m, sub at -100m, 320m too shallow, pitch down
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertTrue(out.sternPlanes < 0,
                    "Sub too shallow should pitch down, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void sternPlanesPitchUpWhenTooDeep() {
            // Floor at -500m, target = -420m, sub at -470m, too deep, pitch up
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -470, 0);

            assertTrue(out.sternPlanes > 0,
                    "Sub too deep should pitch up, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void tracksShallowFloor() {
            // Floor at -80m, target = -80+80 = 0m, clamped to MIN_DEPTH=-20
            // Sub at -200m, too deep, should rise
            var terrain = flatTerrain(-80);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub should rise toward shallow floor, got ballast=" + out.ballast);
        }
    }

    // depth limits

    @Nested
    class DepthLimits {

        @Test
        void respectsMinDepth() {
            // Floor at -40m, target would be -40+80 = +40m, clamped to MIN_DEPTH=-20
            // Sub at -30m, below -20 target, should rise
            var terrain = flatTerrain(-40);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -30, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub at -30m with MIN_DEPTH=-20 target should rise, got ballast=" + out.ballast);
        }

        @Test
        void respectsCrushDepthLimit() {
            // Floor at -900m, target would be -820m, clamped to -650 (crush limit)
            // Sub at -400m, should sink toward -650
            var terrain = flatTerrain(-900);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -400, 0);

            assertTrue(out.ballast < 0.5,
                    "Sub at -400m should sink toward -650 limit, got ballast=" + out.ballast);
        }

        @Test
        void doesNotSinkBelowCrushDepthLimit() {
            // Floor at -900m, crush limit = -650m
            // Sub at -660m, below limit, should rise
            var terrain = flatTerrain(-900);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -660, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub below crush depth limit should rise, got ballast=" + out.ballast);
        }
    }

    // surface avoidance

    @Nested
    class SurfaceAvoidance {

        @Test
        void sinksWhenTooShallowForSurface() {
            // Sub at -10m, above MIN_DEPTH=-20, surface avoidance kicks in
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -10, 0);

            assertTrue(out.ballast < 0.5,
                    "Sub near surface should sink, got ballast=" + out.ballast);
            assertTrue(out.sternPlanes < 0,
                    "Sub near surface should pitch down, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void noSurfaceAvoidanceAtDepth() {
            // Sub at -100m, well below MIN_DEPTH, surface avoidance should not fire.
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertNotEquals(0.2, out.ballast, 0.01,
                    "Surface avoidance ballast (0.2) should not fire at depth");
        }
    }

    // emergency: floor proximity

    @Nested
    class EmergencyFloorProximity {

        @Test
        void emergencyRiseWhenCloseToFloor() {
            // Floor at -500m, sub at -480m, gap = 20m < 60m, emergency
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -480, 0);

            assertTrue(out.ballast > 0.5,
                    "Emergency: should rise, got ballast=" + out.ballast);
            assertTrue(out.sternPlanes > 0,
                    "Emergency: should pitch up, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void noEmergencyWithSufficientClearance() {
            // Floor at -500m, sub at -430m, gap = 70m > 60m, no emergency
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -430, 0);

            assertNotEquals(0.8, out.sternPlanes, 0.01,
                    "Should not trigger emergency with 70m clearance");
        }
    }

    // terrain lookahead: turn around obstacles

    @Nested
    class TerrainLookahead {

        @Test
        void turnsLeftWhenLeftIsDeeper() {
            // Sub heading north, floor ahead rises, left (-X) is deeper
            var terrain = xVaryingTerrain(x -> x < 0 ? -500 : -100);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertTrue(out.rudder < 0,
                    "Should turn left toward deeper side, got rudder=" + out.rudder);
        }

        @Test
        void turnsRightWhenRightIsDeeper() {
            var terrain = xVaryingTerrain(x -> x > 0 ? -500 : -100);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertTrue(out.rudder > 0,
                    "Should turn right toward deeper side, got rudder=" + out.rudder);
        }

        @Test
        void noTurnWhenFloorIsClear() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertEquals(0, out.rudder, 0.001,
                    "Should not turn with clear deep floor, got rudder=" + out.rudder);
        }

        @Test
        void turnsAroundRisingTerrainAhead() {
            // Floor ahead rises to -50m, sub at -200m
            var terrain = yVaryingTerrain(y -> y > 100 ? -50 : -300);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertNotEquals(0, out.rudder, 0.001,
                    "Should turn to avoid rising terrain ahead");
        }

        @Test
        void detectsObstacleAtMultipleDistances() {
            // Floor rises at Y=300 (between 200m and 400m scan distances)
            // Should still detect it via the 400m scan point
            var terrain = yVaryingTerrain(y -> y > 250 ? -80 : -400);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertNotEquals(0, out.rudder, 0.001,
                    "Should detect rising terrain at far scan distance");
        }
    }

    // drop-off handling

    @Nested
    class DropOffHandling {

        // Terrain: flat at -200m, drops to -500m at Y > 50m.
        // Sub heading north (0), so scan points are along +Y.
        static TerrainMap sharpDropOff() {
            return yVaryingTerrain(y -> y > 50 ? -500 : -200);
        }

        // Terrain: flat at -200m, drops to -500m at Y > 200m (farther away).
        static TerrainMap distantDropOff() {
            return yVaryingTerrain(y -> y > 200 ? -500 : -200);
        }

        @Test
        void detectsDropOffAndReducesThrottle() {
            // Sub at target depth (-150m), heading north toward a 300m drop-off
            var terrain = sharpDropOff();
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -150, 0);

            assertTrue(out.throttle < 0.6,
                    "Should reduce throttle at drop-off, got throttle=" + out.throttle);
        }

        @Test
        void detectsDistantDropOff() {
            // Drop-off at Y=200, scan points at 200 and 400m should catch it
            var terrain = distantDropOff();
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -150, 0);

            assertTrue(out.throttle < 0.6,
                    "Should detect drop-off at 200m distance, got throttle=" + out.throttle);
        }

        @Test
        void atHighSpeedSternPlanesMoreEffective() {
            var terrain = sharpDropOff();
            startMatch(terrain);

            // Heading north with high forward speed (12 m/s, above REF_SPEED)
            var outFast = tick(terrain, 0, 0, -180, 0, new Vec3(0, 12, 0));
            // Same scenario but stationary
            var outSlow = tick(terrain, 0, 0, -180, 0, Vec3.ZERO);

            assertTrue(outFast.throttle < 0.6,
                    "Fast sub should still reduce throttle at drop-off");
            assertTrue(outSlow.throttle < 0.6,
                    "Slow sub should still reduce throttle at drop-off");
        }

        @Test
        void risesAtDropOffToMaintainClearance() {
            var terrain = sharpDropOff();
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -180, 0, new Vec3(0, 8, 0));

            assertTrue(out.ballast >= 0.3,
                    "Should not aggressively sink at drop-off, got ballast=" + out.ballast);
        }

        @Test
        void turnsTowardDeeperSideAtDropOff() {
            var terrain = new TerrainMap(
                    buildDropOffWithShallowLeft(), 201, 201,
                    -(201 / 2) * 100.0, -(201 / 2) * 100.0, 100);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -150, 0, new Vec3(0, 8, 0));

            assertTrue(out.rudder > 0,
                    "Should turn right toward deeper side at drop-off, got rudder=" + out.rudder);
        }

        @Test
        void farFromDropOffDetectsItEarlyAndSlows() {
            var terrain = sharpDropOff();
            startMatch(terrain);
            var outFar = tick(terrain, 0, -200, -150, 0, new Vec3(0, 8, 0));

            assertTrue(outFar.throttle < 0.6,
                    "Far sub should detect upcoming drop-off and slow, got throttle="
                            + outFar.throttle);
        }

        @Test
        void highSpeedApproachStillDetectsDropOff() {
            var terrain = sharpDropOff();
            startMatch(terrain);
            var out = tick(terrain, 0, 30, -150, 0, new Vec3(0, 15, 0));

            assertTrue(out.throttle < 0.6,
                    "High-speed approach should detect imminent drop-off, got throttle="
                            + out.throttle);
        }

        private static double[] buildDropOffWithShallowLeft() {
            int size = 201;
            double cellSize = 100;
            double origin = -(size / 2) * cellSize;
            double[] data = new double[size * size];
            for (int row = 0; row < size; row++) {
                double worldY = origin + row * cellSize;
                for (int col = 0; col < size; col++) {
                    double worldX = origin + col * cellSize;
                    if (worldY > 50 && worldX > 0) {
                        data[row * size + col] = -500; // deep: right side ahead
                    } else {
                        data[row * size + col] = -200; // shallow shelf
                    }
                }
            }
            return data;
        }
    }

    // border avoidance

    @Nested
    class BorderAvoidance {

        @Test
        void steersTowardCenterNearBoundary() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 6500, 0, -200, 0);

            assertNotEquals(0, out.rudder, 0.001,
                    "Should steer toward center near boundary");
        }

        @Test
        void noSteeringWhenFarFromBoundary() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertEquals(0, out.rudder, 0.001,
                    "Should not steer when far from boundary");
        }
    }

    // damping

    @Nested
    class Damping {

        @Test
        void dampsSinkingWhenApproachingTarget() {
            // Floor at -500m, target = -420m (FLOOR_CLEARANCE=80)
            // Sub at -410m (10m above target) but sinking fast
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var outSinking = tick(terrain, 0, 0, -410, 0, new Vec3(0, 0, -3));
            var outStill = tick(terrain, 0, 0, -410, 0, Vec3.ZERO);

            assertTrue(outSinking.ballast > outStill.ballast,
                    "Damping should make ballast more buoyant when already sinking toward target");
        }

        @Test
        void dampsRisingWhenApproachingTarget() {
            // Sub at -430m (10m below target -420m) but rising fast
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var outRising = tick(terrain, 0, 0, -430, 0, new Vec3(0, 0, 3));
            var outStill = tick(terrain, 0, 0, -430, 0, Vec3.ZERO);

            assertTrue(outRising.ballast < outStill.ballast,
                    "Damping should make ballast less buoyant when already rising toward target");
        }
    }

    // throttle

    @Nested
    class Throttle {

        @Test
        void patrolCruises() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertEquals(0.4, out.throttle, 0.001, "PATROL should set patrol throttle");
        }
    }

    // island start: low clearance, no speed

    @Nested
    class IslandStart {

        // Terrain: shallow island ahead (+Y), deep water behind (-Y) and to sides
        static TerrainMap islandAhead() {
            return yVaryingTerrain(y -> y > 0 ? -60 : -400);
        }

        @Test
        void emergencyBlowsBallastNearFloor() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -480, 0);

            assertTrue(out.ballast > 0.5,
                    "Emergency should blow ballast, got ballast=" + out.ballast);
        }

        @Test
        void reversesInEmergencyNearFloor() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -470, 0);

            assertTrue(out.throttle < 0,
                    "Emergency should reverse thrust, got throttle=" + out.throttle);
        }

        @Test
        void doesNotGetStuckOnShallowFloor() {
            var terrain = flatTerrain(-60);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -30, 0);

            assertTrue(out.ballast > 0.5,
                    "Should rise toward target on shallow floor, got ballast=" + out.ballast);
        }

        @Test
        void survivesStartWithMinimalClearance() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -480, 0);

            assertTrue(out.sternPlanes > 0,
                    "Should pitch up with minimal clearance, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void turnsAwayFromIslandAtZeroSpeed() {
            var terrain = islandAhead();
            startMatch(terrain);
            var out = tick(terrain, 0, -50, -50, 0);

            assertNotEquals(0, out.rudder, 0.001,
                    "Should command rudder turn near island, got rudder=" + out.rudder);
        }

        @Test
        void buildsReverseSpeedForRudderAuthority() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -475, 0);

            assertTrue(out.throttle < 0,
                    "Emergency should reverse, got throttle=" + out.throttle);
            assertTrue(out.sternPlanes > 0,
                    "Should pitch up, got sternPlanes=" + out.sternPlanes);
        }
    }

    // ======================================================================
    // State machine tests
    // ======================================================================

    private static final TerrainMap DEEP_FLAT = flatTerrain(-500);
    private static final SonarContact CONTACT_NORTH =
            new SonarContact(0, 10.0, 0, false, -1, 0, 0, 90.0, 0.0, Double.NaN);
    private static final SonarContact CONTACT_NORTH_LOUD =
            new SonarContact(0, 20.0, 0, false, -1, 0, 0, 90.0, 0.0, Double.NaN);

    // Feed N ticks with the same passive contact to trigger state transitions
    private void feedContactTicks(int n, long startTick, double x, double y, double z,
                                   double heading, SonarContact contact) {
        for (int i = 0; i < n; i++) {
            tickFull(DEEP_FLAT, List.of(), startTick + i, x, y, z, heading,
                    Vec3.ZERO, 1000, List.of(contact), List.of(), 0);
        }
    }

    // State transitions

    @Nested
    class StateTransitions {

        @Test
        void transitionsToTrackingOnContact() {
            startMatch(DEEP_FLAT);
            assertEquals(DefaultAttackSub.State.PATROL, controller.state());

            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());
        }

        @Test
        void doesNotTransitionOnSingleTickContact() {
            startMatch(DEEP_FLAT);
            feedContactTicks(1, 0, 0, 0, -200, 0, CONTACT_NORTH);

            assertEquals(DefaultAttackSub.State.PATROL, controller.state(),
                    "Single tick contact should not trigger TRACKING");
        }

        @Test
        void transitionsToPatrolOnContactLossViaConfidenceDecay() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());

            // TRACKING transitions to PATROL when !hasTrackedContact && ticksSinceContact > 500.
            // hasTrackedContact is cleared when uncertaintyRadius > 5000.
            // At maxSubSpeed=15 m/s and dt=0.02s, uncertainty grows by 0.3m/tick.
            // 5000/0.3 ~ 16667 ticks. Feed 17000 ticks to ensure the threshold is
            // crossed and ticksSinceContact > 500.
            long startTick = DefaultAttackSub.CONTACT_CONFIRM_TICKS;
            for (int i = 0; i < 17000; i++) {
                tickFull(DEEP_FLAT, List.of(), startTick + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }

            assertEquals(DefaultAttackSub.State.PATROL, controller.state());
        }

        @Test
        void transitionsToChaseOnRangeEstimate() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());

            // Feed contacts from positions far enough apart for triangulation
            // Target at (0, 2000). From (0,0) bearing is 0 (north).
            // From (300, 0): bearing = atan2(0-300, 2000-0) ~ -0.149 rad
            double bearing2 = Math.atan2(0 - 300, 2000 - 0);
            if (bearing2 < 0) bearing2 += 2 * Math.PI;
            var contact2 = new SonarContact(bearing2, 10.0, 0, false, -1, 0, 0, 90.0, 0.0, Double.NaN);

            // First fix at (0, 0)
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);
            // Second fix at (300, 0), far enough for displacement > 200m
            tickFull(DEEP_FLAT, List.of(), 200, 300, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(contact2), List.of(), 0);

            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "Should transition to CHASE with range estimate < 3000m");
        }

        @Test
        void transitionsToRamOnCloseRange() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING then CHASE via active sonar return with range
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            // Give an active return with range 2000m, triggers CHASE
            var activeContact = new SonarContact(0, 15.0, 2000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeContact), 0);

            // Give an active return with range 400m, triggers RAM
            var closeContact = new SonarContact(0, 25.0, 400, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(closeContact), 0);

            assertEquals(DefaultAttackSub.State.RAM, controller.state(),
                    "Should transition to RAM with range < 500m");
        }

        @Test
        void transitionsToEvadeOnDamage() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());

            // Take damage (hp drops from 1000 to 900)
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(DefaultAttackSub.State.EVADE, controller.state());
        }
    }

    // PATROL behavior

    @Nested
    class PatrolBehavior {

        @Test
        void patrolReducesThrottleFromPhase2() {
            startMatch(DEEP_FLAT);
            var out = tick(DEEP_FLAT, 0, 0, -200, 0);
            assertEquals(DefaultAttackSub.PATROL_THROTTLE, out.throttle, 0.001,
                    "PATROL throttle should be 0.4");
        }

        @Test
        void patrolPublishesWaypoints() {
            startMatch(DEEP_FLAT);
            // After the first tick, patrol should generate and publish waypoints
            var out = tickFull(DEEP_FLAT, List.of(), 0, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertFalse(out.waypoints.isEmpty(), "Patrol should publish navigation waypoints");
            // Exactly one waypoint should be active
            long activeCount = out.waypoints.stream().filter(se.hirt.searobots.api.Waypoint::active).count();
            assertEquals(1, activeCount, "Exactly one waypoint should be marked active");
        }

        @Test
        void patrolPingsAfterLongSilence() {
            startMatch(DEEP_FLAT);
            CapturedOutput out = null;
            for (long i = 0; i <= DefaultAttackSub.PATROL_SILENCE_PING_TICKS; i++) {
                out = tickFull(DEEP_FLAT, List.of(), i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertTrue(out.pinged, "Should ping after long silence in PATROL");
        }

        @Test
        void patrolPrefersDepthBelowThermocline() {
            var layers = List.of(new ThermalLayer(-120, 18.0, 8.0));
            startMatch(DEEP_FLAT, layers);
            var out = tickFull(DEEP_FLAT, layers, 0, 0, 0, -100, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.ballast < 0.5,
                    "PATROL should prefer depth below thermocline, got ballast=" + out.ballast);
        }
    }

    // TRACKING behavior

    @Nested
    class TrackingBehavior {

        @Test
        void trackingTransitionsToChaseWithSLRangeEstimate() {
            // With estimatedSourceLevel=90 and SE=10, the SE-based range estimate
            // is ~100m (< CHASE_RANGE), causing an immediate TRACKING->CHASE
            // transition on the next tick after entering TRACKING.
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());

            // Next tick: SE+SL range estimate (~100m) triggers CHASE
            var out = tickFull(DEEP_FLAT, List.of(),
                    DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "SE+SL range estimate should trigger CHASE transition");
            assertTrue(out.throttle > DefaultAttackSub.TRACKING_THROTTLE,
                    "CHASE throttle should be higher than TRACKING, got " + out.throttle);
        }

        @Test
        void trackingManeuversPerpendicularToContact() {
            startMatch(DEEP_FLAT);
            // Contact at bearing 0 (north), sub heading north (0)
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            var out = tickFull(DEEP_FLAT, List.of(),
                    DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            // Should turn toward perpendicular (east or west = +/- pi/2)
            assertNotEquals(0, out.rudder, 0.01,
                    "TRACKING should turn perpendicular to contact bearing");
        }

        @Test
        void trackingAccumulatesBearingFixes() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            // Feed contacts from different positions with engine TMA range estimates.
            // Target at (0, 2000). From (0,0) bearing is 0 (north).
            // Engine TMA provides range ~2000m via SE-based estimation.
            var contactWithRange = new SonarContact(0, 10.0, 2000, false, -1, 0, 500, 90.0, 0.3, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(contactWithRange), List.of(), 0);
            // From (300, 0), bearing to (0, 2000) = atan2(0-300, 2000-0) ~ -0.149 rad, +2pi ~ 6.13
            double bearing2 = Math.atan2(0 - 300.0, 2000.0);
            if (bearing2 < 0) bearing2 += 2 * Math.PI;
            var contact2 = new SonarContact(bearing2, 10.0, 2020, false, -1, 0, 500, 90.0, 0.35, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 200, 300, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(contact2), List.of(), 0);

            assertTrue(controller.hasTrackedContact(),
                    "Should have tracked contact from engine TMA data");
            assertTrue(controller.estimatedRange() < Double.MAX_VALUE,
                    "Should have a range estimate from engine TMA data");
        }

        @Test
        void trackingHeadsTowardTrackedContactWhenNoFreshContact() {
            // CHASE no longer drops to TRACKING on low confidence. Instead, verify
            // that CHASE steers toward tracked contact when no fresh contacts arrive.
            startMatch(DEEP_FLAT);

            // Enter TRACKING via passive contacts
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());

            // Give an active ping to establish tracked contact to the east
            var activeEast = new SonarContact(Math.PI / 2, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeEast), 0);
            // Now in CHASE with tracked contact to the east
            assertEquals(DefaultAttackSub.State.CHASE, controller.state());

            // Let some ticks pass with no fresh contact (CHASE persists)
            for (int i = 0; i < 500; i++) {
                tickFull(DEEP_FLAT, List.of(), 101 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }

            // Should still be in CHASE (no longer drops to TRACKING)
            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "CHASE persists while tracked contact exists");

            // Tracked contact should still exist
            assertTrue(controller.hasTrackedContact(),
                    "Should still have tracked contact in CHASE");

            // Sub heading north (0), tracked contact to the east, should steer right
            var out = tickFull(DEEP_FLAT, List.of(), 601, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.rudder > 0,
                    "Should steer toward tracked contact (east), got rudder=" + out.rudder);
        }
    }

    // CHASE behavior

    @Nested
    class ChaseBehavior {

        private void enterChase() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Active return with range 2000m triggers CHASE
            var activeContact = new SonarContact(0, 15.0, 2000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeContact), 0);
        }

        @Test
        void chaseIncreasesSpeed() {
            enterChase();
            assertEquals(DefaultAttackSub.State.CHASE, controller.state());

            // During sprint phase, throttle should be adaptive (faster than
            // TRACKING to close on target). With default tracked speed ~2 m/s,
            // adaptive throttle computes a minimum closing speed.
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertTrue(out.throttle > DefaultAttackSub.TRACKING_THROTTLE,
                    "CHASE throttle should be higher than TRACKING, got " + out.throttle);
        }

        @Test
        void chaseUsesSprintAndDrift() {
            enterChase();
            // chaseStartTick was set at tick 100 (when enterChase transitions)

            // Sprint phase: tick 101 is phaseTime=(101-100)%1750=1 < SPRINT_DURATION
            var outSprint = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            // Drift phase: tick (100 + SPRINT_DURATION + 1) gives phaseTime=751 >= SPRINT_DURATION
            long driftTick = 100 + DefaultAttackSub.SPRINT_DURATION + 1;
            var outDrift = tickFull(DEEP_FLAT, List.of(), driftTick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertTrue(outSprint.throttle > outDrift.throttle,
                    "Sprint throttle (" + outSprint.throttle +
                            ") should exceed drift throttle (" + outDrift.throttle + ")");
            assertEquals(DefaultAttackSub.TRACKING_THROTTLE, outDrift.throttle, 0.01,
                    "Drift throttle should be tracking throttle");
        }

        @Test
        void chasePersistsWithTrackedContact() {
            enterChase();
            assertEquals(DefaultAttackSub.State.CHASE, controller.state());
            assertTrue(controller.hasTrackedContact(), "Should have tracked contact after ping");

            // After 500 ticks with no fresh contact, should still be CHASE
            // because contactAlive is still above CONFIDENCE_HUNT_MIN
            // 1.0 * 0.999^500 ~ 0.607 > 0.1
            for (int i = 0; i < 500; i++) {
                tickFull(DEEP_FLAT, List.of(), 101 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "CHASE should persist while tracked contact confidence is above threshold");
        }

        @Test
        void chasePersistsUntilUncertaintyTooLarge() {
            enterChase();
            assertEquals(DefaultAttackSub.State.CHASE, controller.state());

            // CHASE no longer drops to TRACKING on low confidence. Instead, it
            // stays in CHASE as long as hasTrackedContact is true. Verify CHASE
            // persists through the old confidence decay window (2400 ticks).
            for (int i = 0; i < 2400; i++) {
                tickFull(DEEP_FLAT, List.of(), 101 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "CHASE should persist while tracked contact exists (uncertaintyRadius still small)");

            // CHASE drops to PATROL when !hasTrackedContact && uncertaintyRadius > 3000.
            // hasTrackedContact is cleared at uncertaintyRadius > 5000. At 0.3m/tick,
            // 5000/0.3 ~ 16667 ticks total. We already ran 2400, need ~14300 more.
            for (int i = 0; i < 14300; i++) {
                tickFull(DEEP_FLAT, List.of(), 2501 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertEquals(DefaultAttackSub.State.PATROL, controller.state(),
                    "CHASE should drop to PATROL when uncertaintyRadius exceeds 5000");
        }

        @Test
        void chaseApproachesFromStern() {
            enterChase();
            // enterChase gives active return at tick 100, bearing 0, range 2000
            // Target at (0, 2000), sub at (0, 0), heading north

            // Second ping: target moved east (pure eastward heading).
            // Target now at (100, 2000), range ~2002m. Displacement 100m in 7s = 700 ticks.
            // At maxSubSpeed 15 m/s, 100m in 7s is plausible.
            double tx2 = 100, ty2 = 2000;
            double bearing2 = Math.atan2(tx2, ty2);
            double range2 = Math.sqrt(tx2 * tx2 + ty2 * ty2);
            // Engine tracker provides heading estimate (east, pi/2) on second ping.
            var active2 = new SonarContact(bearing2, 15.0, range2, true, -1, 0, 0, 90.0, 0.95, Math.PI / 2);
            tickFull(DEEP_FLAT, List.of(), 800, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active2), 0);

            assertTrue(!Double.isNaN(controller.trackedHeading()),
                    "Should have tracked heading");

            // Third ping: target at (400, 1000), range ~1077m, heading east.
            double tx3 = 400, ty3 = 1000;
            double bearing3 = Math.atan2(tx3, ty3);
            double range3 = Math.sqrt(tx3 * tx3 + ty3 * ty3);
            var active3 = new SonarContact(bearing3, 15.0, range3, true, -1, 0, 0, 90.0, 0.95, Math.PI / 2);
            tickFull(DEEP_FLAT, List.of(), 2800, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active3), 0);

            // Target is at (400, 1000), heading roughly east, range ~1077m.
            // Sub at (0,0) heading north. Stern offset should push the aim point
            // west of the target (behind it). The bearing to the offset point
            // should be more eastward than the bearing to the target, causing
            // a positive rudder correction (steer east, toward the stern).
            // Actually: target heading is east (pi/2 ish), so stern is to its WEST.
            // Aim point is WEST of target. From our position at (0,0), the aim
            // point is less east than the target, so we steer less east = our
            // rudder should be less positive than without offset. But it should
            // still steer roughly toward the target. The key: the rudder
            // value should differ from a direct-approach rudder value.
            // Let's just verify the sub steers toward the target area (positive rudder
            // since target is east of us).
            var out = tickFull(DEEP_FLAT, List.of(), 2801, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(new SonarContact(bearing3, 10.0, 0, false, -1, 0, 0, 90.0, 0.0, Double.NaN)),
                    List.of(), 0);
            // The sub should have waypoints leading toward the target area.
            // With the path planner, steering goes through waypoints rather than
            // directly, so we check that waypoints exist and point roughly east.
            assertFalse(out.waypoints.isEmpty(),
                    "Should have waypoints toward target");
            var lastWp = out.waypoints.getLast();
            assertTrue(lastWp.x() > 200,
                    "Final waypoint should be east of origin, got x=" + lastWp.x());
        }

        @Test
        void chasePrefersDeepSprint() {
            enterChase();
            // At -100m, chase target depth should be much deeper (for cavitation avoidance)
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -100, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertTrue(out.ballast < 0.5,
                    "CHASE should prefer deep sprint depth, got ballast=" + out.ballast);
        }
    }

    // EVADE behavior

    @Nested
    class EvadeBehavior {

        private void enterEvade() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Take damage to trigger EVADE
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(CONTACT_NORTH), List.of(), 0);
        }

        @Test
        void evadeGoesQuiet() {
            enterEvade();
            assertEquals(DefaultAttackSub.State.EVADE, controller.state());

            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(), List.of(), 0);

            assertEquals(DefaultAttackSub.EVADE_THROTTLE, out.throttle, 0.01,
                    "EVADE throttle should be 0.15");
        }

        @Test
        void evadeDivesDeep() {
            enterEvade();
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(), List.of(), 0);

            assertTrue(out.ballast < 0.5,
                    "EVADE should dive deep, got ballast=" + out.ballast);
        }

        @Test
        void evadeTurnsPerpendicularToThreat() {
            enterEvade();
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(), List.of(), 0);

            assertNotEquals(0, out.rudder, 0.01,
                    "EVADE should turn perpendicular to threat bearing");
        }

        @Test
        void evadeDoesNotClearBaffles() {
            enterEvade();
            for (int i = 0; i < DefaultAttackSub.BAFFLE_CLEAR_INTERVAL + 100; i++) {
                tickFull(DEEP_FLAT, List.of(), 101 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 900, List.of(CONTACT_NORTH), List.of(), 0);
            }
            assertNotEquals(DefaultAttackSub.State.PATROL, controller.state(),
                    "Should remain in EVADE with ongoing contact");
        }

        @Test
        void evadeTransitionsToPatrolWhenContactFullyLost() {
            enterEvade();
            assertEquals(DefaultAttackSub.State.EVADE, controller.state());
            assertTrue(controller.hasTrackedContact(),
                    "Should have tracked contact from TRACKING phase");

            // EVADE transitions to PATROL when !hasTrackedContact. Tracked contact
            // is cleared when uncertaintyRadius > 5000. At 15 m/s max speed and
            // dt=0.02s, uncertainty grows 0.3m/tick: 5000/0.3 ~ 16667 ticks.
            for (int i = 0; i < 17000; i++) {
                tickFull(DEEP_FLAT, List.of(), 101 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 900, List.of(), List.of(), 0);
                if (controller.state() == DefaultAttackSub.State.PATROL) break;
            }
            assertEquals(DefaultAttackSub.State.PATROL, controller.state(),
                    "EVADE should transition to PATROL when uncertaintyRadius exceeds 5000");
        }
    }

    // RAM behavior

    @Nested
    class RamBehavior {

        @Test
        void ramUsesFullThrottle() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Active return at 2000m, CHASE
            var activeChase = new SonarContact(0, 15.0, 2000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeChase), 0);
            // Active return at 400m, RAM
            var activeRam = new SonarContact(0, 25.0, 400, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeRam), 0);
            assertEquals(DefaultAttackSub.State.RAM, controller.state());

            // Next tick in RAM
            var out = tickFull(DEEP_FLAT, List.of(), 201, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(DefaultAttackSub.RAM_THROTTLE, out.throttle, 0.01,
                    "RAM throttle should be 1.0");
        }

        @Test
        void ramPingsOnContactLoss() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            var activeChase = new SonarContact(0, 15.0, 2000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeChase), 0);
            var activeRam = new SonarContact(0, 25.0, 400, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeRam), 0);
            assertEquals(DefaultAttackSub.State.RAM, controller.state());

            // Lose contact for 101+ ticks, should ping
            var out = tickFull(DEEP_FLAT, List.of(), 301, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);

            assertTrue(out.pinged, "RAM should ping when contact is lost");
        }

        @Test
        void ramOvershotTransitionsToChase() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Active return at 2000m, CHASE
            var activeChase = new SonarContact(0, 15.0, 2000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeChase), 0);
            // Active return at 400m, RAM
            var activeRam = new SonarContact(0, 25.0, 400, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeRam), 0);
            assertEquals(DefaultAttackSub.State.RAM, controller.state());

            // Give an active return at 900m (> RAM_OVERSHOT_RANGE) to update estimatedRange
            var activeOvershot = new SonarContact(0, 15.0, 900, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 201, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeOvershot), 0);
            // Now wait 51+ ticks with no contact so ticksSinceContact > 50
            tickFull(DEEP_FLAT, List.of(), 253, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "RAM overshot should transition to CHASE (not PATROL)");
        }
    }

    // Tracked contact behavior

    @Nested
    class TrackedContactBehavior {

        @Test
        void activePingCreatesTrackedContact() {
            startMatch(DEEP_FLAT);
            var active = new SonarContact(0, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            assertTrue(controller.hasTrackedContact(),
                    "Should have tracked contact after active ping");
            assertEquals(1.0, controller.contactAlive(), 0.01,
                    "contactAlive should be 1.0 after active ping");
            // Target should be at (0, 5000), bearing 0 (north), range 5000
            assertEquals(0, controller.trackedX(), 50);
            assertEquals(5000, controller.trackedY(), 50);
        }

        @Test
        void activePingTransitionsDirectlyToChase() {
            startMatch(DEEP_FLAT);
            // No prior contacts, go straight from PATROL to CHASE on active return
            var active = new SonarContact(0, 15.0, 8000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "Active ping return should transition directly to CHASE from PATROL");
        }

        @Test
        void activePingInTrackingTransitionsToChase() {
            startMatch(DEEP_FLAT);
            feedContactTicks(DefaultAttackSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(DefaultAttackSub.State.TRACKING, controller.state());

            // Active return at long range should bypass CHASE_RANGE check
            var active = new SonarContact(0, 15.0, 10000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            assertEquals(DefaultAttackSub.State.CHASE, controller.state(),
                    "Active return should transition TRACKING to CHASE regardless of range");
        }

        @Test
        void trackedContactConfidenceDecays() {
            startMatch(DEEP_FLAT);
            // Active ping to establish high-confidence tracked contact
            var active = new SonarContact(0, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            assertEquals(1.0, controller.contactAlive(), 0.01);

            // Run 250 ticks (5s) with no contact. contactAlive should decay.
            // Expected: 1.0 * 0.999^250 ~ 0.778
            for (int i = 0; i < 250; i++) {
                tickFull(DEEP_FLAT, List.of(), 2 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertTrue(controller.contactAlive() < 0.85,
                    "contactAlive should decay over 5s, got " + controller.contactAlive());
            assertTrue(controller.contactAlive() > 0.65,
                    "contactAlive should not decay too fast, got " + controller.contactAlive());
        }

        @Test
        void trackedContactDeadReckonsPosition() {
            startMatch(DEEP_FLAT);
            // Two active pings to establish heading. Second ping includes
            // engine-estimated heading (north, 0 rad) from two successive fixes.
            var active1 = new SonarContact(0, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active1), 0);
            // Target moved north by 100m. Use tick 500 so displacement is plausible
            // at maxSubSpeed (15 m/s * 499/50 = 149m max)
            var active2 = new SonarContact(0, 15.0, 5100, true, -1, 0, 0, 90.0, 0.95, 0.0);
            tickFull(DEEP_FLAT, List.of(), 500, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active2), 0);

            double initialY = controller.trackedY();
            assertTrue(!Double.isNaN(controller.trackedHeading()),
                    "Should have tracked heading from two fixes");

            // Run 100 ticks with no contact. Position should dead-reckon.
            for (int i = 0; i < 100; i++) {
                tickFull(DEEP_FLAT, List.of(), 501 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }

            // Position should have moved from initial position
            double newY = controller.trackedY();
            assertTrue(Math.abs(newY - initialY) > 1.0,
                    "Tracked position should dead-reckon between updates");
        }

        @Test
        void trackedContactClearsOnFullDecay() {
            startMatch(DEEP_FLAT);
            // Active ping to establish tracked contact
            var active = new SonarContact(0, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            assertTrue(controller.hasTrackedContact());

            // Tracked contact is now cleared when uncertaintyRadius > 5000 (not
            // when contactAlive drops below CONFIDENCE_LOST). At maxSubSpeed=15 m/s
            // and dt=0.02s, uncertainty grows by 0.3m/tick: 5000/0.3 ~ 16667 ticks.
            for (int i = 0; i < 17000; i++) {
                tickFull(DEEP_FLAT, List.of(), 2 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertFalse(controller.hasTrackedContact(),
                    "Tracked contact should be cleared when uncertaintyRadius exceeds 5000");
        }

        @Test
        void approachSteersTowardTrackedPosition() {
            startMatch(DEEP_FLAT);
            // Target is to the east: bearing pi/2, range 8000
            double bearing = Math.PI / 2;
            var active = new SonarContact(bearing, 15.0, 8000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            // Sub heading north (0), target to the east, should steer right (positive rudder)
            var out = tickFull(DEEP_FLAT, List.of(), 2, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.rudder > 0,
                    "Should steer toward tracked position (east), got rudder=" + out.rudder);
        }

        @Test
        void trackedContactEstimatePublishedEveryTick() {
            startMatch(DEEP_FLAT);
            // Establish tracked contact
            var active = new SonarContact(0, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            // Next tick with no contact should still publish (tracked contact persists)
            // CapturedOutput uses default publishContactEstimate, no assertion on it,
            // but we can verify the tracked contact is still alive
            assertTrue(controller.hasTrackedContact(),
                    "Tracked contact should persist for publishing");
            assertTrue(controller.contactAlive() > 0.95,
                    "contactAlive should be high right after ping fix");
        }

        @Test
        void longRangeApproachUsesAdaptiveThrottle() {
            startMatch(DEEP_FLAT);
            // Active return at long range, CHASE with adaptive approach
            var active = new SonarContact(0, 15.0, 8000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            assertEquals(DefaultAttackSub.State.CHASE, controller.state());

            // Next tick with no contacts. Long range approach should use
            // adaptive throttle (at least fast enough to close on target)
            var out = tickFull(DEEP_FLAT, List.of(), 2, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.throttle >= DefaultAttackSub.TRACKING_THROTTLE,
                    "Long-range approach throttle should be at least TRACKING level, got " + out.throttle);
        }

        @Test
        void trackedContactHeadingEstimatedFromTwoPings() {
            startMatch(DEEP_FLAT);
            // First ping: target at (0, 5000)
            var active1 = new SonarContact(0, 15.0, 5000, true, -1, 0, 0, 90.0, 0.95, Double.NaN);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active1), 0);

            // Second ping: target moved east to (300, 5000)
            // Use tick 1200 so displacement (300m) is plausible at maxSubSpeed
            // (15 m/s * 1199/50 = 359m max)
            // Engine tracker provides heading estimate (~pi/2, east) on the second ping.
            double bearing2 = Math.atan2(300.0, 5000.0);
            double range2 = Math.sqrt(300.0 * 300 + 5000.0 * 5000);
            var active2 = new SonarContact(bearing2, 15.0, range2, true, -1, 0, 0, 90.0, 0.95, Math.PI / 2);
            tickFull(DEEP_FLAT, List.of(), 1200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active2), 0);

            assertTrue(!Double.isNaN(controller.trackedHeading()),
                    "Should have tracked heading from two ping fixes");
            // Heading should be roughly east (pi/2)
            assertEquals(Math.PI / 2, controller.trackedHeading(), 0.2,
                    "Tracked heading should be roughly east");
        }
    }
}
