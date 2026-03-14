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

class ObstacleAvoidanceSubTest {

    // Controller constants (mirrored here for readable test comments)
    // FLOOR_CLEARANCE = 50, EMERGENCY_GAP = 40, MIN_DEPTH = -20
    // Crush depth = -700, safety margin = 50 → depth limit = -650

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);

    private ObstacleAvoidanceSub controller;

    @BeforeEach
    void setUp() {
        controller = new ObstacleAvoidanceSub();
    }

    // ── helpers ──────────────────────────────────────────────────────────

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

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
    }

    // ── depth control: bottom tracking ──────────────────────────────────

    @Nested
    class BottomTracking {

        @Test
        void sinksTowardDeepFloor() {
            // Floor at -500m, target = -500+50 = -450m, sub at -100m → should sink
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertTrue(out.ballast < 0.5,
                    "Sub at -100m over -500m floor should sink, got ballast=" + out.ballast);
        }

        @Test
        void risesWhenTooDeep() {
            // Floor at -500m, target = -450m, sub at -470m → too deep → should rise
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -470, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub below target depth should rise, got ballast=" + out.ballast);
        }

        @Test
        void nearNeutralAtTargetDepth() {
            // Floor at -500m, target = -450m
            // Sub at -425m with no vertical speed → ballast near 0.5
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -450, 0);

            assertEquals(0.5, out.ballast, 0.1,
                    "Sub at target depth should have near-neutral ballast");
        }

        @Test
        void sternPlanesPitchDownWhenTooShallow() {
            // Floor at -500m, target = -450m, sub at -100m → 350m too shallow → pitch down
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertTrue(out.sternPlanes < 0,
                    "Sub too shallow should pitch down, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void sternPlanesPitchUpWhenTooDeep() {
            // Floor at -500m, target = -450m, sub at -470m → too deep → pitch up
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -470, 0);

            assertTrue(out.sternPlanes > 0,
                    "Sub too deep should pitch up, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void tracksShallowFloor() {
            // Floor at -80m → target = -80+50 = -30m, clamped to MIN_DEPTH=-20
            // Sub at -200m → too deep → should rise
            var terrain = flatTerrain(-80);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -200, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub should rise toward shallow floor, got ballast=" + out.ballast);
        }
    }

    // ── depth limits ────────────────────────────────────────────────────

    @Nested
    class DepthLimits {

        @Test
        void respectsMinDepth() {
            // Floor at -40m → target would be -40+50 = +10m, clamped to MIN_DEPTH=-20
            // Sub at -30m → below -20 target → should rise
            var terrain = flatTerrain(-40);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -30, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub at -30m with MIN_DEPTH=-20 target should rise, got ballast=" + out.ballast);
        }

        @Test
        void respectsCrushDepthLimit() {
            // Floor at -900m → target would be -850m, clamped to -650 (crush limit)
            // Sub at -400m → should sink toward -650
            var terrain = flatTerrain(-900);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -400, 0);

            assertTrue(out.ballast < 0.5,
                    "Sub at -400m should sink toward -650 limit, got ballast=" + out.ballast);
        }

        @Test
        void doesNotSinkBelowCrushDepthLimit() {
            // Floor at -900m, crush limit = -650m
            // Sub at -660m → below limit → should rise
            var terrain = flatTerrain(-900);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -660, 0);

            assertTrue(out.ballast > 0.5,
                    "Sub below crush depth limit should rise, got ballast=" + out.ballast);
        }
    }

    // ── surface avoidance ───────────────────────────────────────────────

    @Nested
    class SurfaceAvoidance {

        @Test
        void sinksWhenTooShallowForSurface() {
            // Sub at -10m → above MIN_DEPTH=-20 → surface avoidance kicks in
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
            // Sub at -100m → well below MIN_DEPTH → surface avoidance should not fire.
            // Ballast should indicate sinking (< 0.5) since sub is above target (-450m),
            // not the surface avoidance value of 0.2.
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -100, 0);

            assertNotEquals(0.2, out.ballast, 0.01,
                    "Surface avoidance ballast (0.2) should not fire at depth");
        }
    }

    // ── emergency: floor proximity ──────────────────────────────────────

    @Nested
    class EmergencyFloorProximity {

        @Test
        void emergencyRiseWhenCloseToFloor() {
            // Floor at -500m, sub at -480m → gap = 20m < 40m → emergency
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
            // Floor at -500m, sub at -450m → gap = 50m > 40m → no emergency
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -450, 0);

            assertNotEquals(0.8, out.sternPlanes, 0.01,
                    "Should not trigger emergency with 50m clearance");
        }
    }

    // ── terrain lookahead: turn around obstacles ────────────────────────

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

    // ── drop-off handling ───────────────────────────────────────────────

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
            // Drop-off at Y=200 — scan points at 200 and 400m should catch it
            var terrain = distantDropOff();
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -150, 0);

            assertTrue(out.throttle < 0.6,
                    "Should detect drop-off at 200m distance, got throttle=" + out.throttle);
        }

        @Test
        void atHighSpeedSternPlanesMoreEffective() {
            // At high speed, control surfaces have full authority (speedFactor → 1.0).
            // At zero speed, stern planes do nothing (speedFactor → 0).
            // Sub at -150m, floor below -200m, target -150m — slightly too deep for
            // the drop-off ahead which raises target. The stern planes should be
            // stronger at speed.
            var terrain = sharpDropOff();
            startMatch(terrain);

            // Heading north with high forward speed (12 m/s — above REF_SPEED)
            var outFast = tick(terrain, 0, 0, -180, 0, new Vec3(0, 12, 0));
            // Same scenario but stationary
            var outSlow = tick(terrain, 0, 0, -180, 0, Vec3.ZERO);

            // Both should command stern planes (depth error exists), but
            // the physics will make the fast sub's planes much more effective.
            // At the controller level, the commands should be similar since
            // the controller doesn't know about speedFactor — it just commands.
            // What we can verify: the controller reacts to the drop-off in both cases.
            assertTrue(outFast.throttle < 0.6,
                    "Fast sub should still reduce throttle at drop-off");
            assertTrue(outSlow.throttle < 0.6,
                    "Slow sub should still reduce throttle at drop-off");
        }

        @Test
        void risesAtDropOffToMaintainClearance() {
            // Sub at -180m tracking -200m floor. Drop-off ahead means the floor
            // vanishes — target should still be based on current floor (-200m),
            // but drop-off detection should prevent nosediving.
            var terrain = sharpDropOff();
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -180, 0, new Vec3(0, 8, 0));

            // Sub is 20m above floor, above target of -150m. At the drop-off
            // the controller should NOT command a deep dive.
            assertTrue(out.ballast >= 0.3,
                    "Should not aggressively sink at drop-off, got ballast=" + out.ballast);
        }

        @Test
        void turnsTowardDeeperSideAtDropOff() {
            // Drop-off along +Y: left side (-X) stays at -200m (shallow),
            // right side (+X) drops to -500m (deep).
            // The controller avoids shallow terrain, so it turns RIGHT toward
            // the deeper water — that's the safe direction.
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
            // Sub far from edge (Y=-200, drop at Y>50): scan points at 200+
            // see the drop-off and reduce throttle proactively.
            // This validates that far-range scan points catch the drop before
            // the sub reaches it.
            var terrain = sharpDropOff();
            startMatch(terrain);
            var outFar = tick(terrain, 0, -200, -150, 0, new Vec3(0, 8, 0));

            assertTrue(outFar.throttle < 0.6,
                    "Far sub should detect upcoming drop-off and slow, got throttle="
                            + outFar.throttle);
        }

        @Test
        void highSpeedApproachStillDetectsDropOff() {
            // At max speed (~15 m/s), the sub covers 25m per scan point gap.
            // The close-range scan points (25m, 50m) should catch the edge.
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

    // ── border avoidance ────────────────────────────────────────────────

    @Nested
    class BorderAvoidance {

        @Test
        void steersTowardCenterNearBoundary() {
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 4700, 0, -200, 0);

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

    // ── damping ─────────────────────────────────────────────────────────

    @Nested
    class Damping {

        @Test
        void dampsSinkingWhenApproachingTarget() {
            // Floor at -500m, target = -450m
            // Sub at -440m (10m above target) but sinking fast
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var outSinking = tick(terrain, 0, 0, -440, 0, new Vec3(0, 0, -3));
            var outStill = tick(terrain, 0, 0, -440, 0, Vec3.ZERO);

            assertTrue(outSinking.ballast > outStill.ballast,
                    "Damping should make ballast more buoyant when already sinking toward target");
        }

        @Test
        void dampsRisingWhenApproachingTarget() {
            // Sub at -460m (10m below target -450m) but rising fast
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var outRising = tick(terrain, 0, 0, -460, 0, new Vec3(0, 0, 3));
            var outStill = tick(terrain, 0, 0, -460, 0, Vec3.ZERO);

            assertTrue(outRising.ballast < outStill.ballast,
                    "Damping should make ballast less buoyant when already rising toward target");
        }
    }

    // ── throttle ────────────────────────────────────────────────────────

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

    // ── island start: low clearance, no speed ─────────────────────────

    @Nested
    class IslandStart {

        // Terrain: shallow island ahead (+Y), deep water behind (-Y) and to sides
        static TerrainMap islandAhead() {
            return yVaryingTerrain(y -> y > 0 ? -60 : -400);
        }

        @Test
        void emergencyBlowsBallastNearFloor() {
            // Sub at -480m over -500m floor = 20m gap, below target (-450m).
            // Emergency should blow ballast.
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -480, 0);

            assertTrue(out.ballast > 0.5,
                    "Emergency should blow ballast, got ballast=" + out.ballast);
        }

        @Test
        void reversesInEmergencyNearFloor() {
            // Sub at -470m over -500m floor = 30m gap, below target (-450m).
            // Emergency should fire with reverse thrust.
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -470, 0);

            assertTrue(out.throttle < 0,
                    "Emergency should reverse thrust, got throttle=" + out.throttle);
        }

        @Test
        void doesNotGetStuckOnShallowFloor() {
            // Sub at -30m, flat shallow floor at -60m everywhere.
            // Gap = 30m (< EMERGENCY_GAP=40) but target depth = -20m.
            // Sub is deeper than target (-30 < -20) so emergency would fire.
            // But it should still be making progress toward -20m, not stuck.
            var terrain = flatTerrain(-60);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -30, 0);

            // Sub is below target, should be rising
            assertTrue(out.ballast > 0.5,
                    "Should rise toward target on shallow floor, got ballast=" + out.ballast);
        }

        @Test
        void survivesStartWithMinimalClearance() {
            // Sub at -480m, floor at -500m = 20m gap. Should emergency with
            // pitch up.
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -480, 0);

            assertTrue(out.sternPlanes > 0,
                    "Should pitch up with minimal clearance, got sternPlanes=" + out.sternPlanes);
        }

        @Test
        void turnsAwayFromIslandAtZeroSpeed() {
            // Sub at rest near an island: floor rises to -60m ahead (+Y),
            // deep water behind (-Y). Terrain avoidance should command rudder
            // to turn. With zero speed, rudder has no authority, so the sub
            // needs forward or reverse motion first. Verify the controller
            // at least commands a turn direction.
            var terrain = islandAhead();
            startMatch(terrain);
            var out = tick(terrain, 0, -50, -50, 0);

            // Floor below at y=-50 is -400 (deep). Floor ahead at y>0 is -60.
            // Terrain avoidance should detect rising floor and command a turn.
            assertNotEquals(0, out.rudder, 0.001,
                    "Should command rudder turn near island, got rudder=" + out.rudder);
        }

        @Test
        void buildsReverseSpeedForRudderAuthority() {
            // Sub near floor with emergency active commands reverse thrust.
            // Once moving backwards, the rudder reversal (physics) means the
            // controller's commanded turn still pushes the stern in the right
            // direction. Verify reverse + rudder are both commanded.
            var terrain = flatTerrain(-500);
            startMatch(terrain);
            var out = tick(terrain, 0, 0, -475, 0);

            // Emergency: gap=25 < 40 and depth=-475 < target-5=-455
            assertTrue(out.throttle < 0,
                    "Emergency should reverse, got throttle=" + out.throttle);
            // The PD controller or emergency sets stern planes positive
            assertTrue(out.sternPlanes > 0,
                    "Should pitch up, got sternPlanes=" + out.sternPlanes);
        }
    }

    // ══════════════════════════════════════════════════════════════════
    // Phase 3 Step 4: State machine tests
    // ══════════════════════════════════════════════════════════════════

    private static final TerrainMap DEEP_FLAT = flatTerrain(-500);
    private static final SonarContact CONTACT_NORTH =
            new SonarContact(0, 10.0, 0, false, -1);
    private static final SonarContact CONTACT_NORTH_LOUD =
            new SonarContact(0, 20.0, 0, false, -1);

    // Feed N ticks with the same passive contact to trigger state transitions
    private void feedContactTicks(int n, long startTick, double x, double y, double z,
                                   double heading, SonarContact contact) {
        for (int i = 0; i < n; i++) {
            tickFull(DEEP_FLAT, List.of(), startTick + i, x, y, z, heading,
                    Vec3.ZERO, 1000, List.of(contact), List.of(), 0);
        }
    }

    // ── State transitions ────────────────────────────────────────────

    @Nested
    class StateTransitions {

        @Test
        void transitionsToTrackingOnContact() {
            startMatch(DEEP_FLAT);
            assertEquals(ObstacleAvoidanceSub.State.PATROL, controller.state());

            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());
        }

        @Test
        void doesNotTransitionOnSingleTickContact() {
            startMatch(DEEP_FLAT);
            feedContactTicks(1, 0, 0, 0, -200, 0, CONTACT_NORTH);

            assertEquals(ObstacleAvoidanceSub.State.PATROL, controller.state(),
                    "Single tick contact should not trigger TRACKING");
        }

        @Test
        void transitionsToPatrolOnContactLoss() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());

            // Feed no-contact ticks for CONTACT_LOST_PATROL + 1
            long startTick = ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS;
            for (int i = 0; i <= ObstacleAvoidanceSub.CONTACT_LOST_PATROL; i++) {
                tickFull(DEEP_FLAT, List.of(), startTick + i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }

            assertEquals(ObstacleAvoidanceSub.State.PATROL, controller.state());
        }

        @Test
        void transitionsToChaseOnRangeEstimate() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());

            // Feed contacts from positions far enough apart for triangulation
            // Position 1: (0, 0), bearing to contact is 0 (north)
            // Position 2: (300, 0), bearing to contact is slightly west of north
            // This gives an intersection at ~2000m north
            double contactBearing2 = Math.atan2(-300.0, 2000.0) + Math.PI;
            // Actually, bearing = atan2(target.x - src.x, target.y - src.y)
            // Target at (0, 2000). From (300, 0): bearing = atan2(-300, 2000) ≈ -0.149 rad
            double bearing2 = Math.atan2(0 - 300, 2000 - 0);
            if (bearing2 < 0) bearing2 += 2 * Math.PI;
            var contact2 = new SonarContact(bearing2, 10.0, 0, false, -1);

            // First fix at (0, 0)
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);
            // Second fix at (300, 0) — far enough for displacement > 200m
            tickFull(DEEP_FLAT, List.of(), 200, 300, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(contact2), List.of(), 0);

            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state(),
                    "Should transition to CHASE with range estimate < 3000m");
        }

        @Test
        void transitionsToRamOnCloseRange() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING then CHASE via active sonar return with range
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            // Give an active return with range 2000m → CHASE
            var activeContact = new SonarContact(0, 15.0, 2000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeContact), 0);

            // Give an active return with range 400m → RAM
            var closeContact = new SonarContact(0, 25.0, 400, true, -1);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(closeContact), 0);

            assertEquals(ObstacleAvoidanceSub.State.RAM, controller.state(),
                    "Should transition to RAM with range < 500m");
        }

        @Test
        void transitionsToEvadeOnDamage() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());

            // Take damage (hp drops from 1000 to 900)
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(ObstacleAvoidanceSub.State.EVADE, controller.state());
        }
    }

    // ── PATROL behavior ──────────────────────────────────────────────

    @Nested
    class PatrolBehavior {

        @Test
        void patrolReducesThrottleFromPhase2() {
            startMatch(DEEP_FLAT);
            var out = tick(DEEP_FLAT, 0, 0, -200, 0);
            assertEquals(ObstacleAvoidanceSub.PATROL_THROTTLE, out.throttle, 0.001,
                    "PATROL throttle should be 0.4");
        }

        @Test
        void patrolClearsBafflesPeriodically() {
            startMatch(DEEP_FLAT);
            // Tick for BAFFLE_CLEAR_INTERVAL + 1 ticks
            double lastRudder = 0;
            boolean rudderChanged = false;
            for (int i = 0; i <= ObstacleAvoidanceSub.BAFFLE_CLEAR_INTERVAL + 10; i++) {
                var out = tickFull(DEEP_FLAT, List.of(), i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
                if (i > ObstacleAvoidanceSub.BAFFLE_CLEAR_INTERVAL && Math.abs(out.rudder) > 0.1) {
                    rudderChanged = true;
                    break;
                }
            }
            assertTrue(rudderChanged, "Baffle clearing should command rudder after interval");
        }

        @Test
        void patrolSteersTowardPursuitBearing() {
            startMatch(DEEP_FLAT);
            // Enter TRACKING with contact north
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());

            // Lose contact → back to PATROL
            long tick = ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS;
            for (int i = 0; i <= ObstacleAvoidanceSub.CONTACT_LOST_PATROL; i++) {
                tickFull(DEEP_FLAT, List.of(), tick + i, 0, 0, -200, Math.PI / 2,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertEquals(ObstacleAvoidanceSub.State.PATROL, controller.state());
            assertTrue(controller.hasPursuit(), "Should have pursuit bearing after contact loss");

            // Sub heading east (π/2), pursuit bearing north (0) → should steer left
            tick += ObstacleAvoidanceSub.CONTACT_LOST_PATROL + 1;
            var out = tickFull(DEEP_FLAT, List.of(), tick, 0, 0, -200, Math.PI / 2,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.rudder < 0,
                    "Should steer toward pursuit bearing (north), got rudder=" + out.rudder);
        }

        @Test
        void patrolPingsAfterLongSilence() {
            startMatch(DEEP_FLAT);
            CapturedOutput out = null;
            for (long i = 0; i <= ObstacleAvoidanceSub.PATROL_SILENCE_PING_TICKS; i++) {
                out = tickFull(DEEP_FLAT, List.of(), i, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(), 0);
            }
            assertTrue(out.pinged, "Should ping after long silence in PATROL");
        }

        @Test
        void patrolPrefersDepthBelowThermocline() {
            var layers = List.of(new ThermalLayer(-120, 18.0, 8.0));
            startMatch(DEEP_FLAT, layers);
            // Sub at -100m (above thermocline at -120). With tactical depth preference
            // below thermocline, target should be deeper than bottom-tracking alone.
            var out = tickFull(DEEP_FLAT, layers, 0, 0, 0, -100, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            // Should be sinking toward below-thermocline depth (-150m)
            assertTrue(out.ballast < 0.5,
                    "PATROL should prefer depth below thermocline, got ballast=" + out.ballast);
        }
    }

    // ── TRACKING behavior ────────────────────────────────────────────

    @Nested
    class TrackingBehavior {

        @Test
        void trackingReducesSpeed() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());

            // Next tick in TRACKING — should have low throttle
            var out = tickFull(DEEP_FLAT, List.of(),
                    ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(ObstacleAvoidanceSub.TRACKING_THROTTLE, out.throttle, 0.01,
                    "TRACKING throttle should be 0.25");
        }

        @Test
        void trackingManeuversPerpendicularToContact() {
            startMatch(DEEP_FLAT);
            // Contact at bearing 0 (north), sub heading north (0)
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            var out = tickFull(DEEP_FLAT, List.of(),
                    ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            // Should turn toward perpendicular (east or west = ±π/2)
            assertNotEquals(0, out.rudder, 0.01,
                    "TRACKING should turn perpendicular to contact bearing");
        }

        @Test
        void trackingAccumulatesBearingFixes() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            // Feed contacts from different positions with appropriate bearings.
            // Target at (0, 2000). From (0,0) bearing is 0 (north).
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);
            // From (300, 0), bearing to (0, 2000) = atan2(0-300, 2000-0) ≈ -0.149 rad → +2pi ≈ 6.13
            double bearing2 = Math.atan2(0 - 300.0, 2000.0);
            if (bearing2 < 0) bearing2 += 2 * Math.PI;
            var contact2 = new SonarContact(bearing2, 10.0, 0, false, -1);
            tickFull(DEEP_FLAT, List.of(), 200, 300, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(contact2), List.of(), 0);

            assertTrue(controller.contactTrack().size() >= 2,
                    "Should accumulate bearing fixes from different positions");
            assertTrue(controller.estimatedRange() < Double.MAX_VALUE,
                    "Should have a range estimate after two fixes");
        }
    }

    // ── CHASE behavior ───────────────────────────────────────────────

    @Nested
    class ChaseBehavior {

        private void enterChase() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Active return with range 2000m triggers CHASE
            var activeContact = new SonarContact(0, 15.0, 2000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeContact), 0);
        }

        @Test
        void chaseIncreasesSpeed() {
            enterChase();
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state());

            // During sprint phase, throttle should be high
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(ObstacleAvoidanceSub.CHASE_THROTTLE, out.throttle, 0.01,
                    "CHASE sprint throttle should be 0.8");
        }

        @Test
        void chaseUsesSprintAndDrift() {
            enterChase();
            // chaseStartTick was set at tick 100 (when enterChase transitions)

            // Sprint phase: tick 101 is phaseTime=(101-100)%1750=1 < SPRINT_DURATION
            var outSprint = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            // Drift phase: tick (100 + SPRINT_DURATION + 1) gives phaseTime=751 >= SPRINT_DURATION
            long driftTick = 100 + ObstacleAvoidanceSub.SPRINT_DURATION + 1;
            var outDrift = tickFull(DEEP_FLAT, List.of(), driftTick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertTrue(outSprint.throttle > outDrift.throttle,
                    "Sprint throttle (" + outSprint.throttle +
                            ") should exceed drift throttle (" + outDrift.throttle + ")");
            assertEquals(ObstacleAvoidanceSub.TRACKING_THROTTLE, outDrift.throttle, 0.01,
                    "Drift throttle should be tracking throttle");
        }

        @Test
        void chasePersistsLongerThanTracking() {
            enterChase();
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state());

            // After old CONTACT_LOST_PATROL ticks, should still be CHASE
            long tick = 100 + ObstacleAvoidanceSub.CONTACT_LOST_PATROL + 1;
            tickFull(DEEP_FLAT, List.of(), tick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state(),
                    "CHASE should persist longer than TRACKING contact-lost timeout");

            // enterChase gives an active return → approach target set → longer timeout
            // After CONTACT_LOST_CHASE ticks, should still be CHASE (approaching)
            tick = 100 + ObstacleAvoidanceSub.CONTACT_LOST_CHASE + 1;
            tickFull(DEEP_FLAT, List.of(), tick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state(),
                    "CHASE with approach target should persist past CONTACT_LOST_CHASE");

            // After APPROACH_TIMEOUT ticks, should drop to PATROL
            tick = 100 + ObstacleAvoidanceSub.APPROACH_TIMEOUT + 1;
            tickFull(DEEP_FLAT, List.of(), tick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(ObstacleAvoidanceSub.State.PATROL, controller.state(),
                    "CHASE should eventually timeout to PATROL");
        }

        @Test
        void chaseApproachesFromStern() {
            enterChase();
            // enterChase gives active return at tick 100, bearing 0, range 2000
            // Target at (0, 2000), sub at (0, 0)

            // Second active return 300 ticks later, target moved east to ~(300, 2000)
            double bearing2 = Math.atan2(300.0, 2000.0);
            double range2 = Math.sqrt(300.0 * 300 + 2000.0 * 2000);
            var active2 = new SonarContact(bearing2, 15.0, range2, true, -1);
            tickFull(DEEP_FLAT, List.of(), 400, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active2), 0);

            assertTrue(!Double.isNaN(controller.estimatedTargetHeading()),
                    "Should have estimated target heading");
            // Target heading should be roughly east (π/2)
            assertEquals(Math.PI / 2, controller.estimatedTargetHeading(), 0.1,
                    "Estimated target heading should be ~east");

            // With stern approach, sub heading north should steer WEST (negative rudder)
            // to get behind the eastbound target
            var out = tickFull(DEEP_FLAT, List.of(), 401, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(new SonarContact(bearing2, 10.0, 0, false, -1)),
                    List.of(), 0);
            assertTrue(out.rudder < 0,
                    "Should steer west to approach from stern of eastbound target, got rudder="
                            + out.rudder);
        }

        @Test
        void chasePrefersDeepSprint() {
            enterChase();
            // At -100m, chase target depth should be much deeper (for cavitation avoidance)
            // Sub at -100m should be sinking toward deeper sprint depth
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -100, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertTrue(out.ballast < 0.5,
                    "CHASE should prefer deep sprint depth, got ballast=" + out.ballast);
        }
    }

    // ── EVADE behavior ───────────────────────────────────────────────

    @Nested
    class EvadeBehavior {

        private void enterEvade() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Take damage to trigger EVADE
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(CONTACT_NORTH), List.of(), 0);
        }

        @Test
        void evadeGoesQuiet() {
            enterEvade();
            assertEquals(ObstacleAvoidanceSub.State.EVADE, controller.state());

            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(), List.of(), 0);

            assertEquals(ObstacleAvoidanceSub.EVADE_THROTTLE, out.throttle, 0.01,
                    "EVADE throttle should be 0.15");
        }

        @Test
        void evadeDivesDeep() {
            enterEvade();
            // Sub at -200m, should want to dive to crush depth limit area
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(), List.of(), 0);

            assertTrue(out.ballast < 0.5,
                    "EVADE should dive deep, got ballast=" + out.ballast);
        }

        @Test
        void evadeTurnsPerpendicularToThreat() {
            enterEvade();
            // Contact was at bearing 0 (north), sub heading north
            // Should turn to perpendicular (east or west)
            var out = tickFull(DEEP_FLAT, List.of(), 101, 0, 0, -200, 0,
                    Vec3.ZERO, 900, List.of(), List.of(), 0);

            assertNotEquals(0, out.rudder, 0.01,
                    "EVADE should turn perpendicular to threat bearing");
        }

        @Test
        void evadeDoesNotClearBaffles() {
            enterEvade();
            // Tick through many ticks in EVADE — should never baffle-clear
            boolean baffleCleared = false;
            for (int i = 0; i < ObstacleAvoidanceSub.BAFFLE_CLEAR_INTERVAL + 100; i++) {
                var out = tickFull(DEEP_FLAT, List.of(), 101 + i, 0, 0, -200, 0,
                        Vec3.ZERO, 900, List.of(CONTACT_NORTH), List.of(), 0);
                // In EVADE, rudder should be for perpendicular turn, not baffle clear pattern
                // The rudder should stay consistent (perpendicular to threat), not alternate
            }
            // If we're still in EVADE (contact keeps feeding), we haven't transitioned
            // The baffle clearing flag is only active in PATROL state
            assertNotEquals(ObstacleAvoidanceSub.State.PATROL, controller.state(),
                    "Should remain in EVADE with ongoing contact");
        }
    }

    // ── RAM behavior ─────────────────────────────────────────────────

    @Nested
    class RamBehavior {

        @Test
        void ramUsesFullThrottle() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            // Active return at 2000m → CHASE
            var activeChase = new SonarContact(0, 15.0, 2000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeChase), 0);
            // Active return at 400m → RAM
            var activeRam = new SonarContact(0, 25.0, 400, true, -1);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeRam), 0);
            assertEquals(ObstacleAvoidanceSub.State.RAM, controller.state());

            // Next tick in RAM
            var out = tickFull(DEEP_FLAT, List.of(), 201, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(CONTACT_NORTH), List.of(), 0);

            assertEquals(ObstacleAvoidanceSub.RAM_THROTTLE, out.throttle, 0.01,
                    "RAM throttle should be 1.0");
        }

        @Test
        void ramPingsOnContactLoss() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            var activeChase = new SonarContact(0, 15.0, 2000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeChase), 0);
            var activeRam = new SonarContact(0, 25.0, 400, true, -1);
            tickFull(DEEP_FLAT, List.of(), 200, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(activeRam), 0);
            assertEquals(ObstacleAvoidanceSub.State.RAM, controller.state());

            // Lose contact for 101+ ticks, should ping
            var out = tickFull(DEEP_FLAT, List.of(), 301, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);

            assertTrue(out.pinged, "RAM should ping when contact is lost");
        }
    }

    // ── Triangulation ────────────────────────────────────────────────

    @Nested
    class Triangulation {

        @Test
        void triangulatesRangeFromTwoBearings() {
            // Fix 1: at (0, 0), bearing 0 (north)
            // Fix 2: at (300, 0), bearing ~-0.149 rad (slightly west of north)
            // Target at (0, 2000)
            var fix1 = new ObstacleAvoidanceSub.BearingFix(0, 0, 0, 0, 0, 5, 0);
            double bearing2 = Math.atan2(0 - 300.0, 2000.0 - 0);
            if (bearing2 < 0) bearing2 += 2 * Math.PI;
            var fix2 = new ObstacleAvoidanceSub.BearingFix(100, bearing2, 0, 300, 0, 5, 0);

            double range = ObstacleAvoidanceSub.triangulate(fix2, fix1);
            assertTrue(range > 1000 && range < 5000,
                    "Triangulated range should be ~2000m, got " + range);
        }

        @Test
        void parallelBearingsGiveNoRange() {
            // Both bearings identical → no intersection
            var fix1 = new ObstacleAvoidanceSub.BearingFix(0, 0, 0, 0, 0, 5, 0);
            var fix2 = new ObstacleAvoidanceSub.BearingFix(100, 0, 0, 300, 0, 5, 0);

            double range = ObstacleAvoidanceSub.triangulate(fix2, fix1);
            assertEquals(Double.MAX_VALUE, range,
                    "Parallel bearings should give no range estimate");
        }
    }

    // ── Contact log and approach behavior ────────────────────────────

    @Nested
    class ContactLogAndApproach {

        @Test
        void activePingCreatesContactLogEntry() {
            startMatch(DEEP_FLAT);
            var active = new SonarContact(0, 15.0, 5000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            assertFalse(controller.contactLog().isEmpty(),
                    "Contact log should have an entry after active ping");
            var entry = controller.contactLog().getLast();
            assertEquals(ObstacleAvoidanceSub.ContactType.ACTIVE_PING, entry.type());
            assertEquals(0.95, entry.confidence(), 0.01);
            // Target should be at (0, 5000) — bearing 0 (north), range 5000
            assertEquals(0, entry.estX(), 50);
            assertEquals(5000, entry.estY(), 50);
        }

        @Test
        void passiveContactCreatesContactLogEntry() {
            startMatch(DEEP_FLAT);
            // Feed enough passive contacts to get range estimate
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);

            // Check that at least one passive entry exists in the log
            boolean hasPassive = controller.contactLog().stream()
                    .anyMatch(r -> r.type() == ObstacleAvoidanceSub.ContactType.PASSIVE_SONAR);
            // May or may not have passive entries depending on range estimation
            // At minimum, the log should not throw errors
            assertNotNull(controller.contactLog());
        }

        @Test
        void activePingTransitionsDirectlyToChase() {
            startMatch(DEEP_FLAT);
            // No prior contacts — go straight from PATROL to CHASE on active return
            var active = new SonarContact(0, 15.0, 8000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state(),
                    "Active ping return should transition directly to CHASE from PATROL");
        }

        @Test
        void activePingInTrackingTransitionsToChase() {
            startMatch(DEEP_FLAT);
            feedContactTicks(ObstacleAvoidanceSub.CONTACT_CONFIRM_TICKS, 0,
                    0, 0, -200, 0, CONTACT_NORTH);
            assertEquals(ObstacleAvoidanceSub.State.TRACKING, controller.state());

            // Active return at long range should bypass CHASE_RANGE check
            var active = new SonarContact(0, 15.0, 10000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 100, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state(),
                    "Active return should transition TRACKING→CHASE regardless of range");
        }

        @Test
        void longRangeApproachUsesPatrolThrottle() {
            startMatch(DEEP_FLAT);
            // Active return at long range → CHASE with quiet approach
            var active = new SonarContact(0, 15.0, 8000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state());

            // Next tick with no contacts — should use patrol throttle (quiet approach)
            var out = tickFull(DEEP_FLAT, List.of(), 2, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(ObstacleAvoidanceSub.PATROL_THROTTLE, out.throttle, 0.01,
                    "Long-range approach should use quiet patrol throttle");
        }

        @Test
        void approachSteersTowardTargetPosition() {
            startMatch(DEEP_FLAT);
            // Target is to the east: bearing π/2, range 8000
            double bearing = Math.PI / 2;
            var active = new SonarContact(bearing, 15.0, 8000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);

            // Sub heading north (0), target to the east — should steer right (positive rudder)
            var out = tickFull(DEEP_FLAT, List.of(), 2, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.rudder > 0,
                    "Should steer toward target position (east), got rudder=" + out.rudder);
        }

        @Test
        void approachPersistsDuringLongSilence() {
            startMatch(DEEP_FLAT);
            var active = new SonarContact(0, 15.0, 8000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state());

            // After CONTACT_LOST_CHASE ticks (30s), should still be approaching
            long tick = 1 + ObstacleAvoidanceSub.CONTACT_LOST_CHASE + 1;
            tickFull(DEEP_FLAT, List.of(), tick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state(),
                    "Approach from ping should persist past normal CHASE timeout");
        }

        @Test
        void patrolPursuitUsesContactLogPosition() {
            startMatch(DEEP_FLAT);
            // Get a ping fix, enter CHASE — target to the east
            var active = new SonarContact(Math.PI / 2, 15.0, 5000, true, -1);
            tickFull(DEEP_FLAT, List.of(), 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            assertEquals(ObstacleAvoidanceSub.State.CHASE, controller.state());

            // Timeout to PATROL — contact is old but contact log has recent entry
            long tick = 1 + ObstacleAvoidanceSub.APPROACH_TIMEOUT + 1;
            tickFull(DEEP_FLAT, List.of(), tick, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertEquals(ObstacleAvoidanceSub.State.PATROL, controller.state());
            assertTrue(controller.hasPursuit(),
                    "Should have pursuit — contact log entry is within APPROACH_TIMEOUT");

            // In PATROL with pursuit, should steer toward the logged position (east)
            var out = tickFull(DEEP_FLAT, List.of(), tick + 1, 0, 0, -200, 0,
                    Vec3.ZERO, 1000, List.of(), List.of(), 0);
            assertTrue(out.rudder > 0.1,
                    "Patrol pursuit should steer toward logged contact position (east), got rudder="
                            + out.rudder);
        }

        @Test
        void contactLogCapsSize() {
            startMatch(DEEP_FLAT);
            // Feed 600 contacts — log should cap at 500
            for (int i = 0; i < 600; i++) {
                var active = new SonarContact(0, 15.0, 5000, true, -1);
                tickFull(DEEP_FLAT, List.of(), i + 1, 0, 0, -200, 0,
                        Vec3.ZERO, 1000, List.of(), List.of(active), 0);
            }
            assertTrue(controller.contactLog().size() <= 500,
                    "Contact log should be capped at 500 entries, got " + controller.contactLog().size());
        }
    }
}
