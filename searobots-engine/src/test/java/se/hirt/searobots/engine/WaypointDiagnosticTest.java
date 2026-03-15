/*
 * Diagnostic test: Why is DefaultAttackSub not following waypoints?
 * Theory: terrain avoidance (Step 6) overrides tactical rudder too aggressively.
 */
package se.hirt.searobots.engine;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class WaypointDiagnosticTest {

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);

    private DefaultAttackSub controller;

    @BeforeEach
    void setUp() {
        controller = new DefaultAttackSub();
    }

    // ── Helpers ──────────────────────────────────────────────────────

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

    record TestInputFull(long tick, double deltaTimeSeconds,
                         SubmarineState self, EnvironmentSnapshot environment,
                         List<SonarContact> sonarContacts, List<SonarContact> activeSonarReturns,
                         int activeSonarCooldownTicks)
            implements SubmarineInput {}

    static final class DiagOutput implements SubmarineOutput {
        double rudder, sternPlanes, throttle, ballast;
        boolean pinged;
        String status = "";
        final ArrayList<Waypoint> waypoints = new ArrayList<>();

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void publishWaypoint(Waypoint wp) { waypoints.add(wp); }
        @Override public void setStatus(String s) { status = s; }
    }

    private DiagOutput tickAt(TerrainMap terrain, long tick, double x, double y, double z,
                               double heading, Vec3 linearVelocity) {
        var pose = new Pose(new Vec3(x, y, z), heading, 0, 0);
        var velocity = new Velocity(linearVelocity, Vec3.ZERO);
        var state = new SubmarineState(pose, velocity, 1000, 0);
        var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
        var input = new TestInputFull(tick, 0.02, state, env, List.of(), List.of(), 0);
        var output = new DiagOutput();
        controller.onTick(input, output);
        return output;
    }

    // ── Diagnostic 1: Deep flat terrain (-500m), no obstacles ────────

    @Test
    void diagnostic_deepFlatTerrain_waypointFollowing() {
        var terrain = flatTerrain(-500);
        startMatch(terrain);

        // Sub at (0,0), heading north (0 rad), at depth -100m
        // Floor = -500m, target = -500+80 = -420m
        // worstFloor = -500, floorClearance = 80, margin = depth - (worstFloor + floorClearance)
        // margin = -100 - (-500 + 80) = -100 - (-420) = 320
        // 320 > 80, so terrain avoidance should NOT fire at -100m.
        // But what about scan distances? All flat, so worstFloor stays -500.

        double x = 0, y = 0, z = -100;
        double heading = 0; // north
        double speed = 6.0; // patrol speed

        System.out.println("=== DIAGNOSTIC 1: Deep flat terrain (-500m), sub at z=-100m ===");
        System.out.println("Floor=-500, floorClearance=80, target=-420");
        System.out.println("margin = depth - (worstFloor + floorClearance) = z - (-420)");
        System.out.println("Terrain avoidance fires when margin < 80");
        System.out.println();
        System.out.printf("%-6s %-12s %-12s %-8s %-8s %-30s %-10s %-10s %-10s%n",
                "Tick", "X", "Y", "Heading", "Rudder", "Status", "WP_X", "WP_Y", "BearErr");

        int terrainAvoidCount = 0;
        int borderAvoidCount = 0;
        int correctSteeringCount = 0;
        int totalChecks = 0;

        for (int tick = 0; tick < 1000; tick++) {
            Vec3 vel = new Vec3(speed * Math.sin(heading), speed * Math.cos(heading), 0);
            var out = tickAt(terrain, tick, x, y, z, heading, vel);

            if (tick % 50 == 0) {
                Waypoint activeWp = null;
                for (var wp : out.waypoints) {
                    if (wp.active()) { activeWp = wp; break; }
                }

                double bearingErr = Double.NaN;
                String wpInfo = "none";
                if (activeWp != null) {
                    double bearingToWp = Math.atan2(activeWp.x() - x, activeWp.y() - y);
                    if (bearingToWp < 0) bearingToWp += 2 * Math.PI;
                    bearingErr = bearingToWp - heading;
                    while (bearingErr > Math.PI) bearingErr -= 2 * Math.PI;
                    while (bearingErr < -Math.PI) bearingErr += 2 * Math.PI;
                    wpInfo = String.format("(%.0f,%.0f)", activeWp.x(), activeWp.y());

                    totalChecks++;
                    // Check if rudder steers toward waypoint
                    boolean aligned = Math.abs(bearingErr) < Math.toRadians(10);
                    boolean rudderCorrect = (bearingErr > 0 && out.rudder > 0)
                            || (bearingErr < 0 && out.rudder < 0)
                            || Math.abs(out.rudder) < 0.05;
                    if (aligned || rudderCorrect) correctSteeringCount++;
                }

                double margin = z - (-500 + 80); // z - (-420) for flat -500
                System.out.printf("%-6d %-12.1f %-12.1f %-8.1f %-8.3f %-30s %-10s %-10.1f margin=%.0f%n",
                        tick, x, y, Math.toDegrees(heading), out.rudder,
                        out.status, wpInfo,
                        Double.isNaN(bearingErr) ? 0 : Math.toDegrees(bearingErr),
                        margin);

                if (out.status.contains("/A")) terrainAvoidCount++;
                if (out.status.contains("BORDER") || out.status.contains("/B")) borderAvoidCount++;
            }

            // Simple kinematics: update position/heading based on rudder output
            // (very rough, but enough for diagnostic)
            heading += out.rudder * 0.02; // turn rate
            while (heading < 0) heading += 2 * Math.PI;
            while (heading >= 2 * Math.PI) heading -= 2 * Math.PI;
            x += speed * Math.sin(heading) * 0.02;
            y += speed * Math.cos(heading) * 0.02;
        }

        System.out.println();
        System.out.println("Terrain avoidance active: " + terrainAvoidCount + " / " + (1000/50) + " samples");
        System.out.println("Border avoidance active: " + borderAvoidCount + " / " + (1000/50) + " samples");
        System.out.println("Correct steering: " + correctSteeringCount + " / " + totalChecks);

        // At -100m over -500m floor, margin = 320 >> 80, so terrain avoidance should NOT fire
        assertEquals(0, terrainAvoidCount,
                "Terrain avoidance should NOT fire at z=-100 over -500m floor (margin=320)");
        assertTrue(totalChecks > 0, "Should have waypoints to check");
        double ratio = (double) correctSteeringCount / totalChecks;
        assertTrue(ratio > 0.5,
                "Rudder should steer toward waypoints at least 50% of samples, got "
                        + String.format("%.0f%%", ratio * 100));
    }

    // ── Diagnostic 2: Moderate terrain (-200m floor) ─────────────────

    @Test
    void diagnostic_moderateTerrain_terrainAvoidanceOverride() {
        var terrain = flatTerrain(-200);
        startMatch(terrain);

        // Floor = -200m, target = -200+80 = -120m
        // Sub at z = -100m (a normal operating depth)
        // margin = -100 - (-200 + 80) = -100 - (-120) = 20
        // 20 < 80 => TERRAIN AVOIDANCE FIRES!
        // urgency = clamp(1 - 20/80, 0.2, 1.0) = clamp(0.75, 0.2, 1.0) = 0.75
        //
        // So at a perfectly normal depth of -100m over a -200m floor, terrain avoidance
        // fires with 75% urgency, nearly wiping out the waypoint rudder.

        double x = 0, y = 0, z = -100;
        double heading = 0;
        double speed = 6.0;

        System.out.println();
        System.out.println("=== DIAGNOSTIC 2: Moderate terrain (-200m), sub at z=-100m ===");
        System.out.println("Floor=-200, floorClearance=80, target=-120");
        System.out.println("margin = -100 - (-120) = 20");
        System.out.println("20 < 80 => TERRAIN AVOIDANCE FIRES");
        System.out.println("urgency = clamp(1 - 20/80, 0.2, 1.0) = 0.75");
        System.out.println("rudder = tacticalRudder * (1-0.75) + avoidRudder * 0.75");
        System.out.println("       = tacticalRudder * 0.25 + avoidRudder * 0.75");
        System.out.println("Waypoint steering is reduced to 25% weight!");
        System.out.println();
        System.out.printf("%-6s %-12s %-12s %-8s %-8s %-8s %-30s %-10s %-10s%n",
                "Tick", "X", "Y", "Heading", "Rudder", "Throtl", "Status", "WP_X,Y", "BearErr");

        int terrainAvoidCount = 0;
        int totalSamples = 0;

        for (int tick = 0; tick < 1000; tick++) {
            Vec3 vel = new Vec3(speed * Math.sin(heading), speed * Math.cos(heading), 0);
            var out = tickAt(terrain, tick, x, y, z, heading, vel);

            if (tick % 50 == 0) {
                totalSamples++;
                Waypoint activeWp = null;
                for (var wp : out.waypoints) {
                    if (wp.active()) { activeWp = wp; break; }
                }

                double bearingErr = Double.NaN;
                String wpInfo = "none";
                if (activeWp != null) {
                    double bearingToWp = Math.atan2(activeWp.x() - x, activeWp.y() - y);
                    if (bearingToWp < 0) bearingToWp += 2 * Math.PI;
                    bearingErr = bearingToWp - heading;
                    while (bearingErr > Math.PI) bearingErr -= 2 * Math.PI;
                    while (bearingErr < -Math.PI) bearingErr += 2 * Math.PI;
                    wpInfo = String.format("(%.0f,%.0f)", activeWp.x(), activeWp.y());
                }

                double margin = z - (-200 + 80);
                double urgency = Math.max(0.2, Math.min(1.0, 1.0 - margin / 80.0));

                System.out.printf("%-6d %-12.1f %-12.1f %-8.1f %-8.3f %-8.2f %-30s %-10s %-10.1f urg=%.2f%n",
                        tick, x, y, Math.toDegrees(heading), out.rudder, out.throttle,
                        out.status, wpInfo,
                        Double.isNaN(bearingErr) ? 0 : Math.toDegrees(bearingErr),
                        urgency);

                // P/A = PATROL with AVOIDING TERRAIN override active
                if (out.status.contains("/A")) terrainAvoidCount++;
            }

            heading += out.rudder * 0.02;
            while (heading < 0) heading += 2 * Math.PI;
            while (heading >= 2 * Math.PI) heading -= 2 * Math.PI;
            x += speed * Math.sin(heading) * 0.02;
            y += speed * Math.cos(heading) * 0.02;
        }

        System.out.println();
        System.out.println("Terrain avoidance active: " + terrainAvoidCount + " / " + totalSamples);

        System.out.println();
        System.out.println("BUG CONFIRMATION: terrain avoidance fires " + terrainAvoidCount
                + " out of " + totalSamples + " samples at z=-100m over -200m floor.");
        System.out.println("The sub has 100m of water below it, yet terrain avoidance");
        System.out.println("overrides waypoint steering.");

        // Assert the bug exists
        assertTrue(terrainAvoidCount > 0,
                "Expected terrain avoidance to fire (confirming the bug)");
    }

    // ── Diagnostic 3: Trace all rudder overrides ─────────────────────

    @Test
    void diagnostic_traceRudderOverrides() {
        System.out.println();
        System.out.println("=== DIAGNOSTIC 3: Rudder override analysis ===");
        System.out.println();

        // Test at different depth/floor combinations
        double[][] scenarios = {
                // {floor, subDepth, description_code}
                // description_code: 1=deep_safe, 2=moderate_bug, 3=shallow_extreme, 4=at_target
                {-500, -100, 1},
                {-500, -300, 2},
                {-500, -420, 3}, // exactly at target depth
                {-200, -100, 4},
                {-200, -120, 5}, // exactly at target depth
                {-150, -70, 6},  // shallow water
                {-300, -130, 7}, // just above avoidance threshold
                {-300, -140, 8}, // just below avoidance threshold
        };

        String[] descriptions = {
                "Deep floor(-500), sub at -100: margin=320, safe",
                "Deep floor(-500), sub at -300: margin=120, safe",
                "Deep floor(-500), sub at -420: margin=0, AT TARGET",
                "Mod floor(-200), sub at -100: margin=20, BUG ZONE",
                "Mod floor(-200), sub at -120: margin=0, AT TARGET",
                "Shallow floor(-150), sub at -70: margin=0, BUG ZONE",
                "floor(-300), sub at -130: margin=90, JUST SAFE",
                "floor(-300), sub at -140: margin=80, THRESHOLD",
        };

        System.out.printf("%-55s %-8s %-10s %-8s %-30s%n",
                "Scenario", "Margin", "Urgency", "Rudder", "Status");

        for (int s = 0; s < scenarios.length; s++) {
            double floor = scenarios[s][0];
            double subZ = scenarios[s][1];

            var terrain = flatTerrain(floor);
            var ctrl = new DefaultAttackSub();
            ctrl.onMatchStart(new MatchContext(CONFIG, terrain, List.of(), new CurrentField(List.of())));

            // Tick a few times to get waypoints planned, then check the last tick
            DiagOutput lastOut = null;
            for (int tick = 0; tick < 5; tick++) {
                var pose = new Pose(new Vec3(0, 0, subZ), Math.PI / 4, 0, 0); // heading NE
                var velocity = new Velocity(new Vec3(4, 4, 0), Vec3.ZERO);
                var state = new SubmarineState(pose, velocity, 1000, 0);
                var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
                var input = new TestInputFull(tick, 0.02, state, env, List.of(), List.of(), 0);
                lastOut = new DiagOutput();
                ctrl.onTick(input, lastOut);
            }

            double margin = subZ - (floor + 80); // depth - (worstFloor + floorClearance)
            double urgency = margin < 80
                    ? Math.max(0.2, Math.min(1.0, 1.0 - margin / 80.0))
                    : 0;

            System.out.printf("%-55s %-8.0f %-10.2f %-8.3f %-30s%n",
                    descriptions[s], margin, urgency, lastOut.rudder, lastOut.status);
        }

        System.out.println();
        System.out.println("ANALYSIS:");
        System.out.println("=========");
        System.out.println("The terrain avoidance condition is: margin < floorClearance (80)");
        System.out.println("where margin = depth - (worstFloor + floorClearance)");
        System.out.println();
        System.out.println("This means terrain avoidance triggers when:");
        System.out.println("  depth < worstFloor + 2 * floorClearance");
        System.out.println("  depth < worstFloor + 160");
        System.out.println();
        System.out.println("For a -200m floor: sub must be above -40m to avoid triggering!");
        System.out.println("For a -300m floor: sub must be above -140m to avoid triggering!");
        System.out.println("For a -500m floor: sub must be above -340m to avoid triggering!");
        System.out.println();
        System.out.println("But the sub's target depth is (floor + 80), which is ALWAYS");
        System.out.println("within the terrain avoidance zone (margin = 0 at target depth).");
        System.out.println("The sub can NEVER reach its target depth without terrain avoidance");
        System.out.println("overriding the waypoint rudder.");
        System.out.println();
        System.out.println("ROOT CAUSE: The condition 'margin < floorClearance' uses the SAME");
        System.out.println("constant (80) for both the target depth offset AND the trigger");
        System.out.println("threshold. This creates a zone of [floor+80, floor+160] where");
        System.out.println("terrain avoidance is always active. The sub naturally settles at");
        System.out.println("floor+80 (margin=0), where urgency = 1.0 (MAXIMUM override).");
        System.out.println();
        System.out.println("When terrain avoidance fires with urgency u:");
        System.out.println("  rudder = tacticalRudder * (1-u) + avoidRudder * u");
        System.out.println("At target depth (margin=0), urgency=1.0:");
        System.out.println("  rudder = tacticalRudder * 0 + avoidRudder * 1.0");
        System.out.println("  => waypoint steering is COMPLETELY IGNORED");
    }

    // ── Diagnostic 4: Verify the math at target depth ────────────────

    @Test
    void diagnostic_atTargetDepth_waypointRudderIsZero() {
        // Floor at -200m. Target depth = -200 + 80 = -120m.
        // Sub at -120m (exactly at target depth).
        // margin = -120 - (-120) = 0
        // urgency = clamp(1 - 0/80, 0.2, 1.0) = 1.0
        // rudder = tacticalRudder * (1-1.0) + avoidRudder * 1.0
        // => tacticalRudder contribution = 0%
        var terrain = flatTerrain(-200);
        startMatch(terrain);

        System.out.println();
        System.out.println("=== DIAGNOSTIC 4: At target depth, waypoint rudder is zero ===");

        // First, run several ticks to plan waypoints and get a meaningful tacticalRudder
        double heading = Math.PI / 4; // NE
        DiagOutput out = null;
        for (int tick = 0; tick < 20; tick++) {
            Vec3 vel = new Vec3(6 * Math.sin(heading), 6 * Math.cos(heading), 0);
            out = tickAt(terrain, tick, 0, 0, -120, heading, vel);
        }

        // Find active waypoint and bearing
        Waypoint activeWp = null;
        for (var wp : out.waypoints) {
            if (wp.active()) { activeWp = wp; break; }
        }

        System.out.println("Floor: -200m");
        System.out.println("Target depth: -120m (floor + 80)");
        System.out.println("Sub depth: -120m");
        System.out.println("Margin: 0");
        System.out.println("Urgency: 1.0 (maximum)");
        System.out.println();

        if (activeWp != null) {
            double bearingToWp = Math.atan2(activeWp.x(), activeWp.y());
            if (bearingToWp < 0) bearingToWp += 2 * Math.PI;
            double bearingErr = bearingToWp - heading;
            while (bearingErr > Math.PI) bearingErr -= 2 * Math.PI;
            while (bearingErr < -Math.PI) bearingErr += 2 * Math.PI;

            System.out.println("Active waypoint: (" + activeWp.x() + ", " + activeWp.y() + ")");
            System.out.println("Bearing to waypoint: " + Math.toDegrees(bearingToWp) + " deg");
            System.out.println("Bearing error: " + Math.toDegrees(bearingErr) + " deg");
            System.out.println("Final rudder: " + out.rudder);
            System.out.println("Status: " + out.status);
            System.out.println();
            System.out.println("At urgency=1.0, the formula is:");
            System.out.println("  rudder = tacticalRudder * 0 + avoidRudder * 1.0");
            System.out.println("The waypoint contribution is ZERO regardless of bearing error.");
        } else {
            System.out.println("No active waypoint found (waypoints: " + out.waypoints.size() + ")");
        }

        System.out.println("Rudder output: " + out.rudder);
        System.out.println("Status: " + out.status);

        // The status should show terrain avoidance is active (P/A = PATROL + AVOIDING TERRAIN)
        assertTrue(out.status.contains("/A"),
                "At target depth over -200m floor, terrain avoidance should be active. Status: " + out.status);
    }

    // ── Diagnostic 5: Compute avoidance zone for various floors ──────

    @Test
    void diagnostic_avoidanceZoneCalculation() {
        System.out.println();
        System.out.println("=== DIAGNOSTIC 5: Terrain avoidance zone for various floor depths ===");
        System.out.println();
        System.out.println("The terrain avoidance condition is: margin < 80");
        System.out.println("where margin = subDepth - (floor + 80)");
        System.out.println("So avoidance fires when: subDepth < floor + 160");
        System.out.println("And target depth = floor + 80");
        System.out.println();
        System.out.printf("%-12s %-12s %-12s %-12s %-12s %-15s%n",
                "Floor", "TargetDepth", "AvoidAbove", "MIN_DEPTH", "SafeBand", "Problem?");

        double[] floors = {-100, -150, -200, -300, -500, -700, -1000};
        for (double floor : floors) {
            double targetDepth = floor + 80;
            double avoidanceThreshold = floor + 160; // above this, no avoidance
            double safeBottom = Math.max(avoidanceThreshold, -20); // can't go above MIN_DEPTH=-20
            double safeBand = -20 - avoidanceThreshold; // meters of safe depth

            // Clamp target if necessary
            double clampedTarget = targetDepth;
            double depthLimit = -700 + 50; // crushDepth + CRUSH_SAFETY_MARGIN = -650
            if (clampedTarget < depthLimit) clampedTarget = depthLimit;
            if (clampedTarget > -20) clampedTarget = -20;

            String problem;
            if (safeBand <= 0) {
                problem = "NO SAFE DEPTH!";
            } else if (safeBand < 50) {
                problem = "Very narrow";
            } else {
                problem = "OK (" + (int)safeBand + "m)";
            }

            System.out.printf("%-12.0f %-12.0f %-12.0f %-12.0f %-12.0f %-15s%n",
                    floor, clampedTarget, avoidanceThreshold, -20.0, safeBand, problem);
        }

        System.out.println();
        System.out.println("CONCLUSION:");
        System.out.println("For floors shallower than -180m, there is NO depth where the sub");
        System.out.println("can operate without terrain avoidance overriding waypoint steering.");
        System.out.println("For typical match terrain (-200 to -500m), the 'safe' band where");
        System.out.println("terrain avoidance is inactive is far above the target depth,");
        System.out.println("meaning the depth controller tries to bring the sub DOWN into the");
        System.out.println("terrain avoidance zone, where rudder override becomes permanent.");
    }
}
