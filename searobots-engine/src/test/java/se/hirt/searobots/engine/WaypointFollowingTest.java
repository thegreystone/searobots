package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Measures how accurately the autopilot follows waypoints at various
 * angles, depths, and speeds. Each test places a single strategic waypoint
 * and tracks the sub's closest approach, time to arrival, and path efficiency.
 */
class WaypointFollowingTest {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    record FollowResult(
            double closestApproach,   // minimum distance to waypoint (m)
            double timeToClosest,     // time of closest approach (s)
            boolean arrived,          // came within arrival radius
            double timeToArrival,     // time to arrive (s), or -1
            double pathLength,        // total distance traveled (m)
            double directDistance,    // straight-line distance to waypoint (m)
            double pathEfficiency,    // directDistance / pathLength (1.0 = perfect)
            double finalHeading,      // heading when closest (degrees)
            double finalDepth,        // depth when closest
            double maxCrossTrack,     // max perpendicular deviation from direct line (m)
            int finalHp
    ) {}

    FollowResult runWaypointFollow(double startX, double startY, double startZ,
                                    double startHeading, double startSpeed,
                                    double wpX, double wpY, double wpZ,
                                    int maxTicks) {
        var config = MatchConfig.withDefaults(0);
        var context = new MatchContext(config, world.terrain(), List.of(), world.currentField());
        var autopilot = new SubmarineAutopilot(context);
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(startX, startY, startZ), startHeading, Color.RED, 1000);

        // Pre-accelerate to startSpeed if > 0
        if (startSpeed > 1) {
            entity.setThrottle(0.5);
            for (int t = 0; t < 50 * 30; t++) {
                physics.step(entity, DT, world.terrain(), world.currentField(),
                        world.config().battleArea());
                if (entity.speed() >= startSpeed * 0.9) break;
            }
        }

        var wp = new StrategicWaypoint(wpX, wpY, wpZ, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 200, -1);
        autopilot.setWaypoints(List.of(wp),
                entity.x(), entity.y(), entity.z(),
                entity.heading(), entity.speed());

        // Measure from actual position when waypoint is set (after pre-acceleration)
        double origX = entity.x(), origY = entity.y();
        double directDist = Math.sqrt(Math.pow(wpX - origX, 2)
                + Math.pow(wpY - origY, 2));

        // Direct line unit vector for cross-track error
        double dlx = (wpX - origX) / Math.max(directDist, 1);
        double dly = (wpY - origY) / Math.max(directDist, 1);

        double closestDist = Double.MAX_VALUE;
        double closestTime = 0;
        double closestHeading = 0;
        double closestDepth = 0;
        boolean arrived = false;
        double arrivalTime = -1;
        double pathLength = 0;
        double lastX = entity.x(), lastY = entity.y();
        double maxCrossTrack = 0;

        for (int t = 0; t < maxTicks; t++) {
            var pose = new Pose(new Vec3(entity.x(), entity.y(), entity.z()),
                    entity.heading(), entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(entity.speed() * Math.sin(entity.heading()),
                             entity.speed() * Math.cos(entity.heading()),
                             entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, entity.hp(), 0);
            var env = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            autopilot.tick(input, output);

            entity.setThrottle(output.throttle);
            entity.setRudder(output.rudder);
            entity.setSternPlanes(output.sternPlanes);
            entity.setBallast(output.ballast);
            physics.step(entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());

            // Track metrics (only count path length before arrival)
            double dx = entity.x() - lastX;
            double dy = entity.y() - lastY;
            if (!arrived) {
                pathLength += Math.sqrt(dx * dx + dy * dy);
            }
            lastX = entity.x();
            lastY = entity.y();

            double dist = Math.sqrt(Math.pow(entity.x() - wpX, 2)
                    + Math.pow(entity.y() - wpY, 2));
            if (dist < closestDist) {
                closestDist = dist;
                closestTime = t * DT;
                closestHeading = Math.toDegrees(entity.heading());
                closestDepth = entity.z();
            }
            if (!arrived && dist < 200) {
                arrived = true;
                arrivalTime = t * DT;
            }

            // Cross-track error: perpendicular distance from direct line
            // Only measure while approaching (before arrival)
            if (!arrived) {
                double relX = entity.x() - origX;
                double relY = entity.y() - origY;
                double crossTrack = Math.abs(relX * (-dly) + relY * dlx);
                if (crossTrack > maxCrossTrack) maxCrossTrack = crossTrack;
            }

            if (entity.hp() <= 0) break;
        }

        double efficiency = pathLength > 0 ? directDist / pathLength : 0;
        return new FollowResult(closestDist, closestTime, arrived, arrivalTime,
                pathLength, directDist, efficiency, closestHeading, closestDepth,
                maxCrossTrack, entity.hp());
    }

    void printResult(String label, FollowResult r) {
        System.out.printf("%-30s dist=%.0fm->%.0fm  %s (%.0fs)  path=%.0fm  eff=%.0f%%  " +
                        "xtrack=%.0fm  depth=%.0f  hp=%d%n",
                label, r.directDistance, r.closestApproach,
                r.arrived ? "ARRIVED" : "closest",
                r.arrived ? r.timeToArrival : r.timeToClosest,
                r.pathLength, r.pathEfficiency * 100,
                r.maxCrossTrack, r.finalDepth, r.finalHp);
    }

    // ── Angle sweep: waypoint at 2km, various bearings ──────────────

    @ParameterizedTest(name = "bearing {0}°")
    @CsvSource({
            "0,   straight ahead",
            "15,  slight right",
            "30,  moderate right",
            "45,  diagonal right",
            "60,  sharp right",
            "90,  due east",
            "120, rear-right",
            "150, nearly behind-right",
            "180, directly behind"
    })
    void angleSweep(int bearingDeg, String description) {
        double bearing = Math.toRadians(bearingDeg);
        double dist = 2000;
        double wpX = dist * Math.sin(bearing);
        double wpY = dist * Math.cos(bearing);

        var r = runWaypointFollow(0, 0, -200, 0, 7, wpX, wpY, -200, 50 * 300);
        printResult(bearingDeg + "° " + description, r);

        assertTrue(r.finalHp > 0, "Should survive");
        if (bearingDeg <= 45) {
            assertTrue(r.arrived, bearingDeg + "° turn should arrive at waypoint");
        } else {
            // Wider turns may not arrive in time but should make progress
            assertTrue(r.closestApproach < r.directDistance * 0.7,
                    "Should make progress, closest=" + r.closestApproach);
        }
    }

    // ── Depth transitions at various angles ─────────────────────────

    @ParameterizedTest(name = "dive to {1}m at {0}°")
    @CsvSource({
            "0,   -400, dive straight ahead",
            "0,   -100, rise straight ahead",
            "45,  -400, dive diagonal",
            "45,  -100, rise diagonal",
            "90,  -400, dive 90 degree turn",
            "90,  -100, rise 90 degree turn",
    })
    void depthTransitionAtAngle(int bearingDeg, double targetZ, String description) {
        double bearing = Math.toRadians(bearingDeg);
        double dist = 2000;
        double wpX = dist * Math.sin(bearing);
        double wpY = dist * Math.cos(bearing);

        var r = runWaypointFollow(0, 0, -200, 0, 7, wpX, wpY, targetZ, 50 * 300);
        printResult(description, r);

        assertTrue(r.finalHp > 0, "Should survive");
        // Depth should move toward target
        if (targetZ < -200) {
            assertTrue(r.finalDepth < -250,
                    "Should dive toward " + targetZ + ", at " + r.finalDepth);
        } else {
            assertTrue(r.finalDepth > -180,
                    "Should rise toward " + targetZ + ", at " + r.finalDepth);
        }
    }

    // ── Speed effect: same waypoint at different starting speeds ────

    @Test
    void speedEffect() {
        double wpX = 1414, wpY = 1414; // 45 degrees, 2km
        System.out.println("=== Speed effect on 45° turn to 2km waypoint ===");
        for (int startSpeed : new int[]{0, 3, 5, 7, 10}) {
            var r = runWaypointFollow(0, 0, -200, 0, startSpeed,
                    wpX, wpY, -200, 50 * 300);
            printResult("speed=" + startSpeed + " m/s", r);
            assertTrue(r.finalHp > 0, "Should survive at speed " + startSpeed);
        }
    }

    // ── Distance effect: waypoints at different distances ───────────

    @Test
    void distanceEffect() {
        System.out.println("=== Distance effect on 45° turn ===");
        for (int dist : new int[]{500, 1000, 2000, 4000}) {
            double wpX = dist * Math.sin(Math.toRadians(45));
            double wpY = dist * Math.cos(Math.toRadians(45));
            var r = runWaypointFollow(0, 0, -200, 0, 7,
                    wpX, wpY, -200, 50 * 300);
            printResult("dist=" + dist + "m", r);
            assertTrue(r.finalHp > 0, "Should survive at distance " + dist);
        }
    }

    // ── Tracking diagnostic: straight ahead ────────────────────────

    @Test
    void straightAheadTrackingDiagnostic() {
        // Sub heading north at 7 m/s, waypoint 2km straight ahead.
        // Track position, heading, rudder, and status every second.
        var config = MatchConfig.withDefaults(0);
        var context = new MatchContext(config, world.terrain(), List.of(), world.currentField());
        var autopilot = new SubmarineAutopilot(context);
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(0, 0, -200), 0, java.awt.Color.RED, 1000);

        // Pre-accelerate
        entity.setThrottle(0.5);
        for (int t = 0; t < 50 * 30; t++) {
            physics.step(entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
            if (entity.speed() >= 6.5) break;
        }

        var wp = new StrategicWaypoint(0, 2000, -200, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 200, -1);
        autopilot.setWaypoints(List.of(wp),
                entity.x(), entity.y(), entity.z(),
                entity.heading(), entity.speed());

        System.out.println("=== Straight-ahead tracking diagnostic ===");
        System.out.printf("Start: (%.0f, %.0f, %.0f) hdg=%.1f° spd=%.1f%n",
                entity.x(), entity.y(), entity.z(),
                Math.toDegrees(entity.heading()), entity.speed());
        System.out.println("Nav waypoints:");
        for (int i = 0; i < autopilot.navWaypoints().size(); i++) {
            var nw = autopilot.navWaypoints().get(i);
            System.out.printf("  [%d] (%.0f, %.0f, %.0f)%s%n",
                    i, nw.x(), nw.y(), nw.z(),
                    i == autopilot.currentNavIndex() ? " <-- ACTIVE" : "");
        }

        System.out.printf("%-5s %8s %8s %8s %6s %7s %7s %7s %s%n",
                "t(s)", "x", "y", "z", "hdg", "spd", "rudder", "xtrack", "status");

        double maxXTrack = 0;
        for (int t = 0; t < 50 * 180; t++) {
            var pose = new Pose(new Vec3(entity.x(), entity.y(), entity.z()),
                    entity.heading(), entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(entity.speed() * Math.sin(entity.heading()),
                             entity.speed() * Math.cos(entity.heading()),
                             entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, entity.hp(), 0);
            var env = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, DT, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            autopilot.tick(input, output);

            double xtrack = Math.abs(entity.x());
            if (xtrack > maxXTrack) maxXTrack = xtrack;

            if (t % 250 == 0) { // every 5 seconds
                String status = autopilot.lastStatus();
                if (status != null) {
                    int fIdx = status.indexOf(" f:");
                    if (fIdx >= 0) status = status.substring(0, fIdx);
                }
                System.out.printf("%-5.0f %8.0f %8.0f %8.0f %5.0f° %6.1f %+7.3f %7.0f %s%n",
                        t / 50.0, entity.x(), entity.y(), entity.z(),
                        Math.toDegrees(entity.heading()), entity.speed(),
                        output.rudder, xtrack,
                        status != null ? status : "");
            }

            entity.setThrottle(output.throttle);
            entity.setRudder(output.rudder);
            entity.setSternPlanes(output.sternPlanes);
            entity.setBallast(output.ballast);
            physics.step(entity, DT, world.terrain(), world.currentField(),
                    world.config().battleArea());
        }

        System.out.printf("Max cross-track error: %.0fm%n", maxXTrack);
        assertTrue(maxXTrack < 100,
                "Straight-ahead cross-track should be small, was " + maxXTrack + "m");
    }

    // ── Path efficiency summary ─────────────────────────────────────

    @Test
    void pathEfficiencySummary() {
        System.out.println("=== Path efficiency summary ===");
        System.out.printf("%-15s %8s %8s %8s %8s %8s%n",
                "Scenario", "Direct", "Path", "Effic%", "XTrack", "Arrived");

        int arrivals = 0;
        int total = 0;
        double[] angles = {0, 30, 45, 60, 90, 120, 150, 180};
        for (double deg : angles) {
            double bearing = Math.toRadians(deg);
            double wpX = 2000 * Math.sin(bearing);
            double wpY = 2000 * Math.cos(bearing);
            var r = runWaypointFollow(0, 0, -200, 0, 7,
                    wpX, wpY, -200, 50 * 300);
            System.out.printf("%-15s %7.0fm %7.0fm %7.0f%% %7.0fm %8s%n",
                    deg + "°",
                    r.directDistance, r.pathLength,
                    r.pathEfficiency * 100,
                    r.maxCrossTrack,
                    r.arrived ? "YES" : "no (" + (int) r.closestApproach + "m)");
            total++;
            if (r.arrived) arrivals++;
        }
        System.out.printf("Arrival rate: %d/%d (%.0f%%)%n", arrivals, total,
                100.0 * arrivals / total);

        // At least straight ahead and small angles should arrive
        assertTrue(arrivals >= 3,
                "Should arrive at waypoints for at least small angles, got " + arrivals + "/" + total);
    }
}
