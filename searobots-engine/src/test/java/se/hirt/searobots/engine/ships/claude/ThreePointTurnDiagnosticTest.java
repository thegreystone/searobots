package se.hirt.searobots.engine.ships.claude;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

import static se.hirt.searobots.api.VehicleConfig.submarine;

/**
 * Diagnostic test for the three-point turn behavior near the L-shaped island.
 * Prints detailed per-tick state so we can trace:
 * 1. What safe heading is chosen
 * 2. Where the exit waypoint is placed
 * 3. What happens when the reverse phase ends
 * 4. What causes the replan into the island
 */
class ThreePointTurnDiagnosticTest {

    /**
     * Unit-level diagnostic: directly call the autopilot's planThreePointTurn
     * logic to see what safe heading and exit waypoint it computes.
     * This avoids running the full sim and isolates the geometry.
     */
    @Test
    void diagnoseThreePointTurnGeometry() {
        var world = GeneratedWorld.lIslandRecovery();
        var terrain = world.terrain();
        var config = world.config();
        var context = new MatchContext(config, terrain, world.thermalLayers(), world.currentField());
        var autopilot = new SubmarineAutopilot(context);

        double posX = 0, posY = -800, posZ = -150;
        double heading = 0; // north
        double speed = 3.0;

        System.out.println("=== Three-Point Turn Geometry Diagnostic ===");
        System.out.printf("Sub position: (%.0f, %.0f, %.0f)%n", posX, posY, posZ);
        System.out.printf("Sub heading: %.1f deg (%.4f rad)%n", Math.toDegrees(heading), heading);
        System.out.printf("Sub speed: %.1f m/s%n", speed);

        // Check terrain along heading
        System.out.println("\nTerrain along heading (north, 0 deg):");
        for (double d = 50; d <= 500; d += 50) {
            double fx = posX + Math.sin(heading) * d;
            double fy = posY + Math.cos(heading) * d;
            double elev = terrain.elevationAt(fx, fy);
            System.out.printf("  d=%.0f: (%.0f, %.0f) elev=%.1f %s%n",
                    d, fx, fy, elev, elev > -90 ? "SHALLOW/ISLAND" : "deep");
        }

        // Replicate the safe heading search from planThreePointTurn
        double SHALLOW_WATER_LIMIT = -90.0;
        System.out.println("\nSafe heading search (turn > 60 deg, safe for 500m):");
        double bestSafeHeading = Double.NaN;
        double bestScore = Double.MAX_VALUE;
        for (int deg = 0; deg < 360; deg += 10) {
            double brg = Math.toRadians(deg);
            double turnNeeded = Math.abs(SubmarineAutopilot.angleDiff(brg, heading));
            boolean passesMinTurn = turnNeeded >= Math.toRadians(60);

            boolean safe = true;
            double blockDist = -1;
            for (double d = 50; d <= 500; d += 50) {
                double fx = posX + Math.sin(brg) * d;
                double fy = posY + Math.cos(brg) * d;
                if (terrain.elevationAt(fx, fy) > SHALLOW_WATER_LIMIT) {
                    safe = false;
                    blockDist = d;
                    break;
                }
            }

            if (passesMinTurn && safe) {
                System.out.printf("  deg=%3d turnNeeded=%.0f SAFE -> %s%n",
                        deg, Math.toDegrees(turnNeeded),
                        (Double.isNaN(bestSafeHeading) || turnNeeded < bestScore)
                                ? "*BEST*" : "candidate");
                if (Double.isNaN(bestSafeHeading) || turnNeeded < bestScore) {
                    bestSafeHeading = brg;
                    bestScore = turnNeeded;
                }
            } else if (passesMinTurn && !safe) {
                System.out.printf("  deg=%3d turnNeeded=%.0f BLOCKED at %.0fm%n",
                        deg, Math.toDegrees(turnNeeded), blockDist);
            }
            // Skip printing headings that don't pass min turn
        }

        System.out.printf("\nChosen safe heading: %.1f deg (%.4f rad)%n",
                Math.toDegrees(bestSafeHeading), bestSafeHeading);

        // Compute the exit waypoint the same way the code does
        double oppSafe = SubmarineAutopilot.normalizeBearing(bestSafeHeading + Math.PI);
        double revBack = 200;
        double revSide = 200;
        double revX = posX - Math.sin(heading) * revBack + Math.sin(oppSafe) * revSide;
        double revY = posY - Math.cos(heading) * revBack + Math.cos(oppSafe) * revSide;

        // Shorten check
        while (terrain.elevationAt(revX, revY) > SHALLOW_WATER_LIMIT && revBack > 80) {
            revBack -= 25;
            revX = posX - Math.sin(heading) * revBack + Math.sin(oppSafe) * revSide;
            revY = posY - Math.cos(heading) * revBack + Math.cos(oppSafe) * revSide;
        }

        double exitX = posX + Math.sin(bestSafeHeading) * 800;
        double exitY = posY + Math.cos(bestSafeHeading) * 800;

        System.out.printf("\nReverse WP: (%.0f, %.0f) elev=%.1f%n",
                revX, revY, terrain.elevationAt(revX, revY));
        System.out.printf("Exit WP:    (%.0f, %.0f) elev=%.1f %s%n",
                exitX, exitY, terrain.elevationAt(exitX, exitY),
                terrain.elevationAt(exitX, exitY) > -90 ? "*** IN ISLAND ***" : "safe");

        // Check terrain along exit heading
        System.out.printf("\nTerrain along exit heading (%.0f deg):%n", Math.toDegrees(bestSafeHeading));
        for (double d = 50; d <= 1000; d += 50) {
            double fx = posX + Math.sin(bestSafeHeading) * d;
            double fy = posY + Math.cos(bestSafeHeading) * d;
            double elev = terrain.elevationAt(fx, fy);
            System.out.printf("  d=%.0f: (%.0f, %.0f) elev=%.1f %s%n",
                    d, fx, fy, elev, elev > -90 ? "SHALLOW/ISLAND" : "deep");
        }
    }

    /**
     * Full simulation trace: runs the L-island scenario for 8000 ticks (160 seconds)
     * and prints diagnostic output. Detects:
     * - When three-point turn waypoints appear
     * - Whether exit WP is in the island
     * - The reverse/forward phase transitions
     * - Replans after the turn completes
     */
    @Test
    void traceThreePointTurnAtLIsland() {
        var world = GeneratedWorld.lIslandRecovery();
        var terrain = world.terrain();

        // Print terrain at key points for orientation
        System.out.println("=== Terrain Check ===");
        for (double y = -1200; y <= 2200; y += 200) {
            for (double x = -2200; x <= 2200; x += 400) {
                double elev = terrain.elevationAt(x, y);
                String marker = elev > -90 ? "##" : "..";
                System.out.printf("%s", marker);
            }
            System.out.printf("  y=%.0f%n", y);
        }
        System.out.println("Legend: ## = island/shallow (elev > -90), .. = deep water");

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new ClaudeAttackSub(), new ClaudeAttackSub());
        List<VehicleConfig> configs = List.of(submarine(), submarine());
        // Sub 0 heading = 0 (north), sub 1 far away
        var headings = List.of(0.0, 0.0);

        var log = new ArrayList<String>();
        int maxTicks = 8000; // 160 seconds

        // Header
        log.add(String.format("%-6s %-28s %-8s %-8s %-8s %-8s %-30s %-6s %-40s",
                "TICK", "POSITION (x, y, z)", "HDG_DEG", "SURGE", "THR", "RUD",
                "STATUS", "HP", "NAV_WP (x, y) [flags]"));

        var listener = new SimulationListener() {
            // Track previous waypoints to detect replans
            private int lastWpCount = -1;
            private double lastActiveWpX = Double.NaN;
            private double lastActiveWpY = Double.NaN;
            private boolean lastHasReverse = false;

            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (submarines.isEmpty()) return;
                var s = submarines.get(0);
                var pos = s.pose().position();
                double hdg = s.pose().heading();
                double headingDeg = Math.toDegrees(hdg);
                if (headingDeg < 0) headingDeg += 360;

                // Surge speed from body-frame velocity
                double surge = s.velocity().linear().x();

                // Find active waypoint and detect reverse WPs
                int activeIdx = -1;
                double activeX = 0, activeY = 0;
                boolean activeIsReverse = false;
                boolean hasAnyReverse = false;
                int wpCount = s.waypoints() != null ? s.waypoints().size() : 0;
                StringBuilder wpInfo = new StringBuilder();

                if (s.waypoints() != null) {
                    for (int i = 0; i < s.waypoints().size(); i++) {
                        var wp = s.waypoints().get(i);
                        if (wp.reverse()) hasAnyReverse = true;
                        if (wp.active()) {
                            activeIdx = i;
                            activeX = wp.x();
                            activeY = wp.y();
                            activeIsReverse = wp.reverse();
                        }
                    }
                    wpInfo.append(String.format("wp[%d]=(%.0f,%.0f)%s",
                            activeIdx, activeX, activeY,
                            activeIsReverse ? " REV" : ""));
                } else {
                    wpInfo.append("NO_WPS");
                }

                String status = s.status() != null ? s.status() : "";

                // Detect replans: waypoint count changed, or active waypoint jumped
                boolean replanDetected = false;
                if (wpCount != lastWpCount) {
                    replanDetected = true;
                } else if (activeIdx >= 0
                        && (!Double.isNaN(lastActiveWpX))
                        && (Math.abs(activeX - lastActiveWpX) > 50 || Math.abs(activeY - lastActiveWpY) > 50)) {
                    replanDetected = true;
                }
                // Detect three-point turn start/end
                boolean tptTransition = (hasAnyReverse != lastHasReverse);

                // Check if active waypoint is in the island
                boolean wpInIsland = false;
                if (activeIdx >= 0) {
                    double wpElev = terrain.elevationAt(activeX, activeY);
                    wpInIsland = wpElev > -90;
                }

                // Log every 50 ticks, plus on replans, plus on HP loss, plus TPT transitions
                boolean shouldLog = (tick % 50 == 0)
                        || replanDetected
                        || tptTransition
                        || s.hp() < 1000;

                if (shouldLog) {
                    String flags = "";
                    if (replanDetected) flags += " *REPLAN*";
                    if (tptTransition && hasAnyReverse) flags += " *3PT_START*";
                    if (tptTransition && !hasAnyReverse) flags += " *3PT_END*";
                    if (wpInIsland) flags += " *WP_IN_ISLAND*";

                    log.add(String.format("%-6d (%-7.0f, %-7.0f, %-6.0f) %-8.1f %-8.2f %-8.2f %-8.2f %-30s %-6d %-40s%s",
                            tick, pos.x(), pos.y(), pos.z(),
                            headingDeg, surge, s.throttle(), s.rudder(),
                            status.isEmpty() ? "-" : status, s.hp(),
                            wpInfo.toString(), flags));
                }

                lastWpCount = wpCount;
                lastHasReverse = hasAnyReverse;
                if (activeIdx >= 0) {
                    lastActiveWpX = activeX;
                    lastActiveWpY = activeY;
                }

                // Dump full waypoint list on replans
                if (replanDetected && s.waypoints() != null && !s.waypoints().isEmpty()) {
                    StringBuilder allWps = new StringBuilder();
                    allWps.append(String.format("       >> ALL WPS (%d): ", s.waypoints().size()));
                    for (int i = 0; i < s.waypoints().size(); i++) {
                        var wp = s.waypoints().get(i);
                        double wpElev = terrain.elevationAt(wp.x(), wp.y());
                        allWps.append(String.format("[%d](%.0f,%.0f,%.0f elev=%.0f%s%s) ",
                                i, wp.x(), wp.y(), wp.z(), wpElev,
                                wp.active() ? " ACTIVE" : "",
                                wp.reverse() ? " REV" : ""));
                    }
                    log.add(allWps.toString());
                }

                if (s.hp() <= 0 || tick >= maxTicks) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, headings, listener));
        thread.start();
        try {
            thread.join(60_000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        System.out.println("\n=== Three-Point Turn Simulation Trace (sub 0) ===");
        System.out.println("Sub starts at (0, -900, -150) heading 0 deg (north)");
        System.out.println("Island horizontal arm: x[-2000,2000] y[-500,500]");
        System.out.println("Island vertical arm:   x[-500,500]   y[-500,2000]");
        System.out.println();
        for (var line : log) {
            System.out.println(line);
        }
    }
}
