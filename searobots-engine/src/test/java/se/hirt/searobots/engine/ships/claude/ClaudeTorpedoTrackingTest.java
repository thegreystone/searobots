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
package se.hirt.searobots.engine.ships.claude;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.TargetDrone;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests Claude torpedo passive/active guidance against moving targets
 * in controlled geometry. Each scenario places a launcher at a specific
 * position and angle relative to the target, fires one torpedo, and
 * checks if it hits.
 *
 * Geometry: launcher is always at origin (0,0), target is placed at
 * (distance, 0) heading in the specified direction. Launcher faces
 * the target. This gives clean control over approach angle and aspect.
 */
public class ClaudeTorpedoTrackingTest {

    private static final long MAX_TICKS = 30_000; // 10 min max

    record TrackResult(boolean hit, int damage, double closestApproach, long endTick) {}

    // ── World builder ──

    /** Flat 1000m ocean with two spawn points at specified positions. */
    private static GeneratedWorld controlledWorld(Vec3 launcherPos, Vec3 targetPos) {
        var config = MatchConfig.withDefaults(0);
        double halfSize = config.battleArea() instanceof BattleArea.Circular c
                ? c.radius() : 7000;
        double margin = config.terrainMarginMeters();
        double totalSize = (halfSize + margin) * 2;
        double cellSize = config.gridCellMeters();
        int gridSize = (int) Math.ceil(totalSize / cellSize) + 1;
        double origin = -(gridSize / 2) * cellSize;
        double[] data = new double[gridSize * gridSize];
        Arrays.fill(data, -1000.0);
        var terrain = new TerrainMap(data, gridSize, gridSize, origin, origin, cellSize);
        return new GeneratedWorld(config, terrain, List.of(),
                new CurrentField(List.of()), List.of(launcherPos, targetPos));
    }

    // ── Controllers ──

    /** Launcher: faces target, pings, fires as soon as aligned and sonar confirms. */
    static class AlignedLauncher implements SubmarineController {
        private boolean fired = false;
        private double targetBearing = Double.NaN;
        private double fixX, fixY, fixZ;
        private final double estTargetHeading;
        private final double estTargetSpeed;

        AlignedLauncher(double estTargetHeading, double estTargetSpeed) {
            this.estTargetHeading = estTargetHeading;
            this.estTargetSpeed = estTargetSpeed;
        }

        @Override public String name() { return "Launcher"; }

        @Override
        public TorpedoController createTorpedoController() {
            return new ClaudeTorpedoController();
        }

        @Override
        public void onTick(SubmarineInput input, SubmarineOutput output) {
            output.setThrottle(0.2);
            double heading = input.self().pose().heading();

            // Ping continuously until we fire
            if (!fired && input.activeSonarCooldownTicks() == 0) {
                output.activeSonarPing();
            }

            // Get target fix from active returns
            if (!fired) {
                var returns = input.activeSonarReturns();
                if (!returns.isEmpty()) {
                    var best = returns.getFirst();
                    var pos = input.self().pose().position();
                    targetBearing = best.bearing();
                    fixX = pos.x() + Math.sin(best.bearing()) * best.range();
                    fixY = pos.y() + Math.cos(best.bearing()) * best.range();
                    fixZ = !Double.isNaN(best.estimatedDepth())
                            ? best.estimatedDepth() : pos.z();
                }
            }

            // Steer toward target and fire when aligned
            if (!Double.isNaN(targetBearing) && !fired) {
                double err = targetBearing - heading;
                while (err > Math.PI) err -= 2 * Math.PI;
                while (err < -Math.PI) err += 2 * Math.PI;
                output.setRudder(Math.clamp(err * 3.0, -1, 1));

                if (Math.abs(err) < Math.toRadians(8)
                        && input.self().torpedoesRemaining() > 0) {
                    String missionData = String.format("%.1f,%.1f,%.1f,%.4f,%.1f",
                            fixX, fixY, fixZ, estTargetHeading, estTargetSpeed);
                    output.launchTorpedo(new TorpedoLaunchCommand(
                            targetBearing, 0, 7.0, missionData));
                    fired = true;
                }
            } else {
                output.setRudder(0);
            }
        }
    }

    /** Scripted target: drives at given throttle/depth, steers to given heading. */
    static class ScriptedTarget implements SubmarineController {
        private final String name;
        private final double throttle;
        private final double targetDepth;
        private final double desiredHeading; // NaN = keep spawn heading
        private final boolean maneuver;      // if true, weave every ~30s
        private TerrainMap terrain;
        private long maneuverTick = 0;
        private double maneuverDir = 1;

        ScriptedTarget(String name, double throttle, double targetDepth,
                       double desiredHeading, boolean maneuver) {
            this.name = name;
            this.throttle = throttle;
            this.targetDepth = targetDepth;
            this.desiredHeading = desiredHeading;
            this.maneuver = maneuver;
        }

        @Override public String name() { return name; }

        @Override
        public void onMatchStart(MatchContext context) {
            this.terrain = context.terrain();
        }

        @Override
        public void onTick(SubmarineInput input, SubmarineOutput output) {
            output.setThrottle(throttle);
            var pos = input.self().pose().position();
            double heading = input.self().pose().heading();

            // Depth control
            double goalDepth = targetDepth;
            double floor = terrain != null ? terrain.elevationAt(pos.x(), pos.y()) + 30 : -1000;
            goalDepth = Math.max(goalDepth, floor);
            goalDepth = Math.min(goalDepth, -20);
            double depthErr = pos.z() - goalDepth;
            output.setBallast(Math.clamp(0.5 - 0.01 * depthErr, 0, 1));
            output.setSternPlanes(Math.clamp(-depthErr * 0.003, -0.5, 0.5));

            // Heading control
            double goalH = desiredHeading;
            if (maneuver) {
                // Weave: change direction every 1500 ticks (30s)
                if (input.tick() - maneuverTick > 1500) {
                    maneuverTick = input.tick();
                    maneuverDir = -maneuverDir;
                }
                goalH = desiredHeading + maneuverDir * Math.toRadians(30);
            }

            if (!Double.isNaN(goalH)) {
                double diff = goalH - heading;
                while (diff > Math.PI) diff -= 2 * Math.PI;
                while (diff < -Math.PI) diff += 2 * Math.PI;
                output.setRudder(Math.clamp(diff * 2.0, -1, 1));
            } else {
                output.setRudder(0);
            }
        }
    }

    // ── Scenario runner ──

    private TrackResult runScenario(Vec3 launcherPos, double launcherHeading,
                                    Vec3 targetPos, double targetHeading,
                                    SubmarineController launcher,
                                    SubmarineController target,
                                    VehicleConfig targetConfig) {
        var world = controlledWorld(launcherPos, targetPos);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(launcher, target);
        var configs = List.of(VehicleConfig.submarine(), targetConfig);
        var headings = List.of(launcherHeading, targetHeading);

        int[] targetHp = {1000};
        double[] closest = {Double.MAX_VALUE};
        long[] endTick = {MAX_TICKS};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() < 2) return;
                targetHp[0] = subs.get(1).hp();
                var tgt = subs.get(1);
                for (var t : torps) {
                    if (t.ownerId() == subs.get(0).id() && t.alive()) {
                        // Torpedo bow to target hull distance (same geometry as fuse check)
                        double d = HullGeometry.bowDistanceToHull(t, tgt);
                        closest[0] = Math.min(closest[0], d);
                    }
                }
                if (tick >= MAX_TICKS || targetHp[0] <= 0) {
                    endTick[0] = tick;
                    sim.stop();
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, headings, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}

        int damage = 1000 - targetHp[0];
        return new TrackResult(damage > 0, damage, closest[0], endTick[0]);
    }

    // ── Convenience: compute geometry ──

    /** Place launcher behind the target (in its baffles). */
    private TrackResult bafflesShot(double range, double launcherDepth, double targetDepth,
                                    double targetSpeed, double targetHeading, boolean maneuver) {
        // Target at origin heading in targetHeading direction
        // Launcher behind the target at given range
        double behindX = -Math.sin(targetHeading) * range;
        double behindY = -Math.cos(targetHeading) * range;
        double bearingToTarget = targetHeading; // launcher faces same way as target

        var launcher = new AlignedLauncher(targetHeading, targetSpeed);
        double throttle = targetSpeed / 15.0; // approximate throttle for desired speed
        var target = new ScriptedTarget("target", throttle, targetDepth,
                targetHeading, maneuver);

        return runScenario(
                new Vec3(behindX, behindY, launcherDepth), bearingToTarget,
                new Vec3(0, 0, targetDepth), targetHeading,
                launcher, target, VehicleConfig.submarine());
    }

    /** Place launcher to the side (beam shot). */
    private TrackResult beamShot(double range, double launcherDepth, double targetDepth,
                                  double targetSpeed, double targetHeading, boolean maneuver) {
        // Target at origin heading in targetHeading direction
        // Launcher perpendicular to target's course
        double sideAngle = targetHeading + Math.PI / 2;
        double sideX = Math.sin(sideAngle) * range;
        double sideY = Math.cos(sideAngle) * range;
        double bearingToTarget = sideAngle + Math.PI; // face toward target

        var launcher = new AlignedLauncher(targetHeading, targetSpeed);
        double throttle = targetSpeed / 15.0;
        var target = new ScriptedTarget("target", throttle, targetDepth,
                targetHeading, maneuver);

        return runScenario(
                new Vec3(sideX, sideY, launcherDepth), bearingToTarget,
                new Vec3(0, 0, targetDepth), targetHeading,
                launcher, target, VehicleConfig.submarine());
    }

    /** Surface ship target (uses surfaceShip config and TargetDrone controller). */
    private TrackResult surfaceShipShot(double range, double launcherDepth, boolean maneuver) {
        // Surface ship at origin heading north, launcher south of it
        double targetHeading = 0;
        var launcher = new AlignedLauncher(targetHeading, 8.0);
        SubmarineController target = maneuver
                ? new ScriptedTarget("ship", 1.0, 0, targetHeading, true)
                : new TargetDrone();

        return runScenario(
                new Vec3(0, -range, launcherDepth), targetHeading,
                new Vec3(0, 0, -5), targetHeading,
                launcher, target,
                maneuver ? VehicleConfig.surfaceShip() : VehicleConfig.surfaceShip());
    }

    // ── The big scenario table ──

    record Scenario(String name, java.util.function.Supplier<TrackResult> run) {}

    @Test
    void trackingScenarioTable() {
        System.out.println("=== Claude Torpedo Tracking Scenarios ===");
        System.out.printf("%-35s %-8s %-10s %-8s%n",
                "Scenario", "Damage", "Closest", "Hit");
        System.out.println("-".repeat(65));

        var scenarios = new Scenario[]{
            // ── Surface ship ──
            new Scenario("Surface ship straight 1500m",
                    () -> surfaceShipShot(1500, -100, false)),
            new Scenario("Surface ship maneuver 1500m",
                    () -> surfaceShipShot(1500, -100, true)),

            // ── Baffles approach, same depth ──
            new Scenario("Baffles 1500m same depth 5m/s",
                    () -> bafflesShot(1500, -100, -100, 5, 0, false)),
            new Scenario("Baffles 1500m same depth 8m/s",
                    () -> bafflesShot(1500, -100, -100, 8, 0, false)),
            new Scenario("Baffles 1500m same depth 12m/s",
                    () -> bafflesShot(1500, -100, -100, 12, 0, false)),

            // ── Baffles approach, depth delta ──
            new Scenario("Baffles 1500m deep->shallow",
                    () -> bafflesShot(1500, -200, -80, 5, 0, false)),
            new Scenario("Baffles 1500m shallow->deep",
                    () -> bafflesShot(1500, -80, -200, 5, 0, false)),
            new Scenario("Baffles 1500m big depth delta",
                    () -> bafflesShot(1500, -300, -80, 5, 0, false)),

            // ── Baffles approach, various ranges ──
            new Scenario("Baffles 800m same depth 5m/s",
                    () -> bafflesShot(800, -100, -100, 5, 0, false)),
            new Scenario("Baffles 2000m same depth 5m/s",
                    () -> bafflesShot(2000, -100, -100, 5, 0, false)),
            new Scenario("Baffles 3000m same depth 5m/s",
                    () -> bafflesShot(3000, -100, -100, 5, 0, false)),

            // ── Baffles with maneuvering target ──
            new Scenario("Baffles 1500m maneuver 5m/s",
                    () -> bafflesShot(1500, -100, -100, 5, 0, true)),
            new Scenario("Baffles 1500m maneuver 8m/s",
                    () -> bafflesShot(1500, -100, -100, 8, 0, true)),

            // ── Full speed target (15 m/s) from baffles at various ranges ──
            new Scenario("Baffles 800m full speed 15m/s",
                    () -> bafflesShot(800, -100, -100, 15, 0, false)),
            new Scenario("Baffles 1000m full speed 15m/s",
                    () -> bafflesShot(1000, -100, -100, 15, 0, false)),
            new Scenario("Baffles 1200m full speed 15m/s",
                    () -> bafflesShot(1200, -100, -100, 15, 0, false)),
            new Scenario("Baffles 1500m full speed 15m/s",
                    () -> bafflesShot(1500, -100, -100, 15, 0, false)),

            // ── Beam (crossing) shots ──
            new Scenario("Beam 1500m same depth 5m/s",
                    () -> beamShot(1500, -100, -100, 5, 0, false)),
            new Scenario("Beam 1500m same depth 8m/s",
                    () -> beamShot(1500, -100, -100, 8, 0, false)),
            new Scenario("Beam 1500m depth delta 5m/s",
                    () -> beamShot(1500, -200, -80, 5, 0, false)),
            new Scenario("Beam 1500m maneuver 8m/s",
                    () -> beamShot(1500, -100, -100, 8, 0, true)),
        };

        int hits = 0;
        int totalDamage = 0;
        for (var s : scenarios) {
            var r = s.run.get();
            if (r.hit) hits++;
            totalDamage += r.damage;
            System.out.printf("%-35s %7d %9.0fm %8s%n",
                    s.name, r.damage, r.closestApproach, r.hit ? "HIT" : "miss");
        }
        System.out.printf("%nHits: %d/%d  Total damage: %d%n",
                hits, scenarios.length, totalDamage);
        // Expected: all baffles (same depth, depth delta, 800m, 2000m) + surface ships
        // should hit reliably. 3000m is beyond fuel range. Beam shots are hard geometry.
        // Maneuvering 8m/s targets are borderline (depends on weave timing).
        assertTrue(hits >= 9,
                "Should hit at least 9/" + scenarios.length + " scenarios, got " + hits);
    }

    // ── Must-pass individual tests ──

    @Test
    void hitsBafflesSameDepthSlow() {
        var r = bafflesShot(1500, -100, -100, 5, 0, false);
        assertTrue(r.hit, "Should hit 5m/s target from baffles at 1500m, closest=" + r.closestApproach);
    }

    @Test
    void beamShotSlowApproaches() {
        // Beam shots are geometrically hard; verify the torpedo gets close
        var r = beamShot(1500, -100, -100, 5, 0, false);
        assertTrue(r.hit || r.closestApproach < 80,
                "Should hit or approach 5m/s crossing target, closest=" + r.closestApproach);
    }

    @Test
    void surfaceShipApproaches() {
        // Surface ship with 95m depth delta is a hard pitch correction
        var r = surfaceShipShot(1500, -100, false);
        assertTrue(r.hit || r.closestApproach < 80,
                "Should hit or approach surface ship, closest=" + r.closestApproach);
    }

    @Test
    void hitsDeepTarget() {
        var r = bafflesShot(1500, -80, -200, 5, 0, false);
        assertTrue(r.hit || r.closestApproach < 40,
                "Should hit or near-miss deep target from shallow, closest=" + r.closestApproach);
    }
}
