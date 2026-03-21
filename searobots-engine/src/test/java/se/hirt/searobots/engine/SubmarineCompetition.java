/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * Submarine controller competition framework. Runs multiple controller
 * implementations through the same set of seeded maps and compares
 * navigation performance metrics.
 */
package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import java.util.*;
import java.util.function.Supplier;

/**
 * Runs a competition between submarine controller implementations.
 * Each controller is run through the same set of seeded maps with the
 * same spawn points and conditions. Metrics are collected and compared
 * in a table.
 */
public class SubmarineCompetition {

    static final int TICKS_PER_SECOND = 50;
    static final int DEFAULT_DURATION = TICKS_PER_SECOND * 300; // 5 minutes

    record Competitor(String name, Supplier<SubmarineController> factory) {}

    record Metrics(
            // Navigation
            int longestWaypointChain,    // consecutive waypoints reached without replan
            double waypointHitAccuracy,  // avg distance to waypoint center at arrival (m)
            int waypointsReached,        // total waypoints arrived at
            int waypointsAttempted,      // total strategic waypoints generated
            double timeToFirstWaypoint,  // seconds to reach first waypoint (-1 = never)
            // Stealth
            double avgDepth,             // average operating depth (more negative = better)
            double peakDepthShallowest,  // closest to surface (more negative = better)
            double avgNoiseDb,           // average source level in dB (lower = better)
            double peakNoiseDb,          // max source level in dB
            // Efficiency
            double avgSpeed,             // m/s
            double pathEfficiency,       // straight-line progress / distance traveled
            double normalPatrolPct,      // % of time in normal (not emergency) states
            // Survivability
            double timeOfFirstDamage,    // seconds until first HP loss (-1 = no damage)
            double timeToDeath           // seconds alive (-1 = survived)
    ) {}

    record SeedResult(long seed, String competitorName, Metrics metrics) {}

    /**
     * Runs one competitor on one seed and collects metrics.
     */
    static Metrics runOne(Supplier<SubmarineController> factory, long seed, int durationTicks) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();

        var controller = factory.get();
        var sp = world.spawnPoints().get(0);
        double heading = WorldGenerator.findSafeHeading(terrain, sp.x(), sp.y());
        if (Double.isNaN(heading)) {
            heading = Math.atan2(-sp.x(), -sp.y());
            if (heading < 0) heading += 2 * Math.PI;
        }

        var physics = new SubmarinePhysics();
        var entity = new SubmarineEntity(
                VehicleConfig.submarine(), 0, controller,
                sp, heading, java.awt.Color.BLUE, 1000);

        var context = new MatchContext(config, terrain, world.thermalLayers(),
                world.currentField());
        controller.onMatchStart(context);

        // Tracking state
        double totalDepth = 0;
        double totalNoiseDb = 0;
        double totalSpeed = 0;
        double peakShallowest = Double.NEGATIVE_INFINITY;
        double peakNoiseDb = Double.NEGATIVE_INFINITY;
        int aliveTicks = 0;
        double timeOfFirstDamage = -1;
        double timeToDeath = -1;
        int normalTicks = 0;

        // Waypoint tracking
        int waypointsReached = 0;
        int waypointsAttempted = 0;
        double timeToFirstWaypoint = -1;
        double totalHitDistance = 0;
        int currentChain = 0;
        int longestChain = 0;

        // Track strategic waypoints for arrival detection
        List<double[]> lastStrategicWps = List.of();
        int lastStrategicCount = 0;
        double[] currentTargetWp = null;
        boolean trackingWaypoint = false;

        double startX = sp.x(), startY = sp.y();
        double totalDistTraveled = 0;
        double straightLineProgress = 0;
        double lastX = sp.x(), lastY = sp.y();

        for (int t = 0; t < durationTicks; t++) {
            var pose = new Pose(new Vec3(entity.x(), entity.y(), entity.z()),
                    entity.heading(), entity.pitch(), 0);
            var vel = new Velocity(
                    new Vec3(entity.speed() * Math.sin(entity.heading()),
                             entity.speed() * Math.cos(entity.heading()),
                             entity.verticalSpeed()),
                    Vec3.ZERO);
            var state = new SubmarineState(pose, vel, entity.hp(), 0);
            var env = new EnvironmentSnapshot(terrain, List.of(), world.currentField());
            var input = new NavigationSimTest.TestInputFull(
                    t, 1.0 / TICKS_PER_SECOND, state, env, List.of(), List.of(), 0);
            var output = new NavigationSimTest.CapturedOutput();
            controller.onTick(input, output);

            entity.setThrottle(output.throttle);
            entity.setRudder(output.rudder);
            entity.setSternPlanes(output.sternPlanes);
            entity.setBallast(output.ballast);
            physics.step(entity, 1.0 / TICKS_PER_SECOND, terrain, world.currentField(),
                    config.battleArea());

            if (entity.hp() <= 0) {
                if (timeToDeath < 0) timeToDeath = t / (double) TICKS_PER_SECOND;
                break;
            }
            aliveTicks = t + 1;

            // Distance
            double dx = entity.x() - lastX;
            double dy = entity.y() - lastY;
            totalDistTraveled += Math.sqrt(dx * dx + dy * dy);
            lastX = entity.x();
            lastY = entity.y();

            // Depth
            double depth = entity.z();
            totalDepth += depth;
            if (depth > peakShallowest) peakShallowest = depth;

            // Noise (dB directly from physics)
            double noiseDb = entity.sourceLevelDb();
            totalNoiseDb += noiseDb;
            if (noiseDb > peakNoiseDb) peakNoiseDb = noiseDb;

            // Speed
            totalSpeed += entity.speed();

            // Damage tracking
            if (entity.hp() < 1000 && timeOfFirstDamage < 0) {
                timeOfFirstDamage = t / (double) TICKS_PER_SECOND;
            }

            // Status tracking
            String status = entity.status();
            if (status == null || status.isEmpty()) {
                normalTicks++;
            } else {
                // Strip floor/gap suffix
                int fIdx = status.indexOf(" f:");
                String key = fIdx >= 0 ? status.substring(0, fIdx) : status;
                // Count as "normal" if it's a regular patrol state (P, P/A with high gap, etc.)
                // Count as abnormal if it's emergency, avoiding, pull-up, etc.
                if (!key.contains("EMERGENCY") && !key.contains("AVOIDING")
                        && !key.contains("PULL UP") && !key.contains("SHALLOW")
                        && !key.contains("THREE-PT") && !key.contains("DANGER")) {
                    normalTicks++;
                }
            }

            // Waypoint tracking: detect strategic waypoint changes and arrivals
            // by monitoring the published waypoints
            if (!output.waypoints.isEmpty()) {
                // Find the active waypoint
                Waypoint activeWp = null;
                for (var wp : output.waypoints) {
                    if (wp.active()) {
                        activeWp = wp;
                        break;
                    }
                }

                if (activeWp != null) {
                    if (currentTargetWp == null) {
                        // First waypoint target
                        currentTargetWp = new double[]{activeWp.x(), activeWp.y()};
                        trackingWaypoint = true;
                        waypointsAttempted++;
                    } else {
                        double wpDist = Math.sqrt(
                                Math.pow(activeWp.x() - currentTargetWp[0], 2)
                                + Math.pow(activeWp.y() - currentTargetWp[1], 2));
                        if (wpDist > 300) {
                            // Active waypoint changed significantly
                            // Check if we were close to the old one (arrival)
                            double distToOld = Math.sqrt(
                                    Math.pow(entity.x() - currentTargetWp[0], 2)
                                    + Math.pow(entity.y() - currentTargetWp[1], 2));
                            if (distToOld < 400) {
                                // Arrived at previous waypoint
                                waypointsReached++;
                                totalHitDistance += distToOld;
                                currentChain++;
                                if (currentChain > longestChain) longestChain = currentChain;
                                if (timeToFirstWaypoint < 0) {
                                    timeToFirstWaypoint = t / (double) TICKS_PER_SECOND;
                                }
                            } else {
                                // Abandoned waypoint (replan without arriving)
                                currentChain = 0;
                            }
                            currentTargetWp = new double[]{activeWp.x(), activeWp.y()};
                            waypointsAttempted++;
                        }
                    }
                }
            }
        }

        // Final waypoint check
        if (currentTargetWp != null) {
            double distToFinal = Math.sqrt(
                    Math.pow(entity.x() - currentTargetWp[0], 2)
                    + Math.pow(entity.y() - currentTargetWp[1], 2));
            if (distToFinal < 400) {
                waypointsReached++;
                totalHitDistance += distToFinal;
                currentChain++;
                if (currentChain > longestChain) longestChain = currentChain;
                if (timeToFirstWaypoint < 0) {
                    timeToFirstWaypoint = aliveTicks / (double) TICKS_PER_SECOND;
                }
            }
        }

        // Compute path efficiency: progress toward first strategic waypoint
        // vs total distance traveled
        double progressDist = Math.sqrt(Math.pow(entity.x() - startX, 2)
                + Math.pow(entity.y() - startY, 2));
        double pathEff = totalDistTraveled > 0 ? progressDist / totalDistTraveled : 0;

        double avgHitDist = waypointsReached > 0 ? totalHitDistance / waypointsReached : -1;

        return new Metrics(
                longestChain,
                avgHitDist,
                waypointsReached,
                waypointsAttempted,
                timeToFirstWaypoint,
                aliveTicks > 0 ? totalDepth / aliveTicks : 0,
                peakShallowest,
                aliveTicks > 0 ? totalNoiseDb / aliveTicks : 0,
                peakNoiseDb,
                aliveTicks > 0 ? totalSpeed / aliveTicks : 0,
                pathEff,
                aliveTicks > 0 ? 100.0 * normalTicks / aliveTicks : 0,
                timeOfFirstDamage,
                timeToDeath
        );
    }

    /**
     * Runs the competition and prints results.
     */
    static void compete(List<Competitor> competitors, long[] seeds, int durationTicks) {
        var allResults = new ArrayList<SeedResult>();

        for (long seed : seeds) {
            for (var comp : competitors) {
                var metrics = runOne(comp.factory(), seed, durationTicks);
                allResults.add(new SeedResult(seed, comp.name(), metrics));
            }
        }

        // Aggregate per competitor
        var aggregated = new LinkedHashMap<String, List<Metrics>>();
        for (var comp : competitors) {
            aggregated.put(comp.name(), new ArrayList<>());
        }
        for (var result : allResults) {
            aggregated.get(result.competitorName()).add(result.metrics());
        }

        // Print per-seed detail
        System.out.println("=".repeat(120));
        System.out.println("SUBMARINE COMPETITION - Per-seed results");
        System.out.println("=".repeat(120));

        for (long seed : seeds) {
            System.out.printf("%nSeed: %d%n", seed);
            System.out.printf("%-20s %5s %6s %5s %6s %7s %7s %6s %6s %5s %5s %5s %7s %7s%n",
                    "Competitor", "Chain", "HitAcc", "WpHit", "1stWP",
                    "AvgDep", "PeakDp", "AvgNs", "PeakN",
                    "Spd", "PEff", "Norm%", "1stDmg", "Death");
            System.out.println("-".repeat(120));

            for (var comp : competitors) {
                var m = allResults.stream()
                        .filter(r -> r.seed() == seed && r.competitorName().equals(comp.name()))
                        .findFirst().orElseThrow().metrics();
                printMetricsRow(comp.name(), m);
            }
        }

        // Print aggregate summary
        System.out.println();
        System.out.println("=".repeat(120));
        System.out.println("AGGREGATE SUMMARY (averages across all seeds)");
        System.out.println("=".repeat(120));
        System.out.printf("%-20s %5s %6s %5s %6s %7s %7s %6s %6s %5s %5s %5s %7s %7s  %s%n",
                "Competitor", "Chain", "HitAcc", "WpHit", "1stWP",
                "AvgDep", "PeakDp", "AvgNs", "PeakN",
                "Spd", "PEff", "Norm%", "1stDmg", "Death", "Wins");
        System.out.println("-".repeat(130));

        // Compute averages
        var avgMetrics = new LinkedHashMap<String, Metrics>();
        for (var entry : aggregated.entrySet()) {
            var list = entry.getValue();
            int n = list.size();
            avgMetrics.put(entry.getKey(), new Metrics(
                    (int) list.stream().mapToInt(Metrics::longestWaypointChain).average().orElse(0),
                    list.stream().mapToDouble(m -> m.waypointHitAccuracy() >= 0 ? m.waypointHitAccuracy() : 999)
                            .average().orElse(-1),
                    (int) list.stream().mapToInt(Metrics::waypointsReached).average().orElse(0),
                    (int) list.stream().mapToInt(Metrics::waypointsAttempted).average().orElse(0),
                    list.stream().mapToDouble(m -> m.timeToFirstWaypoint() >= 0 ? m.timeToFirstWaypoint() : 999)
                            .average().orElse(-1),
                    list.stream().mapToDouble(Metrics::avgDepth).average().orElse(0),
                    list.stream().mapToDouble(Metrics::peakDepthShallowest).average().orElse(0),
                    list.stream().mapToDouble(Metrics::avgNoiseDb).average().orElse(0),
                    list.stream().mapToDouble(Metrics::peakNoiseDb).average().orElse(0),
                    list.stream().mapToDouble(Metrics::avgSpeed).average().orElse(0),
                    list.stream().mapToDouble(Metrics::pathEfficiency).average().orElse(0),
                    list.stream().mapToDouble(Metrics::normalPatrolPct).average().orElse(0),
                    list.stream().mapToDouble(m -> m.timeOfFirstDamage() >= 0 ? m.timeOfFirstDamage() : 999)
                            .average().orElse(-1),
                    list.stream().mapToDouble(m -> m.timeToDeath() >= 0 ? m.timeToDeath() : 999)
                            .average().orElse(-1)
            ));
        }

        // Count wins per metric
        var winCounts = new LinkedHashMap<String, Integer>();
        for (var comp : competitors) winCounts.put(comp.name(), 0);

        String[] metricNames = {"Chain", "HitAcc", "WpHit", "1stWP", "AvgDep", "PeakDp",
                "AvgNs", "PeakN", "Spd", "PEff", "Norm%", "1stDmg", "Death"};
        for (var comp : competitors) {
            var m = avgMetrics.get(comp.name());
            for (var other : competitors) {
                if (other.name().equals(comp.name())) continue;
                var o = avgMetrics.get(other.name());
                int wins = 0;
                if (m.longestWaypointChain > o.longestWaypointChain) wins++;
                if (m.waypointHitAccuracy >= 0 && (o.waypointHitAccuracy < 0 || m.waypointHitAccuracy < o.waypointHitAccuracy)) wins++;
                if (m.waypointsReached > o.waypointsReached) wins++;
                if (m.timeToFirstWaypoint >= 0 && (o.timeToFirstWaypoint < 0 || m.timeToFirstWaypoint < o.timeToFirstWaypoint)) wins++;
                if (m.avgDepth < o.avgDepth) wins++; // more negative = deeper = better
                if (m.peakDepthShallowest < o.peakDepthShallowest) wins++;
                if (m.avgNoiseDb < o.avgNoiseDb) wins++;
                if (m.peakNoiseDb < o.peakNoiseDb) wins++;
                if (m.avgSpeed > o.avgSpeed) wins++;
                if (m.pathEfficiency > o.pathEfficiency) wins++;
                if (m.normalPatrolPct > o.normalPatrolPct) wins++;
                if (m.timeOfFirstDamage > o.timeOfFirstDamage || (m.timeOfFirstDamage < 0 && o.timeOfFirstDamage >= 0)) wins++;
                // Death: survived = best, else longest survival
                if (m.timeToDeath < 0 && o.timeToDeath >= 0) wins++;
                else if (m.timeToDeath >= 0 && o.timeToDeath >= 0 && m.timeToDeath > o.timeToDeath) wins++;
                winCounts.merge(comp.name(), wins, Integer::sum);
            }
        }

        for (var comp : competitors) {
            var m = avgMetrics.get(comp.name());
            System.out.printf("%-20s", comp.name());
            printMetricsValues(m);
            System.out.printf("  %d/%d%n", winCounts.get(comp.name()),
                    metricNames.length * (competitors.size() - 1));
        }

        System.out.println();
        System.out.println("Legend: Chain=longest wp chain, HitAcc=avg arrival dist (lower=better),");
        System.out.println("  WpHit=waypoints reached, 1stWP=time to first wp (lower=better),");
        System.out.println("  AvgDep/PeakDp=depth (more negative=stealthier),");
        System.out.println("  AvgNs/PeakN=noise dB (lower=stealthier), Spd=avg speed,");
        System.out.println("  PEff=path efficiency, Norm%=time in normal patrol,");
        System.out.println("  1stDmg=time to first damage (higher=better, -=none),");
        System.out.println("  Death=time to death (higher=better, -=survived)");
    }

    private static void printMetricsRow(String name, Metrics m) {
        System.out.printf("%-20s", name);
        printMetricsValues(m);
        System.out.println();
    }

    private static void printMetricsValues(Metrics m) {
        System.out.printf(" %5d %5.0fm %3d/%-2d %5s %6.0fm %6.0fm %5.0fdB %5.0fdB %4.1f %4.0f%% %4.0f%% %6s %6s",
                m.longestWaypointChain,
                m.waypointHitAccuracy >= 0 ? m.waypointHitAccuracy : 0,
                m.waypointsReached, m.waypointsAttempted,
                m.timeToFirstWaypoint >= 0 ? String.format("%.0fs", m.timeToFirstWaypoint) : "-",
                m.avgDepth, m.peakDepthShallowest,
                m.avgNoiseDb, m.peakNoiseDb,
                m.avgSpeed,
                m.pathEfficiency * 100,
                m.normalPatrolPct,
                m.timeOfFirstDamage >= 0 ? String.format("%.0fs", m.timeOfFirstDamage) : "-",
                m.timeToDeath >= 0 ? String.format("%.0fs", m.timeToDeath) : "-");
    }

    // ── Main entry point ─────────────────────────────────────────────

    public static void main(String[] args) {
        var competitors = List.of(
                new Competitor("ClaudeAttackSub", ClaudeAttackSub::new),
                new Competitor("DefaultAttackSub", DefaultAttackSub::new)
        );

        long[] seeds = {
                42L, 123456789L, -1L, 999999999999L,
                7777777L, -42L, 1234L, Long.MAX_VALUE,
                3823285984661543777L, -8551482231658960540L
        };

        compete(competitors, seeds, DEFAULT_DURATION);
    }
}
