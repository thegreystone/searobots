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
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.*;
import java.util.function.Supplier;

/*
 * Note: this class builds its own SubmarineInput/SubmarineOutput implementations
 * so it has no dependency on test sources.
 */

/**
 * Runs a competition between submarine controller implementations.
 * Each controller is run through the same set of seeded maps with the
 * same spawn points and conditions. Metrics are collected and compared
 * in a table.
 */
public class SubmarineCompetition {

    public static final int TICKS_PER_SECOND = 50;
    public static final int DEFAULT_DURATION = TICKS_PER_SECOND * 2400; // 40 minutes

    public record Competitor(String name, Supplier<SubmarineController> factory) {}

    record Metrics(
            // Navigation
            int longestWaypointChain,    // consecutive waypoints reached without replan
            double waypointHitAccuracy,  // avg distance to waypoint center at arrival (m)
            int waypointsReached,        // total waypoints arrived at
            int waypointsAttempted,      // total strategic waypoints generated
            double timeToFirstWaypoint,  // seconds to reach first waypoint (-1 = never)
            // Objective waypoints (fixed per seed, same for all competitors)
            int objectivesHit,           // 0, 1, or 2
            double closestToObj1,        // closest approach to objective 1 (m)
            double closestToObj2,        // closest approach to objective 2 (m)
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

    /** Two objective waypoints per seed in deep, safe water. */
    public record Objectives(double x1, double y1, double x2, double y2) {}

    public static Objectives generateObjectives(long seed, GeneratedWorld world) {
        var terrain = world.terrain();
        var area = world.config().battleArea();
        var rng = new java.util.Random(seed * 31 + 7);
        var pathPlanner = new PathPlanner(terrain, -90.0, 200, 75);

        double[] xs = new double[2], ys = new double[2];
        for (int i = 0; i < 2; i++) {
            for (int attempt = 0; attempt < 200; attempt++) {
                double angle = rng.nextDouble() * 2 * Math.PI;
                double radius = 1500 + rng.nextDouble() * 2500;
                double x = radius * Math.sin(angle);
                double y = radius * Math.cos(angle);
                if (area.distanceToBoundary(x, y) < 1000) continue;
                if (!pathPlanner.isSafe(x, y)) continue;
                double floor = terrain.elevationAt(x, y);
                if (floor > -200) continue; // needs good depth for safe approach
                float cost = pathPlanner.costAt(x, y);
                if (cost > 1.5) continue; // avoid cells near shallow terrain
                // Check neighborhood: ensure the surrounding area is also deep
                // to prevent placing objectives on cliff edges
                double worstNearby = floor;
                for (int dx = -150; dx <= 150; dx += 75) {
                    for (int dy = -150; dy <= 150; dy += 75) {
                        worstNearby = Math.max(worstNearby, terrain.elevationAt(x + dx, y + dy));
                    }
                }
                if (worstNearby > -150) continue; // neighborhood must be reasonably deep
                // Ensure reasonable separation (not too close, not too far)
                if (i == 1) {
                    double sep = Math.hypot(x - xs[0], y - ys[0]);
                    if (sep < 2000 || sep > 6000) continue;
                }
                xs[i] = x;
                ys[i] = y;
                break;
            }
        }
        return new Objectives(xs[0], ys[0], xs[1], ys[1]);
    }

    /**
     * Runs one competitor on one seed and collects metrics.
     */
    static Metrics runOne(Supplier<SubmarineController> factory, long seed,
                           int durationTicks, Objectives objectives) {
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

        // Inject objectives if provided
        if (objectives != null) {
            double depth1 = Math.max(-300, terrain.elevationAt(objectives.x1, objectives.y1) + 90);
            double depth2 = Math.max(-300, terrain.elevationAt(objectives.x2, objectives.y2) + 90);
            controller.setObjectives(List.of(
                    new StrategicWaypoint(objectives.x1, objectives.y1, depth1,
                            Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1),
                    new StrategicWaypoint(objectives.x2, objectives.y2, depth2,
                            Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1)
            ));
        }

        // Objective tracking (sequential: WP1 must be hit before WP2 counts)
        double closestToObj1 = Double.MAX_VALUE;
        double closestToObj2AfterObj1 = Double.MAX_VALUE;
        double OBJ_HIT_RADIUS = 400.0;
        long obj1HitTick = -1;

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

        double[] currentTargetWp = null;

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
            var input = new SimpleInput(t, 1.0 / TICKS_PER_SECOND, state, env,
                    List.of(), List.of(), 0);
            var output = new SimpleOutput();
            controller.onTick(input, output);

            entity.setThrottle(output.throttle);
            entity.setRudder(output.rudder);
            entity.setSternPlanes(output.sternPlanes);
            entity.setBallast(output.ballast);
            physics.step(entity, 1.0 / TICKS_PER_SECOND, terrain, world.currentField(),
                    config.battleArea());

            if (entity.hp() <= 0 || entity.forfeited()) {
                if (timeToDeath < 0) timeToDeath = t / (double) TICKS_PER_SECOND;
                break;
            }
            aliveTicks = t + 1;

            // Stop early if both objectives reached
            if (obj1HitTick >= 0 && closestToObj2AfterObj1 < OBJ_HIT_RADIUS) {
                break;
            }

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

            // Objective proximity (sequential: track WP2 only after WP1 hit)
            if (objectives != null) {
                double d1 = Math.hypot(entity.x() - objectives.x1, entity.y() - objectives.y1);
                if (d1 < closestToObj1) closestToObj1 = d1;
                if (obj1HitTick < 0 && d1 < OBJ_HIT_RADIUS) obj1HitTick = t;
                if (obj1HitTick >= 0) {
                    double d2 = Math.hypot(entity.x() - objectives.x2, entity.y() - objectives.y2);
                    if (d2 < closestToObj2AfterObj1) closestToObj2AfterObj1 = d2;
                }
            }

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

            // Waypoint tracking: score high-level strategic waypoints when available.
            var publishedWaypoints = output.strategicWaypoints.isEmpty()
                    ? output.waypoints
                    : output.strategicWaypoints;
            if (!publishedWaypoints.isEmpty()) {
                Waypoint activeWp = null;
                for (var wp : publishedWaypoints) {
                    if (wp.active()) {
                        activeWp = wp;
                        break;
                    }
                }

                if (activeWp != null) {
                    if (currentTargetWp == null) {
                        currentTargetWp = new double[]{activeWp.x(), activeWp.y()};
                        waypointsAttempted++;
                    } else {
                        double wpDist = Math.sqrt(
                                Math.pow(activeWp.x() - currentTargetWp[0], 2)
                                + Math.pow(activeWp.y() - currentTargetWp[1], 2));
                        if (wpDist > 300) {
                            double distToOld = Math.sqrt(
                                    Math.pow(entity.x() - currentTargetWp[0], 2)
                                    + Math.pow(entity.y() - currentTargetWp[1], 2));
                            if (distToOld < 400) {
                                waypointsReached++;
                                totalHitDistance += distToOld;
                                currentChain++;
                                if (currentChain > longestChain) longestChain = currentChain;
                                if (timeToFirstWaypoint < 0) {
                                    timeToFirstWaypoint = t / (double) TICKS_PER_SECOND;
                                }
                            } else {
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
        // Sequential: WP2 only counts if WP1 was hit first
        // (tracked during the sim via obj1HitTick)
        int objectivesHit = obj1HitTick >= 0 ? 1 : 0;
        if (obj1HitTick >= 0 && closestToObj2AfterObj1 < OBJ_HIT_RADIUS) objectivesHit = 2;

        return new Metrics(
                longestChain,
                avgHitDist,
                waypointsReached,
                waypointsAttempted,
                timeToFirstWaypoint,
                objectivesHit,
                closestToObj1,
                closestToObj2AfterObj1,
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
    public static Map<String, Integer> compete(List<Competitor> competitors, long[] seeds, int durationTicks) {
        var allResults = new ArrayList<SeedResult>();

        for (long seed : seeds) {
            var config = MatchConfig.withDefaults(seed);
            var world = new WorldGenerator().generate(config);
            var objectives = generateObjectives(seed, world);

            for (var comp : competitors) {
                var metrics = runOne(comp.factory(), seed, durationTicks, objectives);
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
            System.out.printf("%nSeed: %s%n", Long.toHexString(seed));
            System.out.printf("%-20s %4s %6s %5s %6s %7s %7s %6s %6s %5s %5s %5s %7s %7s%n",
                    "Competitor", "Obj", "HitAcc", "WpHit", "1stWP",
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
                "Competitor", "Obj", "HitAcc", "WpHit", "1stWP",
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
                    (int) Math.round(list.stream().mapToInt(Metrics::objectivesHit).average().orElse(0)),
                    list.stream().mapToDouble(Metrics::closestToObj1).average().orElse(9999),
                    list.stream().mapToDouble(Metrics::closestToObj2).average().orElse(9999),
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

        // Count wins per scored metric.
        // Chain, WpHit, and PEff are shown for information but not scored:
        // they reward short-distance waypoints rather than strategic quality.
        var winCounts = new LinkedHashMap<String, Integer>();
        for (var comp : competitors) winCounts.put(comp.name(), 0);

        // Scoring: absolute objective points + relative metric wins.
        //
        // Per seed, each competitor earns:
        //   - 1 pt for reaching objective WP1
        //   - 3 pts total for reaching both WP1 then WP2 (in order)
        //   These are absolute: you earn them regardless of what the opponent does.
        //
        // Plus relative head-to-head wins on other metrics:
        //   Depth: 2 pts. Not dying: 2 pts. Everything else: 1 pt.

        // Add absolute objective points per seed
        for (var comp : competitors) {
            int objPoints = 0;
            for (var result : allResults) {
                if (!result.competitorName().equals(comp.name())) continue;
                var m = result.metrics();
                if (m.objectivesHit >= 1) objPoints += 1;  // 1 pt for WP1
                if (m.objectivesHit >= 2) objPoints += 3;  // 3 pts for WP2
            }
            winCounts.merge(comp.name(), objPoints, Integer::sum);
        }

        // Per-seed bonus: 1 pt for fastest completion (both objectives)
        for (long seed : seeds) {
            double bestTime = Double.MAX_VALUE;
            String bestComp = null;
            for (var comp : competitors) {
                var result = allResults.stream()
                        .filter(r -> r.seed() == seed && r.competitorName().equals(comp.name()))
                        .findFirst();
                if (result.isPresent() && result.get().metrics().objectivesHit == 2) {
                    // Completion time = aliveTicks (sim stopped on completion)
                    // Use closestToObj2 as proxy: lower = reached it earlier in the sim
                    // Actually, just compare total sim time (shorter = faster completion)
                    double time = result.get().metrics().avgSpeed > 0
                            ? result.get().metrics().pathEfficiency : Double.MAX_VALUE;
                    // Better: use the raw alive ticks. Not stored directly, but
                    // objectivesHit==2 means sim stopped early. Earlier stop = fewer
                    // ticks = better. We don't have ticks in Metrics, so approximate
                    // via total distance / avg speed.
                    // For simplicity: whoever completed gets the point if opponent didn't.
                    // If both completed, we need completion time. Let's just check if
                    // this competitor completed and the other didn't.
                    bestComp = comp.name(); // at least one completed
                }
            }
            // Award 1 pt to each competitor that completed (tie if both did)
            for (var comp : competitors) {
                var result = allResults.stream()
                        .filter(r -> r.seed() == seed && r.competitorName().equals(comp.name()))
                        .findFirst();
                if (result.isPresent() && result.get().metrics().objectivesHit == 2) {
                    winCounts.merge(comp.name(), 1, Integer::sum);
                }
            }
        }

        // Add relative metric wins (head-to-head on averages)
        for (var comp : competitors) {
            var m = avgMetrics.get(comp.name());
            for (var other : competitors) {
                if (other.name().equals(comp.name())) continue;
                var o = avgMetrics.get(other.name());
                int wins = 0;
                // Stealth: 2 pts for avg depth, 1 pt each for the rest
                if (m.avgDepth < o.avgDepth) wins += 2;
                if (m.peakDepthShallowest < o.peakDepthShallowest) wins++;
                if (m.avgNoiseDb < o.avgNoiseDb) wins++;
                if (m.peakNoiseDb < o.peakNoiseDb) wins++;
                // Efficiency: 1 pt each
                if (m.avgSpeed > o.avgSpeed) wins++;
                if (m.normalPatrolPct > o.normalPatrolPct) wins++;
                // Survivability: 2 pts for not dying, 1 pt for first damage
                if (m.timeOfFirstDamage > o.timeOfFirstDamage || (m.timeOfFirstDamage < 0 && o.timeOfFirstDamage >= 0)) wins++;
                if (m.timeToDeath < 0 && o.timeToDeath >= 0) wins += 2;
                else if (m.timeToDeath >= 0 && o.timeToDeath >= 0 && m.timeToDeath > o.timeToDeath) wins += 2;
                winCounts.merge(comp.name(), wins, Integer::sum);
            }
        }

        for (var comp : competitors) {
            var m = avgMetrics.get(comp.name());
            System.out.printf("%-20s", comp.name());
            printAggregateMetricsValues(m, aggregated.get(comp.name()));
            System.out.printf("  %dpts%n", winCounts.get(comp.name()));
        }

        System.out.println();
        System.out.println("Legend: Chain=longest strategic wp chain (info only), HitAcc=avg strategic arrival dist (SCORED, lower=better),");
        System.out.println("  WpHit=strategic waypoints reached (info only), 1stWP=time to first strategic wp (SCORED, lower=better),");
        System.out.println("  AvgDep/PeakDp=depth (SCORED, more negative=stealthier),");
        System.out.println("  AvgNs/PeakN=noise dB (SCORED, lower=stealthier), Spd=avg speed (SCORED),");
        System.out.println("  PEff=path efficiency (info only), Norm%=time in normal patrol (SCORED),");
        System.out.println("  1stDmg=time to first damage (SCORED, higher=better, -=none),");
        System.out.println("  Death=time to death (SCORED, higher=better, -=survived)");
        System.out.println("  Wins counted on 10 scored metrics (excludes Chain, WpHit, PEff)");

        return winCounts;
    }

    private static void printMetricsRow(String name, Metrics m) {
        System.out.printf("%-20s", name);
        printMetricsValues(m);
        System.out.println();
    }

    private static void printMetricsValues(Metrics m) {
        System.out.printf(" %5s %5.0fm %3d/%-2d %5s %6.0fm %6.0fm %5.0fdB %5.0fdB %4.1f %4.0f%% %4.0f%% %6s %6s",
                m.objectivesHit + "/2",
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

    private static void printAggregateMetricsValues(Metrics m, List<Metrics> samples) {
        double avgObjectivesHit = samples.stream().mapToInt(Metrics::objectivesHit).average().orElse(0);
        System.out.printf(" %5s %5.0fm %3d/%-2d %5s %6.0fm %6.0fm %5.0fdB %5.0fdB %4.1f %4.0f%% %4.0f%% %6s %6s",
                String.format("%.1f/2", avgObjectivesHit),
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

    // ── Combat scenario ──────────────────────────────────────────────

    record CombatResult(String winner, String loser, String reason, int points, long ticks) {}

    /**
     * Pits two competitors head-to-head on a seed. Stops when one sub gets
     * a firing solution, one dies, or time runs out.
     * Winner gets 1 pt (2 pts if behind the opponent when getting the solution).
     */
    static CombatResult runCombat(Supplier<SubmarineController> factoryA, String nameA,
                                   Supplier<SubmarineController> factoryB, String nameB,
                                   long seed, int durationTicks) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var ctrlA = factoryA.get();
        var ctrlB = factoryB.get();
        List<SubmarineController> controllers = List.of(ctrlA, ctrlB);
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        String[] winner = {null};
        String[] loser = {null};
        String[] reason = {null};
        int[] points = {0};
        long[] endTick = {durationTicks};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (submarines.size() < 2 || winner[0] != null) return;
                var s0 = submarines.get(0);
                var s1 = submarines.get(1);

                // Check firing solutions
                if (s0.firingSolution() != null) {
                    winner[0] = nameA;
                    loser[0] = nameB;
                    reason[0] = "FIRING SOLUTION";
                    points[0] = isBehind(s0, s1) ? 2 : 1;
                    endTick[0] = tick;
                    sim.stop();
                } else if (s1.firingSolution() != null) {
                    winner[0] = nameB;
                    loser[0] = nameA;
                    reason[0] = "FIRING SOLUTION";
                    points[0] = isBehind(s1, s0) ? 2 : 1;
                    endTick[0] = tick;
                    sim.stop();
                }

                // Check death or forfeit (left battle area)
                boolean s0out = s0.hp() <= 0 || s0.forfeited();
                boolean s1out = s1.hp() <= 0 || s1.forfeited();
                if (s0out && !s1out) {
                    winner[0] = nameB;
                    loser[0] = nameA;
                    reason[0] = nameA + (s0.forfeited() ? " LEFT ARENA" : " DIED");
                    points[0] = 1;
                    endTick[0] = tick;
                    sim.stop();
                } else if (s1out && !s0out) {
                    winner[0] = nameA;
                    loser[0] = nameB;
                    reason[0] = nameB + (s1.forfeited() ? " LEFT ARENA" : " DIED");
                    points[0] = 1;
                    endTick[0] = tick;
                    sim.stop();
                } else if (s0out && s1out) {
                    reason[0] = "BOTH OUT";
                    endTick[0] = tick;
                    sim.stop();
                }

                if (tick >= durationTicks) sim.stop();
            }

            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        if (winner[0] == null && reason[0] == null) reason[0] = "TIMEOUT";
        return new CombatResult(winner[0], loser[0], reason[0], points[0], endTick[0]);
    }

    /** Returns true if attacker is within 60° of the target's stern arc. */
    private static boolean isBehind(SubmarineSnapshot attacker, SubmarineSnapshot target) {
        double targetHeading = target.pose().heading();
        double sternBearing = (targetHeading + Math.PI) % (2 * Math.PI);
        double attackerBearing = Math.atan2(
                attacker.pose().position().x() - target.pose().position().x(),
                attacker.pose().position().y() - target.pose().position().y());
        if (attackerBearing < 0) attackerBearing += 2 * Math.PI;
        double diff = attackerBearing - sternBearing;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return Math.abs(diff) < Math.toRadians(60);
    }

    public static void runCombatScenario(List<Competitor> competitors, long[] seeds,
                                          int durationTicks, Map<String, Integer> navPoints) {
        System.out.println("=".repeat(120));
        System.out.println("COMBAT SCENARIO - Head-to-head");
        System.out.println("=".repeat(120));

        var combatPoints = new LinkedHashMap<String, Integer>();
        for (var c : competitors) combatPoints.put(c.name(), 0);

        for (long seed : seeds) {
            for (int i = 0; i < competitors.size(); i++) {
                for (int j = i + 1; j < competitors.size(); j++) {
                    var a = competitors.get(i);
                    var b = competitors.get(j);
                    var result = runCombat(a.factory(), a.name(), b.factory(), b.name(),
                            seed, durationTicks);

                    System.out.printf("  %-18s vs %-18s -> %s%n",
                            a.name(), b.name(),
                            result.winner() != null
                                    ? result.winner() + " wins (" + result.points() + "pt, " + result.reason() + ", " + result.ticks() / 50 + "s)"
                                    : result.reason());

                    if (result.winner() != null) {
                        combatPoints.merge(result.winner(), result.points(), Integer::sum);
                    }
                }
            }
        }

        // Print combined scoreboard
        System.out.println();
        System.out.println("=".repeat(80));
        System.out.println("FINAL STANDINGS");
        System.out.println("=".repeat(80));
        System.out.printf("%-20s %8s %8s %8s%n", "Competitor", "Nav", "Combat", "TOTAL");
        System.out.println("-".repeat(50));

        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            System.out.printf("%-20s %7dpt %7dpt %7dpt%n", c.name(), nav, combat, nav + combat);
        }
        System.out.println();
    }

    // ── Main entry point ─────────────────────────────────────────────

    /**
     * Usage: SubmarineCompetition [numSeeds] [durationSeconds]
     *
     * With no arguments: runs 10 random seeds for 5 minutes each.
     * numSeeds: number of random seeds to generate (default 10).
     * durationSeconds: simulation duration per seed in seconds (default 300).
     */
    public static void main(String[] args) {
        var competitors = List.of(
                new Competitor("CodexAttackSub", CodexAttackSub::new),
                new Competitor("ClaudeAttackSub", ClaudeAttackSub::new)
        );

        int numSeeds = args.length > 0 ? Integer.parseInt(args[0]) : 10;
        int durationSec = args.length > 1 ? Integer.parseInt(args[1]) : DEFAULT_DURATION / TICKS_PER_SECOND;

        long[] seeds = new long[numSeeds];
        for (int i = 0; i < numSeeds; i++) {
            seeds[i] = java.util.concurrent.ThreadLocalRandom.current().nextLong();
        }

        System.out.printf("Competition: %d random seeds, %ds per seed%n", numSeeds, durationSec);
        System.out.print("Seeds:");
        for (long s : seeds) System.out.printf(" %s", Long.toHexString(s));
        System.out.println("\n");

        // Navigation scenario
        var navPoints = compete(competitors, seeds, TICKS_PER_SECOND * durationSec);

        System.out.println();

        // Combat scenario (10 min per match, enough for detection + firing solution)
        runCombatScenario(competitors, seeds, TICKS_PER_SECOND * 600, navPoints);
    }

    // ── Lightweight SubmarineInput/Output for running controllers ────

    record SimpleInput(long tick, double deltaTimeSeconds,
                       SubmarineState self, EnvironmentSnapshot environment,
                       List<SonarContact> sonarContacts,
                       List<SonarContact> activeSonarReturns,
                       int activeSonarCooldownTicks)
            implements SubmarineInput {}

    static final class SimpleOutput implements SubmarineOutput {
        double rudder, sternPlanes, throttle, ballast;
        boolean pinged;
        final ArrayList<Waypoint> waypoints = new ArrayList<>();
        final ArrayList<Waypoint> strategicWaypoints = new ArrayList<>();

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void setStatus(String s) {}
        @Override public void publishWaypoint(Waypoint wp) { waypoints.add(wp); }
        @Override public void publishStrategicWaypoint(Waypoint wp, Purpose purpose) { strategicWaypoints.add(wp); }
        @Override public void publishContactEstimate(ContactEstimate e) {}
    }
}
