/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * Submarine controller competition framework. Runs multiple controller
 * implementations through the same set of seeded maps and compares
 * navigation performance metrics.
 */
package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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

    // Standard competition format
    public static final int STANDARD_NAV_SEEDS = 5;
    public static final int STANDARD_NAV_DURATION_SECONDS = 2400; // 40 minutes per nav
    public static final int STANDARD_COMBAT_DURATION_SECONDS = 1800; // 30 minutes per combat

    /**
     * A competition format: fully determined by a single master seed.
     * The master seed deterministically generates all individual match seeds.
     */
    public record CompetitionFormat(long masterSeed, int navSeeds, int navDurationSeconds,
                                     int combatDurationSeconds, long[] matchSeeds) {

        /** Standard format with the given master seed. */
        public static CompetitionFormat standard(long masterSeed) {
            return create(masterSeed, STANDARD_NAV_SEEDS, STANDARD_NAV_DURATION_SECONDS,
                    STANDARD_COMBAT_DURATION_SECONDS);
        }

        /** Create a format with custom parameters. */
        public static CompetitionFormat create(long masterSeed, int navSeeds,
                                                int navDurationSeconds, int combatDurationSeconds) {
            var rng = new java.util.Random(masterSeed);
            long[] seeds = new long[navSeeds];
            for (int i = 0; i < navSeeds; i++) {
                seeds[i] = rng.nextLong();
            }
            return new CompetitionFormat(masterSeed, navSeeds, navDurationSeconds,
                    combatDurationSeconds, seeds);
        }
    }

    public record Competitor(String name, Supplier<SubmarineController> factory) {}

    /**
     * Per-seed H2H scoring result for two competitors on one seed.
     * Points include absolute objective points + relative metric wins.
     */
    public record SeedScore(int pointsA, List<String> breakdownA,
                             int pointsB, List<String> breakdownB) {}

    /**
     * Scores two competitors' metrics for a single seed.
     * This is the single source of truth for nav scoring.
     */
    public static SeedScore scoreNavSeed(Metrics mA, Metrics mB) {
        var winsA = new ArrayList<String>();
        var winsB = new ArrayList<String>();
        int ptsA = 0, ptsB = 0;

        // Objectives (absolute: 1pt for WP1, +3pt for WP2)
        int objA = (mA.objectivesHit >= 1 ? 1 : 0) + (mA.objectivesHit >= 2 ? 3 : 0);
        int objB = (mB.objectivesHit >= 1 ? 1 : 0) + (mB.objectivesHit >= 2 ? 3 : 0);
        if (objA > 0) { ptsA += objA; winsA.add("obj:" + objA); }
        if (objB > 0) { ptsB += objB; winsB.add("obj:" + objB); }

        // Depth (2pts: more negative = stealthier)
        if (mA.avgDepth < mB.avgDepth) { ptsA += 2; winsA.add("depth:2"); }
        else if (mB.avgDepth < mA.avgDepth) { ptsB += 2; winsB.add("depth:2"); }

        // Peak depth (1pt)
        if (mA.peakDepthShallowest < mB.peakDepthShallowest) { ptsA++; winsA.add("peakDep:1"); }
        else if (mB.peakDepthShallowest < mA.peakDepthShallowest) { ptsB++; winsB.add("peakDep:1"); }

        // Noise (1pt each: lower = stealthier)
        if (mA.avgNoiseDb < mB.avgNoiseDb) { ptsA++; winsA.add("noise:1"); }
        else if (mB.avgNoiseDb < mA.avgNoiseDb) { ptsB++; winsB.add("noise:1"); }
        if (mA.peakNoiseDb < mB.peakNoiseDb) { ptsA++; winsA.add("peakNs:1"); }
        else if (mB.peakNoiseDb < mA.peakNoiseDb) { ptsB++; winsB.add("peakNs:1"); }

        // Speed (1pt: faster = better)
        if (mA.avgSpeed > mB.avgSpeed) { ptsA++; winsA.add("speed:1"); }
        else if (mB.avgSpeed > mA.avgSpeed) { ptsB++; winsB.add("speed:1"); }

        // Normal patrol % (1pt)
        if (mA.normalPatrolPct > mB.normalPatrolPct) { ptsA++; winsA.add("patrol:1"); }
        else if (mB.normalPatrolPct > mA.normalPatrolPct) { ptsB++; winsB.add("patrol:1"); }

        // Late damage (1pt)
        boolean aUndamaged = mA.timeOfFirstDamage < 0;
        boolean bUndamaged = mB.timeOfFirstDamage < 0;
        if (aUndamaged && !bUndamaged) { ptsA++; winsA.add("noDmg:1"); }
        else if (bUndamaged && !aUndamaged) { ptsB++; winsB.add("noDmg:1"); }
        else if (!aUndamaged && !bUndamaged && mA.timeOfFirstDamage > mB.timeOfFirstDamage) { ptsA++; winsA.add("lateDmg:1"); }
        else if (!aUndamaged && !bUndamaged && mB.timeOfFirstDamage > mA.timeOfFirstDamage) { ptsB++; winsB.add("lateDmg:1"); }

        // Survive (2pts)
        boolean aSurvived = mA.timeToDeath < 0;
        boolean bSurvived = mB.timeToDeath < 0;
        if (aSurvived && !bSurvived) { ptsA += 2; winsA.add("survive:2"); }
        else if (bSurvived && !aSurvived) { ptsB += 2; winsB.add("survive:2"); }
        else if (!aSurvived && !bSurvived && mA.timeToDeath > mB.timeToDeath) { ptsA += 2; winsA.add("survive:2"); }
        else if (!aSurvived && !bSurvived && mB.timeToDeath > mA.timeToDeath) { ptsB += 2; winsB.add("survive:2"); }

        return new SeedScore(ptsA, winsA, ptsB, winsB);
    }

    public record Metrics(
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

    public record SeedResult(long seed, String competitorName, Metrics metrics) {}

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
     * Runs one competitor on one seed using SimulationLoop + NavMetricsTracker.
     * This is the same simulation path used by the live viewer, ensuring
     * identical results whether running headless or with the UI.
     */
    static Metrics runOne(Supplier<SubmarineController> factory, long seed,
                           int durationTicks, Objectives objectives) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();

        var controller = factory.get();

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

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000); // run as fast as possible

        var tracker = new NavMetricsTracker(objectives, 0);
        var controllers = List.<SubmarineController>of(controller);
        var configs = List.of(VehicleConfig.submarine());

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                tracker.onTick(tick, submarines, java.util.List.of());
                if (submarines.isEmpty()) return;
                var s = submarines.getFirst();
                if (s.hp() <= 0 || s.forfeited()) sim.stop();
                if (tracker.objectivesHit() >= 2) sim.stop();
                if (tick >= durationTicks) sim.stop();
            }
            @Override public void onMatchEnd() {
                tracker.onMatchEnd();
            }
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        return tracker.getMetrics();
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

        // Per-seed H2H scoring using the shared scoreNavSeed() method
        // (same scoring as the live viewer uses)
        var winCounts = new LinkedHashMap<String, Integer>();
        for (var comp : competitors) winCounts.put(comp.name(), 0);

        if (competitors.size() == 2) {
            String nameA = competitors.get(0).name();
            String nameB = competitors.get(1).name();

            System.out.println();
            System.out.println("=".repeat(120));
            System.out.println("PER-SEED H2H SCORING");
            System.out.println("=".repeat(120));

            for (long seed : seeds) {
                var mA = allResults.stream()
                        .filter(r -> r.seed() == seed && r.competitorName().equals(nameA))
                        .findFirst().orElseThrow().metrics();
                var mB = allResults.stream()
                        .filter(r -> r.seed() == seed && r.competitorName().equals(nameB))
                        .findFirst().orElseThrow().metrics();

                var score = scoreNavSeed(mA, mB);
                winCounts.merge(nameA, score.pointsA(), Integer::sum);
                winCounts.merge(nameB, score.pointsB(), Integer::sum);

                System.out.printf("Seed #%s:  %s %dpts (%s)  |  %s %dpts (%s)%n",
                        Long.toHexString(seed).substring(0, Math.min(8, Long.toHexString(seed).length())),
                        nameA, score.pointsA(), String.join(", ", score.breakdownA()),
                        nameB, score.pointsB(), String.join(", ", score.breakdownB()));
            }
        }

        System.out.println();
        System.out.println("NAV TOTALS:");
        for (var comp : competitors) {
            System.out.printf("  %-20s %dpts%n", comp.name(), winCounts.get(comp.name()));
        }

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

    /**
     * Combat result: points for each competitor + description.
     * Scoring: kill enemy = 5pts, survive = 5pts. Max 10pts per side.
     */
    record CombatResult(int pointsA, int pointsB, String description, long ticks) {}

    /**
     * Pits two competitors head-to-head on a seed. Match runs to completion
     * (or time limit). Both sides score independently: 5pts for killing the
     * enemy, 5pts for surviving.
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

        boolean[] aAlive = {true}, bAlive = {true};
        long[] endTick = {durationTicks};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
                if (submarines.size() < 2) return;
                var s0 = submarines.get(0);
                var s1 = submarines.get(1);

                aAlive[0] = s0.hp() > 0 && !s0.forfeited();
                bAlive[0] = s1.hp() > 0 && !s1.forfeited();

                // End match early if both are dead
                if (!aAlive[0] && !bAlive[0]) { endTick[0] = tick; sim.stop(); }

                if (tick >= durationTicks) sim.stop();
            }

            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        // Score: kill=5pts, survive=5pts
        int ptsA = 0, ptsB = 0;
        var reasons = new ArrayList<String>();
        if (!bAlive[0]) { ptsA += 5; reasons.add(nameA + " KILLED " + nameB); }
        if (!aAlive[0]) { ptsB += 5; reasons.add(nameB + " KILLED " + nameA); }
        if (aAlive[0]) { ptsA += 5; reasons.add(nameA + " SURVIVED"); }
        if (bAlive[0]) { ptsB += 5; reasons.add(nameB + " SURVIVED"); }

        return new CombatResult(ptsA, ptsB, String.join(", ", reasons), endTick[0]);
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

                    System.out.printf("  %-18s vs %-18s -> %s (%ds)  [%s:%dpt %s:%dpt]%n",
                            a.name(), b.name(), result.description(), result.ticks() / 50,
                            a.name(), result.pointsA(), b.name(), result.pointsB());

                    combatPoints.merge(a.name(), result.pointsA(), Integer::sum);
                    combatPoints.merge(b.name(), result.pointsB(), Integer::sum);
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
     * Usage: SubmarineCompetition [masterSeed_hex] [--combat-only]
     *
     * With no arguments: generates a random master seed for a standard competition.
     * With a hex seed: runs a reproducible standard competition with that master seed.
     * With --combat-only: skips the navigation phase and runs combat only (faster iteration).
     * The master seed deterministically generates all match seeds.
     */
    public static void main(String[] args) {
        var competitors = List.of(
                new Competitor("CodexAttackSub", CodexAttackSub::new),
                new Competitor("ClaudeAttackSub", ClaudeAttackSub::new)
        );

        boolean combatOnly = false;
        long masterSeed = java.util.concurrent.ThreadLocalRandom.current().nextLong();

        for (String arg : args) {
            if ("--combat-only".equals(arg)) {
                combatOnly = true;
            } else {
                masterSeed = Long.parseUnsignedLong(arg, 16);
            }
        }

        var format = CompetitionFormat.standard(masterSeed);

        System.out.printf("Competition: master seed %s, %d seeds, %ds nav / %ds combat%s%n",
                Long.toHexString(masterSeed), format.navSeeds(),
                format.navDurationSeconds(), format.combatDurationSeconds(),
                combatOnly ? " [COMBAT ONLY]" : "");
        System.out.print("Match seeds:");
        for (long s : format.matchSeeds()) System.out.printf(" %s", Long.toHexString(s));
        System.out.println("\n");

        Map<String, Integer> navPoints;
        if (combatOnly) {
            navPoints = new LinkedHashMap<>();
            for (var c : competitors) navPoints.put(c.name(), 0);
        } else {
            navPoints = compete(competitors, format.matchSeeds(),
                    TICKS_PER_SECOND * format.navDurationSeconds());
            System.out.println();
        }

        // Combat scenario
        runCombatScenario(competitors, format.matchSeeds(),
                TICKS_PER_SECOND * format.combatDurationSeconds(), navPoints);
    }

}
