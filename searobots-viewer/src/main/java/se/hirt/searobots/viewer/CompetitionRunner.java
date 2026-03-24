/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.viewer;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.SubmarineCompetition.Competitor;
import se.hirt.searobots.engine.SubmarineCompetition.Objectives;

import java.util.*;
import java.util.List;
import java.util.function.Supplier;

/**
 * Runs a competition through the viewer, showing each match live.
 * The viewer stays connected throughout; press 0 for max speed,
 * Space to skip to next match.
 */
final class CompetitionRunner {

    /** Callback interface so CompetitionRunner works without Swing. */
    interface ViewerCallbacks {
        void setTitle(String title);
        /** Compact score line shown at top during competition. */
        void setCompetitionScore(String score);
        /** Current phase label. */
        void setCompetitionPhase(String phase);
        /** Add a line to the detailed results log (toggled with a key). */
        void addDetailLine(String line);
        void clearCompetition();
        void showResultsDialog(String text);
        void scheduleDelayed(long delayMs, Runnable action);
        /** Called with the nav objectives for the current phase (null to clear). */
        default void setObjectives(List<StrategicWaypoint> objectives) {}
    }

    private final ViewerCallbacks viewer;
    private final SimulationManager simManager;
    private final WorldGenerator generator = new WorldGenerator();

    private List<Competitor> competitors;
    private long masterSeed;
    private long[] seeds;
    private int navDurationTicks;
    private int combatDurationTicks;

    // State
    private int currentPhase = 0; // index into phases list
    private final List<Phase> phases = new ArrayList<>();
    private final Map<String, Integer> navPoints = new LinkedHashMap<>();
    private final Map<String, Integer> combatPoints = new LinkedHashMap<>();
    private final List<String> log = new ArrayList<>();
    private final List<SubmarineCompetition.SeedResult> allNavResults = new ArrayList<>();

    // Simulation state
    private SimulationLoop currentSim;
    private Thread currentThread;
    private volatile boolean matchDone;
    private volatile String matchResult;
    private volatile boolean cancelled;

    record Phase(String description, long seed, PhaseType type,
                 String nameA, Supplier<SubmarineController> factoryA,
                 String nameB, Supplier<SubmarineController> factoryB) {}

    enum PhaseType { NAV, COMBAT }

    // Persist speed across phases so the user doesn't have to press 0 every time
    private int currentSpeedMultiplier = 8;

    CompetitionRunner(ViewerCallbacks viewer, SimulationManager simManager) {
        this.viewer = viewer;
        this.simManager = simManager;
    }

    void start(List<Competitor> competitors, SubmarineCompetition.CompetitionFormat format) {
        this.competitors = competitors;
        this.masterSeed = format.masterSeed();
        this.navDurationTicks = format.navDurationSeconds() * 50;
        this.combatDurationTicks = format.combatDurationSeconds() * 50;
        this.seeds = format.matchSeeds();
        viewer.clearCompetition();

        for (var c : competitors) {
            navPoints.put(c.name(), 0);
            combatPoints.put(c.name(), 0);
        }

        // Detail header
        viewer.addDetailLine("Competition seed: " + Long.toHexString(masterSeed));
        var seedList = new StringBuilder("Match seeds:");
        for (long s : seeds) seedList.append(" ").append(hexSeed(s));
        viewer.addDetailLine(seedList.toString());
        viewer.addDetailLine("");

        // Build phase list: nav phases for each competitor on each seed,
        // then combat phases for each pair on each seed
        for (long seed : seeds) {
            for (var c : competitors) {
                phases.add(new Phase(
                        String.format("NAV: %s #%s", shortName(c.name()), hexSeed(seed)),
                        seed, PhaseType.NAV, c.name(), c.factory(), null, null));
            }
        }
        for (long seed : seeds) {
            for (int i = 0; i < competitors.size(); i++) {
                for (int j = i + 1; j < competitors.size(); j++) {
                    var a = competitors.get(i);
                    var b = competitors.get(j);
                    phases.add(new Phase(
                            String.format("COMBAT: %s vs %s #%s",
                                    shortName(a.name()), shortName(b.name()), hexSeed(seed)),
                            seed, PhaseType.COMBAT, a.name(), a.factory(), b.name(), b.factory()));
                }
            }
        }

        currentPhase = 0;
        runNextPhase();
    }

    void skipToNext() {
        if (currentSim != null) {
            currentSim.stop();
        }
        // Interrupt the sim thread so it doesn't linger in sleep
        if (currentThread != null) {
            currentThread.interrupt();
        }
    }

    void stopAll() {
        cancelled = true;
        if (currentSim != null) currentSim.stop();
        if (currentThread != null) currentThread.interrupt();
        currentPhase = phases.size(); // prevent further phases
    }

    boolean isRunning() {
        return currentPhase < phases.size();
    }

    private void runNextPhase() {
        if (cancelled) return;

        // Check if we just completed a seed round (all phases for that seed/type)
        if (currentPhase > 0) {
            var prev = phases.get(currentPhase - 1);
            boolean seedDone = currentPhase >= phases.size()
                    || phases.get(currentPhase).seed() != prev.seed()
                    || phases.get(currentPhase).type() != prev.type();
            if (seedDone) {
                printSeedSummary(prev.seed(), prev.type());
            }
        }

        if (currentPhase >= phases.size()) {
            System.out.println("Competition complete!");
            showResults();
            return;
        }

        var phase = phases.get(currentPhase);
        String phaseLabel = String.format("[%d/%d] %s", currentPhase + 1, phases.size(), phase.description());
        long t0 = System.currentTimeMillis();
        System.out.println("[comp] Starting " + phaseLabel);
        viewer.setTitle("SeaRobots Competition: " + phaseLabel);
        viewer.setCompetitionPhase(phaseLabel);
        // Loading state is read from sim loop via state supplier

        if (phase.type() == PhaseType.NAV) {
            runNavPhase(phase);
        } else {
            runCombatPhase(phase);
        }
    }

    private void runNavPhase(Phase phase) {
        long t0 = System.currentTimeMillis();
        var config = MatchConfig.withDefaults(phase.seed());
        var world = generator.generate(config);
        System.out.printf("[comp] World generated in %dms%n", System.currentTimeMillis() - t0);
        var objectives = SubmarineCompetition.generateObjectives(phase.seed(), world);

        var controller = phase.factoryA().get();

        double depth1 = Math.max(-300, world.terrain().elevationAt(objectives.x1(), objectives.y1()) + 90);
        double depth2 = Math.max(-300, world.terrain().elevationAt(objectives.x2(), objectives.y2()) + 90);
        var objList = List.of(
                new StrategicWaypoint(objectives.x1(), objectives.y1(), depth1,
                        Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1),
                new StrategicWaypoint(objectives.x2(), objectives.y2(), depth2,
                        Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1));
        controller.setObjectives(objList);
        viewer.setObjectives(objList);

        var sim = new SimulationLoop();
        currentSim = sim;
        matchDone = false;
        matchResult = null;

        var controllers = List.<SubmarineController>of(controller);
        var configs = List.of(VehicleConfig.submarine());

        // Full metrics tracker
        var tracker = new NavMetricsTracker(objectives, 0);

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (cancelled) { sim.stop(); return; }
                simManager.fanOutTick(tick, submarines);
                tracker.onTick(tick, submarines, java.util.List.of());

                if (submarines.isEmpty()) return;
                var s = submarines.getFirst();
                if (s.hp() <= 0 || s.forfeited()) { sim.stop(); }
                if (tracker.objectivesHit() >= 2) { sim.stop(); }
                if (tick >= navDurationTicks) sim.stop();
            }

            @Override public void onMatchEnd() {
                tracker.onMatchEnd();
                var metrics = tracker.getMetrics();
                allNavResults.add(new SubmarineCompetition.SeedResult(phase.seed(), phase.nameA(), metrics));

                // Points are awarded in printSeedSummary when both subs have finished this seed
                String result = String.format("%s: %d/2 obj  depth:%.0fm  noise:%.0fdB  speed:%.1fm/s",
                        phase.nameA(), metrics.objectivesHit(),
                        metrics.avgDepth(), metrics.avgNoiseDb(), metrics.avgSpeed());
                log.add(result);
                matchResult = result;
                System.out.println("[comp] " + result);
                matchDone = true;

                if (!cancelled) {
                    System.out.printf("[comp] Phase done, advancing in 1.5s%n");
                    currentPhase++;
                    viewer.scheduleDelayed(1500, () -> { if (!cancelled) runNextPhase(); });
                }
            }
        };

        simManager.setWorld(world);
        sim.setSpeedMultiplier(currentSpeedMultiplier);
        currentThread = Thread.ofPlatform().daemon().name("competition-nav").start(() ->
                sim.run(world, controllers, configs, listener));
    }

    SimulationLoop currentSim() { return currentSim; }

    void setSpeed(int multiplier) {
        currentSpeedMultiplier = multiplier;
        if (currentSim != null) currentSim.setSpeedMultiplier(multiplier);
    }

    private void runCombatPhase(Phase phase) {
        long t0 = System.currentTimeMillis();
        var config = MatchConfig.withDefaults(phase.seed());
        var world = generator.generate(config);
        System.out.printf("[comp] Combat world generated in %dms%n", System.currentTimeMillis() - t0);
        viewer.setObjectives(null); // no objectives in combat phase

        var ctrlA = phase.factoryA().get();
        var ctrlB = phase.factoryB().get();

        var sim = new SimulationLoop();
        currentSim = sim;
        matchDone = false;
        matchResult = null;

        var controllers = List.<SubmarineController>of(ctrlA, ctrlB);
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        long[] combatStartMs = {System.currentTimeMillis()};
        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                if (cancelled) { sim.stop(); return; }
                if (tick == 0) {
                    System.out.printf("[comp] First combat tick after %dms%n",
                            System.currentTimeMillis() - combatStartMs[0]);
                }
                simManager.fanOutTick(tick, submarines);
                if (submarines.size() < 2) return;
                var s0 = submarines.get(0);
                var s1 = submarines.get(1);

                // Check firing solutions
                if (s0.firingSolution() != null) {
                    int pts = isBehind(s0, s1) ? 2 : 1;
                    combatPoints.merge(phase.nameA(), pts, Integer::sum);
                    matchResult = phase.nameA() + " FIRING SOLUTION (" + pts + "pt)";
                    sim.stop();
                } else if (s1.firingSolution() != null) {
                    int pts = isBehind(s1, s0) ? 2 : 1;
                    combatPoints.merge(phase.nameB(), pts, Integer::sum);
                    matchResult = phase.nameB() + " FIRING SOLUTION (" + pts + "pt)";
                    sim.stop();
                }

                boolean s0out = s0.hp() <= 0 || s0.forfeited();
                boolean s1out = s1.hp() <= 0 || s1.forfeited();
                if (s0out && !s1out) {
                    combatPoints.merge(phase.nameB(), 1, Integer::sum);
                    matchResult = phase.nameA() + (s0.forfeited() ? " LEFT ARENA" : " DIED");
                    sim.stop();
                } else if (s1out && !s0out) {
                    combatPoints.merge(phase.nameA(), 1, Integer::sum);
                    matchResult = phase.nameB() + (s1.forfeited() ? " LEFT ARENA" : " DIED");
                    sim.stop();
                } else if (s0out && s1out) {
                    matchResult = "BOTH OUT";
                    sim.stop();
                }

                if (tick >= combatDurationTicks) sim.stop();
            }

            @Override public void onMatchEnd() {
                if (matchResult == null) matchResult = "TIMEOUT";
                String combatLog = shortName(phase.nameA()) + " vs " + shortName(phase.nameB()) + ": " + matchResult;
                log.add(combatLog);
                viewer.addDetailLine(combatLog);
                updateScore();
                matchDone = true;

                if (!cancelled) {
                    System.out.printf("[comp] Phase done, advancing in 1.5s%n");
                    currentPhase++;
                    viewer.scheduleDelayed(1500, () -> { if (!cancelled) runNextPhase(); });
                }
            }
        };

        long tSetWorld = System.currentTimeMillis();
        simManager.setWorld(world);
        // Paused state is read from sim loop via supplier
        System.out.printf("[comp] setWorld in %dms%n", System.currentTimeMillis() - tSetWorld);
        sim.setSpeedMultiplier(currentSpeedMultiplier);
        currentThread = Thread.ofPlatform().daemon().name("competition-combat").start(() ->
                sim.run(world, controllers, configs, listener));
    }

    private void updateScore() {
        var sb = new StringBuilder();
        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            if (sb.length() > 0) sb.append("    ");
            sb.append(String.format("%s: %dpt", shortName(c.name()), nav + combat));
        }
        viewer.setCompetitionScore(sb.toString());
    }

    private static String shortName(String name) {
        int space = name.indexOf(' ');
        return space > 0 ? name.substring(0, space) : name;
    }

    static String hexSeed(long seed) {
        String hex = Long.toHexString(seed);
        return hex.substring(0, Math.min(8, hex.length()));
    }

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

    private void printSeedSummary(long seed, PhaseType type) {
        String seedHex = hexSeed(seed);
        // Find which seed number this is (1-based)
        int seedNum = 0;
        for (int i = 0; i < seeds.length; i++) {
            if (seeds[i] == seed) { seedNum = i + 1; break; }
        }

        if (type == PhaseType.NAV && competitors.size() == 2) {
            SubmarineCompetition.Metrics mA = null, mB = null;
            String nameA = null, nameB = null;
            for (var result : allNavResults) {
                if (result.seed() != seed) continue;
                if (mA == null) { mA = result.metrics(); nameA = result.competitorName(); }
                else { mB = result.metrics(); nameB = result.competitorName(); }
            }
            if (mA != null && mB != null) {
                // Use the shared scoring method (single source of truth)
                var score = SubmarineCompetition.scoreNavSeed(mA, mB);
                int ptsA = score.pointsA(), ptsB = score.pointsB();

                navPoints.merge(nameA, ptsA, Integer::sum);
                navPoints.merge(nameB, ptsB, Integer::sum);
                updateScore();

                // Format: "6/15 NAV Seed #abcd1234"
                String header = String.format("NAV %d/%d Seed #%s", seedNum, seeds.length, seedHex);
                String lineA = String.format("  %s %dpts (%s)", nameA, ptsA, String.join(", ", score.breakdownA()));
                String lineB = String.format("  %s %dpts (%s)", nameB, ptsB, String.join(", ", score.breakdownB()));

                viewer.addDetailLine(header);
                viewer.addDetailLine(lineA);
                viewer.addDetailLine(lineB);
                System.out.println(header);
                System.out.println(lineA);
                System.out.println(lineB);
            }
        } else if (type == PhaseType.COMBAT) {
            // Just echo the combat log line (already added by onMatchEnd)
        }
    }

    private void showResults() {
        var sb = new StringBuilder();
        sb.append("COMPETITION RESULTS\n");
        sb.append("=".repeat(60)).append("\n\n");

        sb.append(String.format("%-20s %8s %8s %8s%n", "Competitor", "Nav", "Combat", "TOTAL"));
        sb.append("-".repeat(50)).append("\n");
        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            sb.append(String.format("%-20s %7dpt %7dpt %7dpt%n", c.name(), nav, combat, nav + combat));
        }

        sb.append("\n\nMatch log:\n");
        for (var entry : log) {
            sb.append("  ").append(entry).append("\n");
        }

        viewer.setTitle("SeaRobots Competition: COMPLETE");
        viewer.setCompetitionPhase("COMPLETE");
        updateScore();
        viewer.addDetailLine("=== FINAL RESULTS ===");
        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            viewer.addDetailLine(String.format("%-12s Nav:%dpt Combat:%dpt TOTAL:%dpt",
                    shortName(c.name()), nav, combat, nav + combat));
        }

        viewer.showResultsDialog(sb.toString());
    }
}
