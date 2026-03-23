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
        void setCompetitionPhase(String phase);
        void addCompetitionResult(String result);
        void clearCompetitionResults();
        void showResultsDialog(String text);
        void scheduleDelayed(long delayMs, Runnable action);
        /** Called with the nav objectives for the current phase (null to clear). */
        default void setObjectives(List<StrategicWaypoint> objectives) {}
    }

    private final ViewerCallbacks viewer;
    private final SimulationManager simManager;
    private final WorldGenerator generator = new WorldGenerator();

    private List<Competitor> competitors;
    private long[] seeds;
    private int navDurationTicks;
    private int combatDurationTicks;

    // State
    private int currentPhase = 0; // index into phases list
    private final List<Phase> phases = new ArrayList<>();
    private final Map<String, Integer> navPoints = new LinkedHashMap<>();
    private final Map<String, Integer> combatPoints = new LinkedHashMap<>();
    private final List<String> log = new ArrayList<>();

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

    void start(List<Competitor> competitors, int numSeeds, int navDurationSeconds) {
        this.competitors = competitors;
        this.navDurationTicks = navDurationSeconds * 50;
        this.combatDurationTicks = 600 * 50; // 10 min combat
        this.seeds = new long[numSeeds];
        for (int i = 0; i < numSeeds; i++) {
            seeds[i] = java.util.concurrent.ThreadLocalRandom.current().nextLong();
        }
        viewer.clearCompetitionResults();

        for (var c : competitors) {
            navPoints.put(c.name(), 0);
            combatPoints.put(c.name(), 0);
        }

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

        // Inject objectives immediately (before sim starts). The controller
        // stores them and applies them after onMatchStart on the first tick.
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

        // Track objectives
        double[] closestObj1 = {Double.MAX_VALUE};
        double[] closestObj2After1 = {Double.MAX_VALUE};
        boolean[] obj1Hit = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (cancelled) { sim.stop(); return; }
                simManager.fanOutTick(tick, submarines);

                if (submarines.isEmpty()) return;
                var s = submarines.getFirst();
                var pos = s.pose().position();

                double d1 = Math.hypot(pos.x() - objectives.x1(), pos.y() - objectives.y1());
                if (d1 < closestObj1[0]) closestObj1[0] = d1;
                if (!obj1Hit[0] && d1 < 400) obj1Hit[0] = true;
                if (obj1Hit[0]) {
                    double d2 = Math.hypot(pos.x() - objectives.x2(), pos.y() - objectives.y2());
                    if (d2 < closestObj2After1[0]) closestObj2After1[0] = d2;
                    if (d2 < 400) { matchDone = true; sim.stop(); }
                }

                if (s.hp() <= 0 || s.forfeited()) { matchDone = true; sim.stop(); }
                if (tick >= navDurationTicks) sim.stop();
            }

            @Override public void onMatchEnd() {
                // Score objectives
                int hits = 0;
                if (obj1Hit[0]) hits++;
                if (obj1Hit[0] && closestObj2After1[0] < 400) hits++;
                int pts = 0;
                if (hits >= 1) pts += 1;
                if (hits >= 2) pts += 3;
                navPoints.merge(phase.nameA(), pts, Integer::sum);

                String result = String.format("%s: %d/2 obj (%+dpts)", phase.nameA(), hits, pts);
                log.add(result);
                matchResult = result;
                viewer.addCompetitionResult(result);
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
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
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
                viewer.addCompetitionResult(combatLog);
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
        var sb = new StringBuilder();
        String seedHex = hexSeed(seed);
        sb.append(String.format("--- %s results for seed #%s ---", type, seedHex));

        // Collect results for this seed
        for (var entry : log) {
            if (entry.contains("#" + seedHex) || entry.contains(seedHex)) {
                // This is a rough match; log entries for this seed's phases
            }
        }

        // Show current standings
        sb.append(String.format("%n  %-20s %8s %8s %8s", "Competitor", "Nav", "Combat", "TOTAL"));
        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            sb.append(String.format("%n  %-20s %7dpt %7dpt %7dpt", c.name(), nav, combat, nav + combat));
        }

        String summary = sb.toString();
        System.out.println(summary);
        viewer.addCompetitionResult("--- Standings after seed #" + seedHex + " " + type + " ---");
        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            viewer.addCompetitionResult(String.format("  %-12s Nav:%dpt Combat:%dpt TOTAL:%dpt",
                    shortName(c.name()), nav, combat, nav + combat));
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
        viewer.setCompetitionPhase("FINAL STANDINGS");
        for (var c : competitors) {
            int nav = navPoints.getOrDefault(c.name(), 0);
            int combat = combatPoints.getOrDefault(c.name(), 0);
            viewer.addCompetitionResult(String.format("%-12s Nav:%dpt Combat:%dpt TOTAL:%dpt",
                    shortName(c.name()), nav, combat, nav + combat));
        }

        viewer.showResultsDialog(sb.toString());
    }
}
