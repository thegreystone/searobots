/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.viewer;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Manages the lifecycle of SimulationLoop instances and fans out tick events
 * to all registered listeners (2D panel, 3D scene, recorders, competition
 * scorers, etc.). Any component that needs simulation data registers once;
 * it doesn't matter whether the sim is free patrol, competition, or a test.
 */
final class SimulationManager {

    private final CopyOnWriteArrayList<SimulationListener> listeners = new CopyOnWriteArrayList<>();
    private volatile SimulationLoop currentLoop;
    private volatile Thread currentThread;

    // Pause-on-event flags
    volatile boolean pauseOnDeath;
    volatile boolean pauseOnTorpedoSolution;
    volatile boolean pauseOnTorpedoLaunch;
    volatile boolean injectObjectives;
    private boolean torpedoSolutionTriggered;
    private final java.util.Set<Integer> deadEntities =
            java.util.Collections.synchronizedSet(new java.util.HashSet<>());

    // ── Listener management ─────────────────────────────────────────

    void addListener(SimulationListener listener) {
        listeners.addIfAbsent(listener);
    }

    void removeListener(SimulationListener listener) {
        listeners.remove(listener);
    }

    // ── Simulation lifecycle ────────────────────────────────────────

    /**
     * Starts a new simulation. Stops any running simulation first and
     * waits for its thread to finish.
     */
    void start(GeneratedWorld world, List<SubmarineController> controllers,
               List<VehicleConfig> vehicleConfigs, List<Double> headings) {
        stop();

        deadEntities.clear();
        torpedoSolutionTriggered = false;

        var sim = new SimulationLoop();
        currentLoop = sim;

        var fanOut = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                // Fan out to all registered listeners
                for (var l : listeners) {
                    l.onTick(tick, submarines, torpedoes);
                }

                // Pause-on-death
                if (pauseOnDeath) {
                    for (var sub : submarines) {
                        if (sub.hp() <= 0 && !deadEntities.contains(sub.id())) {
                            deadEntities.add(sub.id());
                            sim.setPaused(true);
                        }
                    }
                }

                // Pause on first torpedo solution
                if (pauseOnTorpedoSolution && !torpedoSolutionTriggered) {
                    for (var sub : submarines) {
                        if (sub.firingSolution() != null) {
                            torpedoSolutionTriggered = true;
                            sim.setPaused(true);
                            var sol = sub.firingSolution();
                            System.out.printf("TORPEDO SOLUTION at tick %d: %s target=[%.0f,%.0f] hdg=%.0f spd=%.1f q=%.2f%n",
                                    tick, sub.name(), sol.targetX(), sol.targetY(),
                                    Math.toDegrees(sol.targetHeading()), sol.targetSpeed(), sol.quality());
                            break;
                        }
                    }
                }

                // Pause on torpedo launch
                if (pauseOnTorpedoLaunch && torpedoes != null && !torpedoes.isEmpty()) {
                    // Check for new torpedoes (fuel near max = just launched)
                    for (var t : torpedoes) {
                        if (t.fuelRemaining() > 119.0) { // just launched (120s max fuel)
                            sim.setPaused(true);
                            System.out.printf("TORPEDO LAUNCHED at tick %d: torpedo %d from sub %d%n",
                                    tick, t.id(), t.ownerId());
                            break;
                        }
                    }
                }
            }

            @Override
            public void onMatchEnd() {
                for (var l : listeners) {
                    l.onMatchEnd();
                }
            }
        };

        // Inject competition objectives if enabled
        if (injectObjectives) {
            var objectives = SubmarineCompetition.generateObjectives(
                    world.config().worldSeed(), world);
            var terrain = world.terrain();
            double depth1 = Math.max(-300, terrain.elevationAt(objectives.x1(), objectives.y1()) + 90);
            double depth2 = Math.max(-300, terrain.elevationAt(objectives.x2(), objectives.y2()) + 90);
            var objList = java.util.List.of(
                    new StrategicWaypoint(objectives.x1(), objectives.y1(), depth1,
                            Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1),
                    new StrategicWaypoint(objectives.x2(), objectives.y2(), depth2,
                            Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1));
            for (var ctrl : controllers) {
                ctrl.setObjectives(objList);
            }
            System.out.printf("Injected objectives: WP1=(%.0f,%.0f) WP2=(%.0f,%.0f)%n",
                    objectives.x1(), objectives.y1(), objectives.x2(), objectives.y2());
        }

        // Start paused so all viewers can register before ticks flow
        sim.setPaused(true);
        currentThread = Thread.ofPlatform().daemon().name("sim-loop").start(() -> {
            if (headings != null) {
                sim.run(world, controllers, vehicleConfigs, headings, fanOut);
            } else {
                sim.run(world, controllers, vehicleConfigs, fanOut);
            }
        });
    }

    void start(GeneratedWorld world, List<SubmarineController> controllers,
               List<VehicleConfig> vehicleConfigs) {
        start(world, controllers, vehicleConfigs, null);
    }

    /**
     * Unpauses the simulation. Call after all viewers have registered
     * and are ready to receive tick events.
     */
    void play() {
        var sim = currentLoop;
        if (sim != null) sim.setPaused(false);
    }

    /**
     * Stops the current simulation and waits for the thread to finish.
     */
    void stop() {
        var sim = currentLoop;
        if (sim != null) {
            sim.stop();
        }
        var t = currentThread;
        if (t != null) {
            t.interrupt();
            try { t.join(2000); } catch (InterruptedException ignored) {}
        }
        currentLoop = null;
        currentThread = null;
    }

    /**
     * Manually fan out a tick to all registered listeners. Used by
     * CompetitionRunner which manages its own SimulationLoop but still
     * needs to update the viewers.
     */
    /**
     * Notifies all viewers that the world has changed. Called when
     * switching seeds or competition phases.
     */
    void setWorld(GeneratedWorld world) {
        for (var l : listeners) {
            if (l instanceof MapRenderer mr) mr.setWorld(world);
            else if (l instanceof SubmarineScene3D s3d) s3d.setWorld(world);
        }
    }

    /** Set up sim state supplier on all viewers that support it. */
    void configureStateSupplier(java.util.function.Supplier<SimulationLoop.State> supplier) {
        for (var l : listeners) {
            if (l instanceof SubmarineScene3D s3d) s3d.setSimStateSupplier(supplier);
        }
    }

    void fanOutTick(long tick, List<SubmarineSnapshot> submarines) {
        for (var l : listeners) {
            l.onTick(tick, submarines, java.util.List.of());
        }
    }

    void fanOutMatchEnd() {
        for (var l : listeners) {
            l.onMatchEnd();
        }
    }

    /** Returns the current SimulationLoop, or null if not running. */
    SimulationLoop currentLoop() {
        return currentLoop;
    }

    /** Returns true if a simulation is actively running. */
    boolean isRunning() {
        return currentThread != null && currentThread.isAlive();
    }
}
