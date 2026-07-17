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
package se.hirt.searobots.viewer;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.replay.*;

import java.io.IOException;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Manages the lifecycle of SimulationLoop instances and fans out tick events to all registered listeners (2D panel, 3D
 * scene, recorders, competition scorers, etc.). Any component that needs simulation data registers once; it doesn't
 * matter whether the sim is free patrol, competition, or a test.
 */
final class SimulationManager {

	private final CopyOnWriteArrayList<SimulationListener> listeners = new CopyOnWriteArrayList<>();
	private volatile SimClock currentLoop;
	private volatile Thread currentThread;

	/** Directory where live matches are auto-recorded as replayable {@code .srl} files. */
	private static final Path REPLAY_DIR = Path.of("replays");
	private static final DateTimeFormatter STAMP = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");

	// Pause-on-event flags
	volatile boolean pauseOnDeath;
	volatile boolean pauseOnTorpedoSolution;
	volatile boolean pauseOnTorpedoLaunch;
	volatile boolean injectObjectives;
	private boolean torpedoSolutionTriggered;
	private final java.util.Set<Integer> deadEntities = java.util.Collections.synchronizedSet(
			new java.util.HashSet<>());

	// ── Listener management ─────────────────────────────────────────

	void addListener(SimulationListener listener) {
		listeners.addIfAbsent(listener);
	}

	void removeListener(SimulationListener listener) {
		listeners.remove(listener);
	}

	// ── Simulation lifecycle ────────────────────────────────────────

	/**
	 * Starts a new simulation. Stops any running simulation first and waits for its thread to finish.
	 */
	void start(
			GeneratedWorld world, List<SubmarineController> controllers, List<VehicleConfig> vehicleConfigs,
			List<Double> headings) {
		stop();

		deadEntities.clear();
		torpedoSolutionTriggered = false;

		var sim = new SimulationLoop();
		currentLoop = sim;

		// The event-pausing fan-out reads currentLoop at tick time, so it pauses whichever
		// clock is active (this live loop, or a ReplayPlayer). Auto-record the live match so
		// it can be replayed later through the same controls.
		var fanOut = SimulationListeners.composite(buildFanOut(), recorderFor(world));

		// Inject competition objectives if enabled
		if (injectObjectives) {
			var objectives = SubmarineCompetition.generateObjectives(world.config().worldSeed(), world);
			var terrain = world.terrain();
			double depth1 = Math.max(-300, terrain.elevationAt(objectives.x1(), objectives.y1()) + 90);
			double depth2 = Math.max(-300, terrain.elevationAt(objectives.x2(), objectives.y2()) + 90);
			var objList = java.util.List.of(
					new StrategicWaypoint(objectives.x1(), objectives.y1(), depth1, Purpose.PATROL, NoisePolicy.NORMAL,
							MovementPattern.DIRECT, 300, -1),
					new StrategicWaypoint(objectives.x2(), objectives.y2(), depth2, Purpose.PATROL, NoisePolicy.NORMAL,
							MovementPattern.DIRECT, 300, -1));
			for (var ctrl : controllers) {
				ctrl.setObjectives(objList);
			}
			System.out.printf("Injected objectives: WP1=(%.0f,%.0f) WP2=(%.0f,%.0f)%n", objectives.x1(),
					objectives.y1(), objectives.x2(), objectives.y2());
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

	void start(GeneratedWorld world, List<SubmarineController> controllers, List<VehicleConfig> vehicleConfigs) {
		start(world, controllers, vehicleConfigs, null);
	}

	/**
	 * Plays a recorded {@code .srl} match back through the same viewers and controls as a live match. The world is
	 * regenerated from the header seed (terrain is seed-derived), and a {@link ReplayPlayer} drives the viewers as a
	 * {@link SimClock}, so pause / step / speed / fast-forward-to-event all behave exactly as in a live simulation.
	 * <p>
	 * Starts paused; call {@link #play()} once the viewers have registered, exactly like {@link #start}.
	 *
	 * @return the parsed replay header (seed, tick rate, sub defs), or {@code null} if the file could not be read
	 */
	ReplayHeader startReplay(Path srl) {
		stop();
		deadEntities.clear();
		torpedoSolutionTriggered = false;
		try {
			var reader = new ReplayReader(srl);
			ReplayHeader header = reader.header();
			// Terrain and spawns are seed-derived, so the recorded world reconstructs from the seed.
			var world = new WorldGenerator().generate(MatchConfig.withDefaults(header.seed()));
			setWorld(world);

			List<ReplayFrame> frames = reader.readAll();
			var player = new ReplayPlayer(frames, header.tickRateHz(), buildFanOut());
			currentLoop = player;
			player.setPaused(true); // register viewers before frames flow

			currentThread = Thread.ofPlatform().daemon().name("replay-player").start(player::run);
			System.out.printf("Replaying %s: seed=%s, %d frames%n", srl, Long.toHexString(header.seed()),
					frames.size());
			return header;
		} catch (IOException e) {
			System.err.println("Failed to load replay " + srl + ": " + e.getMessage());
			return null;
		}
	}

	/**
	 * Builds the fan-out listener shared by live and replay runs: it forwards every tick to the registered viewers and
	 * pauses the <em>currently active</em> clock ({@link #currentLoop}) on the enabled pause-on-event triggers. Reading
	 * {@code currentLoop} at tick time (rather than capturing a specific loop) is what lets the same logic pause a
	 * {@link ReplayPlayer} as easily as a live {@link SimulationLoop}.
	 */
	private SimulationListener buildFanOut() {
		return new SimulationListener() {
			@Override
			public void onTick(
					long tick, List<SubmarineSnapshot> submarines,
					List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
				for (var l : listeners) {
					l.onTick(tick, submarines, torpedoes);
				}

				var clock = currentLoop;
				if (clock == null)
					return;

				// Pause-on-death
				if (pauseOnDeath) {
					for (var sub : submarines) {
						if (sub.hp() <= 0 && !deadEntities.contains(sub.id())) {
							deadEntities.add(sub.id());
							clock.setPaused(true);
						}
					}
				}

				// Pause on first torpedo solution. Firing solutions are captured in the replay
				// format (v2), so this triggers during replay too, not just live.
				if (pauseOnTorpedoSolution && !torpedoSolutionTriggered) {
					for (var sub : submarines) {
						if (sub.firingSolution() != null) {
							torpedoSolutionTriggered = true;
							clock.setPaused(true);
							var sol = sub.firingSolution();
							System.out.printf(
									"TORPEDO SOLUTION at tick %d: %s target=[%.0f,%.0f] hdg=%.0f spd=%.1f q=%.2f%n",
									tick, sub.name(), sol.targetX(), sol.targetY(), Math.toDegrees(sol.targetHeading()),
									sol.targetSpeed(), sol.quality());
							break;
						}
					}
				}

				// Pause on torpedo launch
				if (pauseOnTorpedoLaunch && torpedoes != null && !torpedoes.isEmpty()) {
					for (var t : torpedoes) {
						if (t.fuelRemaining() > 119.0) { // just launched (120s max fuel)
							clock.setPaused(true);
							System.out.printf("TORPEDO LAUNCHED at tick %d: torpedo %d from sub %d%n", tick, t.id(),
									t.ownerId());
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
	}

	/**
	 * Returns a recording listener that writes the live match to a timestamped {@code .srl} under {@link #REPLAY_DIR},
	 * so it can be replayed later. Returns a no-op listener (never null) if the file cannot be opened, so recording
	 * failures never take down a running match.
	 */
	SimulationListener recorderFor(GeneratedWorld world) {
		try {
			String name = STAMP.format(LocalDateTime.now()) + "-" + Long.toHexString(
					world.config().worldSeed()) + ".srl";
			Path file = REPLAY_DIR.resolve(name);
			var writer = new ReplayWriter(world.config(), world.spawnPoints(), file);
			System.out.println("Recording match to " + file.toAbsolutePath());
			return writer;
		} catch (IOException e) {
			System.err.println("Could not start match recording: " + e.getMessage());
			return NO_OP_LISTENER;
		}
	}

	private static final SimulationListener NO_OP_LISTENER = new SimulationListener() {
		@Override
		public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
		}

		@Override
		public void onMatchEnd() {
		}
	};

	/**
	 * Unpauses the simulation. Call after all viewers have registered and are ready to receive tick events.
	 */
	void play() {
		var sim = currentLoop;
		if (sim != null)
			sim.setPaused(false);
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
			try {
				t.join(2000);
			} catch (InterruptedException ignored) {
			}
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
	 * Notifies all viewers that the world has changed. Called when switching seeds or competition phases.
	 */
	void setWorld(GeneratedWorld world) {
		for (var l : listeners) {
			if (l instanceof MapRenderer mr)
				mr.setWorld(world);
			else if (l instanceof SubmarineScene3D s3d)
				s3d.setWorld(world);
		}
	}

	/**
	 * Set up sim state supplier on all viewers that support it.
	 */
	void configureStateSupplier(java.util.function.Supplier<SimulationLoop.State> supplier) {
		for (var l : listeners) {
			if (l instanceof SubmarineScene3D s3d)
				s3d.setSimStateSupplier(supplier);
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

	/**
	 * Returns the current clock (live {@link SimulationLoop} or {@link ReplayPlayer}), or null if not running.
	 */
	SimClock currentLoop() {
		return currentLoop;
	}

	/**
	 * Returns true if a simulation is actively running.
	 */
	boolean isRunning() {
		return currentThread != null && currentThread.isAlive();
	}
}
