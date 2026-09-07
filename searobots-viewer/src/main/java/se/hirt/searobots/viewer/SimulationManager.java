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

	/** File the current live match is being recorded to, or null. Excluded from "load latest replay". */
	private volatile Path currentRecordingFile;

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

		var sim = new SimulationLoop();
		currentLoop = sim;

		// The fan-out is bound to this loop; if a newer run replaces it, stale ticks from a
		// slow-to-die thread are dropped instead of leaking into the new run. Auto-record the
		// live match so it can be replayed later through the same controls.
		var fan = new FanOut();
		fan.owner = sim;
		var fanOut = SimulationListeners.composite(fan, recorderFor(world));

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

	/** What {@link #startReplay} hands back: the parsed header and the world rebuilt from it. */
	record ReplayStart(ReplayHeader header, GeneratedWorld world) {
	}

	/**
	 * Plays a recorded {@code .srl} match back through the same viewers and controls as a live match. The world is
	 * regenerated from the recorded match config (terrain is seed-derived; hand-built worlds are not reproducible), and
	 * a {@link ReplayPlayer} drives the viewers as a {@link SimClock}, so pause / step / speed / fast-forward-to-event
	 * all behave exactly as in a live simulation.
	 * <p>
	 * Starts paused; call {@link #play()} once the viewers have registered, exactly like {@link #start}. This does not
	 * push the world to the viewers; the caller applies {@link ReplayStart#world()} on its render thread.
	 *
	 * @return the parsed header and rebuilt world, or {@code null} if the file could not be read or is corrupt
	 */
	ReplayStart startReplay(Path srl) {
		stop();
		try {
			var reader = new ReplayReader(srl);
			ReplayHeader header = reader.header();
			var world = new WorldGenerator().generate(header.toMatchConfig());

			List<ReplayFrame> frames = reader.readAll();
			var fan = new FanOut();
			var player = new ReplayPlayer(frames, header.tickRateHz(), fan);
			fan.owner = player;
			currentLoop = player;
			player.setPaused(true); // register viewers before frames flow

			currentThread = Thread.ofPlatform().daemon().name("replay-player").start(player::run);
			System.out.printf("Replaying %s: seed=%s, %d frames%n", srl, Long.toHexString(header.seed()),
					frames.size());
			return new ReplayStart(header, world);
		} catch (IOException | RuntimeException e) {
			// RuntimeException covers parse failures from truncated or corrupt files.
			System.err.println("Failed to load replay " + srl + ": " + e);
			return null;
		}
	}

	/**
	 * The fan-out listener shared by live and replay runs: forwards every tick to the registered viewers and pauses the
	 * clock it was built for on the enabled pause-on-event triggers. Events are edge-detected per entity, so
	 * fast-forward-to-event stops at each <em>new</em> death, firing solution, or torpedo launch instead of latching on
	 * the first or re-firing on the same one. Each run gets its own instance: per-run state needs no reset, and a stale
	 * tick from a previous run's slow-to-die thread is dropped instead of pausing or polluting the new run.
	 */
	private final class FanOut implements SimulationListener {
		/** The clock this fan-out paces. Assigned before the run thread starts. */
		SimClock owner;

		// Per-run edge-detection state, touched only on the run's own thread.
		private final java.util.Set<Integer> deadEntities = new java.util.HashSet<>();
		private final java.util.Set<Integer> subsWithSolution = new java.util.HashSet<>();
		private final java.util.Set<Integer> seenTorpedoes = new java.util.HashSet<>();
		private boolean seeded; // first tick baselines in-flight torpedoes without pausing

		@Override
		public void onTick(
				long tick, List<SubmarineSnapshot> submarines,
				List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
			var clock = owner;
			if (clock != currentLoop) {
				return; // a newer run has replaced this one
			}
			for (var l : listeners) {
				l.onTick(tick, submarines, torpedoes);
			}

			// Pause-on-death (once per entity)
			for (var sub : submarines) {
				if (sub.hp() <= 0 && deadEntities.add(sub.id()) && pauseOnDeath) {
					clock.setPaused(true);
				}
			}

			// Pause when a sub acquires a firing solution (edge per sub, re-arming when the
			// solution is lost). Solutions are captured in the replay format (v2), so this
			// triggers during replay too, not just live.
			for (var sub : submarines) {
				var sol = sub.firingSolution();
				if (sol == null) {
					subsWithSolution.remove(sub.id());
				} else if (subsWithSolution.add(sub.id())) {
					if (pauseOnTorpedoSolution) {
						clock.setPaused(true);
						System.out.printf(
								"TORPEDO SOLUTION at tick %d: %s target=[%.0f,%.0f] hdg=%.0f spd=%.1f q=%.2f%n",
								tick, sub.name(), sol.targetX(), sol.targetY(), Math.toDegrees(sol.targetHeading()),
								sol.targetSpeed(), sol.quality());
					}
				}
			}

			// Pause on torpedo launch: a torpedo id never seen before is a launch. The first
			// tick only baselines (a playback may begin with torpedoes already in flight).
			if (torpedoes != null) {
				for (var t : torpedoes) {
					if (seenTorpedoes.add(t.id()) && seeded && pauseOnTorpedoLaunch) {
						clock.setPaused(true);
						System.out.printf("TORPEDO LAUNCHED at tick %d: torpedo %d from sub %d%n", tick, t.id(),
								t.ownerId());
					}
				}
			}
			seeded = true;
		}

		@Override
		public void onMatchEnd() {
			if (owner != currentLoop) {
				return; // stale end-of-match from a replaced run
			}
			for (var l : listeners) {
				l.onMatchEnd();
			}
		}
	}

	/**
	 * Returns a recording listener that writes the live match to a timestamped {@code .srl} under {@link #REPLAY_DIR},
	 * so it can be replayed later. The name is uniquified so a same-seed match started within the same second (re-run,
	 * consecutive competition phases) never truncates an earlier recording. Returns a no-op listener (never null) if the
	 * file cannot be opened, and disables itself on a mid-match write failure, so recording failures never take down a
	 * running match.
	 */
	SimulationListener recorderFor(GeneratedWorld world) {
		try {
			String base = STAMP.format(LocalDateTime.now()) + "-" + Long.toHexString(world.config().worldSeed());
			Path file = REPLAY_DIR.resolve(base + ".srl");
			for (int n = 2; java.nio.file.Files.exists(file); n++) {
				file = REPLAY_DIR.resolve(base + "-" + n + ".srl");
			}
			var writer = new ReplayWriter(world.config(), world.spawnPoints(), file);
			System.out.println("Recording match to " + file.toAbsolutePath());
			currentRecordingFile = file;
			return new GuardedRecorder(writer);
		} catch (IOException e) {
			System.err.println("Could not start match recording: " + e.getMessage());
			return NO_OP_LISTENER;
		}
	}

	/** The file the current live match is being recorded to, or null if not recording. */
	Path currentRecordingFile() {
		return currentRecordingFile;
	}

	/**
	 * Shields the sim thread from the {@link ReplayWriter}: a write failure ({@code UncheckedIOException} on disk full,
	 * file lock, ...) disables recording for the rest of the match instead of propagating and killing the loop.
	 */
	private final class GuardedRecorder implements SimulationListener {
		private ReplayWriter writer;

		GuardedRecorder(ReplayWriter writer) {
			this.writer = writer;
		}

		@Override
		public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
			var w = writer;
			if (w == null)
				return;
			try {
				w.onTick(tick, submarines, torpedoes);
			} catch (RuntimeException e) {
				abort(w, e);
			}
		}

		@Override
		public void onMatchEnd() {
			var w = writer;
			if (w == null)
				return;
			writer = null;
			try {
				w.onMatchEnd();
			} catch (RuntimeException e) {
				System.err.println("Failed to finalize match recording: " + e);
				w.close();
			}
			clearCurrentRecording(w);
		}

		private void abort(ReplayWriter w, RuntimeException e) {
			writer = null;
			System.err.println("Match recording failed, recording disabled for this match: " + e);
			w.close();
			clearCurrentRecording(w);
		}

		private void clearCurrentRecording(ReplayWriter w) {
			// Only clear if a newer match hasn't already started its own recording.
			if (w.file().equals(currentRecordingFile)) {
				currentRecordingFile = null;
			}
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
