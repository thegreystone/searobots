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
package se.hirt.searobots.engine.replay;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.*;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies {@link ReplayPlayer} drives a listener through a recorded match under the same control surface as a live
 * {@link SimulationLoop}: it emits every recorded frame in order at full fidelity, and honours pause / single-step /
 * stop / speed. This is what lets the viewer replay a battle with the same controls it uses to run one live.
 */
class ReplayPlayerTest {

	/** Thread-safe capture: the player runs on its own thread while the test asserts. */
	private static final class CaptureListener implements SimulationListener {
		final List<Long> ticks = Collections.synchronizedList(new ArrayList<>());
		volatile boolean ended;

		@Override
		public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
			ticks.add(tick);
		}

		@Override
		public void onMatchEnd() {
			ended = true;
		}
	}

	@Test
	@Timeout(value = 60, unit = TimeUnit.SECONDS)
	void playsBackEveryRecordedFrameInOrder(@TempDir Path dir) throws Exception {
		// Record a real short match, then play it back and confirm the player emits the same
		// frame sequence ReplayReader.readAll() reconstructs.
		MatchConfig config = shortMatch(42L, 400);
		var world = new WorldGenerator().generate(config);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9); // no real-time pacing while recording

		Path file = dir.resolve("match.srl");
		try (var writer = new ReplayWriter(config, world.spawnPoints(), file)) {
			sim.run(world, List.of(new DefaultAttackSub(), new DefaultAttackSub()),
					List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), writer);
		}

		var reader = new ReplayReader(file);
		List<ReplayFrame> frames = reader.readAll();
		assertFalse(frames.isEmpty(), "recorded match should have frames");

		var captured = new CaptureListener();
		var player = new ReplayPlayer(frames, reader.header().tickRateHz(), captured);
		player.setSpeedMultiplier(1e9); // as fast as possible
		var thread = new Thread(player::run, "test-replay");
		thread.start();
		thread.join(TimeUnit.SECONDS.toMillis(30));
		assertFalse(thread.isAlive(), "player should finish");

		assertTrue(captured.ended, "onMatchEnd should fire");
		assertEquals(frames.size(), captured.ticks.size(), "played frame count");
		assertEquals(frames.size(), player.frameCount(), "frameCount");
		for (int i = 0; i < frames.size(); i++) {
			assertEquals(frames.get(i).tick(), captured.ticks.get(i), "tick at frame " + i);
		}
		assertEquals(SimulationLoop.State.STOPPED, player.getState());
	}

	@Test
	@Timeout(value = 10, unit = TimeUnit.SECONDS)
	void honoursPauseSingleStepAndStop() throws Exception {
		List<ReplayFrame> frames = new ArrayList<>();
		for (int i = 0; i < 6; i++) {
			frames.add(new ReplayFrame(i, List.of(sub(i)), List.of()));
		}
		var captured = new CaptureListener();
		var player = new ReplayPlayer(frames, 50, captured);
		player.setPaused(true); // start paused, like SimulationManager does

		var thread = new Thread(player::run, "test-replay-step");
		thread.start();

		// Paused: nothing should be emitted.
		Thread.sleep(150);
		assertEquals(0, captured.ticks.size(), "no frames while paused");
		assertEquals(SimulationLoop.State.PAUSED, player.getState());

		// Each stepOnce releases exactly one frame.
		player.stepOnce();
		awaitCount(captured.ticks::size, 1);
		player.stepOnce();
		awaitCount(captured.ticks::size, 2);
		assertTrue(player.isPaused(), "still paused between steps");

		// Unpause at max speed: it runs to the end and fires onMatchEnd.
		player.setSpeedMultiplier(1e9);
		player.setPaused(false);
		thread.join(TimeUnit.SECONDS.toMillis(5));
		assertFalse(thread.isAlive(), "player should finish after unpausing");
		assertEquals(frames.size(), captured.ticks.size(), "all frames emitted");
		assertTrue(captured.ended);
	}

	@Test
	@Timeout(value = 10, unit = TimeUnit.SECONDS)
	void stopEndsPlaybackEarly() throws Exception {
		List<ReplayFrame> frames = new ArrayList<>();
		for (int i = 0; i < 1000; i++) {
			frames.add(new ReplayFrame(i, List.of(sub(i)), List.of()));
		}
		var captured = new CaptureListener();
		var player = new ReplayPlayer(frames, 50, captured);
		player.setPaused(true);
		var thread = new Thread(player::run, "test-replay-stop");
		thread.start();
		Thread.sleep(100);

		player.stop();
		thread.join(TimeUnit.SECONDS.toMillis(5));
		assertFalse(thread.isAlive(), "player should stop");
		assertTrue(captured.ended, "onMatchEnd still fires on stop");
		assertTrue(captured.ticks.size() < frames.size(), "stopped before the end");
		assertEquals(SimulationLoop.State.STOPPED, player.getState());
	}

	// --- helpers ---

	private static void awaitCount(java.util.function.IntSupplier count, int expected) throws InterruptedException {
		long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(3);
		while (count.getAsInt() < expected && System.nanoTime() < deadline) {
			Thread.sleep(5);
		}
		assertEquals(expected, count.getAsInt(), "frame count did not reach " + expected);
	}

	private static SubmarineSnapshot sub(int id) {
		return new SubmarineSnapshot(id, "Sub" + id, Pose.at(Vec3.ZERO), Velocity.ZERO, 0, new Color(60, 220, 120),
				false, 1000, 0, 90, 0, 0, 0, "PATROL", false, 8, List.of(), List.of(), List.of(), null);
	}

	private static MatchConfig shortMatch(long seed, int durationTicks) {
		MatchConfig b = MatchConfig.withDefaults(seed);
		return new MatchConfig(b.worldSeed(), b.tickRateHz(), durationTicks, b.submarineCount(), b.torpedoCount(),
				b.startingHp(), b.blastRadius(), b.minFuseRadius(), b.maxFuseRadius(), b.ratedDepth(), b.crushDepth(),
				b.battleArea(), b.terrainMarginMeters(), b.gridCellMeters(), b.minSeaFloorZ(), b.maxSeaFloorZ(),
				b.maxSubSpeed(), b.startTime());
	}
}
