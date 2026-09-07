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

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Validates the decoder against the real match committed at {@code replays/headless-d.srl} (seed 13, Codex vs Claude,
 * 14,221 frames, format v2, ends in a torpedo kill).
 * <p>
 * {@link ReplayRoundTripTest} writes and reads with the <em>same</em> build, so it can never notice the codec drifting
 * away from files that already exist on disk. This test pins decoding to a fixed on-disk artifact: if a change breaks
 * reading of previously recorded replays, it fails here first. The expected values below were extracted independently
 * from the raw tab-separated file (awk over the {@code T}/{@code s}/{@code f}/{@code p} lines), not via the Java
 * decoder, so decoder and file cannot be wrong in the same way.
 * <p>
 * If the sample file is ever removed from the repository, the tests are skipped rather than failed.
 */
class SampleReplayTest {

	/** Repo-root relative location; resolved from the module dir (surefire) or the repo root (IDE). */
	private static Path sample;

	@BeforeAll
	static void locateSample() {
		Path fromModule = Path.of("..", "replays", "headless-d.srl");
		Path fromRoot = Path.of("replays", "headless-d.srl");
		sample = Files.exists(fromModule) ? fromModule : fromRoot;
		Assumptions.assumeTrue(Files.exists(sample), "committed sample replay not present");
	}

	@Test
	void headerParses() throws Exception {
		var h = new ReplayReader(sample).header();
		assertEquals(2, h.formatVersion());
		assertEquals(13L, h.seed());
		assertEquals(50, h.tickRateHz());
		assertEquals(30_000, h.durationTicks());
		assertEquals(1000, h.startingHp());
		assertEquals(2, h.submarines().size());
		assertEquals("Codex Sub", h.submarines().get(0).name());
		assertEquals("Claude Sub", h.submarines().get(1).name());
	}

	@Test
	@Timeout(value = 60, unit = TimeUnit.SECONDS)
	void fullMatchDecodesToKnownGroundTruth() throws Exception {
		long[] firstSolutionTick = {-1, -1};
		Map<Integer, Integer> finalHp = new HashMap<>();
		Set<Integer> torpedoIds = new HashSet<>();
		Set<Integer> detonatedIds = new HashSet<>();
		long[] lastTick = {-1};

		long ticks = new ReplayReader(sample).replay(new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				lastTick[0] = tick;
				for (var s : subs) {
					finalHp.put(s.id(), s.hp());
					if (s.firingSolution() != null && firstSolutionTick[s.id()] < 0) {
						firstSolutionTick[s.id()] = tick;
					}
				}
				for (var t : torps) {
					torpedoIds.add(t.id());
					if (t.detonated()) {
						detonatedIds.add(t.id());
					}
				}
			}

			@Override
			public void onMatchEnd() {
			}
		});

		// Ground truth extracted from the raw file with awk, independently of the decoder.
		assertEquals(14_221, ticks, "frame count");
		assertEquals(14_220, lastTick[0], "last tick");
		assertEquals(9_521, firstSolutionTick[1], "Claude's first firing solution");
		assertEquals(10_998, firstSolutionTick[0], "Codex's first firing solution");
		assertEquals(11, torpedoIds.size(), "torpedoes launched");
		assertEquals(Set.of(1000, 1001, 1002), detonatedIds, "detonated torpedoes");
		assertEquals(1000, finalHp.get(0).intValue(), "Codex ends untouched");
		assertEquals(0, finalHp.get(1).intValue(), "Claude is killed");
	}

	@Test
	@Timeout(value = 60, unit = TimeUnit.SECONDS)
	void playerFastForwardsAndPausesOnFirstFiringSolution() throws Exception {
		// The viewer flow: [F] pause-on-solution enabled, [.] fast-forward. The fan-out pauses
		// the player when a decoded firing solution first appears; playback must halt on that
		// exact frame instead of racing to the end.
		var reader = new ReplayReader(sample);
		List<ReplayFrame> frames = reader.readAll();
		assertEquals(14_221, frames.size(), "readAll agrees with streaming replay");

		final ReplayPlayer[] ref = new ReplayPlayer[1];
		var pausingFanOut = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				for (var s : subs) {
					if (s.firingSolution() != null) {
						ref[0].setPaused(true);
						return;
					}
				}
			}

			@Override
			public void onMatchEnd() {
			}
		};
		var player = new ReplayPlayer(frames, reader.header().tickRateHz(), pausingFanOut);
		ref[0] = player;
		player.setPaused(true);
		player.setSpeedMultiplier(1e9);
		var thread = new Thread(player::run, "sample-replay-player");
		thread.start();
		player.setPaused(false);

		long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(30);
		while (!player.isPaused() && thread.isAlive() && System.nanoTime() < deadline) {
			Thread.sleep(10);
		}
		Thread.sleep(200); // let the player settle into its pause loop

		assertTrue(player.isPaused(), "player should be paused by the fan-out");
		assertTrue(thread.isAlive(), "playback should be held, not ended");
		assertEquals(9_521, player.currentTick(), "paused exactly on the first-solution frame");
		assertEquals(SimulationLoop.State.PAUSED, player.getState());

		player.stop();
		thread.join(TimeUnit.SECONDS.toMillis(5));
		assertFalse(thread.isAlive(), "player should stop cleanly");
	}
}
