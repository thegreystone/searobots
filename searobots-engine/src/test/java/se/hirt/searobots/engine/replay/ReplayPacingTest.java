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
import se.hirt.searobots.engine.ClockPacingFixture;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.assertEquals;

@Timeout(value = 5, unit = TimeUnit.SECONDS)
class ReplayPacingTest {

	@Test
	void fiveMillisecondsOfEmissionLeavesFifteenMillisecondsAtFiftyHz() {
		var fixture = new ClockPacingFixture();
		runFrames(fixture, 1, 5, 5, 5);
		assertEquals(List.of(ms(15), ms(15)), fixture.waitDurationsNanos());
	}

	@Test
	void overrunSkipsSleepAndTheNextFrameGetsItsOwnBudget() {
		var fixture = new ClockPacingFixture();
		runFrames(fixture, 1, 25, 5, 5);
		assertEquals(List.of(ms(15)), fixture.waitDurationsNanos());
	}

	@Test
	void speedMultiplierScalesTheBudgetIncludingEmission() {
		var fixture = new ClockPacingFixture();
		runFrames(fixture, 2, 3, 3, 3);
		assertEquals(List.of(ms(7), ms(7)), fixture.waitDurationsNanos());
	}

	@Test
	void finalFrameDoesNotWaitForAnotherFrame() {
		var fixture = new ClockPacingFixture();
		runFrames(fixture, 1, 5);
		assertEquals(List.of(), fixture.waitDurationsNanos());
	}

	private static void runFrames(ClockPacingFixture fixture, double speed, long ... emissionMillis) {
		var ticks = new ArrayList<Long>();
		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
				ticks.add(tick);
				fixture.advanceMillis(emissionMillis[(int) tick]);
			}

			@Override
			public void onMatchEnd() {
			}
		};
		var frames = IntStream.range(0, emissionMillis.length)
				.mapToObj(tick -> new ReplayFrame(tick, List.of(), List.of())).toList();
		var player = new ReplayPlayer(frames, 50, listener, fixture.control());
		player.setSpeedMultiplier(speed);
		player.run();
		assertEquals(IntStream.range(0, emissionMillis.length).mapToObj(Long::valueOf).toList(), ticks);
		assertEquals(SimulationLoop.State.STOPPED, player.getState());
	}

	private static long ms(long milliseconds) {
		return TimeUnit.MILLISECONDS.toNanos(milliseconds);
	}
}
