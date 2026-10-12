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
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.junit.jupiter.api.Assertions.assertEquals;

@Timeout(value = 5, unit = TimeUnit.SECONDS)
class SimulationPacingTest {

	@Test
	void fiveMillisecondsOfWorkLeavesFifteenMillisecondsAtFiftyHz() {
		var fixture = new ClockPacingFixture();
		runThreeTicks(fixture, 1, 0, 0, 5, 5, 5);
		assertEquals(List.of(ms(15), ms(15)), fixture.waitDurationsNanos());
	}

	@Test
	void firstTickIncludesControllerInitializationAndListenerWork() {
		var fixture = new ClockPacingFixture();
		runThreeTicks(fixture, 1, 4, 3, 2, 2, 2);
		assertEquals(List.of(ms(11), ms(15)), fixture.waitDurationsNanos());
	}

	@Test
	void overrunSkipsSleepAndTheNextTickGetsItsOwnBudget() {
		var fixture = new ClockPacingFixture();
		runThreeTicks(fixture, 1, 0, 0, 25, 5, 5);
		assertEquals(List.of(ms(15)), fixture.waitDurationsNanos());
	}

	@Test
	void speedMultiplierScalesTheBudgetIncludingWork() {
		var fixture = new ClockPacingFixture();
		runThreeTicks(fixture, 2, 0, 0, 3, 3, 3);
		assertEquals(List.of(ms(7), ms(7)), fixture.waitDurationsNanos());
	}

	private static void runThreeTicks(
		ClockPacingFixture fixture, double speed, long initializationMillis, long controllerMillis,
		long ... listenerMillis) {
		var simulation = new SimulationLoop(fixture.control());
		simulation.setSpeedMultiplier(speed);
		var ticks = new ArrayList<Long>();
		var controller = new SubmarineController() {
			@Override
			public void onMatchStart(MatchContext context) {
				fixture.advanceMillis(initializationMillis);
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				fixture.advanceMillis(controllerMillis);
				output.setBallast(0.5);
			}
		};
		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
				ticks.add(tick);
				fixture.advanceMillis(listenerMillis[(int) tick]);
			}

			@Override
			public void onMatchEnd() {
			}
		};
		var config = MatchConfig.withDefaults(0).withMatchDurationTicks(3);
		var terrain = new TerrainMap(new double[] {-1000, -1000, -1000, -1000}, 2, 2, -3000, -3000, 6000);
		var world = new GeneratedWorld(config, terrain, List.of(), new CurrentField(List.of()),
				List.of(new Vec3(0, 0, -100)));
		simulation.run(world, List.of(controller), List.of(VehicleConfig.submarine()), List.of(0.0), listener);
		assertEquals(List.of(0L, 1L, 2L), ticks);
		assertEquals(SimulationLoop.State.STOPPED, simulation.getState());
	}

	private static long ms(long milliseconds) {
		return TimeUnit.MILLISECONDS.toNanos(milliseconds);
	}
}
