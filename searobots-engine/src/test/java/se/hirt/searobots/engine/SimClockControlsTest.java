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

import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.replay.ReplayFrame;
import se.hirt.searobots.engine.replay.ReplayPlayer;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import java.util.function.LongConsumer;
import java.util.stream.IntStream;

import static org.junit.jupiter.api.Assertions.*;

@Timeout(value = 10, unit = TimeUnit.SECONDS)
class SimClockControlsTest {

	enum Mode {
		REPLAY, LIVE
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void pausedInterruptStopsWithoutEmittingFrame(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setPaused(true);
			fixture.start();
			fixture.awaitPaused();

			fixture.worker.interrupt();
			fixture.assertFinished();
			assertTrue(fixture.interruptedOnExit.get(), "worker preserves its interrupt flag");
			assertEquals(List.of(), fixture.ticks, "interrupting a pause must not release a frame");
			assertEquals(0, fixture.controllerStarts.get(), "paused live controllers must not start");
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void pacingInterruptStopsWithoutEmittingAnotherFrame(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			fixture.start();
			fixture.awaitPacing();

			fixture.worker.interrupt();
			fixture.assertFinished();
			assertTrue(fixture.interruptedOnExit.get(), "worker preserves its interrupt flag");
			assertEquals(List.of(0L), fixture.ticks);
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void stopWakesPausedWorker(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setPaused(true);
			fixture.start();
			fixture.awaitPaused();

			fixture.clock.stop();
			fixture.assertFinished();
			assertEquals(List.of(), fixture.ticks);
			assertFalse(fixture.interruptedOnExit.get(), "stop does not interrupt the worker");
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void stopWakesExtremelySlowPacingWait(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			fixture.start();
			fixture.awaitPacing();

			fixture.clock.stop();
			fixture.assertFinished();
			assertEquals(List.of(0L), fixture.ticks);
			assertFalse(fixture.interruptedOnExit.get(), "stop does not interrupt the worker");
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void speedChangeWakesExtremelySlowPacingWait(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			fixture.start();
			fixture.awaitPacing();

			fixture.clock.setSpeedMultiplier(Double.MAX_VALUE);
			fixture.assertFinished();
			assertEquals(100, fixture.ticks.size());
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void pauseWakesPacingAndSingleStepsStayPaused(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			fixture.start();
			fixture.awaitPacing();

			fixture.clock.setPaused(true);
			fixture.awaitPaused();
			assertEquals(List.of(0L), fixture.ticks);
			fixture.clock.stepOnce();
			fixture.awaitCount(2);
			fixture.awaitPaused();
			assertEquals(List.of(0L, 1L), fixture.ticks);
			assertTrue(fixture.clock.isPaused());
			fixture.clock.stepOnce();
			fixture.awaitCount(3);
			fixture.awaitPaused();
			assertEquals(List.of(0L, 1L, 2L), fixture.ticks);
			fixture.clock.stop();
			fixture.assertFinished();
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void repeatedSingleStepRequestsQueueOneFrameEach(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setPaused(true);
			fixture.clock.stepOnce();
			fixture.clock.stepOnce();
			fixture.clock.stepOnce();
			fixture.start();

			fixture.awaitCount(3);
			fixture.awaitPaused();
			assertEquals(List.of(0L, 1L, 2L), fixture.ticks);
			fixture.clock.stop();
			fixture.assertFinished();
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void singleStepRequestWhileRunningDoesNotReleaseLaterPause(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			fixture.start();
			fixture.awaitPacing();

			fixture.clock.stepOnce();
			fixture.clock.setPaused(true);
			fixture.awaitPaused();
			assertEquals(List.of(0L), fixture.ticks);
			fixture.clock.stop();
			fixture.assertFinished();
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void listenerCanStopWithoutEnteringPacingWait(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			fixture.tickAction = tick -> fixture.clock.stop();
			fixture.start();

			fixture.assertFinished();
			assertEquals(List.of(0L), fixture.ticks);
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void listenerPauseCanBeResumed(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(Double.MAX_VALUE);
			fixture.tickAction = tick -> {
				if (tick == 0) {
					fixture.clock.setPaused(true);
				}
			};
			fixture.start();
			fixture.awaitPaused();
			assertEquals(List.of(0L), fixture.ticks);

			fixture.clock.setPaused(false);
			fixture.assertFinished();
			assertEquals(100, fixture.ticks.size());
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void invalidSpeedsAreRejectedWithoutChangingCurrentSpeed(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.setSpeedMultiplier(1.75);
			for (double speed : new double[] {0, -0.0, -1, Double.NaN, Double.POSITIVE_INFINITY,
					Double.NEGATIVE_INFINITY}) {
				assertThrows(IllegalArgumentException.class, () -> fixture.clock.setSpeedMultiplier(speed));
				assertEquals(1.75, fixture.clock.getSpeedMultiplier());
			}
			fixture.clock.setSpeedMultiplier(Double.MIN_VALUE);
			assertEquals(Double.MIN_VALUE, fixture.clock.getSpeedMultiplier());
			fixture.clock.setSpeedMultiplier(Double.MAX_VALUE);
			assertEquals(Double.MAX_VALUE, fixture.clock.getSpeedMultiplier());
		}
	}

	@ParameterizedTest
	@EnumSource(Mode.class)
	void stopBeforeStartingCannotBeRestarted(Mode mode) throws Exception {
		try (var fixture = new Fixture(mode)) {
			fixture.clock.stop();
			assertEquals(SimulationLoop.State.STOPPED, fixture.clock.getState());
			fixture.clock.setPaused(false);
			fixture.clock.stepOnce();
			fixture.start();

			fixture.assertFinished();
			assertEquals(List.of(), fixture.ticks);
		}
	}

	private static final class Fixture implements AutoCloseable {
		final List<Long> ticks = new CopyOnWriteArrayList<>();
		final AtomicInteger ends = new AtomicInteger();
		final AtomicInteger controllerStarts = new AtomicInteger();
		final AtomicBoolean interruptedOnExit = new AtomicBoolean();
		final SimClock clock;
		final Thread worker;
		volatile LongConsumer tickAction = tick -> {
		};

		Fixture(Mode mode) {
			var listener = new SimulationListener() {
				@Override
				public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
					ticks.add(tick);
					tickAction.accept(tick);
				}

				@Override
				public void onMatchEnd() {
					ends.incrementAndGet();
				}
			};
			Runnable run;
			if (mode == Mode.REPLAY) {
				var frames = IntStream.range(0, 100).mapToObj(i -> new ReplayFrame(i, List.of(), List.of())).toList();
				var player = new ReplayPlayer(frames, 50, listener);
				clock = player;
				run = player::run;
			} else {
				var config = MatchConfig.withDefaults(0).withMatchDurationTicks(100);
				var terrain = new TerrainMap(new double[] {-1000, -1000, -1000, -1000}, 2, 2, -3000, -3000, 6000);
				var world = new GeneratedWorld(config, terrain, List.of(), new CurrentField(List.of()),
						List.of(new Vec3(-1000, 0, -100), new Vec3(1000, 0, -100)));
				var controller = new SubmarineController() {
					@Override
					public void onMatchStart(MatchContext context) {
						controllerStarts.incrementAndGet();
					}

					@Override
					public void onTick(SubmarineInput input, SubmarineOutput output) {
						output.setBallast(0.5);
					}
				};
				var simulation = new SimulationLoop();
				clock = simulation;
				run = () -> simulation.run(world, List.of(controller, controller),
						List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), List.of(0.0, 0.0), listener);
			}
			worker = new Thread(() -> {
				try {
					run.run();
				} finally {
					interruptedOnExit.set(Thread.currentThread().isInterrupted());
				}
			}, "clock-controls-" + mode);
		}

		void start() {
			worker.start();
		}

		void awaitPaused() throws InterruptedException {
			await(() -> clock.getState() == SimulationLoop.State.PAUSED && waiting(), "worker enters pause wait");
		}

		void awaitPacing() throws InterruptedException {
			await(() -> ticks.size() == 1 && clock.getState() == SimulationLoop.State.RUNNING && waiting(),
					"worker enters pacing wait after its first frame");
		}

		void awaitCount(int expected) throws InterruptedException {
			await(() -> ticks.size() >= expected, "worker emits " + expected + " frames");
			assertEquals(expected, ticks.size());
		}

		boolean waiting() {
			return worker.getState() == Thread.State.WAITING || worker.getState() == Thread.State.TIMED_WAITING;
		}

		void assertFinished() throws InterruptedException {
			worker.join(1000);
			assertFalse(worker.isAlive(), "clock must wake and finish promptly");
			assertEquals(SimulationLoop.State.STOPPED, clock.getState());
			assertEquals(1, ends.get(), "onMatchEnd fires exactly once");
		}

		@Override
		public void close() throws InterruptedException {
			clock.stop();
			if (worker.isAlive()) {
				worker.interrupt();
				worker.join(1000);
			}
		}
	}

	private static void await(BooleanSupplier ready, String message) throws InterruptedException {
		long start = System.nanoTime();
		while (!ready.getAsBoolean() && System.nanoTime() - start < TimeUnit.SECONDS.toNanos(3)) {
			Thread.sleep(1);
		}
		assertTrue(ready.getAsBoolean(), message);
	}
}
