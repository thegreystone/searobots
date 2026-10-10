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

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.awt.Color;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Controlled trajectories isolate sonar/TMA and Codex fire gating from navigation and weapons. The
 * engine produces every observation. Ground truth is used only to script motion and score error,
 * and internal tracker identity is never supplied to the controller. No opponent is used.
 */
class CodexTmaCatAndMouseTest {

	private static final int TICK_RATE = 50;
	private static final double DT = 1.0 / TICK_RATE;
	private static final long END_TICK = 300L * TICK_RATE;
	private static final long FIRST_FIX_TICK = 150L * TICK_RATE;
	private static final long GAP_START_TICK = 180L * TICK_RATE;
	private static final long REACQUIRE_FIX_TICK = 240L * TICK_RATE;
	private static final long INTERNAL_PAIR_KEY = 1L; // listener ID 0, source ID 1
	private static final long MASTER_SEED = 0x544D41434154L;
	private static final Field TRACKERS = trackerField();

	@ParameterizedTest(name = "seed={0}, gap={1}s")
	@MethodSource("sensorSeeds")
	void shortAndLongContactGapsPreserveInternalTrackAndPermitRenewedFire(long seed, int gapSeconds) {
		var result = runScenario(seed, gapSeconds);
		var before = result.at("before_gap");
		var lost = result.at("gap_end");
		var passive = result.at("passive_reacquired");
		var active = result.at("active_reacquired");
		assertAll("Gap of " + gapSeconds + " seconds",
				() -> assertTrue(result.sameInternalTracker(), "The internal pair tracker must survive a short gap"),
				() -> assertFalse(lost.heard(), "The scenario must actually lose passive contact"),
				() -> assertTrue(passive.heard(), "The scripted source must be passively reacquired"),
				() -> assertTrue(lost.codexAlive() < before.codexAlive(), "Alive belief must decay in the gap"),
				() -> assertTrue(lost.codexUncertainty() > before.codexUncertainty(),
						"Prediction uncertainty must grow during lost contact"),
				() -> assertTrue(lost.codexConfidence() < before.codexConfidence(),
						"The predicted track must become less confident without observations"),
				() -> assertTrue(passive.codexAlive() > lost.codexAlive(),
						"New acoustic evidence restores alive belief"),
				() -> assertTrue(active.active(), "The final phase must receive a real active echo"),
				() -> assertTrue(active.codexError() < 250, "A new range fix should restore a useful position"),
				() -> assertTrue(active.codexConfidence() > lost.codexConfidence(),
						"Active reacquisition must restore confidence"),
				() -> assertTrue(result.shotsBeforeGap() > 0, "The initial range fix must permit an attack"),
				() -> assertEquals(0, result.shotsDuringGap(), "A stale lost-contact track must not trigger firing"),
				() -> assertEquals(0, result.shotsAfterPassive(),
						"Passive reacquisition alone cannot refresh active range"),
				() -> assertTrue(result.shotsAfterActive() > 0, "A fresh range fix must permit renewed firing"));
	}

	private static Stream<Arguments> sensorSeeds() {
		var random = new Random(MASTER_SEED);
		var arguments = new ArrayList<Arguments>();
		for (int index = 0; index < 20; index++) {
			long seed = random.nextLong();
			arguments.add(Arguments.of(seed, 1));
			arguments.add(Arguments.of(seed, 12));
		}
		return arguments.stream();
	}

	/**
	 * Runs a fixed set of twenty sensor seeds, with both gap durations, and emits CSV diagnostics.
	 */
	public static void main(String[] args) {
		long masterSeed = MASTER_SEED;
		int seedCount = 20;
		for (String arg : args) {
			if (arg.startsWith("--master=")) {
				masterSeed = Long.decode(arg.substring("--master=".length()));
			} else if (arg.startsWith("--count=")) {
				seedCount = Integer.parseInt(arg.substring("--count=".length()));
			} else {
				throw new IllegalArgumentException("Unsupported argument: " + arg);
			}
		}
		var random = new Random(masterSeed);
		System.out
				.println("seed,gap_s,phase,time_s,heard,active,internal_pair_key,same_internal_tracker,engine_quality,"
						+ "engine_range_uncertainty_m,sonar_position_error_m,codex_position_error_m,codex_confidence,codex_alive,"
						+ "codex_uncertainty_m,pings_requested,shots_before_gap,shots_during_gap,shots_after_passive,shots_after_active");
		for (int index = 0; index < seedCount; index++) {
			long seed = random.nextLong();
			for (int gapSeconds : new int[] {1, 12}) {
				var result = runScenario(seed, gapSeconds);
				for (var point : result.checkpoints()) {
					System.out.printf(Locale.US,
							"%d,%d,%s,%.2f,%s,%s,%d,%s,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,%.3f,%d,%d,%d,%d,%d%n", seed,
							gapSeconds, point.phase(), point.tick() * DT, point.heard(), point.active(),
							INTERNAL_PAIR_KEY, result.sameInternalTracker(), point.engineQuality(),
							point.engineUncertainty(), point.sonarError(), point.codexError(), point.codexConfidence(),
							point.codexAlive(), point.codexUncertainty(), result.pingRequests(),
							result.shotsBeforeGap(), result.shotsDuringGap(), result.shotsAfterPassive(),
							result.shotsAfterActive());
				}
			}
		}
	}

	private static Result runScenario(long seed, int gapSeconds) {
		var world = GeneratedWorld.deepFlat();
		var config = MatchConfig.withDefaults(seed);
		var environment = new EnvironmentSnapshot(world.terrain(), List.of(), world.currentField());
		var controller = new CodexAttackSub();
		controller.onMatchStart(new MatchContext(config, world.terrain(), List.of(), world.currentField()));
		var own = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, -160), 0, Color.GREEN, 1000);
		var target = new SubmarineEntity(VehicleConfig.submarine(), 1, null, new Vec3(0, 1800, -220), Math.PI / 2,
				Color.RED, 1000);
		own.setSpeed(5);
		target.setSpeed(7);
		own.setSourceLevelDb(80);
		var sonar = new SonarModel(seed);
		var entities = List.of(own, target);
		long gapEndTick = GAP_START_TICK + gapSeconds * TICK_RATE;
		var checkpoints = new ArrayList<Checkpoint>();
		ContactTracker firstTracker = null;
		boolean sameTracker = true;
		int pingRequests = 0, shotsBeforeGap = 0, shotsDuringGap = 0, shotsAfterPassive = 0, shotsAfterActive = 0;

		for (long tick = 0; tick < END_TICK; tick++) {
			double time = tick * DT;
			setHeadingAndRate(own, ownHeading(time));
			setHeadingAndRate(target, Math.PI / 2 + Math.toRadians(2.5) * Math.clamp(time - 180, 0, gapSeconds));
			boolean inGap = tick >= GAP_START_TICK && tick < gapEndTick;
			target.setSourceLevelDb(inGap ? 60 : 125);
			// Passive-only phases deliberately do not execute controller ping requests. A known first
			// range fix exercises pre-gap firing; active sonar becomes available again after reacquisition.
			if (tick == FIRST_FIX_TICK || tick == REACQUIRE_FIX_TICK) {
				own.setActiveSonarCooldown(0);
				own.activeSonarPing();
			}
			var measurements = sonar.computeContacts(tick, entities, world.terrain(), List.of()).get(own.id());
			var tracker = internalTracker(sonar);
			if (tracker != null) {
				if (firstTracker == null) {
					firstTracker = tracker;
				} else {
					sameTracker &= tracker == firstTracker;
				}
			} else if (firstTracker != null) {
				sameTracker = false;
			}

			int spent = shotsBeforeGap + shotsDuringGap + shotsAfterPassive + shotsAfterActive;
			var self = new SubmarineState(own.pose(), own.velocity(), own.speed(), own.hp(), Math.max(0, 8 - spent));
			var output = new TestHelpers.CapturedOutput();
			controller.onTick(new TestHelpers.TestInput(tick, DT, self, environment, measurements.passiveContacts(),
					measurements.activeReturns(), measurements.cooldownTicks()), output);
			if (output.pinged) {
				pingRequests++;
				if (tick >= REACQUIRE_FIX_TICK) {
					own.activeSonarPing();
				}
			}
			if (tick < GAP_START_TICK) {
				shotsBeforeGap += output.launchedTorpedoCount;
			} else if (inGap) {
				shotsDuringGap += output.launchedTorpedoCount;
			} else if (tick < REACQUIRE_FIX_TICK) {
				shotsAfterPassive += output.launchedTorpedoCount;
			} else {
				shotsAfterActive += output.launchedTorpedoCount;
			}

			String phase = phaseAt(tick, gapEndTick);
			if (phase != null) {
				checkpoints.add(checkpoint(phase, tick, own, target, measurements, tracker, output));
			}
			sonar.postTick(entities);
			advance(own);
			advance(target);
		}
		return new Result(List.copyOf(checkpoints), firstTracker != null && sameTracker, pingRequests, shotsBeforeGap,
				shotsDuringGap, shotsAfterPassive, shotsAfterActive);
	}

	private static Checkpoint checkpoint(
		String phase, long tick, SubmarineEntity own, SubmarineEntity target, SonarModel.SonarResult measurements,
		ContactTracker tracker, TestHelpers.CapturedOutput output) {
		var passive = measurements.passiveContacts();
		var active = measurements.activeReturns();
		SonarContact contact = !active.isEmpty() ? active.getFirst() : passive.isEmpty() ? null : passive.getFirst();
		double sonarError = Double.NaN;
		if (contact != null && contact.range() > 0) {
			double depthDelta = contact.isActive() && Double.isFinite(contact.estimatedDepth())
					? contact.estimatedDepth() - own.z() : 0;
			double horizontalRange = Math
					.sqrt(Math.max(0, contact.range() * contact.range() - depthDelta * depthDelta));
			double ex = own.x() + Math.sin(contact.bearing()) * horizontalRange;
			double ey = own.y() + Math.cos(contact.bearing()) * horizontalRange;
			sonarError = Math.hypot(ex - target.x(), ey - target.y());
		}
		ContactEstimate estimate = output.contactEstimates.isEmpty() ? null : output.contactEstimates.getLast();
		return new Checkpoint(phase, tick, !passive.isEmpty(), !active.isEmpty(),
				tracker == null ? Double.NaN : tracker.solutionQuality(),
				tracker == null ? Double.NaN : tracker.rangeUncertainty(), sonarError,
				estimate == null ? Double.NaN : Math.hypot(estimate.x() - target.x(), estimate.y() - target.y()),
				estimate == null ? 0 : estimate.confidence(), estimate == null ? 0 : estimate.contactAlive(),
				estimate == null ? Double.NaN : estimate.uncertaintyRadius());
	}

	private static String phaseAt(long tick, long gapEndTick) {
		if (tick == 60L * TICK_RATE - 1)
			return "before_leg_change";
		if (tick == FIRST_FIX_TICK - 1)
			return "after_leg_change";
		if (tick == FIRST_FIX_TICK)
			return "first_active_fix";
		if (tick == GAP_START_TICK - 1)
			return "before_gap";
		if (tick == gapEndTick - 1)
			return "gap_end";
		if (tick == gapEndTick)
			return "passive_reacquired";
		if (tick == REACQUIRE_FIX_TICK - 1)
			return "before_active_reacquisition";
		if (tick == REACQUIRE_FIX_TICK)
			return "active_reacquired";
		if (tick == END_TICK - 1)
			return "final";
		return null;
	}

	private static double ownHeading(double time) {
		if (time < 60)
			return 0;
		if (time < 90)
			return Math.toRadians(2 * (time - 60));
		if (time < 140)
			return Math.toRadians(60);
		if (time < 150)
			return Math.toRadians(60 - 2.5 * (time - 140));
		if (time < 210)
			return Math.toRadians(35);
		if (time < 240)
			return Math.toRadians(35 + 2.0 / 3 * (time - 210));
		return Math.toRadians(55);
	}

	private static void advance(SubmarineEntity sub) {
		sub.setX(sub.x() + sub.speed() * Math.sin(sub.heading()) * DT);
		sub.setY(sub.y() + sub.speed() * Math.cos(sub.heading()) * DT);
	}

	private static void setHeadingAndRate(SubmarineEntity sub, double heading) {
		double change = heading - sub.heading();
		sub.setYawRate(Math.atan2(Math.sin(change), Math.cos(change)) / DT);
		sub.setHeading(heading);
	}

	private static Field trackerField() {
		try {
			var field = SonarModel.class.getDeclaredField("trackers");
			field.setAccessible(true);
			return field;
		} catch (ReflectiveOperationException exception) {
			throw new IllegalStateException("The shared-engine continuity probe is unavailable", exception);
		}
	}

	@SuppressWarnings("unchecked")
	private static ContactTracker internalTracker(SonarModel model) {
		try {
			return ((Map<Long, ContactTracker>) TRACKERS.get(model)).get(INTERNAL_PAIR_KEY);
		} catch (IllegalAccessException exception) {
			throw new IllegalStateException("The shared-engine continuity probe is unavailable", exception);
		}
	}

	private record Checkpoint(String phase, long tick, boolean heard, boolean active, double engineQuality,
			double engineUncertainty, double sonarError, double codexError, double codexConfidence, double codexAlive,
			double codexUncertainty) {
	}

	private record Result(List<Checkpoint> checkpoints, boolean sameInternalTracker, int pingRequests,
			int shotsBeforeGap, int shotsDuringGap, int shotsAfterPassive, int shotsAfterActive) {
		Checkpoint at(String phase) {
			return checkpoints.stream().filter(point -> point.phase().equals(phase)).findFirst().orElseThrow();
		}
	}
}
