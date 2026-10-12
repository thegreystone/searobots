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

import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.ThermalLayer;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Test-only access to engine internals for controller-specific test packages. Production engine
 * visibility remains unchanged, and these probes never become controller input.
 */
public final class EngineTestAccess {
	private EngineTestAccess() {
	}

	public record CombatResult(int pointsA, int pointsB, String description, long ticks) {
	}

	public static CombatResult runCombat(
		Supplier<SubmarineController> factoryA, String nameA, Supplier<SubmarineController> factoryB, String nameB,
		long seed, int durationTicks) {
		var result = SubmarineCompetition.runCombat(factoryA, nameA, factoryB, nameB, seed, durationTicks);
		return new CombatResult(result.pointsA(), result.pointsB(), result.description(), result.ticks());
	}

	public static long combatWallClockLimitMillis() {
		return SubmarineCompetition.COMBAT_WALL_CLOCK_LIMIT_MS;
	}

	public record SonarTrackerProbe(Object identity, double solutionQuality, double rangeUncertainty) {
	}

	public record SonarMeasurements(List<SonarContact> passiveContacts, List<SonarContact> activeReturns,
			int cooldownTicks) {
	}

	public static SonarMeasurements computeSonarContacts(
		SonarModel sonar, long tick, List<SubmarineEntity> entities, TerrainMap terrain, List<ThermalLayer> layers,
		int observerId) {
		var result = sonar.computeContacts(tick, entities, terrain, layers).get(observerId);
		return new SonarMeasurements(result.passiveContacts(), result.activeReturns(), result.cooldownTicks());
	}

	@SuppressWarnings("unchecked")
	public static Optional<SonarTrackerProbe> sonarTracker(SonarModel sonar, long pairKey) {
		try {
			var tracker = ((Map<Long, ContactTracker>) SonarInternals.TRACKERS.get(sonar)).get(pairKey);
			return tracker == null ? Optional.empty() : Optional
					.of(new SonarTrackerProbe(tracker, tracker.solutionQuality(), tracker.rangeUncertainty()));
		} catch (IllegalAccessException exception) {
			throw new IllegalStateException("The shared-engine continuity probe is unavailable", exception);
		}
	}

	private static Field sonarTrackersField() {
		try {
			var field = SonarModel.class.getDeclaredField("trackers");
			field.setAccessible(true);
			return field;
		} catch (ReflectiveOperationException exception) {
			throw new IllegalStateException("The shared-engine continuity probe is unavailable", exception);
		}
	}

	private static final class SonarInternals {
		private static final Field TRACKERS = sonarTrackersField();
	}
}
