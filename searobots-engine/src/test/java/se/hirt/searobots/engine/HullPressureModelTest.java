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
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HullPressureModelTest {

	private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
	private static final double DT = 1.0 / CONFIG.tickRateHz();

	@Test
	void ratedAndShallowerDepthsAreSafeEvenWithExistingDamage() {
		var model = new HullPressureModel(CONFIG, id -> 1e-12);
		var sub = submarine(0, 1000);
		sub.setHp(100);

		runAtDepth(model, sub, CONFIG.ratedDepth(), 100, DT);
		runAtDepth(model, sub, -100, 100, DT);

		assertEquals(100, sub.hp(), "Safe depth should neither damage nor implode a damaged hull");
	}

	@Test
	void shallowExcursionCausesSmallGradualDamage() {
		var model = withoutImplosion();
		var sub = submarine(0, 1000);

		model.step(sub, DT, -450);
		assertEquals(1000, sub.hp(), "Fractional damage should not round up to one HP every tick");
		runAtDepth(model, sub, -450, 10 - DT, DT);

		assertTrue(sub.hp() >= 997 && sub.hp() <= 998,
				"Ten seconds 50 m below rated depth should cost only about 3 HP");
	}

	@Test
	void fractionalDamageSurvivesAVisitToSafeDepth() {
		var model = withoutImplosion();
		var sub = submarine(0, 1000);

		runAtDepth(model, sub, -450, 2, DT);
		assertEquals(1000, sub.hp(), "The first excursion should accumulate less than one HP");
		runAtDepth(model, sub, -300, 100, DT);
		assertEquals(1000, sub.hp(), "Ascending should stop further pressure damage");
		runAtDepth(model, sub, -450, 2, DT);

		assertEquals(999, sub.hp(), "The two excursions together should accumulate one whole HP");
	}

	@Test
	void deeperOperationCausesMorePressureDamage() {
		var model = withoutImplosion();
		var shallow = submarine(0, 1000);
		var deep = submarine(1, 1000);

		runAtDepth(model, shallow, -450, 10, DT);
		runAtDepth(model, deep, -550, 10, DT);

		assertTrue(deep.hp() < shallow.hp(), "Greater depth should increase pressure damage");
	}

	@Test
	void existingDamageIncreasesPressureDamage() {
		var model = withoutImplosion();
		var healthy = submarine(0, 1000);
		var damaged = submarine(1, 1000);
		damaged.setHp(500);

		runAtDepth(model, healthy, -550, 10, DT);
		runAtDepth(model, damaged, -550, 10, DT);

		assertTrue(500 - damaged.hp() > 1000 - healthy.hp(), "A weakened hull should lose more HP at the same depth");
	}

	@Test
	void implosionOccursWhenAccumulatedExposureCrossesItsThreshold() {
		// At 650 m, a healthy hull accumulates about 0.00032 hazard per 10 ms.
		var model = new HullPressureModel(CONFIG, id -> 0.0005);
		var sub = submarine(0, 1000);

		model.step(sub, 0.01, -650);
		assertEquals(1000, sub.hp(), "One short exposure should not cross the threshold");
		model.step(sub, 0.01, -650);

		assertEquals(0, sub.hp(), "The second exposure should cross the fixed failure threshold");
	}

	@Test
	void safeDepthPausesButDoesNotEraseImplosionExposure() {
		var model = new HullPressureModel(CONFIG, id -> 0.0005);
		var sub = submarine(0, 1000);

		model.step(sub, 0.01, -650);
		runAtDepth(model, sub, -300, 100, DT);
		assertEquals(1000, sub.hp(), "Safe operation should not accumulate additional hazard");
		model.step(sub, 0.01, -650);

		assertEquals(0, sub.hp(), "A second dive should retain the first dive's exposure");
	}

	@Test
	void greaterDepthIncreasesImplosionRisk() {
		var model = new HullPressureModel(CONFIG, id -> 0.15);
		var shallow = submarine(0, 1000);
		var deep = submarine(1, 1000);

		runAtDepth(model, shallow, -600, 10, DT);
		runAtDepth(model, deep, -650, 10, DT);

		assertTrue(shallow.hp() > 0, "The shallower hull should remain below the same hazard threshold");
		assertEquals(0, deep.hp(), "The deeper hull should cross the threshold first");
	}

	@Test
	void existingDamageIncreasesImplosionRisk() {
		var model = new HullPressureModel(CONFIG, id -> 0.6);
		var healthy = submarine(0, 1000);
		var damaged = submarine(1, 1000);
		damaged.setHp(500);

		runAtDepth(model, healthy, -650, 10, DT);
		runAtDepth(model, damaged, -650, 10, DT);

		assertTrue(healthy.hp() > 0, "The healthy hull should remain below the hazard threshold");
		assertEquals(0, damaged.hp(), "Existing damage should make the same dive more dangerous");
	}

	@Test
	void customDepthLimitsControlDamageAndAbsoluteDestruction() {
		var config = withDepthLimits(-100, -200);
		var model = new HullPressureModel(config, id -> Double.MAX_VALUE);
		var sub = submarine(0, 1000);

		runAtDepth(model, sub, -100, 10, DT);
		assertEquals(1000, sub.hp());
		runAtDepth(model, sub, -125, 10, DT);
		assertTrue(sub.hp() < 1000 && sub.hp() > 0, "Damage should begin at the configured rated depth");
		model.step(sub, DT, -200);

		assertEquals(0, sub.hp(), "The configured crush depth should be an absolute limit");
	}

	@Test
	void usesReachedDepthEvenWhenTheEntityWasRepositioned() {
		var model = withoutImplosion();
		var sub = submarine(0, 1000);
		sub.setZ(-200);

		model.step(sub, DT, CONFIG.crushDepth());

		assertEquals(0, sub.hp(), "A reported crush-depth excursion must survive subsequent terrain correction");
	}

	@Test
	void nonSubmarinesAndInactiveEntitiesAreUnaffected() {
		var model = new HullPressureModel(CONFIG, id -> 1e-12);
		var torpedo = vehicle(VehicleConfig.torpedo(), 0, 1000);
		var surfaceShip = vehicle(VehicleConfig.surfaceShip(), 1, 1000);
		var forfeited = submarine(2, 1000);
		forfeited.setForfeited(true);
		var wreck = submarine(3, 1000);
		wreck.setHp(0);

		for (var entity : new SubmarineEntity[] {torpedo, surfaceShip, forfeited, wreck}) {
			model.step(entity, 10, -800);
		}

		assertEquals(1000, torpedo.hp());
		assertEquals(1000, surfaceShip.hp());
		assertEquals(1000, forfeited.hp());
		assertEquals(0, wreck.hp());
	}

	@Test
	void pressureDamageScalesWithMaximumHp() {
		var model = withoutImplosion();
		var standard = submarine(0, 1000);
		var smallerHpPool = submarine(1, 500);

		runAtDepth(model, standard, -550, 10, DT);
		runAtDepth(model, smallerHpPool, -550, 10, DT);

		assertEquals((1000 - standard.hp()) / 1000.0, (500 - smallerHpPool.hp()) / 500.0, 0.002,
				"Pressure should inflict comparable fractional damage with a different HP pool");
	}

	@Test
	void seedReproducesFailuresRegardlessOfEntityUpdateOrder() {
		int[] forward = seededFailureTicks(false);
		int[] repeated = seededFailureTicks(false);
		int[] reversed = seededFailureTicks(true);

		assertArrayEquals(forward, repeated, "The same match seed and controls must reproduce implosion timing");
		assertArrayEquals(forward, reversed, "Each entity's randomness must be independent of update order");
		for (int tick : forward) {
			assertTrue(tick > 0, "The seeded dives should eventually fail");
		}
	}

	@Test
	void newEntityWithTheSameIdStartsWithFreshExposure() {
		var model = new HullPressureModel(CONFIG, id -> 0.0005);
		var first = submarine(0, 1000);
		var replacement = submarine(0, 1000);
		model.step(first, 0.01, -650);
		model.step(replacement, 0.01, -650);

		assertEquals(1000, first.hp());
		assertEquals(1000, replacement.hp(), "A fresh entity must not inherit another hull's exposure");
		model.step(replacement, 0.01, -650);
		assertEquals(0, replacement.hp());
		assertEquals(1000, first.hp(), "Exposure state must belong to the individual hull");
	}

	@Test
	void damageAndImplosionTimingConvergeAtDifferentTickRates() {
		var slow = submarine(0, 1000);
		var fast = submarine(0, 1000);
		runAtDepth(withoutImplosion(), slow, -650, 10, 1.0 / 20);
		runAtDepth(withoutImplosion(), fast, -650, 10, 1.0 / 100);
		assertEquals(slow.hp(), fast.hp(), 1,
				"HP accounting should be independent of tick rate within integration error");

		double slowFailure = timeToImplosion(1.0 / 20);
		double fastFailure = timeToImplosion(1.0 / 100);
		assertEquals(slowFailure, fastFailure, 0.05 + 1e-9,
				"Integrated hazard should give comparable failure times at different tick rates");
	}

	private static HullPressureModel withoutImplosion() {
		return new HullPressureModel(CONFIG, id -> Double.MAX_VALUE);
	}

	private static SubmarineEntity submarine(int id, int maxHp) {
		return vehicle(VehicleConfig.submarine(), id, maxHp);
	}

	private static SubmarineEntity vehicle(VehicleConfig config, int id, int maxHp) {
		return new SubmarineEntity(config, id, null, new Vec3(0, 0, -200), 0, Color.GREEN, maxHp);
	}

	private static void runAtDepth(HullPressureModel model, SubmarineEntity sub, double z, double seconds, double dt) {
		for (int i = 0; i < Math.round(seconds / dt); i++) {
			model.step(sub, dt, z);
		}
	}

	private static double timeToImplosion(double dt) {
		var model = new HullPressureModel(CONFIG, id -> 0.15);
		var sub = submarine(0, 1000);
		for (int tick = 1; tick <= Math.round(20 / dt); tick++) {
			model.step(sub, dt, -650);
			if (sub.hp() == 0) {
				return tick * dt;
			}
		}
		throw new AssertionError("The fixed exposure threshold should be reached within 20 seconds");
	}

	private static int[] seededFailureTicks(boolean reversed) {
		var model = new HullPressureModel(CONFIG);
		var subs = new SubmarineEntity[] {submarine(0, 1000), submarine(1, 1000), submarine(2, 1000),
				submarine(3, 1000)};
		int[] failureTicks = new int[subs.length];
		for (int tick = 1; tick <= 6000; tick++) {
			for (int i = 0; i < subs.length; i++) {
				int index = reversed ? subs.length - 1 - i : i;
				model.step(subs[index], DT, -650);
				if (failureTicks[index] == 0 && subs[index].hp() == 0) {
					failureTicks[index] = tick;
				}
			}
		}
		return failureTicks;
	}

	private static MatchConfig withDepthLimits(double ratedDepth, double crushDepth) {
		return new MatchConfig(CONFIG.worldSeed(), CONFIG.tickRateHz(), CONFIG.matchDurationTicks(),
				CONFIG.submarineCount(), CONFIG.torpedoCount(), CONFIG.startingHp(), CONFIG.blastRadius(),
				CONFIG.minFuseRadius(), CONFIG.maxFuseRadius(), ratedDepth, crushDepth, CONFIG.battleArea(),
				CONFIG.terrainMarginMeters(), CONFIG.gridCellMeters(), CONFIG.minSeaFloorZ(), CONFIG.maxSeaFloorZ(),
				CONFIG.maxSubSpeed(), CONFIG.startTime());
	}
}
