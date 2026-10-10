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
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SubmarineTerrainCollisionTest {
	private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
	private static final CurrentField NO_CURRENT = new CurrentField(List.of());
	private static final TerrainMap FLAT = flat(-100);
	private static final double DT = 0.02;

	@Test
	void stationaryOverlapIsCorrectedWithoutInventingAnImpact() {
		var sub = submarine(-90, 0);
		step(sub, DT, FLAT);

		assertEquals(1000, sub.hp(), "Position correction must not become impact velocity.");
		assertTrue(sub.z() >= -100 + sub.vehicleConfig().terrainClearance() + 5 - 1e-9);
	}

	@Test
	void embeddedHullMovingAwayFromTheFloorDoesNotTakeImpactDamage() {
		var sub = submarine(-90, 2);
		step(sub, DT, FLAT);

		assertEquals(1000, sub.hp(), "Existing overlap does not make an outward movement an impact.");
	}

	@Test
	void terrainLiftAboveWaterDoesNotBecomeADownwardImpactOnTheNextTick() {
		var sub = submarine(0, 0);
		var shore = flat(0);
		step(sub, DT, shore);
		assertEquals(1000, sub.hp());
		assertTrue(sub.z() > 0, "A grounded hull can require correction above the water surface.");

		step(sub, DT, shore);
		assertEquals(1000, sub.hp(), "The water-surface clamp must not invent a downward impact.");
	}

	@Test
	void realDescentAfterTerrainLiftStillCausesFiniteImpactDamage() {
		var sub = submarine(0, 0);
		var shore = flat(0);
		step(sub, DT, shore);
		assertEquals(1000, sub.hp());
		assertTrue(sub.z() > 0);

		sub.setVerticalSpeed(-2);
		step(sub, DT, shore);

		int damage = 1000 - sub.hp();
		assertTrue(damage >= 40 && damage <= 46,
				"A real 2 m/s descent must still cause about 45 HP of damage after terrain lift: " + damage);
		assertTrue(sub.verticalSpeed() > 0, "The collision must still turn downward movement into an upward bounce.");
	}

	@Test
	void slowGroundingRemainsSurvivableAcrossTimesteps() {
		int minDamage = Integer.MAX_VALUE;
		int maxDamage = 0;
		for (double dt : new double[] {0.01, 0.02, 0.04}) {
			var sub = submarine(-90, -2);
			step(sub, dt, FLAT);
			int damage = 1000 - sub.hp();
			assertTrue(damage >= 40 && damage <= 46,
					"A 2 m/s impact should cost about 45 HP regardless of existing overlap or timestep: " + damage);
			minDamage = Math.min(minDamage, damage);
			maxDamage = Math.max(maxDamage, damage);
		}
		assertTrue(maxDamage - minDamage <= 2, "Changing the step must not amplify collision damage.");
	}

	@Test
	void highSpeedGroundingStillCausesSubstantialDamage() {
		var slow = submarine(-83, -2);
		var fast = submarine(-83, -8);
		slow.setSpeed(6);
		fast.setSpeed(6);
		step(slow, DT, FLAT);
		step(fast, DT, FLAT);

		assertTrue(fast.hp() > 0 && fast.hp() < 350,
				"An 8 m/s impact must remove at least 650 HP while remaining finite.");
		assertTrue(1000 - fast.hp() > 10 * (1000 - slow.hp()));
		assertTrue(fast.verticalSpeed() > 0, "A downward impact must still produce an upward bounce.");
		assertTrue(fast.speed() > 0 && fast.speed() < 4,
				"A hard impact must substantially reduce surge while retaining forward movement.");
		assertTrue(slow.speed() > fast.speed(), "A hard impact must slow the submarine more than a gentle contact.");
	}

	@Test
	void inclinedFloorUsesTheNormalComponentOfImpactSpeed() {
		var sub = submarine(-82, -Math.sqrt(8));
		var slope = terrain((x, y) -> -100 + x);
		step(sub, DT, slope);

		// At the starboard contact, a vertical speed sqrt(8) has a 2 m/s normal component.
		// The combined sway/heave effective mass is about 5.92 million kg, giving 47 HP.
		int damage = 1000 - sub.hp();
		assertTrue(damage >= 43 && damage <= 48, "The surface normal must have unit length: " + damage);
	}

	@Test
	void flankSpeedBowStrikeOnSteepRidgeIsLethal() {
		var sub = submarine(-88, 0);
		sub.setSpeed(15);
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 + 10 * (y - 33.5) : -400);
		step(sub, DT, ridge);

		assertEquals(0, sub.hp(), "A direct 15 m/s bow strike must destroy a default 1000-HP hull.");
	}

	@Test
	void fastBowOnlyDescentCausesMajorDamageEvenWhenItCanRotate() {
		var sub = submarine(-88, -8);
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 : -140);
		step(sub, DT, ridge);

		assertTrue(sub.hp() >= 800 && sub.hp() <= 860,
				"An 8 m/s bow-only descent must remove substantial HP despite rotational relief: " + sub.hp());
		assertTrue(sub.verticalSpeed() > 0);
	}

	@Test
	void massiveKeelContactCanDominateAFasterRotatingSternContact() {
		var sub = submarine(-90, -4);
		sub.setPitchRate(0.04);
		step(sub, DT, FLAT);

		// The pitching stern moves down faster, but can rotate away from its contact.
		// The centre/keel contact has more normal impact energy despite its lower speed.
		int damage = 1000 - sub.hp();
		assertTrue(damage >= 170 && damage <= 182,
				"Choose the largest contact energy rather than the fastest contact or their sum: " + damage);
	}

	@Test
	void rotatingBowCanImpactTerrainWithAStationaryCentre() {
		var sub = submarine(-88, 0);
		sub.setYawRate(0.1);
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 + x : -140);
		step(sub, DT, ridge);

		assertEquals(0, sub.speed(), 1e-9);
		assertTrue(sub.hp() > 900 && sub.hp() < 1000, "Bow rotation is real contact motion even without surge.");
	}

	@Test
	void pitchingBowCanImpactTerrainWithAStationaryCentre() {
		var sub = submarine(-88, 0);
		sub.setPitchRate(-0.1);
		var level = submarine(-88, 0);
		// Only the bow reaches this ridge. The stationary keel remains clear of the deeper floor.
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 : -140);
		step(level, DT, ridge);
		step(sub, DT, ridge);

		assertEquals(1000, level.hp(), "A level, stationary hull must remain clear of the ridge.");
		assertEquals(-88, level.z(), 1e-9);
		assertEquals(0, sub.x(), 1e-9);
		assertEquals(0, sub.y(), 1e-9);
		assertEquals(0, sub.speed(), 1e-9);
		assertTrue(sub.hp() > 900 && sub.hp() < 1000,
				"Pitch moves the bow into terrain even when the centre has no incoming linear motion.");
	}

	@Test
	void bouncePitchCorrectionLeavesTheHullClearOnTheNextTick() {
		var sub = submarine(-83.03, 0);
		sub.setPitch(-0.08);
		step(sub, DT, FLAT);

		assertEquals(0, sub.pitch(), 1e-9);
		assertTrue(sub.verticalSpeed() > 0, "Pitch correction must preserve the upward collision response.");
		assertTrue(sub.z() - 5 >= -100 + sub.vehicleConfig().terrainClearance() - 1e-9,
				"Changing pitch during the bounce must not leave the keel embedded.");
		int hpAfterContact = sub.hp();
		step(sub, DT, FLAT);
		assertEquals(hpAfterContact, sub.hp(), "The previous geometric correction must not cause another impact.");
	}

	private static SubmarineEntity submarine(double z, double verticalSpeed) {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, z), 0, Color.BLUE, 1000);
		sub.setVerticalSpeed(verticalSpeed);
		return sub;
	}

	private static void step(SubmarineEntity sub, double dt, TerrainMap terrain) {
		new SubmarinePhysics(CONFIG).step(sub, dt, terrain, NO_CURRENT, CONFIG.battleArea());
	}

	private static TerrainMap flat(double elevation) {
		double[] elevations = new double[441];
		Arrays.fill(elevations, elevation);
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}

	private static TerrainMap terrain(java.util.function.DoubleBinaryOperator elevation) {
		double[] elevations = new double[441];
		for (int row = 0; row < 21; row++) {
			for (int col = 0; col < 21; col++) {
				elevations[row * 21 + col] = elevation.applyAsDouble(-100 + col * 10, -100 + row * 10);
			}
		}
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}
}
