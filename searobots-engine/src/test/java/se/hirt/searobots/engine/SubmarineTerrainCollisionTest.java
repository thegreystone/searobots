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
		var sub = submarine(-102, 0);
		step(sub, DT, FLAT);

		assertEquals(1000, sub.hp(), "Position correction must not become impact velocity.");
		assertEquals(-95, sub.z(), 1e-9, "The physical keel must rest on the floor without a navigation buffer.");
		assertEquals(0, sub.verticalSpeed(), 1e-9, "Correcting a stationary overlap must not create a bounce.");
		for (int tick = 0; tick < 100; tick++) {
			step(sub, DT, FLAT);
			assertEquals(-95, sub.z(), 1e-9);
			assertEquals(0, sub.verticalSpeed(), 1e-9);
			assertEquals(1000, sub.hp(), "Resting on the floor must not accumulate impact damage.");
		}
	}

	@Test
	void pitchedHullAndSternAppendageAreClearedFromAShallowFloor() {
		// These are actual lowest rendered vertices at the two pitches, in engine-local coordinates.
		// The old seven contact points left the forebody 1.74 m and stern appendage 1.34 m embedded.
		var forebody = new Vec3(0, 29.149572, -2.723796);
		var sternAppendage = new Vec3(2.847645, -35.355881, -2.744716);
		for (double degrees : new double[] {-10, 10}) {
			var sub = submarine(0, 0);
			var clearWater = submarine(0, 0);
			sub.setPitch(Math.toRadians(degrees));
			clearWater.setPitch(sub.pitch());
			var shallowFloor = flat(degrees < 0 ? -6 : -7.5);
			step(sub, DT, shallowFloor);
			step(clearWater, DT, flat(-1000));

			assertTrue(sub.z() > 1, "A pitched hull must be lifted enough to clear its rendered underside.");
			assertRenderedPointClear(sub, degrees < 0 ? forebody : sternAppendage, shallowFloor);
			assertEquals(clearWater.pitch(), sub.pitch(), 1e-9,
					"Geometric correction must preserve the normal hydrostatic pitch integration.");
			assertEquals(0, sub.verticalSpeed(), 1e-9, "Existing overlap must not launch the hull upward.");
			assertEquals(1000, sub.hp(), "Existing overlap without inward motion must not cause impact damage.");
		}
	}

	@Test
	void ridgeBetweenTheFormerCentreAndBowSamplesClearsTheWholeHull() {
		var sub = submarine(-100, 0);
		// A single raised row in the default 10 m terrain grid lies between the former samples.
		var ridge = terrain((x, y) -> y == 20 ? -98 : -120);
		step(sub, DT, ridge);

		assertTrue(sub.z() > -98, "The interior hull section must detect the ridge under its belly.");
		assertRenderedPointClear(sub, new Vec3(0, 20.477005, -3.511588), ridge);
		assertEquals(1000, sub.hp());
		assertEquals(0, sub.speed(), 1e-9);
		assertEquals(0, sub.verticalSpeed(), 1e-9, "Static contact projection must not create upward velocity.");
		for (int tick = 0; tick < 10; tick++) {
			step(sub, DT, ridge);
			assertRenderedPointClear(sub, new Vec3(0, 20.477005, -3.511588), ridge);
			assertEquals(1000, sub.hp());
			assertEquals(0, sub.verticalSpeed(), 1e-9);
		}
	}

	@Test
	void tangentialMotionWhileEmbeddedDoesNotInventAnImpactOrContactDrag() {
		var sub = submarine(-102, 0);
		var clearWater = submarine(-102, 0);
		sub.setSpeed(5);
		clearWater.setSpeed(5);
		for (int tick = 0; tick < 20; tick++) {
			step(sub, DT, FLAT);
			step(clearWater, DT, flat(-1000));
			assertEquals(clearWater.speed(), sub.speed(), 1e-9,
					"Tangential movement must retain ordinary water drag rather than an artificial impact penalty.");
			assertEquals(0, sub.verticalSpeed(), 1e-9);
			assertEquals(1000, sub.hp());
		}
	}

	@Test
	void ridgeBetweenSternFinCornersClearsTheRenderedAppendage() {
		var sub = submarine(-100, 0);
		sub.setY(4);
		// The raised grid row passes through the fin between its local forward endpoints.
		// Its underside reaches below the tapered body here, so body contact alone is insufficient.
		var ridge = terrain((x, y) -> y == -30 ? -98 : -120);
		step(sub, DT, ridge);

		assertRenderedPointClear(sub, new Vec3(2.784986, -33.997710, -2.797510), ridge);
		assertEquals(1000, sub.hp());
		assertEquals(0, sub.verticalSpeed(), 1e-9);
	}

	@Test
	void surfacedSubmarineKeepsItsWaterlineWhenTheKeelClearsTheFloor() {
		for (double waterDepth : new double[] {10, 5}) {
			var sub = submarine(0, 0);
			var water = flat(-waterDepth);
			for (int tick = 0; tick < 100; tick++) {
				step(sub, DT, water);
				assertEquals(0, sub.z(), 1e-9, "Water depth " + waterDepth + " must not lift a surfaced hull.");
				assertEquals(0, sub.verticalSpeed(), 1e-9, "Navigation clearance must not cause a bounce.");
				assertEquals(1000, sub.hp());
			}
		}
	}

	@Test
	void fastDescentInsideNavigationMarginDoesNotCausePhysicalContact() {
		var nearFloor = submarine(-90, -1);
		var clearWater = submarine(-90, -1);
		nearFloor.setSpeed(10);
		clearWater.setSpeed(10);
		var deepFloor = flat(-1000);
		for (int tick = 0; tick < 100; tick++) {
			step(nearFloor, DT, FLAT);
			step(clearWater, DT, deepFloor);
			double keelGap = nearFloor.z() - 5 + 100;
			assertTrue(keelGap > 0 && keelGap < nearFloor.vehicleConfig().terrainClearance(),
					"The hull remains physically clear while inside its navigation safety margin.");
			assertEquals(1000, nearFloor.hp(), "Clear water below the keel must not cause impact damage.");
			assertEquals(clearWater.z(), nearFloor.z(), 1e-9, "Navigation clearance must not lift the hull.");
			assertEquals(clearWater.speed(), nearFloor.speed(), 1e-9,
					"Navigation clearance must not apply contact drag.");
			assertEquals(clearWater.verticalSpeed(), nearFloor.verticalSpeed(), 1e-9,
					"Navigation clearance must not reverse a real descent into a bounce.");
			assertEquals(clearWater.sourceLevelDb(), nearFloor.sourceLevelDb(), 1e-9,
					"A hull clear of the seabed must not generate scraping noise.");
		}
	}

	@Test
	void wreckRestsDirectlyOnItsPhysicalKeel() {
		var wreck = submarine(-96, -1);
		wreck.setHp(0);
		for (int tick = 0; tick < 100; tick++) {
			step(wreck, DT, FLAT);
			assertEquals(-95, wreck.z(), 1e-9, "A wreck must not retain a one-metre clearance buffer.");
			assertEquals(0, wreck.z() - 5 + 100, 1e-9);
			assertEquals(0, wreck.verticalSpeed(), 1e-9);
			assertEquals(0, wreck.hp());
		}
	}

	@Test
	void embeddedHullMovingAwayFromTheFloorDoesNotTakeImpactDamage() {
		var sub = submarine(-102, 2);
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
			var sub = submarine(-102, -2);
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
		var slow = submarine(-95, -2);
		var fast = submarine(-95, -8);
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
		var sub = submarine(-94, -Math.sqrt(8));
		var slope = terrain((x, y) -> -100 + x);
		step(sub, DT, slope);

		// At the starboard contact, a vertical speed sqrt(8) has a 2 m/s normal component.
		// The combined sway/heave effective mass is about 5.92 million kg, giving 47 HP.
		int damage = 1000 - sub.hp();
		assertTrue(damage >= 43 && damage <= 48, "The surface normal must have unit length: " + damage);
	}

	@Test
	void flankSpeedBowStrikeOnSteepRidgeIsLethal() {
		var sub = submarine(-100, 0);
		sub.setSpeed(15);
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 + 10 * (y - 33.5) : -400);
		step(sub, DT, ridge);

		assertEquals(0, sub.hp(), "A direct 15 m/s bow strike must destroy a default 1000-HP hull.");
	}

	@Test
	void fastBowOnlyDescentCausesMajorDamageEvenWhenItCanRotate() {
		var sub = submarine(-100, -8);
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 : -140);
		step(sub, DT, ridge);

		assertTrue(sub.hp() >= 800 && sub.hp() <= 860,
				"An 8 m/s bow-only descent must remove substantial HP despite rotational relief: " + sub.hp());
		assertTrue(sub.verticalSpeed() > 0);
	}

	@Test
	void massiveKeelContactCanDominateAFasterRotatingSternContact() {
		var sub = submarine(-102, -4);
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
		var sub = submarine(-100, 0);
		sub.setYawRate(0.1);
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -100 + x : -140);
		step(sub, DT, ridge);

		assertEquals(0, sub.speed(), 1e-9);
		assertTrue(sub.hp() > 900 && sub.hp() < 1000, "Bow rotation is real contact motion even without surge.");
	}

	@Test
	void pitchingBowCanImpactTerrainWithAStationaryCentre() {
		var sub = submarine(-100, 0);
		sub.setPitchRate(-0.1);
		var level = submarine(-100, 0);
		// At forward=30 m the continuous lower body is 2.65 m below its origin. A 2.70 m
		// gap leaves the level bow clear, but real inward pitch moves it into the ridge.
		// The centre keel remains clear of the deeper floor.
		var ridge = terrain((x, y) -> y >= 30 && y <= 40 ? -102.7 : -140);
		step(level, DT, ridge);
		step(sub, DT, ridge);

		assertEquals(1000, level.hp(), "A level, stationary hull must remain clear of the ridge.");
		assertEquals(-100, level.z(), 1e-9);
		assertEquals(0, sub.x(), 1e-9);
		assertEquals(0, sub.y(), 1e-9);
		assertEquals(0, sub.speed(), 1e-9);
		assertTrue(sub.hp() > 900 && sub.hp() < 1000,
				"Pitch moves the bow into terrain even when the centre has no incoming linear motion.");
	}

	@Test
	void bouncePitchCorrectionLeavesTheHullClearOnTheNextTick() {
		var sub = submarine(-95.03, -2);
		sub.setPitch(-0.08);
		step(sub, DT, FLAT);

		assertEquals(0, sub.pitch(), 1e-9);
		assertTrue(sub.verticalSpeed() > 0, "Pitch correction must preserve the upward collision response.");
		assertTrue(sub.z() - 5 >= -100 - 1e-9, "Changing pitch during the bounce must not leave the keel embedded.");
		int hpAfterContact = sub.hp();
		step(sub, DT, FLAT);
		assertEquals(hpAfterContact, sub.hp(), "The previous geometric correction must not cause another impact.");
	}

	@Test
	void surfaceShipBowAndSternReachTerrainBeyondTheSubmarineSamples() {
		for (double heading : new double[] {0, Math.PI / 2, Math.PI}) {
			var ship = surfaceShip(heading);
			var shoal = terrain((x, y) -> (heading == Math.PI / 2 ? x : Math.abs(y)) >= 80 ? -2 : -50);
			step(ship, DT, shoal);
			assertTrue(ship.z() > 0, "The ship's long submerged hull must detect the distant shoal.");
			assertEquals(1000, ship.hp(), "Correcting a stationary overlap must not invent an impact.");
		}
	}

	@Test
	void surfaceShipBroadBeamDetectsAShoalOutsideTheSubmarineBeam() {
		var ship = surfaceShip(0);
		// The raised band starts beyond the submarine's six-metre half-beam.
		// It must reach the physical ship side, rather than only its navigation margin.
		var shoal = terrain((x, y) -> x >= 10 ? -2 : -50);
		step(ship, DT, shoal);
		assertTrue(ship.z() > 0, "The ship's side extends beyond the submarine's terrain sample.");
		assertEquals(1000, ship.hp());
	}

	@Test
	void surfaceShipGroundingUsesItsOwnPhysicalDraft() {
		var ship = surfaceShip(0);
		var shoal = flat(-9);
		step(ship, DT, shoal);
		var hull = HullGeometry.envelope(ship.vehicleConfig());
		assertEquals(-9 - hull.upOffset() + hull.semiHeight(), ship.z(), 1e-9);
		assertEquals(1000, ship.hp());
		step(ship, DT, shoal);
		assertEquals(1000, ship.hp(), "Restoring the waterline must not turn grounding correction into damage.");
	}

	@Test
	void surfaceShipKeepsItsWaterlineInsideNavigationClearance() {
		var ship = surfaceShip(0);
		var clearWater = surfaceShip(0);
		ship.setSpeed(5);
		clearWater.setSpeed(5);
		var shallowFloor = flat(-12);
		var deepFloor = flat(-50);
		for (int tick = 0; tick < 100; tick++) {
			step(ship, DT, shallowFloor);
			step(clearWater, DT, deepFloor);
			assertEquals(0, ship.z(), 1e-9, "The 9.5-metre draft clears a 12-metre water column.");
			assertEquals(1000, ship.hp());
			assertEquals(clearWater.speed(), ship.speed(), 1e-9);
			assertEquals(clearWater.sourceLevelDb(), ship.sourceLevelDb(), 1e-9);
		}
	}

	@Test
	void surfaceShipKeepsItsWaterlineWhenTheWholeHullIsClear() {
		var ship = surfaceShip(0);
		ship.setSpeed(5);
		step(ship, DT, flat(-50));
		assertEquals(0, ship.z(), 1e-9);
		assertEquals(1000, ship.hp());
		assertTrue(ship.y() > 0);
	}

	private static SubmarineEntity surfaceShip(double heading) {
		return new SubmarineEntity(VehicleConfig.surfaceShip(), 0, null, Vec3.ZERO, heading, Color.BLUE, 1000);
	}

	private static void assertRenderedPointClear(SubmarineEntity sub, Vec3 local, TerrainMap terrain) {
		double sinHeading = Math.sin(sub.heading()), cosHeading = Math.cos(sub.heading());
		double sinPitch = Math.sin(sub.pitch()), cosPitch = Math.cos(sub.pitch());
		double x = sub.x() + cosHeading * local.x() + sinHeading * cosPitch * local.y()
				- sinHeading * sinPitch * local.z();
		double y = sub.y() - sinHeading * local.x() + cosHeading * cosPitch * local.y()
				- cosHeading * sinPitch * local.z();
		double z = sub.z() + sinPitch * local.y() + cosPitch * local.z();
		assertTrue(z >= terrain.elevationAt(x, y) - 1e-6,
				"Rendered hull point " + local + " must clear the floor after contact correction.");
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
