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
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class HullContactSolverTest {

	private static final CurrentField NO_CURRENT = new CurrentField(List.of());

	@Test
	void stationaryStackAtSeabedStaysClearWithoutDamageOrBounceOnFollowingTicks() {
		var terrain = flat(-20);
		var lower = sub(0, new Vec3(0, 0, -15), 0, 0);
		var upper = sub(1, new Vec3(0, 0, -7), 0, 0);
		var subs = List.of(lower, upper);
		var physics = new SubmarinePhysics();
		var area = MatchConfig.withDefaults(42).battleArea();

		for (int tick = 0; tick < 20; tick++) {
			for (var sub : subs) {
				physics.step(sub, 0.02, terrain, NO_CURRENT, area);
			}
			SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);
			assertAllClear(subs, terrain);
			assertStationaryAndUndamaged(subs);
		}
		assertEquals(-15, lower.z(), 1e-9, "The lower keel should remain at the seabed.");
		assertTrue(upper.z() > -7, "The upper hull must take the available upward separation.");
	}

	@Test
	void surfaceAndSeabedConstrainedStackSeparatesLaterally() {
		var terrain = flat(-10);
		for (double heading : new double[] {0, Math.PI / 4, 1.3}) {
			var lower = sub(0, new Vec3(0, 0, -5), heading, 0);
			var upper = sub(1, new Vec3(0, 0, -1), heading, 0);
			var subs = List.of(lower, upper);

			SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);

			assertAllClear(subs, terrain);
			assertStationaryAndUndamaged(subs);
			assertTrue(lower.z() >= -5 && upper.z() <= 0);
			double lateralSeparation = Math.hypot(lower.x() - upper.x(), lower.y() - upper.y());
			assertTrue(lateralSeparation > 0, "There is insufficient water depth for a purely vertical separation.");
			assertTrue(lateralSeparation <= 11.001,
					"The correction should use the hull beam rather than a long world-axis envelope.");
		}
	}

	@Test
	void surfaceShipAndSubmarineInShallowWaterStayInsideBothPhysicalBounds() {
		var terrain = flat(-10);
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, null, Vec3.ZERO, 0, Color.RED, 1000);
		var submarine = sub(1, new Vec3(0, 0, -5), 0, 0);
		var subs = List.of(ship, submarine);

		SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);

		assertEquals(0, ship.z(), 1e-12, "The surface-locked ship must remain at its waterline.");
		assertAllClear(subs, terrain);
		assertStationaryAndUndamaged(subs);
	}

	@Test
	void threeRotatedPitchedHullsHaveNoResidualPairOverlap() {
		var terrain = flat(-300);
		for (double heading : new double[] {0, 0.7, Math.PI / 2}) {
			for (double pitch : new double[] {0, 0.18, -0.18}) {
				var right = new Vec3(Math.cos(heading), -Math.sin(heading), 0);
				var origin = new Vec3(0, 0, -200);
				var subs = List.of(sub(0, origin, heading, pitch), sub(1, origin.add(right.scale(10)), heading, pitch),
						sub(2, origin.add(right.scale(20)), heading, pitch));

				SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);

				assertAllClear(subs, terrain);
				assertStationaryAndUndamaged(subs);
			}
		}
	}

	@Test
	void movingThreeHullContactAppliesEachImpactOnlyOnce() {
		var a = sub(0, new Vec3(0, 0, -200), 0, 0);
		var b = sub(1, new Vec3(10, 0, -200), 0, 0);
		var c = sub(2, new Vec3(20, 0, -200), 0, 0);
		a.setSwaySpeed(2);
		double mass = a.vehicleConfig().dryMass();

		SimulationLoop.checkSubCollisions(List.of(a, b, c));

		assertAllClear(List.of(a, b, c), null);
		// Equal masses and restitution 0.1: the first contact gives A=0.9 and B=1.1 m/s;
		// the second gives B=0.495 and C=0.605. Damage is 5 times normal closing speed squared.
		assertEquals(980, a.hp(), "Repeated projections must not add damage.");
		assertEquals(974, b.hp());
		assertEquals(994, c.hp());
		assertEquals(0.9, a.swaySpeed(), 1e-12, "Repeated projections must not add impulses.");
		assertEquals(0.495, b.swaySpeed(), 1e-12);
		assertEquals(0.605, c.swaySpeed(), 1e-12);
		double momentum = List.of(a, b, c).stream().mapToDouble(sub -> mass * sub.swaySpeed()).sum();
		double energy = List.of(a, b, c).stream()
				.mapToDouble(sub -> 0.5 * mass * sub.velocity().linear().lengthSquared()).sum();
		assertEquals(2 * mass, momentum, 1e-6);
		assertTrue(energy < 0.5 * mass * 4, "The contact chain must dissipate kinetic energy.");
	}

	@Test
	void tiltedHullsRemainClearWhenHorizontalCorrectionMovesOntoHigherTerrain() {
		double[] heights = {-50, -40, -30, -50, -40, -30, -50, -40, -30};
		var terrain = new TerrainMap(heights, 3, 3, -100, -100, 100);
		var a = sub(0, new Vec3(0, 0, -39), 0.4, 0.1);
		var b = sub(1, new Vec3(9, -4, -39), 0.4, 0.1);
		var subs = List.of(a, b);
		for (var sub : subs) {
			sub.setZ(sub.z() + HullGeometry.terrainPenetration(sub.pose().position(), sub.heading(), sub.pitch(),
					sub.vehicleConfig(), terrain));
		}
		assertTrue(HullOverlap.overlaps(a, b), "The slope fixture must begin with touching hulls.");

		SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);

		assertAllClear(subs, terrain);
		assertStationaryAndUndamaged(subs);
	}

	private static SubmarineEntity sub(int id, Vec3 position, double heading, double pitch) {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), id, null, position, heading, Color.RED, 1000);
		sub.setPitch(pitch);
		return sub;
	}

	private static TerrainMap flat(double elevation) {
		double[] heights = new double[9];
		Arrays.fill(heights, elevation);
		return new TerrainMap(heights, 3, 3, -1000, -1000, 1000);
	}

	private static void assertAllClear(List<SubmarineEntity> subs, TerrainMap terrain) {
		for (int i = 0; i < subs.size(); i++) {
			var sub = subs.get(i);
			assertTrue(sub.z() <= 0, "Contact correction must respect the water surface.");
			if (terrain != null) {
				assertEquals(0,
						HullGeometry.terrainPenetration(sub.pose().position(), sub.heading(), sub.pitch(),
								sub.vehicleConfig(), terrain),
						1e-9, "Contact correction must respect physical terrain.");
			}
			for (int j = i + 1; j < subs.size(); j++) {
				assertFalse(HullOverlap.overlaps(sub, subs.get(j)), "Every pair must be separated in the final pose.");
			}
		}
	}

	private static void assertStationaryAndUndamaged(List<SubmarineEntity> subs) {
		for (var sub : subs) {
			assertEquals(1000, sub.hp(), "Positional overlap is not a physical impact.");
			assertEquals(Vec3.ZERO, sub.velocity().linear(), "Positional correction must not create velocity.");
			assertEquals(0, sub.yawRate(), 1e-12);
			assertEquals(0, sub.pitchRate(), 1e-12);
		}
	}
}
