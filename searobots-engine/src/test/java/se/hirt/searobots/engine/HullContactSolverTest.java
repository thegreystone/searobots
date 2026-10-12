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
import java.util.ArrayList;
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
	void movingContactWithGroundedHullDoesNotLeaveEitherContactApproaching() {
		var terrain = flat(-20);
		var lower = sub(0, new Vec3(0, 0, -15), 0, 0);
		var upper = sub(1, new Vec3(0, 0, -7), 0, 0);
		upper.setVerticalSpeed(-2);
		var subs = List.of(lower, upper);
		var contacts = contacts(subs);

		SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);

		assertAllClear(subs, terrain);
		assertSeparating(contacts);
		assertEquals(0, lower.velocity().linear().z(), 1e-8, "The seabed reaction must join the hull velocity solve.");
		assertTrue(upper.velocity().linear().z() >= 0,
				"The upper hull must leave both the grounded hull and seabed separating.");
		int lowerHp = lower.hp(), upperHp = upper.hp();
		var physics = new SubmarinePhysics();
		var area = MatchConfig.withDefaults(42).battleArea();
		for (int tick = 0; tick < 20; tick++) {
			for (var sub : subs) {
				physics.step(sub, 0.02, terrain, NO_CURRENT, area);
			}
			SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);
		}
		assertEquals(lowerHp, lower.hp(), "A resolved grounding must not produce another impact next tick.");
		assertEquals(upperHp, upper.hp());
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
		// Simultaneous compression shares momentum through all three hulls. Restitution
		// leaves A separating from B, and B and C moving together. The transmitted
		// loads produce equivalent isolated closing speeds of 8/3 and 4/3 m/s.
		assertEquals(965, a.hp(), "Each accumulated contact load should be counted once.");
		assertEquals(957, b.hp());
		assertEquals(992, c.hp());
		assertEquals(8.0 / 15, a.swaySpeed(), 1e-9);
		assertEquals(11.0 / 15, b.swaySpeed(), 1e-9);
		assertEquals(11.0 / 15, c.swaySpeed(), 1e-9);
		double momentum = List.of(a, b, c).stream().mapToDouble(sub -> mass * sub.swaySpeed()).sum();
		double energy = List.of(a, b, c).stream()
				.mapToDouble(sub -> 0.5 * mass * sub.velocity().linear().lengthSquared()).sum();
		assertEquals(2 * mass, momentum, 1e-6);
		assertTrue(energy < 0.5 * mass * 4, "The contact chain must dissipate kinetic energy.");
		SimulationLoop.checkSubCollisions(List.of(a, b, c));
		assertEquals(965, a.hp(), "Resolved contacts must not cause damage on another pass.");
		assertEquals(957, b.hp());
		assertEquals(992, c.hp());
	}

	@Test
	void simultaneousContactIsIndependentOfEntityIterationOrder() {
		double[] expected = null;
		for (int[] order : new int[][] {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}) {
			var a = sub(0, new Vec3(0, 0, -200), 0, 0);
			var b = sub(1, new Vec3(10, 0, -200), 0, 0);
			var c = sub(2, new Vec3(20, 0, -200), 0, 0);
			a.setSwaySpeed(2);
			var byId = List.of(a, b, c);

			SimulationLoop.checkSubCollisions(List.of(byId.get(order[0]), byId.get(order[1]), byId.get(order[2])));

			var outcome = new double[] {a.x(), b.x(), c.x(), a.swaySpeed(), b.swaySpeed(), c.swaySpeed(), a.hp(),
					b.hp(), c.hp()};
			if (expected == null) {
				expected = outcome;
			} else {
				assertArrayEquals(expected, outcome, 1e-12, "Changing list order must preserve the complete outcome.");
			}
		}
	}

	@Test
	void contactCreatedByPositionCorrectionJoinsTheSameImpact() {
		var a = sub(0, new Vec3(0, 0, -200), 0, 0);
		var b = sub(1, new Vec3(10, 0, -200), 0, 0);
		var c = sub(2, new Vec3(21.2, 0, -200), 0, 0);
		a.setSwaySpeed(2);
		var subs = List.of(a, b, c);
		assertTrue(HullOverlap.overlaps(a, b));
		assertFalse(HullOverlap.overlaps(b, c), "B/C initially have a gap that A/B correction will close.");
		var incomingMomentum = momentum(subs);
		double incomingEnergy = energy(subs);

		SimulationLoop.checkSubCollisions(subs);

		assertAllClear(subs, null);
		assertTrue(a.swaySpeed() <= b.swaySpeed() + 1e-8);
		assertTrue(b.swaySpeed() <= c.swaySpeed() + 1e-8,
				"The newly created B/C contact must leave nonapproaching in this tick.");
		assertEquals(incomingMomentum.x(), momentum(subs).x(), 1e-6);
		assertTrue(energy(subs) < incomingEnergy, "The coupled impact must dissipate energy.");
		assertTrue(c.hp() < 1000, "The transmitted load belongs to the original impact.");
		int[] resolvedHp = subs.stream().mapToInt(SubmarineEntity::hp).toArray();
		var physics = new SubmarinePhysics();
		var terrain = flat(-300);
		var area = MatchConfig.withDefaults(42).battleArea();
		for (var sub : subs) {
			physics.step(sub, 0.02, terrain, NO_CURRENT, area);
		}
		SimulationLoop.checkSubCollisions(subs, NO_CURRENT, terrain);

		assertAllClear(subs, terrain);
		assertArrayEquals(resolvedHp, subs.stream().mapToInt(SubmarineEntity::hp).toArray(),
				"A contact resolved by positional projection must not manufacture another impact next tick.");
	}

	@Test
	void multipleHullContactsLeaveNoInwardContactVelocityAcrossAnglesAndSpeeds() {
		for (double heading : new double[] {0, 0.7, Math.PI / 2}) {
			for (double pitch : new double[] {0, 0.18, -0.18}) {
				for (double speed : new double[] {0.2, 2, 8}) {
					var right = new Vec3(Math.cos(heading), -Math.sin(heading), 0);
					var origin = new Vec3(0, 0, -200);
					var subs = List.of(sub(0, origin, heading, pitch),
							sub(1, origin.add(right.scale(10)), heading, pitch),
							sub(2, origin.add(right.scale(20)), heading, pitch));
					subs.getFirst().setSwaySpeed(speed);
					var contacts = contacts(subs);
					var momentum = momentum(subs);
					double energy = energy(subs);

					SimulationLoop.checkSubCollisions(subs);

					assertSeparating(contacts);
					assertAllClear(subs, null);
					assertEquals(momentum.x(), momentum(subs).x(), 1e-6);
					assertEquals(momentum.y(), momentum(subs).y(), 1e-6);
					assertEquals(momentum.z(), momentum(subs).z(), 1e-6);
					assertTrue(energy(subs) <= energy + 1e-6, "Simultaneous contact must not create energy.");
				}
			}
		}
	}

	@Test
	void restitutionCannotReactivateAnInitiallySeparatingContact() {
		var a = sub(0, new Vec3(0, 0, -200), 0, 0);
		var b = sub(1, new Vec3(10, 0, -200), 0, 0);
		var c = sub(2, new Vec3(20, 0, -200), 0, 0);
		a.setSwaySpeed(2);
		c.setSwaySpeed(0.5);
		var subs = List.of(a, b, c);
		var contacts = contacts(subs);
		var momentum = momentum(subs);
		double energy = energy(subs);

		SimulationLoop.checkSubCollisions(subs);

		assertSeparating(contacts);
		assertEquals(momentum.x(), momentum(subs).x(), 1e-6);
		assertTrue(energy(subs) < energy);
		assertTrue(c.hp() < 1000, "A contact closed by the other hull's load still transmits impact damage.");
	}

	@Test
	void longerContactChainConvergesWithoutMomentumOrEnergyGain() {
		var subs = new ArrayList<SubmarineEntity>();
		for (int i = 0; i < 5; i++) {
			subs.add(sub(i, new Vec3(i * 10, 0, -200), 0, 0));
		}
		subs.getFirst().setSwaySpeed(5);
		var contacts = contacts(subs);
		var momentum = momentum(subs);
		double energy = energy(subs);

		SimulationLoop.checkSubCollisions(subs);

		assertSeparating(contacts);
		assertAllClear(subs, null);
		assertEquals(momentum.x(), momentum(subs).x(), 1e-6);
		assertTrue(energy(subs) < energy);
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

	private record ContactVelocity(SubmarineEntity a, SubmarineEntity b, Vec3 offsetA, Vec3 offsetB, Vec3 normal) {
	}

	private static List<ContactVelocity> contacts(List<SubmarineEntity> subs) {
		var contacts = new ArrayList<ContactVelocity>();
		for (int i = 0; i < subs.size(); i++) {
			for (int j = i + 1; j < subs.size(); j++) {
				var a = subs.get(i);
				var b = subs.get(j);
				var contact = HullOverlap.contact(a, b);
				if (contact != null) {
					var point = contact.pointA().add(contact.pointB()).scale(0.5);
					contacts.add(new ContactVelocity(a, b, point.subtract(a.pose().position()),
							point.subtract(b.pose().position()), contact.normal()));
				}
			}
		}
		return contacts;
	}

	private static void assertSeparating(List<ContactVelocity> contacts) {
		for (var contact : contacts) {
			double closing = SubCollisionResponse.contactVelocity(contact.a(), contact.offsetA(), NO_CURRENT)
					.subtract(SubCollisionResponse.contactVelocity(contact.b(), contact.offsetB(), NO_CURRENT))
					.dot(contact.normal());
			assertTrue(closing <= 1e-8, "Every simultaneous contact must leave separating, closing=" + closing);
		}
	}

	private static Vec3 momentum(List<SubmarineEntity> subs) {
		var result = Vec3.ZERO;
		for (var sub : subs) {
			result = result.add(sub.velocity().linear().scale(sub.vehicleConfig().dryMass()));
		}
		return result;
	}

	private static double energy(List<SubmarineEntity> subs) {
		double result = 0;
		for (var sub : subs) {
			var cfg = sub.vehicleConfig();
			double cosP = Math.cos(sub.pitch()), sinP = Math.sin(sub.pitch());
			double yawInertia = cfg.collisionYawInertia() * cosP * cosP + cfg.collisionRollInertia() * sinP * sinP;
			result += 0.5 * cfg.dryMass() * sub.velocity().linear().lengthSquared()
					+ 0.5 * yawInertia * sub.yawRate() * sub.yawRate()
					+ 0.5 * cfg.collisionPitchInertia() * sub.pitchRate() * sub.pitchRate();
		}
		return result;
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
