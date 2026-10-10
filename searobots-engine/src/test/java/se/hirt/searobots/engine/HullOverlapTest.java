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
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HullOverlapTest {

	private static final double CONTACT_STEP = 1e-5;

	@Test
	void parallelSideBySideHullsOverlapWithoutContainingCenterlineSamples() {
		assertOverlap(true, centeredSub(Vec3.ZERO, 0, 0), centeredSub(new Vec3(8, 0, 0), 0, 0));
	}

	@Test
	void parallelOffsetHullsOverlapWithoutContainingCenterlineSamples() {
		// Identical ellipsoids intersect iff their center separation lies in the ellipsoid with doubled axes.
		// (8 / 11)^2 + (30 / 76)^2 = 0.6847, but none of the old center/bow/stern samples is inside.
		assertOverlap(true, centeredSub(Vec3.ZERO, 0, 0), centeredSub(new Vec3(8, 30, 0), 0, 0));
	}

	@Test
	void parallelHullsOverlapVerticallyWithoutContainingCenterlineSamples() {
		assertOverlap(true, centeredSub(Vec3.ZERO, 0, 0), centeredSub(new Vec3(0, 0, 8), 0, 0));
	}

	@Test
	void crossingHullBodiesOverlapEvenWhenEveryCenterlineSampleMisses() {
		// The point (0, 18, 0) lies strictly inside both hulls, near the middle of each long axis.
		assertOverlap(true, positionedSub(Vec3.ZERO, 0, 0), positionedSub(new Vec3(20, 18, 0), Math.PI / 2, 0));
	}

	@Test
	void contactAndSeparationAreCorrectAlongEveryPrincipalAxis() {
		Vec3[] touchingOffsets = {new Vec3(2 * HullGeometry.SEMI_BEAM, 0, 0),
				new Vec3(0, 2 * HullGeometry.SEMI_LENGTH, 0), new Vec3(0, 0, 2 * HullGeometry.SEMI_HEIGHT)};
		for (Vec3 offset : touchingOffsets) {
			var a = centeredSub(Vec3.ZERO, 0, 0);
			Vec3 step = offset.normalize().scale(CONTACT_STEP);
			assertOverlap(true, a, centeredSub(offset.subtract(step), 0, 0));
			assertOverlap(true, a, centeredSub(offset, 0, 0));
			assertOverlap(false, a, centeredSub(offset.add(step), 0, 0));
		}
	}

	@Test
	void overlappingBoundingSpheresDoNotImplyHullOverlap() {
		var a = centeredSub(Vec3.ZERO, 0, 0);
		assertOverlap(false, a, centeredSub(new Vec3(12, 0, 0), 0, 0));
		assertOverlap(false, a, centeredSub(new Vec3(0, 0, 10), 0, 0));
		// This diagonal is also outside the doubled ellipsoid despite a center distance below 76m.
		assertOverlap(false, a, centeredSub(new Vec3(10, 50, 0), 0, 0));
	}

	@Test
	void coincidentHullCentersOverlapWithDifferentOrientations() {
		Vec3 center = new Vec3(120, -350, -200);
		assertOverlap(true, centeredSub(center, 0.4, -0.3), centeredSub(center, 2.1, 0.6));
	}

	@Test
	void oppositeHeadingsAccountForTheAftCenterOffset() {
		// Pose separation includes both semi-lengths and the opposing local forward offsets.
		double touchingSeparation = 2 * (HullGeometry.SEMI_LENGTH + HullGeometry.AFT_OFFSET);
		var a = positionedSub(Vec3.ZERO, 0, 0);
		assertOverlap(true, a, positionedSub(new Vec3(0, touchingSeparation - CONTACT_STEP, 0), Math.PI, 0));
		assertOverlap(true, a, positionedSub(new Vec3(0, touchingSeparation, 0), Math.PI, 0));
		assertOverlap(false, a, positionedSub(new Vec3(0, touchingSeparation + CONTACT_STEP, 0), Math.PI, 0));
	}

	@Test
	void parallelAnalyticalOverlapIsInvariantUnderHeadingPitchAndTranslation() {
		double[][] orientations = {{0, 0}, {1.7, -0.4}, {4.9, 0.7}};
		Vec3[] translations = {Vec3.ZERO, new Vec3(6500, -5000, -600)};
		for (double[] orientation : orientations) {
			double heading = orientation[0], pitch = orientation[1];
			for (Vec3 center : translations) {
				var a = centeredSub(center, heading, pitch);
				// Analytic doubled-ellipsoid values are 0.7341 (inside) and 1.0316 (outside).
				Vec3 overlappingOffset = localOffset(8, 30, 2, heading, pitch);
				Vec3 separatedOffset = localOffset(10, 30, 2, heading, pitch);
				assertOverlap(true, a, centeredSub(center.add(overlappingOffset), heading, pitch));
				assertOverlap(false, a, centeredSub(center.add(separatedOffset), heading, pitch));
			}
		}
	}

	@Test
	void differentlyOrientedHullsHaveCorrectSupportPlaneContact() {
		double[][] poses = {{0, 0, Math.PI / 2, 0.3}, {0.4, -0.6, 2.2, 0.5}, {5.8, 0.7, 1.1, -0.4}};
		Vec3[] normals = {new Vec3(1, 2, 3).normalize(), new Vec3(-2, 1, 0.4).normalize(),
				new Vec3(0.1, -0.3, 1).normalize()};
		Vec3 center = new Vec3(300, -400, -250);
		for (int i = 0; i < poses.length; i++) {
			double[] pose = poses[i];
			Vec3 normal = normals[i];
			var a = centeredSub(center, pose[0], pose[1]);
			// Matching support points meet on a common plane with normal n; each hull stays on its own side.
			Vec3 tangentCenter = center.add(supportPoint(normal, pose[0], pose[1]))
					.add(supportPoint(normal, pose[2], pose[3]));
			Vec3 step = normal.scale(CONTACT_STEP);
			assertOverlap(true, a, centeredSub(tangentCenter.subtract(step), pose[2], pose[3]));
			assertOverlap(true, a, centeredSub(tangentCenter, pose[2], pose[3]));
			assertOverlap(false, a, centeredSub(tangentCenter.add(step), pose[2], pose[3]));
			var contact = HullOverlap.contact(a, centeredSub(tangentCenter, pose[2], pose[3]));
			assertNotNull(contact);
			assertVector(normal, contact.normal(), 1e-8);
			assertVector(center.add(supportPoint(normal, pose[0], pose[1])), contact.pointA(), 1e-8);
			assertVector(contact.pointA(), contact.pointB(), 1e-8);
			assertEquals(0, contact.penetration(), 1e-8);
		}
	}

	@Test
	void grazingContactNormalFollowsHullSurfaceInsteadOfCenterLine() {
		Vec3 offset = new Vec3(
				2 * HullGeometry.SEMI_BEAM * Math.sqrt(1 - Math.pow(30 / (2 * HullGeometry.SEMI_LENGTH), 2)), 30, 0);
		var a = centeredSub(Vec3.ZERO, 0, 0);
		var b = centeredSub(offset, 0, 0);
		var contact = HullOverlap.contact(a, b);
		assertNotNull(contact);
		var expectedNormal = new Vec3(offset.x() / Math.pow(HullGeometry.SEMI_BEAM, 2),
				offset.y() / Math.pow(HullGeometry.SEMI_LENGTH, 2), 0).normalize();
		assertVector(expectedNormal, contact.normal(), 1e-10);
		assertTrue(contact.normal().dot(offset.normalize()) < 0.4, "A grazing surface normal differs from centre line");
		assertEquals(0, contact.penetration(), 1e-10);
	}

	@Test
	void contactGeometryReversesWhenHullsAreSwapped() {
		var a = centeredSub(new Vec3(300, -400, -200), 0.4, -0.3);
		var b = centeredSub(new Vec3(307, -395, -197), 1.3, 0.2);
		var forward = HullOverlap.contact(a, b);
		var reverse = HullOverlap.contact(b, a);
		assertNotNull(forward);
		assertNotNull(reverse);
		assertVector(forward.normal().scale(-1), reverse.normal(), 1e-10);
		assertVector(forward.pointA(), reverse.pointB(), 1e-8);
		assertVector(forward.pointB(), reverse.pointA(), 1e-8);
		assertEquals(forward.penetration(), reverse.penetration(), 1e-8);
	}

	@Test
	void supportPlanePenetrationSeparatesDeepAndCoincidentOverlaps() {
		Vec3 center = new Vec3(0, 0, -200);
		Vec3[] offsets = {Vec3.ZERO, new Vec3(8, 30, 0), new Vec3(20, 18, 2)};
		for (Vec3 offset : offsets) {
			var a = centeredSub(center, 0, 0);
			var b = centeredSub(center.add(offset), 1.4, 0.3);
			var contact = HullOverlap.contact(a, b);
			assertNotNull(contact);
			assertEquals(1.0, contact.normal().length(), 1e-10);
			assertTrue(Double.isFinite(contact.penetration()) && contact.penetration() > 0);
			var reverse = HullOverlap.contact(b, a);
			assertNotNull(reverse);
			assertVector(contact.normal().scale(-1), reverse.normal(), 1e-10);
			var separation = contact.normal().scale((contact.penetration() + CONTACT_STEP) * 0.5);
			assertNull(
					HullOverlap.contact(centeredSub(center.subtract(separation), 0, 0),
							centeredSub(center.add(offset).add(separation), 1.4, 0.3)),
					"Support planes must separate hulls");
		}
	}

	@Test
	void coincidentSurfaceLockedHullsSeparateHorizontally() {
		var a = new SubmarineEntity(VehicleConfig.surfaceShip(), 1, (input, output) -> {
		}, Vec3.ZERO, 0, Color.RED, 1000);
		var b = new SubmarineEntity(VehicleConfig.surfaceShip(), 2, (input, output) -> {
		}, Vec3.ZERO, 0, Color.BLUE, 1000);
		var contact = HullOverlap.contact(a, b);
		assertNotNull(contact);
		assertEquals(0, contact.normal().z(), 0);
		assertEquals(2 * HullGeometry.envelope(VehicleConfig.surfaceShip()).semiBeam(), contact.penetration(), 1e-10);
		var reverse = HullOverlap.contact(b, a);
		assertNotNull(reverse);
		assertVector(contact.normal().scale(-1), reverse.normal(), 1e-10);
	}

	@Test
	void coincidentMixedSurfacePairHasHorizontalNormalForEitherIdentityOrder() {
		for (int shipId = 0; shipId <= 1; shipId++) {
			var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), shipId, (input, output) -> {
			}, Vec3.ZERO, 0, Color.RED, 1000);
			var envelope = HullGeometry.envelope(VehicleConfig.surfaceShip());
			var commonCenter = envelope.worldPoint(Vec3.ZERO, Vec3.ZERO, 0, 0);
			var sub = new SubmarineEntity(VehicleConfig.submarine(), 1 - shipId, (input, output) -> {
			}, commonCenter.subtract(new Vec3(0, HullGeometry.AFT_OFFSET, HullGeometry.UP_OFFSET)), 0, Color.BLUE,
					1000);
			var contact = HullOverlap.contact(ship, sub);
			assertNotNull(contact);
			assertEquals(0, contact.normal().z(), 0);
			assertEquals(1, contact.normal().length(), 1e-10);
			assertEquals(envelope.semiBeam() + HullGeometry.SEMI_BEAM, contact.penetration(), 1e-10);
			var reverse = HullOverlap.contact(sub, ship);
			assertNotNull(reverse);
			assertVector(contact.normal().scale(-1), reverse.normal(), 1e-10);
		}
	}

	@Test
	void shipBowCanMeetASubmarineBeyondTheFormerBroadPhaseRadius() {
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, (input, output) -> {
		}, Vec3.ZERO, 0, Color.RED, 1000);
		var envelope = HullGeometry.envelope(ship.vehicleConfig());
		var center = envelope.worldPoint(Vec3.ZERO, Vec3.ZERO, 0, 0);
		double separation = envelope.semiLength() + HullGeometry.SEMI_LENGTH;
		assertOverlap(true, ship, centeredSub(center.add(new Vec3(0, separation - CONTACT_STEP, 0)), 0, 0));
		assertOverlap(true, ship, centeredSub(center.add(new Vec3(0, separation, 0)), 0, 0));
		assertOverlap(false, ship, centeredSub(center.add(new Vec3(0, separation + CONTACT_STEP, 0)), 0, 0));
	}

	@Test
	void mixedVehicleContactIsCorrectAcrossDifferentHeadingsAndPitch() {
		var shipEnvelope = HullGeometry.envelope(VehicleConfig.surfaceShip());
		var normal = new Vec3(1, 2, 0.3).normalize();
		Vec3 center = new Vec3(300, -400, -200);
		double shipHeading = 0.4, subHeading = 2.2, subPitch = -0.3;
		var shipPosition = center.subtract(forward(shipHeading, 0).scale(shipEnvelope.forwardOffset()))
				.subtract(up(shipHeading, 0).scale(shipEnvelope.upOffset()));
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, (input, output) -> {
		}, shipPosition, shipHeading, Color.RED, 1000);
		ship.setZ(shipPosition.z()); // Artificial translation for the support-plane geometry check.
		var touchingCenter = center.add(supportPoint(normal, shipHeading, 0, shipEnvelope))
				.add(supportPoint(normal, subHeading, subPitch));
		assertOverlap(true, ship,
				centeredSub(touchingCenter.subtract(normal.scale(CONTACT_STEP)), subHeading, subPitch));
		assertOverlap(true, ship, centeredSub(touchingCenter, subHeading, subPitch));
		assertOverlap(false, ship, centeredSub(touchingCenter.add(normal.scale(CONTACT_STEP)), subHeading, subPitch));
		var contact = HullOverlap.contact(ship, centeredSub(touchingCenter, subHeading, subPitch));
		assertNotNull(contact);
		assertVector(normal, contact.normal(), 1e-8);
		assertVector(contact.pointA(), contact.pointB(), 1e-8);
	}

	@Test
	void approachingSubmarineHitsShipForebodyBeyondTheFormerCollisionRadius() {
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, (input, output) -> {
		}, Vec3.ZERO, 0, Color.RED, 1000);
		var center = HullGeometry.envelope(ship.vehicleConfig()).worldPoint(Vec3.ZERO, Vec3.ZERO, 0, 0);
		var approaching = centeredSub(center.add(new Vec3(0, 125, 0)), Math.PI, 0);
		approaching.setSpeed(5);

		SimulationLoop.checkSubCollisions(List.of(ship, approaching));

		assertTrue(ship.hp() < 1000 && ship.hp() > 0, "The ship's forebody must receive the collision");
		assertEquals(ship.hp(), approaching.hp());
		assertTrue(ship.speed() < 0, "The ship must receive southward momentum from the incoming submarine");
		assertEquals(false, SimulationLoop.ellipsoidsOverlap(ship, approaching));
	}

	@Test
	void formerlyMissedOffsetCollisionAppliesRammingDamage() {
		var a = centeredSub(new Vec3(0, 0, -200), 0, 0);
		var b = centeredSub(new Vec3(8, 30, -200), 0, 0);
		a.setSpeed(5);

		SimulationLoop.checkSubCollisions(List.of(a, b));

		assertTrue(a.hp() > 0 && a.hp() < 1000, "The offset impact should damage the moving submarine");
		assertTrue(b.hp() > 0 && b.hp() < 1000, "The offset impact should damage the stationary submarine");
		assertEquals(a.hp(), b.hp(), "Both hulls should receive collision damage");
	}

	private static void assertOverlap(boolean expected, SubmarineEntity a, SubmarineEntity b) {
		assertEquals(expected, SimulationLoop.ellipsoidsOverlap(a, b), "A against B");
		assertEquals(expected, SimulationLoop.ellipsoidsOverlap(b, a), "B against A");
	}

	private static void assertVector(Vec3 expected, Vec3 actual, double tolerance) {
		assertEquals(expected.x(), actual.x(), tolerance);
		assertEquals(expected.y(), actual.y(), tolerance);
		assertEquals(expected.z(), actual.z(), tolerance);
	}

	private static SubmarineEntity positionedSub(Vec3 position, double heading, double pitch) {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, (input, output) -> {
		}, position, heading, Color.RED, 1000);
		sub.setPitch(pitch);
		return sub;
	}

	private static SubmarineEntity centeredSub(Vec3 center, double heading, double pitch) {
		Vec3 position = center.subtract(forward(heading, pitch).scale(HullGeometry.AFT_OFFSET))
				.subtract(up(heading, pitch).scale(HullGeometry.UP_OFFSET));
		return positionedSub(position, heading, pitch);
	}

	private static Vec3 localOffset(double right, double forward, double up, double heading, double pitch) {
		return right(heading).scale(right).add(forward(heading, pitch).scale(forward))
				.add(up(heading, pitch).scale(up));
	}

	private static Vec3 supportPoint(Vec3 normal, double heading, double pitch) {
		return supportPoint(normal, heading, pitch, HullGeometry.envelope(VehicleConfig.submarine()));
	}

	private static Vec3 supportPoint(Vec3 normal, double heading, double pitch, HullGeometry.Envelope envelope) {
		Vec3 f = forward(heading, pitch), r = right(heading), u = up(heading, pitch);
		double along = envelope.semiLength() * f.dot(normal);
		double across = envelope.semiBeam() * r.dot(normal);
		double above = envelope.semiHeight() * u.dot(normal);
		double scale = Math.sqrt(along * along + across * across + above * above);
		return f.scale(envelope.semiLength() * along / scale).add(r.scale(envelope.semiBeam() * across / scale))
				.add(u.scale(envelope.semiHeight() * above / scale));
	}

	private static Vec3 forward(double heading, double pitch) {
		return new Vec3(Math.sin(heading) * Math.cos(pitch), Math.cos(heading) * Math.cos(pitch), Math.sin(pitch));
	}

	private static Vec3 right(double heading) {
		return new Vec3(Math.cos(heading), -Math.sin(heading), 0);
	}

	private static Vec3 up(double heading, double pitch) {
		return new Vec3(-Math.sin(heading) * Math.sin(pitch), -Math.cos(heading) * Math.sin(pitch), Math.cos(pitch));
	}
}
