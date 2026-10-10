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
		}
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
		Vec3 f = forward(heading, pitch), r = right(heading), u = up(heading, pitch);
		double along = HullGeometry.SEMI_LENGTH * f.dot(normal);
		double across = HullGeometry.SEMI_BEAM * r.dot(normal);
		double above = HullGeometry.SEMI_HEIGHT * u.dot(normal);
		double scale = Math.sqrt(along * along + across * across + above * above);
		return f.scale(HullGeometry.SEMI_LENGTH * along / scale).add(r.scale(HullGeometry.SEMI_BEAM * across / scale))
				.add(u.scale(HullGeometry.SEMI_HEIGHT * above / scale));
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
