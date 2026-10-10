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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static se.hirt.searobots.engine.HullGeometry.AFT_OFFSET;
import static se.hirt.searobots.engine.HullGeometry.SEMI_BEAM;
import static se.hirt.searobots.engine.HullGeometry.SEMI_HEIGHT;
import static se.hirt.searobots.engine.HullGeometry.SEMI_LENGTH;
import static se.hirt.searobots.engine.HullGeometry.UP_OFFSET;

class HullGeometryTest {

	private static final double EPSILON = 1e-10;

	@Test
	void principalAxisDistancesIncludeAftOffset() {
		double distance = 7.25;
		assertEquals(distance, distance(new Vec3(SEMI_LENGTH + distance, 0, 0)), EPSILON);
		assertEquals(distance, distance(new Vec3(-SEMI_LENGTH - distance, 0, 0)), EPSILON);
		assertEquals(distance, distance(new Vec3(0, SEMI_BEAM + distance, 0)), EPSILON);
		assertEquals(distance, distance(new Vec3(0, -SEMI_BEAM - distance, 0)), EPSILON);
		assertEquals(distance, distance(new Vec3(0, 0, SEMI_HEIGHT + distance)), EPSILON);
		assertEquals(distance, distance(new Vec3(0, 0, -SEMI_HEIGHT - distance)), EPSILON);
	}

	@Test
	void pointsInsideAndOnSurfaceHaveZeroDistance() {
		assertEquals(0, distance(Vec3.ZERO), EPSILON);
		assertEquals(0, distance(new Vec3(SEMI_LENGTH * 0.9, 0, 0)), EPSILON);
		assertEquals(0, distance(new Vec3(0, SEMI_BEAM * 0.9, 0)), EPSILON);
		assertEquals(0, distance(new Vec3(0, 0, SEMI_HEIGHT * 0.9)), EPSILON);
		assertEquals(0, distance(new Vec3(SEMI_LENGTH * 0.6, SEMI_BEAM * 0.48, SEMI_HEIGHT * 0.64)), EPSILON);
	}

	@Test
	void obliqueApproachUsesNearestSurfaceRatherThanRadialIntersection() {
		var surface = new Vec3(SEMI_LENGTH * Math.cos(0.3), SEMI_BEAM * Math.sin(0.3), 0);
		var point = outsideAlongNormal(surface, 3.33);

		// The old radial intersection reports about 8.83 m for this point, missing a 4 m fuse.
		assertEquals(3.33, distance(point), EPSILON);
	}

	@Test
	void outwardNormalsGiveKnownDistancesFromNearSurfaceToFarAway() {
		var surfaces = new Vec3[] {new Vec3(SEMI_LENGTH * 0.6, SEMI_BEAM * 0.48, SEMI_HEIGHT * 0.64),
				new Vec3(-SEMI_LENGTH * 0.36, -SEMI_BEAM * 0.48, SEMI_HEIGHT * 0.8),
				new Vec3(0, SEMI_BEAM * 0.6, -SEMI_HEIGHT * 0.8)};
		for (var surface : surfaces) {
			for (double expected : new double[] {1e-7, 0.25, 3.33, 100, 1e6}) {
				assertEquals(expected, distance(outsideAlongNormal(surface, expected)),
						Math.max(EPSILON, expected * 1e-12), "Point offset along an ellipsoid's outward normal");
			}
		}
	}

	@Test
	void distanceIsInvariantUnderTranslationHeadingAndPitch() {
		var surface = new Vec3(SEMI_LENGTH * 0.6, SEMI_BEAM * 0.48, SEMI_HEIGHT * 0.64);
		var localPoint = outsideAlongNormal(surface, 3.33);
		var position = new Vec3(1250, -870, -200);
		for (double heading : new double[] {0, Math.PI / 2, 1.1, -2.4}) {
			for (double pitch : new double[] {0, -0.6, 0.45, Math.PI / 2}) {
				var point = worldPoint(localPoint, position, heading, pitch);
				assertEquals(3.33, HullGeometry.distanceToHull(point.x(), point.y(), point.z(), position.x(),
						position.y(), position.z(), heading, pitch), EPSILON);
			}
		}
	}

	@Test
	void bowAndEntityConvenienceMethodsRespectBothPitches() {
		double heading = 0.8;
		double pitch = 0.4;
		double expectedBowDistance = 3.33;
		double halfLength = VehicleConfig.torpedo().hullHalfLength();
		var position = new Vec3(1250, -870, -200);
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, (input, output) -> {
		}, position, heading, Color.BLUE, 1000);
		sub.setPitch(pitch);
		// The torpedo approaches along the submarine's negative up direction.
		var torpedoPosition = worldPoint(new Vec3(0, 0, SEMI_HEIGHT + expectedBowDistance + halfLength), position,
				heading, pitch);
		double torpedoPitch = pitch - Math.PI / 2;
		var torpedo = new TorpedoEntity(1, 1, VehicleConfig.torpedo(), null, torpedoPosition, heading, torpedoPitch, 4,
				Color.RED);

		assertEquals(expectedBowDistance + halfLength, HullGeometry.distanceToHull(torpedo, sub), EPSILON);
		assertEquals(expectedBowDistance,
				HullGeometry.bowDistanceToHull(torpedoPosition.x(), torpedoPosition.y(), torpedoPosition.z(), heading,
						torpedoPitch, halfLength, position.x(), position.y(), position.z(), heading, pitch),
				EPSILON);
		assertEquals(expectedBowDistance, HullGeometry.bowDistanceToHull(torpedo.snapshot(), sub.snapshot()), EPSILON);
	}

	private static Vec3 outsideAlongNormal(Vec3 surface, double distance) {
		var normal = new Vec3(surface.x() / (SEMI_LENGTH * SEMI_LENGTH), surface.y() / (SEMI_BEAM * SEMI_BEAM),
				surface.z() / (SEMI_HEIGHT * SEMI_HEIGHT)).normalize();
		return surface.add(normal.scale(distance));
	}

	private static double distance(Vec3 localPoint) {
		var point = worldPoint(localPoint, Vec3.ZERO, 0, 0);
		return HullGeometry.distanceToHull(point.x(), point.y(), point.z(), 0, 0, 0, 0, 0);
	}

	private static Vec3 worldPoint(Vec3 localPoint, Vec3 position, double heading, double pitch) {
		double sinH = Math.sin(heading), cosH = Math.cos(heading);
		double sinP = Math.sin(pitch), cosP = Math.cos(pitch);
		var forward = new Vec3(sinH * cosP, cosH * cosP, sinP);
		var right = new Vec3(cosH, -sinH, 0);
		var up = new Vec3(-sinH * sinP, -cosH * sinP, cosP);
		return position.add(forward.scale(AFT_OFFSET + localPoint.x())).add(right.scale(localPoint.y()))
				.add(up.scale(UP_OFFSET + localPoint.z()));
	}
}
