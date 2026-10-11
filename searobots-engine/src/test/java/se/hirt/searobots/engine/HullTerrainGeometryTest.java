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
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.util.Arrays;
import java.util.function.DoubleBinaryOperator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HullTerrainGeometryTest {

	@Test
	void flatFloorKeepsTheEstablishedPhysicalKeelAndShipDraft() {
		var floor = terrain((x, y) -> -100, 10);
		assertEquals(0, HullGeometry.terrainPenetration(new Vec3(0, 0, -95), 0, 0, VehicleConfig.submarine(), floor),
				1e-10);
		assertEquals(0.25,
				HullGeometry.terrainPenetration(new Vec3(0, 0, -95.25), 0, 0, VehicleConfig.submarine(), floor), 1e-10);
		assertEquals(0,
				HullGeometry.terrainPenetration(new Vec3(0, 0, -90.5), 0, 0, VehicleConfig.surfaceShip(), floor),
				1e-10);
	}

	@Test
	void slopedFloorUsesTheContinuousEllipsoidNormalSupport() {
		double slopeX = 0.2, slopeY = 0.1;
		var floor = terrain((x, y) -> -100 + slopeX * x + slopeY * y, 10);
		double expected = Math.sqrt(Math.pow(HullGeometry.SEMI_BEAM * slopeX, 2)
				+ Math.pow(HullGeometry.SEMI_LENGTH * slopeY, 2) + Math.pow(HullGeometry.SEMI_HEIGHT, 2))
				- HullGeometry.UP_OFFSET;
		assertEquals(expected,
				HullGeometry.terrainPenetration(new Vec3(0, 0, -100), 0, 0, VehicleConfig.submarine(), floor), 1e-9);
	}

	@Test
	void latticeRidgeBetweenTheFormerHullSamplesIsDetectedAfterTranslationAndRotation() {
		for (double heading : new double[] {0, Math.PI / 2}) {
			var floor = terrain((x, y) -> (heading == 0 ? y : x) == 20 ? -98 : -120, 10);
			var position = heading == 0 ? new Vec3(2, 0.1, -100) : new Vec3(0.1, 2, -100);
			double penetration = HullGeometry.terrainPenetration(position, heading, 0, VehicleConfig.submarine(),
					floor);
			assertTrue(penetration > 5, "A 10 m terrain crest inside the body must not fall between contact points");
			assertEquals(0, HullGeometry.terrainPenetration(position.add(new Vec3(0, 0, penetration)), heading, 0,
					VehicleConfig.submarine(), floor), 1e-8);
		}
	}

	@Test
	void obliquePitchedHullClearsACrestBetweenLowerSurfaceSeeds() {
		var floor = terrain((x, y) -> y == 20 ? -109 : -125, 10);
		var position = new Vec3(6, 6, -110);
		double heading = 0.37, pitch = Math.toRadians(-45);
		double penetration = HullGeometry.terrainPenetration(position, heading, pitch, VehicleConfig.submarine(),
				floor);
		// A referenced Body vertex from the current OBJ, missed by the half-grid seed query.
		var vertex = new Vec3(1.540739, 25.19474, -2.977329);
		var world = HullTerrainGeometry.worldPoint(vertex, position.add(new Vec3(0, 0, penetration)), heading, pitch);
		assertTrue(world.z() >= floor.elevationAt(world.x(), world.y()) - 1e-6,
				"The inclined forebody must clear the terrain crest after contact projection");
	}

	@Test
	void bilinearCellInteriorCannotHideAnInclinedForebodyContact() {
		var floor = new TerrainMap(new double[] {-129, -121, -87, -139}, 2, 2, 10, 10, 10);
		var position = new Vec3(4, 0, -100);
		double heading = 0.7, pitch = -0.55;
		// Referenced Body vertex from submarine-hybrid.obj. Neither the former support
		// iteration nor the exact terrain-grid edge supports reached this cell interior.
		var vertex = new Vec3(-4.266573, 21.711071, -0.356215);
		var embedded = HullTerrainGeometry.worldPoint(vertex, position, heading, pitch);
		assertTrue(floor.elevationAt(embedded.x(), embedded.y()) - embedded.z() > 2.77);

		double penetration = HullGeometry.terrainPenetration(position, heading, pitch, VehicleConfig.submarine(),
				floor);
		assertTrue(penetration > 2.77, "A real body contact inside a bilinear cell must project the hull clear");
		var corrected = HullTerrainGeometry.worldPoint(vertex, position.add(new Vec3(0, 0, penetration)), heading,
				pitch);
		assertTrue(corrected.z() >= floor.elevationAt(corrected.x(), corrected.y()) - 1e-8);
	}

	@Test
	void standaloneEllipsoidKeepsClearWaterBroadphaseAndItsOwnDimensions() {
		var floor = terrain((x, y) -> -100, 10);
		var torpedo = new HullGeometry.Envelope(2.5, 0.25, 0.25, 0, 0);
		assertEquals(0,
				HullTerrainGeometry.ellipsoidContactPoints(new Vec3(0, 0, -90), 0.7, -0.55, torpedo, floor).length);
		var position = new Vec3(0, 0, -100);
		double lowest = Double.POSITIVE_INFINITY;
		for (var local : HullTerrainGeometry.ellipsoidContactPoints(position, 0, 0, torpedo, floor)) {
			lowest = Math.min(lowest, HullTerrainGeometry.worldPoint(local, position, 0, 0).z());
		}
		assertEquals(-100.25, lowest, 1e-10, "The generic ellipsoid must not acquire submarine appendages or draft");
	}

	@Test
	void symmetricBilinearSaddleFindsBothDegenerateInteriorMaxima() {
		var sphere = new HullGeometry.Envelope(1, 1, 1, 0, 0);
		var position = new Vec3(0, 0, -100);
		for (double[] heights : new double[][] {{-98, -102, -102, -98}, {-102, -98, -98, -102}}) {
			var floor = new TerrainMap(heights, 2, 2, -1, -1, 2);
			double penetration = 0;
			int maxima = 0;
			for (var local : HullTerrainGeometry.ellipsoidContactPoints(position, 0, 0, sphere, floor)) {
				var world = HullTerrainGeometry.worldPoint(local, position, 0, 0);
				double candidate = floor.elevationAt(world.x(), world.y()) - world.z();
				penetration = Math.max(penetration, candidate);
				if (Math.abs(candidate - 1.25) < 1e-9) {
					maxima++;
				}
			}
			// Maximize +/-2xy-z on x^2+y^2+z^2=1: z=-1/2, xy=+/-3/8.
			assertEquals(1.25, penetration, 1e-9);
			assertEquals(2, maxima, "The eigenvalue hard case has two distinct physical interior maxima");
		}
	}

	@Test
	void contactGeometryIgnoresNavigationMargin() {
		var floor = terrain((x, y) -> y == 20 ? -98 : -120, 10);
		var config = VehicleConfig.submarine();
		var largerMargin = new VehicleConfig(config.dryMass(), config.addedMassSurge(), config.addedMassSway(),
				config.addedMassHeave(), config.maxThrust(), config.reverseThrustFactor(), config.maxReverseSpeed(),
				config.dragCoeff(), config.swayDragCoeff(), config.hullMomentArm(), config.rudderArea(),
				config.rudderArm(), config.planesArea(), config.planesArm(), config.stallAngle(),
				config.rotationalInertia(), config.ballastSlewRate(), config.ballastForceMax(),
				config.verticalDragCoeff(), config.terrainClearance() + 100, config.hullHalfLength(),
				config.hullHalfBeam(), config.collisionDamageFactor(), config.bounceSpeed(), config.propDragFactor(),
				config.baseSlDb(), config.clutchDisengagedSlReduction(), config.speedNoiseDbPerMs(),
				config.baseCavitationSpeed(), config.cavitationDepthFactor(), config.cavitationMaxDb(),
				config.reverseCavitationDb(), config.surfaceNoiseDepth(), config.surfaceNoiseDb(),
				config.ballastNoiseDb(), config.thrustSlewRate(), config.sonarSelfNoiseOffsetDb(),
				config.surfaceLocked(), config.hasBallast());
		assertTrue(Arrays.equals(HullGeometry.terrainContactPoints(new Vec3(0, 0, -100), 0, 0, config, floor),
				HullGeometry.terrainContactPoints(new Vec3(0, 0, -100), 0, 0, largerMargin, floor)));
	}

	@Test
	void tinyFineMapCrestIsCoveredWithoutSamplingTheInfiniteGridOutsideTheMap() {
		int side = 21;
		double[] values = new double[side * side];
		Arrays.fill(values, -120);
		values[10 * side + 10] = -98;
		var floor = new TerrainMap(values, side, side, 0, 0, 0.001);
		var position = new Vec3(0, 0, -100);
		var contacts = HullGeometry.terrainContactPoints(position, 0, 0, VehicleConfig.submarine(), floor);
		assertTrue(contacts.length < 12000, "Contact work must follow the finite map, rather than off-map gridlines");
		assertTrue(HullGeometry.terrainPenetration(position, 0, 0, VehicleConfig.submarine(), floor) > 6,
				"A tiny map's real lattice crest must survive the contact sampling budget");
	}

	private static TerrainMap terrain(DoubleBinaryOperator elevation, double cell) {
		int count = (int) (200 / cell) + 1;
		double[] values = new double[count * count];
		for (int row = 0; row < count; row++) {
			for (int col = 0; col < count; col++) {
				values[row * count + col] = elevation.applyAsDouble(-100 + col * cell, -100 + row * cell);
			}
		}
		return new TerrainMap(values, count, count, -100, -100, cell);
	}
}
