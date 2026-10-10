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

import java.awt.Color;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TorpedoTerrainCollisionTest {

	private static final double DT = 1.0 / 50;
	private static final double FLOOR = -100;
	private static final TerrainMap FLAT = terrain(0, 0);

	@Test
	void steepDiveDetonatesWhenTheHullTouchesBeforeTheCentre() {
		var torpedo = torpedo(-97.5, -Math.PI / 3, 5, 0);
		var physics = new TorpedoPhysics();
		for (int tick = 0; tick < 10 && torpedo.alive(); tick++) {
			physics.step(torpedo, DT, FLAT, null, null);
		}
		assertTrue(torpedo.detonated());
		assertFalse(torpedo.alive());
		var cfg = torpedo.vehicleConfig();
		double verticalExtent = Math.hypot(cfg.hullHalfLength() * Math.sin(torpedo.pitch()),
				cfg.hullHalfBeam() * Math.cos(torpedo.pitch()));
		assertEquals(FLOOR, torpedo.z() - verticalExtent, 1e-6,
				"The explosion pose must stop at the first oriented-hull contact");
		assertTrue(torpedo.z() > FLOOR, "The nose must trigger impact before the centre reaches the floor");
	}

	@Test
	void aNoseAlreadyBelowTheFloorDetonatesOnItsFirstFreeSwimmingTick() {
		var torpedo = torpedo(-98.8, -Math.PI / 3, 5, 0);
		assertFalse(torpedo.inTube());
		assertTrue(torpedo.z() + Math.sin(torpedo.pitch()) * torpedo.vehicleConfig().hullHalfLength() < FLOOR);

		new TorpedoPhysics().step(torpedo, DT, FLAT, null, null);

		assertTrue(torpedo.detonated());
		assertFalse(torpedo.alive());
	}

	@Test
	void pitchedUpSternContactIsDetectedEvenWhenTheNoseIsClear() {
		var torpedo = torpedo(-98.8, Math.PI / 3, 5, 0);
		double halfLength = torpedo.vehicleConfig().hullHalfLength();
		assertTrue(torpedo.z() + Math.sin(torpedo.pitch()) * halfLength > FLOOR);
		assertTrue(torpedo.z() - Math.sin(torpedo.pitch()) * halfLength < FLOOR);

		new TorpedoPhysics().step(torpedo, DT, FLAT, null, null);

		assertTrue(torpedo.detonated(), "The tail must not pass through terrain when the torpedo pitches up");
	}

	@Test
	void levelHullScrapeIsDetectedWhileItsCentreIsStillAboveTheFloor() {
		var torpedo = torpedo(-99.8, 0, 5, 0);
		assertTrue(torpedo.z() > FLOOR);
		assertTrue(torpedo.z() - torpedo.vehicleConfig().hullHalfBeam() < FLOOR);

		new TorpedoPhysics().step(torpedo, DT, FLAT, null, null);

		assertTrue(torpedo.detonated(), "The keel must trigger a glancing floor contact");
	}

	@Test
	void verticalSinkingStopsAtFirstKeelContact() {
		var torpedo = torpedo(-99.7, 0, 0, 0);
		torpedo.setVerticalSpeed(-3);

		new TorpedoPhysics().step(torpedo, DT, FLAT, null, null);

		assertTrue(torpedo.detonated());
		assertEquals(FLOOR + torpedo.vehicleConfig().hullHalfBeam(), torpedo.z(), 1e-6);
	}

	@Test
	void obliqueSlopeContactBetweenAxisExtremaIsDetected() {
		var slope = terrain(0.8, 0);
		var torpedo = torpedo(-99.71, 0, 5, 0);
		double radius = torpedo.vehicleConfig().hullHalfBeam();
		assertTrue(torpedo.z() - radius > slope.elevationAt(0, 0));
		assertTrue(torpedo.z() > slope.elevationAt(radius, 0),
				"The fixture must keep both the keel and side-axis sample clear");

		new TorpedoPhysics().step(torpedo, DT, slope, null, null);

		assertTrue(torpedo.detonated(),
				"The ellipsoid extremity along the terrain normal must detect the contact between samples");
	}

	@Test
	void slopeContactDependsOnHullOrientation() {
		var slope = terrain(0, 0.9);
		var towardSlope = torpedo(-97.8, 0, 5, 0);
		var acrossSlope = torpedo(-97.8, 0, 5, Math.PI / 2);
		var physics = new TorpedoPhysics();

		physics.step(towardSlope, DT, slope, null, null);
		physics.step(acrossSlope, DT, slope, null, null);

		assertTrue(towardSlope.detonated(), "The bow must contact terrain uphill of the centre");
		assertTrue(acrossSlope.alive(), "A sideways hull with ample radial clearance must stay alive");
	}

	@Test
	void hullWithRequiredClearanceRemainsFreeSwimming() {
		var torpedo = torpedo(-98.6, 0, 5, 0);
		var physics = new TorpedoPhysics();
		for (int tick = 0; tick < 20; tick++) {
			physics.step(torpedo, DT, FLAT, null, null);
		}
		assertTrue(torpedo.alive());
		assertFalse(torpedo.detonated());
		assertTrue(torpedo.y() > 1);
	}

	@Test
	void hullInsideNavigationClearanceRemainsFreeSwimmingUntilPhysicalContact() {
		var torpedo = torpedo(-99.25, 0, 5, 0);
		var cfg = torpedo.vehicleConfig();
		double physicalGap = torpedo.z() - cfg.hullHalfBeam() - FLOOR;
		assertTrue(physicalGap > 0 && physicalGap < cfg.terrainClearance(),
				"The hull must start inside the navigation margin while remaining clear of the seabed");

		var physics = new TorpedoPhysics();
		for (int tick = 0; tick < 20; tick++) {
			physics.step(torpedo, DT, FLAT, null, null);
		}

		assertTrue(torpedo.alive(), "Navigation clearance must not trigger a terrain detonation");
		assertFalse(torpedo.detonated());
		assertTrue(torpedo.y() > 1, "The torpedo must continue swimming inside the navigation margin");
		assertTrue(torpedo.z() - cfg.hullHalfBeam() > FLOOR);
	}

	@Test
	void crossingARidgeStopsAtContactEvenWhenBothTickEndpointsAreClear() {
		double[] depths = new double[21 * 21];
		java.util.Arrays.fill(depths, FLOOR);
		for (int row = 0; row < 21; row++) {
			depths[row * 21 + 10] = -95;
		}
		var ridge = new TerrainMap(depths, 21, 21, -10, -10, 1);
		var torpedo = torpedo(-97, 0, 23, Math.PI / 2);
		torpedo.setX(-6);

		new TorpedoPhysics().step(torpedo, 0.5, ridge, null, null);

		assertTrue(torpedo.detonated(), "An intervening ridge must not be skipped between clear endpoints");
		assertTrue(torpedo.x() > -6 && torpedo.x() < 0,
				"The explosion pose must remain on the approaching side of the ridge");
	}

	@Test
	void rotatingHullContactsARidgeBetweenClearEndpointOrientations() {
		double[] depths = new double[41 * 41];
		java.util.Arrays.fill(depths, FLOOR);
		for (int col = 0; col < 41; col++) {
			depths[30 * 41 + col] = -95; // A narrow ridge at y=2.5, beginning at y=2.25.
		}
		var ridge = new TerrainMap(depths, 41, 41, -5, -5, 0.25);
		var torpedo = torpedo(-97, 0, 0, 2 * Math.PI - Math.PI / 4);
		// Existing yaw momentum rotates the hull by 90 degrees while its centre stays still.
		torpedo.setYawRate(Math.PI / 2 * Math.exp(1.0 / 10));
		torpedo.setVerticalSpeed(TorpedoEntity.sinkAcceleration());
		var initialPosition = torpedo.pose().position();
		var cfg = torpedo.vehicleConfig();
		double endpointNorthExtent = Math.hypot(cfg.hullHalfLength() * Math.cos(Math.PI / 4),
				cfg.hullHalfBeam() * Math.sin(Math.PI / 4));
		assertTrue(endpointNorthExtent < 2.25, "Both endpoint orientations must lie entirely south of the ridge");

		new TorpedoPhysics().step(torpedo, 1, ridge, null, null);

		assertTrue(torpedo.detonated(), "Rotating hull tips must not tunnel through intervening terrain");
		assertEquals(initialPosition, torpedo.pose().position(), "This fixture must have no centre translation");
		double signedHeading = Math.atan2(Math.sin(torpedo.heading()), Math.cos(torpedo.heading()));
		assertTrue(signedHeading > -Math.PI / 4 && signedHeading < 0,
				"Contact must stop the rotation before the bow points across the ridge");
	}

	@Test
	void physicsDoesNotApplyFreeSwimmingCollisionChecksInsideATube() {
		var owner = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, -100), 0, Color.BLUE, 1000);
		var torpedo = torpedo(-98.8, -Math.PI / 3, 5, 0);
		torpedo.loadIntoTube(owner, TorpedoTubes.TUBES.getFirst(), DT, "");
		var position = torpedo.pose().position();

		new TorpedoPhysics().step(torpedo, DT, FLAT, null, null);

		assertTrue(torpedo.inTube());
		assertTrue(torpedo.alive());
		assertFalse(torpedo.detonated());
		assertEquals(position, torpedo.pose().position());
	}

	private static TorpedoEntity torpedo(double z, double pitch, double speed, double heading) {
		var cfg = VehicleConfig.torpedo();
		var torpedo = new TorpedoEntity(9000, 0, cfg, null, new Vec3(0, 0, z), heading, pitch, 20, Color.RED);
		double throttle = cfg.dragCoeff() * speed * speed / cfg.maxThrust();
		torpedo.setSpeed(speed);
		torpedo.setActualThrottle(throttle);
		torpedo.createOutput().setThrottle(throttle);
		return torpedo;
	}

	private static TerrainMap terrain(double slopeX, double slopeY) {
		double[] depths = new double[21 * 21];
		for (int row = 0; row < 21; row++) {
			for (int col = 0; col < 21; col++) {
				depths[row * 21 + col] = FLOOR + slopeX * (col - 10) + slopeY * (row - 10);
			}
		}
		return new TerrainMap(depths, 21, 21, -10, -10, 1);
	}
}
