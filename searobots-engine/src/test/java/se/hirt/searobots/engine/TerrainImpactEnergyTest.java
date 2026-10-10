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

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TerrainImpactEnergyTest {

	private static final VehicleConfig SUBMARINE = VehicleConfig.submarine();
	private static final Vec3 FORWARD = new Vec3(0, 1, 0);
	private static final Vec3 RIGHT = new Vec3(1, 0, 0);
	private static final Vec3 UP = new Vec3(0, 0, 1);

	@Test
	void stationaryGrazingAndSeparatingNormalMotionReleaseNoImpactEnergy() {
		var contact = new Vec3(0, 33.5, 0);
		assertAll(() -> assertEquals(0, energy(SUBMARINE, contact, UP, 0)),
				() -> assertEquals(0, energy(SUBMARINE, contact, UP, -8)));
		assertEquals(0, TerrainImpactEnergy.damage(0, SUBMARINE.collisionDamageFactor()));
	}

	@Test
	void centredImpactUsesDryMassPlusTheDirectionalAddedMass() {
		// For a centre contact there is no angular lever. E = 1/2 * directional mass * v^2.
		assertAll(() -> assertRelative(7_500_000, energy(SUBMARINE, Vec3.ZERO, FORWARD, 2)),
				() -> assertRelative(12_500_000, energy(SUBMARINE, Vec3.ZERO, RIGHT, 2)),
				() -> assertRelative(11_250_000, energy(SUBMARINE, Vec3.ZERO, UP, 2)));
	}

	@Test
	void doublingNormalSpeedQuadruplesImpactEnergy() {
		var contact = new Vec3(3, 30, -4);
		var normal = new Vec3(-1, -2, 3).normalize();
		double slow = energy(SUBMARINE, contact, normal, 3);

		assertRelative(4 * slow, energy(SUBMARINE, contact, normal, 6));
	}

	@Test
	void scalingDryAndAllAddedMassesScalesImpactEnergy() {
		var heavier = changedConfig(2 * SUBMARINE.dryMass(), 2 * SUBMARINE.addedMassSurge(),
				2 * SUBMARINE.addedMassSway(), 2 * SUBMARINE.addedMassHeave(), SUBMARINE.hullHalfLength(),
				SUBMARINE.hullHalfBeam(), false);
		var contact = new Vec3(3, 30, -4);
		var normal = new Vec3(-1, -2, 3).normalize();
		double original = TerrainImpactEnergy.energyJoules(SUBMARINE, contact, normal, 0.6, 0.2, 4);

		assertRelative(2 * original, TerrainImpactEnergy.energyJoules(heavier, contact, normal, 0.6, 0.2, 4));
		assertRelative(2 * SUBMARINE.collisionPitchInertia(), heavier.collisionPitchInertia());
		assertRelative(2 * SUBMARINE.collisionYawInertia(), heavier.collisionYawInertia());
	}

	@Test
	void increasedAddedSurgeMassAffectsForwardCentreImpact() {
		var modified = changedMasses(SUBMARINE.dryMass(), 2 * SUBMARINE.addedMassSurge(), SUBMARINE.addedMassSway(),
				SUBMARINE.addedMassHeave());

		assertAll(() -> assertRelative(10_000_000, energy(modified, Vec3.ZERO, FORWARD, 2)),
				() -> assertRelative(12_500_000, energy(modified, Vec3.ZERO, RIGHT, 2)),
				() -> assertRelative(11_250_000, energy(modified, Vec3.ZERO, UP, 2)));
	}

	@Test
	void increasedAddedSwayMassAffectsLateralCentreImpact() {
		var modified = changedMasses(SUBMARINE.dryMass(), SUBMARINE.addedMassSurge(), 2 * SUBMARINE.addedMassSway(),
				SUBMARINE.addedMassHeave());

		assertAll(() -> assertRelative(7_500_000, energy(modified, Vec3.ZERO, FORWARD, 2)),
				() -> assertRelative(20_000_000, energy(modified, Vec3.ZERO, RIGHT, 2)),
				() -> assertRelative(11_250_000, energy(modified, Vec3.ZERO, UP, 2)));
	}

	@Test
	void increasedAddedHeaveMassAffectsVerticalCentreImpact() {
		var modified = changedMasses(SUBMARINE.dryMass(), SUBMARINE.addedMassSurge(), SUBMARINE.addedMassSway(),
				2 * SUBMARINE.addedMassHeave());

		assertAll(() -> assertRelative(7_500_000, energy(modified, Vec3.ZERO, FORWARD, 2)),
				() -> assertRelative(12_500_000, energy(modified, Vec3.ZERO, RIGHT, 2)),
				() -> assertRelative(17_500_000, energy(modified, Vec3.ZERO, UP, 2)));
	}

	@Test
	void increasedDryMassAffectsAllTranslationalDirections() {
		var modified = changedMasses(2 * SUBMARINE.dryMass(), SUBMARINE.addedMassSurge(), SUBMARINE.addedMassSway(),
				SUBMARINE.addedMassHeave());

		assertAll(() -> assertRelative(12_500_000, energy(modified, Vec3.ZERO, FORWARD, 2)),
				() -> assertRelative(17_500_000, energy(modified, Vec3.ZERO, RIGHT, 2)),
				() -> assertRelative(16_250_000, energy(modified, Vec3.ZERO, UP, 2)));
	}

	@Test
	void collisionMomentsUsePhysicalDimensionsAndDirectionalMasses() {
		assertAll(() -> assertRelative(1_609_031_250, SUBMARINE.collisionPitchInertia()),
				() -> assertRelative(1_784_812_500, SUBMARINE.collisionYawInertia()),
				() -> assertRelative(85_500_000, SUBMARINE.collisionRollInertia()));
		var larger = changedConfig(SUBMARINE.dryMass(), SUBMARINE.addedMassSurge(), SUBMARINE.addedMassSway(),
				SUBMARINE.addedMassHeave(), 2 * SUBMARINE.hullHalfLength(), 2 * SUBMARINE.hullHalfBeam(), false);

		assertAll(() -> assertRelative(4 * SUBMARINE.collisionPitchInertia(), larger.collisionPitchInertia()),
				() -> assertRelative(4 * SUBMARINE.collisionYawInertia(), larger.collisionYawInertia()));
	}

	@Test
	void rotationOnlyPitchContactHasFiniteEnergyFromPitchInertia() {
		// At zero surge, +0.1 rad/s pitch drives the stern 40m aft downward at 4m/s.
		// Independent effective mass: 1 / (1 / 5,625,000 + 40^2 / 1,609,031,250).
		double value = energy(SUBMARINE, new Vec3(0, -40, 0), UP, 4);

		assertRelative(6_824_978.128893719, value);
		assertTrue(value < 0.5 * SUBMARINE.collisionPitchInertia() * 0.1 * 0.1,
				"One contact cannot release more than the incoming rotational kinetic energy.");
	}

	@Test
	void rotationOnlyYawContactHasFiniteEnergyFromYawInertia() {
		// At zero surge, +0.1 rad/s yaw drives the bow 33.5m ahead toward +X at 3.35m/s.
		double value = energy(SUBMARINE, new Vec3(0, 33.5, 0), new Vec3(-1, 0, 0), 3.35);

		assertRelative(7_113_856.274683553, value);
		assertTrue(value < 0.5 * SUBMARINE.collisionYawInertia() * 0.1 * 0.1,
				"One contact cannot release more than the incoming rotational kinetic energy.");
	}

	@Test
	void pitchedYawContactUsesTheMomentAboutWorldVertical() {
		var bow = new Vec3(0, 33.5 / Math.sqrt(2), 33.5 / Math.sqrt(2));
		// At 45 degrees, the world-Z moment is (1,784,812,500 + 85,500,000) / 2.
		double value = TerrainImpactEnergy.energyJoules(SUBMARINE, bow, RIGHT, 0, Math.PI / 4, 3.35 / Math.sqrt(2));
		assertRelative(3_691_449.530645445, value);
	}

	@Test
	void offcentreImpactAccountsForRotationInsteadOfChargingFullTranslationEnergy() {
		double centre = energy(SUBMARINE, Vec3.ZERO, UP, 4);
		double bow = energy(SUBMARINE, new Vec3(0, 20, 0), UP, 4);
		double furtherBow = energy(SUBMARINE, new Vec3(0, 40, 0), UP, 4);

		assertTrue(centre > bow && bow > furtherBow,
				"A longer free rotation lever reduces the effective contact mass.");
		assertRelative(6_824_978.128893719, furtherBow);
	}

	@Test
	void increasedPhysicalPitchAndYawInertiasIncreaseOffcentreImpactEnergy() {
		var larger = changedConfig(SUBMARINE.dryMass(), SUBMARINE.addedMassSurge(), SUBMARINE.addedMassSway(),
				SUBMARINE.addedMassHeave(), 2 * SUBMARINE.hullHalfLength(), 2 * SUBMARINE.hullHalfBeam(), false);
		var contact = new Vec3(3, 30, 0);

		assertTrue(energy(larger, contact, UP, 4) > energy(SUBMARINE, contact, UP, 4));
		assertTrue(energy(larger, contact, RIGHT, 4) > energy(SUBMARINE, contact, RIGHT, 4));
		assertRelative(energy(SUBMARINE, Vec3.ZERO, UP, 4), energy(larger, Vec3.ZERO, UP, 4));
	}

	@Test
	void pitchedForwardNormalStillUsesSurgeMass() {
		double heading = Math.toRadians(60);
		double pitch = Math.toRadians(25);
		var forward = new Vec3(Math.sin(heading) * Math.cos(pitch), Math.cos(heading) * Math.cos(pitch),
				Math.sin(pitch));

		assertRelative(7_500_000, TerrainImpactEnergy.energyJoules(SUBMARINE, Vec3.ZERO, forward, heading, pitch, 2));
	}

	@Test
	void worldRotationOfHullAndTerrainPreservesEnergy() {
		var contact = new Vec3(3, 30, -4);
		var normal = new Vec3(-1, -2, 3).normalize();
		double heading = Math.toRadians(60);
		double original = TerrainImpactEnergy.energyJoules(SUBMARINE, contact, normal, 0, 0.2, 4);
		double rotated = TerrainImpactEnergy.energyJoules(SUBMARINE, rotateHeading(contact, heading),
				rotateHeading(normal, heading), heading, 0.2, 4);

		assertRelative(original, rotated);
	}

	@Test
	void calibratedLowModerateAndHighSpeedImpactsHaveMeaningfulSeverity() {
		double factor = SUBMARINE.collisionDamageFactor();
		assertAll(() -> assertEquals(45, TerrainImpactEnergy.damage(energy(SUBMARINE, Vec3.ZERO, UP, 2), factor)),
				() -> assertEquals(720, TerrainImpactEnergy.damage(energy(SUBMARINE, Vec3.ZERO, UP, 8), factor)),
				() -> assertEquals(1687,
						TerrainImpactEnergy.damage(energy(SUBMARINE, Vec3.ZERO, FORWARD, 15), factor)));
		assertTrue(TerrainImpactEnergy.damage(energy(SUBMARINE, Vec3.ZERO, FORWARD, 15), factor) >= 1000,
				"A direct flank-speed impact must be lethal to the default 1000-HP submarine.");
	}

	@Test
	void damageUsesFixedReferenceMassSoHeavierVehiclesAreNotNormalizedAway() {
		var heavier = changedMasses(2 * SUBMARINE.dryMass(), 2 * SUBMARINE.addedMassSurge(),
				2 * SUBMARINE.addedMassSway(), 2 * SUBMARINE.addedMassHeave());

		assertEquals(90,
				TerrainImpactEnergy.damage(energy(heavier, Vec3.ZERO, UP, 2), heavier.collisionDamageFactor()));
	}

	@Test
	void surfaceLockedContactCannotRelieveImpactThroughHeaveOrPitch() {
		var locked = changedConfig(SUBMARINE.dryMass(), SUBMARINE.addedMassSurge(), SUBMARINE.addedMassSway(),
				SUBMARINE.addedMassHeave(), SUBMARINE.hullHalfLength(), SUBMARINE.hullHalfBeam(), true);
		var slopeNormal = new Vec3(0, -1, 1).normalize();
		var contact = new Vec3(0, 30, 0);

		assertEquals(0, energy(locked, Vec3.ZERO, UP, 4), "A fully locked normal has no legitimate inward motion.");
		assertRelative(7_500_000, energy(locked, Vec3.ZERO, FORWARD, 2));
		assertRelative(15_000_000, energy(locked, contact, slopeNormal, 2));
		assertTrue(energy(locked, contact, slopeNormal, 2) > energy(SUBMARINE, contact, slopeNormal, 2));
	}

	private static double energy(VehicleConfig config, Vec3 contact, Vec3 normal, double closingSpeed) {
		return TerrainImpactEnergy.energyJoules(config, contact, normal, 0, 0, closingSpeed);
	}

	private static Vec3 rotateHeading(Vec3 vector, double angle) {
		return new Vec3(Math.cos(angle) * vector.x() + Math.sin(angle) * vector.y(),
				-Math.sin(angle) * vector.x() + Math.cos(angle) * vector.y(), vector.z());
	}

	private static void assertRelative(double expected, double actual) {
		assertEquals(expected, actual, Math.max(1e-9, Math.abs(expected) * 1e-12));
	}

	private static VehicleConfig changedMasses(double dryMass, double addedSurge, double addedSway, double addedHeave) {
		return changedConfig(dryMass, addedSurge, addedSway, addedHeave, SUBMARINE.hullHalfLength(),
				SUBMARINE.hullHalfBeam(), false);
	}

	private static VehicleConfig changedConfig(
		double dryMass, double addedSurge, double addedSway, double addedHeave, double halfLength, double halfBeam,
		boolean surfaceLocked) {
		var c = SUBMARINE;
		return new VehicleConfig(dryMass, addedSurge, addedSway, addedHeave, c.maxThrust(), c.reverseThrustFactor(),
				c.maxReverseSpeed(), c.dragCoeff(), c.swayDragCoeff(), c.hullMomentArm(), c.rudderArea(), c.rudderArm(),
				c.planesArea(), c.planesArm(), c.stallAngle(), c.rotationalInertia(), c.ballastSlewRate(),
				c.ballastForceMax(), c.verticalDragCoeff(), c.terrainClearance(), halfLength, halfBeam,
				c.collisionDamageFactor(), c.bounceSpeed(), c.propDragFactor(), c.baseSlDb(),
				c.clutchDisengagedSlReduction(), c.speedNoiseDbPerMs(), c.baseCavitationSpeed(),
				c.cavitationDepthFactor(), c.cavitationMaxDb(), c.reverseCavitationDb(), c.surfaceNoiseDepth(),
				c.surfaceNoiseDb(), c.ballastNoiseDb(), c.thrustSlewRate(), c.sonarSelfNoiseOffsetDb(), surfaceLocked,
				c.hasBallast());
	}
}
