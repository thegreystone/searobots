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
import se.hirt.searobots.api.Vec2;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.*;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static se.hirt.searobots.api.VehicleConfig.submarine;

class SubCollisionTest {

	private static final CurrentField NO_CURRENT = new CurrentField(List.of());
	private static final double CONTACT_HALF_SEPARATION = 37.99;

	private SubmarineEntity makeSub(int id, Vec3 pos, double speed, double heading) {
		return makeSub(submarine(), id, pos, speed, heading);
	}

	private SubmarineEntity makeSub(VehicleConfig cfg, int id, Vec3 pos, double speed, double heading) {
		var controller = new DefaultAttackSub();
		var sub = new SubmarineEntity(cfg, id, controller, pos, heading, Color.RED, 1000);
		sub.setSpeed(speed);
		return sub;
	}

	@Test
	void fastGrazingContactPreservesTangentialMotionAndSurvives() {
		// The hulls touch at a common normal almost perpendicular to northward travel.
		// Centre-to-centre closing speed is 14.2 m/s, but normal closing speed is 0.93 m/s.
		var approaching = makeSub(0, new Vec3(0, 0, -200), 15, 0);
		var stationary = makeSub(1, new Vec3(10.106736460582, 30, -200), 0, 0);
		var contact = HullOverlap.contact(approaching, stationary);
		assertNotNull(contact, "The grazing regression fixture must touch");
		var tangent = new Vec3(-contact.normal().y(), contact.normal().x(), 0);
		var momentumBefore = totalMomentum(approaching, stationary);
		double energyBefore = kineticEnergy(approaching) + kineticEnergy(stationary);
		double tangentialSpeedBefore = approaching.velocity().linear().dot(tangent);

		SimulationLoop.checkSubCollisions(List.of(approaching, stationary));

		assertTrue(approaching.hp() >= 990 && approaching.hp() < 1000,
				"A fast grazing contact should cause minor damage, hp=" + approaching.hp());
		assertEquals(approaching.hp(), stationary.hp());
		assertTrue(approaching.speed() > 14, "A scrape must preserve most forward motion");
		assertTrue(stationary.velocity().linear().length() > 0, "The rammed hull should receive momentum");
		assertEquals(tangentialSpeedBefore, approaching.velocity().linear().dot(tangent), 1e-9);
		assertEquals(0, stationary.velocity().linear().dot(tangent), 1e-9);
		assertVectorEquals(momentumBefore, totalMomentum(approaching, stationary), 1e-6);
		assertTrue(kineticEnergy(approaching) + kineticEnergy(stationary) <= energyBefore + 1e-6,
				"The scrape must dissipate rather than create kinetic energy");
	}

	@Test
	void nearbyButSeparatedParallelHullsDoNotCauseDamage() {
		var approaching = makeSub(0, new Vec3(0, 0, -200), 2, 0);
		var stationary = makeSub(1, new Vec3(12, 30, -200), 0, 0);

		SimulationLoop.checkSubCollisions(List.of(approaching, stationary));

		assertEquals(1000, approaching.hp());
		assertEquals(1000, stationary.hp());
		assertEquals(2, approaching.speed(), 1e-12);
	}

	@Test
	void headOnRamIsMutuallyFatal() {
		// Two bows touch while heading toward each other at 10 m/s each.
		// Closing speed = 20 m/s → damage = 5 * 400 = 2000 → both dead
		var sub1 = makeSub(0, new Vec3(0, -CONTACT_HALF_SEPARATION, -200), 10, 0);
		var sub2 = makeSub(1, new Vec3(0, CONTACT_HALF_SEPARATION, -200), 10, Math.PI);

		SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

		assertTrue(sub1.hp() <= 0, "Sub1 should be dead from head-on ram, hp=" + sub1.hp());
		assertTrue(sub2.hp() <= 0, "Sub2 should be dead from head-on ram, hp=" + sub2.hp());
	}

	@Test
	void slowBowContactIsSurvivable() {
		var sub1 = makeSub(0, new Vec3(0, -CONTACT_HALF_SEPARATION, -200), 2, 0);
		var sub2 = makeSub(1, new Vec3(0, CONTACT_HALF_SEPARATION, -200), 0, 0);

		SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

		assertEquals(980, sub1.hp());
		assertEquals(980, sub2.hp());
	}

	@Test
	void bowCollisionTransfersMomentumAndDissipatesEnergy() {
		var ramming = makeSub(0, new Vec3(0, -CONTACT_HALF_SEPARATION, -200), 5, 0);
		var stationary = makeSub(1, new Vec3(0, CONTACT_HALF_SEPARATION, -200), 0, 0);
		var momentumBefore = totalMomentum(ramming, stationary);
		double energyBefore = kineticEnergy(ramming) + kineticEnergy(stationary);

		SimulationLoop.checkSubCollisions(List.of(ramming, stationary));

		assertEquals(875, ramming.hp());
		assertEquals(875, stationary.hp());
		assertTrue(ramming.speed() > 0 && ramming.speed() < 5, "The moving hull must slow down");
		assertTrue(stationary.speed() > ramming.speed(), "Both hulls must leave the contact separating");
		assertVectorEquals(momentumBefore, totalMomentum(ramming, stationary), 1e-6);
		assertTrue(kineticEnergy(ramming) + kineticEnergy(stationary) < energyBefore);
		assertEquals(0, ramming.yawRate(), 1e-12, "A centred bow impact should not impart yaw");
		assertEquals(0, stationary.yawRate(), 1e-12);
	}

	@Test
	void unequalMassCollisionConservesMomentumAndDoesNotGainEnergy() {
		var ramming = makeSub(0, new Vec3(0, -CONTACT_HALF_SEPARATION, -200), 5, 0);
		var heavier = makeSub(withDryMass(submarine(), 5_000_000), 1, new Vec3(0, CONTACT_HALF_SEPARATION, -200), 0, 0);
		var momentumBefore = totalMomentum(ramming, heavier);
		double energyBefore = kineticEnergy(ramming) + kineticEnergy(heavier);

		SimulationLoop.checkSubCollisions(List.of(ramming, heavier));

		assertTrue(heavier.speed() > 0 && heavier.speed() < 2.5,
				"The heavier hull should accelerate less than an equal-mass target");
		assertTrue(heavier.speed() > ramming.speed());
		assertVectorEquals(momentumBefore, totalMomentum(ramming, heavier), 1e-6);
		assertTrue(kineticEnergy(ramming) + kineticEnergy(heavier) < energyBefore);
	}

	@Test
	void rotationOnlyContactChangesAngularAndLinearMotion() {
		var rotating = makeSub(0, new Vec3(0, 0, -200), 0, 0.75);
		var stationary = makeSub(1, new Vec3(30, 0, -200), 0, 0);
		rotating.setYawRate(0.1);
		assertTrue(SimulationLoop.ellipsoidsOverlap(rotating, stationary));
		double energyBefore = kineticEnergy(rotating);

		SimulationLoop.checkSubCollisions(List.of(rotating, stationary));

		assertTrue(rotating.hp() < 1000, "A rotating bow must cause impact damage even with stationary centres");
		assertEquals(rotating.hp(), stationary.hp());
		assertTrue(Math.abs(rotating.yawRate()) < 0.1, "The impact should resist the rotation into the other hull");
		assertTrue(stationary.velocity().linear().length() > 0);
		assertTrue(Math.abs(stationary.yawRate()) > 0, "An off-centre impact should also rotate the other hull");
		assertVectorEquals(Vec3.ZERO, totalMomentum(rotating, stationary), 1e-6);
		assertTrue(kineticEnergy(rotating) + kineticEnergy(stationary) <= energyBefore + 1e-6);
		assertFalse(SimulationLoop.ellipsoidsOverlap(rotating, stationary), "The hulls must be separated");
	}

	@Test
	void pitchOnlyContactAppliesPitchImpulse() {
		double verticalSeparation = 9 * Math.sqrt(1 - Math.pow(30.0 / 76, 2)) - 0.01;
		var rotating = makeSub(0, new Vec3(0, 0, -200), 0, 0);
		var stationary = makeSub(1, new Vec3(0, 30, -200 + verticalSeparation), 0, 0);
		rotating.setPitchRate(0.1);
		assertTrue(SimulationLoop.ellipsoidsOverlap(rotating, stationary));
		double energyBefore = kineticEnergy(rotating);

		SimulationLoop.checkSubCollisions(List.of(rotating, stationary));

		assertTrue(rotating.hp() < 1000);
		assertTrue(Math.abs(rotating.pitchRate()) < 0.1);
		assertTrue(Math.abs(stationary.pitchRate()) > 0);
		assertTrue(stationary.velocity().linear().z() > 0, "The pitched bow must impart vertical velocity");
		assertVectorEquals(Vec3.ZERO, totalMomentum(rotating, stationary), 1e-6);
		assertTrue(kineticEnergy(rotating) + kineticEnergy(stationary) <= energyBefore + 1e-6);
	}

	@Test
	void pitchedGlancingCollisionConservesMomentumAndDissipatesEnergy() {
		double heading = 0.7, pitch = 0.35;
		var forward = new Vec3(Math.sin(heading) * Math.cos(pitch), Math.cos(heading) * Math.cos(pitch),
				Math.sin(pitch));
		var right = new Vec3(Math.cos(heading), -Math.sin(heading), 0);
		var origin = new Vec3(0, 0, -200);
		var approaching = makeSub(0, origin, 15, heading);
		var stationary = makeSub(1, origin.add(forward.scale(30)).add(right.scale(10.1067)), 0, heading);
		approaching.setPitch(pitch);
		stationary.setPitch(pitch);
		approaching.setYawRate(0.025);
		approaching.setPitchRate(-0.015);
		stationary.setYawRate(-0.005);
		stationary.setPitchRate(0.02);
		assertTrue(SimulationLoop.ellipsoidsOverlap(approaching, stationary));
		var momentumBefore = totalMomentum(approaching, stationary);
		double energyBefore = kineticEnergy(approaching) + kineticEnergy(stationary);

		SimulationLoop.checkSubCollisions(List.of(approaching, stationary));

		assertTrue(approaching.hp() < 1000, "This pitched fixture must approach at its contact point");
		assertVectorEquals(momentumBefore, totalMomentum(approaching, stationary), 1e-6);
		assertTrue(kineticEnergy(approaching) + kineticEnergy(stationary) < energyBefore,
				"A pitched impact must dissipate energy with the correct world-Z inertia");
		assertFalse(SimulationLoop.ellipsoidsOverlap(approaching, stationary));
	}

	@Test
	void surfaceLockedHullCannotAcquireVerticalOrPitchMotion() {
		var surface = makeSub(VehicleConfig.surfaceShip(), 0, new Vec3(0, 0, 0), 0, 0);
		var rising = makeSub(1, new Vec3(0, 0, -8.99), 0, 0);
		rising.setVerticalSpeed(2);

		SimulationLoop.checkSubCollisions(List.of(surface, rising));

		assertTrue(surface.hp() < 1000);
		assertEquals(0, surface.z(), 1e-12);
		assertEquals(0, surface.velocity().linear().z(), 1e-12);
		assertEquals(0, surface.pitchRate(), 1e-12);
		assertTrue(rising.velocity().linear().z() < 0, "The mobile hull must rebound from the surface-locked hull");
		assertFalse(SimulationLoop.ellipsoidsOverlap(surface, rising));
	}

	@Test
	void stationaryOverlapIsSeparatedWithoutInventingMotionOrDamage() {
		var sub1 = makeSub(0, new Vec3(0, 0, -200), 0, 0);
		var sub2 = makeSub(1, new Vec3(10.9, 0, -200), 0, 0);

		SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

		assertFalse(SimulationLoop.ellipsoidsOverlap(sub1, sub2));
		assertEquals(1000, sub1.hp());
		assertEquals(1000, sub2.hp());
		assertVectorEquals(Vec3.ZERO, sub1.velocity().linear(), 1e-12);
		assertVectorEquals(Vec3.ZERO, sub2.velocity().linear(), 1e-12);
	}

	@Test
	void overlapAtSurfaceMovesTheSubmergedHullFarEnoughToSeparate() {
		var surfaced = makeSub(0, new Vec3(0, 0, 0), 0, 0);
		var submerged = makeSub(1, new Vec3(0, 0, -4), 0, 0);

		SimulationLoop.checkSubCollisions(List.of(surfaced, submerged));

		assertEquals(0, surfaced.z(), 1e-12, "Separation must not push a vessel above the surface");
		assertTrue(submerged.z() <= -9, "The submerged hull must take the remaining separation");
		assertFalse(SimulationLoop.ellipsoidsOverlap(surfaced, submerged));
		assertEquals(1000, surfaced.hp());
		assertEquals(1000, submerged.hp());
		assertVectorEquals(Vec3.ZERO, surfaced.velocity().linear(), 1e-12);
		assertVectorEquals(Vec3.ZERO, submerged.velocity().linear(), 1e-12);
	}

	@Test
	void partialSurfaceClearanceRedistributesRemainingSeparation() {
		var shallow = makeSub(0, new Vec3(0, 0, -1), 0, 0);
		var deep = makeSub(1, new Vec3(0, 0, -5), 0, 0);

		SimulationLoop.checkSubCollisions(List.of(shallow, deep));

		assertTrue(shallow.z() <= 0 && deep.z() <= 0);
		assertTrue(deep.z() < -7.5,
				"The deeper hull must take separation left over after the shallow hull reaches water level");
		assertFalse(SimulationLoop.ellipsoidsOverlap(shallow, deep));
		assertEquals(1000, shallow.hp());
		assertEquals(1000, deep.hp());
		assertVectorEquals(Vec3.ZERO, shallow.velocity().linear(), 1e-12);
		assertVectorEquals(Vec3.ZERO, deep.velocity().linear(), 1e-12);
	}

	@Test
	void resolvedContactDoesNotApplyDamageAgain() {
		var sub1 = makeSub(0, new Vec3(0, -CONTACT_HALF_SEPARATION, -200), 5, 0);
		var sub2 = makeSub(1, new Vec3(0, CONTACT_HALF_SEPARATION, -200), 0, 0);
		SimulationLoop.checkSubCollisions(List.of(sub1, sub2));
		int hpAfterImpact = sub1.hp();
		var velocity1 = sub1.velocity();
		var velocity2 = sub2.velocity();

		for (int i = 0; i < 10; i++) {
			SimulationLoop.checkSubCollisions(List.of(sub1, sub2));
		}

		assertEquals(hpAfterImpact, sub1.hp());
		assertEquals(hpAfterImpact, sub2.hp());
		assertEquals(velocity1, sub1.velocity());
		assertEquals(velocity2, sub2.velocity());
	}

	@Test
	void commonCurrentDoesNotChangeCollisionResponse() {
		var still1 = makeSub(0, new Vec3(0, 0, -200), 15, 0);
		var still2 = makeSub(1, new Vec3(10.106736460582, 30, -200), 0, 0);
		var drifting1 = makeSub(0, new Vec3(0, 0, -200), 15, 0);
		var drifting2 = makeSub(1, new Vec3(10.106736460582, 30, -200), 0, 0);
		var current = new CurrentField(List.of(new CurrentField.CurrentBand(-500, 0, new Vec2(3, -4))));

		SimulationLoop.checkSubCollisions(List.of(still1, still2));
		SimulationLoop.checkSubCollisions(List.of(drifting1, drifting2), current);

		assertEquals(still1.hp(), drifting1.hp());
		assertEquals(still2.hp(), drifting2.hp());
		assertVectorEquals(still1.velocity().linear(), drifting1.velocity().linear(), 1e-12);
		assertVectorEquals(still2.velocity().linear(), drifting2.velocity().linear(), 1e-12);
		assertEquals(still1.yawRate(), drifting1.yawRate(), 1e-12);
		assertEquals(still2.yawRate(), drifting2.yawRate(), 1e-12);
	}

	@Test
	void currentShearCanCauseCollisionBetweenWaterStationaryHulls() {
		var drifting = makeSub(0, new Vec3(0, 0, -200), 0, 0);
		var stationary = makeSub(1, new Vec3(10.7, 0, -198), 0, 0);
		var current = new CurrentField(List.of(new CurrentField.CurrentBand(-300, -199, new Vec2(2, 0))));
		assertTrue(SimulationLoop.ellipsoidsOverlap(drifting, stationary));

		SimulationLoop.checkSubCollisions(List.of(drifting, stationary), current);

		assertTrue(drifting.hp() < 1000, "Ground-relative approach due to shear must register an impact");
		assertEquals(drifting.hp(), stationary.hp());
		assertTrue(drifting.velocity().linear().x() < 0, "The drifting hull should slow relative to the current");
		assertTrue(stationary.velocity().linear().x() > 0, "The other hull must receive the impulse");
		assertVectorEquals(Vec3.ZERO, totalMomentum(drifting, stationary), 1e-6);
	}

	@Test
	void lateralImpulsePersistsIntoNextPhysicsStep() {
		var moving = makeSub(0, new Vec3(0, 0, -200), 0, 0);
		var stationary = makeSub(1, new Vec3(10.9, 0, -200), 0, 0);
		moving.setSwaySpeed(2);
		SimulationLoop.checkSubCollisions(List.of(moving, stationary));
		assertTrue(stationary.swaySpeed() > 0, "A side impact must be represented in persistent lateral velocity");
		double lateralSpeed = stationary.velocity().linear().x();
		double xBefore = stationary.x();
		double[] floor = new double[9];
		java.util.Arrays.fill(floor, -500);
		var terrain = new TerrainMap(floor, 3, 3, -1000, -1000, 1000);
		var config = MatchConfig.withDefaults(42);
		double dt = 1.0 / config.tickRateHz();

		new SubmarinePhysics().step(stationary, dt, terrain, NO_CURRENT, config.battleArea());

		assertTrue(stationary.x() > xBefore);
		assertEquals(lateralSpeed * dt, stationary.x() - xBefore, 1e-5,
				"The next tick should integrate the collision's side velocity, with only minor water drag");
	}

	@Test
	void noCollisionWhenFarApart() {
		// Subs 100m apart, beyond the combined 76m hull length.
		var sub1 = makeSub(0, new Vec3(0, -50, -200), 10, 0);
		var sub2 = makeSub(1, new Vec3(0, 50, -200), 10, Math.PI);

		SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

		assertEquals(1000, sub1.hp(), "Sub1 should be undamaged");
		assertEquals(1000, sub2.hp(), "Sub2 should be undamaged");
	}

	@Test
	void separatingOverlapGetsSeparatedWithoutExtraImpulseOrDamage() {
		var sub1 = makeSub(0, new Vec3(0, 0, -200), 0, 0);
		var sub2 = makeSub(1, new Vec3(10.9, 0, -200), 0, 0);
		sub1.setSwaySpeed(-2);
		sub2.setSwaySpeed(2);
		var velocity1 = sub1.velocity();
		var velocity2 = sub2.velocity();

		SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

		assertEquals(1000, sub1.hp(), "Separating sub1 should be undamaged");
		assertEquals(1000, sub2.hp(), "Separating sub2 should be undamaged");
		assertEquals(velocity1, sub1.velocity());
		assertEquals(velocity2, sub2.velocity());
		assertFalse(SimulationLoop.ellipsoidsOverlap(sub1, sub2));
	}

	private static Vec3 totalMomentum(SubmarineEntity a, SubmarineEntity b) {
		return a.velocity().linear().scale(a.vehicleConfig().dryMass())
				.add(b.velocity().linear().scale(b.vehicleConfig().dryMass()));
	}

	private static double kineticEnergy(SubmarineEntity sub) {
		var cfg = sub.vehicleConfig();
		double cosP = Math.cos(sub.pitch()), sinP = Math.sin(sub.pitch());
		double yawInertia = cfg.collisionYawInertia() * cosP * cosP + cfg.collisionRollInertia() * sinP * sinP;
		return 0.5 * cfg.dryMass() * sub.velocity().linear().lengthSquared()
				+ 0.5 * yawInertia * sub.yawRate() * sub.yawRate()
				+ 0.5 * cfg.collisionPitchInertia() * sub.pitchRate() * sub.pitchRate();
	}

	private static void assertVectorEquals(Vec3 expected, Vec3 actual, double tolerance) {
		assertEquals(expected.x(), actual.x(), tolerance);
		assertEquals(expected.y(), actual.y(), tolerance);
		assertEquals(expected.z(), actual.z(), tolerance);
	}

	private static VehicleConfig withDryMass(VehicleConfig cfg, double mass) {
		return new VehicleConfig(mass, cfg.addedMassSurge(), cfg.addedMassSway(), cfg.addedMassHeave(), cfg.maxThrust(),
				cfg.reverseThrustFactor(), cfg.maxReverseSpeed(), cfg.dragCoeff(), cfg.swayDragCoeff(),
				cfg.hullMomentArm(), cfg.rudderArea(), cfg.rudderArm(), cfg.planesArea(), cfg.planesArm(),
				cfg.stallAngle(), cfg.rotationalInertia(), cfg.ballastSlewRate(), cfg.ballastForceMax(),
				cfg.verticalDragCoeff(), cfg.terrainClearance(), cfg.hullHalfLength(), cfg.hullHalfBeam(),
				cfg.collisionDamageFactor(), cfg.bounceSpeed(), cfg.propDragFactor(), cfg.baseSlDb(),
				cfg.clutchDisengagedSlReduction(), cfg.speedNoiseDbPerMs(), cfg.baseCavitationSpeed(),
				cfg.cavitationDepthFactor(), cfg.cavitationMaxDb(), cfg.reverseCavitationDb(), cfg.surfaceNoiseDepth(),
				cfg.surfaceNoiseDb(), cfg.ballastNoiseDb(), cfg.thrustSlewRate(), cfg.sonarSelfNoiseOffsetDb(),
				cfg.surfaceLocked(), cfg.hasBallast());
	}
}
