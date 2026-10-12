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
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.*;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class WreckPhysicsTest {

	private static final MatchConfig CONFIG = MatchConfig.withDefaults(17);
	private static final CurrentField NO_CURRENT = new CurrentField(List.of());

	@Test
	void livingSurfaceShipRemainsBuoyantAndSurfaceConstrained() {
		var ship = vehicle(VehicleConfig.surfaceShip(), 0, Vec3.ZERO);
		var physics = new SubmarinePhysics(CONFIG);
		var terrain = flat(-100);
		for (int tick = 0; tick < 100; tick++) {
			physics.step(ship, 0.1, terrain, NO_CURRENT, CONFIG.battleArea());
		}
		assertTrue(ship.surfaceConstrained());
		assertEquals(0, ship.z(), 1e-12);
		assertEquals(0, ship.verticalSpeed(), 1e-12);
		assertEquals(0, ship.wreckFloodFraction(), 1e-12);
		assertEquals(1000, ship.hp());
	}

	@Test
	void destroyedShipFloodsGraduallyAndSinksWithBoundedSpeed() {
		var ship = vehicle(VehicleConfig.surfaceShip(), 0, Vec3.ZERO);
		ship.setHp(0);
		var physics = new SubmarinePhysics(CONFIG);
		var terrain = flat(-200);
		physics.step(ship, 0.1, terrain, NO_CURRENT, CONFIG.battleArea());
		assertTrue(ship.z() < 0 && ship.z() > -0.001, "Flooding starts gently rather than dropping the wreck.");
		assertTrue(ship.verticalSpeed() < 0 && ship.verticalSpeed() > -0.001);
		for (int tick = 1; tick < 600; tick++) {
			physics.step(ship, 0.1, terrain, NO_CURRENT, CONFIG.battleArea());
			assertTrue(ship.verticalSpeed() > -0.75, "Broadside water drag bounds terminal descent.");
		}
		assertTrue(ship.z() < -20, "A destroyed ship must leave the surface within one minute.");
		assertFalse(ship.surfaceConstrained());
		assertEquals(0, ship.hp());
		assertTrue(ship.snapshot().surfaceLocked(), "The snapshot must keep the surface-ship model when it sinks.");
		assertTrue(ship.snapshot().velocity().linear().z() < 0, "Sinking velocity must be visible in telemetry.");
	}

	@Test
	void shipWreckSettlesOnItsOwnPhysicalDraft() {
		var ship = vehicle(VehicleConfig.surfaceShip(), 0, Vec3.ZERO);
		ship.setHp(0);
		var physics = new SubmarinePhysics(CONFIG);
		var terrain = flat(-40);
		for (int tick = 0; tick < 1800; tick++) {
			physics.step(ship, 0.1, terrain, NO_CURRENT, CONFIG.battleArea());
		}
		var hull = HullGeometry.envelope(ship.vehicleConfig());
		double sinPitch = Math.sin(ship.pitch()), cosPitch = Math.cos(ship.pitch());
		double lowestOffset = hull.forwardOffset() * sinPitch + hull.upOffset() * cosPitch
				- Math.hypot(hull.semiLength() * sinPitch, hull.semiHeight() * cosPitch);
		assertEquals(-40, ship.z() + lowestOffset, 1e-6,
				"The actual rotated ship draft, including its forward offset, must rest on the floor.");
		assertEquals(0, ship.velocity().linear().length(), 1e-4, "The wreck should have settled translationally.");
		assertEquals(0, ship.pitchRate(), 1e-4, "Only tiny numerical rocking may remain at rest.");
		assertEquals(0, HullGeometry.terrainPenetration(ship.pose().position(), ship.heading(), ship.pitch(),
				ship.vehicleConfig(), terrain), 1e-6);
		assertEquals(0, ship.hp());
	}

	@Test
	void submarineWreckStillFloodsAndSettlesOnTheKeel() {
		var sub = vehicle(VehicleConfig.submarine(), 0, new Vec3(0, 0, -20));
		sub.setHp(0);
		var physics = new SubmarinePhysics(CONFIG);
		var terrain = flat(-50);
		for (int tick = 0; tick < 1500; tick++) {
			physics.step(sub, 0.1, terrain, NO_CURRENT, CONFIG.battleArea());
		}
		assertEquals(-45, sub.z(), 1e-6);
		assertEquals(0, sub.verticalSpeed(), 1e-12);
		assertEquals(0, sub.actualBallast(), 1e-12);
		assertEquals(0, sub.hp());
	}

	@Test
	void destructionDisablesThrustAndSteeringImmediatelyButRetainsDrift() {
		var ship = vehicle(VehicleConfig.surfaceShip(), 0, Vec3.ZERO);
		ship.setThrottle(1);
		ship.setActualThrottle(1);
		ship.setRudder(1);
		ship.setActualRudder(1);
		ship.setSternPlanes(1);
		ship.setActualSternPlanes(1);
		ship.setSpeed(4);
		ship.setSwaySpeed(0.2);
		ship.setHp(0);
		new SubmarinePhysics(CONFIG).step(ship, 0.1, flat(-100), NO_CURRENT, CONFIG.battleArea());
		assertEquals(0, ship.actualThrottle(), 1e-12);
		assertEquals(0, ship.actualRudder(), 1e-12);
		assertEquals(0, ship.actualSternPlanes(), 1e-12);
		assertEquals(0, ship.yawRate(), 1e-12, "Dead helm commands must not generate a new turn.");
		assertTrue(ship.speed() > 3.9 && ship.speed() < 4, "Existing motion should coast under passive drag.");
		assertTrue(ship.swaySpeed() > 0);
		assertTrue(ship.y() > 0 && ship.x() > 0);
	}

	@Test
	void floodExposureIsIndependentOfTickSubdivision() {
		var oneStep = vehicle(VehicleConfig.surfaceShip(), 0, Vec3.ZERO);
		var manySteps = vehicle(VehicleConfig.surfaceShip(), 1, Vec3.ZERO);
		oneStep.setHp(0);
		manySteps.setHp(0);
		double oneForce = WreckDynamics.advance(oneStep, 20);
		double manyForce = 0;
		for (int tick = 0; tick < 1000; tick++) {
			manyForce = WreckDynamics.advance(manySteps, 0.02);
		}
		assertEquals(1 - Math.exp(-1), oneStep.wreckFloodFraction(), 1e-12);
		assertEquals(oneStep.wreckFloodFraction(), manySteps.wreckFloodFraction(), 1e-12);
		assertEquals(oneForce, manyForce, 1e-6);
	}

	@Test
	void shipWreckRetainsCollisionImpartedPitchMotion() {
		var ship = vehicle(VehicleConfig.surfaceShip(), 0, Vec3.ZERO);
		ship.setZ(-50);
		ship.setHp(0);
		ship.setPitchRate(0.02);
		new SubmarinePhysics(CONFIG).step(ship, 0.1, flat(-200), NO_CURRENT, CONFIG.battleArea());
		assertTrue(ship.pitch() > 0, "A sunk ship's pitch impulse must advance its orientation.");
		assertTrue(ship.pitchRate() > 0 && ship.pitchRate() < 0.02,
				"Passive rotary drag may damp the impulse without freezing it.");
		assertTrue(ship.snapshot().surfaceLocked());
	}

	@Test
	void livingHullCannotPassThroughAStationaryWreck() {
		var living = vehicle(VehicleConfig.submarine(), 0, new Vec3(0, -37.99, -200));
		var wreck = vehicle(VehicleConfig.submarine(), 1, new Vec3(0, 37.99, -200));
		living.setSpeed(2);
		wreck.setHp(0);
		assertNotNull(HullOverlap.contact(living, wreck));
		SimulationLoop.checkSubCollisions(List.of(living, wreck));
		assertFalse(HullOverlap.overlaps(living, wreck), "Destroyed hulls must still block physical passage.");
		assertEquals(980, living.hp(), "A 2 m/s bow impact should remove 20 HP from the living hull.");
		assertEquals(0, wreck.hp(), "Collision damage must not revive or make a wreck's HP negative.");
		assertTrue(wreck.speed() > 0, "A finite-mass wreck receives collision momentum.");
		assertEquals(2, living.speed() + wreck.speed(), 1e-9);
	}

	@Test
	void sunkenShipWreckCanReceiveVerticalCollisionMomentum() {
		var ship = vehicle(VehicleConfig.surfaceShip(), 1, Vec3.ZERO);
		ship.setZ(-200);
		ship.setHp(0);
		var living = vehicle(VehicleConfig.submarine(), 0, new Vec3(0, 1, -211));
		living.setVerticalSpeed(1);
		assertNotNull(HullOverlap.contact(living, ship));
		SimulationLoop.checkSubCollisions(List.of(living, ship));
		assertFalse(HullOverlap.overlaps(living, ship));
		assertTrue(ship.velocity().linear().z() > 0, "A sunken ship wreck must have free heave mobility.");
		assertEquals(0, ship.hp());
		assertTrue(ship.snapshot().surfaceLocked());
		assertEquals(2_500_000, living.velocity().linear().z() * living.vehicleConfig().dryMass()
				+ ship.velocity().linear().z() * ship.vehicleConfig().dryMass(), 1e-6);
	}

	private static SubmarineEntity vehicle(VehicleConfig config, int id, Vec3 position) {
		return new SubmarineEntity(config, id, new DefaultAttackSub(), position, 0, Color.BLUE, 1000);
	}

	private static TerrainMap flat(double elevation) {
		double[] elevations = new double[441];
		Arrays.fill(elevations, elevation);
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}
}
