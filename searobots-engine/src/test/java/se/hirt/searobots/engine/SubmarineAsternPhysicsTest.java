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

class SubmarineAsternPhysicsTest {

	private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
	private static final CurrentField STILL_WATER = new CurrentField(List.of());
	private static final TerrainMap DEEP_WATER = flat(-300);

	@Test
	void shipImpactRetainsBackwardMomentumOnFollowingPhysicsTick() {
		for (double dt : new double[] {0.01, 0.02, 0.1}) {
			var sub = vessel(VehicleConfig.submarine(), 0, new Vec3(0, 0, -2.61), 0);
			var ship = vessel(VehicleConfig.surfaceShip(), 1, new Vec3(0, 128.99, 0), Math.PI);
			ship.setSpeed(8);
			SimulationLoop.checkSubCollisions(List.of(sub, ship));
			double incomingSpeed = sub.speed();
			assertTrue(incomingSpeed < -5.8 && incomingSpeed > -5.9,
					"A normal ship impact should impart astern drift faster than the engine operating limit.");
			assertEquals(-40_000_000,
					sub.velocity().linear().y() * sub.vehicleConfig().dryMass()
							+ ship.velocity().linear().y() * ship.vehicleConfig().dryMass(),
					1e-6, "The collision must exchange equal and opposite momentum.");
			assertTrue(sub.hp() > 0 && ship.hp() > 0, "The fixture must involve two surviving vessels.");
			double dragCoefficient = reverseDrag(sub.vehicleConfig());
			double maximumDragChange = dragCoefficient * incomingSpeed * incomingSpeed / sub.vehicleConfig().massSurge()
					* dt;

			step(sub, dt);

			assertTrue(sub.speed() < -sub.vehicleConfig().maxReverseSpeed(),
					"The first postimpact tick must retain physically imparted drift.");
			assertTrue(sub.speed() > incomingSpeed, "Passive water drag must slow the astern drift.");
			assertTrue(sub.speed() - incomingSpeed <= maximumDragChange + 1e-8,
					"Momentum loss must be bounded by the physical drag force over this tick.");
			assertEquals(680, sub.hp(), "Coasting away from the impact must not inflict further damage.");
		}
	}

	@Test
	void passiveAsternDriftDecaysGraduallyAcrossTimesteps() {
		var cfg = VehicleConfig.submarine();
		double duration = 1;
		double exactMagnitude = 8 / (1 + reverseDrag(cfg) * 8 * duration / cfg.massSurge());
		for (double dt : new double[] {0.01, 0.02, 0.1}) {
			var sub = vessel(cfg, 0, new Vec3(0, 0, -100), 0);
			sub.setSpeed(-8);
			for (int tick = 0; tick < Math.round(duration / dt); tick++) {
				double previous = sub.speed();
				step(sub, dt);
				assertTrue(sub.speed() > previous && sub.speed() < -cfg.maxReverseSpeed());
			}
			assertEquals(exactMagnitude, -sub.speed(), 0.03,
					"A one-second coast should follow the analytic quadratic-drag decay, without clipping.");
			assertEquals(1000, sub.hp());
		}
	}

	@Test
	void reverseThrottleCannotAccelerateDriftBeyondOperatingLimit() {
		for (var cfg : new VehicleConfig[] {VehicleConfig.submarine(), VehicleConfig.surfaceShip()}) {
			for (double dt : new double[] {0.01, 0.02, 0.1}) {
				var powered = vessel(cfg, 0, new Vec3(0, 0, -100), 0);
				var coasting = vessel(cfg, 1, new Vec3(0, 0, -100), 0);
				powered.setSpeed(-8);
				coasting.setSpeed(-8);
				powered.setEngineClutch(true);
				powered.setThrottle(-1);
				powered.setActualThrottle(-1);

				step(powered, dt);
				step(coasting, dt);

				assertEquals(coasting.speed(), powered.speed(), 1e-12,
						"Beyond the astern limit, reverse throttle must contribute no accelerating thrust.");
				assertTrue(powered.speed() > -8 && powered.speed() < -cfg.maxReverseSpeed());
			}
		}
	}

	@Test
	void sustainedFullAsternApproachesConfiguredSpeedWithoutClippingOrOvershoot() {
		for (var cfg : new VehicleConfig[] {VehicleConfig.submarine(), VehicleConfig.surfaceShip()}) {
			for (double dt : new double[] {0.02, 0.05, 0.1}) {
				var sub = vessel(cfg, 0, new Vec3(0, 0, -100), 0);
				sub.setEngineClutch(true);
				sub.setThrottle(-1);
				sub.setActualThrottle(-1);
				for (int tick = 0; tick < Math.round(240 / dt); tick++) {
					double previous = sub.speed();
					step(sub, dt);
					assertTrue(sub.speed() <= 0 && sub.speed() >= -cfg.maxReverseSpeed() - 1e-12);
					assertTrue(Math.abs(sub.speed() - previous) < 0.025,
							"Normal reverse propulsion must build speed gradually.");
				}
				assertEquals(-cfg.maxReverseSpeed(), sub.speed(), cfg.maxReverseSpeed() * 0.005,
						"The drag/propulsion balance should preserve each vehicle's astern operating speed.");
				assertEquals(1000, sub.hp());
			}
		}
	}

	@Test
	void equalAheadAndAsternSpeedsHaveOppositeComparableControlAuthority() {
		var ahead = vessel(VehicleConfig.submarine(), 0, new Vec3(0, 0, -100), 0);
		var astern = vessel(VehicleConfig.submarine(), 1, new Vec3(0, 0, -100), 0);
		ahead.setSpeed(3);
		astern.setSpeed(-3);
		for (var sub : List.of(ahead, astern)) {
			sub.setRudder(1);
			sub.setActualRudder(1);
			sub.setSternPlanes(1);
			sub.setActualSternPlanes(1);
			step(sub, 0.02);
		}
		assertTrue(ahead.yawRate() > 0 && astern.yawRate() < 0);
		assertTrue(ahead.pitchRate() > 0 && astern.pitchRate() < 0);
		// Passive stern-first drag slightly reduces the astern speed during this tick.
		assertEquals(ahead.yawRate(), -astern.yawRate(), ahead.yawRate() * 0.005);
		assertEquals(ahead.pitchRate(), -astern.pitchRate(), ahead.pitchRate() * 0.005);
	}

	@Test
	void passivePitchRestorationHasSameSignAndComparableStrengthAheadAndAstern() {
		var ahead = vessel(VehicleConfig.submarine(), 0, new Vec3(0, 0, -100), 0);
		var astern = vessel(VehicleConfig.submarine(), 1, new Vec3(0, 0, -100), 0);
		ahead.setSpeed(3);
		astern.setSpeed(-3);
		for (var sub : List.of(ahead, astern)) {
			sub.setPitch(0.1);
			step(sub, 0.02);
			assertTrue(sub.pitchRate() < 0, "A raised bow should restore toward level in either travel direction.");
		}
		assertEquals(ahead.pitchRate(), astern.pitchRate(), Math.abs(ahead.pitchRate()) * 0.005);
	}

	@Test
	void fastCollisionImpartedAsternMotionKeepsFiniteCorrectlySignedSteering() {
		var sub = vessel(VehicleConfig.submarine(), 0, new Vec3(0, 0, -100), 0);
		sub.setSpeed(-8);
		sub.setPitch(0.1);
		sub.setRudder(1);
		sub.setActualRudder(1);
		step(sub, 0.02);
		assertTrue(sub.speed() < -4.47, "The test must retain motion beyond the old signed-inertia singularity.");
		assertTrue(Double.isFinite(sub.yawRate()) && sub.yawRate() < 0);
		assertTrue(Double.isFinite(sub.pitchRate()) && sub.pitchRate() < 0);
		assertTrue(Math.abs(sub.yawRate()) < 0.01 && Math.abs(sub.pitchRate()) < 0.01,
				"Fast astern drift must retain damping rather than causing a violent turn.");
		assertTrue(Double.isFinite(sub.heading()) && Double.isFinite(sub.pitch()));
	}

	private static double reverseDrag(VehicleConfig cfg) {
		return cfg.maxThrust() * cfg.reverseThrustFactor() / (cfg.maxReverseSpeed() * cfg.maxReverseSpeed());
	}

	private static SubmarineEntity vessel(VehicleConfig cfg, int id, Vec3 position, double heading) {
		var sub = new SubmarineEntity(cfg, id, null, position, heading, Color.BLUE, 1000);
		sub.setEngineClutch(false);
		return sub;
	}

	private static void step(SubmarineEntity sub, double dt) {
		new SubmarinePhysics(CONFIG).step(sub, dt, DEEP_WATER, STILL_WATER, CONFIG.battleArea());
	}

	private static TerrainMap flat(double elevation) {
		double[] heights = new double[9];
		Arrays.fill(heights, elevation);
		return new TerrainMap(heights, 3, 3, -1000, -1000, 1000);
	}
}
