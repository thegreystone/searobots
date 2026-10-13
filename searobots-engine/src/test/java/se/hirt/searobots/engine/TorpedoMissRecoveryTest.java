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
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.codex.CodexTorpedoController;

import java.awt.Color;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Real guidance, submarine dynamics, and armed hull fuses through a missed approach and recovery.
 */
class TorpedoMissRecoveryTest {
	private static final double DT = 1.0 / 50;
	private static final double FUSE_RADIUS = 22;
	private static final double RECOVERY_WINDOW = 90;

	@Test
	void evadingSubmarineGainsTimeAfterMissedPass() {
		var outcome = runApproach(1_200, -160, Math.PI / 4, 9, true);

		assertTrue(Double.isFinite(outcome.firstRecedingTime),
				"The target must evade the initial approach: " + outcome);
		assertTrue(outcome.firstApproachGap > FUSE_RADIUS + 5,
				"The initial miss must have meaningful clearance from the armed hull fuse: " + outcome);
		assertTrue(Double.isFinite(outcome.targetBehindTime) && outcome.targetBehindTime > outcome.firstRecedingTime,
				"The weapon must actually overrun the target before its recovery is assessed: " + outcome);
		assertTrue(outcome.alive && Double.isNaN(outcome.detonationTime),
				"A missed pass must not become a rapid return hit against the evading submarine: " + outcome);
		assertTrue(outcome.elapsed >= outcome.firstRecedingTime + RECOVERY_WINDOW - DT,
				"The complete recovery window must be simulated: " + outcome);
		assertTrue(outcome.minimumGapAfterMiss > FUSE_RADIUS + 5,
				"The evading hull must retain clearance throughout the attempted return: " + outcome);
		assertTrue(outcome.fuelRemaining > 0 && outcome.targetHp == 1_000,
				"The escape must come from maneuvering, with a powered weapon and undamaged target: " + outcome);
	}

	@Test
	void steadyCrossingTargetStillReceivesArmedFirstPassHit() {
		assertFirstPassHit(runApproach(1_200, -100, Math.PI / 2, 6, false));
	}

	@Test
	void stationaryTargetStillReceivesArmedFirstPassHit() {
		assertFirstPassHit(runApproach(600, -160, 0, 0, false));
	}

	private static void assertFirstPassHit(Outcome outcome) {
		assertTrue(Double.isFinite(outcome.detonationTime) && outcome.detonationTime < 90,
				"Ordinary aimed shots must remain effective: " + outcome);
		assertTrue(Double.isNaN(outcome.firstRecedingTime) && Double.isNaN(outcome.targetBehindTime),
				"The ordinary target must be intercepted on the first approach: " + outcome);
		assertTrue(outcome.targetHp < 1_000, "The armed hit must damage the actual submarine hull: " + outcome);
	}

	private static Outcome runApproach(
		double range, double launchDepth, double targetHeading, double targetSpeed, boolean evade) {
		var world = GeneratedWorld.deepFlat();
		var submarineConfig = VehicleConfig.submarine();
		var target = new SubmarineEntity(submarineConfig, 1, (input, output) -> {
		}, new Vec3(0, range, -200), targetHeading, Color.BLUE, 1_000);
		double targetThrottle = submarineConfig.dragCoeff() * targetSpeed * targetSpeed / submarineConfig.maxThrust();
		target.setSpeed(targetSpeed);
		target.setThrottle(targetThrottle);
		target.setActualThrottle(targetThrottle);
		var submarines = List.of(target);

		var controller = new CodexTorpedoController();
		var launch = new Vec3(0, 0, launchDepth);
		var torpedo = new TorpedoEntity(2, 0, VehicleConfig.torpedo(), controller, launch, 0, 0, FUSE_RADIUS,
				Color.RED);
		torpedo.setSpeed(23);
		controller.onLaunch(new TorpedoLaunchContext(world.config(), world.terrain(), launch, 0, 0,
				String.format(Locale.ROOT, "0;%s;-200;%s;%s;5", range, targetHeading, targetSpeed)));
		var torpedoPhysics = new TorpedoPhysics();
		var submarinePhysics = new SubmarinePhysics(world.config());

		double evasionStarted = Double.NaN;
		double firstRecedingTime = Double.NaN;
		double firstApproachGap = Double.NaN;
		double targetBehindTime = Double.NaN;
		double detonationTime = Double.NaN;
		double previousGap = Double.POSITIVE_INFINITY;
		double minimumGapAfterMiss = Double.POSITIVE_INFINITY;
		double elapsed = 0;
		for (int tick = 0; tick < 10_000 && torpedo.alive(); tick++) {
			double time = tick * DT;
			var position = torpedo.pose().position();
			var targetPosition = target.pose().position();
			double distance = position.distanceTo(targetPosition);
			if (evade && Double.isNaN(evasionStarted) && distance < 300) {
				evasionStarted = time;
			}
			if (Double.isFinite(evasionStarted)) {
				boolean turning = time - evasionStarted < 35;
				target.setRudder(turning ? -0.55 : 0);
				target.setSternPlanes(turning ? -0.55 : 0);
				target.setBallast(turning ? 1 : 0.5);
			}

			// Perfect omnidirectional fixes every second deliberately favor recovery: any
			// escape must result from the physics rather than sonar or tracking dropouts.
			List<SonarContact> fixes = tick % 50 == 0 ? List.of(
					new SonarContact(Math.atan2(targetPosition.x() - position.x(), targetPosition.y() - position.y()),
							35, distance, true, target.speed(), 0, 0, 220, 1, target.heading(), target.z(),
							SonarContact.Classification.SUBMARINE))
					: List.of();
			controller.onTick(new Input(tick, torpedo.pose(), torpedo.velocity(), torpedo.speed(),
					torpedo.fuelRemaining(), fixes), torpedo.createOutput());
			submarinePhysics.step(target, DT, world.terrain(), world.currentField(), world.config().battleArea());
			if (torpedo.detonateRequested()) {
				SimulationLoop.handleDetonation(torpedo, submarines, world.config(), false);
			} else {
				torpedoPhysics.step(torpedo, DT, world.terrain(), world.currentField(), world.config().battleArea());
			}
			double gap = HullGeometry.bowDistanceToHull(torpedo, target);
			if (Double.isNaN(firstRecedingTime) && gap > previousGap + 0.001 && previousGap < 100) {
				firstRecedingTime = time;
				firstApproachGap = previousGap;
			}
			if (Double.isFinite(firstRecedingTime)) {
				minimumGapAfterMiss = Math.min(minimumGapAfterMiss, gap);
			}
			var relative = target.pose().position().subtract(torpedo.pose().position());
			double cosPitch = Math.cos(torpedo.pitch());
			double forward = relative.x() * Math.sin(torpedo.heading()) * cosPitch
					+ relative.y() * Math.cos(torpedo.heading()) * cosPitch + relative.z() * Math.sin(torpedo.pitch());
			if (Double.isNaN(targetBehindTime) && forward < 0 && gap < 200) {
				targetBehindTime = time;
			}
			SimulationLoop.checkProximityFuse(torpedo, submarines, world.config(), tick);
			if (torpedo.detonated()) {
				detonationTime = time;
			}
			elapsed = time;
			previousGap = gap;
			if (Double.isFinite(firstRecedingTime) && time >= firstRecedingTime + RECOVERY_WINDOW) {
				break;
			}
		}
		return new Outcome(firstRecedingTime, firstApproachGap, targetBehindTime, detonationTime, minimumGapAfterMiss,
				elapsed, torpedo.fuelRemaining(), target.hp(), torpedo.alive());
	}

	private record Outcome(double firstRecedingTime, double firstApproachGap, double targetBehindTime,
			double detonationTime, double minimumGapAfterMiss, double elapsed, double fuelRemaining, int targetHp,
			boolean alive) {
	}

	private record Input(long tick, Pose self, Velocity velocity, double speed, double fuelRemaining,
			List<SonarContact> activeSonarReturns) implements TorpedoInput {
		@Override
		public double deltaTimeSeconds() {
			return DT;
		}
	}
}
