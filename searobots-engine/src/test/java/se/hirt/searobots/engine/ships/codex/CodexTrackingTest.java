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
package se.hirt.searobots.engine.ships.codex;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.TestHelpers;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CodexTrackingTest {
	@Test
	void activeSlantRangeAndDepthLocateTargetHorizontally() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		Vec3 ownPosition = new Vec3(100.0, -200.0, -100.0);
		Vec3 targetPosition = new Vec3(400.0, 200.0, -340.0);
		var output = track(controller, world, 0L, ownPosition, targetPosition, 1_000);

		assertFalse(output.contactEstimates.isEmpty(), "An active submarine return must publish a target estimate.");
		var estimate = output.contactEstimates.getLast();
		assertEquals(targetPosition.x(), estimate.x(), 1.0e-6,
				"Target X must exclude the vertical component of active sonar range.");
		assertEquals(targetPosition.y(), estimate.y(), 1.0e-6,
				"Target Y must exclude the vertical component of active sonar range.");
	}

	@Test
	void unchangedTargetKeepsDamageEgressWaypointWhileOwnshipMoves() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		Vec3 targetPosition = new Vec3(0.0, 0.0, -140.0);
		var firstOutput = track(controller, world, 0L, new Vec3(-1_800.0, 0.0, -140.0), targetPosition, 999);

		assertFalse(firstOutput.strategicWaypoints.isEmpty(), "The damaged submarine must plan an escape route.");
		assertEquals(Purpose.EVADE, firstOutput.strategicPurposes.getLast(),
				"This scenario must exercise a combat waypoint offset from the stationary target.");
		var escapeWaypoint = firstOutput.strategicWaypoints.getLast();
		assertTrue(Math.hypot(escapeWaypoint.x() - targetPosition.x(), escapeWaypoint.y() - targetPosition.y()) > 500.0,
				"The escape waypoint must be distinct from the target's position.");

		for (long tick = 1; tick <= 20; tick++) {
			// Six metres per second northward; the active return still describes the same world position.
			Vec3 ownPosition = new Vec3(-1_800.0, tick * 6.0 / 50.0, -140.0);
			var output = track(controller, world, tick, ownPosition, targetPosition, 999);
			var waypoint = output.strategicWaypoints.getLast();
			assertEquals(escapeWaypoint.x(), waypoint.x(), 1.0e-9,
					"Ownship motion alone must not reset the escape route at tick " + tick);
			assertEquals(escapeWaypoint.y(), waypoint.y(), 1.0e-9,
					"A stationary target must preserve the escape waypoint at tick " + tick);
		}
	}

	@Test
	void lostContactPredictionUsesTheInputElapsedTime() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		var initial = tick(controller, world, 0, 0.02, List.of(),
				List.of(activeContact(0, 1500, 8, Math.PI / 2, -140, Math.toRadians(0.5), 30)), 0);
		var coast = tick(controller, world, 1, 0.5, List.of(), List.of(), 0);
		var next = tick(controller, world, 2, 0.5, List.of(), List.of(), 0);

		assertEquals(0, initial.contactEstimates.getLast().x(), 1e-9);
		assertEquals(4, coast.contactEstimates.getLast().x(), 1e-9,
				"Half a simulation second must advance an eight-metre-per-second contact by four metres");
		assertEquals(8, next.contactEstimates.getLast().x(), 1e-9);
		assertEquals(1500, next.contactEstimates.getLast().y(), 1e-9);
		assertTrue(next.contactEstimates.getLast().uncertaintyRadius() > initial.contactEstimates.getLast()
				.uncertaintyRadius());
	}

	@Test
	void futureTorpedoAimDoesNotMoveTheCurrentEstimateOrAdvanceItsClock() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		var output = tick(controller, world, 500, 0.02, List.of(),
				List.of(activeContact(0, 1500, 8, 0, -140, Math.toRadians(0.5), 30)), 8);

		assertEquals(1, output.launchedTorpedoCount, "A fresh, aligned active fix must exercise the launch lookahead");
		double aimY = Double.parseDouble(output.launchedTorpedo.missionData().split(";")[1]);
		assertTrue(aimY > 1500, "The torpedo mission must lead the northbound target");
		assertEquals(1500, output.contactEstimates.getLast().y(), 1e-9,
				"Computing a future aim must leave the current published contact at its observed position");

		var coast = tick(controller, world, 501, 0.5, List.of(), List.of(), 0);
		assertEquals(1504, coast.contactEstimates.getLast().y(), 1e-9,
				"The next current estimate may advance only by the actual half-second tick");
	}

	@Test
	void activeBearingAndDepthUncertaintyPreventAnOverconfidentShot() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		var output = tick(controller, world, 500, 0.02, List.of(), List.of(activeContact(0, 2000, 8, 0, -440, 0.2, 20)),
				8);

		assertFalse(output.contactEstimates.isEmpty());
		var estimate = output.contactEstimates.getLast();
		double horizontal = Math.sqrt(2000 * 2000 - 300 * 300);
		assertEquals(horizontal, estimate.y(), 1e-6, "The active slant range must account for the target's depth");
		assertTrue(estimate.uncertaintyRadius() >= horizontal * Math.sin(0.4),
				"The displayed bound must include the active bearing error, rather than using a fixed eighty-metre radius");
		assertEquals(0, output.launchedTorpedoCount,
				"High active quality cannot bypass the shot's position-uncertainty gate");
	}

	@Test
	void bearingOnlyAndUnboundedRangeContactsDoNotInventCartesianTargets() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		var bearingOnly = new SonarContact(0, 20, 0, false, 8, Math.toRadians(1), 0, Double.NaN, 0.8, 0, Double.NaN,
				SonarContact.Classification.SUBMARINE);
		var first = tick(controller, world, 500, 0.02, List.of(bearingOnly), List.of(), 8);
		assertTrue(first.contactEstimates.isEmpty(),
				"A bearing alone cannot put a target at an arbitrary search range");
		assertEquals(0, first.launchedTorpedoCount);

		var unbounded = new SonarContact(0, 20, 2000, false, 8, Math.toRadians(1), Double.MAX_VALUE, Double.NaN, 0.8, 0,
				Double.NaN, SonarContact.Classification.SUBMARINE);
		var next = tick(controller, world, 501, 0.02, List.of(unbounded), List.of(), 8);
		assertTrue(next.contactEstimates.isEmpty(), "An unavailable range bound must not create a Cartesian fix");
		assertEquals(0, next.launchedTorpedoCount);
	}

	@Test
	void freshUnknownSpeedStopsThePreviousVelocityEvenWhenTheActivePositionIsRetained() {
		var world = GeneratedWorld.deepFlat();
		var controller = startedController(world);
		tick(controller, world, 400, 0.02, List.of(),
				List.of(activeContact(0, 1500, 8, 0, -140, Math.toRadians(0.5), 30)), 8);
		var unknownSpeed = new SonarContact(0, 12, 1500, false, -1, Math.toRadians(1), 1200, Double.NaN, 0.3, 0,
				Double.NaN, SonarContact.Classification.SUBMARINE);
		var fresh = tick(controller, world, 500, 2.0, List.of(unknownSpeed), List.of(), 8);
		var coast = tick(controller, world, 501, 0.5, List.of(), List.of(), 8);

		assertEquals(-1, fresh.contactEstimates.getLast().estimatedSpeed());
		assertEquals(-1, coast.contactEstimates.getLast().estimatedSpeed());
		assertEquals(fresh.contactEstimates.getLast().x(), coast.contactEstimates.getLast().x(), 1e-9);
		assertEquals(fresh.contactEstimates.getLast().y(), coast.contactEstimates.getLast().y(), 1e-9,
				"Fresh unknown speed must not quietly preserve eight-metre-per-second motion in the independent active anchor");
		assertTrue(coast.contactEstimates.getLast().uncertaintyRadius() > fresh.contactEstimates.getLast()
				.uncertaintyRadius());
		assertEquals(0, fresh.launchedTorpedoCount,
				"Unresolved fresh speed contradicts the prior resolved motion and must revoke its shot authorization");
		assertEquals(0, coast.launchedTorpedoCount, "Silence must not restore that stale shot authorization");
	}

	private static CodexAttackSub startedController(GeneratedWorld world) {
		var controller = new CodexAttackSub();
		controller.onMatchStart(
				new MatchContext(world.config(), world.terrain(), world.thermalLayers(), world.currentField()));
		return controller;
	}

	private static SonarContact activeContact(
		double bearing, double range, double speed, double heading, double depth, double bearingError,
		double rangeError) {
		return new SonarContact(bearing, 20, range, true, speed, bearingError, rangeError, Double.NaN, 0.95, heading,
				depth, SonarContact.Classification.SUBMARINE);
	}

	private static TestHelpers.CapturedOutput tick(
		CodexAttackSub controller, GeneratedWorld world, long tick, double dt, List<SonarContact> passive,
		List<SonarContact> active, int ammunition) {
		var self = new SubmarineState(new Pose(new Vec3(0, 0, -140), 0, 0, 0), Velocity.ZERO, 1000, ammunition);
		var environment = new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
		var output = new TestHelpers.CapturedOutput();
		controller.onTick(new TestHelpers.TestInput(tick, dt, self, environment, passive, active, 250), output);
		return output;
	}

	private static TestHelpers.CapturedOutput track(
		CodexAttackSub controller, GeneratedWorld world, long tick, Vec3 ownPosition, Vec3 targetPosition, int ownHp) {
		var self = new SubmarineState(new Pose(ownPosition, 0.0, 0.0, 0.0),
				new Velocity(new Vec3(0.0, 6.0, 0.0), Vec3.ZERO), ownHp, 0);
		double dx = targetPosition.x() - ownPosition.x();
		double dy = targetPosition.y() - ownPosition.y();
		double dz = targetPosition.z() - ownPosition.z();
		var contact = new SonarContact(CodexAutopilot.norm(Math.atan2(dx, dy)), 30.0,
				Math.sqrt(dx * dx + dy * dy + dz * dz), true, 0.0, Math.toRadians(0.4), 70.0, 210.0, 0.86, Double.NaN,
				targetPosition.z(), SonarContact.Classification.SUBMARINE);
		var environment = new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
		var output = new TestHelpers.CapturedOutput();
		controller.onTick(
				new TestHelpers.TestInput(tick, 1.0 / 50.0, self, environment, List.of(), List.of(contact), 250),
				output);
		return output;
	}
}
