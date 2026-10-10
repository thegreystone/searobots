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

	private static CodexAttackSub startedController(GeneratedWorld world) {
		var controller = new CodexAttackSub();
		controller.onMatchStart(
				new MatchContext(world.config(), world.terrain(), world.thermalLayers(), world.currentField()));
		return controller;
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
