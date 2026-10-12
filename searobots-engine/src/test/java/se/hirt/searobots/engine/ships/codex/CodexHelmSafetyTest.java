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

import static org.junit.jupiter.api.Assertions.assertTrue;

class CodexHelmSafetyTest {
	@Test
	void torpedoThreatPreservesEmergencySeafloorBraking() {
		var world = GeneratedWorld.flatOcean(-170.0, -140.0);
		var output = defend(world, new Vec3(0.0, 0.0, -140.0), 0.0, Math.toRadians(50.0));

		assertTrue(output.status.contains("PULL UP"), "Scenario must require emergency seafloor avoidance.");
		assertTrue(output.throttle < 0.0, "A torpedo threat must preserve emergency reverse thrust.");
		assertTrue(Math.abs(output.rudder) <= 0.3, "Emergency rudder limit must survive tactical steering.");
		assertTrue(output.sternPlanes > 0.0 && output.ballast > 0.9,
				"Seafloor avoidance must keep the upward recovery command.");
	}

	@Test
	void torpedoThreatPreservesBoundaryTurnAndReverseThrust() {
		var world = GeneratedWorld.deepFlat();
		var output = defend(world, new Vec3(6_900.0, 0.0, -140.0), Math.PI / 2.0, Math.PI / 2.0);

		assertTrue(output.status.contains("BORDER"), "Scenario must require boundary recovery.");
		assertTrue(Math.abs(output.rudder) > 0.5, "The submarine must keep turning back into the battle area.");
		assertTrue(output.throttle < 0.0, "Near the boundary, the planned reverse speed must produce reverse thrust.");
	}

	@Test
	void torpedoThreatPreservesSurfaceSpeedLimit() {
		var world = GeneratedWorld.deepFlat();
		var output = defend(world, new Vec3(0.0, 0.0, -5.0), 0.0, 0.0);

		assertTrue(output.status.contains("SURFACE"), "Scenario must require surface recovery.");
		assertTrue(output.throttle <= 0.2, "A torpedo threat must preserve the surface recovery speed limit.");
		assertTrue(output.sternPlanes < 0.0 && output.ballast < 0.5,
				"Surface recovery must retain the command to dive.");
	}

	private static TestHelpers.CapturedOutput defend(
		GeneratedWorld world, Vec3 position, double heading, double threatBearing) {
		var controller = new CodexAttackSub();
		controller.onMatchStart(
				new MatchContext(world.config(), world.terrain(), world.thermalLayers(), world.currentField()));
		var velocity = new Velocity(new Vec3(Math.sin(heading) * 7.5, Math.cos(heading) * 7.5, 0.0), Vec3.ZERO);
		var self = new SubmarineState(new Pose(position, heading, 0.0, 0.0), velocity, 1_000, 8);
		var threat = new SonarContact(threatBearing, 14.0, 500.0, true, 25.0, Math.toRadians(0.3), 45.0, 115.0, 0.65,
				threatBearing + Math.PI, position.z(), SonarContact.Classification.TORPEDO);
		var environment = new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
		var output = new TestHelpers.CapturedOutput();
		controller.onTick(
				new TestHelpers.TestInput(700L, 1.0 / 50.0, self, environment, List.of(), List.of(threat), 250),
				output);
		return output;
	}
}
