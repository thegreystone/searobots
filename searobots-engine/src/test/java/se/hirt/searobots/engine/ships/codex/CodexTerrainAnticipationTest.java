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
import se.hirt.searobots.engine.SubmarineEntity;
import se.hirt.searobots.engine.SubmarinePhysics;
import se.hirt.searobots.engine.TestHelpers;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CodexTerrainAnticipationTest {
	private static final double DT = 1.0 / 50.0;

	@Test
	void brakesForShoalOnCurrentCourseBeforeTurnReachesSafeRoute() {
		var terrain = risingShoal();
		var config = MatchConfig.withDefaults(0L);
		var currents = new CurrentField(List.of());
		var autopilot = new CodexAutopilot(new MatchContext(config, terrain, List.of(), currents));
		var goal = new StrategicWaypoint(-1_200.0, 0.0, -140.0, Purpose.RALLY, NoisePolicy.NORMAL,
				MovementPattern.DIRECT, 180.0, 9.0);
		autopilot.setWaypoints(List.of(goal), 0.0, 0.0, -140.0, 0.0, 9.0);
		var submarine = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0.0, 0.0, -140.0), 0.0,
				Color.BLUE, 1_000);
		submarine.setSpeed(9.0);
		submarine.setActualThrottle(0.36);
		var environment = new EnvironmentSnapshot(terrain, List.of(), currents);
		var initialOutput = new TestHelpers.CapturedOutput();
		autopilot.tick(input(0L, submarine, environment), initialOutput);

		assertEquals("TERRAIN", autopilot.lastStatus(),
				"Current centre clearance is safe, but the turning hull is not.");
		assertTrue(initialOutput.throttle < 0.0, "The current course needs braking before the shoal reaches the bow.");
		assertTrue(initialOutput.sternPlanes > 0.0 && initialOutput.ballast > 0.9,
				"The submarine must begin climbing before its centre reaches the slope.");

		var physics = new SubmarinePhysics();
		for (long tick = 0; tick < 6_000; tick++) {
			autopilot.tick(input(tick, submarine, environment), submarine);
			physics.step(submarine, DT, terrain, currents, config.battleArea());
			assertEquals(1_000, submarine.hp(), "Turning away from the shoal must avoid hull damage at tick " + tick);
		}
		assertTrue(Math.hypot(submarine.x(), submarine.y()) > 100.0,
				"Terrain anticipation must allow the submarine to make progress after recovering.");
	}

	private static TestHelpers.TestInput input(long tick, SubmarineEntity submarine, EnvironmentSnapshot environment) {
		return new TestHelpers.TestInput(tick, DT, submarine.state(), environment, List.of(), List.of(), 250);
	}

	private static TerrainMap risingShoal() {
		int cells = 401;
		double origin = -2_000.0;
		double[] elevations = new double[cells * cells];
		for (int row = 0; row < cells; row++) {
			double y = origin + row * 10.0;
			for (int col = 0; col < cells; col++) {
				double x = origin + col * 10.0;
				double floor = -300.0;
				if (Math.abs(x) <= 180.0 && y > 80.0) {
					floor = Math.min(-90.0, -300.0 + (y - 80.0) * 1.5);
				}
				elevations[row * cells + col] = floor;
			}
		}
		return new TerrainMap(elevations, cells, cells, origin, origin, 10.0);
	}
}
