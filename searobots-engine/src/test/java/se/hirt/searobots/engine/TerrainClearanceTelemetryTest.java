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
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY WAY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

class TerrainClearanceTelemetryTest {

	private static final TerrainMap TERRAIN = flatTerrain();
	private static final CurrentField CURRENTS = new CurrentField(List.of());

	@Test
	void legacyInputRetainsSubmarineNavigationMargin() {
		var entity = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, -100), 0, Color.GREEN,
				1000);
		var input = TestHelpers.makeInput(0, entity, TERRAIN, CURRENTS);

		assertEquals(VehicleConfig.submarine().terrainClearance(), input.recommendedTerrainClearance());
	}

	@Test
	void liveInputsExposeEachVehiclesAdvisoryMargin() {
		var submarineMargins = new ArrayList<Double>();
		var shipMargins = new ArrayList<Double>();
		SubmarineController submarine = (input, output) -> submarineMargins.add(input.recommendedTerrainClearance());
		SubmarineController ship = (input, output) -> shipMargins.add(input.recommendedTerrainClearance());
		var submarineConfig = VehicleConfig.submarine();
		var shipConfig = VehicleConfig.surfaceShip();
		var config = MatchConfig.withDefaults(42).withMatchDurationTicks(2);
		var world = new GeneratedWorld(config, TERRAIN, List.of(), CURRENTS,
				List.of(new Vec3(-500, 0, -100), new Vec3(500, 0, 0)));
		var loop = new SimulationLoop();
		loop.setSpeedMultiplier(1e9);

		loop.run(world, List.of(submarine, ship), List.of(submarineConfig, shipConfig), List.of(0.0, 0.0),
				new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
					}

					@Override
					public void onMatchEnd() {
					}
				});

		assertEquals(List.of(submarineConfig.terrainClearance(), submarineConfig.terrainClearance()), submarineMargins);
		assertEquals(List.of(shipConfig.terrainClearance(), shipConfig.terrainClearance()), shipMargins);
		assertNotEquals(submarineMargins.getFirst(), shipMargins.getFirst(),
				"A surface ship must receive its own navigation margin rather than the legacy submarine default");
	}

	private static TerrainMap flatTerrain() {
		double[] depths = new double[21 * 21];
		Arrays.fill(depths, -1000);
		return new TerrainMap(depths, 21, 21, -2000, -2000, 200);
	}
}
