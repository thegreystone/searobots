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
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Pressure-hull regression tests through the simulation loop, where match depth limits are
 * available. Flat terrain, neutral ballast and no propulsion isolate pressure from collision
 * damage.
 */
class SubmarineDepthLimitTest {

	private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
	private static final TerrainMap TERRAIN = deepFlat();
	private static final CurrentField NO_CURRENT = new CurrentField(List.of());

	@Test
	void ratedDepthDoesNotDamageTheHull() {
		var sub = runAtDepth(CONFIG.ratedDepth(), 10 * CONFIG.tickRateHz());

		assertEquals(CONFIG.ratedDepth(), sub.pose().position().z(), 1e-9);
		assertEquals(CONFIG.startingHp(), sub.hp(), "A submarine at its rated depth should remain undamaged");
	}

	@Test
	void briefExcursionBelowRatedDepthDamagesTheHullWithoutImmediateDestruction() {
		double depth = CONFIG.ratedDepth() - 50;
		var sub = runAtDepth(depth, 10 * CONFIG.tickRateHz());

		assertEquals(depth, sub.pose().position().z(), 1e-9);
		assertTrue(sub.hp() < CONFIG.startingHp(), "Ten seconds below rated depth should cause pressure damage");
		assertTrue(sub.hp() > 0, "A brief 50 m excursion below rated depth should be survivable for this seed");
	}

	@Test
	void reachingCrushDepthDestroysTheHullImmediately() {
		var sub = runAtDepth(CONFIG.crushDepth(), 1);

		assertEquals(0, sub.hp(), "Reaching the absolute crush depth should destroy the submarine on that tick");
	}

	private static TerrainMap deepFlat() {
		double[] elevations = new double[9];
		Arrays.fill(elevations, -1500);
		return new TerrainMap(elevations, 3, 3, -100, -100, 100);
	}

	private static SubmarineSnapshot runAtDepth(double depth, int durationTicks) {
		var config = CONFIG.withMatchDurationTicks(durationTicks);
		var world = new GeneratedWorld(config, TERRAIN, List.of(), NO_CURRENT, List.of(new Vec3(0, 0, depth)));
		SubmarineController holdPosition = (input, output) -> {
			output.setThrottle(0);
			output.setBallast(0.5);
			output.setRudder(0);
			output.setSternPlanes(0);
		};
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1_000_000);
		var lastSnapshot = new SubmarineSnapshot[1];
		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
				lastSnapshot[0] = submarines.getFirst();
			}

			@Override
			public void onMatchEnd() {
			}
		};
		sim.run(world, List.of(holdPosition), List.of(VehicleConfig.submarine()), List.of(0.0), listener);
		assertNotNull(lastSnapshot[0], "The simulation should publish a submarine snapshot");
		return lastSnapshot[0];
	}
}
