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
import java.awt.Color;

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

	@Test
	void simulationUsesConfiguredDepthLimits() {
		var config = new MatchConfig(CONFIG.worldSeed(), CONFIG.tickRateHz(), CONFIG.matchDurationTicks(),
				CONFIG.submarineCount(), CONFIG.torpedoCount(), CONFIG.startingHp(), CONFIG.blastRadius(),
				CONFIG.minFuseRadius(), CONFIG.maxFuseRadius(), -100, -200, CONFIG.battleArea(),
				CONFIG.terrainMarginMeters(), CONFIG.gridCellMeters(), CONFIG.minSeaFloorZ(), CONFIG.maxSeaFloorZ(),
				CONFIG.maxSubSpeed(), CONFIG.startTime());
		var stressed = runAtDepth(config, -150, 10 * config.tickRateHz());
		assertTrue(stressed.hp() > 0 && stressed.hp() < config.startingHp());
		assertEquals(0, runAtDepth(config, -200, 1).hp());
	}

	@Test
	void crossingCrushDepthDuringMovementDestroysTheHull() {
		var sub = movingSub(CONFIG.crushDepth() + 0.01, -2);
		new SubmarinePhysics(CONFIG).step(sub, 1.0 / CONFIG.tickRateHz(), TERRAIN, NO_CURRENT, CONFIG.battleArea());

		assertTrue(sub.z() < CONFIG.crushDepth(), "The step should cross the crush-depth boundary");
		assertEquals(0, sub.hp());
	}

	@Test
	void terrainCorrectionCannotRescueACrushDepthCrossing() {
		var sub = movingSub(CONFIG.crushDepth() + 0.01, -2);
		double[] elevations = new double[9];
		// Physical keel contact raises the centre one metre above the limit.
		Arrays.fill(elevations, CONFIG.crushDepth() - 4);
		var terrain = new TerrainMap(elevations, 3, 3, -100, -100, 100);
		new SubmarinePhysics(CONFIG).step(sub, 1.0 / CONFIG.tickRateHz(), terrain, NO_CURRENT, CONFIG.battleArea());

		assertTrue(sub.z() > CONFIG.crushDepth(), "Terrain should have moved the wreck back above crush depth");
		assertEquals(0, sub.hp(), "The pressure failure must survive terrain correction");
		assertEquals(0, sub.verticalSpeed(), "The imploded hull should settle as a wreck rather than bounce");
	}

	@Test
	void startingBeyondCrushDepthCannotEscapeInTheSameTick() {
		var sub = movingSub(CONFIG.crushDepth() - 0.01, 2);
		new SubmarinePhysics(CONFIG).step(sub, 1.0 / CONFIG.tickRateHz(), TERRAIN, NO_CURRENT, CONFIG.battleArea());

		assertTrue(sub.z() > CONFIG.crushDepth(), "The movement should end above crush depth");
		assertEquals(0, sub.hp(), "The starting depth must also obey the absolute limit");
	}

	@Test
	void collisionProjectionPastCrushDepthDestroysTheHullBeforeItsSnapshot() {
		var config = CONFIG.withMatchDurationTicks(1);
		var world = new GeneratedWorld(config, TERRAIN, List.of(), NO_CURRENT,
				List.of(new Vec3(0, 0, config.crushDepth() + 0.1), new Vec3(0, 0, config.crushDepth() + 8.1)));
		SubmarineController hold = (input, output) -> output.setBallast(0.5);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1_000_000);
		var lastSnapshot = new SubmarineSnapshot[1];
		sim.run(world, List.of(hold, hold), List.of(VehicleConfig.submarine(), VehicleConfig.submarine()),
				List.of(0.0, 0.0), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
						lastSnapshot[0] = submarines.getFirst();
					}

					@Override
					public void onMatchEnd() {
					}
				});

		assertNotNull(lastSnapshot[0]);
		assertTrue(lastSnapshot[0].pose().position().z() <= config.crushDepth(),
				"Pair separation must push the lower hull across the crush boundary in this fixture");
		assertEquals(0, lastSnapshot[0].hp(), "Contact projection must not bypass the absolute depth limit");
	}

	@Test
	void checkingProjectedDepthDoesNotChargeExtraPressureExposure() {
		var physics = new SubmarinePhysics(CONFIG);
		var projected = movingSub(CONFIG.ratedDepth() - 50, 0);
		var control = movingSub(CONFIG.ratedDepth() - 50, 0);
		var controlPhysics = new SubmarinePhysics(CONFIG);
		for (int tick = 0; tick < 500; tick++) {
			physics.step(projected, 1.0 / CONFIG.tickRateHz(), TERRAIN, NO_CURRENT, CONFIG.battleArea());
			controlPhysics.step(control, 1.0 / CONFIG.tickRateHz(), TERRAIN, NO_CURRENT, CONFIG.battleArea());
			physics.enforceCrushDepth(projected);
			assertEquals(control.hp(), projected.hp(), "A final-position guard must not count a second physics tick");
		}
	}

	private static SubmarineEntity movingSub(double z, double verticalSpeed) {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, z), 0, Color.GREEN,
				CONFIG.startingHp());
		sub.setVerticalSpeed(verticalSpeed);
		return sub;
	}

	private static TerrainMap deepFlat() {
		double[] elevations = new double[9];
		Arrays.fill(elevations, -1500);
		return new TerrainMap(elevations, 3, 3, -100, -100, 100);
	}

	private static SubmarineSnapshot runAtDepth(double depth, int durationTicks) {
		return runAtDepth(CONFIG, depth, durationTicks);
	}

	private static SubmarineSnapshot runAtDepth(MatchConfig matchConfig, double depth, int durationTicks) {
		var config = matchConfig.withMatchDurationTicks(durationTicks);
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
