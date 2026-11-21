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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.SubmarineDrone;

/**
 * A decided match ends on the deciding tick unless asked to run on, so a recording can include the
 * aftermath (a wreck going down).
 */
class SimulationAftermathTest {

	@Test
	void decidedMatchEndsOnTheDecidingTickByDefault() {
		var ticks = run(0);
		assertTrue(ticks.size() <= 2, "A match decided on its first tick should end at once, ran " + ticks.size());
	}

	@Test
	void decidedMatchRunsOnForTheRequestedTicks() {
		var early = run(0);
		var aftermath = run(50);
		assertEquals(early.size() + 50, aftermath.size(), "Fifty more ticks should be simulated after the decision");
		assertEquals(aftermath.getLast() - early.getLast(), 50L);
	}

	@Test
	void negativeAftermathIsRejected() {
		assertThrows(IllegalArgumentException.class, () -> new SimulationLoop().setTicksAfterDecision(-1));
	}

	/** One submarine spawned below crush depth, so the match is decided on its first tick. */
	private static List<Long> run(long ticksAfterDecision) {
		var flat = GeneratedWorld.flatOcean(-1000, -100);
		double crushed = flat.config().crushDepth() - 50;
		var world = new GeneratedWorld(flat.config().withMatchDurationTicks(1000), flat.terrain(), flat.thermalLayers(),
				flat.currentField(), List.of(new Vec3(-1000, 0, -100), new Vec3(1000, 0, crushed)));
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		sim.setTicksAfterDecision(ticksAfterDecision);
		var ticks = new ArrayList<Long>();
		sim.run(world, List.of(new SubmarineDrone(), new SubmarineDrone()),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
						ticks.add(tick);
					}

					@Override
					public void onMatchEnd() {
					}
				});
		return ticks;
	}
}
