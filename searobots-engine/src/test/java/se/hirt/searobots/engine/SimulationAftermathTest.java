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
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.Test;

import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoOutput;
import se.hirt.searobots.api.TorpedoLaunchCommand;
import se.hirt.searobots.api.TorpedoLaunchContext;
import se.hirt.searobots.engine.ships.SubmarineDrone;

/**
 * A decided match ends on the deciding tick unless asked to run on, so a recording can include the
 * aftermath (a wreck going down).
 */
class SimulationAftermathTest {

	@Test
	void torpedoCanKillTheSurvivorAfterItsLauncherForfeits() {
		var plain = runWithPendingTorpedo(true, 0);
		var recorded = runWithPendingTorpedo(true, 50);
		assertTrue(plain.tick() > 150, "A live torpedo must keep the match open after its owner forfeits");
		assertTrue(plain.controllers().get(0).forfeited());
		assertTrue(plain.controllers().get(1).hp() <= 0,
				"The shot in the water must be allowed to kill the surviving opponent");
		assertEquals(plain.tick(), recorded.tick());
		assertEquals(plain.controllers().get(1).hp(), recorded.controllers().get(1).hp());
	}

	@Test
	void survivorsOwnTorpedoAlsoKeepsTheMatchOpen() {
		var result = runWithPendingTorpedo(false, 50);
		assertEquals(200, result.tick(), "Even the survivor's own weapon must resolve before scoring");
		assertFalse(result.controllers().get(0).forfeited());
		assertTrue(result.controllers().get(0).hp() > 0);
		assertEquals(1, result.controllers().get(0).torpedoesFired());
	}

	private static SimulationLoop.MatchOutcome runWithPendingTorpedo(boolean launcherForfeits, long aftermath) {
		var flat = GeneratedWorld.flatOcean(-1000, -100);
		var world = new GeneratedWorld(flat.config().withMatchDurationTicks(2000), flat.terrain(), flat.thermalLayers(),
				flat.currentField(), List.of(new Vec3(0, 0, -100), new Vec3(0, launcherForfeits ? 200 : 2000, -100)));
		var launcher = new SubmarineController() {
			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				if (launcherForfeits && input.tick() >= 150) {
					throw new IllegalStateException("launcher forfeits while its weapon is running");
				}
				output.setBallast(0.5);
				if (input.tick() == 0 || launcherForfeits && input.tick() == 100) {
					output.launchTorpedo(new TorpedoLaunchCommand(0, 0, Double.NaN, ""));
				}
			}

			@Override
			public TorpedoController createTorpedoController() {
				return new TorpedoController() {
					@Override
					public void onLaunch(TorpedoLaunchContext context) {
					}

					@Override
					public void onTick(TorpedoInput input, TorpedoOutput output) {
						output.setThrottle(1);
						if (!launcherForfeits && input.tick() >= 200) {
							output.detonate();
						}
					}
				};
			}
		};
		var opponent = new SubmarineController() {
			@Override
			public void onMatchStart(MatchContext context) {
				if (!launcherForfeits) {
					throw new IllegalStateException("opponent forfeits before launch");
				}
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setBallast(0.5);
			}
		};
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		sim.setControllerLimits(SimulationLoop.ControllerLimits.leagueDefaults());
		sim.setTicksAfterDecision(aftermath);
		var pending = new AtomicBoolean();
		long[] lastTick = {-1};
		sim.run(world, List.of(launcher, opponent), List.of(VehicleConfig.submarine(), VehicleConfig.submarine()),
				List.of(0.0, 0.0), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
						lastTick[0] = tick;
						if (tick == (launcherForfeits ? 150 : 0)) {
							pending.set(torpedoes.stream().anyMatch(TorpedoSnapshot::alive));
							assertTrue(subs.stream().anyMatch(SubmarineSnapshot::forfeited));
						}
					}

					@Override
					public void onMatchEnd() {
					}
				});
		assertTrue(pending.get(), "The fixture must have an active torpedo when a submarine forfeits");
		assertNotNull(sim.matchOutcome());
		assertEquals(sim.matchOutcome().tick() + aftermath, lastTick[0]);
		return sim.matchOutcome();
	}

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
