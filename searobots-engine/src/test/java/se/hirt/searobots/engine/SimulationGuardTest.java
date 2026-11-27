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
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Color;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import se.hirt.searobots.api.ContactEstimate;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.MatchResult;
import se.hirt.searobots.api.Purpose;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoLaunchCommand;
import se.hirt.searobots.api.TorpedoLaunchContext;
import se.hirt.searobots.api.TorpedoOutput;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.api.Waypoint;
import se.hirt.searobots.engine.SimulationLoop.ControllerLimits;
import se.hirt.searobots.engine.replay.ReplayReader;
import se.hirt.searobots.engine.replay.ReplayWriter;

class SimulationGuardTest {

	private static GeneratedWorld twoSubFlatWorld(int durationTicks) {
		var flat = GeneratedWorld.flatOcean(-1000, -150);
		return new GeneratedWorld(flat.config().withMatchDurationTicks(durationTicks), flat.terrain(),
				flat.thermalLayers(), flat.currentField(), List.of(new Vec3(-1500, 0, -150), new Vec3(1500, 0, -150)));
	}

	private static SubmarineController calmSub(String name) {
		return new SubmarineController() {
			@Override
			public String name() {
				return name;
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setThrottle(0.2);
				output.setRudder(0);
				output.setSternPlanes(0);
				output.setBallast(0.5);
			}
		};
	}

	@Test
	void outputPassedToControllerIsNotSubmarineEntity() {
		var sawSubmarineEntity = new AtomicBoolean(false);
		var sawThrottleApplied = new AtomicBoolean(false);
		var probe = new SubmarineController() {
			@Override
			public String name() {
				return "Probe";
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				if (output instanceof SubmarineEntity) {
					sawSubmarineEntity.set(true);
				}
				output.setThrottle(0.6);
				output.setBallast(0.5);
			}
		};

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		sim.run(twoSubFlatWorld(10), List.of(probe, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
						if (subs.getFirst().throttle() > 0.5) {
							sawThrottleApplied.set(true);
						}
					}

					@Override
					public void onMatchEnd() {
					}
				});

		assertFalse(sawSubmarineEntity.get(), "Controller must not receive raw SubmarineEntity");
		assertTrue(sawThrottleApplied.get(), "Delegating facade must forward actuator commands");
	}

	@Test
	void nonFiniteActuatorsAndExcessiveTelemetryAreGuarded() {
		var lastSnap = new AtomicReference<SubmarineSnapshot>();
		var spammer = new SubmarineController() {
			@Override
			public String name() {
				return "Spam\tSub\n" + "X".repeat(80);
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setRudder(0.25);
				output.setThrottle(0.5);
				output.setBallast(0.5);
				// Non-finite attempts should be ignored without poisoning state
				output.setRudder(Double.NaN);
				output.setSternPlanes(Double.POSITIVE_INFINITY);
				output.setThrottle(Double.NEGATIVE_INFINITY);
				output.setBallast(Double.NaN);
				output.setStatus("bad\tstatus\n" + "S".repeat(100));

				String hugeLabel = "lbl\t" + "L".repeat(200);
				for (int i = 0; i < 400; i++) {
					output.publishContactEstimate(new ContactEstimate(i, i, 0.8, 0.8, 50, 0, 5, hugeLabel));
					output.publishWaypoint(new Waypoint(i, i, -150, i == 0));
					output.publishStrategicWaypoint(new Waypoint(i, i, -150, i == 0), Purpose.PATROL);
				}
			}
		};

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		sim.run(twoSubFlatWorld(5), List.of(spammer, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
						lastSnap.set(subs.getFirst());
					}

					@Override
					public void onMatchEnd() {
					}
				});

		SubmarineSnapshot snap = lastSnap.get();
		assertTrue(Double.isFinite(snap.pose().position().x()));
		assertTrue(Double.isFinite(snap.rudder()));
		assertEquals(0.25, snap.rudder(), 1e-9);
		assertEquals(0.5, snap.throttle(), 1e-9);
		assertEquals(SubmarineEntity.MAX_NAME_LENGTH, snap.name().length());
		assertFalse(snap.name().contains("\t") || snap.name().contains("\n"));
		assertEquals(SubmarineEntity.MAX_STATUS_LENGTH, snap.status().length());
		assertFalse(snap.status().contains("\t") || snap.status().contains("\n"));
		assertEquals(SubmarineEntity.MAX_CONTACT_ESTIMATES_PER_TICK, snap.contactEstimates().size());
		assertTrue(snap.contactEstimates().getFirst().label().length() <= SubmarineEntity.MAX_LABEL_LENGTH);
		assertEquals(SubmarineEntity.MAX_WAYPOINTS_PER_TICK, snap.waypoints().size());
		assertEquals(SubmarineEntity.MAX_STRATEGIC_WAYPOINTS_PER_TICK, snap.strategicWaypoints().size());
	}

	@Test
	void throwingControllerPropagatesByDefault() {
		var broken = new SubmarineController() {
			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				throw new IllegalStateException("boom");
			}
		};
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		assertThrows(IllegalStateException.class, () -> sim.run(twoSubFlatWorld(10), List.of(broken, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
					}

					@Override
					public void onMatchEnd() {
					}
				}));
	}

	@Test
	void throwingControllerForfeitsCleanlyWhenGuarded() {
		var listenerEnded = new AtomicBoolean(false);
		var opponentMatchEnd = new AtomicBoolean(false);
		var lastSubs = new AtomicReference<List<SubmarineSnapshot>>();

		var broken = new SubmarineController() {
			@Override
			public String name() {
				return "Broken";
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				if (input.tick() == 3) {
					throw new IllegalStateException("controller crash");
				}
			}

			@Override
			public void onMatchEnd(MatchResult result) {
				throw new RuntimeException("onMatchEnd crash too");
			}
		};

		var calm = new SubmarineController() {
			@Override
			public String name() {
				return "Calm";
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setThrottle(0.2);
				output.setBallast(0.5);
			}

			@Override
			public void onMatchEnd(MatchResult result) {
				opponentMatchEnd.set(!result.forfeited());
			}
		};

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		sim.setControllerLimits(ControllerLimits.leagueDefaults());
		sim.run(twoSubFlatWorld(100), List.of(broken, calm),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
						lastSubs.set(subs);
					}

					@Override
					public void onMatchEnd() {
						listenerEnded.set(true);
					}
				});

		assertTrue(lastSubs.get().get(0).forfeited(), "Throwing controller should be forfeited");
		assertFalse(lastSubs.get().get(1).forfeited(), "Opponent should remain un-forfeited");
		assertTrue(sim.forfeitReason(0).contains("onTick threw IllegalStateException"));
		assertEquals("", sim.forfeitReason(1));
		assertTrue(opponentMatchEnd.get(), "Opponent onMatchEnd must still run");
		assertTrue(listenerEnded.get(), "Listener onMatchEnd must always run");
	}

	@Test
	void throwingOnMatchStartAndTorpedoCallbacksForfeitCleanly() {
		var badStart = new SubmarineController() {
			@Override
			public void onMatchStart(MatchContext context) {
				throw new RuntimeException("bad init");
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
			}
		};

		var sim1 = new SimulationLoop();
		sim1.setSpeedMultiplier(1e9);
		sim1.setControllerLimits(ControllerLimits.leagueDefaults());
		sim1.run(twoSubFlatWorld(50), List.of(badStart, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
					}

					@Override
					public void onMatchEnd() {
					}
				});
		assertTrue(sim1.forfeitReason(0).contains("onMatchStart threw RuntimeException"));

		var badTorpSub = new SubmarineController() {
			boolean fired;

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setThrottle(0.2);
				output.setBallast(0.5);
				if (!fired) {
					fired = true;
					output.launchTorpedo(new TorpedoLaunchCommand(0, 0, Double.NaN, "M".repeat(8000)));
				}
			}

			@Override
			public TorpedoController createTorpedoController() {
				return new TorpedoController() {
					@Override
					public void onLaunch(TorpedoLaunchContext context) {
						assertTrue(context.missionData().length() <= SubmarineEntity.MAX_MISSION_DATA_LENGTH);
					}

					@Override
					public void onTick(TorpedoInput input, TorpedoOutput output) {
						throw new IllegalArgumentException("bad torp tick");
					}
				};
			}
		};

		var sim2 = new SimulationLoop();
		sim2.setSpeedMultiplier(1e9);
		sim2.setControllerLimits(ControllerLimits.leagueDefaults());
		sim2.run(twoSubFlatWorld(500), List.of(badTorpSub, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
					}

					@Override
					public void onMatchEnd() {
					}
				});
		assertTrue(sim2.forfeitReason(0).contains("torpedo.onTick threw IllegalArgumentException"));
	}

	@Test
	void matchCpuBudgetForfeitsHeavyController() {
		var heavy = new SubmarineController() {
			double sink;

			@Override
			public String name() {
				return "Heavy";
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				for (int i = 0; i < 200_000; i++) {
					sink += Math.sin(i + input.tick());
				}
				output.setThrottle(0.2 + sink * 0.0);
				output.setBallast(0.5);
			}
		};

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		// Tight 5 ms cumulative CPU budget so test finishes fast and deterministically
		sim.setControllerLimits(new ControllerLimits(true, 0L, 5_000_000L, 0));
		var lastSubs = new AtomicReference<List<SubmarineSnapshot>>();
		sim.run(twoSubFlatWorld(500), List.of(heavy, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
						lastSubs.set(subs);
					}

					@Override
					public void onMatchEnd() {
					}
				});

		assertTrue(lastSubs.get().get(0).forfeited(), "CPU-heavy controller must be forfeited");
		assertFalse(lastSubs.get().get(1).forfeited(), "Light opponent must survive");
		assertTrue(sim.forfeitReason(0).contains("Match CPU budget exceeded"));
		assertTrue(sim.controllerCpuNanos(0) >= 5_000_000L);
	}

	@Test
	void perTickCpuBudgetForfeitsAfterAllowedOverruns() {
		var ticksSeen = new AtomicInteger();
		var slow = new SubmarineController() {
			double sink;

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				ticksSeen.incrementAndGet();
				for (int i = 0; i < 150_000; i++) {
					sink += Math.cos(i + input.tick());
				}
				output.setThrottle(0.2 + sink * 0.0);
				output.setBallast(0.5);
			}
		};

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		// 100 us/tick limit, allow 2 overruns -> forfeited on 3rd slow tick
		sim.setControllerLimits(new ControllerLimits(true, 100_000L, 0L, 2));
		sim.run(twoSubFlatWorld(200), List.of(slow, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
					}

					@Override
					public void onMatchEnd() {
					}
				});

		assertTrue(sim.forfeitReason(0).contains("Per-tick CPU limit exceeded"));
		assertTrue(ticksSeen.get() >= 3 && ticksSeen.get() < 50,
				"Should forfeit promptly after allowed overruns, ran " + ticksSeen.get());
	}

	@Test
	void replayWriterEnforcesMaxBytesCleanly(@TempDir
	Path tempDir) throws Exception {
		Path replay = tempDir.resolve("capped.srl");
		var world = twoSubFlatWorld(200);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);

		long capBytes = 4096L;
		try (var writer = new ReplayWriter(world.config(), world.spawnPoints(), replay, capBytes)) {
			sim.run(world, List.of(calmSub("Alpha"), calmSub("Beta")),
					List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), writer);
			assertTrue(writer.sizeLimitExceeded(), "Tiny byte cap should trigger");
			assertTrue(writer.bytesWritten() <= capBytes);
		}

		// File stays near the cap (cap + final E line) and remains readable by ReplayReader
		assertTrue(Files.size(replay) <= capBytes + 16);
		var reader = new ReplayReader(replay);
		assertEquals(2, reader.header().submarines().size());
		assertFalse(reader.readAll().isEmpty());
	}

	@Test
	void slowOnMatchStartDoesNotTriggerTickZeroPerTickOverrun() {
		var threadMx = java.lang.management.ManagementFactory.getThreadMXBean();
		boolean cpuSupported = threadMx.isCurrentThreadCpuTimeSupported() && threadMx.isThreadCpuTimeEnabled();
		var heavyInit = new SubmarineController() {
			double sink;

			@Override
			public String name() {
				return "HeavyInit";
			}

			@Override
			public void onMatchStart(MatchContext context) {
				// Burn at least one full OS thread-CPU quantum (>20 ms) so onMatchStart is guaranteed
				// to exceed the 5 ms per-tick cap below if it were wrongly billed to tick 0.
				long start = cpuSupported ? threadMx.getCurrentThreadCpuTime() : System.nanoTime();
				long targetNanos = 25_000_000L;
				int iter = 0;
				while ((cpuSupported ? threadMx.getCurrentThreadCpuTime() : System.nanoTime()) - start < targetNanos) {
					for (int i = 0; i < 10_000; i++) {
						sink += Math.sin(iter++);
					}
				}
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setThrottle(0.2 + sink * 0.0);
				output.setBallast(0.5);
			}
		};

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		// 5 ms per-tick cap with 0 allowed overruns, generous cumulative match cap.
		// If onMatchStart (>25 ms) were billed to tick 0's per-tick budget, tick 0 would forfeit.
		sim.setControllerLimits(new ControllerLimits(true, 5_000_000L, 30_000_000_000L, 0));
		var lastSubs = new AtomicReference<List<SubmarineSnapshot>>();
		sim.run(twoSubFlatWorld(20), List.of(heavyInit, calmSub("Calm")),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
						lastSubs.set(subs);
					}

					@Override
					public void onMatchEnd() {
					}
				});

		assertFalse(lastSubs.get().get(0).forfeited(),
				"One-time onMatchStart initialization must not trigger a tick-0 per-tick overrun: "
						+ sim.forfeitReason(0));
		assertEquals("", sim.forfeitReason(0));
		assertTrue(sim.controllerCpuNanos(0) >= 20_000_000L,
				"onMatchStart CPU time should still count toward cumulative match CPU, got "
						+ sim.controllerCpuNanos(0));
	}

	@Test
	void repeatedAndConcurrentSimulationRunsAreBitIdenticalWithTorpedoes() throws Exception {
		var world = twoSubFlatWorld(300);
		long baseline = runAndHash(world);
		assertNotEquals(0x9E3779B97F4A7C15L, baseline, "Simulation must execute ticks");
		assertEquals(baseline, runAndHash(world), "Sequential runs must produce bit-identical trajectories");

		try (var pool = java.util.concurrent.Executors.newFixedThreadPool(4)) {
			var futures = new java.util.ArrayList<java.util.concurrent.Future<Long>>();
			for (int i = 0; i < 4; i++) {
				futures.add(pool.submit(() -> runAndHash(world)));
			}
			for (var f : futures) {
				assertEquals(baseline, f.get(), "Concurrent runs must match sequential trajectory bit-for-bit");
			}
		}
	}

	private static long runAndHash(GeneratedWorld world) {
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		var shooter = new SubmarineController() {
			boolean fired;

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
				output.setThrottle(0.5);
				output.setRudder(0.1);
				output.setBallast(0.5);
				if (!fired && input.tick() == 10) {
					fired = true;
					output.launchTorpedo(new TorpedoLaunchCommand(0, 0, 20.0, "test"));
				}
			}
		};
		long[] hash = {0x9E3779B97F4A7C15L};
		sim.run(world, List.of(shooter, calmSub("Calm")), List.of(VehicleConfig.submarine(), VehicleConfig.submarine()),
				new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
						long h = hash[0] ^ tick;
						for (var s : subs) {
							h = mix(h, s.id());
							h = mix(h, Double.doubleToRawLongBits(s.pose().position().x()));
							h = mix(h, Double.doubleToRawLongBits(s.pose().position().y()));
							h = mix(h, Double.doubleToRawLongBits(s.pose().position().z()));
							h = mix(h, Double.doubleToRawLongBits(s.pose().heading()));
							h = mix(h, Double.doubleToRawLongBits(s.speed()));
							h = mix(h, s.hp());
						}
						for (var t : torps) {
							h = mix(h, t.id());
							h = mix(h, Double.doubleToRawLongBits(t.pose().position().x()));
							h = mix(h, Double.doubleToRawLongBits(t.pose().position().y()));
							h = mix(h, Double.doubleToRawLongBits(t.pose().position().z()));
							h = mix(h, Double.doubleToRawLongBits(t.speed()));
						}
						hash[0] = h;
					}

					@Override
					public void onMatchEnd() {
					}
				});
		return hash[0];
	}

	private static long mix(long h, long v) {
		h ^= v + 0x9E3779B97F4A7C15L + (h << 6) + (h >>> 2);
		return h;
	}

	@Test
	void proximityFuseUsesCachedDisplayNameWithoutCallingController() {
		var config = MatchConfig.withDefaults(42);
		var afterConstruction = new AtomicBoolean(false);
		var hostileName = new SubmarineController() {
			@Override
			public String name() {
				if (afterConstruction.get()) {
					throw new IllegalStateException("name() should not be invoked during proximity fuse check");
				}
				return "Hostile";
			}

			@Override
			public void onTick(SubmarineInput input, SubmarineOutput output) {
			}
		};

		var subWithHostileName = new SubmarineEntity(VehicleConfig.submarine(), 0, hostileName, new Vec3(0, 0, -200), 0,
				Color.RED, 1000);
		afterConstruction.set(true);
		var torp1 = new TorpedoEntity(1000, 1, VehicleConfig.torpedo(), null, new Vec3(0, 10, -200), Math.PI, 0, 20.0,
				Color.GREEN);
		for (int i = 0; i < 100; i++) {
			torp1.decrementArmingDelay();
		}
		SimulationLoop.checkProximityFuse(torp1, List.of(subWithHostileName), config, 300);
		assertTrue(torp1.detonated(), "Armed torpedo near hull must detonate");
		assertTrue(subWithHostileName.hp() < 1000, "Detonation must damage target");

		var subWithoutController = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, -200), 0,
				Color.BLUE, 1000);
		var torp2 = new TorpedoEntity(1001, 1, VehicleConfig.torpedo(), null, new Vec3(0, 10, -200), Math.PI, 0, 20.0,
				Color.GREEN);
		for (int i = 0; i < 100; i++) {
			torp2.decrementArmingDelay();
		}
		SimulationLoop.checkProximityFuse(torp2, List.of(subWithoutController), config, 300);
		assertTrue(torp2.detonated(), "Proximity fuse must handle entities with a null controller");
	}
}
