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
package se.hirt.searobots.engine.replay;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Diagnostic harness (not an assertion test): finds a match where torpedoes are fired but no clean kill results, then
 * explains why each torpedo missed.
 * <p>
 * For every torpedo it tracks the minimum <em>bow-to-hull</em> distance to each enemy submarine over the torpedo's
 * whole life: this is exactly the metric the proximity fuse compares against the fuse radius (5-30m), so the closest
 * approach tells the story directly. It also records the torpedo's fate, fuel and speed at death, and the guidance
 * phase it was in, which separates the miss causes: ran out of fuel, stalled, hit terrain, or flew past outside the
 * fuse radius (a terminal-geometry miss).
 * <p>
 * Assertion-free dev tool: skipped on normal builds, run on demand with
 * {@code mvn test -Dsearobots.diag=true -Dtest=TorpedoMissAnalysis}.
 */
@EnabledIfSystemProperty(named = "searobots.diag", matches = "true")
class TorpedoMissAnalysis {

	private static final long[] SEEDS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 15};
	private static final int MAX_TICKS = 30_000; // ~10 min cap

	/** Accumulated facts about one torpedo over its lifetime. */
	private static final class TorpTrack {
		int id;
		int ownerId;
		long launchTick = -1;
		double launchX, launchY, launchZ;
		double targetX, targetY, targetZ;
		// enemy's TRUE position at launch, and how far the aim point was from it
		double enemyXAtLaunch, enemyYAtLaunch, enemyZAtLaunch;
		double aimErrorAtLaunch = Double.NaN;
		double minBowDistToEnemy = Double.MAX_VALUE;
		long minApproachTick;
		double approachX, approachY, approachZ;
		// last observed state (i.e. at/just before death)
		long lastTick;
		double lastFuel, lastSpeed, lastZ;
		String lastPhase = "";
		boolean detonated;
		boolean wasAlive = true;
	}

	@Test
	void analyzeTorpedoMisses() {
		var missSeeds = new java.util.ArrayList<long[]>();   // {seed} list
		var missTracks = new java.util.ArrayList<Map<Integer, TorpTrack>>();

		for (long seed : SEEDS) {
			Result r = runSeed(seed);
			int launches = r.tracks.size();
			long enemyKills = r.tracks.values().stream().filter(t -> t.detonated)
					.filter(t -> t.minBowDistToEnemy < 35.0) // within plausible fuse+blast band
					.count();
			System.out.printf(Locale.US, "seed %-3d  launches=%-2d  subDeaths=%d  (torpedoes that hit near a sub=%d)%n",
					seed, launches, r.subDeaths, enemyKills);

			// Want: torpedoes fired, but no clean kill (no sub died).
			if (launches > 0 && r.subDeaths == 0) {
				missSeeds.add(new long[] {seed});
				missTracks.add(r.tracks);
			}
		}

		if (missSeeds.isEmpty()) {
			System.out.println("No 'fired-but-no-kill' seed found in the scanned set.");
			return;
		}

		for (int si = 0; si < missSeeds.size(); si++) {
			analyzeSeed(missSeeds.get(si)[0], missTracks.get(si));
		}
	}

	private void analyzeSeed(long seed, Map<Integer, TorpTrack> tracks) {
		System.out.printf(Locale.US, "%n===== ANALYSIS: seed %d (torpedoes fired, no kill) =====%n", seed);
		for (TorpTrack t : tracks.values().stream().sorted((a, b) -> Long.compare(a.launchTick, b.launchTick))
				.toList()) {
			String fate;
			if (t.detonated) {
				fate = t.minBowDistToEnemy < 35.0 ? "DETONATED near sub" : "DETONATED (not near a sub)";
			} else if (t.lastSpeed < 3.0) {
				fate = "STALLED/SANK (lost control authority)";
			} else if (t.lastFuel <= 0) {
				fate = "FUEL DEPLETED";
			} else {
				fate = "LOST (terrain/boundary)";
			}
			System.out.printf(Locale.US,
					"  torp %d (owner %d): launch t=%d at (%.0f,%.0f,%.0f) -> aim point (%.0f,%.0f,%.0f)%n", t.id,
					t.ownerId, t.launchTick, t.launchX, t.launchY, t.launchZ, t.targetX, t.targetY, t.targetZ);
			System.out.printf(Locale.US,
					"      enemy TRUE pos at launch: (%.0f,%.0f,%.0f) -> firing-solution error: %.0fm%n",
					t.enemyXAtLaunch, t.enemyYAtLaunch, t.enemyZAtLaunch, t.aimErrorAtLaunch);
			System.out.printf(Locale.US,
					"      closest bow-to-hull approach to enemy: %.1fm at t=%d (pos %.0f,%.0f,%.0f)%n",
					t.minBowDistToEnemy, t.minApproachTick, t.approachX, t.approachY, t.approachZ);
			System.out.printf(Locale.US, "      fate: %s | last t=%d fuel=%.1fs speed=%.1f z=%.0f phase=%s%n", fate,
					t.lastTick, t.lastFuel, t.lastSpeed, t.lastZ, t.lastPhase);
			System.out.println("      => MISS CAUSE: " + diagnose(t));
		}
	}

	private static String diagnose(TorpTrack t) {
		double fuse = 30.0; // max fuse radius; min is 5m
		if (t.minBowDistToEnemy <= fuse) {
			return String.format(Locale.US,
					"came within fuse range (%.1fm <= %.0fm) but did not detonate - possible arming/timing or owner-skip issue",
					t.minBowDistToEnemy, fuse);
		}
		if (!t.detonated && t.lastFuel <= 0) {
			return String.format(Locale.US, "ran out of fuel before reaching target; closest it ever got was %.0fm",
					t.minBowDistToEnemy);
		}
		if (!t.detonated && t.lastSpeed < 3.0) {
			return "stalled (speed below 3 m/s) and sank, losing control authority before intercept";
		}
		if (!t.detonated) {
			return String.format(Locale.US, "terminated on terrain/boundary while still %.0fm from the target",
					t.minBowDistToEnemy);
		}
		return String.format(Locale.US,
				"detonated %.0fm from the enemy hull - terminal geometry miss (passed outside the %.0fm fuse radius)",
				t.minBowDistToEnemy, fuse);
	}

	private record Result(Map<Integer, TorpTrack> tracks, int subDeaths) {
	}

	private Result runSeed(long seed) {
		MatchConfig base = MatchConfig.withDefaults(seed);
		MatchConfig config = new MatchConfig(base.worldSeed(), base.tickRateHz(), MAX_TICKS, base.submarineCount(),
				base.torpedoCount(), base.startingHp(), base.blastRadius(), base.minFuseRadius(), base.maxFuseRadius(),
				base.ratedDepth(), base.crushDepth(), base.battleArea(), base.terrainMarginMeters(),
				base.gridCellMeters(), base.minSeaFloorZ(), base.maxSeaFloorZ(), base.maxSubSpeed(), base.startTime());

		var world = new WorldGenerator().generate(config);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);

		List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
		List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		Map<Integer, TorpTrack> tracks = new HashMap<>();
		int[] subDeaths = {0};
		boolean[] deadSubsSeen = new boolean[8];

		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				for (int i = 0; i < subs.size(); i++) {
					var s = subs.get(i);
					if ((s.hp() <= 0 || s.forfeited()) && i < deadSubsSeen.length && !deadSubsSeen[i]) {
						deadSubsSeen[i] = true;
						subDeaths[0]++;
					}
				}
				for (var torp : torps) {
					TorpTrack t = tracks.computeIfAbsent(torp.id(), id -> {
						var nt = new TorpTrack();
						nt.id = torp.id();
						nt.ownerId = torp.ownerId();
						nt.launchTick = tick;
						nt.launchX = torp.pose().position().x();
						nt.launchY = torp.pose().position().y();
						nt.launchZ = torp.pose().position().z();
						nt.targetX = torp.targetX();
						nt.targetY = torp.targetY();
						nt.targetZ = torp.targetZ();
						// Enemy true position at launch -> firing-solution error.
						for (var sub : subs) {
							if (sub.id() == torp.ownerId())
								continue;
							var ep = sub.pose().position();
							nt.enemyXAtLaunch = ep.x();
							nt.enemyYAtLaunch = ep.y();
							nt.enemyZAtLaunch = ep.z();
							double dx = torp.targetX() - ep.x();
							double dy = torp.targetY() - ep.y();
							double dz = torp.targetZ() - ep.z();
							nt.aimErrorAtLaunch = Math.sqrt(dx * dx + dy * dy + dz * dz);
						}
						return nt;
					});
					t.lastTick = tick;
					t.lastFuel = torp.fuelRemaining();
					t.lastSpeed = torp.speed();
					t.lastZ = torp.pose().position().z();
					t.lastPhase = torp.diagPhase() == null ? "" : torp.diagPhase();
					if (torp.detonated())
						t.detonated = true;
					for (var sub : subs) {
						if (sub.id() == torp.ownerId())
							continue;
						double d = HullGeometry.bowDistanceToHull(torp, sub);
						if (d < t.minBowDistToEnemy) {
							t.minBowDistToEnemy = d;
							t.minApproachTick = tick;
							t.approachX = torp.pose().position().x();
							t.approachY = torp.pose().position().y();
							t.approachZ = torp.pose().position().z();
						}
					}
				}
			}

			@Override
			public void onMatchEnd() {
			}
		};

		sim.run(world, controllers, vehicles, listener);
		return new Result(tracks, subDeaths[0]);
	}
}
