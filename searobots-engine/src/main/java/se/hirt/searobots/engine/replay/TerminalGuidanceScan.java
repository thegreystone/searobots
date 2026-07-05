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

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Headless diagnostic: runs Codex vs Claude across many seeds and measures per-side <em>terminal guidance</em> quality,
 * without writing replay files. For every torpedo it records the closest it ever came to the enemy hull and whether it
 * detonated, so a systematic guidance problem (fish that track but can never close the last hundred metres) shows up as
 * a side whose torpedoes cluster far from the target and rarely detonate.
 * <p>
 * Usage: {@code TerminalGuidanceScan [seed ...]} (defaults to a fixed spread of seeds). Sub 0 is Codex, sub 1 is Claude.
 */
public final class TerminalGuidanceScan {

	private static final long[] DEFAULT_SEEDS = {1, 2, 3, 5, 7, 11, 13, 17, 23, 42, 99, 101, 404, 777, 1234, 31337};
	private static final int MAX_TICKS = 30_000;
	private static final double HIT_NEAR = 80.0; // detonation within this of the enemy centre counts as "on target"

	private TerminalGuidanceScan() {
	}

	private static final class Torp {
		int owner;
		double minDist = Double.MAX_VALUE;
		boolean detonated;
		String lastPhase = "";
		long launchTick = -1, endTick = -1;
	}

	private record SeedResult(long seed, int winner, int codexFired, int codexDet, int codexHit, double codexBest,
	                          int claudeFired, int claudeDet, int claudeHit, double claudeBest,
	                          List<Double> codexApproaches, List<Double> claudeApproaches) {
	}

	public static void main(String[] args) {
		long[] seeds = args.length > 0 ? parseSeeds(args) : DEFAULT_SEEDS;

		List<SeedResult> results = new ArrayList<>();
		for (long seed : seeds) {
			results.add(runSeed(seed));
		}

		System.out.println();
		System.out.printf("%-8s %-8s | %-26s | %-26s%n", "seed", "winner", "CODEX fired/det/hit best", "CLAUDE fired/det/hit best");
		System.out.println("-".repeat(78));
		int codexWins = 0, claudeWins = 0, draws = 0;
		List<Double> allCodex = new ArrayList<>(), allClaude = new ArrayList<>();
		int cFired = 0, cDet = 0, cHit = 0, clFired = 0, clDet = 0, clHit = 0;
		for (var r : results) {
			String w = r.winner == 0 ? "Codex" : r.winner == 1 ? "Claude" : "draw";
			if (r.winner == 0) codexWins++; else if (r.winner == 1) claudeWins++; else draws++;
			System.out.printf("%-8d %-8s | %2d /%2d /%2d  best=%-7s | %2d /%2d /%2d  best=%-7s%n", r.seed, w,
					r.codexFired, r.codexDet, r.codexHit, fmt(r.codexBest), r.claudeFired, r.claudeDet, r.claudeHit,
					fmt(r.claudeBest));
			allCodex.addAll(r.codexApproaches); allClaude.addAll(r.claudeApproaches);
			cFired += r.codexFired; cDet += r.codexDet; cHit += r.codexHit;
			clFired += r.claudeFired; clDet += r.claudeDet; clHit += r.claudeHit;
		}

		System.out.println();
		System.out.printf("Wins: Codex %d, Claude %d, draw/timeout %d  (of %d seeds)%n", codexWins, claudeWins, draws,
				seeds.length);
		System.out.println();
		System.out.printf("%-8s %8s %8s %8s %10s %10s %10s%n", "side", "fired", "deton", "on-tgt", "hit%", "med.appr",
				"min.appr");
		reportSide("CODEX", cFired, cDet, cHit, allCodex);
		reportSide("CLAUDE", clFired, clDet, clHit, allClaude);
		System.out.println();
		System.out.println("appr = closest a torpedo came to the enemy centre (m). A side whose torpedoes reach small");
		System.out.println("approaches but rarely detonate points at fuse/terminal-lock; large approaches point at homing.");
	}

	private static SeedResult runSeed(long seed) {
		MatchConfig config = withDuration(MatchConfig.withDefaults(seed), MAX_TICKS);
		var world = new WorldGenerator().generate(config);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);

		List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
		List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		Map<Integer, Torp> torps = new HashMap<>();
		int[] finalHp = {config.startingHp(), config.startingHp()};

		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> tps) {
				Map<Integer, SubmarineSnapshot> byId = new HashMap<>();
				for (var s : subs) {
					byId.put(s.id(), s);
					finalHp[s.id()] = s.hp();
				}
				for (var t : tps) {
					var rec = torps.computeIfAbsent(t.id(), k -> {
						var nt = new Torp();
						nt.owner = t.ownerId();
						nt.launchTick = tick;
						return nt;
					});
					rec.endTick = tick;
					if (t.detonated()) rec.detonated = true;
					if (t.diagPhase() != null && !t.diagPhase().isEmpty()) rec.lastPhase = t.diagPhase();
					var enemy = byId.get(1 - t.ownerId());
					if (enemy != null) {
						double dx = t.pose().position().x() - enemy.pose().position().x();
						double dy = t.pose().position().y() - enemy.pose().position().y();
						double dz = t.pose().position().z() - enemy.pose().position().z();
						double d = Math.sqrt(dx * dx + dy * dy + dz * dz);
						if (d < rec.minDist) rec.minDist = d;
					}
				}
				boolean dead = subs.stream().anyMatch(s -> s.hp() <= 0);
				if (dead) sim.stop();
			}

			@Override
			public void onMatchEnd() {
			}
		};

		sim.run(world, controllers, vehicles, listener);

		// tally
		int cf = 0, cd = 0, ch = 0, lf = 0, ld = 0, lh = 0;
		double cb = Double.MAX_VALUE, lb = Double.MAX_VALUE;
		List<Double> ca = new ArrayList<>(), la = new ArrayList<>();
		for (var t : torps.values()) {
			boolean hit = t.detonated && t.minDist < HIT_NEAR;
			if (t.owner == 0) {
				cf++; if (t.detonated) cd++; if (hit) ch++; cb = Math.min(cb, t.minDist); ca.add(t.minDist);
			} else {
				lf++; if (t.detonated) ld++; if (hit) lh++; lb = Math.min(lb, t.minDist); la.add(t.minDist);
			}
		}
		int winner = -1;
		if (finalHp[1] <= 0 && finalHp[0] > 0) winner = 0;
		else if (finalHp[0] <= 0 && finalHp[1] > 0) winner = 1;
		System.out.printf("  seed %d done: Codex hp=%d, Claude hp=%d, torps=%d%n", seed, finalHp[0], finalHp[1],
				torps.size());
		return new SeedResult(seed, winner, cf, cd, ch, cb, lf, ld, lh, lb, ca, la);
	}

	private static void reportSide(String name, int fired, int det, int hit, List<Double> appr) {
		double med = median(appr), min = appr.isEmpty() ? Double.NaN : appr.stream().mapToDouble(d -> d).min().orElse(Double.NaN);
		double hitPct = fired == 0 ? 0 : 100.0 * hit / fired;
		System.out.printf("%-8s %8d %8d %8d %9.0f%% %10s %10s%n", name, fired, det, hit, hitPct, fmt(med), fmt(min));
	}

	private static double median(List<Double> xs) {
		if (xs.isEmpty()) return Double.NaN;
		var s = new ArrayList<>(xs);
		s.sort(Double::compare);
		int n = s.size();
		return n % 2 == 1 ? s.get(n / 2) : (s.get(n / 2 - 1) + s.get(n / 2)) / 2;
	}

	private static String fmt(double d) {
		return (d == Double.MAX_VALUE || Double.isNaN(d)) ? "-" : String.format("%.0fm", d);
	}

	private static long[] parseSeeds(String[] args) {
		long[] s = new long[args.length];
		for (int i = 0; i < args.length; i++) s[i] = Long.parseLong(args[i]);
		return s;
	}

	private static MatchConfig withDuration(MatchConfig b, int durationTicks) {
		return new MatchConfig(b.worldSeed(), b.tickRateHz(), durationTicks, b.submarineCount(), b.torpedoCount(),
				b.startingHp(), b.blastRadius(), b.minFuseRadius(), b.maxFuseRadius(), b.ratedDepth(), b.crushDepth(),
				b.battleArea(), b.terrainMarginMeters(), b.gridCellMeters(), b.minSeaFloorZ(), b.maxSeaFloorZ(),
				b.maxSubSpeed(), b.startTime());
	}
}
