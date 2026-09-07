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

import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Head-to-head outcome scoreboard: Claude (index 1) vs Codex (index 0) over a range of seeds. The bottom line for
 * "conclusively win" work: wins, losses, draws, and how stealthily Claude fought (ping fraction) versus how much it
 * fired. Run on demand with {@code mvn test -Dsearobots.diag=true -Dtest=ClaudeVsCodexWinRate}.
 */
@EnabledIfSystemProperty(named = "searobots.diag", matches = "true")
class ClaudeVsCodexWinRate {

	private static final int SEED_COUNT = 16;
	private static final int SEED_START = Integer.getInteger("searobots.seedStart", 1);
	private static final int MAX_TICKS = 30_000; // ~10 min
	private static final int CLAUDE = 1;
	private static final int CODEX = 0;

	private static final class Outcome {
		int claudeWins, claudeLosses, draws;
		int claudeFiredTotal, codexFiredTotal;
		long claudePingTicks, totalTicks;
	}

	@Test
	void scoreboard() {
		Outcome o = new Outcome();
		var lines = new StringBuilder();
		for (int seed = SEED_START; seed < SEED_START + SEED_COUNT; seed++) {
			SeedResult r = runSeed(seed);
			o.totalTicks += r.ticks;
			o.claudePingTicks += r.claudePingTicks;
			o.claudeFiredTotal += r.claudeFired;
			o.codexFiredTotal += r.codexFired;
			String verdict;
			if (r.codexDead && !r.claudeDead) {
				o.claudeWins++;
				verdict = "WIN ";
			} else if (r.claudeDead && !r.codexDead) {
				o.claudeLosses++;
				verdict = "LOSS";
			} else {
				o.draws++;
				verdict = "draw";
			}
			lines.append(String.format(Locale.US,
					"  seed %-3d %s  claudeFired=%d codexFired=%d  claudePing=%.0f%%  (claudeHp=%d codexHp=%d)%n", seed,
					verdict, r.claudeFired, r.codexFired, r.ticks == 0 ? 0 : 100.0 * r.claudePingTicks / r.ticks,
					r.claudeHp, r.codexHp));
		}
		System.out.println("\n===== Claude vs Codex scoreboard =====");
		System.out.print(lines);
		System.out.printf(Locale.US, "TOTAL over %d seeds:  Claude WINS=%d  LOSSES=%d  DRAWS=%d%n", SEED_COUNT,
				o.claudeWins, o.claudeLosses, o.draws);
		System.out.printf(Locale.US,
				"  Claude torpedoes fired=%d   Codex torpedoes fired=%d   Claude ping fraction=%.1f%%%n",
				o.claudeFiredTotal, o.codexFiredTotal,
				o.totalTicks == 0 ? 0 : 100.0 * o.claudePingTicks / o.totalTicks);
	}

	private record SeedResult(long ticks, boolean claudeDead, boolean codexDead, int claudeHp, int codexHp,
	                          int claudeFired, int codexFired, long claudePingTicks) {
	}

	private SeedResult runSeed(int seed) {
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

		long[] ticks = {0};
		boolean[] claudeDead = {false}, codexDead = {false};
		int[] claudeHp = {base.startingHp()}, codexHp = {base.startingHp()};
		long[] claudePingTicks = {0};
		Set<Integer> claudeTorps = new HashSet<>(), codexTorps = new HashSet<>();

		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				ticks[0] = tick + 1;
				for (var s : subs) {
					boolean dead = s.hp() <= 0 || s.forfeited();
					if (s.id() == CLAUDE) {
						claudeHp[0] = s.hp();
						if (dead)
							claudeDead[0] = true;
						// "ping" label means Claude's range is active-confirmed this tick
						if (!s.contactEstimates().isEmpty() && "ping".equals(s.contactEstimates().get(0).label())) {
							claudePingTicks[0]++;
						}
					} else if (s.id() == CODEX) {
						codexHp[0] = s.hp();
						if (dead)
							codexDead[0] = true;
					}
				}
				for (var t : torps) {
					if (t.ownerId() == CLAUDE)
						claudeTorps.add(t.id());
					else if (t.ownerId() == CODEX)
						codexTorps.add(t.id());
				}
			}

			@Override
			public void onMatchEnd() {
			}
		};

		sim.run(world, controllers, vehicles, listener);
		return new SeedResult(ticks[0], claudeDead[0], codexDead[0], claudeHp[0], codexHp[0], claudeTorps.size(),
				codexTorps.size(), claudePingTicks[0]);
	}
}
