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

import java.util.List;
import java.util.Locale;

/**
 * Diagnostic harness focused on the Claude controller's combat conversion: why Claude so rarely fires torpedoes. It
 * reconstructs Claude's firing gates from the data Claude itself publishes each tick (status mode, contact estimate
 * with uncertaintyRadius and ping/passive label, and the firing solution), plus the true geometry, so no change to the
 * controller is needed.
 * <p>
 * Claude (index 1) only fires when ALL hold simultaneously: mode==CHASE, contactAlive>=0.5, true-track distance in
 * [800,2000]m, uncertaintyRadius<=300 (effectively requires a fresh active ping, since passive is floored ~450), and
 * heading aligned within 25 degrees. The counters below show which link is the binding constraint.
 * <p>
 * Assertion-free dev tool: skipped on normal builds, run on demand with
 * {@code mvn test -Dsearobots.diag=true -Dtest=ClaudeFireDiagnostics}.
 */
@EnabledIfSystemProperty(named = "searobots.diag", matches = "true")
class ClaudeFireDiagnostics {

	private static final long[] SEEDS = {1, 3, 5, 9, 13};
	private static final int MAX_TICKS = 30_000;
	private static final int CLAUDE = 1; // controllers = [Codex(0), Claude(1)]

	private static final class Stats {
		long ticks;
		long ticksChase, ticksTrack, ticksPatrolOrObj;
		long ticksWithContact;
		long ticksContactAlive50;          // contactAlive >= 0.5
		long ticksPingFresh;               // contact label == "ping" (range active-confirmed)
		double minUncertainty = Double.MAX_VALUE;
		long ticksInTrueWindow;            // true enemy dist in [800,2000]
		long ticksFireSolutionPublished;   // Claude says "I'd fire now if I could"
		long ticksAllGatesButHeading;      // CHASE & alive>=.5 & dist 800-2000 & uncertOK
		double closestTrueApproach = Double.MAX_VALUE;
		int torpedoesFired;
	}

	@Test
	void whyClaudeDoesNotFire() {
		for (long seed : SEEDS) {
			Stats s = runSeed(seed);
			System.out.printf(Locale.US, "%n===== seed %d (Claude=owner %d) =====%n", seed, CLAUDE);
			System.out.printf(Locale.US, "  match ticks: %d%n", s.ticks);
			System.out.printf(Locale.US, "  mode time:  CHASE %.1f%%   TRACK %.1f%%   PATROL/OBJ %.1f%%%n",
					pct(s.ticksChase, s.ticks), pct(s.ticksTrack, s.ticks), pct(s.ticksPatrolOrObj, s.ticks));
			System.out.printf(Locale.US,
					"  has contact: %.1f%%   contactAlive>=0.5: %.1f%%   fresh active ping: %.1f%%%n",
					pct(s.ticksWithContact, s.ticks), pct(s.ticksContactAlive50, s.ticks),
					pct(s.ticksPingFresh, s.ticks));
			System.out.printf(Locale.US, "  best (min) uncertaintyRadius seen: %.0fm (firing needs <=300)%n",
					s.minUncertainty == Double.MAX_VALUE ? -1 : s.minUncertainty);
			System.out.printf(Locale.US, "  in TRUE 800-2000m firing window: %.1f%%   closest true approach: %.0fm%n",
					pct(s.ticksInTrueWindow, s.ticks), s.closestTrueApproach);
			System.out.printf(Locale.US, "  firing SOLUTION published (all gates but heading/cooldown): %d ticks%n",
					s.ticksFireSolutionPublished);
			System.out.printf(Locale.US, "  all gates but heading-alignment: %d ticks%n", s.ticksAllGatesButHeading);
			System.out.printf(Locale.US, "  >>> Claude torpedoes fired: %d%n", s.torpedoesFired);
			System.out.println("  >>> binding constraint: " + diagnose(s));
		}
	}

	private static String diagnose(Stats s) {
		if (s.torpedoesFired > 0) {
			return "fired " + s.torpedoesFired + " - conversion working at least sometimes";
		}
		if (s.ticksChase == 0) {
			return "NEVER reached CHASE mode (needs active-confirmed range, or dist<3500 & alive>0.18, " + "or uncertainty<300) - tracking never firmed up enough to commit";
		}
		if (s.ticksInTrueWindow == 0) {
			return String.format(Locale.US,
					"never closed into the 800-2000m firing window (closest %.0fm) - approach/closure too slow",
					s.closestTrueApproach);
		}
		if (s.minUncertainty > 300 && s.ticksPingFresh == 0) {
			return "range never confirmed: passive uncertaintyRadius floored ~450 and Claude never got a " + "fresh active ping while in window - the uncertainty<=300 gate is the blocker";
		}
		if (s.ticksFireSolutionPublished == 0) {
			return "all individual gates occur but never SIMULTANEOUSLY (CHASE + alive + window + low " + "uncertainty never coincide) - the conjunction is too tight";
		}
		return "had a fireable solution for " + s.ticksFireSolutionPublished + " ticks but the 25-degree heading-alignment (or refire cooldown) blocked every shot";
	}

	private static double pct(long n, long d) {
		return d == 0 ? 0 : 100.0 * n / d;
	}

	private Stats runSeed(long seed) {
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

		Stats st = new Stats();
		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				SubmarineSnapshot claude = null, enemy = null;
				for (var s : subs) {
					if (s.id() == CLAUDE)
						claude = s;
					else
						enemy = s;
				}
				if (claude == null || enemy == null)
					return;
				st.ticks++;

				String status = claude.status() == null ? "" : claude.status();
				char mode = status.isEmpty() ? '?' : status.charAt(0);
				boolean chase = mode == 'C';
				if (chase)
					st.ticksChase++;
				else if (mode == 'T')
					st.ticksTrack++;
				else
					st.ticksPatrolOrObj++; // P or OBJ

				// True geometry
				var cp = claude.pose().position();
				var ep = enemy.pose().position();
				double trueDist = Math.hypot(cp.x() - ep.x(), cp.y() - ep.y());
				st.closestTrueApproach = Math.min(st.closestTrueApproach, trueDist);
				boolean inWindow = trueDist >= 800 && trueDist <= 2000;
				if (inWindow)
					st.ticksInTrueWindow++;

				// Claude's published contact estimate (its own track)
				boolean alive50 = false, freshPing = false, uncertOk = false;
				if (!claude.contactEstimates().isEmpty()) {
					st.ticksWithContact++;
					var ce = claude.contactEstimates().get(0);
					if (ce.contactAlive() >= 0.50) {
						alive50 = true;
						st.ticksContactAlive50++;
					}
					boolean ping = "ping".equals(ce.label());
					if (ping) {
						freshPing = true;
						st.ticksPingFresh++;
					}
					st.minUncertainty = Math.min(st.minUncertainty, ce.uncertaintyRadius());
					// firing uncertainty gate: <=300, OR active-confirmed and <=400
					uncertOk = ce.uncertaintyRadius() <= 300 || (ping && ce.uncertaintyRadius() <= 400);
				}

				if (claude.firingSolution() != null)
					st.ticksFireSolutionPublished++;
				if (chase && alive50 && inWindow && uncertOk)
					st.ticksAllGatesButHeading++;
			}

			@Override
			public void onMatchEnd() {
			}
		};

		// Count Claude launches by watching for new torpedo ids owned by Claude.
		var launchCounter = new java.util.HashSet<Integer>();
		var counting = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				for (var t : torps) {
					if (t.ownerId() == CLAUDE && launchCounter.add(t.id()))
						st.torpedoesFired++;
				}
				listener.onTick(tick, subs, torps);
			}

			@Override
			public void onMatchEnd() {
				listener.onMatchEnd();
			}
		};

		sim.run(world, controllers, vehicles, counting);
		return st;
	}
}
