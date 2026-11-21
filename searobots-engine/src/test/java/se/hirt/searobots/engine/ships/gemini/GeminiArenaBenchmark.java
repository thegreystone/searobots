/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.HullGeometry;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineCompetition;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Supplier;

/**
 * Standalone benchmark and diagnostic runner for iterating on {@link GeminiAttackSub} and
 * {@link GeminiTorpedoController} against Claude and Codex across both Navigation and Combat.
 * <p>
 * Usage:
 * {@code GeminiArenaBenchmark [--master=deadbeef] [--count=10] [--opponent=both|claude|codex] [--nav] [--swap] [--trace-seed=hex]}
 */
public final class GeminiArenaBenchmark {

	private static final int COMBAT_TICKS = 50 * 7200; // 2 hours (ends early once decided)
	private static final int NAV_TICKS = 50 * 2400; // 40 minutes

	record MatchStats(long seed, String nameA, String nameB, int hpA, int hpB, boolean forfeitA, boolean forfeitB,
			int torpsFiredA, int torpsFiredB, int pingsA, int pingsB, long ticks, double avgNoiseA, double avgNoiseB,
			double minTorpDistAToB, double minTorpDistBToA, double minSubDist, String deathInfoA, String deathInfoB,
			String lastStatusA, String lastStatusB) {
		int pointsA() {
			int pts = 0;
			if (hpB <= 0 || forfeitB)
				pts += 5;
			if (hpA > 0 && !forfeitA)
				pts += 5;
			return pts;
		}

		int pointsB() {
			int pts = 0;
			if (hpA <= 0 || forfeitA)
				pts += 5;
			if (hpB > 0 && !forfeitB)
				pts += 5;
			return pts;
		}
	}

	public static void main(String[] args) {
		long masterSeed = 0xdeadbeefL;
		int count = 10;
		String opponent = "both";
		boolean runNav = false;
		boolean swapSides = false;
		Long traceSeed = null;

		for (String arg : args) {
			if (arg.startsWith("--master=")) {
				masterSeed = Long.parseUnsignedLong(arg.substring("--master=".length()), 16);
			} else if (arg.startsWith("--count=")) {
				count = Integer.parseInt(arg.substring("--count=".length()));
			} else if (arg.startsWith("--opponent=")) {
				opponent = arg.substring("--opponent=".length()).toLowerCase();
			} else if ("--nav".equals(arg)) {
				runNav = true;
			} else if ("--swap".equals(arg)) {
				swapSides = true;
			} else if (arg.startsWith("--trace-seed=")) {
				traceSeed = Long.parseUnsignedLong(arg.substring("--trace-seed=".length()), 16);
			}
		}

		if (traceSeed != null) {
			runSingleTrace(traceSeed, "codex".equals(opponent) ? "Codex" : "Claude",
					"codex".equals(opponent) ? CodexAttackSub::new : ClaudeAttackSub::new);
			return;
		}

		var rng = new Random(masterSeed);
		long[] seeds = new long[count];
		for (int i = 0; i < count; i++) {
			seeds[i] = rng.nextLong();
		}

		System.out.printf("=== Gemini Arena Benchmark (master=%s, seeds=%d, opponent=%s, nav=%s, swap=%s) ===%n",
				Long.toHexString(masterSeed), count, opponent, runNav, swapSides);

		if (runNav) {
			long[] navSeeds = new long[Math.min(5, count)];
			System.arraycopy(seeds, 0, navSeeds, 0, navSeeds.length);
			if ("both".equals(opponent) || "claude".equals(opponent)) {
				System.out.println("\n--- NAV: Gemini vs Claude ---");
				SubmarineCompetition.compete(
						List.of(new SubmarineCompetition.Competitor("Gemini", GeminiAttackSub::new),
								new SubmarineCompetition.Competitor("Claude", ClaudeAttackSub::new)),
						navSeeds, NAV_TICKS);
			}
			if ("both".equals(opponent) || "codex".equals(opponent)) {
				System.out.println("\n--- NAV: Gemini vs Codex ---");
				SubmarineCompetition.compete(
						List.of(new SubmarineCompetition.Competitor("Gemini", GeminiAttackSub::new),
								new SubmarineCompetition.Competitor("Codex", CodexAttackSub::new)),
						navSeeds, NAV_TICKS);
			}
		}

		if ("both".equals(opponent) || "claude".equals(opponent)) {
			runCombatSeries("Gemini", GeminiAttackSub::new, "Claude", ClaudeAttackSub::new, seeds, swapSides);
		}
		if ("both".equals(opponent) || "codex".equals(opponent)) {
			runCombatSeries("Gemini", GeminiAttackSub::new, "Codex", CodexAttackSub::new, seeds, swapSides);
		}
	}

	private static void runCombatSeries(
		String nameA, Supplier<SubmarineController> facA, String nameB, Supplier<SubmarineController> facB,
		long[] seeds, boolean swapSides) {
		int threads = Math.min(Math.max(1, Runtime.getRuntime().availableProcessors() - 2), 12);
		System.out.printf("%n================ COMBAT: %s vs %s (%d seeds%s, %d threads) ================%n", nameA,
				nameB, seeds.length, swapSides ? " x2 swapped" : "", threads);

		record Job(int idx, String slot, boolean swapped, long seed) {
		}
		var jobs = new ArrayList<Job>();
		for (int i = 0; i < seeds.length; i++) {
			jobs.add(new Job(i + 1, "A0", false, seeds[i]));
			if (swapSides) {
				jobs.add(new Job(i + 1, "A1", true, seeds[i]));
			}
		}

		PrintStream origOut = System.out;
		System.setOut(new PrintStream(OutputStream.nullOutputStream()));
		var results = new ArrayList<MatchStats>();
		long t0 = System.nanoTime();
		try (var pool = Executors.newFixedThreadPool(threads)) {
			var futures = new ArrayList<Future<MatchStats>>();
			for (var job : jobs) {
				futures.add(pool.submit(() -> job.swapped() ? runMatch(facB, nameB, facA, nameA, job.seed(), false)
						: runMatch(facA, nameA, facB, nameB, job.seed(), false)));
			}
			for (var f : futures) {
				results.add(f.get());
			}
		} catch (Exception e) {
			System.setOut(origOut);
			throw new RuntimeException(e);
		} finally {
			System.setOut(origOut);
		}
		double wallSec = (System.nanoTime() - t0) / 1e9;

		int totalPtsA = 0, totalPtsB = 0;
		int winsA = 0, winsB = 0, draws = 0, mutuals = 0;
		int totalTorpsA = 0, totalTorpsB = 0;
		int totalPingsA = 0, totalPingsB = 0;
		int totalMatches = results.size();

		for (int k = 0; k < results.size(); k++) {
			var job = jobs.get(k);
			var s = results.get(k);
			printMatchLine(job.idx(), job.slot(), s);
			if (!job.swapped()) {
				totalPtsA += s.pointsA();
				totalPtsB += s.pointsB();
				totalTorpsA += s.torpsFiredA();
				totalTorpsB += s.torpsFiredB();
				totalPingsA += s.pingsA();
				totalPingsB += s.pingsB();
				if (s.pointsA() == 10)
					winsA++;
				else if (s.pointsB() == 10)
					winsB++;
				else if (s.hpA() <= 0 && s.hpB() <= 0)
					mutuals++;
				else
					draws++;
			} else {
				totalPtsA += s.pointsB();
				totalPtsB += s.pointsA();
				totalTorpsA += s.torpsFiredB();
				totalTorpsB += s.torpsFiredA();
				totalPingsA += s.pingsB();
				totalPingsB += s.pingsA();
				if (s.pointsB() == 10)
					winsA++;
				else if (s.pointsA() == 10)
					winsB++;
				else if (s.hpA() <= 0 && s.hpB() <= 0)
					mutuals++;
				else
					draws++;
			}
		}

		System.out.println("-".repeat(115));
		System.out.printf(
				"SUMMARY %s vs %s (%d matches in %.1fs): W=%d L=%d Draw=%d Mutual=%d | Pts %s:%d vs %s:%d | "
						+ "AvgTorps %.1f vs %.1f | AvgPings %.1f vs %.1f%n",
				nameA, nameB, totalMatches, wallSec, winsA, winsB, draws, mutuals, nameA, totalPtsA, nameB, totalPtsB,
				(double) totalTorpsA / totalMatches, (double) totalTorpsB / totalMatches,
				(double) totalPingsA / totalMatches, (double) totalPingsB / totalMatches);
	}

	private static void printMatchLine(int idx, String slot, MatchStats s) {
		String outcome;
		if (s.pointsA() == 10)
			outcome = s.nameA() + " WIN";
		else if (s.pointsB() == 10)
			outcome = s.nameB() + " WIN";
		else if (s.hpA() <= 0 && s.hpB() <= 0)
			outcome = "MUTUAL";
		else
			outcome = "TIMEOUT";

		System.out.printf(
				"#%02d [%s] seed=%-16s | %-11s (%4ds) | %s hp=%4d t=%d p=%2d ns=%4.1f minD=%4.0f (%s) | "
						+ "%s hp=%4d t=%d p=%2d ns=%4.1f minD=%4.0f (%s)%n",
				idx, slot, Long.toHexString(s.seed()), outcome, s.ticks() / 50, s.nameA(), s.hpA(), s.torpsFiredA(),
				s.pingsA(), s.avgNoiseA(), s.minTorpDistAToB() < 9999 ? s.minTorpDistAToB() : -1, s.deathInfoA(),
				s.nameB(), s.hpB(), s.torpsFiredB(), s.pingsB(), s.avgNoiseB(),
				s.minTorpDistBToA() < 9999 ? s.minTorpDistBToA() : -1, s.deathInfoB());
	}

	private static MatchStats runMatchQuiet(
		Supplier<SubmarineController> facA, String nameA, Supplier<SubmarineController> facB, String nameB, long seed) {
		PrintStream origOut = System.out;
		System.setOut(new PrintStream(OutputStream.nullOutputStream()));
		try {
			return runMatch(facA, nameA, facB, nameB, seed, false);
		} finally {
			System.setOut(origOut);
		}
	}

	private static void runSingleTrace(long seed, String oppName, Supplier<SubmarineController> oppFac) {
		System.out.printf("=== DETAILED TRACE: Gemini vs %s on seed %s ===%n", oppName, Long.toHexString(seed));
		var stats = runMatch(GeminiAttackSub::new, "Gemini", oppFac, oppName, seed, true);
		printMatchLine(1, "TR", stats);
	}

	private static MatchStats runMatch(
		Supplier<SubmarineController> facA, String nameA, Supplier<SubmarineController> facB, String nameB, long seed,
		boolean verbose) {
		var config = MatchConfig.withDefaults(seed).withMatchDurationTicks(COMBAT_TICKS);
		var world = new WorldGenerator().generate(config);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1_000_000);

		var ctrlA = facA.get();
		var ctrlB = facB.get();
		List<SubmarineController> controllers = List.of(ctrlA, ctrlB);
		List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		int[] hp = {1000, 1000};
		int[] prevHp = {1000, 1000};
		boolean[] forfeit = {false, false};
		int[] pings = {0, 0};
		double[] noiseSum = {0, 0};
		long[] aliveTicks = {0, 0};
		double[] minTorpToOther = {Double.MAX_VALUE, Double.MAX_VALUE};
		double[] minSubDist = {Double.MAX_VALUE};
		long[] ticksRun = {0};
		String[] deathInfo = {"ok", "ok"};
		String[] lastStatus = {"", ""};
		Set<Integer> torpsA = new HashSet<>();
		Set<Integer> torpsB = new HashSet<>();
		Map<Integer, String> lastTorpPhase = new HashMap<>();

		var listener = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				ticksRun[0] = tick + 1;
				if (subs.size() < 2)
					return;
				var s0 = subs.get(0);
				var s1 = subs.get(1);

				double subDist = s0.pose().position().distanceTo(s1.pose().position());
				if (subDist < minSubDist[0])
					minSubDist[0] = subDist;

				for (int i = 0; i < 2; i++) {
					var s = subs.get(i);
					hp[i] = s.hp();
					forfeit[i] = s.forfeited();
					lastStatus[i] = s.status();
					if (s.hp() > 0 && !s.forfeited()) {
						noiseSum[i] += s.sourceLevelDb();
						aliveTicks[i]++;
						if (s.pingRequested())
							pings[i]++;
					}
					if (s.hp() < prevHp[i]) {
						int dmg = prevHp[i] - s.hp();
						boolean torpDetNear = torps.stream().anyMatch(
								t -> t.detonated() && t.pose().position().distanceTo(s.pose().position()) < 100.0);
						String cause = torpDetNear ? "TORP" : (s.pose().position().z() < -400.0 ? "DEPTH" : "TERRAIN");
						if (verbose) {
							System.out.printf("  [t=%4ds] %s took %d %s dmg -> hp=%d pos=(%.0f,%.0f,%.0f) st=%s%n",
									tick / 50, s.name(), dmg, cause, s.hp(), s.pose().position().x(),
									s.pose().position().y(), s.pose().position().z(), s.status());
						}
						if (s.hp() <= 0 && "ok".equals(deathInfo[i])) {
							deathInfo[i] = cause + "@" + (tick / 50) + "s";
						}
					}
					if (s.forfeited() && "ok".equals(deathInfo[i])) {
						deathInfo[i] = "OOB@" + (tick / 50) + "s";
					}
					prevHp[i] = s.hp();
				}

				for (var t : torps) {
					if (t.ownerId() == s0.id()) {
						if (torpsA.add(t.id()) && verbose) {
							System.out.printf("  [t=%4ds] %s FIRED T%d at dist=%.0fm%n", tick / 50, nameA, t.id(),
									subDist);
						}
						double d = HullGeometry.bowDistanceToHull(t, s1);
						if (d < minTorpToOther[0])
							minTorpToOther[0] = d;
					} else if (t.ownerId() == s1.id()) {
						if (torpsB.add(t.id()) && verbose) {
							System.out.printf("  [t=%4ds] %s FIRED T%d at dist=%.0fm%n", tick / 50, nameB, t.id(),
									subDist);
						}
						double d = HullGeometry.bowDistanceToHull(t, s0);
						if (d < minTorpToOther[1])
							minTorpToOther[1] = d;
					}
					if (verbose && !t.diagPhase().equals(lastTorpPhase.get(t.id()))) {
						lastTorpPhase.put(t.id(), t.diagPhase());
						if (!t.diagPhase().isEmpty()) {
							double dToOther = t.ownerId() == s0.id()
									? t.pose().position().distanceTo(s1.pose().position())
									: t.pose().position().distanceTo(s0.pose().position());
							System.out.printf("    [t=%4ds] T%d phase=%s spd=%.1f distToEnemy=%.0fm fuel=%.0fs%n",
									tick / 50, t.id(), t.diagPhase(), t.speed(), dToOther, t.fuelRemaining());
						}
					}
				}

				if (verbose && tick % 2500 == 0) {
					System.out.printf(
							"  [t=%4ds] dist=%4.0fm | %s hp=%4d z=%4.0f spd=%4.1f ns=%4.0f [%s] | "
									+ "%s hp=%4d z=%4.0f spd=%4.1f ns=%4.0f [%s] | torps=%d%n",
							tick / 50, subDist, nameA, s0.hp(), s0.pose().position().z(), s0.speed(),
							s0.sourceLevelDb(), s0.status(), nameB, s1.hp(), s1.pose().position().z(), s1.speed(),
							s1.sourceLevelDb(), s1.status(), torps.size());
				}
			}

			@Override
			public void onMatchEnd() {
			}
		};

		sim.run(world, controllers, configs, listener);

		return new MatchStats(seed, nameA, nameB, hp[0], hp[1], forfeit[0], forfeit[1], torpsA.size(), torpsB.size(),
				pings[0], pings[1], ticksRun[0], aliveTicks[0] > 0 ? noiseSum[0] / aliveTicks[0] : 0,
				aliveTicks[1] > 0 ? noiseSum[1] / aliveTicks[1] : 0, minTorpToOther[0], minTorpToOther[1],
				minSubDist[0], deathInfo[0], deathInfo[1], lastStatus[0], lastStatus[1]);
	}
}
