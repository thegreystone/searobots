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

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ControllerLoader.ResolvedController;
import se.hirt.searobots.engine.SimulationLoop.ControllerLimits;
import se.hirt.searobots.engine.replay.ReplayWriter;

import java.io.Closeable;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.SplittableRandom;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Local headless sparring CLI for submarine authors. Runs a reproducible Best-of-N (default 33)
 * combat series under league rules (alternating spawn slots, optional draw-aware clinch, optional
 * {@link ControllerLimits#leagueDefaults()} guardrails, multi-threaded execution, and automatic
 * {@code .srl} replay recording of losses or all games).
 * <p>
 * Example:
 *
 * <pre>
 *   java -cp target/classes:... se.hirt.searobots.engine.LeagueSparringRunner \
 *     --sub com.example.mysub.StarterSubmarine --vs claude --games 33 --record-losses target/replays
 * </pre>
 */
public final class LeagueSparringRunner {

	public static final int DEFAULT_GAMES = 33;
	public static final int DEFAULT_COMBAT_MINUTES = 120;
	public static final long AFTERMATH_TICKS = 25L * 50L;
	private static final SimulationListener NO_RECORDING = new SimulationListener() {
		@Override
		public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
		}

		@Override
		public void onMatchEnd() {
		}
	};

	public enum RecordMode {
		NONE, LOSSES, ALL
	}

	public record Options(String subSpec, String vsSpec, int games, boolean clinch, Long masterSeed, int combatMinutes,
			int threads, boolean leagueLimits, RecordMode recordMode, Path recordDir, String extraCp) {
	}

	public record GameOutcome(int gameNumber, long seed, boolean swappedSpawns, int winnerSide, String reason,
			long ticks, int hpSub, int hpOpp, int torpedoesSub, int torpedoesOpp, long cpuMsSub, long cpuMsOpp,
			String forfeitSub, String forfeitOpp, Path savedReplay) {
		// winnerSide: 1 = challenger (--sub), -1 = opponent (--vs), 0 = draw
	}

	public record SeriesSummary(ResolvedController challenger, ResolvedController opponent, long masterSeed,
			List<GameOutcome> games, int wins, int losses, int draws, int forfeits, double avgKillSeconds,
			double avgCpuMs, long maxCpuMs, Path highlightedLossReplay) {
	}

	private LeagueSparringRunner() {
	}

	public static void main(String[] args) throws Exception {
		Options opts = parseArgs(args);
		List<SeriesSummary> results = runAll(opts);
		if (results.size() > 1) {
			printGauntletSummary(results);
		}
	}

	public static List<SeriesSummary> runAll(Options opts) throws IOException, InterruptedException {
		ClassLoader baseLoader = LeagueSparringRunner.class.getClassLoader();
		ClassLoader loader = ControllerLoader.createClassLoader(opts.extraCp(), baseLoader);
		try {
			ResolvedController challenger = ControllerLoader.resolve(opts.subSpec(), loader);
			List<ResolvedController> opponents = "all".equalsIgnoreCase(opts.vsSpec().trim())
					? ControllerLoader.allCombatBuiltIns() : List.of(ControllerLoader.resolve(opts.vsSpec(), loader));

			long masterSeed = opts.masterSeed() != null ? opts.masterSeed() : ThreadLocalRandom.current().nextLong();
			var summaries = new ArrayList<SeriesSummary>(opponents.size());
			for (ResolvedController opp : opponents) {
				summaries.add(runSeries(challenger, opp, masterSeed, opts));
			}
			return List.copyOf(summaries);
		} finally {
			if (loader != baseLoader && loader instanceof Closeable c) {
				c.close();
			}
		}
	}

	public static SeriesSummary runSeries(
		ResolvedController challenger, ResolvedController opponent, long masterSeed, Options opts)
			throws IOException, InterruptedException {
		long[] seeds = deriveSeeds(masterSeed, opts.games());
		int durationTicks = opts.combatMinutes() * 60 * 50;
		int threads = Math.max(1, opts.threads());

		Path replayDir = opts.recordMode() != RecordMode.NONE ? opts.recordDir() : null;
		if (replayDir != null) {
			Files.createDirectories(replayDir);
		}

		String seedHex = String.format(Locale.ROOT, "%016x", masterSeed);
		System.out.println("=".repeat(92));
		System.out.printf(Locale.ROOT, "SPARRING: %s [%s] vs %s [%s]  |  %d games%s  |  seed=%s  |  threads=%d%n",
				challenger.displayName(), challenger.shortName(), opponent.displayName(), opponent.shortName(),
				opts.games(), opts.clinch() ? " (Bo" + opts.games() + " clinch)" : "", seedHex, threads);
		System.out.println("=".repeat(92));
		System.out.printf(Locale.ROOT, "%4s  %-16s  %-9s  %-10s  %-10s  %7s  %9s  %7s  %7s%n", "Game", "Seed", "Spawn",
				"Outcome", "Reason", "Time", "HP (S/O)", "Torps", "CPU ms");
		System.out.println("-".repeat(92));

		var outcomes = new ArrayList<GameOutcome>(opts.games());
		int wins = 0;
		int losses = 0;
		int draws = 0;
		int played = 0;
		long tStart = System.currentTimeMillis();

		try (ExecutorService pool = Executors.newFixedThreadPool(threads)) {
			while (played < opts.games()) {
				int remaining = opts.games() - played;
				if (opts.clinch() && isClinched(wins, losses, remaining)) {
					break;
				}
				int batchSize;
				if (opts.clinch()) {
					int neededWin = Math.max(1, (losses + remaining - wins) / 2 + 1);
					int neededLoss = Math.max(1, (wins + remaining - losses) / 2 + 1);
					batchSize = Math.min(remaining, Math.min(threads, Math.min(neededWin, neededLoss)));
				} else {
					batchSize = Math.min(remaining, threads);
				}

				var futures = new ArrayList<Future<GameOutcome>>(batchSize);
				for (int b = 0; b < batchSize; b++) {
					final int gameNum = played + b + 1;
					final long seed = seeds[gameNum - 1];
					futures.add(pool.submit(() -> runSingleGame(gameNum, seed, challenger, opponent, durationTicks,
							opts.leagueLimits(), opts.recordMode(), replayDir)));
				}

				for (var f : futures) {
					GameOutcome g = await(f);
					outcomes.add(g);
					played++;
					if (g.winnerSide() > 0) {
						wins++;
					} else if (g.winnerSide() < 0) {
						losses++;
					} else {
						draws++;
					}
					printGameLine(g, challenger, opponent);
				}
			}
		}

		long wallMs = System.currentTimeMillis() - tStart;
		int forfeits = (int) outcomes.stream().filter(g -> g.forfeitSub() != null).count();
		double avgKillSec = outcomes.stream().filter(g -> g.winnerSide() > 0 && "KILL".equals(g.reason()))
				.mapToLong(GameOutcome::ticks).average().orElse(0.0) / 50.0;
		double avgCpuMs = outcomes.stream().mapToLong(GameOutcome::cpuMsSub).average().orElse(0.0);
		long maxCpuMs = outcomes.stream().mapToLong(GameOutcome::cpuMsSub).max().orElse(0L);

		// Highlight the fastest loss (or first draw) with a saved replay for immediate debugging
		Path highlightedLoss = outcomes.stream().filter(g -> g.winnerSide() < 0 && g.savedReplay() != null)
				.min(java.util.Comparator.comparingLong(GameOutcome::ticks)).map(GameOutcome::savedReplay)
				.orElseGet(() -> outcomes.stream().filter(g -> g.winnerSide() <= 0 && g.savedReplay() != null)
						.findFirst().map(GameOutcome::savedReplay).orElse(null));

		double winPct = outcomes.isEmpty() ? 0.0 : (100.0 * wins) / outcomes.size();
		System.out.println("-".repeat(92));
		System.out.printf(Locale.ROOT, "RESULT: %s %d - %d %s (%d draws, %.1f%% win rate) in %.1fs wall time%n",
				challenger.displayName(), wins, losses, opponent.displayName(), draws, winPct, wallMs / 1000.0);
		System.out.printf(Locale.ROOT,
				"TELEMETRY: avg win time=%.0fs  |  CPU avg=%.0fms, peak=%dms (league cap: 30000ms)  |  forfeits=%d%n",
				avgKillSec, avgCpuMs, maxCpuMs, forfeits);
		if (highlightedLoss != null) {
			System.out.printf(Locale.ROOT, "INSPECT WORST LOSS: mvn -Preplay -Dreplay=%s%n",
					highlightedLoss.toString().replace('\\', '/'));
		}
		System.out.printf(Locale.ROOT, "REPRODUCE THIS SERIES: --seed %s%n%n", seedHex);

		return new SeriesSummary(challenger, opponent, masterSeed, List.copyOf(outcomes), wins, losses, draws, forfeits,
				avgKillSec, avgCpuMs, maxCpuMs, highlightedLoss);
	}

	private static GameOutcome runSingleGame(
		int gameNumber, long seed, ResolvedController challenger, ResolvedController opponent, int durationTicks,
		boolean leagueLimits, RecordMode recordMode, Path replayDir) throws IOException {
		boolean swapSpawns = (gameNumber % 2 == 0);
		ResolvedController slot0 = swapSpawns ? opponent : challenger;
		ResolvedController slot1 = swapSpawns ? challenger : opponent;

		MatchConfig config = MatchConfig.withDefaults(seed).withMatchDurationTicks(durationTicks);
		var world = new WorldGenerator().generate(config);

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		if (leagueLimits) {
			sim.setControllerLimits(ControllerLimits.leagueDefaults());
		}
		if (recordMode != RecordMode.NONE) {
			sim.setTicksAfterDecision(AFTERMATH_TICKS);
		}

		List<SubmarineController> controllers = List.of(slot0.factory().get(), slot1.factory().get());
		List<VehicleConfig> vehicles = List.of(slot0.vehicleConfig(), slot1.vehicleConfig());

		Path tempReplay = null;
		if (recordMode != RecordMode.NONE && replayDir != null) {
			String oppSlug = opponent.shortName().toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]", "");
			String fileName = String.format(Locale.ROOT, "vs-%s-game-%02d-%016x.srl", oppSlug, gameNumber, seed);
			tempReplay = replayDir.resolve(fileName);
			try (var writer = new ReplayWriter(config, world.spawnPoints(), tempReplay, 128L * 1024L * 1024L)) {
				sim.run(world, controllers, vehicles, writer);
			}
		} else {
			sim.run(world, controllers, vehicles, NO_RECORDING);
		}

		var result = sim.matchOutcome();
		var result0 = result.controllers().get(0);
		var result1 = result.controllers().get(1);
		boolean alive0 = result0.hp() > 0 && !result0.forfeited();
		boolean alive1 = result1.hp() > 0 && !result1.forfeited();

		int winnerSlot = 0; // 0=draw, 1=slot0, 2=slot1
		String reason;
		if (alive0 && !alive1) {
			winnerSlot = 1;
			reason = result1.forfeited() ? "FORFEIT" : "KILL";
		} else if (alive1 && !alive0) {
			winnerSlot = 2;
			reason = result0.forfeited() ? "FORFEIT" : "KILL";
		} else if (!alive0 && !alive1) {
			reason = (result0.forfeited() || result1.forfeited()) ? "DBL_FORF" : "MUTUAL";
		} else {
			reason = "TIMEOUT";
		}

		int winnerSide = 0;
		if (winnerSlot == 1) {
			winnerSide = swapSpawns ? -1 : 1;
		} else if (winnerSlot == 2) {
			winnerSide = swapSpawns ? 1 : -1;
		}

		int subSlot = swapSpawns ? 1 : 0;
		int oppSlot = swapSpawns ? 0 : 1;
		var subResult = result.controllers().get(subSlot);
		var oppResult = result.controllers().get(oppSlot);
		int hpSub = subResult.hp();
		int hpOpp = oppResult.hp();
		int torpsSub = subResult.torpedoesFired();
		int torpsOpp = oppResult.torpedoesFired();
		long cpuSub = subResult.cpuNanos() / 1_000_000L;
		long cpuOpp = oppResult.cpuNanos() / 1_000_000L;
		String forfSub = subResult.forfeitReason();
		String forfOpp = oppResult.forfeitReason();

		Path savedReplay = tempReplay;
		if (tempReplay != null && recordMode == RecordMode.LOSSES && winnerSide > 0) {
			Files.deleteIfExists(tempReplay);
			savedReplay = null;
		}

		return new GameOutcome(gameNumber, seed, swapSpawns, winnerSide, reason, result.tick() + 1, hpSub, hpOpp,
				torpsSub, torpsOpp, cpuSub, cpuOpp, forfSub.isEmpty() ? null : forfSub,
				forfOpp.isEmpty() ? null : forfOpp, savedReplay);
	}

	private static void printGameLine(GameOutcome g, ResolvedController sub, ResolvedController opp) {
		String spawn = g.swappedSpawns() ? opp.shortName() + "->" + sub.shortName()
				: sub.shortName() + "->" + opp.shortName();
		String outcome = g.winnerSide() > 0 ? "WIN" : (g.winnerSide() < 0 ? "LOSS" : "DRAW");
		long secs = g.ticks() / 50L;
		String timeStr = String.format(Locale.ROOT, "%dm%02ds", secs / 60, secs % 60);
		String hpStr = Math.max(0, g.hpSub()) + "/" + Math.max(0, g.hpOpp());
		String torpStr = g.torpedoesSub() + "/" + g.torpedoesOpp();
		String note = g.forfeitSub() != null ? "  [FORFEIT: " + g.forfeitSub() + "]"
				: (g.savedReplay() != null ? "  [" + g.savedReplay().getFileName() + "]" : "");

		System.out.printf(Locale.ROOT, "%4d  %016x  %-9s  %-10s  %-10s  %7s  %9s  %7s  %7d%s%n", g.gameNumber(),
				g.seed(), spawn, outcome, g.reason(), timeStr, hpStr, torpStr, g.cpuMsSub(), note);
	}

	private static void printGauntletSummary(List<SeriesSummary> results) {
		System.out.println("=".repeat(84));
		System.out.println("GAUNTLET SUMMARY");
		System.out.println("=".repeat(84));
		System.out.printf(Locale.ROOT, "%-18s  %5s  %5s  %5s  %7s  %8s  %9s  %8s%n", "Opponent", "Wins", "Loss", "Draw",
				"Win %", "Avg Kill", "Peak CPU", "Forfeits");
		System.out.println("-".repeat(84));
		int totalW = 0, totalL = 0, totalD = 0, totalF = 0;
		for (var s : results) {
			int n = s.games().size();
			double pct = n == 0 ? 0.0 : (100.0 * s.wins()) / n;
			System.out.printf(Locale.ROOT, "%-18s  %5d  %5d  %5d  %6.1f%%  %7.0fs  %7dms  %8d%n",
					s.opponent().displayName(), s.wins(), s.losses(), s.draws(), pct, s.avgKillSeconds(), s.maxCpuMs(),
					s.forfeits());
			totalW += s.wins();
			totalL += s.losses();
			totalD += s.draws();
			totalF += s.forfeits();
		}
		int totalN = totalW + totalL + totalD;
		double totalPct = totalN == 0 ? 0.0 : (100.0 * totalW) / totalN;
		System.out.println("-".repeat(84));
		System.out.printf(Locale.ROOT, "%-18s  %5d  %5d  %5d  %6.1f%%  %8s  %9s  %8d%n", "OVERALL", totalW, totalL,
				totalD, totalPct, "-", "-", totalF);
		System.out.println();
	}

	static boolean isClinched(int wins, int losses, int remaining) {
		return wins > losses + remaining || losses > wins + remaining;
	}

	static long[] deriveSeeds(long masterSeed, int count) {
		var rng = new SplittableRandom(masterSeed);
		long[] seeds = new long[count];
		for (int i = 0; i < count; i++) {
			seeds[i] = rng.nextLong();
		}
		return seeds;
	}

	static long parseSeed(String text) {
		String clean = text.trim();
		if (clean.startsWith("0x") || clean.startsWith("0X")) {
			clean = clean.substring(2);
		}
		try {
			return Long.parseUnsignedLong(clean, 16);
		} catch (NumberFormatException e) {
			return Long.parseLong(clean);
		}
	}

	static Options parseArgs(String[] args) {
		String subSpec = null;
		String vsSpec = "claude";
		int games = DEFAULT_GAMES;
		boolean clinch = false;
		Long masterSeed = null;
		int combatMinutes = DEFAULT_COMBAT_MINUTES;
		int threads = Math.max(1, Runtime.getRuntime().availableProcessors() / 2);
		boolean leagueLimits = true;
		RecordMode recordMode = RecordMode.NONE;
		Path recordDir = Path.of("target", "replays");
		String extraCp = null;

		for (int i = 0; i < args.length; i++) {
			switch (args[i]) {
			case "--sub" -> subSpec = args[++i];
			case "--vs" -> vsSpec = args[++i];
			case "--games" -> games = Integer.parseInt(args[++i]);
			case "--clinch" -> clinch = true;
			case "--seed" -> {
				if (i + 1 < args.length && !args[i + 1].startsWith("--")) {
					String raw = args[++i];
					if (!raw.isBlank()) {
						masterSeed = parseSeed(raw);
					}
				}
			}
			case "--combat-minutes" -> combatMinutes = Integer.parseInt(args[++i]);
			case "--threads" -> threads = Integer.parseInt(args[++i]);
			case "--no-league-limits" -> leagueLimits = false;
			case "--record-losses" -> {
				recordMode = RecordMode.LOSSES;
				if (i + 1 < args.length && !args[i + 1].startsWith("--")) {
					recordDir = Path.of(args[++i]);
				}
			}
			case "--record-all" -> {
				recordMode = RecordMode.ALL;
				if (i + 1 < args.length && !args[i + 1].startsWith("--")) {
					recordDir = Path.of(args[++i]);
				}
			}
			case "--extra-cp" -> {
				if (i + 1 < args.length && !args[i + 1].startsWith("--")) {
					String raw = args[++i];
					if (!raw.isBlank()) {
						extraCp = raw;
					}
				}
			}
			default -> throw new IllegalArgumentException("Unknown argument: " + args[i]);
			}
		}

		if (subSpec == null || subSpec.isBlank()) {
			throw new IllegalArgumentException(
					"Usage: LeagueSparringRunner --sub <FQCN|alias> [--vs claude|codex|gemini|default|drone|all|FQCN] "
							+ "[--games 33] [--clinch] [--seed <hex>] [--combat-minutes 120] [--threads <N>] "
							+ "[--record-losses <dir> | --record-all <dir>] [--extra-cp <path>] [--no-league-limits]");
		}
		if (games <= 0) {
			throw new IllegalArgumentException("--games must be positive, got " + games);
		}
		return new Options(subSpec, vsSpec, games, clinch, masterSeed, combatMinutes, threads, leagueLimits, recordMode,
				recordDir, extraCp);
	}

	private static <T> T await(Future<T> future) throws IOException, InterruptedException {
		try {
			return future.get();
		} catch (ExecutionException e) {
			if (e.getCause() instanceof IOException io) {
				throw io;
			}
			if (e.getCause() instanceof RuntimeException re) {
				throw re;
			}
			throw new IllegalStateException(e.getCause());
		}
	}

}
