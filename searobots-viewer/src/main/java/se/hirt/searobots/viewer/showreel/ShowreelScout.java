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
package se.hirt.searobots.viewer.showreel;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.SplittableRandom;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Supplier;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListeners;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.replay.ReplayFrame;
import se.hirt.searobots.engine.replay.ReplayReader;
import se.hirt.searobots.engine.replay.ReplayWriter;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;
import se.hirt.searobots.engine.ships.gemini.GeminiAttackSub;

/**
 * Finds a match worth filming. Surveys the terrain of many seeds (cheap), simulates the most
 * photogenic ones headless (Claude vs Codex, as in the viewer), and ranks them by scenery and by
 * how exciting the fight turned out. Every simulated match is kept as a replay with its
 * {@link MatchIndex} written beside it ({@code .index.txt}).
 * <p>
 * Standalone use prints the ranking: {@code ShowreelScout [--master HEX] [--terrain N]
 * [--matches N] [--dir DIR]}.
 */
public final class ShowreelScout {
	/** Seconds simulated after the outcome is decided, so a replay shows the wreck going down. */
	static final int AFTERMATH_SECONDS = 25;

	/** One scouted match. */
	public record Candidate(long seed, TerrainSurvey survey, MatchIndex index, Path replay) {
		public double score() {
			return 0.35 * survey.score() + 0.55 * index.excitement() + 10 * index.actionNearLand();
		}

		@Override
		public String toString() {
			return String.format(Locale.ROOT, "seed %-16s score %3.0f  excitement %3.0f  near land %3.0f%%  %s",
					Long.toHexString(seed), score(), index.excitement(), 100 * index.actionNearLand(), survey);
		}
	}

	private final Path dir;
	private final int threads;
	private final long maxTicks;
	private final List<Supplier<SubmarineController>> controllerFactories;
	private final String replaySuffix;
	private final PrintStream log;

	public ShowreelScout(Path dir, int threads, long maxTicks, PrintStream log) {
		this(dir, threads, maxTicks, defaultControllers(), log);
	}

	public ShowreelScout(Path dir, int threads, long maxTicks, List<Supplier<SubmarineController>> controllerFactories,
			PrintStream log) {
		this.dir = dir;
		this.threads = Math.max(1, threads);
		this.maxTicks = maxTicks;
		this.controllerFactories = List.copyOf(controllerFactories);
		this.replaySuffix = replaySuffix(this.controllerFactories);
		this.log = log;
	}

	/** The viewer's default line-up: Claude in the first slot, Codex in the second. */
	public static List<Supplier<SubmarineController>> defaultControllers() {
		return List.of(ClaudeAttackSub::new, CodexAttackSub::new);
	}

	/**
	 * Parses a two-ship matchup specification such as {@code gemini:claude} or
	 * {@code claude,codex}.
	 */
	public static List<Supplier<SubmarineController>> parseShips(String spec) {
		if (spec == null || spec.isBlank())
			return defaultControllers();
		String[] parts = spec.trim().split("\\s*[:,/]\\s*");
		if (parts.length != 2)
			throw new IllegalArgumentException(
					"Expected two ships separated by ':' or ',' (e.g. gemini:claude), got: " + spec);
		return List.of(controllerFactory(parts[0]), controllerFactory(parts[1]));
	}

	private static Supplier<SubmarineController> controllerFactory(String name) {
		return switch (name.trim().toLowerCase(Locale.ROOT)) {
		case "claude" -> ClaudeAttackSub::new;
		case "codex" -> CodexAttackSub::new;
		case "gemini" -> GeminiAttackSub::new;
		case "default" -> DefaultAttackSub::new;
		default -> throw new IllegalArgumentException(
				"Unknown submarine '" + name + "' (expected claude, codex, gemini, or default)");
		};
	}

	private static String replaySuffix(List<Supplier<SubmarineController>> factories) {
		List<String> names = factories.stream()
				.map(f -> f.get().name().toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]+", "-")).toList();
		if (names.equals(List.of("claude", "codex")))
			return "";
		return "-" + String.join("-vs-", names);
	}

	/**
	 * Surveys {@code terrainCount} seeds drawn from {@code master}, simulates the best
	 * {@code matchCount} of them, and returns the simulated ones, best first.
	 */
	public List<Candidate> scout(long master, int terrainCount, int matchCount)
			throws IOException, InterruptedException {
		Files.createDirectories(dir);
		var random = new SplittableRandom(master);
		long[] seeds = new long[terrainCount];
		for (int i = 0; i < terrainCount; i++)
			seeds[i] = random.nextLong();

		log.printf("Surveying the terrain of %d seeds...%n", terrainCount);
		record Surveyed(long seed, TerrainSurvey survey) {
		}
		var surveyed = new ArrayList<Surveyed>();
		try (var pool = Executors.newFixedThreadPool(threads)) {
			var futures = new ArrayList<Future<Surveyed>>();
			for (long seed : seeds)
				futures.add(pool.submit(() -> new Surveyed(seed,
						new TerrainSurvey(new WorldGenerator().generate(MatchConfig.withDefaults(seed)).terrain()))));
			for (var f : futures)
				surveyed.add(get(f));
		}
		surveyed.sort(Comparator.comparingDouble((Surveyed s) -> s.survey().score()).reversed());
		for (var s : surveyed)
			log.printf(Locale.ROOT, "  %-16s %s%n", Long.toHexString(s.seed()), s.survey());

		var chosen = surveyed.subList(0, Math.min(matchCount, surveyed.size()));
		log.printf("Simulating the %d most photogenic...%n", chosen.size());
		var results = new ArrayList<Candidate>();
		try (var pool = Executors.newFixedThreadPool(threads)) {
			var futures = new ArrayList<Future<Candidate>>();
			for (var s : chosen)
				futures.add(pool.submit(() -> simulate(s.seed())));
			for (var f : futures) {
				var c = get(f);
				log.println("  " + c);
				results.add(c);
			}
		}
		results.sort(Comparator.comparingDouble(Candidate::score).reversed());
		return results;
	}

	/** Simulates one seed, recording its replay and index into the scout directory. */
	public Candidate simulate(long seed) throws IOException {
		Files.createDirectories(dir);
		var config = MatchConfig.withDefaults(seed).withMatchDurationTicks((int) maxTicks);
		var world = new WorldGenerator().generate(config);
		var survey = new TerrainSurvey(world.terrain());
		var index = new MatchIndex(survey, config.tickRateHz());
		Path replay = dir.resolve("seed-" + Long.toHexString(seed) + replaySuffix + ".srl");

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);
		sim.setTicksAfterDecision((long) AFTERMATH_SECONDS * config.tickRateHz());
		List<SubmarineController> controllers = controllerFactories.stream().map(Supplier::get).toList();
		List<VehicleConfig> vehicles = controllers.stream().map(c -> VehicleConfig.submarine()).toList();
		try (var writer = new ReplayWriter(config, world.spawnPoints(), replay)) {
			sim.run(world, controllers, vehicles, SimulationListeners.composite(index, writer));
		}
		index.write(indexFile(replay), "seed " + Long.toHexString(seed) + ", replay " + replay.getFileName());
		return new Candidate(seed, survey, index, replay);
	}

	/** Indexes an existing replay, writing the index beside it. */
	public static Candidate index(Path replay) throws IOException {
		var reader = new ReplayReader(replay);
		var header = reader.header();
		var world = new WorldGenerator().generate(header.toMatchConfig());
		var survey = new TerrainSurvey(world.terrain());
		var index = new MatchIndex(survey, header.tickRateHz());
		List<ReplayFrame> frames = reader.readAll();
		for (var frame : frames)
			frame.emit(index);
		index.onMatchEnd();
		index.write(indexFile(replay), "seed " + Long.toHexString(header.seed()) + ", replay " + replay.getFileName());
		return new Candidate(header.seed(), survey, index, replay);
	}

	static Path indexFile(Path replay) {
		String name = replay.getFileName().toString();
		return replay.resolveSibling(name.replaceFirst("\\.srl$", "") + ".index.txt");
	}

	private static <T> T get(Future<T> future) throws IOException, InterruptedException {
		try {
			return future.get();
		} catch (ExecutionException e) {
			if (e.getCause() instanceof IOException io)
				throw io;
			if (e.getCause() instanceof RuntimeException re)
				throw re;
			throw new IllegalStateException(e.getCause());
		}
	}

	/**
	 * Runs {@code work} with the simulation's own console chatter (torpedo and match logs,
	 * controller diagnostics) sent to {@code logFile} rather than the console.
	 */
	public static <T> T quietly(Path logFile, IoSupplier<T> work) throws IOException, InterruptedException {
		PrintStream console = System.out;
		try (OutputStream file = Files.newOutputStream(logFile); var stream = new PrintStream(file, true)) {
			System.setOut(stream);
			return work.get();
		} finally {
			System.setOut(console);
		}
	}

	public interface IoSupplier<T> {
		T get() throws IOException, InterruptedException;
	}

	public static void main(String[] args) throws Exception {
		long master = 0x5eab07L;
		int terrain = 32, matches = 8;
		Path dir = Path.of("target", "showreel", "scout");
		String ships = null;
		for (int i = 0; i < args.length; i++) {
			switch (args[i]) {
			case "--master" -> master = Long.parseUnsignedLong(args[++i], 16);
			case "--terrain" -> terrain = Integer.parseInt(args[++i]);
			case "--matches" -> matches = Integer.parseInt(args[++i]);
			case "--dir" -> dir = Path.of(args[++i]);
			case "--ships" -> ships = args[++i];
			default -> throw new IllegalArgumentException("Unknown option " + args[i]);
			}
		}
		var console = System.out;
		var scout = new ShowreelScout(dir, Runtime.getRuntime().availableProcessors() / 2, 25L * 60 * 50,
				parseShips(ships), console);
		long m = master;
		int t = terrain, n = matches;
		Files.createDirectories(dir);
		var ranked = quietly(dir.resolve("scout.log"), () -> scout.scout(m, t, n));
		console.println();
		console.println("Ranking:");
		for (var c : ranked)
			console.println("  " + c + "  " + c.replay());
	}
}
