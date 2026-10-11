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
package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.engine.EngineTestAccess;
import se.hirt.searobots.engine.SubmarineCompetition;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import java.util.*;
import java.io.BufferedWriter;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.nio.file.Files;
import java.nio.file.Path;

public final class CodexCombatBatch {
	private static final int DEFAULT_MATCH_TICKS = SubmarineCompetition.STANDARD_COMBAT_DURATION_SECONDS
			* SubmarineCompetition.TICKS_PER_SECOND;
	private static final int DEFAULT_MASTER_COUNT = 10;

	private CodexCombatBatch() {
	}

	public static void main(String[] args) throws IOException {
		var options = parseArgs(args);
		if (options.seeds().length == 0) {
			System.err.println("Provide seeds or use --master=<seed> [--count=<n>] [--ticks=<n>].");
			return;
		}

		var reasonCounts = new LinkedHashMap<String, Integer>();
		int codexWins = 0;
		int claudeWins = 0;
		int draws = 0;
		long totalTicks = 0;
		double totalWallSeconds = 0, totalCpuSeconds = 0;
		var process = (com.sun.management.OperatingSystemMXBean) ManagementFactory.getOperatingSystemMXBean();
		BufferedWriter csv = null;
		if (options.csv() != null) {
			Files.createDirectories(options.csv().toAbsolutePath().getParent());
			if (Files.exists(options.csv()))
				throw new IllegalArgumentException("CSV already exists: " + options.csv());
			csv = Files.newBufferedWriter(options.csv());
			csv.write(
					"phase,match,seed,winner,codex_points,claude_points,ticks,outcome,wall_seconds,cpu_seconds,wall_us_per_tick,cpu_us_per_tick,truncated\n");
			csv.flush();
		}
		System.out.printf(Locale.US, "CONFIG phase=%s ticks=%d seeds=%d java=%s%n", options.phase(),
				options.matchTicks(), options.seeds().length, System.getProperty("java.version"));
		System.out.println("SEEDS " + Arrays.toString(options.seeds()));

		for (int index = 0; index < options.seeds().length; index++) {
			long seed = options.seeds()[index];
			String progress = String.format(Locale.US, "START phase=%s match=%d/%d seed=%d", options.phase(), index + 1,
					options.seeds().length, seed);
			System.out.println(progress);
			writeStatus(options, progress);
			long wallStart = System.nanoTime(), cpuStart = process.getProcessCpuTime();
			var result = EngineTestAccess.runCombat(CodexAttackSub::new, "CodexAttackSub", ClaudeAttackSub::new,
					"ClaudeAttackSub", seed, options.matchTicks());
			double wallSeconds = (System.nanoTime() - wallStart) / 1e9;
			double cpuSeconds = (process.getProcessCpuTime() - cpuStart) / 1e9;
			double wallUsPerTick = wallSeconds * 1e6 / result.ticks(), cpuUsPerTick = cpuSeconds * 1e6 / result.ticks();
			boolean truncated = wallSeconds >= EngineTestAccess.combatWallClockLimitMillis() / 1000.0
					&& result.ticks() < options.matchTicks();
			String winner = result.pointsA() > result.pointsB() ? "Codex"
					: result.pointsB() > result.pointsA() ? "Claude" : "draw";

			reasonCounts.merge(result.description(), 1, Integer::sum);
			if (result.pointsA() > result.pointsB()) {
				codexWins++;
			} else if (result.pointsB() > result.pointsA()) {
				claudeWins++;
			} else {
				draws++;
			}

			totalTicks += result.ticks();
			totalWallSeconds += wallSeconds;
			totalCpuSeconds += cpuSeconds;
			progress = String.format(Locale.US,
					"DONE phase=%s match=%d/%d seed=%d winner=%s ticks=%d codex=%dpt claude=%dpt wall=%.3fs cpu=%.3fs wallUsPerTick=%.3f cpuUsPerTick=%.3f truncated=%s outcome=%s",
					options.phase(), index + 1, options.seeds().length, seed, winner, result.ticks(), result.pointsA(),
					result.pointsB(), wallSeconds, cpuSeconds, wallUsPerTick, cpuUsPerTick, truncated,
					result.description());
			System.out.println(progress);
			if (csv != null) {
				csv.write(String.format(Locale.US, "%s,%d,%d,%s,%d,%d,%d,%s,%.9f,%.9f,%.6f,%.6f,%s%n",
						csv(options.phase()), index + 1, seed, winner, result.pointsA(), result.pointsB(),
						result.ticks(), csv(result.description()), wallSeconds, cpuSeconds, wallUsPerTick, cpuUsPerTick,
						truncated));
				csv.flush();
			}
			writeStatus(options, progress);
		}
		if (csv != null)
			csv.close();

		System.out.println();
		System.out.printf("CONFIG ticks=%d seeds=%d%n", options.matchTicks(), options.seeds().length);
		System.out.printf("SUMMARY codexWins=%d claudeWins=%d draws=%d%n", codexWins, claudeWins, draws);
		System.out.printf(Locale.US, "TIMING ticks=%d wall=%.3fs cpu=%.3fs wallUsPerTick=%.3f cpuUsPerTick=%.3f%n",
				totalTicks, totalWallSeconds, totalCpuSeconds, totalWallSeconds * 1e6 / totalTicks,
				totalCpuSeconds * 1e6 / totalTicks);
		writeStatus(options, String.format(Locale.US,
				"COMPLETE phase=%s matches=%d codexWins=%d claudeWins=%d draws=%d ticks=%d wall=%.3fs cpu=%.3fs",
				options.phase(), options.seeds().length, codexWins, claudeWins, draws, totalTicks, totalWallSeconds,
				totalCpuSeconds));
		for (var entry : reasonCounts.entrySet()) {
			System.out.printf("reason=%s count=%d%n", entry.getKey(), entry.getValue());
		}
	}

	private static String csv(String value) {
		return '"' + value.replace("\"", "\"\"") + '"';
	}

	private static void writeStatus(Options options, String status) throws IOException {
		if (options.csv() != null)
			Files.writeString(Path.of(options.csv() + ".status"), java.time.Instant.now() + " " + status + "\n");
	}

	private static Options parseArgs(String[] args) {
		int matchTicks = DEFAULT_MATCH_TICKS;
		int masterCount = DEFAULT_MASTER_COUNT;
		Long masterSeed = null;
		Path csv = null;
		String phase = "batch";
		var explicitSeeds = new ArrayList<Long>();

		for (String arg : args) {
			if (arg.startsWith("--csv=")) {
				csv = Path.of(arg.substring("--csv=".length()));
				continue;
			}
			if (arg.startsWith("--phase=")) {
				phase = arg.substring("--phase=".length());
				continue;
			}
			if (arg.startsWith("--ticks=")) {
				matchTicks = Integer.parseInt(arg.substring("--ticks=".length()));
				continue;
			}
			if (arg.startsWith("--count=")) {
				masterCount = Integer.parseInt(arg.substring("--count=".length()));
				continue;
			}
			if (arg.startsWith("--master=")) {
				masterSeed = parseSeed(arg.substring("--master=".length()));
				continue;
			}
			explicitSeeds.add(parseSeed(arg));
		}

		var seeds = new LinkedHashSet<Long>(explicitSeeds);
		if (masterSeed != null) {
			var rng = new Random(masterSeed);
			for (int i = 0; i < masterCount; i++) {
				seeds.add(rng.nextLong());
			}
		}
		return new Options(matchTicks, seeds.stream().mapToLong(Long::longValue).toArray(), csv, phase);
	}

	private static long parseSeed(String value) {
		String trimmed = value.trim();
		if (trimmed.startsWith("0x") || trimmed.startsWith("0X")) {
			return Long.parseUnsignedLong(trimmed.substring(2), 16);
		}
		if (trimmed.matches("(?i)[0-9a-f]{8,16}")) {
			return Long.parseUnsignedLong(trimmed, 16);
		}
		return Long.parseLong(trimmed);
	}

	private record Options(int matchTicks, long[] seeds, Path csv, String phase) {
		@Override
		public String toString() {
			return "Options[ticks=" + matchTicks + ", seeds=" + Arrays.toString(seeds) + "]";
		}
	}
}
