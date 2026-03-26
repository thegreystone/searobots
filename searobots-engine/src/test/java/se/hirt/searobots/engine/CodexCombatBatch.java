package se.hirt.searobots.engine;

import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Random;

public final class CodexCombatBatch {
    private static final int DEFAULT_MATCH_TICKS =
            SubmarineCompetition.STANDARD_COMBAT_DURATION_SECONDS * SubmarineCompetition.TICKS_PER_SECOND;
    private static final int DEFAULT_MASTER_COUNT = 10;

    private CodexCombatBatch() {}

    public static void main(String[] args) {
        var options = parseArgs(args);
        if (options.seeds().length == 0) {
            System.err.println("Provide seeds or use --master=<seed> [--count=<n>] [--ticks=<n>].");
            return;
        }

        var reasonCounts = new LinkedHashMap<String, Integer>();
        int codexWins = 0;
        int claudeWins = 0;
        int draws = 0;

        for (long seed : options.seeds()) {
            var result = SubmarineCompetition.runCombat(
                    CodexAttackSub::new, "CodexAttackSub",
                    ClaudeAttackSub::new, "ClaudeAttackSub",
                    seed, options.matchTicks());

            reasonCounts.merge(result.description(), 1, Integer::sum);
            if (result.pointsA() > result.pointsB()) {
                codexWins++;
            } else if (result.pointsB() > result.pointsA()) {
                claudeWins++;
            } else {
                draws++;
            }

            System.out.printf(Locale.US,
                    "seed=%d %s ticks=%d codex=%dpt claude=%dpt%n",
                    seed, result.description(), result.ticks(), result.pointsA(), result.pointsB());
        }

        System.out.println();
        System.out.printf("CONFIG ticks=%d seeds=%d%n", options.matchTicks(), options.seeds().length);
        System.out.printf("SUMMARY codexWins=%d claudeWins=%d draws=%d%n",
                codexWins, claudeWins, draws);
        for (var entry : reasonCounts.entrySet()) {
            System.out.printf("reason=%s count=%d%n", entry.getKey(), entry.getValue());
        }
    }

    private static Options parseArgs(String[] args) {
        int matchTicks = DEFAULT_MATCH_TICKS;
        int masterCount = DEFAULT_MASTER_COUNT;
        Long masterSeed = null;
        var explicitSeeds = new ArrayList<Long>();

        for (String arg : args) {
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
        return new Options(matchTicks, seeds.stream().mapToLong(Long::longValue).toArray());
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

    private record Options(int matchTicks, long[] seeds) {
        @Override
        public String toString() {
            return "Options[ticks=" + matchTicks + ", seeds=" + Arrays.toString(seeds) + "]";
        }
    }
}
