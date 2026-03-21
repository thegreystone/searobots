package se.hirt.searobots.engine;

import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.LinkedHashMap;
import java.util.Locale;

public final class CodexCombatBatch {
    private static final int MATCH_TICKS = 600 * 50;

    private CodexCombatBatch() {}

    public static void main(String[] args) {
        if (args.length == 0) {
            System.err.println("Provide one or more seeds.");
            return;
        }

        var reasonCounts = new LinkedHashMap<String, Integer>();
        int codexWins = 0;
        int claudeWins = 0;
        int draws = 0;

        for (String arg : args) {
            long seed = Long.parseLong(arg);
            var result = SubmarineCompetition.runCombat(
                    CodexAttackSub::new, "CodexAttackSub",
                    ClaudeAttackSub::new, "ClaudeAttackSub",
                    seed, MATCH_TICKS);

            reasonCounts.merge(result.reason(), 1, Integer::sum);
            if ("CodexAttackSub".equals(result.winner())) {
                codexWins++;
            } else if ("ClaudeAttackSub".equals(result.winner())) {
                claudeWins++;
            } else {
                draws++;
            }

            System.out.printf(Locale.US,
                    "seed=%d winner=%s reason=%s ticks=%d points=%d%n",
                    seed, result.winner(), result.reason(), result.ticks(), result.points());
        }

        System.out.println();
        System.out.printf("SUMMARY codexWins=%d claudeWins=%d draws=%d%n",
                codexWins, claudeWins, draws);
        for (var entry : reasonCounts.entrySet()) {
            System.out.printf("reason=%s count=%d%n", entry.getKey(), entry.getValue());
        }
    }
}
