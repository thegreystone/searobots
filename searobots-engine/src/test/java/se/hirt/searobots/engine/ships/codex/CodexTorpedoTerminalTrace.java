package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;
import se.hirt.searobots.engine.WorldGenerator;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class CodexTorpedoTerminalTrace {
    private static final long DEFAULT_SEED = 3019826740869568801L;
    private static final int MATCH_TICKS = 600 * 50;
    private static final double TRACE_RADIUS_METERS = 250.0;

    private CodexTorpedoTerminalTrace() {}

    public static void main(String[] args) {
        long seed = args.length > 0 ? Long.parseLong(args[0]) : DEFAULT_SEED;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());
        Map<Integer, TorpedoState> states = new HashMap<>();

        System.out.printf("%n=== Codex Torpedo Terminal Trace seed=%d ===%n", seed);

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() < 2) {
                    return;
                }

                var codex = subs.get(0);
                var claude = subs.get(1);
                for (TorpedoSnapshot torp : torps) {
                    if (torp.ownerId() != codex.id()) {
                        continue;
                    }
                    var state = states.computeIfAbsent(torp.id(), ignored -> new TorpedoState());
                    double distToClaude = torp.pose().position().distanceTo(claude.pose().position());
                    double targetError = Double.NaN;
                    double targetJump = Double.NaN;
                    if (!Double.isNaN(torp.targetX()) && !Double.isNaN(torp.targetY())) {
                        targetError = Math.hypot(
                                torp.targetX() - claude.pose().position().x(),
                                torp.targetY() - claude.pose().position().y());
                        if (!Double.isNaN(state.lastTargetX)) {
                            targetJump = Math.hypot(
                                    torp.targetX() - state.lastTargetX,
                                    torp.targetY() - state.lastTargetY);
                        }
                        state.lastTargetX = torp.targetX();
                        state.lastTargetY = torp.targetY();
                    }

                    boolean enteringTrace = !state.tracing && distToClaude <= TRACE_RADIUS_METERS;
                    if (enteringTrace) {
                        state.tracing = true;
                        System.out.printf(Locale.US,
                                "ENTER tick=%d torp=%d dist=%.1f targetErr=%.1f targetJump=%.1f pos=(%.0f,%.0f,%.0f) target=(%.0f,%.0f,%.0f) claude=(%.0f,%.0f,%.0f)%n",
                                tick, torp.id(), distToClaude, targetError, targetJump,
                                torp.pose().position().x(), torp.pose().position().y(), torp.pose().position().z(),
                                torp.targetX(), torp.targetY(), torp.targetZ(),
                                claude.pose().position().x(), claude.pose().position().y(), claude.pose().position().z());
                    }

                    if (state.tracing) {
                        System.out.printf(Locale.US,
                                "TRACE tick=%d torp=%d dist=%.1f speed=%.1f ping=%s targetErr=%.1f targetJump=%.1f pos=(%.0f,%.0f,%.0f) target=(%.0f,%.0f,%.0f) claude=(%.0f,%.0f,%.0f)%n",
                                tick, torp.id(), distToClaude, torp.speed(), torp.pingRequested(),
                                targetError, targetJump,
                                torp.pose().position().x(), torp.pose().position().y(), torp.pose().position().z(),
                                torp.targetX(), torp.targetY(), torp.targetZ(),
                                claude.pose().position().x(), claude.pose().position().y(), claude.pose().position().z());
                    }
                }

                for (TorpedoSnapshot torp : torps) {
                    if (torp.ownerId() != codex.id()) {
                        continue;
                    }
                    var state = states.get(torp.id());
                    if (state == null || !state.tracing || state.reportedDone) {
                        continue;
                    }
                    if (torp.detonated() || !torp.alive()) {
                        double distToClaude = torp.pose().position().distanceTo(claude.pose().position());
                        double targetError = Double.NaN;
                        if (!Double.isNaN(torp.targetX()) && !Double.isNaN(torp.targetY())) {
                            targetError = Math.hypot(
                                    torp.targetX() - claude.pose().position().x(),
                                    torp.targetY() - claude.pose().position().y());
                        }
                        System.out.printf(Locale.US,
                                "END tick=%d torp=%d detonated=%s dist=%.1f targetErr=%.1f codexHp=%d claudeHp=%d%n",
                                tick, torp.id(), torp.detonated(), distToClaude, targetError,
                                codex.hp(), claude.hp());
                        state.reportedDone = true;
                    }
                }

                if (tick >= MATCH_TICKS || codex.hp() <= 0 || claude.hp() <= 0) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {
                System.out.println("TRACE END");
            }
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try {
            thread.join(60_000);
        } catch (InterruptedException ignored) {
        }
        sim.stop();
        try {
            thread.join(5_000);
        } catch (InterruptedException ignored) {
        }
    }

    private static final class TorpedoState {
        private double lastTargetX = Double.NaN;
        private double lastTargetY = Double.NaN;
        private boolean tracing;
        private boolean reportedDone;
    }
}
