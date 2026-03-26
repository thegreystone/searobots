package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;
import org.junit.jupiter.api.Test;

import java.util.*;

/**
 * Detailed combat trace to understand torpedo engagement patterns.
 */
public class ClaudeCombatTrace {

    @Test
    void traceEngagement() {
        long seed = 0xac28e4112d80c751L; // match seed where Claude dies
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(new ClaudeAttackSub(), new CodexAttackSub());
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        Set<Integer> seenTorps = new HashSet<>();

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() < 2) return;
                var claude = subs.get(0);
                var codex = subs.get(1);
                var cp = claude.pose().position();
                var xp = codex.pose().position();
                double range = cp.distanceTo(xp);

                // Log new torpedo launches
                for (var t : torps) {
                    if (seenTorps.add(t.id())) {
                        System.out.printf("t=%5d LAUNCH torp#%d owner=%d pos=(%.0f,%.0f,%.0f) target=(%.0f,%.0f,%.0f)%n",
                                tick, t.id(), t.ownerId(),
                                t.pose().position().x(), t.pose().position().y(), t.pose().position().z(),
                                t.targetX(), t.targetY(), t.targetZ());
                    }
                    if (t.detonated()) {
                        // Check distance to both subs at detonation
                        double dClaude = t.pose().position().distanceTo(cp);
                        double dCodex = t.pose().position().distanceTo(xp);
                        System.out.printf("t=%5d DETONATE torp#%d at (%.0f,%.0f,%.0f) distClaude=%.0f distCodex=%.0f%n",
                                tick, t.id(),
                                t.pose().position().x(), t.pose().position().y(), t.pose().position().z(),
                                dClaude, dCodex);
                    }
                }

                // Log HP changes
                if (tick > 0 && (claude.hp() < 1000 || codex.hp() < 1000)) {
                    if (tick % 50 == 0) { // don't spam
                        System.out.printf("t=%5d HP claude=%d codex=%d%n", tick, claude.hp(), codex.hp());
                    }
                }

                // Periodic status
                if (tick % 2500 == 0) {
                    String cTrack = claude.contactEstimates().isEmpty() ? "no contact"
                            : String.format("track c=%.2f u=%.0f",
                            claude.contactEstimates().getFirst().confidence(),
                            claude.contactEstimates().getFirst().uncertaintyRadius());
                    System.out.printf("t=%5d range=%.0f claude[hp=%d spd=%.1f depth=%.0f %s %s] codex[hp=%d spd=%.1f depth=%.0f]%n",
                            tick, range,
                            claude.hp(), claude.speed(), -cp.z(), claude.status(), cTrack,
                            codex.hp(), codex.speed(), -xp.z());

                    // Log active torpedoes
                    for (var t : torps) {
                        if (t.alive()) {
                            double dTarget = t.pose().position().distanceTo(xp);
                            System.out.printf("       torp#%d dist=%.0f speed=%.1f fuel=%.0f target=(%.0f,%.0f)%n",
                                    t.id(), dTarget, t.speed(), t.fuelRemaining(),
                                    t.targetX(), t.targetY());
                        }
                    }
                }

                if (tick >= 90_000 || claude.hp() <= 0 || codex.hp() <= 0) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}
    }
}
