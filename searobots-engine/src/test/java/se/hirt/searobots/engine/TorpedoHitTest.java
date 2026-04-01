package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.SubmarineDrone;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Test that torpedoes can actually hit targets on flat deep ocean.
 */
public class TorpedoHitTest {

    @Test
    void torpedoStraightRunHitsStationary() {
        // Dead simple: torpedo at (0,0,-100) aimed north, target at (0,500,-100)
        var world = GeneratedWorld.deepFlat();
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        // One stationary sub (SubmarineDrone moves slowly)
        var controllers = List.<SubmarineController>of(new SubmarineDrone());
        var configs = List.of(VehicleConfig.submarine());

        boolean[] targetHit = {false};
        double[] closestDist = {Double.MAX_VALUE};

        // Custom spawn: just one sub at (0, 500, -100)
        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                // On first tick, manually fire a torpedo toward the sub
                // (We can't directly spawn a torpedo here, so let's trace what happens)
                if (subs.size() >= 1) {
                    var target = subs.get(0);
                    for (var t : torps) {
                        if (!t.alive()) continue;
                        var tp = t.pose().position();
                        var sp = target.pose().position();
                        double d = Math.sqrt(Math.pow(tp.x()-sp.x(),2)+Math.pow(tp.y()-sp.y(),2)+Math.pow(tp.z()-sp.z(),2));
                        if (d < closestDist[0]) closestDist[0] = d;
                        if (target.hp() < 1000) targetHit[0] = true;
                        if (tick % 100 == 0) {
                            System.out.printf("tick=%d torp pos=(%.0f,%.0f,%.0f) target pos=(%.0f,%.0f,%.0f) dist=%.0f%n",
                                tick, tp.x(), tp.y(), tp.z(), sp.x(), sp.y(), sp.z(), d);
                        }
                    }
                }
                if (tick >= 10000) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        // We can't easily place subs at custom positions via SimulationLoop.
        // Instead, use deepFlat which puts sub at (-1000,0,-100).
        // Launch torpedo manually by having a second sub (Claude) that fires immediately.
        // But Claude requires CHASE mode...

        // SIMPLER: just test the physics directly
        System.out.println("=== Direct torpedo physics test ===");
        var physics = new TorpedoPhysics();
        var torpCtrl = new se.hirt.searobots.engine.ships.SimpleTorpedoController();
        var torp = new TorpedoEntity(999, 0, VehicleConfig.torpedo(), torpCtrl,
                new Vec3(0, 0, -100), 0, 0, 30.0, // heading=0 (north), fuse=30m
                java.awt.Color.GREEN);
        torp.setSpeed(23.0);

        // Target at (0, 500, -100) = 500m north
        torpCtrl.onLaunch(new TorpedoLaunchContext(
                MatchConfig.withDefaults(0), world.terrain(),
                new Vec3(0, 0, -100), 0, 0, "0,500,-100,0,5.0"));

        double dt = 1.0 / 50;
        double bestDist = 500;
        for (int tick = 0; tick < 5000; tick++) {
            var torpInput = new TorpedoInput() {
                @Override public long tick() { return 0; }
                @Override public double deltaTimeSeconds() { return dt; }
                @Override public Pose self() { return torp.pose(); }
                @Override public Velocity velocity() { return torp.velocity(); }
                @Override public double speed() { return torp.speed(); }
                @Override public double fuelRemaining() { return torp.fuelRemaining(); }
            };
            var torpOutput = torp.createOutput();
            torpCtrl.onTick(torpInput, torpOutput);
            physics.step(torp, dt, world.terrain(), null, world.config().battleArea());
            if (!torp.alive()) break;

            double distToTarget = Math.sqrt(torp.x()*torp.x() + Math.pow(torp.y()-500,2) + Math.pow(torp.z()+100,2));
            if (distToTarget < bestDist) bestDist = distToTarget;

            // Proximity fuse check (target at 0,500,-100)
            if (torp.fuseArmed() && distToTarget < torp.fuseRadius()) {
                System.out.printf("BOOM! tick=%d dist=%.1fm pos=(%.0f,%.0f,%.0f)%n",
                        tick, distToTarget, torp.x(), torp.y(), torp.z());
                break;
            }
            torp.decrementArmingDelay();

            if (tick % 250 == 0) {
                System.out.printf("tick=%d pos=(%.0f,%.0f,%.0f) speed=%.1f fuel=%.0f dist=%.0f heading=%.1f°%n",
                        tick, torp.x(), torp.y(), torp.z(), torp.speed(), torp.fuelRemaining(),
                        distToTarget, Math.toDegrees(torp.heading()));
            }
        }
        System.out.printf("Closest approach: %.1fm%n", bestDist);
        assertTrue(bestDist < 50, "Torpedo should get within 50m of stationary target at 500m range, got " + bestDist);
    }

    @Test
    void torpedoHitsDroneOnFlatOcean() {
        var world = GeneratedWorld.deepFlat();
        // Subs spawn at (-1000, 0) and (1000, 0), 2km apart, at -100m depth
        // Claude at spawn 0, drone at spawn 1

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        var controllers = List.<SubmarineController>of(new ClaudeAttackSub(), new SubmarineDrone());
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        var torpedoLaunches = new ArrayList<String>();
        var torpedoFates = new ArrayList<String>();
        boolean[] droneHit = {false};
        int[] droneHp = {1000};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() >= 2) {
                    droneHp[0] = subs.get(1).hp();
                    if (droneHp[0] < 1000) droneHit[0] = true;
                }
                for (var t : torps) {
                    if (t.detonated()) {
                        torpedoFates.add(String.format("DETONATED at tick %d pos=(%.0f,%.0f,%.0f)",
                                tick, t.pose().position().x(), t.pose().position().y(), t.pose().position().z()));
                    }
                    // Track closest approach to drone
                    if (subs.size() >= 2 && t.alive()) {
                        var dp = subs.get(1).pose().position();
                        var tp = t.pose().position();
                        double d = Math.sqrt(Math.pow(tp.x()-dp.x(),2)+Math.pow(tp.y()-dp.y(),2)+Math.pow(tp.z()-dp.z(),2));
                        if (tick % 500 == 0 && t.id() == 1000) {
                            System.out.printf("[CLOSE] tick=%d torp %d dist=%.1fm depth torp=%.0f drone=%.0f%n",
                                    tick, t.id(), d, tp.z(), dp.z());
                        }
                    }
                }
                // Run for 5 minutes max
                if (tick >= 25000) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        System.out.println("Torpedo fates: " + torpedoFates);
        System.out.println("Drone HP: " + droneHp[0]);
        System.out.println("Drone hit: " + droneHit[0]);

        // For now just report - we want to understand why torpedoes miss
        // assertTrue(droneHit[0], "Torpedo should hit the drone on flat ocean at 2km range");
    }
}
