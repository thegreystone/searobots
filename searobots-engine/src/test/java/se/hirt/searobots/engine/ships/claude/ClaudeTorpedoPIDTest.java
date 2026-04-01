package se.hirt.searobots.engine.ships.claude;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.SubmarineDrone;

import java.awt.*;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests the Claude torpedo PID depth controller and targeting across
 * various scenarios: stationary targets, moving subs, surface ships,
 * different depths and angles.
 */
public class ClaudeTorpedoPIDTest {

    private static final double DT = 1.0 / 50;
    private static final GeneratedWorld WORLD = GeneratedWorld.deepFlat();

    record RunResult(double minDist, double minDepth, double maxDepth,
                     double maxPitchDeg, int oscillations, boolean hit, int ticks) {}

    private RunResult fireTorpedo(double launchZ, double targetDist, double targetZ, double launchHeading) {
        double targetX = 0;
        double targetY = targetDist;

        var cfg = VehicleConfig.torpedo();
        var ctrl = new ClaudeTorpedoController();
        var torp = new TorpedoEntity(9000, 0, cfg, ctrl,
                new Vec3(0, 0, launchZ), launchHeading, 0, 20.0, Color.RED);
        torp.setSpeed(23.0);
        torp.setActualThrottle(1.0);

        ctrl.onLaunch(new TorpedoLaunchContext(
                MatchConfig.withDefaults(0), WORLD.terrain(),
                new Vec3(0, 0, launchZ), launchHeading, 0,
                String.format("%.0f,%.0f,%.0f,0,5.0", targetX, targetY, targetZ)));

        var physics = new TorpedoPhysics();
        double minDist = Double.MAX_VALUE;
        double minDepth = launchZ, maxDepth = launchZ;
        double maxPitchDeg = 0;
        int oscillations = 0;
        double prevDz = 0;
        int dirChanges = 0;

        for (int tick = 0; tick < 12000; tick++) {
            final int t = tick;
            var torpInput = new TorpedoInput() {
                @Override public long tick() { return t; }
                @Override public double deltaTimeSeconds() { return DT; }
                @Override public Pose self() { return torp.pose(); }
                @Override public Velocity velocity() { return torp.velocity(); }
                @Override public double speed() { return torp.speed(); }
                @Override public double fuelRemaining() { return torp.fuelRemaining(); }
            };
            var torpOutput = torp.createOutput();
            ctrl.onTick(torpInput, torpOutput);
            physics.step(torp, DT, WORLD.terrain(), null, WORLD.config().battleArea());
            if (!torp.alive()) break;
            torp.decrementArmingDelay();

            double dx = torp.x() - targetX;
            double dy = torp.y() - targetY;
            double dz = torp.z() - targetZ;
            double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
            minDist = Math.min(minDist, dist);
            minDepth = Math.min(minDepth, torp.z());
            maxDepth = Math.max(maxDepth, torp.z());
            maxPitchDeg = Math.max(maxPitchDeg, Math.abs(Math.toDegrees(torp.pitch())));

            double currentDz = torp.verticalSpeed();
            if (tick > 50 && prevDz * currentDz < 0 && Math.abs(currentDz) > 0.1) {
                dirChanges++;
                if (dirChanges > 2) oscillations++;
            }
            prevDz = currentDz;

            if (torp.fuseArmed() && dist < torp.fuseRadius()) {
                return new RunResult(minDist, minDepth, maxDepth, maxPitchDeg, oscillations, true, tick);
            }
        }
        return new RunResult(minDist, minDepth, maxDepth, maxPitchDeg, oscillations, false, 12000);
    }

    // ── Stationary target tests (must hit) ──

    @Test
    void hitsAtSameDepthNearRange() {
        var r = fireTorpedo(-100, 500, -100, 0);
        assertTrue(r.hit, "Should hit stationary target at 500m same depth, got " + r.minDist + "m");
        assertTrue(r.maxPitchDeg < 5, "Same depth near: pitch should be <5°, got " + r.maxPitchDeg);
    }

    @Test
    void hitsAtSameDepthMidRange() {
        var r = fireTorpedo(-100, 1500, -100, 0);
        assertTrue(r.hit, "Should hit at 1500m same depth, got " + r.minDist + "m");
    }

    @Test
    void hitsAtSameDepthFarRange() {
        var r = fireTorpedo(-100, 3000, -100, 0);
        assertTrue(r.hit, "Should hit at 3000m same depth, got " + r.minDist + "m");
    }

    @Test
    void hitsDeepTargetFromShallow() {
        var r = fireTorpedo(-50, 1500, -300, 0);
        assertTrue(r.hit, "Should hit target 250m below, got " + r.minDist + "m");
    }

    @Test
    void hitsVeryDeepTargetFromShallow() {
        var r = fireTorpedo(-50, 2000, -500, 0);
        assertTrue(r.hit, "Should hit target 450m below, got " + r.minDist + "m");
    }

    @Test
    void hitsShallowTargetFromDeep() {
        var r = fireTorpedo(-400, 1500, -80, 0);
        assertTrue(r.hit || r.minDist < 80,
                "Should hit or approach target 320m above, got " + r.minDist + "m");
    }

    @Test
    void hitsDeepSameDepth() {
        var r = fireTorpedo(-400, 1500, -400, 0);
        assertTrue(r.hit, "Should hit at deep same depth, got " + r.minDist + "m");
    }

    @Test
    void hitsWith15DegreeOffset() {
        var r = fireTorpedo(-200, 1500, -200, Math.toRadians(15));
        assertTrue(r.hit, "Should correct 15° offset and hit, got " + r.minDist + "m");
    }

    @Test
    void hitsWith30DegreeOffset() {
        var r = fireTorpedo(-200, 1500, -200, Math.toRadians(30));
        assertTrue(r.hit || r.minDist < 30,
                "Should correct 30° offset, got " + r.minDist + "m");
    }

    @Test
    void hitsSurfaceTarget() {
        // Target at -30m (surface ship depth)
        var r = fireTorpedo(-30, 1500, -400, 0);
        assertTrue(r.hit, "Should hit deep target from surface depth, got " + r.minDist + "m");
    }

    // ── Surface safety ──

    @Test
    void neverBreachesModerateClimb() {
        double[][] cases = {
            {-150, 1500, -100},
            {-120, 1000, -80},
            {-300, 1500, -200},
            {-100, 1500, -100},
        };
        for (var c : cases) {
            var r = fireTorpedo(c[0], c[1], c[2], 0);
            assertTrue(r.maxDepth < -10,
                    String.format("Torpedo from %.0fm targeting %.0fm should not breach, got %.0fm",
                            c[0], c[2], r.maxDepth));
        }
    }

    @Test
    void sameDeptMinimalPitch() {
        var r = fireTorpedo(-200, 1500, -200, 0);
        assertTrue(r.oscillations < 3,
                "Same-depth should have minimal oscillation, got " + r.oscillations);
        assertTrue(r.maxPitchDeg < 15,
                "Same-depth max pitch should be under 15°, got " + r.maxPitchDeg);
    }

    // ── Moving target tests (full simulation with sonar) ──

    private record CombatResult(int hpTarget, double closestApproach) {}

    private CombatResult fireTorpedoAtMovingTarget(
            SubmarineController target, long seed, double launchRange) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        // Claude fires, target evades
        var claude = new ClaudeAttackSub();
        var controllers = List.<SubmarineController>of(claude, target);
        var configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        int[] targetHp = {1000};
        double[] closest = {Double.MAX_VALUE};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
                if (subs.size() >= 2) {
                    targetHp[0] = subs.get(1).hp();
                    for (var t : torps) {
                        if (t.ownerId() == subs.get(0).id() && t.alive()) {
                            double d = t.pose().position().distanceTo(subs.get(1).pose().position());
                            closest[0] = Math.min(closest[0], d);
                        }
                    }
                }
                if (tick >= 90_000 || targetHp[0] <= 0) sim.stop();
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(120_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(3000); } catch (InterruptedException e) {}

        return new CombatResult(targetHp[0], closest[0]);
    }

    @Test
    void torpedoDamagesInCombat() {
        // Run multiple seeds; Claude should land at least one torpedo hit across them
        int totalDamage = 0;
        for (long seed : new long[]{0x1000, 0x3000, 0x6000, 0xa000, 0xdeadbeefL}) {
            var r = fireTorpedoAtMovingTarget(new SubmarineDrone(), seed, 2000);
            int damage = 1000 - r.hpTarget;
            if (damage > 0) {
                System.out.printf("  seed=%x: damage=%d closest=%.0fm%n", seed, damage, r.closestApproach);
            }
            totalDamage += damage;
        }
        System.out.printf("Total damage across 5 seeds: %d%n", totalDamage);
        assertTrue(totalDamage > 0, "Claude should land at least one torpedo hit across 5 seeds");
    }

    @Test
    void depthProfileTable() {
        System.out.println("=== Torpedo PID Depth Controller Test ===");
        System.out.printf("%-12s %-8s %-8s %-8s %-10s %-10s %-8s %-6s %-5s%n",
                "Scenario", "LaunchZ", "TargetZ", "Range", "MinDist", "MaxPitch", "Oscill", "Breach", "Hit");
        System.out.println("-".repeat(85));

        record Scenario(String name, double launchZ, double targetDist, double targetZ, double heading) {}
        var scenarios = new Scenario[] {
            new Scenario("Near same",  -100, 500,  -100, 0),
            new Scenario("Mid same",   -100, 1500, -100, 0),
            new Scenario("Far same",   -100, 3000, -100, 0),
            new Scenario("Deep->shal", -400, 1500, -80,  0),
            new Scenario("Shal->deep", -50,  1500, -300, 0),
            new Scenario("Shal->Vdeep",-50,  2000, -500, 0),
            new Scenario("Deep same",  -400, 1500, -400, 0),
            new Scenario("15deg off",  -200, 1500, -200, Math.toRadians(15)),
            new Scenario("30deg off",  -200, 1500, -200, Math.toRadians(30)),
            new Scenario("Surf->deep", -30,  1500, -400, 0),
        };

        int hits = 0;
        for (var s : scenarios) {
            var r = fireTorpedo(s.launchZ, s.targetDist, s.targetZ, s.heading);
            boolean breach = r.maxDepth > -10;
            if (r.hit) hits++;
            System.out.printf("%-12s %7.0fm %7.0fm %7.0fm %9.0fm %9.1f° %7d %6s %5s%n",
                    s.name, s.launchZ, s.targetZ, s.targetDist, r.minDist,
                    r.maxPitchDeg, r.oscillations,
                    breach ? "YES" : "no", r.hit ? "HIT" : "miss");
        }
        System.out.printf("%nHits: %d/%d%n", hits, scenarios.length);
        assertTrue(hits >= 8, "Should hit at least 8/10 stationary targets, got " + hits);
    }
}
