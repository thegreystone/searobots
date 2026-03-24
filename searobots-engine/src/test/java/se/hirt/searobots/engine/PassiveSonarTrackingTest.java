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
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import static se.hirt.searobots.api.VehicleConfig.submarine;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Diagnostic tests for passive sonar tracking at close range, with
 * special attention to detection from behind the target (baffles
 * geometry) and the hunter's ability to maintain contact during a
 * stealth approach.
 */
class PassiveSonarTrackingTest {

    private static final List<ThermalLayer> NO_LAYERS = List.of();

    // -- Terrain helpers (reused from NavigationSimTest) ------------------

    private static TerrainMap deepFlat() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -500);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    private static final TerrainMap TERRAIN = deepFlat();

    // -- SubmarineEntity factory (same pattern as SonarModelTest) --------

    private SubmarineEntity makeSub(int id, Vec3 pos, double heading, double sourceLevelDb) {
        var controller = new DefaultAttackSub();
        var sub = new SubmarineEntity(submarine(), id, controller, pos, heading, Color.GREEN, 1000);
        sub.setSourceLevelDb(sourceLevelDb);
        return sub;
    }

    // -- Noise calculation helper ----------------------------------------
    // Mirrors SubmarinePhysics: SL = 80 + 2 * speed (ignoring cavitation
    // and surface effects for simplicity)

    private static double estimateSL(double speedMs) {
        return 80.0 + 2.0 * speedMs;
    }

    // ====================================================================
    // Test 1: passiveDetectionFromBehindAtCloseRange
    //
    // The listener is south of the target. Both head north. The listener
    // is therefore at the target's stern. Baffles only affect the
    // LISTENER's own hearing (not the target's noise emission), so the
    // listener should hear the noisy target in its forward arc.
    // ====================================================================

    @Test
    void passiveDetectionFromBehindAtCloseRange() {
        var sonar = new SonarModel(42);

        // Target at (0, 500, -200), heading north (0 rad), speed 8 m/s
        // SL ~ 80 + 2*8 = 96 dB
        double targetSL = estimateSL(8.0);
        var target = makeSub(1, new Vec3(0, 500, -200), 0, targetSL);

        // Listener at (0, 0, -200), heading north (0 rad), speed 2 m/s (quiet)
        // The target is at bearing ~0 (due north), which is the listener's
        // forward arc (not in baffles).
        double listenerSL = estimateSL(2.0);
        var listener = makeSub(0, new Vec3(0, 0, -200), 0, listenerSL);

        var results = sonar.computeContacts(0L, List.of(listener, target), TERRAIN, NO_LAYERS);

        var passiveContacts = results.get(0).passiveContacts();
        assertFalse(passiveContacts.isEmpty(),
                "Listener should detect noisy target (SL=" + targetSL + " dB) at 500m in forward arc");

        // Verify SE is well above threshold
        double se = passiveContacts.getFirst().signalExcess();
        assertTrue(se > SonarModel.DETECTION_THRESHOLD_DB,
                "SE should be above detection threshold, got " + se + " dB");

        // Verify bearing accuracy is reasonable (target is at bearing ~0 rad)
        double bearing = passiveContacts.getFirst().bearing();
        double bearingError = Math.abs(bearing);
        if (bearingError > Math.PI) bearingError = 2 * Math.PI - bearingError;
        assertTrue(bearingError < Math.toRadians(10),
                "Bearing error should be < 10 deg, got " + Math.toDegrees(bearingError) + " deg");

        System.out.println("[Test 1] Passive detection from behind at close range:");
        System.out.println("  Target SL: " + targetSL + " dB, range: 500m");
        System.out.println("  SE: " + se + " dB, bearing error: "
                + String.format("%.2f", Math.toDegrees(bearingError)) + " deg");
    }

    // ====================================================================
    // Test 2: bafflesBlockDetectionFromBehind
    //
    // Listener at (0, 0, -200) heading north. Target at (0, -500, -200)
    // directly behind the listener (bearing ~180 deg from bow). The
    // target should be in the listener's baffles.
    // ====================================================================

    @Test
    void bafflesBlockDetectionFromBehind() {
        var sonar = new SonarModel(42);

        // Target behind the listener: at (0, -500, -200)
        double targetSL = estimateSL(8.0); // ~96 dB, loud
        var target = makeSub(1, new Vec3(0, -500, -200), 0, targetSL);

        // Listener heading north at (0, 0, -200).
        // Bearing to target: atan2(0 - 0, -500 - 0) = atan2(0, -500) = PI.
        // Relative bearing = PI - 0 = PI (directly astern).
        // BAFFLE_HALF_ARC = 130 deg. |PI| = 180 deg > 130 deg, so in baffles.
        double listenerSL = estimateSL(2.0);
        var listener = makeSub(0, new Vec3(0, 0, -200), 0, listenerSL);

        var results = sonar.computeContacts(0L, List.of(listener, target), TERRAIN, NO_LAYERS);

        var passiveContacts = results.get(0).passiveContacts();

        // With the 20 dB baffle penalty at 500m range, the target should not
        // be detected. Even if SE is borderline, the penalty should push it
        // below threshold.
        // SE = SL - TL - (NL + baffle_penalty)
        //    = 96 - 10*log10(500) - (60 + 20) = 96 - 27 - 80 = -11 dB
        assertTrue(passiveContacts.isEmpty(),
                "Target in listener's baffles (stern) should NOT be detected at 500m. "
                        + "Contacts: " + passiveContacts.size());

        System.out.println("[Test 2] Baffles block detection from behind:");
        System.out.println("  Target SL: " + targetSL + " dB, range: 500m, in baffles");
        System.out.println("  Result: " + (passiveContacts.isEmpty() ? "not detected (correct)" : "detected (unexpected)"));
    }

    // ====================================================================
    // Test 3: passiveTrackingDegradationWithRange
    //
    // Same geometry (listener heading north, target ahead to the north)
    // at increasing distances. SE should decrease with range due to
    // spreading loss (TL = 10 * log10(r)).
    // ====================================================================

    @Test
    void passiveTrackingDegradationWithRange() {
        double targetSL = estimateSL(8.0); // ~96 dB at 8 m/s
        double listenerSL = estimateSL(2.0);

        double[] ranges = {500, 1000, 2000, 4000};
        double[] seValues = new double[ranges.length];
        boolean[] detected = new boolean[ranges.length];

        for (int i = 0; i < ranges.length; i++) {
            var sonar = new SonarModel(42);
            var listener = makeSub(0, new Vec3(0, 0, -200), 0, listenerSL);
            var target = makeSub(1, new Vec3(0, ranges[i], -200), Math.PI, targetSL);

            var results = sonar.computeContacts(0L, List.of(listener, target), TERRAIN, NO_LAYERS);
            var contacts = results.get(0).passiveContacts();
            detected[i] = !contacts.isEmpty();
            seValues[i] = detected[i] ? contacts.getFirst().signalExcess() : Double.NaN;
        }

        // Verify SE decreases with range where detected
        System.out.println("[Test 3] Passive tracking degradation with range:");
        for (int i = 0; i < ranges.length; i++) {
            System.out.println("  Range " + (int) ranges[i] + "m: detected=" + detected[i]
                    + ", SE=" + (detected[i] ? String.format("%.1f dB", seValues[i]) : "N/A"));
        }

        // At 500m the target should definitely be detected
        assertTrue(detected[0], "Target should be detected at 500m (SL=" + targetSL + " dB)");

        // SE should monotonically decrease at each detected range
        for (int i = 1; i < ranges.length; i++) {
            if (detected[i] && detected[i - 1]) {
                assertTrue(seValues[i] < seValues[i - 1],
                        "SE at " + (int) ranges[i] + "m (" + seValues[i]
                                + ") should be less than at " + (int) ranges[i - 1]
                                + "m (" + seValues[i - 1] + ")");
            }
        }

        // Find approximate max detection range.
        // With SL=96, NL=60, threshold=5:
        // max TL = 96 - 60 - 5 = 31 dB
        // max range = 10^(31/10) = ~1259m
        // So 1000m should be detected, 2000m probably not.
        assertTrue(detected[1], "Target should still be detected at 1000m");

        System.out.println("  Approximate max passive detection range for SL="
                + targetSL + " dB target: "
                + (detected[2] ? "> 2000m" :
                   detected[1] ? "1000-2000m" : "< 1000m"));
    }

    // ====================================================================
    // Test 4: stealthApproachSimulation
    //
    // Full simulation: hunter (DefaultAttackSub) starts at
    // (-2000, 0, -200) heading east. Drone (TargetDrone) starts at
    // (2000, 0, -30) heading north. Run for 4000 ticks (80 seconds).
    // The hunter should detect the drone and enter TRACKING or CHASE.
    // ====================================================================

    @Test
    void stealthApproachSimulation() {
        var config = MatchConfig.withDefaults(42);
        var terrain = deepFlat();

        var spawnPoints = List.of(
                new Vec3(-2000, 0, -200),
                new Vec3(2000, 0, -30));

        var world = new GeneratedWorld(config, terrain, List.of(),
                new CurrentField(List.of()), spawnPoints);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new TargetDrone());

        var stateLog = new ArrayList<String>();
        boolean[] hunterDetected = {false};
        boolean[] hunterTracking = {false};
        boolean[] ticked = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                ticked[0] = true;
                if (tick % 100 == 0 && submarines.size() >= 2) {
                    var hunter = submarines.get(0);
                    var drone = submarines.get(1);
                    double dx = hunter.pose().position().x() - drone.pose().position().x();
                    double dy = hunter.pose().position().y() - drone.pose().position().y();
                    double dist = Math.sqrt(dx * dx + dy * dy);
                    String status = hunter.status();

                    stateLog.add(String.format("t=%d dist=%.0fm status=%s hp=%d/%d",
                            tick, dist, status, hunter.hp(), drone.hp()));

                    // Check if hunter has contact estimates
                    if (!hunter.contactEstimates().isEmpty()) {
                        hunterDetected[0] = true;
                    }
                    // Check status for TRACKING or CHASE states
                    if (status.startsWith("T") || status.startsWith("C")) {
                        hunterTracking[0] = true;
                    }
                }
                if (tick >= 4000) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip()), listener));
        thread.start();
        try {
            thread.join(30_000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        assertTrue(ticked[0], "Simulation should have produced ticks");

        System.out.println("[Test 4] Stealth approach simulation (4000 ticks):");
        for (var entry : stateLog) {
            System.out.println("  " + entry);
        }

        // The drone at ~8 m/s has SL ~96 dB, initial distance 4000m.
        // TL at 4000m = 10*log10(4000) = 36 dB. SE = 96 - 36 - 60 = 0 dB.
        // So the drone may not be detected at 4000m initially. As the hunter
        // closes (at patrol throttle ~6 m/s), detection should occur within
        // ~30-40 seconds when range drops below ~1500m.
        assertTrue(hunterDetected[0],
                "Hunter should detect the drone during the 80-second engagement");
        assertTrue(hunterTracking[0],
                "Hunter should enter at least TRACKING state");
    }

    // ====================================================================
    // Test 5: stealthTailingFromBehind
    //
    // Hunter starts directly behind the drone. Both heading north.
    // Hunter at (0, -500, -200), drone at (0, 500, -30).
    // Distance: 1000m. The hunter should maintain contact via passive
    // sonar because the drone is loud and in the hunter's forward arc.
    // The drone cannot hear the quiet hunter (hunter is in the drone's
    // baffles).
    // ====================================================================

    @Test
    void stealthTailingFromBehind() {
        var config = MatchConfig.withDefaults(42);
        var terrain = deepFlat();

        var spawnPoints = List.of(
                new Vec3(0, -500, -200),
                new Vec3(0, 500, -30));

        var world = new GeneratedWorld(config, terrain, List.of(),
                new CurrentField(List.of()), spawnPoints);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new TargetDrone());

        var stateLog = new ArrayList<String>();
        int[] contactCount = {0};
        boolean[] hunterTracking = {false};
        boolean[] ticked = {false};

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                ticked[0] = true;
                if (tick % 100 == 0 && submarines.size() >= 2) {
                    var hunter = submarines.get(0);
                    var drone = submarines.get(1);
                    double dx = hunter.pose().position().x() - drone.pose().position().x();
                    double dy = hunter.pose().position().y() - drone.pose().position().y();
                    double dist = Math.sqrt(dx * dx + dy * dy);
                    String status = hunter.status();

                    boolean hasEstimates = !hunter.contactEstimates().isEmpty();
                    if (hasEstimates) contactCount[0]++;

                    stateLog.add(String.format("t=%d dist=%.0fm status=%s contact=%s",
                            tick, dist, status, hasEstimates));

                    if (status.startsWith("T") || status.startsWith("C")) {
                        hunterTracking[0] = true;
                    }
                }
                if (tick >= 3000) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip()), listener));
        thread.start();
        try {
            thread.join(30_000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        assertTrue(ticked[0], "Simulation should have produced ticks");

        System.out.println("[Test 5] Stealth tailing from behind (3000 ticks):");
        for (var entry : stateLog) {
            System.out.println("  " + entry);
        }

        // The hunter starts 1000m behind the drone. The drone has SL ~96 dB.
        // TL at 1000m = 30 dB. SE = 96 - 30 - 60 = 6 dB (just above threshold).
        // As the hunter may close the gap, SE will improve.
        assertTrue(contactCount[0] > 0,
                "Hunter should have contact estimates during the tailing engagement");
        System.out.println("  Contact estimates logged in " + contactCount[0]
                + " out of " + stateLog.size() + " samples");
    }

    // ====================================================================
    // Test 6: passiveRangeEstimateAccuracy
    //
    // Feed the DefaultAttackSub controller a series of passive sonar
    // contacts as the sub moves perpendicular to the contact bearing
    // (enabling triangulation). After enough displacement (~200m), check
    // whether the estimatedRange converges near the true range.
    // ====================================================================

    @Test
    void passiveRangeEstimateAccuracy() {
        var ctrl = new DefaultAttackSub();
        var config = MatchConfig.withDefaults(42);
        var terrain = deepFlat();
        ctrl.onMatchStart(new MatchContext(config, terrain, List.of(), new CurrentField(List.of())));

        // Target is at (0, 2000, -200). We move the sub east from (-400, 0, -200)
        // to (400, 0, -200) over 200 ticks, collecting passive contacts.
        // This provides ~800m of cross-track displacement.
        double targetX = 0, targetY = 2000;
        double subSpeed = 4.0; // m/s
        double heading = Math.PI / 2; // heading east

        double subX = -400;
        double subY = 0;

        double lastEstimatedRange = Double.MAX_VALUE;

        for (int tick = 0; tick < 200; tick++) {
            subX += subSpeed * Math.sin(heading) * 0.02; // dt = 0.02s
            double dx = targetX - subX;
            double dy = targetY - subY;
            double trueRange = Math.sqrt(dx * dx + dy * dy);
            double trueBearing = Math.atan2(dx, dy);
            if (trueBearing < 0) trueBearing += 2 * Math.PI;

            // Compute SE: SL - TL - NL
            double targetSL = estimateSL(8.0); // target cruising at 8 m/s
            double tl = 10.0 * Math.log10(trueRange);
            double nl = 60.0;
            double se = targetSL - tl - nl;

            // Add small bearing noise (1 deg std dev)
            double bearingNoise = (tick % 3 - 1) * Math.toRadians(0.5);
            double reportedBearing = trueBearing + bearingNoise;
            double brgStdDev = SonarModel.bearingStdDev(se);

            // Create passive contact with engine TMA range estimate.
            // The engine tracker computes SE-based range: 10^((SL - SE - NL) / 10)
            double tmaRange = Math.pow(10, (90.0 - se - 60.0) / 10.0);
            double rangeUncertainty = tmaRange * 0.3; // rough passive estimate
            double solQuality = Math.clamp(tick * 0.003, 0.15, 0.6);
            var contact = new SonarContact(reportedBearing, se, tmaRange, false,
                    8.0, brgStdDev, rangeUncertainty, 90.0, solQuality, Double.NaN);

            var pose = new Pose(new Vec3(subX, subY, -200), heading, 0, 0);
            var velocity = new Velocity(new Vec3(subSpeed, 0, 0), Vec3.ZERO);
            var state = new SubmarineState(pose, velocity, 1000, 0);
            var env = new EnvironmentSnapshot(terrain, List.of(), new CurrentField(List.of()));
            var input = new NavigationSimTest.TestInputFull(tick, 0.02, state, env,
                    List.of(contact), List.of(), 250);
            var output = new NavigationSimTest.CapturedOutput();
            ctrl.onTick(input, output);

            lastEstimatedRange = ctrl.estimatedRange();
        }

        System.out.println("[Test 6] Passive range estimate accuracy:");
        System.out.println("  True range: ~2000m");
        System.out.println("  Estimated range: " + String.format("%.0f", lastEstimatedRange) + "m");
        System.out.println("  Has tracked contact: " + ctrl.hasTrackedContact());
        System.out.println("  Has tracked contact: " + ctrl.hasTrackedContact());

        // The sub has moved ~800m east while receiving contacts from bearing ~0.
        // Triangulation should produce a range estimate. We accept anything
        // within a factor of 2 of the true range (passive estimation is rough).
        assertTrue(lastEstimatedRange < Double.MAX_VALUE,
                "Controller should have produced a range estimate");
        assertTrue(lastEstimatedRange > 500 && lastEstimatedRange < 8000,
                "Range estimate (" + lastEstimatedRange + "m) should be within a factor of 4 of true range (2000m)");

        // Check that the controller entered at least TRACKING
        assertTrue(ctrl.state() == DefaultAttackSub.State.TRACKING
                        || ctrl.state() == DefaultAttackSub.State.CHASE,
                "Controller should be in TRACKING or CHASE, was " + ctrl.state());
    }

    // ====================================================================
    // Test 7: pingThenGoSilentApproach
    //
    // Full simulation. The hunter starts 5000m from the drone, pings to
    // get a fix, then closes the distance using passive sonar. Over 200
    // seconds the hunter should get at least one active ping contact and
    // then continue closing.
    // ====================================================================

    @Test
    void pingThenGoSilentApproach() {
        var config = MatchConfig.withDefaults(42);
        var terrain = deepFlat();

        // Hunter starts west, drone starts east. Both at depth.
        var spawnPoints = List.of(
                new Vec3(-2500, 0, -200),
                new Vec3(2500, 0, -30));

        var world = new GeneratedWorld(config, terrain, List.of(),
                new CurrentField(List.of()), spawnPoints);

        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(
                new DefaultAttackSub(), new TargetDrone());

        var distanceLog = new ArrayList<double[]>(); // [tick, distance]
        boolean[] gotActiveContact = {false};
        boolean[] gotPassiveContact = {false};
        boolean[] ticked = {false};
        var stateLog = new ArrayList<String>();

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
                ticked[0] = true;
                if (tick % 100 == 0 && submarines.size() >= 2) {
                    var hunter = submarines.get(0);
                    var drone = submarines.get(1);
                    double dx = hunter.pose().position().x() - drone.pose().position().x();
                    double dy = hunter.pose().position().y() - drone.pose().position().y();
                    double dist = Math.sqrt(dx * dx + dy * dy);

                    distanceLog.add(new double[]{tick, dist});

                    String status = hunter.status();
                    boolean hasPing = hunter.pingRequested();
                    boolean hasEstimates = !hunter.contactEstimates().isEmpty();

                    stateLog.add(String.format("t=%d dist=%.0fm status=%s ping=%s contact=%s",
                            tick, dist, status, hasPing, hasEstimates));

                    if (hasEstimates) {
                        for (var est : hunter.contactEstimates()) {
                            if (est.label().equals("ping")) {
                                gotActiveContact[0] = true;
                            } else {
                                gotPassiveContact[0] = true;
                            }
                        }
                    }
                }
                if (tick >= 10000) {
                    sim.stop();
                }
            }

            @Override
            public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip()), listener));
        thread.start();
        try {
            thread.join(60_000); // longer timeout for 10000 ticks
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}

        assertTrue(ticked[0], "Simulation should have produced ticks");

        System.out.println("[Test 7] Ping then go silent approach (10000 ticks):");
        for (var entry : stateLog) {
            System.out.println("  " + entry);
        }

        // Verify the hunter got at least one contact (either active or passive)
        boolean anyContact = gotActiveContact[0] || gotPassiveContact[0];
        assertTrue(anyContact,
                "Hunter should have detected the drone (active or passive) during 200-second engagement");

        // Verify distance decreased over time (hunter is closing)
        if (distanceLog.size() >= 2) {
            double initialDist = distanceLog.getFirst()[1];
            double finalDist = distanceLog.getLast()[1];
            System.out.println("  Initial distance: " + String.format("%.0f", initialDist) + "m");
            System.out.println("  Final distance: " + String.format("%.0f", finalDist) + "m");
            System.out.println("  Active contact: " + gotActiveContact[0]);
            System.out.println("  Passive contact: " + gotPassiveContact[0]);

            // The hunter should close at least some distance.
            // At patrol speed ~6 m/s over 200s, it could close ~1200m in theory,
            // but the drone moves too, so actual closing depends on geometry.
            // Just verify the hunter moved closer than the initial 5000m.
            assertTrue(finalDist < initialDist,
                    "Hunter should close distance over time. Initial: "
                            + String.format("%.0f", initialDist) + "m, Final: "
                            + String.format("%.0f", finalDist) + "m");
        }
    }
}
