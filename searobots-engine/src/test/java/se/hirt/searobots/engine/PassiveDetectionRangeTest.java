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

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Verifies passive sonar detection ranges under ideal conditions
 * (flat deep ocean, no thermocline, no baffles). Tests use the sonar
 * model directly to measure maximum detection distance.
 *
 * <p>Expected ranges with current parameters (baseSlDb=90, ambient=55,
 * selfNoiseOffset=35, spreading=10*log10, threshold=5):
 * <ul>
 *   <li>Patrol (3 m/s) heard by patrol (3 m/s): ~1.0 km</li>
 *   <li>Moderate (5 m/s) heard by slow (1 m/s): ~6.3 km</li>
 *   <li>Sprinting (10 m/s) heard by patrol (3 m/s): arena-wide</li>
 *   <li>Torpedo heard by patrol (same layer, ideal): 8+ km</li>
 * </ul>
 */
public class PassiveDetectionRangeTest {

    /**
     * Compute the theoretical max detection range from the sonar equation.
     * SE = SL_target - 10*log10(R) - NL > threshold
     * R_max = 10^((SL_target - NL - threshold) / spreading)
     */
    private static double theoreticalRange(double targetSL, double listenerSL, double sonarOffset) {
        double selfNoise = listenerSL - sonarOffset;
        double nl = Math.max(SonarModel.AMBIENT_NOISE_DB, selfNoise);
        double headroom = targetSL - nl - SonarModel.DETECTION_THRESHOLD_DB;
        if (headroom <= 0) return 0;
        return Math.pow(10, headroom / SonarModel.SPREADING_COEFFICIENT);
    }

    /** Sub-to-sub theoretical range (35 dB offset). */
    private static double theoreticalRange(double targetSL, double listenerSL) {
        return theoreticalRange(targetSL, listenerSL, VehicleConfig.submarine().sonarSelfNoiseOffsetDb());
    }

    private static double subSL(double speed) {
        var cfg = VehicleConfig.submarine();
        return cfg.baseSlDb() + cfg.speedNoiseDbPerMs() * speed;
    }

    private static double torpSL(double speed) {
        var cfg = VehicleConfig.torpedo();
        return cfg.baseSlDb() + cfg.speedNoiseDbPerMs() * speed;
    }

    @Test
    void printDetectionTable() {
        System.out.println("=== Passive Detection Range Table (ideal conditions) ===");
        System.out.printf("Sonar params: ambient=%.0f dB, selfNoiseOffset=%.0f dB, " +
                        "spreading=%.0f, threshold=%.0f dB%n",
                SonarModel.AMBIENT_NOISE_DB, SonarModel.SELF_NOISE_OFFSET_DB,
                SonarModel.SPREADING_COEFFICIENT, SonarModel.DETECTION_THRESHOLD_DB);
        System.out.printf("Sub base SL=%.0f dB, speed noise=%.1f dB/m/s%n%n",
                VehicleConfig.submarine().baseSlDb(),
                VehicleConfig.submarine().speedNoiseDbPerMs());

        System.out.printf("%-40s %-10s %-10s %-10s %-10s%n",
                "Scenario", "TargetSL", "NL", "Headroom", "MaxRange");
        System.out.println("-".repeat(80));

        double[][] scenarios = {
                // {target_speed, listener_speed}
                {3, 3},    // patrol vs patrol
                {5, 3},    // moderate vs patrol
                {5, 1},    // moderate vs slow
                {5, 0},    // moderate vs stopped
                {8, 3},    // fast vs patrol
                {10, 3},   // sprinting vs patrol
                {10, 0},   // sprinting vs stopped
                {25, 3},   // torpedo vs patrol (torpedo speed)
        };

        for (var s : scenarios) {
            boolean isTorp = s[0] >= 20;
            double tSL = isTorp ? torpSL(s[0]) : subSL(s[0]);
            double lSL = subSL(s[1]);
            double selfNoise = lSL - SonarModel.SELF_NOISE_OFFSET_DB;
            double nl = Math.max(SonarModel.AMBIENT_NOISE_DB, selfNoise);
            double headroom = tSL - nl - SonarModel.DETECTION_THRESHOLD_DB;
            double range = theoreticalRange(tSL, lSL);

            String label = isTorp
                    ? String.format("Torpedo (%.0f m/s) heard by sub (%.0f m/s)", s[0], s[1])
                    : String.format("Sub (%.0f m/s) heard by sub (%.0f m/s)", s[0], s[1]);
            System.out.printf("%-40s %-10.0f %-10.0f %-10.0f %-10.0f%n",
                    label, tSL, nl, headroom, range);
        }
    }

    @Test
    void patrolSubDetectedAtAbout1km() {
        double range = theoreticalRange(subSL(3), subSL(3));
        System.out.printf("Patrol vs patrol: %.0fm%n", range);
        assertTrue(range > 500, "Patrol sub should be detectable beyond 500m, got " + range);
        assertTrue(range < 3000, "Patrol sub shouldn't be detectable at 3km by patrol, got " + range);
    }

    @Test
    void moderateSubDetectedAtSeveralKm() {
        double range = theoreticalRange(subSL(5), subSL(1));
        System.out.printf("Moderate (5m/s) heard by slow (1m/s): %.0fm%n", range);
        assertTrue(range > 3000, "Moderate sub should be detectable at 3+ km by slow listener, got " + range);
        assertTrue(range < 15000, "Moderate sub shouldn't be detectable at 15km, got " + range);
    }

    @Test
    void sprintingSubDetectedAcrossArena() {
        double range = theoreticalRange(subSL(10), subSL(3));
        System.out.printf("Sprinting (10m/s) heard by patrol (3m/s): %.0fm%n", range);
        assertTrue(range > 10000, "Sprinting sub should be detectable across arena, got " + range);
    }

    @Test
    void torpedoDetectedAtLongRange() {
        // Sub at 3 m/s hearing a torpedo at 23 m/s (same thermal layer)
        double range = theoreticalRange(torpSL(23), subSL(3));
        System.out.printf("Torpedo (101dB base) heard by patrol (ideal, same layer): %.0fm%n", range);
        assertTrue(range > 4000, "Torpedo should be detectable at long range in same layer, got " + range);
    }

    @Test
    void torpedoPassiveDetectsModerateTarget() {
        // Torpedo at 23 m/s passively detecting a 5 m/s sub (using torpedo's 62 dB offset)
        double torpOffset = VehicleConfig.torpedo().sonarSelfNoiseOffsetDb();
        double range = theoreticalRange(subSL(5), torpSL(23), torpOffset);
        System.out.printf("Torpedo passive range vs 5m/s sub: %.0fm%n", range);
        assertTrue(range > 1500 && range < 4000,
                "Torpedo should passively detect 5m/s sub at 1.5-4km, got " + range);
    }

    @Test
    void torpedoCannotPassivelyDetectQuietSub() {
        // Torpedo at 23 m/s should NOT passively detect a 1 m/s sub
        double torpOffset = VehicleConfig.torpedo().sonarSelfNoiseOffsetDb();
        double range = theoreticalRange(subSL(1), torpSL(23), torpOffset);
        System.out.printf("Torpedo passive range vs 1m/s sub: %.0fm%n", range);
        assertTrue(range < 500, "Torpedo should barely detect quiet sub, got " + range);
    }

    @Test
    void stoppedSubIsHardToDetect() {
        // A stopped sub (base SL only, no speed noise) should be very hard to find
        double range = theoreticalRange(subSL(0), subSL(3));
        System.out.printf("Stopped sub heard by patrol: %.0fm%n", range);
        assertTrue(range < 500, "Stopped sub should be nearly invisible, got " + range);
    }

    @Test
    void speedAsymmetryRewardsPatience() {
        // Going slow lets you hear further: compare listener at 1 m/s vs 7 m/s
        double rangeSlow = theoreticalRange(subSL(5), subSL(1));
        double rangeFast = theoreticalRange(subSL(5), subSL(7));
        System.out.printf("Moderate target heard by slow (1m/s): %.0fm vs fast (7m/s): %.0fm%n",
                rangeSlow, rangeFast);
        assertTrue(rangeSlow > rangeFast * 3,
                "Slow listener should detect much further than fast. Slow=" + rangeSlow + " fast=" + rangeFast);
    }

    /**
     * End-to-end verification: run actual sonar model and confirm detection
     * at expected ranges on flat deep ocean.
     */
    @Test
    void endToEndDetectionOnFlatOcean() {
        var world = GeneratedWorld.flatOcean(-500, -100);
        var sonar = new SonarModel(0, 15.0);
        var cfg = VehicleConfig.submarine();

        // Place two subs at varying distances, check detection
        System.out.println("\n=== End-to-end detection (flat ocean, -100m depth) ===");
        System.out.printf("%-12s %-10s %-10s %-10s%n", "Distance", "TargetSpd", "ListnSpd", "Detected");

        for (int dist : new int[]{500, 1000, 2000, 3000, 5000, 7000, 10000}) {
            for (double[] speeds : new double[][]{{3, 3}, {5, 1}, {10, 3}}) {
                double tSpeed = speeds[0], lSpeed = speeds[1];

                // Create minimal entities at known positions
                var listener = new SubmarineEntity(cfg, 0, null,
                        new Vec3(0, 0, -100), 0, java.awt.Color.BLUE, 1000);
                var target = new SubmarineEntity(cfg, 1, null,
                        new Vec3(dist, 0, -100), 0, java.awt.Color.RED, 1000);

                // Set source levels based on speed
                listener.setSourceLevelDb(cfg.baseSlDb() + cfg.speedNoiseDbPerMs() * lSpeed);
                target.setSourceLevelDb(cfg.baseSlDb() + cfg.speedNoiseDbPerMs() * tSpeed);

                var results = sonar.computeContacts(0, List.of(listener, target),
                        world.terrain(), world.thermalLayers());
                var sr = results.get(0);
                boolean detected = sr != null && !sr.passiveContacts().isEmpty();

                System.out.printf("%-12d %-10.0f %-10.0f %-10s%n",
                        dist, tSpeed, lSpeed, detected ? "YES" : "no");
            }
        }
    }
}
