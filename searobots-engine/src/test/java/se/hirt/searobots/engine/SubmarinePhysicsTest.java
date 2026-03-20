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
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SubmarinePhysicsTest {

    private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
    private static final double DT = 1.0 / CONFIG.tickRateHz();

    private static TerrainMap deepFlat() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -500);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    private static final TerrainMap TERRAIN = deepFlat();
    private static final CurrentField NO_CURRENT = new CurrentField(List.of());

    private SubmarineEntity makeSub(double speed, double heading) {
        return makeSub(speed, heading, -200);
    }

    private SubmarineEntity makeSub(double speed, double heading, double depth) {
        var controller = new DefaultAttackSub();
        var sub = new SubmarineEntity(submarine(), 0, controller, new Vec3(0, 0, depth),
                heading, Color.GREEN, 1000);
        sub.setSpeed(speed);
        return sub;
    }

    // ── Control surface reversal in reverse ───────────────────────────

    @Test
    void rudderTurnsRightAtForwardSpeed() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(10, 0); // heading north, 10 m/s forward
        sub.setRudder(1.0); // full right rudder

        double headingBefore = sub.heading();
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertTrue(sub.heading() > headingBefore,
                "Right rudder at forward speed should turn right (increase heading)");
    }

    @Test
    void rudderReversesInReverse() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(-3, 0); // heading north, 3 m/s reverse
        sub.setRudder(1.0); // full right rudder

        double headingBefore = sub.heading();
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // In reverse, water flows from stern → bow, so right rudder
        // should turn LEFT (decrease heading)
        assertTrue(sub.heading() < headingBefore || sub.heading() > Math.PI,
                "Right rudder in reverse should turn left, heading went from "
                        + headingBefore + " to " + sub.heading());
    }

    @Test
    void sternPlanesReverseInReverse() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(-3, 0); // heading north, 3 m/s reverse
        sub.setSternPlanes(1.0); // positive = pitch up at forward speed

        double pitchBefore = sub.pitch();
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // In reverse, positive stern planes should pitch DOWN
        assertTrue(sub.pitch() < pitchBefore,
                "Positive stern planes in reverse should pitch down, pitch went from "
                        + pitchBefore + " to " + sub.pitch());
    }

    @Test
    void noControlAuthorityAtZeroSpeed() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(0, 0);
        sub.setRudder(1.0);
        sub.setSternPlanes(1.0);

        double headingBefore = sub.heading();
        double pitchBefore = sub.pitch();
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertEquals(headingBefore, sub.heading(), 1e-10,
                "Rudder should have no effect at zero speed");
        assertEquals(pitchBefore, sub.pitch(), 1e-10,
                "Stern planes should have no effect at zero speed");
    }

    // ── Reverse thrust ────────────────────────────────────────────────

    @Test
    void reverseThrottleDeceleratesForwardMotion() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(10, 0);
        sub.setThrottle(-1.0);

        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertTrue(sub.speed() < 10,
                "Reverse throttle should decelerate, got speed=" + sub.speed());
    }

    @Test
    void reverseSpeedIsCapped() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(-10, 0); // already going fast in reverse
        sub.setThrottle(-1.0);

        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertTrue(sub.speed() >= -3.0,
                "Reverse speed should be capped at -3 m/s, got speed=" + sub.speed());
    }

    // ── Cavitation noise ──────────────────────────────────────────────

    @Test
    void reverseThrottleCausesCavitationNoise() {
        var physics = new SubmarinePhysics();
        var subForward = makeSub(5, 0);
        subForward.setThrottle(0.6);
        // Let thrust lag settle
        for (int i = 0; i < 250; i++) physics.step(subForward, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double noiseForward = subForward.noiseLevel();

        var subReverse = makeSub(5, 0);
        subReverse.setThrottle(-1.0);
        for (int i = 0; i < 250; i++) physics.step(subReverse, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double noiseReverse = subReverse.noiseLevel();

        assertTrue(noiseReverse > noiseForward,
                "Reverse thrust should be noisier: forward=" + noiseForward
                        + " reverse=" + noiseReverse);
    }

    // ── dB noise model ───────────────────────────────────────────────

    @Test
    void stationarySubHasBaseSourceLevel() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(0, 0, -200);
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertEquals(80.0, sub.sourceLevelDb(), 0.5,
                "Stationary sub at depth should have ~80 dB SL (machinery only)");
    }

    @Test
    void speedIncreasesSourceLevel() {
        var physics = new SubmarinePhysics();
        var slow = makeSub(2, 0, -200);
        var fast = makeSub(8, 0, -200);
        physics.step(slow, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        physics.step(fast, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertTrue(fast.sourceLevelDb() > slow.sourceLevelDb(),
                "Faster sub should be louder: slow=" + slow.sourceLevelDb()
                        + " fast=" + fast.sourceLevelDb());
    }

    @Test
    void noCavitationAtLowSpeed() {
        var physics = new SubmarinePhysics();
        // At -200m, cavitation onset = 5.0 + 200*0.02 = 9 m/s
        // At 4 m/s, well below onset → no cavitation
        // Speed drops slightly due to drag during step, so allow tolerance
        var sub = makeSub(4, 0, -200);
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // Expected: 80 + 2*speed (speed ≈ 3.98 after drag) ≈ 87.96 dB
        assertTrue(sub.sourceLevelDb() >= 87.0 && sub.sourceLevelDb() <= 89.0,
                "At 4 m/s / -200m, SL should be ~88 dB (no cavitation), got " + sub.sourceLevelDb());
    }

    @Test
    void cavitationAtShallowHighSpeed() {
        var physics = new SubmarinePhysics();
        // At -50m, cavitation onset = 5.0 + 50*0.02 = 6 m/s
        // At 10 m/s, 4 m/s above onset → cavitation adds dB
        var sub = makeSub(10, 0, -50);
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        double baseAndSpeed = 80.0 + 2.0 * 10.0; // 100 dB
        assertTrue(sub.sourceLevelDb() > baseAndSpeed + 5,
                "At 10 m/s / -50m, SL should include cavitation penalty, got " + sub.sourceLevelDb());
    }

    @Test
    void noCavitationAtDeepHighSpeed() {
        var physics = new SubmarinePhysics();
        // At -400m, cavitation onset = 5.0 + 400*0.02 = 13 m/s
        // At 10 m/s, below onset → no cavitation
        // Speed drops slightly due to drag (~9.87 after step)
        var sub = makeSub(10, 0, -400);
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // Expected: 80 + 2*9.87 ≈ 99.7 dB, no cavitation component
        assertTrue(sub.sourceLevelDb() >= 99.0 && sub.sourceLevelDb() <= 101.0,
                "At 10 m/s / -400m, SL should have no cavitation, got " + sub.sourceLevelDb());
    }

    @Test
    void deeperDepthRaisesCavitationOnset() {
        // At -100m: onset = 5.0 + 100*0.02 = 7 m/s
        // At -400m: onset = 5.0 + 400*0.02 = 13 m/s
        var physics = new SubmarinePhysics();

        // 10 m/s at -100m: above onset (7) → cavitates
        var shallow = makeSub(10, 0, -100);
        physics.step(shallow, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // 10 m/s at -400m: below onset (13) → no cavitation
        var deep = makeSub(10, 0, -400);
        physics.step(deep, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertTrue(shallow.sourceLevelDb() > deep.sourceLevelDb(),
                "Same speed should be louder at shallow depth: shallow=" + shallow.sourceLevelDb()
                        + " deep=" + deep.sourceLevelDb());
    }

    @Test
    void ballastChangeAddsNoise() {
        var physics = new SubmarinePhysics();
        // Quiet sub: neutral ballast, no change
        var quiet = makeSub(0, 0, -200);
        physics.step(quiet, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double quietSl = quiet.sourceLevelDb();

        // Sub actively changing ballast: set commanded ballast far from actual
        // to trigger maximum slew rate
        var blowing = makeSub(0, 0, -200);
        blowing.setBallast(1.0);  // command full blow (actual starts at 0.5)
        physics.step(blowing, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double blowingSl = blowing.sourceLevelDb();

        assertTrue(blowingSl > quietSl,
                "Ballast change should increase SL: quiet=" + quietSl
                        + " blowing=" + blowingSl);
    }

    // ── Thrust lag ─────────────────────────────────────────────────

    @Test
    void thrustLagsCommandedThrottle() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(0, 0);
        sub.setThrottle(1.0);
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // After 1 tick (0.02s), actual throttle should be 0.25 * 0.02 = 0.005
        assertEquals(0.005, sub.actualThrottle(), 1e-6,
                "Thrust should lag: after 1 tick actual=" + sub.actualThrottle());
    }

    @Test
    void thrustReachesFullPowerIn4Seconds() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(0, 0);
        sub.setThrottle(1.0);
        // 4 seconds = 200 ticks at 50 Hz
        for (int i = 0; i < 200; i++) physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        assertEquals(1.0, sub.actualThrottle(), 0.01,
                "Thrust should reach full power in 4s, actual=" + sub.actualThrottle());
    }

    @Test
    void thrustLagReducesInitialAcceleration() {
        var physics = new SubmarinePhysics();
        // With lag: throttle 1.0 from rest, check speed after 1 second
        var withLag = makeSub(0, 0);
        withLag.setThrottle(1.0);
        for (int i = 0; i < 50; i++) physics.step(withLag, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // Without lag, at t=1s full thrust would give much higher speed.
        // With lag at 0.25/s slew, average actual throttle over 1s is ~0.125.
        // So speed should be much less than what full throttle would give.
        assertTrue(withLag.speed() < 0.2,
                "With thrust lag, speed after 1s should be low, got " + withLag.speed());
        assertTrue(withLag.speed() > 0,
                "Speed should be positive after 1s of throttle");
    }

    @Test
    void emergencyBrakeHasDelay() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(10, 0);
        sub.setThrottle(0.8);
        // Let engine reach 0.8 actual throttle
        for (int i = 0; i < 200; i++) physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        assertTrue(sub.actualThrottle() > 0.7, "Engine should be near 0.8");

        // Command full reverse
        sub.setThrottle(-1.0);
        // After 0.5s (25 ticks), engine should still be positive (slewing down)
        for (int i = 0; i < 25; i++) physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        assertTrue(sub.actualThrottle() > 0,
                "After 0.5s of emergency brake, engine should still be positive, actual=" + sub.actualThrottle());
    }

    // ── Lift-curve control surfaces ─────────────────────────────────

    @Test
    void liftCoefficientLinearBelowStall() {
        double stallAngle = Math.toRadians(16);
        double cl5 = SubmarinePhysics.liftCoefficient(Math.toRadians(5), stallAngle);
        double cl10 = SubmarinePhysics.liftCoefficient(Math.toRadians(10), stallAngle);
        // In linear region, Cl should roughly double when angle doubles
        assertEquals(cl10 / cl5, 2.0, 0.01, "Cl should be linear below stall");
    }

    @Test
    void liftCoefficientReducedAboveStall() {
        double stallAngle = Math.toRadians(16);
        double clAtStall = SubmarinePhysics.liftCoefficient(stallAngle, stallAngle);
        double clAt30 = SubmarinePhysics.liftCoefficient(Math.toRadians(30), stallAngle);
        assertTrue(Math.abs(clAt30) < Math.abs(clAtStall),
                "Cl should be reduced above stall: atStall=" + clAtStall + " at30=" + clAt30);
    }

    @Test
    void moderateDeflectionTurnsBetterThanFull() {
        var physics = new SubmarinePhysics();
        // At 10 m/s, near-stall rudder (0.55, just below 25° stall) vs full (1.0, well past stall)
        // Run enough steps for rudder slew to reach target (~2 seconds)
        var subModerate = makeSub(10, 0);
        subModerate.setRudder(0.55);
        for (int i = 0; i < 100; i++)
            physics.step(subModerate, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double headingModerate = subModerate.heading();

        var subFull = makeSub(10, 0);
        subFull.setRudder(1.0);
        for (int i = 0; i < 100; i++)
            physics.step(subFull, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double headingFull = subFull.heading();

        assertTrue(headingModerate > headingFull,
                "Moderate rudder should turn faster than full due to stall: moderate="
                        + headingModerate + " full=" + headingFull);
    }

    @Test
    void controlAuthorityScalesWithSpeedSquared() {
        var physics = new SubmarinePhysics();
        // Same rudder at 5 m/s vs 10 m/s
        var subSlow = makeSub(5, 0);
        subSlow.setRudder(0.3);
        physics.step(subSlow, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double yawSlow = subSlow.heading(); // heading delta from 0

        var subFast = makeSub(10, 0);
        subFast.setRudder(0.3);
        physics.step(subFast, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double yawFast = subFast.heading();

        // Lift force scales with v^2, but effective inertia also increases with v^2
        // (centrifugal resistance), so net yaw response scales ~2x for 2x speed
        double ratio = yawFast / yawSlow;
        assertTrue(ratio > 1.5 && ratio < 2.5,
                "Yaw rate should scale ~2x with 2x speed (v^2 lift vs v^2 inertia), ratio=" + ratio);
    }

    @Test
    void reverseStillWorksWithLiftModel() {
        var physics = new SubmarinePhysics();
        var subForward = makeSub(5, Math.PI); // heading south, 5 m/s forward
        subForward.setRudder(1.0);
        double headingBefore = subForward.heading();
        physics.step(subForward, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double deltaForward = subForward.heading() - headingBefore;

        var subReverse = makeSub(-3, Math.PI); // heading south, 3 m/s reverse
        subReverse.setRudder(1.0);
        headingBefore = subReverse.heading();
        physics.step(subReverse, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());
        double deltaReverse = subReverse.heading() - headingBefore;

        // Forward and reverse should turn in opposite directions
        assertTrue(deltaForward * deltaReverse < 0,
                "Rudder should reverse in reverse: fwd=" + deltaForward + " rev=" + deltaReverse);
    }

    @Test
    void linearNoiseLevelMatchesDbConversion() {
        var physics = new SubmarinePhysics();
        var sub = makeSub(7, 0, -200);
        physics.step(sub, DT, TERRAIN, NO_CURRENT, CONFIG.battleArea());

        // noiseLevel should be 10^((sl - 80) / 20)
        double expectedLinear = Math.pow(10, (sub.sourceLevelDb() - 80) / 20.0);
        assertEquals(expectedLinear, sub.noiseLevel(), 0.001,
                "Linear noiseLevel should match dB conversion");
    }
}
