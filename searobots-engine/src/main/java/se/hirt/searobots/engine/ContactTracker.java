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

import java.util.ArrayList;
import java.util.List;

/**
 * Engine-side contact tracker for automatic Target Motion Analysis (TMA).
 * Maintains bearing history, cross-track displacement, and range/heading
 * estimates for a single listener-source pair.
 */
final class ContactTracker {
    // Constants
    private static final double AMBIENT_NOISE_DB = 60.0;
    private static final double SPREADING_COEFFICIENT = 10.0;
    private static final double MIN_BEARING_RATE = Math.toRadians(0.05);

    // Bearing history (capped at 200)
    record BearingObs(long tick, double bearing, double ownX, double ownY, double se) {}
    private final List<BearingObs> history = new ArrayList<>();

    // Cross-track displacement
    private double accumulatedCrossTrack;
    private double prevOwnX = Double.NaN, prevOwnY = Double.NaN;
    private double prevOwnHeading = Double.NaN;

    // Range estimation
    private double estimatedRange = Double.NaN;
    private double rangeBias = Double.NaN;  // systematic error that decays with quality
    private double rangeUncertainty = Double.MAX_VALUE;

    // Heading estimation
    private double estimatedHeading = Double.NaN;

    // Solution quality
    private double solutionQuality;

    // Active calibration
    private double calibratedSL = Double.NaN;

    // Contact continuity
    private long lastObservationTick = -1;
    private int legCount;

    // Target position history for heading estimation
    private double prevTargetX = Double.NaN, prevTargetY = Double.NaN;
    private long prevTargetTick = -1;

    void update(long tick, double bearing, double se, double estSpeed,
                double estSL, double ownX, double ownY, double ownHeading,
                boolean inBaffles, double actualDistance,
                double actualTargetX, double actualTargetY,
                java.util.Random rng) {
        lastObservationTick = tick;

        // Don't update TMA from baffle-degraded observations
        if (inBaffles) return;

        // Record observation
        history.add(new BearingObs(tick, bearing, ownX, ownY, se));
        while (history.size() > 200) history.removeFirst();

        // Compute cross-track displacement
        if (!Double.isNaN(prevOwnX)) {
            double dx = ownX - prevOwnX;
            double dy = ownY - prevOwnY;
            double displacement = Math.sqrt(dx * dx + dy * dy);
            double moveHeading = Math.atan2(dx, dy);
            double crossFraction = Math.abs(Math.sin(moveHeading - bearing));
            accumulatedCrossTrack += displacement * crossFraction;
        }

        // Detect leg changes (own-ship course change > 15 degrees)
        if (!Double.isNaN(prevOwnHeading)) {
            double headingChange = ownHeading - prevOwnHeading;
            while (headingChange > Math.PI) headingChange -= 2 * Math.PI;
            while (headingChange < -Math.PI) headingChange += 2 * Math.PI;
            if (Math.abs(headingChange) > Math.toRadians(15)) {
                legCount++;
            }
        }
        prevOwnX = ownX;
        prevOwnY = ownY;
        prevOwnHeading = ownHeading;

        // Compute solution quality FIRST (determines range noise)
        updateSolutionQuality();

        // Range estimation: use actual distance with quality-dependent noise.
        // Range estimation with persistent bias that decays with quality.
        // A real TMA starts with a guess along the bearing line (too close
        // or too far) and refines it with cross-track data over time.
        // The bias represents this systematic error.
        if (Double.isNaN(rangeBias)) {
            // First observation: pick a random bias. The initial estimate
            // could be 0.5x to 2x the actual distance (along the bearing line).
            rangeBias = actualDistance * (rng.nextGaussian() * 0.7);
        }
        // Decay the bias toward zero as quality improves. At low quality,
        // the bias persists for a long time (30-60 seconds without good
        // geometry). At high quality, the bias decays in a few seconds.
        // Half-life at q=0.15: ln(2)/0.0001 = 6931 ticks = 139 seconds
        // Half-life at q=0.5: ln(2)/0.0013 = 533 ticks = 11 seconds
        // Half-life at q=0.8: ln(2)/0.0021 = 330 ticks = 7 seconds
        double biasDecay = 0.0001 + solutionQuality * 0.004;
        rangeBias *= (1.0 - biasDecay);

        // Add small random noise on top of the biased estimate
        double noiseLevel = 0.05 + 0.15 * (1.0 - solutionQuality); // 20% at q=0, 5% at q=1
        double noisyRange = actualDistance + rangeBias + actualDistance * rng.nextGaussian() * noiseLevel;
        noisyRange = Math.max(100, noisyRange);
        if (Double.isNaN(estimatedRange)) {
            estimatedRange = noisyRange;
        } else {
            // Smoothing: very slow at low quality (the biased estimate
            // should persist), faster as quality improves.
            // At q=0.15: alpha=0.003, converges in ~300 ticks (6 sec) to 63%
            // At q=0.5: alpha=0.01, converges in ~100 ticks (2 sec)
            // At q=0.8: alpha=0.02, converges in ~50 ticks (1 sec)
            double alpha = 0.002 + solutionQuality * 0.025;
            estimatedRange = estimatedRange * (1 - alpha) + noisyRange * alpha;
        }

        // Range uncertainty
        if (!Double.isNaN(estimatedRange)) {
            rangeUncertainty = estimatedRange * (1.0 - solutionQuality) * 0.5;
        }

        // Heading estimation from actual target displacement (with noise).
        // Like range, the engine uses ground truth with quality-dependent noise.
        // Requires quality > 0.3 and at least 5 seconds between samples.
        if (solutionQuality > 0.3 && !Double.isNaN(prevTargetX)
                && tick - prevTargetTick >= 250) { // 5 seconds between samples
            double tdx = actualTargetX - prevTargetX;
            double tdy = actualTargetY - prevTargetY;
            double targetMoved = Math.sqrt(tdx * tdx + tdy * tdy);

            if (targetMoved > 10) {
                double trueHeading = Math.atan2(tdx, tdy);
                if (trueHeading < 0) trueHeading += 2 * Math.PI;

                // Add noise proportional to inverse quality
                double headingNoise = Math.toRadians(30) * (1.0 - solutionQuality);
                double noisyHeading = trueHeading + rng.nextGaussian() * headingNoise;
                if (noisyHeading < 0) noisyHeading += 2 * Math.PI;
                if (noisyHeading >= 2 * Math.PI) noisyHeading -= 2 * Math.PI;

                if (Double.isNaN(estimatedHeading)) {
                    estimatedHeading = noisyHeading;
                } else {
                    double hDiff = noisyHeading - estimatedHeading;
                    while (hDiff > Math.PI) hDiff -= 2 * Math.PI;
                    while (hDiff < -Math.PI) hDiff += 2 * Math.PI;
                    estimatedHeading += hDiff * 0.3;
                    if (estimatedHeading < 0) estimatedHeading += 2 * Math.PI;
                    if (estimatedHeading >= 2 * Math.PI) estimatedHeading -= 2 * Math.PI;
                }
            }
            prevTargetX = actualTargetX;
            prevTargetY = actualTargetY;
            prevTargetTick = tick;
        } else if (Double.isNaN(prevTargetX)) {
            prevTargetX = actualTargetX;
            prevTargetY = actualTargetY;
            prevTargetTick = tick;
        }
    }

    private void updateSolutionQuality() {
        double geoQuality = 0;
        if (!Double.isNaN(estimatedRange) && estimatedRange > 0) {
            geoQuality = Math.clamp(accumulatedCrossTrack / (estimatedRange * 2.0), 0, 0.8);
        }
        double legBonus = Math.min(legCount * 0.15, 0.3);
        double timeBonus = Math.min(history.size() * 0.003, 0.4);
        solutionQuality = Math.clamp(geoQuality + legBonus + timeBonus, 0.15, 0.95);
    }

    void updateFromPing(long tick, double range, double se) {
        estimatedRange = range;
        rangeBias = 0; // ping eliminates systematic error
        rangeUncertainty = range * 0.02; // 2% RMS
        solutionQuality = 0.95;
        lastObservationTick = tick;

        // Calibrate SL
        if (se > 5.0 && range > 1.0) {
            calibratedSL = se + SPREADING_COEFFICIENT * Math.log10(range) + AMBIENT_NOISE_DB;
        }
    }

    void decay(long tick, double maxSubSpeed) {
        if (lastObservationTick < 0) return;
        double dtSec = (tick - lastObservationTick) / 50.0;
        solutionQuality = Math.max(0, solutionQuality - 0.01 * dtSec);
        rangeUncertainty += maxSubSpeed * dtSec;
    }

    boolean isExpired(long tick) {
        return lastObservationTick >= 0 && (tick - lastObservationTick) > 1500;
    }

    // Accessors
    double estimatedRange() { return Double.isNaN(estimatedRange) ? 0 : estimatedRange; }
    double rangeUncertainty() { return rangeUncertainty; }
    double solutionQuality() { return solutionQuality; }
    double estimatedHeading() { return estimatedHeading; }
    long lastObservationTick() { return lastObservationTick; }
}
