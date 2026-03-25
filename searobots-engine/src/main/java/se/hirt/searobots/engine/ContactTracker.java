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
import java.util.Random;

/**
 * Engine-side contact tracker that simulates the output of a Kalman-filter
 * / batch least-squares TMA system. Uses ground-truth distance with
 * quality-dependent noise and bias to model realistic convergence behavior.
 *
 * <p>Key behaviors:
 * <ul>
 *   <li>Bearing is always available when a contact is detected</li>
 *   <li>Range starts with a large systematic bias (factor 1.5-2x)</li>
 *   <li>Bias only decays with cross-track maneuvering and leg changes</li>
 *   <li>Time alone gives almost nothing (no free convergence)</li>
 *   <li>Heading requires quality &gt; 0.5 (real maneuvering needed)</li>
 *   <li>Active sonar bypasses TMA: instant accurate range</li>
 * </ul>
 */
final class ContactTracker {
    // Constants
    private static final double AMBIENT_NOISE_DB = 60.0;
    private static final double SPREADING_COEFFICIENT = 10.0;

    // Bearing history (capped at 200)
    record BearingObs(long tick, double bearing, double ownX, double ownY, double se) {}
    private final List<BearingObs> history = new ArrayList<>();

    // Cross-track displacement (accumulated perpendicular motion relative to bearing)
    private double accumulatedCrossTrack;
    private double prevOwnX = Double.NaN, prevOwnY = Double.NaN;
    private double prevOwnHeading = Double.NaN;

    // Range estimation
    private double estimatedRange = Double.NaN;
    private double rangeBias = Double.NaN;  // systematic error, decays with geometry
    private double rangeUncertainty = Double.MAX_VALUE;

    // Heading estimation
    private double estimatedHeading = Double.NaN;

    // Solution quality
    private double solutionQuality;

    // Active sonar calibration of source level
    private double calibratedSL = Double.NaN;

    // Contact continuity
    private long lastObservationTick = -1;
    private int legCount;
    private double legHeadingAccumulator; // accumulated heading change since last leg count

    // Heading estimation: uses ground truth with quality-dependent noise
    private double prevTargetX = Double.NaN, prevTargetY = Double.NaN;
    private long prevTargetTick = -1;

    /**
     * Update tracker with a new passive bearing observation. Range is modeled
     * as ground-truth with a large persistent bias that decays only through
     * cross-track maneuvering. This simulates the output of a real TMA filter.
     */
    void update(long tick, double bearing, double se, double estSpeed,
                double estSL, double ownX, double ownY, double ownHeading,
                boolean inBaffles, double actualDistance,
                double actualTargetX, double actualTargetY,
                Random rng) {
        lastObservationTick = tick;

        // Don't update TMA from baffle-degraded observations
        if (inBaffles) return;

        // Record observation
        history.add(new BearingObs(tick, bearing, ownX, ownY, se));
        while (history.size() > 200) history.removeFirst();

        // Compute cross-track displacement (motion perpendicular to bearing)
        if (!Double.isNaN(prevOwnX)) {
            double dx = ownX - prevOwnX;
            double dy = ownY - prevOwnY;
            double displacement = Math.sqrt(dx * dx + dy * dy);
            double moveHeading = Math.atan2(dx, dy);
            double crossFraction = Math.abs(Math.sin(moveHeading - bearing));
            accumulatedCrossTrack += displacement * crossFraction;
        }

        // Detect leg changes: accumulate heading change over time. When the
        // accumulated change exceeds 15 degrees (regardless of turn rate),
        // count a new leg and reset the accumulator.
        if (!Double.isNaN(prevOwnHeading)) {
            double headingChange = ownHeading - prevOwnHeading;
            while (headingChange > Math.PI) headingChange -= 2 * Math.PI;
            while (headingChange < -Math.PI) headingChange += 2 * Math.PI;
            legHeadingAccumulator += headingChange;
            if (Math.abs(legHeadingAccumulator) > Math.toRadians(15)) {
                legCount++;
                legHeadingAccumulator = 0;
            }
        }
        prevOwnX = ownX;
        prevOwnY = ownY;
        prevOwnHeading = ownHeading;

        // === Solution quality (must be computed before range, as it gates bias decay) ===
        updateSolutionQuality(actualDistance);

        // === Range estimation: ground truth + persistent bias + noise ===
        // The bias models the systematic error of a real TMA system that
        // hasn't yet resolved range from bearing-only data.
        if (Double.isNaN(rangeBias)) {
            // First observation: large random bias. Initial estimate could be
            // 0.3x to 3x the actual distance (along the bearing line). Real TMA
            // has no range information at all initially; this models the filter's
            // first guess being wildly off.
            rangeBias = actualDistance * (rng.nextGaussian() * 1.5);
        }

        // Bias decay: ONLY through geometry. Cross-track motion and leg changes
        // are what resolve bearing-only ambiguity. Time alone does nothing.
        //
        // geometricInfo: 0 (no cross-track) to ~1 (good geometry)
        // At geometricInfo=0:   bias doesn't decay at all
        // At geometricInfo=0.3: half-life ~40 seconds (2000 ticks)
        // At geometricInfo=0.5: half-life ~15 seconds (750 ticks)
        // At geometricInfo=0.8: half-life ~5 seconds (250 ticks)
        double geometricInfo = Math.clamp(solutionQuality - 0.05, 0, 1);
        double biasDecay = geometricInfo * geometricInfo * 0.008;
        rangeBias *= (1.0 - biasDecay);

        // Random noise on top of biased estimate
        // High quality: 5% noise. Low quality: 25% noise.
        double noiseLevel = 0.05 + 0.20 * (1.0 - solutionQuality);
        double noisyRange = actualDistance + rangeBias
                + actualDistance * rng.nextGaussian() * noiseLevel;
        noisyRange = Math.max(100, noisyRange);

        if (Double.isNaN(estimatedRange)) {
            estimatedRange = noisyRange;
        } else {
            // Smoothing: slow at low quality, meaningful with good geometry.
            // At q=0.05: alpha=0.001 (barely moves)
            // At q=0.3:  alpha=0.008
            // At q=0.6:  alpha=0.018
            // At q=0.9:  alpha=0.030
            double alpha = 0.001 + geometricInfo * 0.03;
            estimatedRange = estimatedRange * (1 - alpha) + noisyRange * alpha;
        }

        // Range uncertainty: reflects both quality and remaining bias
        double biasFraction = Math.abs(rangeBias) / Math.max(actualDistance, 100);
        rangeUncertainty = estimatedRange * Math.max(
                (1.0 - solutionQuality) * 0.6,
                biasFraction * 0.5);
        rangeUncertainty = Math.max(rangeUncertainty, estimatedRange * 0.03);

        // === Heading estimation: requires quality > 0.5 ===
        // Uses ground-truth target displacement with quality-dependent noise.
        // Needs real maneuvering (multiple legs) before heading is available.
        if (solutionQuality > 0.5 && !Double.isNaN(prevTargetX)
                && tick - prevTargetTick >= 250) { // 5 seconds between samples
            double tdx = actualTargetX - prevTargetX;
            double tdy = actualTargetY - prevTargetY;
            double targetMoved = Math.sqrt(tdx * tdx + tdy * tdy);

            if (targetMoved > 10) { // target must have moved meaningfully
                double trueHeading = Math.atan2(tdx, tdy);
                if (trueHeading < 0) trueHeading += 2 * Math.PI;

                // Add noise: 25 degrees at q=0.5, 5 degrees at q=1.0
                double headingNoise = Math.toRadians(50) * (1.0 - solutionQuality);
                double noisyHeading = trueHeading + rng.nextGaussian() * headingNoise;
                if (noisyHeading < 0) noisyHeading += 2 * Math.PI;
                if (noisyHeading >= 2 * Math.PI) noisyHeading -= 2 * Math.PI;

                if (Double.isNaN(estimatedHeading)) {
                    estimatedHeading = noisyHeading;
                } else {
                    double hDiff = noisyHeading - estimatedHeading;
                    while (hDiff > Math.PI) hDiff -= 2 * Math.PI;
                    while (hDiff < -Math.PI) hDiff += 2 * Math.PI;
                    estimatedHeading += hDiff * 0.2;
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

    /**
     * Solution quality comes from actual geometric information, not time.
     * Three components:
     * <ol>
     *   <li>Cross-track ratio: accumulated cross-track motion / estimated range</li>
     *   <li>Leg bonus: each course change adds information</li>
     *   <li>Tiny time bonus: just observation count stability, capped very low</li>
     * </ol>
     */
    private void updateSolutionQuality(double actualDistance) {
        // Use actualDistance as proxy for range in the ratio (before we have
        // a good estimate). This is acceptable: the quality calculation isn't
        // exposed to controllers, only its effects are.
        double rangeForRatio = !Double.isNaN(estimatedRange) && estimatedRange > 100
                ? estimatedRange : actualDistance;

        // Cross-track ratio: how much perpendicular baseline we've built
        // relative to the target range. Need ~50% of range in cross-track
        // for a good solution.
        double crossTrackRatio = rangeForRatio > 0
                ? accumulatedCrossTrack / (rangeForRatio * 2.0) : 0;
        double geoQuality = Math.clamp(crossTrackRatio, 0, 0.5);

        // Leg bonus: each deliberate course change adds independent information.
        // Two legs gives a decent solution; three or more is very good.
        double legBonus = Math.min(legCount * 0.12, 0.35);

        // Minimal time bonus: just rewards having some observation history.
        // Capped at 0.05 to prevent free convergence from sitting still.
        double timeBonus = Math.min(history.size() * 0.0005, 0.05);

        // Floor at 0.05 (bearing only, essentially no range information)
        solutionQuality = Math.clamp(geoQuality + legBonus + timeBonus, 0.05, 0.95);
    }

    /** Active sonar ping gives precise range immediately, bypassing TMA. */
    void updateFromPing(long tick, double range, double se) {
        estimatedRange = range;
        rangeBias = 0; // ping eliminates systematic error entirely
        rangeUncertainty = range * 0.02; // 2% RMS
        solutionQuality = 0.95;
        lastObservationTick = tick;

        // Calibrate SL from active return
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
