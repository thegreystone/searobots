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

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Random;

/**
 * Engine-side contact tracker that simulates the output of a Kalman-filter / batch least-squares
 * TMA system. Uses ground-truth distance with quality-dependent noise and bias to model realistic
 * convergence behavior.
 * <p>
 * Key behaviors:
 * <ul>
 * <li>Bearing is always available when a contact is detected</li>
 * <li>Range starts with a large systematic bias (factor 1.5-2x)</li>
 * <li>Bias only decays with cross-track maneuvering and leg changes</li>
 * <li>Time alone gives almost nothing (no free convergence)</li>
 * <li>Heading requires quality &gt; 0.5 (real maneuvering needed)</li>
 * <li>Active sonar bypasses TMA: instant accurate range</li>
 * <li>All measurement errors are correlated over tens of seconds, so averaging a few seconds of
 * samples does not improve them (see {@link CorrelatedNoise})</li>
 * </ul>
 */
final class ContactTracker {
	// Constants
	// Correlation times of the measurement errors (seconds). Bearing wander from
	// array and multipath effects persists for tens of seconds; blade-rate
	// speed tracking re-locks faster; the TMA range and heading estimates are
	// filter outputs and drift slowly.
	static final double BEARING_CORRELATION_S = 20.0;
	static final double SPEED_CORRELATION_S = 10.0;
	static final double RANGE_CORRELATION_S = 20.0;
	static final double HEADING_CORRELATION_S = 60.0;
	// Fraction of sensor error variance that is slow wander; the rest is
	// tick-to-tick jitter, which is the only part averaging can remove.
	static final double SENSOR_CORRELATED_FRACTION = 0.8;

	// Bearing history (capped at 200)
	record BearingObs(long tick, double bearing, double ownX, double ownY, double se) {
	}

	private final Deque<BearingObs> history = new ArrayDeque<>();
	private static final long GEOMETRY_MEMORY_TICKS = 120 * 50;
	private static final long MOTION_WINDOW_TICKS = 5 * 50;
	private static final double MIN_LEG_TRAVEL = 25;

	record TargetObs(long tick, double x, double y) {
	}

	private final Deque<TargetObs> targetMotion = new ArrayDeque<>();

	// Cross-track displacement (accumulated perpendicular motion relative to bearing)
	private double accumulatedCrossTrack;
	private long lastGeometryTick = -1;
	private long lastGeometryAgingTick = -1;
	private double prevOwnX = Double.NaN, prevOwnY = Double.NaN;
	private double prevOwnHeading = Double.NaN;
	private double legTravel;
	private double qualityDebt;

	// Range estimation
	private double estimatedRange = Double.NaN;
	private double rangeBias = Double.NaN; // systematic error, decays with geometry
	private double rangeUncertainty = Double.MAX_VALUE;

	// Heading estimation
	private double estimatedHeading = Double.NaN;

	// Solution quality
	private double solutionQuality;

	// Tick of the last active range fix, or -1. A fix anchors the range
	// solution; its value fades over PING_FIX_MEMORY_S as the target is free
	// to change course and speed.
	private long pingTick = -1;
	static final double PING_FIX_MEMORY_S = 60.0;
	private static final double PING_RANGE_NOISE = 0.02;

	// Contact continuity
	private long lastObservationTick = -1;
	private long lastDecayTick = -1;
	private long lastUsefulObservationTick = -1;
	private double legQuality;
	private double legHeadingAccumulator; // accumulated heading change since last leg count

	// Heading estimation: uses ground truth with quality-dependent noise
	private double motionHeading = Double.NaN;
	private double motionSpeed = Double.NaN;
	private long lastMotionSampleTick = -1;
	private int consistentMotionWindows;

	// Measurement error processes. All errors reported for this contact are
	// correlated in time (see CorrelatedNoise) so that a controller cannot
	// average them away over a few seconds. The sonar model draws the
	// per-tick bearing and blade-rate speed errors from the first two;
	// the tracker uses the last two internally.
	final CorrelatedNoise bearingNoise = new CorrelatedNoise(BEARING_CORRELATION_S, SENSOR_CORRELATED_FRACTION);
	final CorrelatedNoise speedNoise = new CorrelatedNoise(SPEED_CORRELATION_S, SENSOR_CORRELATED_FRACTION);
	private final CorrelatedNoise rangeNoise = new CorrelatedNoise(RANGE_CORRELATION_S, 1.0);
	private final CorrelatedNoise headingNoise = new CorrelatedNoise(HEADING_CORRELATION_S, 1.0);

	/**
	 * Update tracker with a new passive bearing observation. Range is modeled as ground-truth with
	 * a large persistent bias that decays only through cross-track maneuvering. This simulates the
	 * output of a real TMA filter.
	 */
	void update(
		long tick, double bearing, double se, double ownX, double ownY, double ownHeading, boolean inBaffles,
		double actualDistance, double actualTargetX, double actualTargetY, Random rng) {
		// Don't update TMA from baffle-degraded observations
		if (inBaffles) {
			interruptObservations();
			lastObservationTick = tick;
			lastDecayTick = tick;
			return;
		}
		if (lastUsefulObservationTick >= 0 && tick - lastUsefulObservationTick > 1)
			interruptObservations();
		lastObservationTick = tick;
		lastUsefulObservationTick = tick;
		lastDecayTick = tick;

		// Record observation
		history.addLast(new BearingObs(tick, bearing, ownX, ownY, se));
		while (history.size() > 200)
			history.removeFirst();

		// Compute cross-track displacement (motion perpendicular to bearing)
		double crossTrack = 0;
		double displacement = 0;
		boolean newLeg = false;
		if (!Double.isNaN(prevOwnX)) {
			double dx = ownX - prevOwnX;
			double dy = ownY - prevOwnY;
			displacement = Math.sqrt(dx * dx + dy * dy);
			double moveHeading = Math.atan2(dx, dy);
			double crossFraction = Math.abs(Math.sin(moveHeading - bearing));
			crossTrack = displacement * crossFraction;
			legTravel += displacement;
		}

		// Translating turns produce independent bearing legs. Rotation while
		// stationary does not create a baseline, even if the array turns far.
		if (displacement > 0 && !Double.isNaN(prevOwnHeading)) {
			double headingChange = ownHeading - prevOwnHeading;
			while (headingChange > Math.PI)
				headingChange -= 2 * Math.PI;
			while (headingChange < -Math.PI)
				headingChange += 2 * Math.PI;
			legHeadingAccumulator += headingChange;
			if (displacement > 0 && legTravel >= MIN_LEG_TRAVEL
					&& Math.abs(legHeadingAccumulator) > Math.toRadians(15)) {
				newLeg = true;
				legTravel = 0;
				legHeadingAccumulator = 0;
			}
		}
		prevOwnX = ownX;
		prevOwnY = ownY;
		prevOwnHeading = ownHeading;
		if (newLeg || (displacement > 0 && crossTrack > displacement * 0.1)) {
			// Permanently age the prior before adding this fresh baseline. A tiny
			// new movement must not reset a multiplier on the whole old solution.
			ageGeometry(tick - 1);
			lastGeometryTick = tick;
			lastGeometryAgingTick = tick;
		} else {
			ageGeometry(tick);
		}
		accumulatedCrossTrack += crossTrack;
		if (newLeg)
			legQuality = Math.min(0.35, legQuality + 0.12);
		// Missing observations reduce maturity. Only new spatial information can
		// repay that loss; recomputing the old geometry must not undo coasting.
		qualityDebt = Math.max(0, qualityDebt - crossTrack / (2 * rangeScale()) - (newLeg ? 0.12 : 0));
		updateTargetMotion(tick, actualTargetX, actualTargetY);

		// === Solution quality (must be computed before range, as it gates bias decay) ===
		updateSolutionQuality();
		// A recent active fix is worth more than any passive geometry, and
		// fades as it ages instead of vanishing on the next passive tick.
		double fix = pingFixFactor(tick);
		solutionQuality = Math.max(solutionQuality, Math.max(0, 0.95 * fix - qualityDebt));

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
		// High quality: 5% noise. Low quality: 25% noise. The noise is a slow
		// wander (20 s correlation time), not fresh per tick: a filter output
		// drifts, it does not jitter, and drift cannot be averaged away.
		double noiseLevel = 0.05 + 0.20 * (1.0 - solutionQuality);
		// Right after a ping the range is known to the active-return accuracy;
		// the passive wander takes over as the fix ages.
		noiseLevel = noiseLevel * (1.0 - fix) + PING_RANGE_NOISE * fix;
		double noisyRange = actualDistance + rangeBias + actualDistance * rangeNoise.next(tick, rng) * noiseLevel;
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

		// Range uncertainty: reflects both quality and remaining bias. The bias
		// term is kept in metres on purpose: scaling it by the estimate would
		// collapse the reported uncertainty together with the estimate when the
		// initial guess is short and the estimate sits on the 100 m floor,
		// telling the controller "100 m +- 60 m" about a target 500 m away.
		// The wander passes through the smoothing largely intact, so the
		// quality term can never drop below the current noise level.
		double qualityFraction = Math.max((1.0 - solutionQuality) * 0.6, noiseLevel);
		rangeUncertainty = Math.max(estimatedRange * qualityFraction, Math.abs(rangeBias));

		if (solutionQuality > 0.5 && consistentMotionWindows >= 2 && tick == lastMotionSampleTick
				&& Double.isFinite(motionHeading)) {
			double headingSigma = Math.toRadians(50) * (1.0 - solutionQuality);
			double noisyHeading = motionHeading + headingNoise.next(tick, rng) * headingSigma;
			estimatedHeading = Double.isNaN(estimatedHeading) ? normalizeHeading(noisyHeading)
					: normalizeHeading(estimatedHeading + angleDifference(noisyHeading, estimatedHeading) * 0.2);
		}
	}

	/** Motion samples never span discarded or missing bearings. */
	private void interruptObservations() {
		prevOwnX = prevOwnY = prevOwnHeading = Double.NaN;
		legTravel = legHeadingAccumulator = 0;
		targetMotion.clear();
		motionHeading = motionSpeed = estimatedHeading = Double.NaN;
		lastMotionSampleTick = -1;
		consistentMotionWindows = 0;
	}

	private void ageGeometry(long tick) {
		if (lastGeometryTick < 0 || tick <= lastGeometryAgingTick)
			return;
		double previousRemaining = Math.max(1, GEOMETRY_MEMORY_TICKS - (lastGeometryAgingTick - lastGeometryTick));
		double remaining = Math.max(0, GEOMETRY_MEMORY_TICKS - (tick - lastGeometryTick));
		double retained = Math.clamp(remaining / previousRemaining, 0, 1);
		accumulatedCrossTrack *= retained;
		legQuality *= retained;
		legTravel *= retained;
		lastGeometryAgingTick = tick;
		if (remaining == 0) {
			accumulatedCrossTrack = 0;
			legQuality = 0;
			legTravel = legHeadingAccumulator = 0;
			lastGeometryTick = -1;
		}
	}

	/**
	 * Keep a rolling five-second displacement window, independent of when range quality first
	 * crossed its gate. Contradictory motion invalidates the old geometry; a new passive solution
	 * requires new translating listener legs.
	 */
	private void updateTargetMotion(long tick, double x, double y) {
		targetMotion.addLast(new TargetObs(tick, x, y));
		while (targetMotion.size() > 1 && tick - targetMotion.getFirst().tick() > MOTION_WINDOW_TICKS)
			targetMotion.removeFirst();
		var first = targetMotion.getFirst();
		if (tick - first.tick() < MOTION_WINDOW_TICKS)
			return;
		double dx = x - first.x(), dy = y - first.y();
		double speed = Math.hypot(dx, dy) / 5.0;
		double heading = speed > 2 ? normalizeHeading(Math.atan2(dx, dy)) : Double.NaN;
		boolean wasMoving = Double.isFinite(motionHeading);
		boolean moving = Double.isFinite(heading);
		boolean changed = Double.isFinite(motionSpeed) && (wasMoving != moving
				|| (moving && Math.abs(angleDifference(heading, motionHeading)) > Math.toRadians(30))
				|| Math.abs(speed - motionSpeed) > 2);
		if (changed) {
			accumulatedCrossTrack = 0;
			lastGeometryTick = -1;
			legQuality = 0;
			legTravel = legHeadingAccumulator = 0;
			history.clear();
			pingTick = -1;
			estimatedHeading = Double.NaN;
			consistentMotionWindows = 0;
			lastMotionSampleTick = tick;
			motionHeading = heading;
			motionSpeed = speed;
			return;
		}
		if (!Double.isFinite(heading)) {
			estimatedHeading = motionHeading = Double.NaN;
			motionSpeed = speed;
			consistentMotionWindows = 0;
			return;
		}
		if (lastMotionSampleTick < 0 || tick - lastMotionSampleTick >= MOTION_WINDOW_TICKS) {
			motionHeading = heading;
			motionSpeed = speed;
			lastMotionSampleTick = tick;
			consistentMotionWindows++;
		}
	}

	private static double angleDifference(double a, double b) {
		return Math.atan2(Math.sin(a - b), Math.cos(a - b));
	}

	private static double normalizeHeading(double heading) {
		return (heading % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
	}

	private double rangeScale() {
		return !Double.isNaN(estimatedRange) && estimatedRange > 100 ? estimatedRange : Math.max(100, rangeUncertainty);
	}

	/**
	 * Solution quality comes from actual geometric information, not time. Three components:
	 * <ol>
	 * <li>Cross-track ratio: accumulated cross-track motion / estimated range</li>
	 * <li>Leg bonus: each course change adds information</li>
	 * <li>Tiny time bonus: just observation count stability, capped very low</li>
	 * </ol>
	 */
	private void updateSolutionQuality() {
		// Quality is public too: never normalize a short initial guess by hidden true range.
		// At the range floor, use the reported uncertainty as a conservative baseline scale.
		double rangeForRatio = rangeScale();

		// Cross-track ratio: how much perpendicular baseline we've built
		// relative to the target range. Need ~50% of range in cross-track
		// for a good solution.
		double crossTrackRatio = rangeForRatio > 0 ? accumulatedCrossTrack / (rangeForRatio * 2.0) : 0;
		double geoQuality = Math.clamp(crossTrackRatio, 0, 0.5);

		// Leg bonus: each deliberate course change adds independent information.
		// Two legs gives a decent solution; three or more is very good.
		double legBonus = legQuality;

		// Minimal time bonus: just rewards having some observation history.
		// Capped at 0.05 to prevent free convergence from sitting still.
		double timeBonus = Math.min(history.size() * 0.0005, 0.05);

		// Floor at 0.05 (bearing only, essentially no range information)
		// A coherent moving track can keep extending its baseline. Once useful
		// ownship geometry stops, its information fades over two minutes instead
		// of remaining a permanent source of confidence.
		double geometryFreshness = lastGeometryTick < 0 ? 0 : 1;
		solutionQuality = Math.clamp((geoQuality + legBonus) * geometryFreshness + timeBonus - qualityDebt, 0.05, 0.95);
	}

	/**
	 * Active sonar ping gives precise range immediately, bypassing TMA.
	 */
	void updateFromPing(long tick, double range) {
		estimatedRange = range;
		rangeBias = 0; // ping eliminates systematic error entirely
		rangeUncertainty = range * PING_RANGE_NOISE; // 2% RMS
		solutionQuality = 0.95;
		qualityDebt = 0;
		lastObservationTick = tick;
		lastDecayTick = tick;
		pingTick = tick;
	}

	/** 1.0 at the moment of an active fix, decaying to 0 as the fix ages; 0 if never pinged. */
	private double pingFixFactor(long tick) {
		if (pingTick < 0)
			return 0;
		return Math.exp(-(tick - pingTick) / (PING_FIX_MEMORY_S * CorrelatedNoise.TICKS_PER_SECOND));
	}

	void decay(long tick, double maxSubSpeed) {
		if (lastObservationTick < 0 || tick <= lastDecayTick)
			return;
		// Integrate only the unaccounted interval. Sonar calls this on every missed
		// tick; applying the whole observation age each time would compound decay.
		double dtSec = (tick - lastDecayTick) / CorrelatedNoise.TICKS_PER_SECOND;
		lastDecayTick = tick;
		solutionQuality = Math.max(0, solutionQuality - 0.01 * dtSec);
		qualityDebt = Math.min(0.95, qualityDebt + 0.01 * dtSec);
		rangeUncertainty += maxSubSpeed * dtSec;
		interruptObservations();
		ageGeometry(tick);
	}

	boolean isExpired(long tick) {
		return lastObservationTick >= 0 && (tick - lastObservationTick) > 1500;
	}

	// Accessors
	double estimatedRange() {
		return Double.isNaN(estimatedRange) ? 0 : estimatedRange;
	}

	double rangeUncertainty() {
		return rangeUncertainty;
	}

	double solutionQuality() {
		return solutionQuality;
	}

	double estimatedHeading() {
		return solutionQuality > 0.5 && consistentMotionWindows >= 2 ? estimatedHeading : Double.NaN;
	}

	long lastObservationTick() {
		return lastObservationTick;
	}
}
