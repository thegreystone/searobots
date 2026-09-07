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

import java.util.Random;

/**
 * Unit-variance Gaussian measurement error that is correlated in time: a first-order Gauss-Markov
 * (Ornstein-Uhlenbeck) process sampled at tick resolution, optionally mixed with a white component.
 * <p>
 * Real sonar errors are not fresh every 20 ms. Array calibration, multipath, and the integration time of the
 * beamformer make consecutive bearing estimates share most of their error for tens of seconds. Modelling the error
 * as independent per tick would let a controller average 50 samples over one second and cut a 10 degree error to
 * under 1.5 degrees, which no real system can do. With this process, averaging over a window shorter than the
 * correlation time gives almost nothing; a window of ten correlation times is needed for a four-fold gain.
 * <p>
 * Every sample is marginally N(0, 1) regardless of how it is correlated with its neighbours, so callers scale by the
 * current 1-sigma error and the uncertainty they report stays honest.
 */
final class CorrelatedNoise {
	static final double TICKS_PER_SECOND = 50.0;

	private final double tauTicks;
	private final double slowWeight;
	private final double whiteWeight;

	private double slow = Double.NaN;
	private long lastTick;

	/**
	 * @param correlationTimeSeconds
	 * 		time constant of the slow component; the autocorrelation drops to 1/e over this interval
	 * @param correlatedFraction
	 * 		fraction of the variance carried by the slow component (1.0 = no white jitter at all)
	 */
	CorrelatedNoise(double correlationTimeSeconds, double correlatedFraction) {
		this.tauTicks = correlationTimeSeconds * TICKS_PER_SECOND;
		this.slowWeight = Math.sqrt(correlatedFraction);
		this.whiteWeight = Math.sqrt(1.0 - correlatedFraction);
	}

	/**
	 * Advance to {@code tick} and return the unit-variance error sample for it. Gaps between calls (contact lost and
	 * regained) decorrelate the slow component by the elapsed time, so a long gap gives a fresh draw.
	 */
	double next(long tick, Random rng) {
		double w = rng.nextGaussian();
		if (Double.isNaN(slow)) {
			slow = w;
		} else {
			double dt = Math.max(0, tick - lastTick);
			double rho = Math.exp(-dt / tauTicks);
			slow = rho * slow + Math.sqrt(1.0 - rho * rho) * w;
		}
		lastTick = tick;
		if (whiteWeight == 0.0)
			return slow;
		return slowWeight * slow + whiteWeight * rng.nextGaussian();
	}
}
