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

import se.hirt.searobots.api.MatchConfig;

import java.util.IdentityHashMap;
import java.util.Map;
import java.util.SplittableRandom;
import java.util.function.IntToDoubleFunction;

/**
 * Pressure-hull damage and cumulative implosion hazard for one match. Damage weakens the hull,
 * while a seeded exponential failure threshold makes risk independent of tick frequency and entity
 * iteration order. See docs/pressure-hull-model.md for the gameplay calibration.
 */
final class HullPressureModel {

	private static final double DAMAGE_FRACTION_PER_SECOND = 0.01;
	private static final double DAMAGE_WEAKENING = 0.35;
	private static final double IMPLOSION_RATE = Math.log(2.0) / 5.0;
	private static final int HAZARD_EXPONENT = 8;
	private static final long PRESSURE_SEED = 0x48554c4c50524553L;
	private static final long ENTITY_SEED_MULTIPLIER = 0x9e3779b97f4a7c15L;

	private final double ratedDepth;
	private final double crushDepth;
	private final IntToDoubleFunction failureThresholdForId;
	private final Map<SubmarineEntity, Exposure> exposures = new IdentityHashMap<>();

	HullPressureModel(MatchConfig config) {
		this(config, id -> {
			var random = new SplittableRandom(
					config.worldSeed() ^ PRESSURE_SEED ^ (ENTITY_SEED_MULTIPLIER * (id + 1L)));
			double uniform;
			do {
				uniform = random.nextDouble();
			} while (uniform == 0.0);
			return -Math.log(uniform);
		});
	}

	HullPressureModel(MatchConfig config, IntToDoubleFunction failureThresholdForId) {
		if (!Double.isFinite(config.ratedDepth()) || !Double.isFinite(config.crushDepth()) || config.ratedDepth() > 0
				|| config.crushDepth() >= config.ratedDepth()) {
			throw new IllegalArgumentException("Crush depth must be deeper than a finite, nonpositive rated depth");
		}
		ratedDepth = -config.ratedDepth();
		crushDepth = -config.crushDepth();
		this.failureThresholdForId = failureThresholdForId;
	}

	/** Applies pressure at the supplied centre Z, before terrain can correct the position. */
	void step(SubmarineEntity sub, double dt, double z) {
		var vehicle = sub.vehicleConfig();
		if (sub.forfeited() || sub.hp() <= 0 || sub.maxHp() <= 0 || vehicle.surfaceLocked() || !vehicle.hasBallast()
				|| !Double.isFinite(z)) {
			return;
		}
		double depth = -z;
		if (depth >= crushDepth) {
			sub.setHp(0);
			return;
		}
		if (depth <= ratedDepth || !Double.isFinite(dt) || dt <= 0) {
			return;
		}

		var exposure = exposures.computeIfAbsent(sub,
				entity -> new Exposure(failureThresholdForId.applyAsDouble(entity.id())));
		// Include pending fractional HP loss so weakening is smooth even at high tick rates.
		double damageFraction = Math.clamp(1.0 - (sub.hp() - exposure.fractionalDamage) / sub.maxHp(), 0.0, 1.0);
		double collapseScale = ratedDepth + (crushDepth - ratedDepth) * (1.0 - DAMAGE_WEAKENING * damageFraction);
		double stress = (depth - ratedDepth) / (collapseScale - ratedDepth);
		exposure.hazard += IMPLOSION_RATE * Math.pow(stress, HAZARD_EXPONENT) * dt;
		exposure.fractionalDamage += sub.maxHp() * DAMAGE_FRACTION_PER_SECOND * stress * stress * dt;

		if (exposure.hazard >= exposure.failureThreshold || exposure.fractionalDamage >= sub.hp()) {
			sub.setHp(0);
			return;
		}
		int damage = (int) exposure.fractionalDamage;
		sub.setHp(sub.hp() - damage);
		exposure.fractionalDamage -= damage;
	}

	private static final class Exposure {
		final double failureThreshold;
		double hazard;
		double fractionalDamage;

		Exposure(double failureThreshold) {
			if (!Double.isFinite(failureThreshold) || failureThreshold <= 0) {
				throw new IllegalArgumentException("Failure threshold must be finite and positive");
			}
			this.failureThreshold = failureThreshold;
		}
	}
}
