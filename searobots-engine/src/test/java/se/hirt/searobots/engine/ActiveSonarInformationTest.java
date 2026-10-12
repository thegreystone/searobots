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
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Active range accuracy cannot be bypassed through echo amplitude or uncertainty metadata. */
class ActiveSonarInformationTest {

	private static final TerrainMap TERRAIN = deepFlat();
	private static final int SAMPLE_SEEDS = 48;
	private static final double ACTUAL_RANGE = 1000;
	private static final double QUIET_NOISE = 55;

	enum PairKind {
		SUB_SUB, SUB_TORPEDO, TORPEDO_SUB
	}

	@Test
	void rangeUncertaintyInversionOnlyRecoversTheReportedNoisyRange() {
		for (var kind : PairKind.values()) {
			var fixture = new Fixture(kind, 42, QUIET_NOISE);
			var echo = fixture.echo(0);
			assertTrue(echo.isActive());
			assertEquals(echo.range(), echo.rangeUncertainty() / SonarModel.RANGE_NOISE_FRACTION, 1e-9);
			assertNotEquals(ACTUAL_RANGE, echo.rangeUncertainty() / SonarModel.RANGE_NOISE_FRACTION, 1e-6,
					kind + " uncertainty must not expose hidden exact range");
		}
	}

	@Test
	void echoRangesRetainApproximatelyTwoPercentMeasurementError() {
		for (var kind : PairKind.values()) {
			double squaredError = 0;
			for (int seed = 0; seed < SAMPLE_SEEDS; seed++) {
				double error = new Fixture(kind, seed, QUIET_NOISE).echo(0).range() / ACTUAL_RANGE - 1;
				squaredError += error * error;
			}
			double rms = Math.sqrt(squaredError / SAMPLE_SEEDS);
			assertTrue(rms > 0.01 && rms < 0.035,
					kind + " should retain useful but imperfect active ranging; RMS fraction=" + rms);
		}
	}

	@Test
	void echoStrengthCannotBeInvertedToRecoverExactRange() {
		for (var kind : PairKind.values()) {
			double[] relativeErrors = new double[SAMPLE_SEEDS];
			for (int seed = 0; seed < SAMPLE_SEEDS; seed++) {
				var echo = new Fixture(kind, seed, QUIET_NOISE).echo(0);
				double inferredRange = Math.pow(10, (SonarModel.ACTIVE_PING_SL_DB + SonarModel.TARGET_STRENGTH_DB
						- QUIET_NOISE - echo.signalExcess()) / (2 * SonarModel.SPREADING_COEFFICIENT));
				relativeErrors[seed] = Math.abs(inferredRange / ACTUAL_RANGE - 1);
			}
			assertTrue(median(relativeErrors) > 0.05,
					kind + " uncertain reflection strength cannot supply a second precise range measurement");
		}
	}

	@Test
	void bearingPrecisionOnlyExposesMeasuredEchoStrength() {
		// Strong own machinery puts the documented bearing curve away from its accuracy floor.
		double listenerNoise = 160;
		double trueSe = SonarModel.ACTIVE_PING_SL_DB + SonarModel.TARGET_STRENGTH_DB - listenerNoise
				- 2 * SonarModel.SPREADING_COEFFICIENT * Math.log10(ACTUAL_RANGE);
		for (var kind : PairKind.values()) {
			double[] errors = new double[SAMPLE_SEEDS];
			for (int seed = 0; seed < SAMPLE_SEEDS; seed++) {
				var echo = new Fixture(kind, seed, listenerNoise).echo(0);
				double decodedStrength = 30 / Math.toDegrees(echo.bearingUncertainty());
				assertEquals(echo.signalExcess(), decodedStrength, 1e-9);
				errors[seed] = Math.abs(decodedStrength - trueSe);
			}
			assertTrue(median(errors) > 1, kind + " precision must not leak the exact sonar-equation strength");
		}
	}

	@Test
	void calibrationDoesNotChangeThePhysicalActiveDetectionThreshold() {
		// At 1 km the echo before listener noise is 180 dB. Test both sides of SE=5 dB.
		for (var kind : PairKind.values()) {
			for (int seed = 0; seed < SAMPLE_SEEDS; seed++) {
				assertEquals(1, new Fixture(kind, seed, 174.9).measure(0).activeReturns().size(),
						kind + " a positive measurement error is not needed to detect a physically audible echo");
				assertTrue(new Fixture(kind, seed, 175.1).measure(0).activeReturns().isEmpty(),
						kind + " calibration error must not create an echo below the physical threshold");
			}
		}
	}

	@Test
	void averagingEchoStrengthCannotRemoveCalibrationInAFewSeconds() {
		for (var kind : PairKind.values()) {
			double[] errors = new double[SAMPLE_SEEDS];
			for (int seed = 0; seed < SAMPLE_SEEDS; seed++) {
				var fixture = new Fixture(kind, seed, QUIET_NOISE);
				double sum = 0;
				for (int sample = 0; sample < 100; sample++) {
					// Forced pulses exercise the measurement channel at a higher cadence than gameplay permits.
					sum += fixture.echo(sample * 2L).signalExcess() - fixture.trueStrength();
				}
				errors[seed] = Math.abs(sum / 100);
			}
			assertTrue(median(errors) > 1,
					kind + " repeated echo samples must retain reflection/calibration error after averaging");
		}
	}

	@Test
	void simultaneousSourcePingAndEchoCannotCancelTheirAmplitudeErrors() {
		for (var kind : PairKind.values()) {
			double[] errors = new double[SAMPLE_SEEDS];
			for (int seed = 0; seed < SAMPLE_SEEDS; seed++) {
				var fixture = new Fixture(kind, seed, QUIET_NOISE);
				fixture.pingSource();
				var result = fixture.measure(0);
				double difference = result.passiveContacts().getFirst().signalExcess()
						- result.activeReturns().getFirst().signalExcess();
				// Same ping level and receiver noise cancel, leaving TL-TS only if both errors cancel.
				double actualDifference = SonarModel.SPREADING_COEFFICIENT * Math.log10(ACTUAL_RANGE)
						- SonarModel.TARGET_STRENGTH_DB;
				errors[seed] = Math.abs(difference - actualDifference);
			}
			assertTrue(median(errors) > 1,
					kind + " passive and reflected strength need independent calibration errors");
		}
	}

	@Test
	void contactMeasurementsDoNotExposeAnInternalPairIdentifier() {
		var names = Arrays.stream(SonarContact.class.getRecordComponents()).map(component -> component.getName())
				.toList();
		for (String identifier : List.of("id", "sourceId", "targetId", "listenerId", "trackerKey")) {
			assertFalse(names.contains(identifier), "Pair identity is engine state, not a sonar measurement");
		}
	}

	private static double median(double[] values) {
		Arrays.sort(values);
		return values[values.length / 2];
	}

	private static final class Fixture {
		final PairKind kind;
		final SonarModel sonar;
		final SubmarineEntity sub;
		final SubmarineEntity source;
		final TorpedoEntity torpedo;
		final double listenerNoise;

		Fixture(PairKind kind, int seed, double listenerNoise) {
			this.kind = kind;
			this.listenerNoise = listenerNoise;
			sonar = new SonarModel(seed);
			sub = submarine(0, kind == PairKind.TORPEDO_SUB ? ACTUAL_RANGE : 0);
			source = submarine(1, ACTUAL_RANGE);
			torpedo = new TorpedoEntity(100, 99, VehicleConfig.torpedo(), null,
					new Vec3(0, kind == PairKind.SUB_TORPEDO ? ACTUAL_RANGE : 0, -200), 0, 0, 20, Color.YELLOW);
			torpedo.setSourceLevelDb(80);
			if (kind == PairKind.TORPEDO_SUB) {
				torpedo.setSourceLevelDb(listenerNoise + torpedo.vehicleConfig().sonarSelfNoiseOffsetDb());
			} else {
				sub.setSourceLevelDb(listenerNoise + sub.vehicleConfig().sonarSelfNoiseOffsetDb());
			}
		}

		double trueStrength() {
			return SonarModel.ACTIVE_PING_SL_DB + SonarModel.TARGET_STRENGTH_DB - listenerNoise
					- 2 * SonarModel.SPREADING_COEFFICIENT * Math.log10(ACTUAL_RANGE);
		}

		void pingSource() {
			if (kind == PairKind.SUB_SUB) {
				source.activeSonarPing();
			} else if (kind == PairKind.SUB_TORPEDO) {
				torpedo.createOutput().activeSonarPing();
			} else {
				sub.activeSonarPing();
			}
		}

		SonarContact echo(long tick) {
			return measure(tick).activeReturns().getFirst();
		}

		SonarModel.SonarResult measure(long tick) {
			if (kind == PairKind.TORPEDO_SUB) {
				torpedo.setActiveSonarCooldown(0);
				torpedo.createOutput().activeSonarPing();
			} else {
				sub.setActiveSonarCooldown(0);
				sub.activeSonarPing();
			}
			List<SubmarineEntity> subs = kind == PairKind.SUB_SUB ? List.of(sub, source) : List.of(sub);
			List<TorpedoEntity> torps = kind == PairKind.SUB_SUB ? List.of() : List.of(torpedo);
			return sonar.computeContacts(tick, subs, torps, TERRAIN, List.of())
					.get(kind == PairKind.TORPEDO_SUB ? torpedo.id() : sub.id());
		}
	}

	private static SubmarineEntity submarine(int id, double y) {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), id, null, new Vec3(0, y, -200), 0, Color.GREEN, 1000);
		sub.setSourceLevelDb(80);
		sub.setSpeed(8);
		return sub;
	}

	private static TerrainMap deepFlat() {
		int size = 81;
		double[] depths = new double[size * size];
		Arrays.fill(depths, -500);
		return new TerrainMap(depths, size, size, -4000, -4000, 100);
	}
}
