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
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/** Checks that public acoustic measurements cannot bypass bearing-only TMA. */
class PassiveSonarInformationTest {

	private static final TerrainMap TERRAIN = deepFlat();
	private static final double QUIET_LISTENER_NOISE_DB = 55;
	private static final int SAMPLE_SEEDS = 48;

	@Test
	void solutionQualityCannotRevealTrueRangeBehindAnInitialRangeFloor() {
		double actualDistance = 1000;
		ContactTracker tracker = null;
		Random random = null;
		double initialUncertainty = 0;
		// Select an unresolved public solution, without depending on a particular noise draw.
		for (int seed = 0; seed < 256; seed++) {
			var candidate = new ContactTracker();
			var candidateRandom = new Random(seed);
			candidate.update(0, 0, 20, 0, 0, 0, false, actualDistance, 0, 1000, candidateRandom);
			if (candidate.estimatedRange() == 100 && candidate.rangeUncertainty() > actualDistance
					&& candidate.rangeUncertainty() < 5000) {
				tracker = candidate;
				random = candidateRandom;
				initialUncertainty = candidate.rangeUncertainty();
				break;
			}
		}
		assertNotNull(tracker, "The fixture needs an unresolved range floor with substantial reported uncertainty");

		// One perpendicular displacement, no course change: decode the public quality curve.
		tracker.update(1, 0, 20, 600, 0, 0, false, actualDistance, 600, 1000, random);
		double inferredBaseline = 600 / (2 * (tracker.solutionQuality() - 0.001));
		assertEquals(initialUncertainty, inferredBaseline, 1e-7,
				"Quality must use the public uncertainty when the public range is still at its initial floor");
		assertNotEquals(actualDistance, inferredBaseline, 1e-7,
				"Maneuvering must not reveal hidden exact range through the solution-quality denominator");
	}

	@Test
	void amplitudeInversionReturnsTheReportedTmaRange() {
		var listener = submarine(0, 0, 0, 80);
		var source = submarine(1, 2000, Math.PI, 110);
		var contact = new SonarModel(42).computeContacts(0, List.of(listener, source), TERRAIN, List.of())
				.get(listener.id()).passiveContacts().getFirst();

		assertTrue(contact.range() > 0);
		assertTrue(contact.rangeUncertainty() > contact.range() * 0.3,
				"An unmaneuvered contact must still have a poor range solution");
		assertEquals(contact.range(), rangeFromAmplitude(contact), contact.range() * 1e-10,
				"Estimated source level and received strength must not provide a second, better range");
	}

	@Test
	void baffleContactHasNoIndependentSourceLevelWithoutARangeSolution() {
		var listener = submarine(0, 0, Math.PI, 80);
		var source = submarine(1, 500, Math.PI, 125);
		var contact = new SonarModel(42).computeContacts(0, List.of(listener, source), TERRAIN, List.of())
				.get(listener.id()).passiveContacts().getFirst();

		assertTrue(contact.range() <= 0 || Double.isNaN(contact.range()), "Baffles do not create TMA range");
		assertTrue(Double.isNaN(contact.estimatedSourceLevel()),
				"Without a range, received loudness cannot determine the target's radiated level");
	}

	@Test
	void passiveTorpedoPairsDoNotExposeRadiatedSourceLevel() {
		var listener = submarine(0, 0, 0, 80);
		var weapon = torpedo(100, 1000, Math.PI, 110);
		var contacts = new SonarModel(42).computeContacts(0, List.of(listener), List.of(weapon), TERRAIN, List.of());
		var heardWeapon = contacts.get(listener.id()).passiveContacts().getFirst();

		var quietWeapon = torpedo(100, 0, 0, 80);
		var source = submarine(1, 1000, Math.PI, 110);
		var heardSub = new SonarModel(42).computeContacts(0, List.of(source), List.of(quietWeapon), TERRAIN, List.of())
				.get(quietWeapon.id()).passiveContacts().getFirst();

		for (var contact : List.of(heardWeapon, heardSub)) {
			assertEquals(0, contact.range(), "Untracked passive pairs have no range measurement");
			assertTrue(Double.isNaN(contact.estimatedSourceLevel()),
					"A torpedo-involved passive contact must not expose exact source-level telemetry");
		}
	}

	@Test
	void activeEchoOfAnOtherwiseInaudibleTargetHasNoRadiatedSourceLevel() {
		var listener = submarine(0, 0, 0, 80);
		var source = submarine(1, 1000, Math.PI, 70);
		listener.activeSonarPing();
		var contacts = new SonarModel(42).computeContacts(0, List.of(listener, source), TERRAIN, List.of())
				.get(listener.id());

		assertTrue(contacts.passiveContacts().isEmpty());
		var echo = contacts.activeReturns().getFirst();
		assertEquals(1000, echo.range(), 100, "The echo still supplies a useful independent range fix");
		assertTrue(Double.isNaN(echo.estimatedSourceLevel()),
				"Echo strength measures reflection, not an inaudible target's machinery noise");
	}

	@Test
	void activeRangeCalibratesSourceLevelUsingTheSamePassiveMeasurement() {
		var listener = submarine(0, 0, 0, 80);
		var source = submarine(1, 1000, Math.PI, 110);
		listener.activeSonarPing();
		var contacts = new SonarModel(42).computeContacts(0, List.of(listener, source), TERRAIN, List.of())
				.get(listener.id());
		var passive = contacts.passiveContacts().getFirst();
		var echo = contacts.activeReturns().getFirst();

		assertEquals(1000, echo.range(), 100);
		double expectedLevel = passive.signalExcess() + QUIET_LISTENER_NOISE_DB + 10 * Math.log10(echo.range());
		assertEquals(expectedLevel, echo.estimatedSourceLevel(), 1e-9,
				"The active fix calibrates measured amplitude; it must not reveal the true radiated level");
	}

	@Test
	void torpedoInvolvedEchoesDoNotInventRadiatedSourceLevel() {
		var listener = submarine(0, 0, 0, 80);
		var weapon = torpedo(100, 1000, Math.PI, 110);
		listener.activeSonarPing();
		weapon.createOutput().activeSonarPing();
		var contacts = new SonarModel(42).computeContacts(0, List.of(listener), List.of(weapon), TERRAIN, List.of());

		assertTrue(Double.isNaN(contacts.get(listener.id()).activeReturns().getFirst().estimatedSourceLevel()));
		assertTrue(Double.isNaN(contacts.get(weapon.id()).activeReturns().getFirst().estimatedSourceLevel()));
	}

	@Test
	void averagingKnownStrengthPingsForAMinuteCannotRecoverExactRange() {
		double[] errors = new double[SAMPLE_SEEDS];
		for (int seed = 0; seed < errors.length; seed++) {
			var sonar = new SonarModel(seed);
			var source = submarine(1, 1000, Math.PI, 80);
			var listener = torpedo(100, 0, 0, 80);
			errors[seed] = Math.abs(averagePingStrengthError(sonar, source, listener, 0));
		}
		Arrays.sort(errors);
		assertTrue(errors[errors.length / 2] > 2,
				"Known-strength pings retain calibration error after a minute of averaging; median error was "
						+ errors[errors.length / 2] + " dB");
	}

	@Test
	void bearingUncertaintyCannotBeInvertedToObtainTrueReceivedStrength() {
		double[] errors = new double[SAMPLE_SEEDS];
		for (int seed = 0; seed < errors.length; seed++) {
			var listener = submarine(0, 0, 0, 80);
			var source = submarine(1, 1000, Math.PI, 104);
			var contact = new SonarModel(seed).computeContacts(0, List.of(listener, source), TERRAIN, List.of())
					.get(listener.id()).passiveContacts().getFirst();
			// Attack the documented uncertainty curve where it has not reached the accuracy floor.
			double strengthFromUncertainty = 30 / Math.toDegrees(contact.bearingUncertainty());
			double actualStrength = 104 - 30 - QUIET_LISTENER_NOISE_DB;
			errors[seed] = Math.abs(strengthFromUncertainty - actualStrength);
		}
		Arrays.sort(errors);
		assertTrue(errors[errors.length / 2] > 2,
				"Bearing precision must not expose the true signal excess through another public field");
	}

	@Test
	void losingAndReacquiringAContactDoesNotRerollItsAmplitudeCalibration() {
		double[] before = new double[SAMPLE_SEEDS];
		double[] after = new double[SAMPLE_SEEDS];
		for (int seed = 0; seed < before.length; seed++) {
			var sonar = new SonarModel(seed);
			var source = submarine(1, 1000, Math.PI, 80);
			var listener = torpedo(100, 0, 0, 80);
			before[seed] = averagePingStrengthError(sonar, source, listener, 0);
			// No source for over the sensor-noise retention interval.
			sonar.computeContacts(5000, List.of(), List.of(listener), TERRAIN, List.of());
			after[seed] = averagePingStrengthError(sonar, source, listener, 5001);
		}
		double beforeMean = Arrays.stream(before).average().orElseThrow();
		double afterMean = Arrays.stream(after).average().orElseThrow();
		double covariance = 0;
		for (int i = 0; i < before.length; i++) {
			covariance += (before[i] - beforeMean) * (after[i] - afterMean);
		}
		covariance /= before.length;
		assertTrue(covariance > 12,
				"Persistent calibration should survive reacquisition; covariance was " + covariance + " dB²");
	}

	private static double rangeFromAmplitude(SonarContact contact) {
		return Math.pow(10, (contact.estimatedSourceLevel() - contact.signalExcess() - QUIET_LISTENER_NOISE_DB) / 10);
	}

	private static double averagePingStrengthError(
		SonarModel sonar, SubmarineEntity source, TorpedoEntity listener, long startTick) {
		double sum = 0;
		int samples = 60;
		for (int sample = 0; sample < samples; sample++) {
			// Repeated pulses emulate an observer accumulating known-strength transmissions.
			source.setActiveSonarCooldown(0);
			source.activeSonarPing();
			var contact = sonar
					.computeContacts(startTick + sample * 50L, List.of(source), List.of(listener), TERRAIN, List.of())
					.get(listener.id()).passiveContacts().getFirst();
			double actualStrength = 220 - 30 - QUIET_LISTENER_NOISE_DB;
			sum += contact.signalExcess() - actualStrength;
		}
		return sum / samples;
	}

	private static SubmarineEntity submarine(int id, double y, double heading, double sourceLevel) {
		var entity = new SubmarineEntity(VehicleConfig.submarine(), id, null, new Vec3(0, y, -200), heading,
				Color.GREEN, 1000);
		entity.setSourceLevelDb(sourceLevel);
		entity.setSpeed(8);
		return entity;
	}

	private static TorpedoEntity torpedo(int id, double y, double heading, double sourceLevel) {
		var entity = new TorpedoEntity(id, 99, VehicleConfig.torpedo(), null, new Vec3(0, y, -200), heading, 0, 20,
				Color.GREEN);
		entity.setSourceLevelDb(sourceLevel);
		entity.setSpeed(8);
		return entity;
	}

	private static TerrainMap deepFlat() {
		int size = 201;
		double[] elevations = new double[size * size];
		Arrays.fill(elevations, -500);
		return new TerrainMap(elevations, size, size, -10000, -10000, 100);
	}
}
