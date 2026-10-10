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
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/** Lifecycle scenarios for the existing truth-plus-correlated-noise TMA model. */
class ContactTrackerLifecycleTest {

	@Test
	void audibleBafflesAgeTheSolutionWithoutLosingAcousticContinuity() throws Exception {
		var fixture = new SonarFixture();
		fixture.own.activeSonarPing();
		var initial = fixture.measure(0).activeReturns().getFirst();
		var originalTracker = fixture.tracker();
		fixture.own.setHeading(Math.PI);
		SonarContact latest = null;
		for (long tick = 1; tick <= 600; tick++) {
			var contacts = fixture.measure(tick).passiveContacts();
			assertEquals(1, contacts.size(), "The deliberately loud source remains audible through the baffles");
			latest = contacts.getFirst();
		}

		assertSame(originalTracker, fixture.tracker(), "Audibility retains the internal pair's tracker");
		assertTrue(latest.solutionQuality() <= initial.solutionQuality() - 0.1,
				"Twelve seconds of degraded bearings cannot preserve the original TMA confidence");
		assertTrue(latest.rangeUncertainty() >= initial.rangeUncertainty() + 179.9,
				"A baffled solution must coast at the configured 15 m/s uncertainty growth rate");
	}

	@Test
	void missingIntervalCannotBeCreditedAsObservedCrossTrackOrANewLeg() {
		var tracker = new ContactTracker();
		var rng = zeroNoise();
		observe(tracker, rng, 0, 0, 0, 0, false, 0, 500);
		for (long tick = 1; tick < 600; tick++) {
			tracker.decay(tick, 15);
		}
		// The listener travelled and turned while there was no usable bearing history.
		observe(tracker, rng, 600, 180, 0, Math.PI / 2, false, 0, 500);
		assertTrue(tracker.solutionQuality() <= 0.1,
				"Reacquisition is one bearing, not 180 m of newly observed triangulation");
		assertTrue(Double.isNaN(tracker.estimatedHeading()));
	}

	@Test
	void baffledIntervalCannotBeCreditedAsObservedCrossTrackOrANewLeg() {
		var tracker = new ContactTracker();
		var rng = zeroNoise();
		observe(tracker, rng, 0, 0, 0, 0, false, 0, 500);
		for (long tick = 1; tick <= 600; tick++) {
			observe(tracker, rng, tick, 0.3 * tick, 0, Math.PI, true, 0, 500);
		}
		observe(tracker, rng, 601, 180, 0, Math.PI / 2, false, 0, 500);
		assertTrue(tracker.solutionQuality() <= 0.1,
				"Audible but discarded bearings cannot turn a blind transit into useful TMA geometry");
	}

	@Test
	void turningInPlaceCannotBuildIndependentRangeGeometry() {
		var tracker = new ContactTracker();
		var rng = zeroNoise();
		for (long tick = 0; tick <= 6000; tick++) {
			// Four complete rotations at a fixed location, with continuous clean bearings.
			observe(tracker, rng, tick, 0, 0, tick * 8 * Math.PI / 6000, false, 0, 2000);
		}
		assertTrue(tracker.solutionQuality() <= 0.1,
				"Array rotation supplies no spatial baseline and must not earn a leg bonus");
		assertTrue(Double.isNaN(tracker.estimatedHeading()));
	}

	@Test
	void oldManeuverGeometryAgesOutDespiteContinuousDetection() {
		var drive = trainedDrive();
		assertTrue(drive.tracker.solutionQuality() > 0.5, "Useful moving legs first earn a real solution");
		for (int index = 0; index < 6251; index++) {
			drive.step(false, 5, 0, false);
		}
		assertTrue(drive.tracker.solutionQuality() <= 0.1,
				"After more than 120 seconds stationary, lifetime geometry cannot keep range confidence high");
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"Motion heading is unavailable once current geometry falls below the quality gate");
	}

	@Test
	void loweringQualityHidesAnOtherwiseEstablishedHeading() {
		var drive = trainedDrive();
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()), "Moving target first has a motion solution");
		double quality = drive.tracker.solutionQuality();
		long lapse = (long) Math.ceil((quality - 0.49) / 0.01 * 50);
		drive.tracker.decay(drive.tick + lapse, 15);
		assertTrue(drive.tracker.solutionQuality() <= 0.5);
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"The heading accessor must respect the current quality gate, not historical availability");
	}

	@Test
	void stoppedTargetInvalidatesItsOldHeadingAndMotionConfidence() {
		var drive = trainedDrive();
		double qualityBeforeStop = drive.tracker.solutionQuality();
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()));
		for (int index = 0; index < 250; index++) {
			drive.step(true, 0, 0, false);
		}
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"A fresh five-second stationary displacement window invalidates the old course");
		assertTrue(drive.tracker.solutionQuality() < qualityBeforeStop,
				"A stopped target contradicts the previous constant-motion solution");
	}

	@Test
	void activeRangeFixDoesNotInventMotionForAStoppedTarget() {
		var drive = trainedDrive();
		for (int index = 0; index < 250; index++) {
			drive.step(true, 0, 0, false);
		}
		drive.tracker.updateFromPing(drive.tick, Math.hypot(drive.targetX - drive.ownX, drive.targetY));
		assertTrue(drive.tracker.solutionQuality() > 0.9, "An echo still gives a precise current range");
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"Range accuracy cannot manufacture a stopped target's direction of motion");
	}

	@Test
	void rightAngleTargetTurnInvalidatesThenRebuildsMotionFromNewEvidence() {
		var drive = trainedDrive();
		double qualityBeforeTurn = drive.tracker.solutionQuality();
		assertEquals(0, angleError(drive.tracker.estimatedHeading(), 0), 1e-8);
		for (int index = 0; index < 250; index++) {
			drive.step(true, 5, Math.PI / 2, false);
		}
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"A contradictory five-second displacement window cannot publish the blended old course");
		assertTrue(drive.tracker.solutionQuality() < qualityBeforeTurn,
				"The old maneuver solution loses maturity when target motion changes by ninety degrees");
		for (int index = 0; index < 6000; index++) {
			drive.step(true, 5, Math.PI / 2, false);
		}
		assertTrue(drive.tracker.solutionQuality() > 0.5,
				"Consistent new target motion and useful ownship legs eventually regain a solution");
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()));
		assertTrue(angleError(drive.tracker.estimatedHeading(), Math.PI / 2) < Math.toRadians(25),
				"The regained heading describes the new eastbound motion");
	}

	@Test
	void firstBearingAfterABlindTargetTurnCannotRecoverHeadingFromHiddenTransit() {
		var drive = trainedDrive();
		for (int index = 0; index < 600; index++) {
			drive.step(true, 5, Math.PI / 2, true);
		}
		drive.step(true, 5, Math.PI / 2, false);
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"The first fresh bearing is not a motion sample spanning the unobserved twelve-second turn");
		for (int index = 0; index < 6000; index++) {
			drive.step(true, 5, Math.PI / 2, false);
		}
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()), "Fresh observed movement restores heading");
		assertTrue(angleError(drive.tracker.estimatedHeading(), Math.PI / 2) < Math.toRadians(25));
	}

	@Test
	void sonarRetainsShortGapButReplacesExpiredSilentTrack() throws Exception {
		var fixture = new SonarFixture();
		fixture.own.activeSonarPing();
		var initial = fixture.measure(0).activeReturns().getFirst();
		var originalTracker = fixture.tracker();
		// An active contact reports echo RMS noise; the tracker anchors uncertainty to its noisy range.
		double initialTrackerUncertainty = originalTracker.rangeUncertainty();
		fixture.target.setSourceLevelDb(60);
		for (long tick = 1; tick <= 50; tick++) {
			assertTrue(fixture.measure(tick).passiveContacts().isEmpty());
		}
		assertSame(originalTracker, fixture.tracker());
		assertEquals(initial.solutionQuality() - 0.01, fixture.tracker().solutionQuality(), 1e-10);
		assertEquals(initialTrackerUncertainty + 15, fixture.tracker().rangeUncertainty(), 1e-8);
		fixture.target.setSourceLevelDb(125);
		assertEquals(1, fixture.measure(51).passiveContacts().size());
		assertSame(originalTracker, fixture.tracker(), "A short gap preserves the internal acoustic pair");
		fixture.target.setSourceLevelDb(60);
		for (long tick = 52; tick <= 1552; tick++) {
			assertTrue(fixture.measure(tick).passiveContacts().isEmpty());
		}
		assertNull(fixture.tracker(), "More than thirty seconds without audibility expires the pair");
		fixture.target.setSourceLevelDb(125);
		var fresh = fixture.measure(1553).passiveContacts().getFirst();
		assertNotSame(originalTracker, fixture.tracker());
		assertTrue(fresh.solutionQuality() <= 0.1, "A replacement track cannot inherit the old active fix");
		assertTrue(Double.isNaN(fresh.estimatedHeading()));
	}

	private static TrackerDrive trainedDrive() {
		var drive = new TrackerDrive();
		for (int index = 0; index < 12500; index++) {
			drive.step(true, 5, 0, false);
		}
		assertTrue(drive.tracker.solutionQuality() > 0.5, "Fixture must first establish useful moving geometry");
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()), "Fixture must first establish target motion");
		return drive;
	}

	private static void observe(
		ContactTracker tracker, Random rng, long tick, double ownX, double ownY, double ownHeading, boolean baffles,
		double targetX, double targetY) {
		double dx = targetX - ownX, dy = targetY - ownY;
		tracker.update(tick, Math.atan2(dx, dy), 30, ownX, ownY, ownHeading, baffles, Math.hypot(dx, dy), targetX,
				targetY, rng);
	}

	private static double angleError(double actual, double expected) {
		return Math.abs(Math.atan2(Math.sin(actual - expected), Math.cos(actual - expected)));
	}

	private static Random zeroNoise() {
		// Isolate lifecycle state from random error; real SonarFixture scenarios retain seeded noise.
		return new Random(42) {
			@Override
			public double nextGaussian() {
				return 0;
			}
		};
	}

	private static final class TrackerDrive {
		final ContactTracker tracker = new ContactTracker();
		final Random rng = zeroNoise();
		long tick;
		double ownX, ownHeading = Math.PI / 2, targetX, targetY = 1000;

		TrackerDrive() {
			observe(tracker, rng, 0, ownX, 0, ownHeading, false, targetX, targetY);
		}

		void step(boolean moveOwn, double targetSpeed, double targetHeading, boolean baffles) {
			tick++;
			if (moveOwn) {
				// Twelve m/s east/west ten-second legs, with a genuine translating baseline.
				ownHeading = ((tick - 1) / 500) % 2 == 0 ? Math.PI / 2 : 3 * Math.PI / 2;
				ownX += 0.24 * Math.sin(ownHeading);
			}
			targetX += targetSpeed * 0.02 * Math.sin(targetHeading);
			targetY += targetSpeed * 0.02 * Math.cos(targetHeading);
			observe(tracker, rng, tick, ownX, 0, ownHeading, baffles, targetX, targetY);
		}
	}

	private static final class SonarFixture {
		final GeneratedWorld world = GeneratedWorld.deepFlat();
		final SonarModel sonar = new SonarModel(0x4C494645L);
		final SubmarineEntity own = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, -160), 0,
				Color.GREEN, 1000);
		final SubmarineEntity target = new SubmarineEntity(VehicleConfig.submarine(), 1, null, new Vec3(0, 2000, -160),
				0, Color.RED, 1000);

		SonarFixture() {
			own.setSourceLevelDb(80);
			target.setSourceLevelDb(125);
		}

		SonarModel.SonarResult measure(long tick) {
			var result = sonar.computeContacts(tick, List.of(own, target), world.terrain(), List.of()).get(0);
			sonar.postTick(List.of(own, target));
			return result;
		}

		@SuppressWarnings("unchecked")
		ContactTracker tracker() throws Exception {
			// Engine pair identity is test-only; SonarContact has no public target identifier.
			var field = SonarModel.class.getDeclaredField("trackers");
			field.setAccessible(true);
			return ((Map<Long, ContactTracker>) field.get(sonar)).get(1L);
		}
	}

	@Test
	void stationaryPostTurnBearingsCannotRestoreTheOldManeuverSolution() {
		var drive = trainedDrive();
		double qualityBeforeTurn = drive.tracker.solutionQuality();
		for (int index = 0; index < 250; index++) {
			drive.step(false, 5, Math.PI / 2, false);
		}
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()), "First contradictory motion window invalidates");
		for (int index = 0; index < 1000; index++) {
			drive.step(false, 5, Math.PI / 2, false);
		}
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"Consistent target motion alone cannot repay the missing ownship geometry");
		assertTrue(drive.tracker.solutionQuality() < qualityBeforeTurn,
				"Stationary passive observations must not instantly restore pre-turn maturity");
		for (int index = 0; index < 6000; index++) {
			drive.step(true, 5, Math.PI / 2, false);
		}
		assertTrue(drive.tracker.solutionQuality() > 0.5);
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()), "New translating legs recover observability");
		assertTrue(angleError(drive.tracker.estimatedHeading(), Math.PI / 2) < Math.toRadians(25));
	}

	@Test
	void sonarContactStopsPublishingTheCourseWhenObservedTargetMotionStops() {
		var fixture = new SonarFixture();
		fixture.own.setSpeed(12);
		fixture.target.setSpeed(5);
		SonarContact latest = null;
		for (long tick = 0; tick <= 12500; tick++) {
			if (tick > 0) {
				double heading = ((tick - 1) / 500) % 2 == 0 ? Math.PI / 2 : 3 * Math.PI / 2;
				fixture.own.setHeading(heading);
				fixture.own.setX(fixture.own.x() + 0.24 * Math.sin(heading));
				fixture.target.setY(fixture.target.y() + 0.1);
			}
			latest = fixture.measure(tick).passiveContacts().getFirst();
		}
		assertTrue(latest.solutionQuality() > 0.5, "Real sonar first obtains a useful moving solution");
		assertTrue(Double.isFinite(latest.estimatedHeading()), "The moving source first has an observed course");
		double qualityBeforeStop = latest.solutionQuality();
		fixture.target.setSpeed(0);
		for (long tick = 12501; tick <= 12750; tick++) {
			double heading = ((tick - 1) / 500) % 2 == 0 ? Math.PI / 2 : 3 * Math.PI / 2;
			fixture.own.setHeading(heading);
			fixture.own.setX(fixture.own.x() + 0.24 * Math.sin(heading));
			latest = fixture.measure(tick).passiveContacts().getFirst();
		}
		assertTrue(Double.isNaN(latest.estimatedHeading()),
				"The public contact must hide a stale course after a stationary five-second motion window");
		assertTrue(latest.solutionQuality() < qualityBeforeStop,
				"Public passive confidence must reflect the changed target-motion model");
	}

	@Test
	void tinyCrossTrackStepCannotReviveAnAlmostExpiredManeuverPrior() {
		var drive = trainedDrive();
		double matureQuality = drive.tracker.solutionQuality();
		for (int index = 0; index < 5950; index++) {
			drive.step(false, 5, 0, false);
		}
		double coastedQuality = drive.tracker.solutionQuality();
		drive.step(true, 5, 0, false);
		double refreshedQuality = drive.tracker.solutionQuality();
		assertTrue(coastedQuality < 0.1, "119 seconds idle first ages the mature prior nearly to the bearing floor");
		assertTrue(refreshedQuality <= coastedQuality + 0.01,
				"One fresh 0.24 m spatial baseline cannot resurrect the old mature quality " + matureQuality);
	}

	@Test
	void departingStoppedTargetCannotReuseTheStationaryMotionPrior() {
		var drive = trainedDrive();
		for (int index = 0; index < 6250; index++) {
			drive.step(true, 0, 0, false);
		}
		double stationaryQuality = drive.tracker.solutionQuality();
		assertTrue(stationaryQuality > 0.5, "New ownship legs first resolve the stationary source");
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()), "Stationary source has no motion direction");
		for (int index = 0; index < 250; index++) {
			drive.step(false, 3, Math.PI / 2, false);
		}
		double departureQuality = drive.tracker.solutionQuality();
		assertTrue(departureQuality < 0.25,
				"A moving departure contradicts the stationary model rather than inheriting its mature geometry");
		for (int index = 0; index < 1000; index++) {
			drive.step(false, 3, Math.PI / 2, false);
		}
		assertTrue(Double.isNaN(drive.tracker.estimatedHeading()),
				"Consistent movement alone does not resolve the new passive motion model");
		for (int index = 0; index < 6000; index++) {
			drive.step(true, 3, Math.PI / 2, false);
		}
		assertTrue(drive.tracker.solutionQuality() > 0.5);
		assertTrue(Double.isFinite(drive.tracker.estimatedHeading()),
				"New translating legs regain the moving solution");
		assertTrue(angleError(drive.tracker.estimatedHeading(), Math.PI / 2) < Math.toRadians(25));
	}
}
