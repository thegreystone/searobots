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
package se.hirt.searobots.api;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SonarTrackTest {

	private static final double EPSILON = 1e-8;
	private static final Vec3 OBSERVER = new Vec3(100, 200, -50);

	@Test
	void lossPropagatesLastReportedMotionAndPreservesTheActualFix() {
		var track = new SonarTrack(15);
		var observation = contact(Math.PI / 2, 1000, true, 5, 10, 0.01, 0.9, Math.PI / 2, -50);
		var initial = track.update(OBSERVER, observation, 0).orElseThrow();
		var fix = track.lastFix().orElseThrow();
		assertEquals(1100, initial.x(), EPSILON);
		assertEquals(200, initial.y(), EPSILON);
		assertSame(observation, fix.observation());

		var coasted = track.update(OBSERVER, null, 10).orElseThrow();

		assertEquals(1150, coasted.x(), EPSILON);
		assertEquals(200, coasted.y(), EPSILON);
		assertEquals(Math.PI / 2, coasted.estimatedHeading(), EPSILON);
		assertEquals(5, coasted.estimatedSpeed(), EPSILON);
		assertEquals(10, coasted.secondsSinceContact(), EPSILON);
		assertEquals(10, coasted.secondsSinceFix(), EPSILON);
		assertEquals(fix, track.lastFix().orElseThrow(), "Coasting must not manufacture a new sensor fix");
		assertTrue(coasted.uncertainty().uncertaintyRadius() > initial.uncertainty().uncertaintyRadius());
	}

	@Test
	void coastingDependsOnElapsedTimeRatherThanControllerTickPartition() {
		var model = new SonarTrack.MotionModel(15, 0.2, 0.1, 1, 0.1);
		var oneStep = new SonarTrack(model);
		var manySteps = new SonarTrack(model);
		var observation = contact(0.4, 500, true, 7, 20, 0.02, 0.8, 1.2, -50);
		oneStep.update(OBSERVER, observation, 0);
		manySteps.update(OBSERVER, observation, 0);
		var expected = oneStep.update(OBSERVER, null, 30).orElseThrow();
		for (int step = 0; step < 300; step++) {
			manySteps.update(OBSERVER, null, 0.1);
		}
		var actual = manySteps.estimate().orElseThrow();
		assertEquals(expected.x(), actual.x(), EPSILON);
		assertEquals(expected.y(), actual.y(), EPSILON);
		assertEquals(expected.secondsSinceFix(), actual.secondsSinceFix(), EPSILON);
		assertEquals(expected.solutionQuality(), actual.solutionQuality(), EPSILON);
		assertEquals(expected.uncertainty().majorRadius(), actual.uncertainty().majorRadius(), EPSILON);
		assertEquals(expected.uncertainty().minorRadius(), actual.uncertainty().minorRadius(), EPSILON);
	}

	@Test
	void bearingOnlyOrUnknownRangeCannotCreateACartesianFix() {
		for (double range : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
			var track = new SonarTrack(15);
			assertTrue(track.update(OBSERVER, contact(0, range, false, 5, 100, 0.02, 0.5, 0, Double.NaN), 1).isEmpty());
			assertTrue(track.lastFix().isEmpty());
		}
		var track = new SonarTrack(15);
		assertTrue(track.update(OBSERVER, contact(0, 1000, false, 5, Double.MAX_VALUE, 0.02, 0.5, 0, Double.NaN), 1)
				.isEmpty());
		assertTrue(track.estimate().isEmpty());
	}

	@Test
	void bearingOnlyReportsAgeTheLastFixWithoutReplacingItsPosition() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.02, 0.8, 0, -50), 0);
		var fix = track.lastFix().orElseThrow();

		var estimate = track
				.update(OBSERVER, contact(0.2, 0, false, -1, Double.MAX_VALUE, 0.1, 0, Double.NaN, Double.NaN), 5)
				.orElseThrow();

		assertEquals(fix, track.lastFix().orElseThrow());
		assertEquals(5, estimate.secondsSinceFix(), EPSILON);
		assertEquals(0, estimate.secondsSinceContact(), EPSILON,
				"Hearing a bearing is evidence of contact continuity, but supplies no new range fix");
		assertTrue(estimate.uncertainty().uncertaintyRadius() > 20);
	}

	@Test
	void repeatedCorrelatedPassiveReportsDoNotAverageAwayTheirUncertainty() {
		var track = new SonarTrack(15);
		var observation = contact(0.3, 1000, false, 5, 300, 0.03, 0.4, Double.NaN, Double.NaN);
		var initial = track.update(OBSERVER, observation, 0).orElseThrow();
		for (int step = 0; step < 500; step++) {
			track.update(OBSERVER, observation, 0.02);
		}
		var repeated = track.estimate().orElseThrow();
		assertEquals(initial.x(), repeated.x(), EPSILON);
		assertEquals(initial.y(), repeated.y(), EPSILON);
		assertEquals(initial.uncertainty().majorRadius(), repeated.uncertainty().majorRadius(), EPSILON);
		assertEquals(initial.uncertainty().minorRadius(), repeated.uncertainty().minorRadius(), EPSILON);
		assertEquals(0.4, repeated.solutionQuality(), EPSILON);
	}

	@Test
	void aNewPoorSolutionCanReduceQualityAndIncreaseItsReportedUncertainty() {
		var track = new SonarTrack(15);
		var accurate = track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.95, 0, -50), 0).orElseThrow();
		var poor = track.update(OBSERVER, contact(0, 1000, false, 5, 500, 0.1, 0.2, Double.NaN, Double.NaN), 1)
				.orElseThrow();
		assertEquals(0.2, poor.solutionQuality(), EPSILON);
		assertTrue(poor.uncertainty().uncertaintyRadius() > accurate.uncertainty().uncertaintyRadius());
		assertTrue(poor.uncertainty().majorRadius() >= 500);
	}

	@Test
	void activePositionUncertaintyIncludesTheBearingError() {
		var track = new SonarTrack(15);
		var estimate = track.update(OBSERVER, contact(0, 2000, true, -1, 10, 0.2, 0.95, Double.NaN, -50), 0)
				.orElseThrow();
		assertTrue(estimate.uncertainty().majorRadius() >= 2000 * Math.sin(0.2),
				"A precise active range does not remove hundreds of metres of cross-bearing uncertainty");
		assertTrue(estimate.uncertainty().minorRadius() >= 10);
	}

	@Test
	void activeDepthProjectsSlantRangeOntoTheHorizontalPlane() {
		var track = new SonarTrack(15);
		var estimate = track.update(OBSERVER, contact(0, 500, true, -1, 10, 0.01, 0.95, Double.NaN, -350), 0)
				.orElseThrow();
		assertEquals(100, estimate.x(), EPSILON);
		assertEquals(600, estimate.y(), EPSILON,
				"A 500 m slant range and 300 m depth difference imply 400 m horizontally");
	}

	@Test
	void unknownDepthRetainsAPlanarMeanButBoundsItsVerticalAmbiguity() {
		var track = new SonarTrack(15);
		var estimate = track.update(OBSERVER, contact(0, 500, false, -1, 10, 0.01, 0.5, Double.NaN, Double.NaN), 0)
				.orElseThrow();
		assertEquals(100, estimate.x(), EPSILON);
		assertEquals(700, estimate.y(), EPSILON);
		assertTrue(estimate.uncertainty().uncertaintyRadius() >= 500,
				"Unknown vertical separation can make the horizontal range anywhere from zero to the reported slant range");
	}

	@Test
	void initialEllipseEnclosesTheFullTwoErrorBearingAndRangeSectorWithUnknownDepth() {
		var track = new SonarTrack(15);
		var observation = contact(1.1, 1000, false, -1, 100, 0.08, 0.5, Double.NaN, Double.NaN);
		var estimate = track.update(OBSERVER, observation, 0).orElseThrow();
		for (int radial = 0; radial <= 40; radial++) {
			double horizontal = radial * 1200.0 / 40;
			for (int angle = 0; angle <= 32; angle++) {
				double bearing = observation.bearing() - 0.16 + angle * 0.32 / 32;
				assertEnclosed(estimate, OBSERVER.x() + horizontal * Math.sin(bearing),
						OBSERVER.y() + horizontal * Math.cos(bearing));
			}
		}
	}

	@Test
	void initialEllipseEnclosesKnownDepthRangeAndBearingErrorCombinations() {
		var track = new SonarTrack(15);
		var observation = contact(1.1, 500, true, -1, 25, 0.08, 0.9, Double.NaN, -350);
		var estimate = track.update(OBSERVER, observation, 0).orElseThrow();
		for (int radial = 0; radial <= 16; radial++) {
			double slantRange = 450 + radial * 100.0 / 16;
			for (int depth = 0; depth <= 16; depth++) {
				double separation = 250 + depth * 100.0 / 16;
				double horizontal = Math.sqrt(slantRange * slantRange - separation * separation);
				for (int angle = 0; angle <= 32; angle++) {
					double bearing = observation.bearing() - 0.16 + angle * 0.32 / 32;
					assertEnclosed(estimate, OBSERVER.x() + horizontal * Math.sin(bearing),
							OBSERVER.y() + horizontal * Math.cos(bearing));
				}
			}
		}
	}

	@Test
	void coastingEnvelopeEnclosesBoundedTurningAndAcceleratingPathsFromTheInitialPositionEnvelope() {
		var model = new SonarTrack.MotionModel(15, 0.3, 0.06, 1, 0.1);
		var observation = contact(0.3, 1000, true, 5, 20, 0.01, 0.9, 1.2, -50);
		for (double elapsed : new double[] {1, 10, 60}) {
			var track = new SonarTrack(model);
			var initial = track.update(OBSERVER, observation, 0).orElseThrow();
			var coasted = track.update(OBSERVER, null, elapsed).orElseThrow();
			for (int initialSpeed = -1; initialSpeed <= 1; initialSpeed++) {
				for (int initialHeading = -1; initialHeading <= 1; initialHeading++) {
					for (int acceleration = -1; acceleration <= 1; acceleration++) {
						for (int turn = -2; turn <= 2; turn++) {
							var displacement = boundedPath(5 + initialSpeed * model.speedUncertainty(),
									1.2 + initialHeading * model.headingUncertainty(),
									acceleration * model.maxAcceleration(), turn * model.maxTurnRate() / 2,
									model.maxSpeed(), elapsed);
							for (int materialPoint = 0; materialPoint < 16; materialPoint++) {
								var offset = ellipsePoint(initial.uncertainty(), materialPoint * 2 * Math.PI / 16);
								assertEnclosed(coasted, initial.x() + offset.x() + displacement.x(),
										initial.y() + offset.y() + displacement.y());
							}
						}
					}
				}
			}
		}
	}

	@Test
	void longUnresolvedCoastsExpandAnEllipseTowardACircle() {
		var track = new SonarTrack(15);
		var initial = track.update(OBSERVER, contact(0, 1000, true, -1, 100, 0.05, 0.9, Double.NaN, -50), 0)
				.orElseThrow();
		assertTrue(initial.uncertainty().majorRadius() > 1.5 * initial.uncertainty().minorRadius(),
				"The initial fixture must have distinct radial and cross-bearing axes");
		var coasted = track.update(OBSERVER, null, 1000).orElseThrow();
		assertTrue(coasted.uncertainty().majorRadius() / coasted.uncertainty().minorRadius() < 1.01);
	}

	@Test
	void aNewUnresolvedHeadingInvalidatesPreviouslyReportedMotion() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.9, Math.PI / 2, -50), 0);
		var unresolved = track.update(OBSERVER, contact(0, 1200, true, 5, 20, 0.01, 0.9, Double.NaN, -50), 1)
				.orElseThrow();
		var coasted = track.update(OBSERVER, null, 10).orElseThrow();
		assertTrue(Double.isNaN(coasted.estimatedHeading()));
		assertEquals(unresolved.x(), coasted.x(), EPSILON);
		assertEquals(unresolved.y(), coasted.y(), EPSILON);
		assertTrue(coasted.uncertainty().majorRadius() >= unresolved.uncertainty().majorRadius() + 150 - EPSILON);
		assertTrue(coasted.uncertainty().minorRadius() >= unresolved.uncertainty().minorRadius() + 150 - EPSILON);
	}

	@Test
	void aNewUnknownSpeedInvalidatesPreviouslyReportedMotion() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.9, Math.PI / 2, -50), 0);
		var unresolved = track.update(OBSERVER, contact(0, 1200, true, -1, 20, 0.01, 0.9, Math.PI / 2, -50), 1)
				.orElseThrow();
		var coasted = track.update(OBSERVER, null, 10).orElseThrow();
		assertEquals(unresolved.x(), coasted.x(), EPSILON);
		assertEquals(unresolved.y(), coasted.y(), EPSILON);
		assertTrue(coasted.estimatedSpeed() < 0);
	}

	@Test
	void tighterMotionLimitsProduceANarrowerCoastingModel() {
		var tight = new SonarTrack(new SonarTrack.MotionModel(15, 0.1, 0.01, 0.1, 0.01));
		var loose = new SonarTrack(new SonarTrack.MotionModel(15, 2, 0.3, 3, 0.3));
		var observation = contact(0, 1000, true, 5, 20, 0.01, 0.9, Math.PI / 2, -50);
		tight.update(OBSERVER, observation, 0);
		loose.update(OBSERVER, observation, 0);
		var tightCoast = tight.update(OBSERVER, null, 10).orElseThrow();
		var looseCoast = loose.update(OBSERVER, null, 10).orElseThrow();
		assertEquals(tightCoast.x(), looseCoast.x(), EPSILON);
		assertEquals(tightCoast.y(), looseCoast.y(), EPSILON);
		assertTrue(tightCoast.uncertainty().uncertaintyRadius() < looseCoast.uncertainty().uncertaintyRadius());
	}

	@Test
	void contactEstimatePublicationPreservesTrackValuesAndCallerAliveBelief() {
		var track = new SonarTrack(15);
		var estimate = track.update(OBSERVER, contact(0.4, 1000, true, 5, 20, 0.01, 0.9, 0.2, -50), 0).orElseThrow();
		var published = estimate.toContactEstimate(0.7, "coasting");
		assertEquals(estimate.x(), published.x(), EPSILON);
		assertEquals(estimate.y(), published.y(), EPSILON);
		assertEquals(estimate.estimatedHeading(), published.estimatedHeading(), EPSILON);
		assertEquals(estimate.estimatedSpeed(), published.estimatedSpeed(), EPSILON);
		assertEquals(estimate.uncertainty().uncertaintyRadius(), published.uncertaintyRadius(), EPSILON);
		assertEquals(0.7, published.contactAlive(), EPSILON);
		assertEquals("coasting", published.label());
	}

	@Test
	void resetDiscardsTheFixPredictionAndClock() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.9, 0, -50), 3);
		track.update(OBSERVER, null, 10);
		track.reset();
		assertTrue(track.estimate().isEmpty());
		assertTrue(track.lastFix().isEmpty());
		track.update(OBSERVER, contact(0, 500, true, 5, 20, 0.01, 0.9, 0, -50), 0);
		assertEquals(0, track.lastFix().orElseThrow().timeSeconds(), EPSILON);
	}

	@Test
	void invalidMotionLimitsAndElapsedTimeAreRejectedWithoutMutatingTheTrack() {
		assertThrows(IllegalArgumentException.class, () -> new SonarTrack(-1));
		assertThrows(IllegalArgumentException.class, () -> new SonarTrack(Double.NaN));
		assertThrows(IllegalArgumentException.class, () -> new SonarTrack.MotionModel(15, -1, 0.1, 1, 0.1));
		assertThrows(IllegalArgumentException.class, () -> new SonarTrack.MotionModel(15, 1, -0.1, 1, 0.1));
		assertThrows(IllegalArgumentException.class, () -> new SonarTrack.MotionModel(15, 1, 0.1, -1, 0.1));
		assertThrows(IllegalArgumentException.class, () -> new SonarTrack.MotionModel(15, 1, 0.1, 1, -0.1));
		var track = new SonarTrack(15);
		var observation = contact(0, 500, true, 5, 20, 0.01, 0.9, 0, -50);
		var estimate = track.update(OBSERVER, observation, 0).orElseThrow();
		for (double invalidTime : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY}) {
			assertThrows(IllegalArgumentException.class, () -> track.update(OBSERVER, null, invalidTime));
		}
		assertEquals(estimate, track.estimate().orElseThrow());
		assertFalse(track.lastFix().isEmpty());
	}

	@Test
	void finiteLargeEnvelopeExpandsWithoutLosingItsReachableMotionRadius() {
		var track = new SonarTrack(1e307);
		track.update(Vec3.ZERO, contact(0, 1.4e308, false, -1, 0, 0, 0.5, Double.NaN, Double.NaN), 0);
		var estimate = track.update(Vec3.ZERO, null, 1).orElseThrow();
		assertTrue(Double.isFinite(estimate.uncertainty().majorRadius()));
		assertTrue(Double.isFinite(estimate.uncertainty().minorRadius()));
		assertTrue(estimate.uncertainty().minorRadius() >= 1.5e308,
				"A target initially directly below the observer could then move 1e307 m south of it");
	}

	@Test
	void finiteElapsedTimeThatWouldOverflowPredictionIsRejectedWithoutMutation() {
		var track = new SonarTrack(new SonarTrack.MotionModel(100, 0, 0, 0, 0));
		var initial = track.update(OBSERVER, contact(0, 500, true, 100, 20, 0.01, 0.9, 0, -50), 0).orElseThrow();
		var fix = track.lastFix().orElseThrow();

		assertThrows(IllegalArgumentException.class, () -> track.update(OBSERVER, null, Double.MAX_VALUE));

		assertEquals(initial, track.estimate().orElseThrow());
		assertEquals(fix, track.lastFix().orElseThrow());
	}

	@Test
	void veryLargeFiniteMeasurementAndMotionUncertaintiesProduceFiniteAxes() {
		var measurementTrack = new SonarTrack(15);
		measurementTrack.update(OBSERVER,
				contact(0, 500, true, -1, Double.MAX_VALUE / 2, Double.MAX_VALUE / 2, 0.9, Double.NaN, -50), 0);
		assertFiniteEstimate(measurementTrack.update(OBSERVER, null, 1).orElseThrow());

		var motionTrack = new SonarTrack(new SonarTrack.MotionModel(Double.MAX_VALUE, 1, 0, Double.MAX_VALUE, 0));
		motionTrack.update(OBSERVER, contact(0, 500, true, 100, 20, 0.01, 0.9, 0, -50), 0);
		assertFiniteEstimate(motionTrack.update(OBSERVER, null, 1).orElseThrow());
	}

	@Test
	void contradictoryDepthAndRangeCannotManufactureAPreciseZeroHorizontalRange() {
		var track = new SonarTrack(15);
		var estimate = track.update(OBSERVER, contact(0, 100, true, -1, 5, 0.01, 0.9, Double.NaN, -1050), 0)
				.orElseThrow();
		assertEquals(100, estimate.x(), EPSILON);
		assertEquals(300, estimate.y(), EPSILON,
				"Inconsistent depth evidence must fall back to an unknown-depth projection");
		assertTrue(estimate.uncertainty().uncertaintyRadius() >= 100);
	}

	@Test
	void invalidObserverCoordinatesAreRejectedWithoutAdvancingTheClockOrChangingEvidence() {
		var track = new SonarTrack(15);
		var initial = track.update(OBSERVER, contact(0, 500, true, 5, 20, 0.01, 0.9, 0, -50), 0).orElseThrow();
		var fix = track.lastFix().orElseThrow();
		for (var observer : new Vec3[] {new Vec3(Double.NaN, 0, 0), new Vec3(0, Double.POSITIVE_INFINITY, 0),
				new Vec3(0, 0, Double.NEGATIVE_INFINITY)}) {
			assertThrows(IllegalArgumentException.class, () -> track.update(observer, null, 10));
		}
		assertThrows(NullPointerException.class, () -> track.update(null, null, 10));
		assertEquals(initial, track.estimate().orElseThrow());
		assertEquals(fix, track.lastFix().orElseThrow());
	}

	@Test
	void futurePredictionAdvancesItsMeanAgesAndEnvelopeWithoutChangingTheLiveTrack() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.9, Math.PI / 2, -50), 2);
		var current = track.update(OBSERVER, null, 3).orElseThrow();
		var fix = track.lastFix().orElseThrow();

		var future = track.predict(7).orElseThrow();

		assertEquals(150, future.x(), EPSILON);
		assertEquals(1200, future.y(), EPSILON);
		assertEquals(10, future.secondsSinceContact(), EPSILON);
		assertEquals(10, future.secondsSinceFix(), EPSILON);
		assertTrue(future.uncertainty().majorRadius() > current.uncertainty().majorRadius());
		assertTrue(future.uncertainty().minorRadius() > current.uncertainty().minorRadius());
		assertTrue(future.solutionQuality() < current.solutionQuality());
		assertEquals(current, track.estimate().orElseThrow());
		assertEquals(current, track.predict(0).orElseThrow());
		assertSame(fix, track.lastFix().orElseThrow(), "A forecast must preserve the actual evidence fix");

		var next = track.update(OBSERVER, null, 1).orElseThrow();
		assertEquals(120, next.x(), EPSILON, "Only actual elapsed time may advance the live mean");
		assertEquals(4, next.secondsSinceFix(), EPSILON);
	}

	@Test
	void invalidOrOverflowingForecastsDoNotMutateCurrentEvidenceOrTime() {
		var track = new SonarTrack(new SonarTrack.MotionModel(100, 0, 0, 0, 0));
		track.observePosition(new Vec2(100, 200), new SonarTrack.Ellipse(60, 30, 0), 0, 100, 0.8);
		var current = track.update(OBSERVER, null, 1).orElseThrow();
		var fix = track.lastFix().orElseThrow();
		for (double horizon : new double[] {-1, Double.NaN, Double.POSITIVE_INFINITY, Double.MAX_VALUE}) {
			assertThrows(IllegalArgumentException.class, () -> track.predict(horizon));
			assertEquals(current, track.estimate().orElseThrow());
			assertSame(fix, track.lastFix().orElseThrow());
		}
		var next = track.update(OBSERVER, null, 1).orElseThrow();
		assertEquals(400, next.y(), EPSILON);
		assertEquals(2, next.secondsSinceFix(), EPSILON);

		var stationary = new SonarTrack(15);
		stationary.update(OBSERVER, null, Double.MAX_VALUE);
		var lateFix = stationary.observePosition(Vec2.ZERO, new SonarTrack.Ellipse(1, 1, 0), 0, 0, 0.5).orElseThrow();
		assertThrows(IllegalArgumentException.class, () -> stationary.predict(Double.MAX_VALUE),
				"Forecast time itself must remain finite even when the nominal target is stationary");
		assertEquals(lateFix, stationary.estimate().orElseThrow());
	}

	@Test
	void explicitWorldPositionEvidenceCreatesAnInspectableFixWithoutFabricatingSonar() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.9, 0, -50), 2);
		track.update(OBSERVER, null, 6);
		var estimate = track.observePosition(new Vec2(300, 400), new SonarTrack.Ellipse(60, 25, 2 * Math.PI + 0.3),
				Math.PI / 2, 7, 0.75).orElseThrow();
		var fix = track.lastFix().orElseThrow();

		assertEquals(300, estimate.x(), EPSILON);
		assertEquals(400, estimate.y(), EPSILON);
		assertEquals(0, estimate.secondsSinceContact(), EPSILON);
		assertEquals(0, estimate.secondsSinceFix(), EPSILON);
		assertEquals(0.75, estimate.solutionQuality(), EPSILON);
		assertEquals(60, estimate.uncertainty().majorRadius(), EPSILON);
		assertEquals(25, estimate.uncertainty().minorRadius(), EPSILON);
		assertEquals(0.3, estimate.uncertainty().heading(), EPSILON);
		assertEquals(8, fix.timeSeconds(), EPSILON);
		assertEquals(300, fix.x(), EPSILON);
		assertEquals(400, fix.y(), EPSILON);
		assertNull(fix.observation(), "Controller evidence must not masquerade as an engine sonar report");

		var coasted = track.update(OBSERVER, null, 2).orElseThrow();
		assertEquals(314, coasted.x(), EPSILON);
		assertEquals(400, coasted.y(), EPSILON);
		assertEquals(2, coasted.secondsSinceFix(), EPSILON);
		assertSame(fix, track.lastFix().orElseThrow());
	}

	@Test
	void invalidExplicitPositionEnvelopesAndQualityAreRejectedWithoutReplacingEvidence() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 5, 20, 0.01, 0.9, 0, -50), 2);
		var current = track.update(OBSERVER, null, 3).orElseThrow();
		var fix = track.lastFix().orElseThrow();
		var valid = new SonarTrack.Ellipse(60, 30, 0);
		for (var position : new Vec2[] {new Vec2(Double.NaN, 0), new Vec2(0, Double.POSITIVE_INFINITY)}) {
			assertThrows(IllegalArgumentException.class, () -> track.observePosition(position, valid, 0, 5, 0.8));
		}
		for (var ellipse : new SonarTrack.Ellipse[] {new SonarTrack.Ellipse(-1, 0, 0),
				new SonarTrack.Ellipse(60, -1, 0), new SonarTrack.Ellipse(Double.NaN, 30, 0),
				new SonarTrack.Ellipse(60, Double.NaN, 0), new SonarTrack.Ellipse(Double.POSITIVE_INFINITY, 30, 0),
				new SonarTrack.Ellipse(60, Double.POSITIVE_INFINITY, 0), new SonarTrack.Ellipse(20, 30, 0),
				new SonarTrack.Ellipse(60, 30, Double.NaN), new SonarTrack.Ellipse(60, 30, Double.POSITIVE_INFINITY)}) {
			assertThrows(IllegalArgumentException.class, () -> track.observePosition(Vec2.ZERO, ellipse, 0, 5, 0.8));
		}
		for (double quality : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
			assertThrows(IllegalArgumentException.class, () -> track.observePosition(Vec2.ZERO, valid, 0, 5, quality));
		}
		assertThrows(NullPointerException.class, () -> track.observePosition(null, valid, 0, 5, 0.8));
		assertThrows(NullPointerException.class, () -> track.observePosition(Vec2.ZERO, null, 0, 5, 0.8));
		assertEquals(current, track.estimate().orElseThrow());
		assertSame(fix, track.lastFix().orElseThrow());
	}

	@Test
	void explicitPositionQualityIsClampedAndInvalidMotionBecomesUnresolved() {
		var track = new SonarTrack(15);
		var low = track.observePosition(Vec2.ZERO, new SonarTrack.Ellipse(60, 30, 0), Double.NaN, -1, -0.5)
				.orElseThrow();
		assertEquals(0, low.solutionQuality(), EPSILON);
		assertTrue(Double.isNaN(low.estimatedHeading()));
		assertEquals(-1, low.estimatedSpeed(), EPSILON);
		var high = track.observePosition(Vec2.ZERO, new SonarTrack.Ellipse(60, 30, 0), Double.POSITIVE_INFINITY,
				Double.POSITIVE_INFINITY, 1.5).orElseThrow();
		assertEquals(1, high.solutionQuality(), EPSILON);
		assertTrue(Double.isNaN(high.estimatedHeading()));
		assertEquals(-1, high.estimatedSpeed(), EPSILON);
	}

	@Test
	void motionOnlyEvidenceKeepsTheOldFixAndGrownEnvelopeWhileInvalidatingUnknownMotion() {
		var track = new SonarTrack(15);
		track.update(OBSERVER, contact(0, 1000, true, 8, 20, 0.01, 0.9, Math.PI / 2, -50), 2);
		var grown = track.update(OBSERVER, null, 5).orElseThrow();
		var fix = track.lastFix().orElseThrow();

		var unresolved = track.observeMotion(Double.NaN, -1).orElseThrow();

		assertEquals(grown.x(), unresolved.x(), EPSILON);
		assertEquals(grown.y(), unresolved.y(), EPSILON);
		assertEquals(grown.uncertainty(), unresolved.uncertainty());
		assertEquals(grown.solutionQuality(), unresolved.solutionQuality(), EPSILON);
		assertEquals(0, unresolved.secondsSinceContact(), EPSILON);
		assertEquals(5, unresolved.secondsSinceFix(), EPSILON);
		assertSame(fix, track.lastFix().orElseThrow());
		assertTrue(Double.isNaN(unresolved.estimatedHeading()));
		assertEquals(-1, unresolved.estimatedSpeed(), EPSILON);

		var coasted = track.update(OBSERVER, null, 2).orElseThrow();
		assertEquals(unresolved.x(), coasted.x(), EPSILON);
		assertEquals(unresolved.y(), coasted.y(), EPSILON);
		assertEquals(7, coasted.secondsSinceFix(), EPSILON);
		assertEquals(2, coasted.secondsSinceContact(), EPSILON);
		assertTrue(coasted.uncertainty().majorRadius() >= grown.uncertainty().majorRadius() + 30 - EPSILON);
		assertTrue(coasted.uncertainty().minorRadius() >= grown.uncertainty().minorRadius() + 30 - EPSILON);
	}

	@Test
	void freshMotionOnlyEvidenceChangesFutureDirectionWithoutMovingTheCurrentMean() {
		var track = new SonarTrack(15);
		track.observePosition(new Vec2(300, 400), new SonarTrack.Ellipse(60, 30, 0), Math.PI / 2, 8, 0.8);
		var grown = track.update(OBSERVER, null, 5).orElseThrow();
		var fix = track.lastFix().orElseThrow();
		var fresh = track.observeMotion(0, 7).orElseThrow();

		assertEquals(grown.x(), fresh.x(), EPSILON);
		assertEquals(grown.y(), fresh.y(), EPSILON);
		assertEquals(grown.uncertainty(), fresh.uncertainty());
		var future = track.predict(3).orElseThrow();
		assertEquals(fresh.x(), future.x(), EPSILON);
		assertEquals(fresh.y() + 21, future.y(), EPSILON);
		assertEquals(8, future.secondsSinceFix(), EPSILON);
		assertSame(fix, track.lastFix().orElseThrow());
		assertEquals(fresh, track.estimate().orElseThrow());
	}

	@Test
	void repeatedMotionReportsDoNotRestorePositionConfidenceOrResetTheFixAge() {
		var track = new SonarTrack(new SonarTrack.MotionModel(15, 0, 0, 1, 0.1));
		track.observePosition(new Vec2(300, 400), new SonarTrack.Ellipse(80, 20, 0.5), 0, 5, 0.9);
		var grown = track.update(OBSERVER, null, 3).orElseThrow();
		var fix = track.lastFix().orElseThrow();
		var refreshed = track.observeMotion(0, 5).orElseThrow();
		for (int report = 0; report < 100; report++) {
			assertEquals(refreshed, track.observeMotion(0, 5).orElseThrow(),
					"Repeated motion at the same instant supplies no new position precision");
		}
		var previous = refreshed;
		for (int report = 0; report < 20; report++) {
			track.update(OBSERVER, null, 0.1);
			var next = track.observeMotion(0, 5).orElseThrow();
			assertTrue(next.uncertainty().majorRadius() >= previous.uncertainty().majorRadius());
			assertTrue(next.uncertainty().minorRadius() >= previous.uncertainty().minorRadius());
			assertTrue(next.solutionQuality() <= previous.solutionQuality());
			previous = next;
		}
		assertEquals(grown.x(), previous.x(), EPSILON);
		assertEquals(grown.y() + 10, previous.y(), EPSILON);
		assertEquals(5, previous.secondsSinceFix(), EPSILON);
		assertSame(fix, track.lastFix().orElseThrow());
	}

	@Test
	void forecastingAndMotionReportsStayEmptyUntilAnActualPositionFixArrives() {
		var track = new SonarTrack(15);
		assertTrue(track.predict(0).isEmpty());
		assertTrue(track.predict(10).isEmpty());
		assertTrue(track.observeMotion(0, 5).isEmpty());
		assertTrue(track.lastFix().isEmpty());
		track.update(OBSERVER, null, 5);
		assertTrue(track.observeMotion(Double.NaN, -1).isEmpty());
		assertTrue(track.predict(100).isEmpty());
		assertThrows(IllegalArgumentException.class, () -> track.predict(-1));
		assertThrows(IllegalArgumentException.class, () -> track.predict(Double.NaN));
		assertThrows(IllegalArgumentException.class, () -> track.predict(Double.POSITIVE_INFINITY));
		track.observePosition(Vec2.ZERO, new SonarTrack.Ellipse(0, 0, 0), 0, 0, 0.8);
		assertEquals(5, track.lastFix().orElseThrow().timeSeconds(), EPSILON,
				"A forecast or empty motion report must not advance the real clock");
	}

	private static void assertFiniteEstimate(SonarTrack.Estimate estimate) {
		assertTrue(Double.isFinite(estimate.x()));
		assertTrue(Double.isFinite(estimate.y()));
		assertTrue(Double.isFinite(estimate.solutionQuality()));
		assertTrue(Double.isFinite(estimate.uncertainty().majorRadius()));
		assertTrue(Double.isFinite(estimate.uncertainty().minorRadius()));
		assertTrue(estimate.uncertainty().majorRadius() <= Double.MAX_VALUE);
		assertTrue(estimate.uncertainty().minorRadius() <= Double.MAX_VALUE);
	}

	private static Vec2 boundedPath(
		double initialSpeed, double initialHeading, double acceleration, double turnRate, double maxSpeed,
		double elapsed) {
		int steps = 3000;
		double dt = elapsed / steps;
		double x = 0, y = 0;
		// Integrate a target trajectory from configured physical bounds, independently of
		// the helper's analytic velocity-error and ellipse expansion formulas.
		for (int step = 0; step < steps; step++) {
			double time = (step + 0.5) * dt;
			double speed = Math.clamp(initialSpeed + acceleration * time, 0, maxSpeed);
			double heading = initialHeading + turnRate * time;
			x += speed * Math.sin(heading) * dt;
			y += speed * Math.cos(heading) * dt;
		}
		return new Vec2(x, y);
	}

	private static Vec2 ellipsePoint(SonarTrack.Ellipse ellipse, double angle) {
		double along = ellipse.majorRadius() * Math.cos(angle);
		double across = ellipse.minorRadius() * Math.sin(angle);
		return new Vec2(along * Math.sin(ellipse.heading()) + across * Math.cos(ellipse.heading()),
				along * Math.cos(ellipse.heading()) - across * Math.sin(ellipse.heading()));
	}

	private static void assertEnclosed(SonarTrack.Estimate estimate, double x, double y) {
		double dx = x - estimate.x(), dy = y - estimate.y();
		var ellipse = estimate.uncertainty();
		double along = (dx * Math.sin(ellipse.heading()) + dy * Math.cos(ellipse.heading())) / ellipse.majorRadius();
		double across = (dx * Math.cos(ellipse.heading()) - dy * Math.sin(ellipse.heading())) / ellipse.minorRadius();
		assertTrue(along * along + across * across <= 1 + 1e-8,
				"Sampled allowed position must lie inside the track's uncertainty ellipse");
	}

	private static SonarContact contact(
		double bearing, double range, boolean active, double speed, double rangeUncertainty, double bearingUncertainty,
		double quality, double heading, double depth) {
		return new SonarContact(bearing, 10, range, active, speed, bearingUncertainty, rangeUncertainty, Double.NaN,
				quality, heading, depth);
	}
}
