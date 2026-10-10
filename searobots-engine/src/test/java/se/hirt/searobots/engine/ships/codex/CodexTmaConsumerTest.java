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
package se.hirt.searobots.engine.ships.codex;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.TestHelpers;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CodexTmaConsumerTest {

	@Test
	void passiveRangeUncertaintyBoundsThePublishedEstimate() {
		var fixture = new Fixture();
		var contact = passive(0.0, 2_000.0, 12.0, 0.65, 1_200.0, 0.0);
		var output = fixture.tick(0L, List.of(contact), List.of(), 0);

		assertFalse(output.contactEstimates.isEmpty(), "A finite passive range bound must retain a usable track.");
		assertTrue(output.contactEstimates.getLast().uncertaintyRadius() >= contact.rangeUncertainty(),
				"A new passive track has no independent range fix that could justify a smaller uncertainty.");
	}

	@Test
	void highQualityTrackLeavesChaseAfterTwentySecondsOfSilenceButStillCoasts() {
		var fixture = new Fixture();
		var first = fixture.tick(0L, List.of(passive(0.0, 2_000.0, 12.0, 0.85, 300.0, 0.0)), List.of(), 0);
		assertTrue(first.status.startsWith("C"), "Fresh, high-quality passive evidence should permit pursuit.");
		TestHelpers.CapturedOutput output = first;
		for (long tick = 1L; tick <= 1_000L; tick++) {
			output = fixture.tick(tick, List.of(), List.of(), 0);
		}

		assertFalse(output.status.startsWith("C"),
				"Old TMA quality must not keep tactical pursuit alive indefinitely.");
		assertFalse(output.contactEstimates.isEmpty(), "Losing pursuit confidence should preserve a coasted estimate.");
		var estimate = output.contactEstimates.getLast();
		assertTrue(estimate.contactAlive() > 0.0 && estimate.contactAlive() < 0.35,
				"The scenario must exercise declining contact belief, rather than a discarded or refreshed track.");
		assertTrue(estimate.uncertaintyRadius() > first.contactEstimates.getLast().uncertaintyRadius(),
				"The coasted position bound should grow during silence.");
	}

	@Test
	void freshUnresolvedHeadingDoesNotPublishThePreviousResolvedHeading() {
		var fixture = new Fixture();
		var first = fixture.tick(0L, List.of(), List.of(active(0.0, 2_000.0, Math.PI / 2.0)), 0);
		assertEquals(Math.PI / 2.0, first.contactEstimates.getLast().estimatedHeading(), 1.0e-9);
		var output = fixture.tick(1L, List.of(passive(0.0, 2_000.0, 12.0, 0.30, 1_200.0, Double.NaN)), List.of(), 0);

		assertFalse(output.contactEstimates.isEmpty());
		assertTrue(Double.isNaN(output.contactEstimates.getLast().estimatedHeading()),
				"A fresh report of unresolved motion must not advertise the old heading as a current solution.");
	}

	@Test
	void louderDistantContactDoesNotBlendIntoAnEstablishedTrackInEitherListOrder() {
		for (boolean distractorFirst : List.of(false, true)) {
			var fixture = new Fixture();
			fixture.tick(0L, List.of(), List.of(active(0.0, 2_000.0, 0.0)), 0);
			var incumbent = passive(0.0, 2_000.16, 10.0, 0.65, 100.0, 0.0);
			var distractor = passive(Math.PI / 2.0, 2_000.0, 30.0, 0.65, 100.0, Math.PI / 2.0);
			var contacts = distractorFirst ? List.of(distractor, incumbent) : List.of(incumbent, distractor);
			var output = fixture.tick(1L, contacts, List.of(), 0);

			assertFalse(output.contactEstimates.isEmpty());
			var estimate = output.contactEstimates.getLast();
			assertTrue(Math.hypot(estimate.x(), estimate.y() - 2_000.16) < 100.0,
					"The incumbent is acoustically continuous; a louder contact ninety degrees away must not move it."
							+ " distractorFirst=" + distractorFirst);
			assertEquals(0.0, estimate.estimatedHeading(), 1.0e-9,
					"Motion estimates from distinct contacts must not be combined.");
		}
	}

	@Test
	void activeReturnForTheSameMovingTargetStillRefreshesItsPosition() {
		var fixture = new Fixture();
		fixture.tick(0L, List.of(), List.of(active(0.0, 2_000.0, 0.0)), 0);
		var output = fixture.tick(1L, List.of(), List.of(active(0.0, 2_000.16, 0.0)), 0);

		assertFalse(output.contactEstimates.isEmpty());
		assertEquals(0.0, output.contactEstimates.getLast().x(), 1.0e-9);
		assertEquals(2_000.16, output.contactEstimates.getLast().y(), 1.0e-9,
				"Continuity gating must allow a new active fix for the tracked target.");
	}

	@Test
	void freshTargetManeuverCannotLaunchUsingTheCachedPreTurnHeading() {
		var fixture = new Fixture();
		fixture.tick(400L, List.of(), List.of(active(0.0, 1_500.0, Math.PI / 2.0)), 8);
		var output = continueNorthboundTrack(fixture);

		assertFalse(output.contactEstimates.isEmpty());
		assertEquals(0.0, output.contactEstimates.getLast().estimatedHeading(), 1.0e-9,
				"Fresh passive fixes must describe the post-turn motion before checking the launch.");
		if (output.launchedTorpedo != null) {
			double missionHeading = Double.parseDouble(output.launchedTorpedo.missionData().split(";")[3]);
			assertEquals(0.0, missionHeading, 1.0e-6,
					"A launch may use updated motion or wait for confirmation, but cannot use the pre-turn heading.");
		}
	}

	@Test
	void unchangedTargetMotionKeepsTheRecentActiveFixLaunchable() {
		var fixture = new Fixture();
		fixture.tick(400L, List.of(), List.of(active(0.0, 1_500.0, 0.0)), 8);
		var output = continueNorthboundTrack(fixture);

		assertEquals(1, output.launchedTorpedoCount,
				"The maneuver safeguard must preserve firing on a recent, continuous, unchanged-motion target.");
		assertEquals(0.0, Double.parseDouble(output.launchedTorpedo.missionData().split(";")[3]), 1.0e-6);
	}

	@Test
	void discontinuousReplacementAfterSilenceDoesNotInheritMotionOrActiveFireConfirmation() {
		var fixture = new Fixture();
		fixture.tick(400L, List.of(), List.of(active(0.0, 1_500.0, Math.PI / 2.0)), 8);
		for (long tick = 401L; tick <= 600L; tick++) {
			fixture.tick(tick, List.of(), List.of(), 8);
		}
		var replacement = new SonarContact(Math.PI / 2.0, 30.0, 1_500.0, false, -1.0, Math.toRadians(1.0), 300.0,
				Double.NaN, 0.65, Double.NaN, Double.NaN, SonarContact.Classification.SUBMARINE);
		var output = fixture.tick(601L, List.of(replacement), List.of(), 8);

		assertFalse(output.contactEstimates.isEmpty(),
				"A stale track must allow a distinct new contact to be acquired.");
		var estimate = output.contactEstimates.getLast();
		assertEquals(1_500.0, estimate.x(), 1.0e-6,
				"A new track must start at the new contact, without blending identities.");
		assertEquals(0.0, estimate.y(), 1.0e-6);
		assertTrue(Double.isNaN(estimate.estimatedHeading()),
				"Unknown replacement heading must not inherit old motion.");
		assertEquals(-1.0, estimate.estimatedSpeed(), "Unknown replacement speed must not inherit old motion.");
		assertEquals("passive", estimate.label(), "The replacement has never received an active range fix.");
		assertEquals(0, output.launchedTorpedoCount,
				"The old target's recent active fix cannot authorize a new-target shot.");
	}

	@Test
	void passiveRangeClampDisplacementIsIncludedInThePositionBound() {
		var fixture = new Fixture();
		var contact = passive(0.0, 8_000.0, 12.0, 0.85, 300.0, 0.0);
		var output = fixture.tick(0L, List.of(contact), List.of(), 0);

		assertFalse(output.contactEstimates.isEmpty());
		var estimate = output.contactEstimates.getLast();
		double shiftFromReportedPosition = Math.hypot(estimate.x(), estimate.y() - contact.range());
		assertTrue(estimate.uncertaintyRadius() >= shiftFromReportedPosition + contact.rangeUncertainty(),
				"Moving a range estimate into the tactical planning corridor must widen its published position bound.");
	}

	@Test
	void broadFreshPassiveContactRemainsSearchableThroughShortSilence() {
		var fixture = new Fixture();
		var first = fixture.tick(0L, List.of(passive(0.0, 4_000.0, 12.0, 0.65, 6_000.0, Double.NaN)), List.of(), 0);
		assertFalse(first.contactEstimates.isEmpty(),
				"Broad fresh range information should remain available for search.");
		assertTrue(first.contactEstimates.getLast().uncertaintyRadius() >= 6_000.0);
		assertFalse(first.status.startsWith("C"), "A broad range bound does not support confident tactical pursuit.");

		var coast = fixture.tick(1L, List.of(), List.of(), 0);
		assertFalse(coast.contactEstimates.isEmpty(),
				"A one-tick detection gap should preserve the broad search track.");
		assertTrue(coast.contactEstimates.getLast().uncertaintyRadius() >= 6_000.0);
	}

	@Test
	void recentActiveFixWithInitiallyUnresolvedHeadingRemainsLaunchable() {
		var fixture = new Fixture();
		fixture.tick(400L, List.of(), List.of(active(0.0, 1_500.0, Double.NaN)), 8);
		TestHelpers.CapturedOutput output = null;
		for (long tick = 401L; tick <= 500L; tick++) {
			output = fixture.tick(tick,
					List.of(passive(0.0, 1_500.0 + (tick - 400L) * 8.0 / 50.0, 12.0, 0.30, 60.0, Double.NaN)),
					List.of(), 8);
		}

		assertTrue(Double.isNaN(output.contactEstimates.getLast().estimatedHeading()));
		assertEquals(1, output.launchedTorpedoCount,
				"An initially unresolved heading is not a maneuver contradicting a recent active range fix.");
	}

	@Test
	void credibleActiveEchoCanReviseThePassiveAcousticClassification() {
		var fixture = new Fixture();
		var passiveSurface = new SonarContact(0.0, 20.0, 2_000.0, false, 8.0, Math.toRadians(1.0), 1_200.0, Double.NaN,
				0.65, Double.NaN, Double.NaN, SonarContact.Classification.SURFACE_SHIP);
		fixture.tick(0L, List.of(passiveSurface), List.of(), 0);
		var output = fixture.tick(1L, List.of(), List.of(active(0.0, 2_000.0, 0.0)), 0);

		assertFalse(output.contactEstimates.isEmpty());
		var estimate = output.contactEstimates.getLast();
		assertEquals("ping", estimate.label(),
				"A matching active echo must correct an uncertain passive classification.");
		assertEquals(80.0, estimate.uncertaintyRadius(), 1.0e-9);
		assertEquals(0.0, estimate.estimatedHeading(), 1.0e-9);
	}

	@Test
	void independentActiveRangeBoundIncludesThePublishedPositionOffset() {
		var fixture = new Fixture();
		fixture.tick(0L, List.of(), List.of(active(0.0, 2_000.0, 0.0)), 0);
		var output = fixture.tick(1L, List.of(passive(Math.toRadians(14.9), 2_000.0, 12.0, 0.95, 1_500.0, 0.0)),
				List.of(), 0);

		assertFalse(output.contactEstimates.isEmpty());
		var estimate = output.contactEstimates.getLast();
		double predictionOffset = Math.hypot(estimate.x(), estimate.y() - 2_000.16);
		assertTrue(predictionOffset > 170.0,
				"The observation must measurably shift the published estimate from the fix.");
		assertTrue(estimate.uncertaintyRadius() >= 80.0 + predictionOffset,
				"An independent active bound must be translated to cover the actual published estimate centre.");
	}

	@Test
	void targetDepartureInvalidatesTheEarlierStoppedMotionFix() {
		var fixture = new Fixture();
		fixture.tick(400L, List.of(), List.of(activeMotion(0.0)), 8);
		var output = continueDepartingTrack(fixture);

		assertEquals(5.0, output.contactEstimates.getLast().estimatedSpeed(), 1.0e-6,
				"The fresh motion estimate must reflect the target's departure.");
		assertEquals(0, output.launchedTorpedoCount,
				"Known zero speed is a motion fix contradicted by a fresh five-metre-per-second departure.");
	}

	@Test
	void newlyResolvedSpeedDoesNotContradictAnInitiallyUnknownMotionFix() {
		var fixture = new Fixture();
		fixture.tick(400L, List.of(), List.of(activeMotion(-1.0)), 8);
		var output = continueDepartingTrack(fixture);

		assertEquals(5.0, output.contactEstimates.getLast().estimatedSpeed(), 1.0e-6);
		assertEquals(1, output.launchedTorpedoCount,
				"Resolving previously unknown speed must preserve a recent, independent active range fix.");
	}

	private static SonarContact activeMotion(double speed) {
		return new SonarContact(0.0, 20.0, 1_500.0, true, speed, Math.toRadians(0.5), 30.0, Double.NaN, 0.95,
				Double.NaN, -140.0, SonarContact.Classification.SUBMARINE);
	}

	private static TestHelpers.CapturedOutput continueDepartingTrack(Fixture fixture) {
		TestHelpers.CapturedOutput output = null;
		for (long tick = 401L; tick <= 500L; tick++) {
			var contact = new SonarContact(0.0, 12.0, 1_500.0 + (tick - 400L) * 5.0 / 50.0, false, 5.0,
					Math.toRadians(1.0), 60.0, Double.NaN, 0.90, 0.0, Double.NaN,
					SonarContact.Classification.SUBMARINE);
			output = fixture.tick(tick, List.of(contact), List.of(), 8);
		}
		return output;
	}

	private static TestHelpers.CapturedOutput continueNorthboundTrack(Fixture fixture) {
		TestHelpers.CapturedOutput output = null;
		for (long tick = 401L; tick <= 500L; tick++) {
			double range = 1_500.0 + (tick - 400L) * 8.0 / 50.0;
			output = fixture.tick(tick, List.of(passive(0.0, range, 12.0, 0.90, 60.0, 0.0)), List.of(), 8);
		}
		return output;
	}

	private static SonarContact active(double bearing, double range, double heading) {
		return new SonarContact(bearing, 20.0, range, true, 8.0, Math.toRadians(0.5), 30.0, Double.NaN, 0.95, heading,
				-140.0, SonarContact.Classification.SUBMARINE);
	}

	private static SonarContact passive(
		double bearing, double range, double signalExcess, double quality, double rangeUncertainty, double heading) {
		return new SonarContact(bearing, signalExcess, range, false, 8.0, Math.toRadians(1.0), rangeUncertainty,
				Double.NaN, quality, heading, Double.NaN, SonarContact.Classification.SUBMARINE);
	}

	private static final class Fixture {
		private final GeneratedWorld world = GeneratedWorld.deepFlat();
		private final CodexAttackSub controller = new CodexAttackSub();
		private final EnvironmentSnapshot environment = new EnvironmentSnapshot(world.terrain(), world.thermalLayers(),
				world.currentField());

		private Fixture() {
			controller.onMatchStart(
					new MatchContext(world.config(), world.terrain(), world.thermalLayers(), world.currentField()));
		}

		private TestHelpers.CapturedOutput tick(
			long tick, List<SonarContact> passive, List<SonarContact> active, int ammunition) {
			var self = new SubmarineState(new Pose(new Vec3(0.0, 0.0, -140.0), 0.0, 0.0, 0.0),
					new Velocity(Vec3.ZERO, Vec3.ZERO), 1_000, ammunition);
			var output = new TestHelpers.CapturedOutput();
			controller.onTick(new TestHelpers.TestInput(tick, 1.0 / 50.0, self, environment, passive, active, 250),
					output);
			return output;
		}
	}
}
