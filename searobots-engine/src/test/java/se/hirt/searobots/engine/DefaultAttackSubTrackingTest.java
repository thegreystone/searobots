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
import se.hirt.searobots.api.ContactEstimate;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.EnvironmentSnapshot;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** The starter controller uses public measurements and elapsed time for its contact estimates. */
class DefaultAttackSubTrackingTest {

	@Test
	void closeActiveFixesAreNotReplacedByAnInventedStandOffRange() {
		for (double range : new double[] {25, 0.5}) {
			var fixture = new Fixture();
			var output = fixture.tick(0, 0.02, List.of(), List.of(active(Math.PI / 2, range, 0, Double.NaN)));
			assertTrue(fixture.controller.hasTrackedContact());
			assertEquals(range, fixture.controller.estimatedRange(), 1e-9);
			assertEquals(range, estimate(output).x(), 1e-9);
			assertEquals(0, estimate(output).y(), 1e-9);
		}
	}

	@Test
	void silenceProjectsTheWorldFixUsingActualElapsedTimeAndCurrentOwnPosition() {
		var fixture = new Fixture();
		var first = fixture.tick(0, 0.02, List.of(), List.of(active(0, 1000, 10, Math.PI / 2)));
		fixture.own.setX(400);
		fixture.own.setY(300);
		var output = fixture.tick(100, 2.0, List.of(), List.of());
		var prediction = estimate(output);
		assertEquals(20, prediction.x(), 1e-9, "Two seconds elapsed even though the controller was called only once");
		assertEquals(1000, prediction.y(), 1e-9, "Ownship movement cannot move the target's world-coordinate anchor");
		assertEquals(Math.hypot(20 - 400, 1000 - 300), fixture.controller.estimatedRange(), 1e-9);
		assertEquals(Math.pow(0.9985, 100), prediction.contactAlive(), 1e-12);
		assertTrue(prediction.uncertaintyRadius() > estimate(first).uncertaintyRadius());
		assertEquals("ping", estimate(first).label());
		assertEquals("coast", prediction.label());
	}

	@Test
	void coastingDoesNotDependOnControllerCallCadence() {
		var once = new Fixture();
		var frequently = new Fixture();
		var fix = active(0, 1000, 10, Math.PI / 2);
		once.tick(0, 0.02, List.of(), List.of(fix));
		frequently.tick(0, 0.02, List.of(), List.of(fix));
		var single = estimate(once.tick(100, 2, List.of(), List.of()));
		ContactEstimate repeated = null;
		for (int index = 1; index <= 20; index++) {
			repeated = estimate(frequently.tick(index * 5L, 0.1, List.of(), List.of()));
		}
		assertNotNull(repeated);
		assertEquals(single.x(), repeated.x(), 1e-9);
		assertEquals(single.y(), repeated.y(), 1e-9);
		assertEquals(single.uncertaintyRadius(), repeated.uncertaintyRadius(), 1e-6);
		assertEquals(single.contactAlive(), repeated.contactAlive(), 1e-12);
	}

	@Test
	void passiveMeasurementsKeepTheirReportedUncertainty() {
		var fixture = new Fixture();
		var contact = passive(0, 2000, 8, 0, 1200, 0.65);
		for (int tick = 0; tick < 5; tick++) {
			var output = fixture.tick(tick, 0.02, List.of(contact), List.of());
			assertTrue(estimate(output).uncertaintyRadius() >= contact.rangeUncertainty(),
					"A passive report cannot be made precise by the controller's old range-fraction cap");
			assertNull(output.launchedTorpedo, "A target with this broad uncertainty is not a precise shot");
		}
	}

	@Test
	void bearingOnlyEvidenceCanBeInvestigatedWithoutInventingACartesianFix() {
		var fixture = new Fixture();
		var bearing = passive(0, 0, 8, Double.NaN, Double.MAX_VALUE, 0.05);
		for (int tick = 0; tick < 5; tick++) {
			var output = fixture.tick(tick, 0.02, List.of(bearing), List.of());
			assertFalse(fixture.controller.hasTrackedContact());
			assertTrue(output.contactEstimates.isEmpty(), "A bearing alone has no world-coordinate position");
			assertNull(output.launchedTorpedo);
		}
		assertEquals(DefaultAttackSub.State.TRACKING, fixture.controller.state(),
				"Confirmed bearing-only contacts still warrant information-gathering maneuvers");
	}

	@Test
	void freshUnresolvedMotionStopsUsingTheEarlierHeading() {
		var fixture = new Fixture();
		fixture.tick(0, 0.02, List.of(), List.of(active(0, 1000, 10, Math.PI / 2)));
		var fresh = fixture.tick(1, 0.02, List.of(passive(0, 1000, 8, Double.NaN, 400, 0.65)), List.of());
		assertTrue(Double.isNaN(fixture.controller.trackedHeading()));
		assertTrue(Double.isNaN(estimate(fresh).estimatedHeading()));
		var coast = estimate(fixture.tick(101, 2, List.of(), List.of()));
		assertEquals(estimate(fresh).x(), coast.x(), 1e-9);
		assertEquals(estimate(fresh).y(), coast.y(), 1e-9);
		assertTrue(coast.uncertaintyRadius() > estimate(fresh).uncertaintyRadius());
	}

	@Test
	void unknownSpeedCannotKeepStalkingOrAuthorizeAnInventedFiringLead() {
		var fixture = new Fixture();
		fixture.own.setTorpedoesRemaining(0);
		for (int tick = 0; tick < 4; tick++) {
			fixture.tick(tick, 0.02, List.of(), List.of(active(0, 1500, 8, 0)));
		}
		assertEquals(DefaultAttackSub.State.STALKING, fixture.controller.state());
		fixture.own.setTorpedoesRemaining(8);
		var output = fixture.tick(4, 0.02, List.of(), List.of(active(0, 1500, -1, 0)));
		assertEquals(-1, estimate(output).estimatedSpeed());
		assertEquals(DefaultAttackSub.State.TRACKING, fixture.controller.state(),
				"A known heading alone does not resolve the target's velocity");
		assertNull(output.launchedTorpedo, "The controller cannot substitute a made-up speed for unknown motion");
	}

	@Test
	void aReportedStoppedTargetKeepsItsZeroSpeedInTheMission() {
		var fixture = new Fixture();
		TestHelpers.CapturedOutput output = null;
		for (int tick = 0; tick < 4; tick++) {
			output = fixture.tick(tick, 0.02, List.of(), List.of(active(0, 1500, 0, 0)));
		}
		assertNotNull(output);
		assertNotNull(output.launchedTorpedo, "A precise, aligned shot with known motion remains available");
		assertEquals(0, Double.parseDouble(output.launchedTorpedo.missionData().split(",")[4]));
	}

	@Test
	void maturePassiveApproachRequestsConfirmationThenFiresWithAKnownDepthFix() {
		var fixture = new Fixture();
		var passive = passive(0, 1200, 8, 0, 30, 0.9);
		TestHelpers.CapturedOutput output = null;
		for (int tick = 0; tick < 5; tick++) {
			output = fixture.tick(tick, 0.02, List.of(passive), List.of(), 0);
			assertNull(output.launchedTorpedo,
					"A precise slant range with unknown depth does not authorize a precise horizontal shot");
		}
		assertNotNull(output);
		assertEquals(DefaultAttackSub.State.ATTACKING, fixture.controller.state());
		assertTrue(output.pinged, "The existing stern approach requests an active confirmation before firing");
		assertTrue(estimate(output).uncertaintyRadius() >= 1200);

		var confirmed = fixture.tick(5, 0.02, List.of(passive), List.of(active(0, 1200, 8, 0)), 250);
		assertTrue(estimate(confirmed).uncertaintyRadius() < 250);
		assertNotNull(confirmed.launchedTorpedo, "A same-depth active fix makes the aligned shot viable");
		assertEquals(8, Double.parseDouble(confirmed.launchedTorpedo.missionData().split(",")[4]));
	}

	@Test
	void aNewMatchCannotCoastThePreviousMatchesTrack() {
		var fixture = new Fixture();
		fixture.tick(0, 0.02, List.of(), List.of(active(0, 1000, 10, Math.PI / 2)));
		assertTrue(fixture.controller.hasTrackedContact());
		fixture.controller.onMatchStart(fixture.context);
		var output = fixture.tick(0, 1, List.of(), List.of());
		assertFalse(fixture.controller.hasTrackedContact());
		assertTrue(output.contactEstimates.isEmpty());
		assertEquals(0, fixture.controller.contactAlive());
	}

	@Test
	void anExpiredBroadTrackIsDiscardedDuringSilence() {
		var fixture = new Fixture();
		fixture.tick(0, 0.02, List.of(), List.of(active(0, 1000, 10, Math.PI / 2)));
		var output = fixture.tick(20000, 400, List.of(), List.of());
		assertFalse(fixture.controller.hasTrackedContact());
		assertTrue(output.contactEstimates.isEmpty());
		assertTrue(Double.isNaN(fixture.controller.trackedX()));
	}

	private static ContactEstimate estimate(TestHelpers.CapturedOutput output) {
		assertFalse(output.contactEstimates.isEmpty());
		return output.contactEstimates.getLast();
	}

	private static SonarContact active(double bearing, double range, double speed, double heading) {
		return new SonarContact(bearing, 30, range, true, speed, Math.toRadians(1), range * 0.02, Double.NaN, 0.95,
				heading, -200, SonarContact.Classification.SUBMARINE);
	}

	private static SonarContact passive(
		double bearing, double range, double speed, double heading, double rangeUncertainty, double quality) {
		return new SonarContact(bearing, 15, range, false, speed, Math.toRadians(2), rangeUncertainty, Double.NaN,
				quality, heading, Double.NaN, SonarContact.Classification.SUBMARINE);
	}

	private static final class Fixture {
		final DefaultAttackSub controller = new DefaultAttackSub();
		final CurrentField current = new CurrentField(List.of());
		final TerrainMap terrain;
		final MatchContext context;
		final EnvironmentSnapshot environment;
		final SubmarineEntity own = new SubmarineEntity(VehicleConfig.submarine(), 0, controller, new Vec3(0, 0, -200),
				0, Color.GREEN, 1000);

		Fixture() {
			double[] elevations = new double[201 * 201];
			Arrays.fill(elevations, -500);
			terrain = new TerrainMap(elevations, 201, 201, -10000, -10000, 100);
			context = new MatchContext(MatchConfig.withDefaults(42), terrain, List.of(), current);
			environment = new EnvironmentSnapshot(terrain, List.of(), current);
			controller.onMatchStart(context);
			own.setTorpedoesRemaining(8);
		}

		TestHelpers.CapturedOutput tick(long tick, double dt, List<SonarContact> passive, List<SonarContact> active) {
			return tick(tick, dt, passive, active, 250);
		}

		TestHelpers.CapturedOutput tick(
			long tick, double dt, List<SonarContact> passive, List<SonarContact> active, int activeSonarCooldownTicks) {
			var output = new TestHelpers.CapturedOutput();
			controller.onTick(new TestHelpers.TestInput(tick, dt, own.state(), environment, passive, active,
					activeSonarCooldownTicks), output);
			return output;
		}
	}
}
