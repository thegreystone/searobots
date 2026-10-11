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

/** Range accuracy and uncertainty across active fixes and passive reacquisition. */
class ContactTrackerRangeTest {

	@Test
	void passiveUpdatesPreserveCloseAndSubmeterActiveFixes() {
		for (double distance : new double[] {50, 0.5}) {
			var tracker = new ContactTracker();
			var rng = zeroNoise();
			tracker.updateFromPing(0, distance);
			for (long tick = 1; tick <= 250; tick++) {
				observe(tracker, rng, tick, 0, 0, distance);
				assertEquals(distance, tracker.estimatedRange(), 1e-10,
						"A stationary target's active fix cannot drift toward an arbitrary minimum distance");
			}
			assertTrue(tracker.solutionQuality() > 0.85, "The active fix is still recent");
		}
	}

	@Test
	void unresolvedNegativeGuessesRemainPhysicalAndUncertain() {
		var tracker = new ContactTracker();
		var rng = new Random(42) {
			@Override
			public double nextGaussian() {
				return -1;
			}
		};
		observe(tracker, rng, 0, 0, 0, 500);
		assertTrue(tracker.estimatedRange() > 0 && tracker.estimatedRange() <= 1e-6);
		assertTrue(tracker.rangeUncertainty() >= 500,
				"Clamping a negative guess cannot make its substantial initial bias disappear");
		assertTrue(tracker.solutionQuality() <= 0.1);
	}

	@Test
	void closeSonarEchoStaysUsefulOnFollowingPassiveTicks() {
		var fixture = new SonarFixture(50);
		var echo = fixture.ping(0);
		assertEquals(50, echo.range(), 5);
		SonarContact passive = null;
		for (long tick = 1; tick <= 250; tick++) {
			passive = fixture.measure(tick).passiveContacts().getFirst();
		}
		assertNotNull(passive);
		assertEquals(50, passive.range(), 10, "Passive TMA must preserve the measured close-range fix");
		assertTrue(passive.solutionQuality() > 0.85);
	}

	@Test
	void passiveReacquisitionPreservesPossibleUnobservedTargetMovement() {
		var fixture = new SonarFixture(200);
		var echo = fixture.ping(0);
		fixture.target.setSourceLevelDb(60);
		for (long tick = 1; tick <= 1000; tick++) {
			// An unobserved departure at the configured maximum submarine speed.
			fixture.target.setX(200 + tick * 0.3);
			assertTrue(fixture.measure(tick).passiveContacts().isEmpty());
		}
		fixture.target.setSourceLevelDb(125);
		var bearing = fixture.measure(1001).passiveContacts().getFirst();
		assertTrue(bearing.rangeUncertainty() >= echo.rangeUncertainty() + 300 - 1e-8,
				"A single fresh bearing has no geometry to remove twenty seconds of possible radial movement");
		assertTrue(bearing.rangeUncertainty() >= Math.abs(bearing.range() - 500));
	}

	@Test
	void onlyFreshSpatialGeometryRepaysCoastingUncertainty() {
		var tracker = new ContactTracker();
		var rng = zeroNoise();
		tracker.updateFromPing(0, 200);
		tracker.decay(1000, 15);
		double coasted = tracker.rangeUncertainty();
		for (long tick = 1001; tick <= 1500; tick++) {
			observe(tracker, rng, tick, 0, 0, 500);
			assertTrue(tracker.rangeUncertainty() >= coasted,
					"Stationary bearings cannot repay the possible movement during an earlier gap");
		}
		double ownX = 0;
		for (long tick = 1501; tick <= 7500; tick++) {
			double heading = ((tick - 1501) / 500) % 2 == 0 ? Math.PI / 2 : 3 * Math.PI / 2;
			ownX += 0.24 * Math.sin(heading);
			observe(tracker, rng, tick, ownX, heading, 500);
		}
		assertTrue(tracker.solutionQuality() > 0.5);
		assertTrue(tracker.rangeUncertainty() < coasted * 0.5,
				"Useful fresh translating legs can resolve the uncertainty left by the gap");
	}

	@Test
	void aNewActiveFixClearsCoastingUncertaintyForSubsequentPassiveUpdates() {
		var tracker = new ContactTracker();
		var rng = zeroNoise();
		tracker.updateFromPing(0, 200);
		tracker.decay(1000, 15);
		assertEquals(304, tracker.rangeUncertainty(), 1e-9);
		tracker.updateFromPing(1001, 500);
		assertEquals(10, tracker.rangeUncertainty(), 1e-9);
		observe(tracker, rng, 1002, 0, 0, 500);
		assertTrue(tracker.rangeUncertainty() < 20,
				"A measured new range clears the unobserved-motion bound rather than restoring it next tick");
	}

	private static void observe(
		ContactTracker tracker, Random rng, long tick, double ownX, double heading, double targetY) {
		tracker.update(tick, Math.atan2(-ownX, targetY), 30, ownX, 0, heading, false, Math.hypot(ownX, targetY), 0,
				targetY, rng);
	}

	private static Random zeroNoise() {
		return new Random(42) {
			@Override
			public double nextGaussian() {
				return 0;
			}
		};
	}

	private static final class SonarFixture {
		final SonarModel sonar = new SonarModel(42);
		final SubmarineEntity own = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, -200), 0,
				Color.GREEN, 1000);
		final SubmarineEntity target;
		final TerrainMap terrain;

		SonarFixture(double distance) {
			target = new SubmarineEntity(VehicleConfig.submarine(), 1, null, new Vec3(distance, 0, -200), 0, Color.RED,
					1000);
			own.setSourceLevelDb(80);
			target.setSourceLevelDb(125);
			double[] elevations = new double[201 * 201];
			Arrays.fill(elevations, -500);
			terrain = new TerrainMap(elevations, 201, 201, -10000, -10000, 100);
		}

		SonarContact ping(long tick) {
			own.activeSonarPing();
			return measure(tick).activeReturns().getFirst();
		}

		SonarModel.SonarResult measure(long tick) {
			var result = sonar.computeContacts(tick, List.of(own, target), terrain, List.of()).get(0);
			sonar.postTick(List.of(own, target));
			return result;
		}
	}
}
