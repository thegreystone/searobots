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
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.EnvironmentSnapshot;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Vec2;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static se.hirt.searobots.api.VehicleConfig.submarine;

class SubmarineVelocityTelemetryTest {

	private static final double EPSILON = 1e-12;

	private SubmarineEntity pitchedSubmarine() {
		var sub = new SubmarineEntity(submarine(), 0, new DefaultAttackSub(), new Vec3(0, 0, -200), Math.toRadians(60),
				Color.GREEN, 1000);
		sub.setPitch(Math.toRadians(25));
		return sub;
	}

	@Test
	void controllerStateAndSnapshotReportPitchAndYawRatesAtPitchedHeading() {
		var sub = pitchedSubmarine();
		sub.setPitchRate(0.17);
		sub.setYawRate(-0.31);

		var reported = sub.velocity();
		var expected = new Vec3(0, 0.17, -0.31);
		assertAll(() -> assertEquals(expected, reported.angular()),
				() -> assertEquals(expected, sub.state().velocity().angular()),
				() -> assertEquals(expected, sub.snapshot().velocity().angular()));
	}

	@Test
	void stationaryPitchedOrientationDoesNotReportAngularMotion() {
		var sub = pitchedSubmarine();

		assertEquals(Vec3.ZERO, sub.velocity().angular());
	}

	@Test
	void physicsProducedRatesMatchMeasuredHeadingAndPitchChange() {
		var config = MatchConfig.withDefaults(42);
		double dt = 1.0 / config.tickRateHz();
		var sub = pitchedSubmarine();
		sub.setSpeed(8);
		sub.setPitchRate(0.04);
		sub.setYawRate(-0.07);
		sub.setRudder(0.4);
		sub.setSternPlanes(-0.4);
		double headingBefore = sub.heading();
		double pitchBefore = sub.pitch();
		double[] depths = new double[25];
		Arrays.fill(depths, -500);
		var terrain = new TerrainMap(depths, 5, 5, -200, -200, 100);

		new SubmarinePhysics().step(sub, dt, terrain, new CurrentField(List.of()), config.battleArea());

		var angular = sub.state().velocity().angular();
		assertTrue(Math.abs(sub.heading() - headingBefore) > 1e-5, "The fixture must actually turn");
		assertTrue(Math.abs(sub.pitch() - pitchBefore) > 1e-5, "The fixture must actually change pitch");
		assertAll(() -> assertEquals(0, angular.x(), EPSILON),
				() -> assertEquals((sub.pitch() - pitchBefore) / dt, angular.y(), EPSILON),
				() -> assertEquals((sub.heading() - headingBefore) / dt, angular.z(), EPSILON));
	}

	@Test
	void linearVelocityRetainsWorldAxesAtPitchedHeading() {
		var sub = pitchedSubmarine();
		sub.setSpeed(8);
		sub.setVerticalSpeed(-1);

		var linear = sub.velocity().linear();
		assertAll(
				() -> assertEquals(8 * Math.sin(Math.toRadians(60)) * Math.cos(Math.toRadians(25)), linear.x(),
						EPSILON),
				() -> assertEquals(8 * Math.cos(Math.toRadians(60)) * Math.cos(Math.toRadians(25)), linear.y(),
						EPSILON),
				() -> assertEquals(8 * Math.sin(Math.toRadians(25)) - 1, linear.z(), EPSILON));
	}

	@Test
	void pitchLimitsStopOutwardAngularMotion() {
		for (int direction : new int[] {-1, 1}) {
			var sub = pitchedSubmarine();
			sub.setPitch(direction * Math.PI / 4);
			sub.setSpeed(8);
			sub.setPitchRate(direction * 0.5);

			step(sub);

			assertEquals(direction * Math.PI / 4, sub.pitch(), EPSILON);
			assertEquals(0, sub.pitchRate(), EPSILON, "A pitch stop must remove outward angular motion");
			assertEquals(0, sub.state().velocity().angular().y(), EPSILON);
		}
	}

	@Test
	void pitchLimitsAllowInwardAngularMotion() {
		for (int direction : new int[] {-1, 1}) {
			var sub = pitchedSubmarine();
			sub.setPitch(direction * Math.PI / 4);
			sub.setSpeed(8);
			sub.setPitchRate(-direction * 0.5);
			double pitchBefore = sub.pitch();

			step(sub);

			assertTrue(direction * sub.pitchRate() < 0, "Motion away from a pitch stop must remain possible");
			assertEquals((sub.pitch() - pitchBefore) / 0.02, sub.velocity().angular().y(), EPSILON);
		}
	}

	@Test
	void repeatedSurfaceTicksReportClippedVerticalMotionAndPreserveHorizontalDrift() {
		var sub = pitchedSubmarine();
		sub.setZ(-0.01);
		sub.setSpeed(10);
		sub.setSwaySpeed(1.5);
		sub.setVerticalSpeed(2);
		var current = new CurrentField(List.of(new CurrentField.CurrentBand(-1000, 0, new Vec2(2, -3))));
		for (int tick = 0; tick < 20; tick++) {
			double previousX = sub.x(), previousY = sub.y();
			step(sub, current);
			var velocity = sub.velocity().linear();
			var input = new TestHelpers.TestInput(tick, 0.02, sub.state(),
					new EnvironmentSnapshot(flatTerrain(), List.of(), current), List.of(), List.of(), 0);
			assertAll(() -> assertEquals(0, sub.z(), EPSILON), () -> assertEquals(0, velocity.z(), EPSILON),
					() -> assertEquals(0, sub.state().velocity().linear().z(), EPSILON),
					() -> assertEquals(0, sub.snapshot().velocity().linear().z(), EPSILON),
					() -> assertEquals(new Vec3(velocity.x() + 2, velocity.y() - 3, 0), input.groundVelocity()),
					() -> assertEquals(velocity.x() + 2, (sub.x() - previousX) / 0.02, 1e-10),
					() -> assertEquals(velocity.y() - 3, (sub.y() - previousY) / 0.02, 1e-10));
		}
		assertEquals(0, sub.verticalSpeed(), EPSILON, "The surface must remove independent upward heave.");
	}

	@Test
	void levelingAfterSurfaceContactDoesNotInventDownwardHeave() {
		var sub = pitchedSubmarine();
		sub.setZ(0);
		sub.setSpeed(10);
		step(sub);
		sub.setPitch(0);
		sub.setPitchRate(0);
		step(sub);
		assertEquals(0, sub.z(), EPSILON);
		assertEquals(0, sub.velocity().linear().z(), EPSILON);
		assertEquals(0, sub.verticalSpeed(), EPSILON);
	}

	@Test
	void divingPitchImmediatelyReleasesSurfaceConstraint() {
		var sub = pitchedSubmarine();
		sub.setZ(0);
		sub.setSpeed(10);
		step(sub);
		sub.setPitch(-Math.toRadians(20));
		sub.setPitchRate(0);
		step(sub);
		assertTrue(sub.z() < 0, "The submarine must be able to leave the surface on the next tick.");
		assertEquals(sub.speed() * Math.sin(sub.pitch()), sub.velocity().linear().z(), EPSILON);
		assertEquals(sub.velocity().linear().z(), sub.z() / 0.02, EPSILON);
	}

	@Test
	void surfaceAllowsNegativeHeaveAndStopsPositiveHeave() {
		var rising = pitchedSubmarine();
		rising.setZ(-0.01);
		rising.setSpeed(0);
		rising.setVerticalSpeed(2);
		step(rising);
		assertEquals(0, rising.z(), EPSILON);
		assertEquals(0, rising.velocity().linear().z(), EPSILON);
		assertEquals(0, rising.verticalSpeed(), EPSILON);

		var sinking = pitchedSubmarine();
		sinking.setZ(0);
		sinking.setVerticalSpeed(-2);
		step(sinking);
		assertTrue(sinking.z() < 0);
		assertEquals(sinking.velocity().linear().z(), sinking.z() / 0.02, EPSILON);
	}

	@Test
	void surfaceLockedVehicleNeverReportsVerticalTranslation() {
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, new DefaultAttackSub(), Vec3.ZERO, 0, Color.BLUE,
				1000);
		ship.setSpeed(8);
		ship.setPitch(0.3);
		ship.setVerticalSpeed(-2);
		assertEquals(0, ship.velocity().linear().z(), EPSILON);
		step(ship);
		assertEquals(0, ship.z(), EPSILON);
		assertEquals(0, ship.state().velocity().linear().z(), EPSILON);
	}

	private void step(SubmarineEntity sub) {
		step(sub, new CurrentField(List.of()));
	}

	private void step(SubmarineEntity sub, CurrentField currents) {
		new SubmarinePhysics().step(sub, 0.02, flatTerrain(), currents, MatchConfig.withDefaults(42).battleArea());
	}

	private TerrainMap flatTerrain() {
		double[] depths = new double[25];
		Arrays.fill(depths, -500);
		return new TerrainMap(depths, 5, 5, -200, -200, 100);
	}
}
