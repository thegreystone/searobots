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
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
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
}
