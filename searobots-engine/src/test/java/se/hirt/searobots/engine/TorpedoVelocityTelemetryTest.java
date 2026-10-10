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
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TorpedoVelocityTelemetryTest {

	private static final double EPSILON = 1e-10;

	private TorpedoEntity pitchedTorpedo() {
		return new TorpedoEntity(1, 0, VehicleConfig.torpedo(), null, new Vec3(0, 0, -200), Math.toRadians(60),
				Math.toRadians(25), 10, Color.YELLOW);
	}

	@Test
	void velocityAndSnapshotReportPitchAndYawRatesAtPitchedHeading() {
		var torpedo = pitchedTorpedo();
		torpedo.setPitchRate(0.17);
		torpedo.setYawRate(-0.31);

		var expected = new Vec3(0, 0.17, -0.31);
		assertAll(() -> assertEquals(expected, torpedo.velocity().angular()),
				() -> assertEquals(expected, torpedo.snapshot().velocity().angular()));
	}

	@Test
	void stationaryPitchedOrientationDoesNotReportAngularMotion() {
		var torpedo = pitchedTorpedo();
		torpedo.setSpeed(0);

		assertEquals(Vec3.ZERO, torpedo.velocity().angular());
	}

	@Test
	void physicsProducedRatesMatchMeasuredPitchAndWrappedHeadingChange() {
		double dt = 1.0 / 50;
		var torpedo = pitchedTorpedo();
		torpedo.setHeading(2 * Math.PI - 1e-4);
		torpedo.setSpeed(8);
		torpedo.setPitchRate(0.04);
		torpedo.setYawRate(0.07);
		torpedo.createOutput().setRudder(0.4);
		torpedo.createOutput().setSternPlanes(-0.4);
		double headingBefore = torpedo.heading();
		double pitchBefore = torpedo.pitch();

		new TorpedoPhysics().step(torpedo, dt, null, new CurrentField(List.of()), null);

		double headingDelta = torpedo.heading() - headingBefore;
		double wrappedHeadingDelta = Math.atan2(Math.sin(headingDelta), Math.cos(headingDelta));
		var angular = torpedo.velocity().angular();
		assertTrue(torpedo.heading() < headingBefore, "The fixture must cross the heading wrap");
		assertTrue(Math.abs(torpedo.pitch() - pitchBefore) > 1e-5, "The fixture must actually change pitch");
		assertAll(() -> assertEquals(0, angular.x(), EPSILON),
				() -> assertEquals((torpedo.pitch() - pitchBefore) / dt, angular.y(), EPSILON),
				() -> assertEquals(wrappedHeadingDelta / dt, angular.z(), EPSILON));
	}

	@ParameterizedTest
	@ValueSource(ints = {-1, 1})
	void pitchLimitStopsOutwardRateAndAllowsImmediateInwardRecovery(int direction) {
		double dt = 1.0 / 50;
		double pitchLimit = direction * Math.PI / 3;
		var torpedo = pitchedTorpedo();
		torpedo.setSpeed(8);
		torpedo.setPitch(pitchLimit);
		torpedo.setPitchRate(direction * 0.17);
		var physics = new TorpedoPhysics();

		physics.step(torpedo, dt, null, new CurrentField(List.of()), null);

		assertAll(() -> assertEquals(pitchLimit, torpedo.pitch(), EPSILON),
				() -> assertEquals(0, torpedo.pitchRate(), EPSILON, "The stored rate must not wind up at the limit"),
				() -> assertEquals(0, torpedo.velocity().angular().y(), EPSILON),
				() -> assertEquals(0, torpedo.snapshot().velocity().angular().y(), EPSILON));

		torpedo.setActualSternPlanes(-direction * 0.4);
		torpedo.createOutput().setSternPlanes(-direction * 0.4);
		physics.step(torpedo, dt, null, new CurrentField(List.of()), null);

		assertTrue(direction * torpedo.pitchRate() < 0, "An inward command must immediately reverse the rate");
		assertTrue(direction * (torpedo.pitch() - pitchLimit) < 0, "The pitch must leave the limit inward");
		assertAll(() -> assertEquals((torpedo.pitch() - pitchLimit) / dt, torpedo.pitchRate(), EPSILON),
				() -> assertEquals(torpedo.pitchRate(), torpedo.velocity().angular().y(), EPSILON));
	}

	@ParameterizedTest
	@ValueSource(ints = {-1, 1})
	void pitchLimitPreservesExistingInwardRate(int direction) {
		double dt = 1.0 / 50;
		double pitchLimit = direction * Math.PI / 3;
		var torpedo = pitchedTorpedo();
		torpedo.setSpeed(8);
		torpedo.setPitch(pitchLimit);
		torpedo.setPitchRate(-direction * 0.17);

		new TorpedoPhysics().step(torpedo, dt, null, new CurrentField(List.of()), null);

		assertTrue(direction * torpedo.pitchRate() < 0, "The limit must preserve an inward rate");
		assertTrue(direction * (torpedo.pitch() - pitchLimit) < 0, "An inward rate must move away from the limit");
		assertAll(() -> assertEquals((torpedo.pitch() - pitchLimit) / dt, torpedo.pitchRate(), EPSILON),
				() -> assertEquals(torpedo.pitchRate(), torpedo.velocity().angular().y(), EPSILON));
	}

	@Test
	void linearVelocityUsesWorldAxesAndAddsVerticalHeaveAtPitchedHeading() {
		var torpedo = pitchedTorpedo();
		torpedo.setSpeed(8);
		torpedo.setVerticalSpeed(-1);

		var linear = torpedo.velocity().linear();
		assertAll(
				() -> assertEquals(8 * Math.sin(Math.toRadians(60)) * Math.cos(Math.toRadians(25)), linear.x(),
						EPSILON),
				() -> assertEquals(8 * Math.cos(Math.toRadians(60)) * Math.cos(Math.toRadians(25)), linear.y(),
						EPSILON),
				() -> assertEquals(8 * Math.sin(Math.toRadians(25)) - 1, linear.z(), EPSILON));
	}
}
