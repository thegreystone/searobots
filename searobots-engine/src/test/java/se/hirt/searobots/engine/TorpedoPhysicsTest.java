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
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.*;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TorpedoPhysicsTest {

	private static final double DT = 1.0 / 50;
	private static final TorpedoPhysics PHYSICS = new TorpedoPhysics();

	@Test
	void yawAuthorityIsContinuousAcrossStall() {
		var before = settled(20, Math.toRadians(29.99), 0);
		var after = settled(20, Math.toRadians(30.01), 0);
		assertEquals(before.yawRate(), after.yawRate(), before.yawRate() * 0.002,
				"Crossing the stall angle must not abruptly remove rudder authority");
	}

	@Test
	void pitchAuthorityIsContinuousAcrossStall() {
		var before = settled(20, 0, Math.toRadians(29.99));
		var after = settled(20, 0, Math.toRadians(30.01));
		assertEquals(before.pitchRate(), after.pitchRate(), before.pitchRate() * 0.002,
				"Crossing the stall angle must not abruptly remove stern-plane authority");
	}

	@Test
	void fullRudderRetainsUsefulButReducedAuthority() {
		var peak = settled(20, Math.toRadians(30), 0);
		var full = settled(20, Math.toRadians(45), 0);
		double retainedAuthority = full.yawRate() / peak.yawRate();
		assertTrue(retainedAuthority >= 0.55 && retainedAuthority <= 0.65,
				"Full rudder should retain about 60% of peak authority, got " + retainedAuthority);
	}

	@Test
	void fullSternPlanesRetainUsefulButReducedAuthority() {
		var peak = settled(20, 0, Math.toRadians(30));
		var full = settled(20, 0, Math.toRadians(45));
		double retainedAuthority = full.pitchRate() / peak.pitchRate();
		assertTrue(retainedAuthority >= 0.55 && retainedAuthority <= 0.65,
				"Full stern planes should retain about 60% of peak authority, got " + retainedAuthority);
	}

	@Test
	void postStallAuthorityFallsProgressively() {
		double previousYaw = Double.POSITIVE_INFINITY;
		double previousPitch = Double.POSITIVE_INFINITY;
		for (double angle : new double[] {30, 35, 40, 45}) {
			var yaw = settled(20, Math.toRadians(angle), 0);
			var pitch = settled(20, 0, Math.toRadians(angle));
			assertTrue(yaw.yawRate() < previousYaw, "Increasing stalled rudder must reduce yaw authority");
			assertTrue(pitch.pitchRate() < previousPitch,
					"Increasing stalled stern planes must reduce pitch authority");
			previousYaw = yaw.yawRate();
			previousPitch = pitch.pitchRate();
		}
	}

	@Test
	void oppositeControlAnglesProduceOppositeRates() {
		for (double degrees : new double[] {15, 30, 35, 45}) {
			double angle = Math.toRadians(degrees);
			var positive = settled(20, angle, angle);
			var negative = settled(20, -angle, -angle);
			assertEquals(positive.yawRate(), -negative.yawRate(), 1e-12,
					"Port and starboard authority must be symmetric at " + degrees + " degrees");
			assertEquals(positive.pitchRate(), -negative.pitchRate(), 1e-12,
					"Up and down authority must be symmetric at " + degrees + " degrees");
		}
	}

	@Test
	void controlAuthorityIsWeakAtVeryLowSpeedAndAbsentAtRest() {
		double angle = Math.toRadians(30);
		var stopped = settled(0, angle, angle);
		var slow = settled(0.1, angle, angle);
		var moving = settled(5, angle, angle);
		assertEquals(0, stopped.yawRate(), 0);
		assertEquals(0, stopped.pitchRate(), 0);
		assertTrue(slow.yawRate() > 0 && slow.yawRate() < moving.yawRate() * 0.01,
				"Water flow must remain necessary for rudder authority");
		assertTrue(slow.pitchRate() > 0 && slow.pitchRate() < moving.pitchRate() * 0.01,
				"Water flow must remain necessary for stern-plane authority");
	}

	@Test
	void reducingStalledDeflectionRestoresAuthorityWithoutAnInstantRateChange() {
		var yaw = settled(20, Math.toRadians(45), 0);
		var pitch = settled(20, 0, Math.toRadians(45));
		double stalledYaw = yaw.yawRate();
		double stalledPitch = pitch.pitchRate();
		yaw.createOutput().setRudder(2.0 / 3);
		pitch.createOutput().setSternPlanes(2.0 / 3);
		advance(yaw, DT);
		advance(pitch, DT);
		assertEquals(stalledYaw, yaw.yawRate(), stalledYaw * 0.01, "Stall recovery must preserve gradual yaw response");
		assertEquals(stalledPitch, pitch.pitchRate(), stalledPitch * 0.01,
				"Stall recovery must preserve gradual pitch response");
		advance(yaw, 60);
		advance(pitch, 60);
		assertEquals(2.0 / 3, yaw.actualRudder(), 0);
		assertEquals(2.0 / 3, pitch.actualSternPlanes(), 0);
		assertTrue(yaw.yawRate() > stalledYaw * 1.5,
				"Backing off full rudder to the stall angle must restore yaw authority");
		assertTrue(pitch.pitchRate() > stalledPitch * 1.5,
				"Backing off full stern planes to the stall angle must restore pitch authority");
	}

	@Test
	void combinedSteeringSharesTheAuthorityOfOneAxis() {
		for (double speed : new double[] {5, 10, 15, 20, 23}) {
			var yaw = settled(speed, Math.toRadians(30), 0);
			var pitch = settled(speed, 0, Math.toRadians(30));
			var combined = settled(speed, Math.toRadians(30), Math.toRadians(30));
			double maximum = Math.max(yaw.yawRate(), pitch.pitchRate());
			assertTrue(Math.hypot(combined.yawRate(), combined.pitchRate()) <= maximum * 1.002,
					"Using both fins must not gain a second turning budget at " + speed + " m/s");
			assertTrue(combined.yawRate() > 0 && combined.pitchRate() > 0,
					"Sharing authority must preserve both requested steering directions");
		}
	}

	@Test
	void inwardPitchSweepCannotIncreaseYawContributionBeyondTheSharedBudget() {
		var peak = settled(20, Math.toRadians(30), 0);
		advance(peak, 60); // converge the reference beyond the ordinary 60-second response fixture
		var torpedo = fixedSpeed(20, Math.toRadians(30), -Math.toRadians(30));
		torpedo.setPitch(Math.toRadians(50));
		torpedo.setYawRate(0.12);
		torpedo.setPitchRate(-0.095);
		assertTrue(Math.hypot(torpedo.yawRate() * Math.cos(torpedo.pitch()), torpedo.pitchRate()) < peak.yawRate(),
				"The fixture must start inside the steering envelope");
		PHYSICS.step(torpedo, 1, null, null, null);
		assertTrue(torpedo.pitch() < Math.toRadians(50), "The step must move pitch toward level");
		assertTrue(Math.hypot(torpedo.yawRate() * Math.cos(torpedo.pitch()), torpedo.pitchRate()) <= peak.yawRate(),
				"The pitch sweep must not gain curvature just because yaw becomes more horizontal");
	}

	@Test
	void poweredHardTurnCostsSpeedWhileStraightRunRetainsCruiseSpeed() {
		var straight = fixedSpeed(23, 0, 0);
		var turning = fixedSpeed(23, Math.toRadians(30), Math.toRadians(30));
		for (int tick = 0; tick < 6_000; tick++) {
			PHYSICS.step(straight, DT, null, null, null);
			PHYSICS.step(turning, DT, null, null, null);
		}
		assertEquals(23, straight.speed(), 1e-10, "Neutral fins must retain the configured top speed");
		assertTrue(turning.speed() > 18.5 && turning.speed() < 20,
				"A sustained combined turn should lose a modest amount of cruise speed, got " + turning.speed());
	}

	@Test
	void hardSteeringAlsoDissipatesSpeedWhileCoasting() {
		var straight = fixedSpeed(20, 0, 0);
		var turning = fixedSpeed(20, Math.toRadians(30), Math.toRadians(30));
		for (var torpedo : new TorpedoEntity[] {straight, turning}) {
			torpedo.createOutput().setThrottle(0);
			torpedo.setActualThrottle(0);
		}
		for (int tick = 0; tick < 1_000; tick++) {
			PHYSICS.step(straight, DT, null, null, null);
			PHYSICS.step(turning, DT, null, null, null);
		}
		assertTrue(turning.speed() < straight.speed() - 0.5,
				"A coasting weapon must pay the same steering loss without propulsion to replace it");
	}

	@Test
	void reversingControlsPreservesSlewAndRotationalInertia() {
		var torpedo = fixedSpeed(20, 0, 0);
		var output = torpedo.createOutput();
		output.setRudder(1);
		output.setSternPlanes(1);
		advance(torpedo, DT);
		assertEquals(1.5 * DT, torpedo.actualRudder(), 1e-12);
		assertEquals(1.5 * DT, torpedo.actualSternPlanes(), 1e-12);
		assertTrue(torpedo.yawRate() > 0 && torpedo.pitchRate() > 0);

		advance(torpedo, 60);
		double yawBefore = torpedo.yawRate();
		double pitchBefore = torpedo.pitchRate();
		output.setRudder(-1);
		output.setSternPlanes(-1);
		advance(torpedo, DT);
		assertEquals(1 - 1.5 * DT, torpedo.actualRudder(), 1e-12);
		assertEquals(1 - 1.5 * DT, torpedo.actualSternPlanes(), 1e-12);
		assertTrue(torpedo.yawRate() > yawBefore * 0.99, "Command reversal must not instantly reverse angular motion");
		assertTrue(torpedo.pitchRate() > pitchBefore * 0.99,
				"Command reversal must not instantly reverse angular motion");

		advance(torpedo, 0.68);
		assertTrue(torpedo.actualRudder() < 0 && torpedo.actualSternPlanes() < 0,
				"The fins must have crossed neutral before checking rotational inertia");
		assertTrue(torpedo.yawRate() > 0 && torpedo.pitchRate() > 0,
				"Angular motion must persist briefly after the fins reverse direction");

		advance(torpedo, 60);
		assertEquals(-1, torpedo.actualRudder(), 0);
		assertEquals(-1, torpedo.actualSternPlanes(), 0);
		assertTrue(torpedo.yawRate() < -yawBefore * 0.95);
		assertTrue(torpedo.pitchRate() < -pitchBefore * 0.95);
		assertTrue(Double.isFinite(torpedo.heading()) && Double.isFinite(torpedo.pitch())
				&& Double.isFinite(torpedo.yawRate()) && Double.isFinite(torpedo.pitchRate())
				&& Double.isFinite(torpedo.x()) && Double.isFinite(torpedo.y()) && Double.isFinite(torpedo.z()));
	}

	private static TorpedoEntity settled(double speed, double rudderAngle, double planesAngle) {
		var torpedo = fixedSpeed(speed, rudderAngle, planesAngle);
		advance(torpedo, 60);
		assertEquals(speed, torpedo.speed(), 1e-10, "The fixture must maintain the requested speed");
		return torpedo;
	}

	private static TorpedoEntity fixedSpeed(double speed, double rudderAngle, double planesAngle) {
		var config = VehicleConfig.torpedo();
		var torpedo = new TorpedoEntity(9000, 0, config, null, new Vec3(0, 0, -5000), 0, 0, 20, Color.RED);
		double throttle = config.dragCoeff() * speed * speed / config.maxThrust();
		double rudder = rudderAngle / (Math.PI / 4);
		double planes = planesAngle / (Math.PI / 4);
		var output = torpedo.createOutput();
		output.setThrottle(throttle);
		output.setRudder(rudder);
		output.setSternPlanes(planes);
		torpedo.setSpeed(speed);
		torpedo.setActualThrottle(throttle);
		torpedo.setActualRudder(rudder);
		torpedo.setActualSternPlanes(planes);
		return torpedo;
	}

	private static void advance(TorpedoEntity torpedo, double seconds) {
		double speed = torpedo.speed();
		for (int tick = 0; tick < Math.round(seconds / DT); tick++) {
			// Isolate fin authority at fixed flow speed, away from pitch stops, while keeping
			// angular momentum. Powered/coasting speed loss is exercised separately above.
			torpedo.setPitch(0);
			PHYSICS.step(torpedo, DT, null, null, null);
			torpedo.setSpeed(speed);
		}
	}
}
