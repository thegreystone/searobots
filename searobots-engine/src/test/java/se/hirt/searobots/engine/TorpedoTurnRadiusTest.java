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

/**
 * Guards maneuverability at stable speed, including the smallest radius available near stall. Pitch
 * rates are measured directly because accumulated pitch saturates at the 60-degree limit.
 */
class TorpedoTurnRadiusTest {

	private static final double DT = 1.0 / 50;
	private static final double[] FAST_CONTROL_ANGLES = {15, 25, 29.9, 30, 30.1, 35, 40, 45};

	@Test
	void optimumRudderTurnRadiusIncreasesWithSpeed() {
		double slow = yawRadius(settled(5, 30, 0));
		double fast = yawRadius(settled(20, 30, 0));
		double maximum = yawRadius(settled(23, 30, 0));
		assertTrue(slow >= 40 && slow <= 50, "5 m/s should permit a roughly 44 m turn, got " + slow);
		assertTrue(fast >= 150 && fast <= 170, "20 m/s should need roughly 160 m to turn, got " + fast);
		assertTrue(maximum >= 175 && maximum <= 195, "23 m/s should need roughly 184 m to turn, got " + maximum);
		assertTrue(slow < fast && fast < maximum, "Higher speed must require a wider turn");
	}

	@Test
	void fullRudderTurnRadiusIncreasesWithSpeedAndExceedsOptimum() {
		double previous = 0;
		for (double speed : new double[] {5, 20, 23}) {
			double optimum = yawRadius(settled(speed, 30, 0));
			double full = yawRadius(settled(speed, 45, 0));
			assertTrue(full > optimum * 1.5 && full < optimum * 1.85,
					"Stalled full rudder must still turn, but need a wider radius at " + speed + " m/s");
			assertTrue(full > previous, "Full-rudder turns must also widen with speed");
			previous = full;
		}
	}

	@Test
	void fastTorpedoCannotTurnTightlyByChoosingAnotherRudderAngle() {
		for (double angle : FAST_CONTROL_ANGLES) {
			double radius = yawRadius(settled(23, angle, 0));
			assertTrue(radius >= 175,
					"23 m/s needs at least a 175 m yaw radius at " + angle + " degrees, got " + radius);
		}
	}

	@Test
	void fasterPitchResponseStillHasAWideRadiusAtMaximumSpeed() {
		for (double angle : FAST_CONTROL_ANGLES) {
			var torpedo = settled(23, 0, angle);
			double radius = torpedo.speed() / Math.abs(torpedo.pitchRate());
			assertTrue(radius >= 80,
					"23 m/s needs at least an 80 m pitch radius at " + angle + " degrees, got " + radius);
		}
	}

	@Test
	void combiningYawAndPitchCannotProduceAnInstantTurnAtMaximumSpeed() {
		for (double angle : FAST_CONTROL_ANGLES) {
			var torpedo = settled(23, angle, angle);
			// Omitting the cos(pitch) factor overestimates yaw curvature, so this is a conservative bound.
			double radius = torpedo.speed() / Math.hypot(torpedo.yawRate(), torpedo.pitchRate());
			assertTrue(radius >= 70, "Combined steering at 23 m/s needs at least a 70 m radius, got " + radius);
		}
	}

	private static double yawRadius(TorpedoEntity torpedo) {
		return torpedo.speed() / Math.abs(torpedo.yawRate());
	}

	private static TorpedoEntity settled(double speed, double rudderDegrees, double planesDegrees) {
		var config = VehicleConfig.torpedo();
		var torpedo = new TorpedoEntity(9000, 0, config, null, new Vec3(0, 0, -5000), 0, 0, 20, Color.RED);
		double throttle = config.dragCoeff() * speed * speed / config.maxThrust();
		double rudder = rudderDegrees / 45;
		double planes = planesDegrees / 45;
		var output = torpedo.createOutput();
		output.setThrottle(throttle);
		output.setRudder(rudder);
		output.setSternPlanes(planes);
		torpedo.setSpeed(speed);
		torpedo.setActualThrottle(throttle);
		torpedo.setActualRudder(rudder);
		torpedo.setActualSternPlanes(planes);
		var physics = new TorpedoPhysics();
		for (int tick = 0; tick < Math.round(60 / DT); tick++) {
			physics.step(torpedo, DT, null, null, null);
		}
		assertEquals(speed, torpedo.speed(), 1e-10, "The fixture must maintain the requested speed");
		assertTrue(torpedo.alive() && torpedo.fuelRemaining() > 0,
				"The fixture must measure a powered torpedo rather than a stopped or destroyed one");
		return torpedo;
	}
}
