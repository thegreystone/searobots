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
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class TerrainContactResponseTest {

	private static final CurrentField STILL = new CurrentField(List.of());

	@Test
	void centredKeelImpactPreservesTangentialMomentumAndReturnsFivePercentNormalSpeed() {
		for (double descent : new double[] {0.01, 0.1, 2, 8}) {
			var sub = submarine(new Vec3(0, 0, -95));
			var incoming = new Vec3(3, 4, -descent);
			SubCollisionResponse.setLinearVelocity(sub, incoming);
			TerrainContactResponse.apply(sub, sub.pose().position(), new Vec3[] {new Vec3(0, 0, -5)}, incoming, 0, 0,
					terrain((x, y) -> -100), STILL);
			var actual = sub.velocity().linear();
			assertEquals(3, actual.x(), 1e-9);
			assertEquals(4, actual.y(), 1e-9);
			assertEquals(0.05 * descent, actual.z(), 1e-9);
			assertEquals(0, sub.yawRate(), 1e-9);
			assertEquals(0, sub.pitchRate(), 1e-9);
			assertTrue(actual.dot(actual) < incoming.dot(incoming), "An inelastic impact must lose kinetic energy.");
		}
	}

	@Test
	void slopeImpactRemovesNormalMotionWithoutChangingThePlaneTangent() {
		var normal = new Vec3(-1, 0, 1).normalize();
		var tangent = new Vec3(1, 0, 1).normalize();
		var sub = submarine(new Vec3(0, 0, -100).add(normal.scale(5)));
		var incoming = tangent.scale(3).add(normal.scale(-2));
		SubCollisionResponse.setLinearVelocity(sub, incoming);
		TerrainContactResponse.apply(sub, sub.pose().position(), new Vec3[] {normal.scale(-5)}, incoming, 0, 0,
				terrain((x, y) -> -100 + x), STILL);
		var actual = sub.velocity().linear();
		assertEquals(3, actual.dot(tangent), 1e-9);
		assertEquals(0.1, actual.dot(normal), 1e-9);
		assertEquals(0, sub.pitchRate(), 1e-9, "A normal through the centre of mass must not generate pitch torque.");
	}

	@Test
	void sharpCrestUsesItsIncomingCellSlopeRatherThanAnAveragedNormal() {
		var terrain = terrain((x, y) -> y == 0 ? -100 : -120);
		var point = new Vec3(0, -0.01, -100.02);
		var normal = new Vec3(0, -2, 1).normalize();
		var sub = submarine(point.add(normal.scale(5)));
		var incoming = new Vec3(0, 3, 0);
		SubCollisionResponse.setLinearVelocity(sub, incoming);
		TerrainContactResponse.apply(sub, sub.pose().position(), new Vec3[] {normal.scale(-5)}, incoming, 0, 0, terrain,
				STILL);
		assertEquals(-0.05 * incoming.dot(normal), sub.velocity().linear().dot(normal), 1e-9);
		assertTrue(sub.speed() < 1, "The steep flank must remove most forward motion.");
		assertEquals(0, sub.pitchRate(), 1e-9);
	}

	@Test
	void offCentreImpactRotatesTheHullAndDissipatesTotalEnergy() {
		var sub = submarine(new Vec3(0, 0, -95));
		var incoming = new Vec3(0, 0, -2);
		SubCollisionResponse.setLinearVelocity(sub, incoming);
		double before = kineticEnergy(sub);
		var point = new Vec3(0, 30, -5);
		TerrainContactResponse.apply(sub, sub.pose().position(), new Vec3[] {point}, incoming, 0, 0,
				terrain((x, y) -> -100), STILL);
		assertTrue(sub.pitchRate() > 0);
		assertEquals(0, sub.pitch(), 1e-12, "An impulse changes angular velocity, not orientation.");
		assertTrue(SubCollisionResponse.contactVelocity(sub, point, STILL).z() > 0);
		assertTrue(kineticEnergy(sub) < before);
	}

	@Test
	void wreckSettlesWithoutReboundOrLossOfTangentialVelocity() {
		var sub = submarine(new Vec3(0, 0, -95));
		sub.setHp(0);
		var incoming = new Vec3(1, 2, -0.01);
		SubCollisionResponse.setLinearVelocity(sub, incoming);
		TerrainContactResponse.apply(sub, sub.pose().position(), new Vec3[] {new Vec3(0, 0, -5)}, incoming, 0, 0,
				terrain((x, y) -> -100), STILL);
		assertEquals(new Vec3(1, 2, 0), sub.velocity().linear());
		assertEquals(0, sub.hp());
	}

	private static double kineticEnergy(SubmarineEntity sub) {
		var velocity = sub.velocity().linear();
		return 0.5 * sub.vehicleConfig().dryMass() * velocity.dot(velocity)
				+ 0.5 * sub.vehicleConfig().collisionPitchInertia() * sub.pitchRate() * sub.pitchRate()
				+ 0.5 * sub.vehicleConfig().collisionYawInertia() * sub.yawRate() * sub.yawRate();
	}

	private static SubmarineEntity submarine(Vec3 position) {
		return new SubmarineEntity(VehicleConfig.submarine(), 0, null, position, 0, Color.BLUE, 1000);
	}

	private static TerrainMap terrain(java.util.function.DoubleBinaryOperator elevation) {
		double[] elevations = new double[441];
		for (int row = 0; row < 21; row++) {
			for (int col = 0; col < 21; col++) {
				elevations[row * 21 + col] = elevation.applyAsDouble(-100 + col * 10, -100 + row * 10);
			}
		}
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}
}
