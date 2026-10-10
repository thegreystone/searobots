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

import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.Vec3;

/**
 * A frictionless, inelastic hull impact in the engine's supported translation, pitch and yaw axes.
 */
final class SubCollisionResponse {

	private static final double RESTITUTION = 0.1;
	private static final double DAMAGE_FACTOR = 5.0;
	private static final double SEPARATION_SLOP = 1e-4;

	private SubCollisionResponse() {
	}

	static void resolve(SubmarineEntity a, SubmarineEntity b, HullOverlap.Contact contact, CurrentField currents) {
		var normal = contact.normal();
		// Separate support points would add spurious torque when the hulls overlap;
		// apply the opposite impulses at one shared physical contact point.
		var point = contact.pointA().add(contact.pointB()).scale(0.5);
		var offsetA = point.subtract(a.pose().position());
		var offsetB = point.subtract(b.pose().position());
		double closingSpeed = contactVelocity(a, offsetA, currents).subtract(contactVelocity(b, offsetB, currents))
				.dot(normal);
		double inverseMass = inverseContactMass(a, offsetA, normal) + inverseContactMass(b, offsetB, normal);
		if (closingSpeed > 1e-9 && inverseMass > 0) {
			double magnitude = (1 + RESTITUTION) * closingSpeed / inverseMass;
			var impulse = normal.scale(magnitude);
			applyImpulse(a, offsetA, impulse.scale(-1));
			applyImpulse(b, offsetB, impulse);

			// Retain the game's ramming damage calibration, but use the actual normal
			// closing speed at the contact instead of centre-line or tangential speed.
			int damage = Math.max(1, (int) (DAMAGE_FACTOR * closingSpeed * closingSpeed));
			a.setHp(Math.max(0, a.hp() - damage));
			b.setHp(Math.max(0, b.hp() - damage));
		}

		// Resolve penetration even for stationary or separating hulls, without adding energy.
		double remaining = contact.penetration() + SEPARATION_SLOP;
		for (int i = 0; i < 2 && remaining > 1e-10; i++) {
			var mobilityA = separationMobility(a, normal.scale(-1));
			var mobilityB = separationMobility(b, normal);
			double mobility = mobilityB.subtract(mobilityA).dot(normal);
			if (mobility <= 0) {
				break;
			}
			var previousA = a.pose().position();
			var previousB = b.pose().position();
			double correction = remaining / mobility;
			translate(a, mobilityA.scale(correction));
			translate(b, mobilityB.scale(correction));
			remaining -= b.pose().position().subtract(previousB).subtract(a.pose().position().subtract(previousA))
					.dot(normal);
		}
	}

	private static Vec3 right(SubmarineEntity sub) {
		return new Vec3(Math.cos(sub.heading()), -Math.sin(sub.heading()), 0);
	}

	private static Vec3 contactVelocity(SubmarineEntity sub, Vec3 offset, CurrentField currents) {
		var current = currents.currentAt(sub.z());
		var linear = sub.velocity().linear();
		if (sub.vehicleConfig().surfaceLocked()) {
			linear = new Vec3(linear.x(), linear.y(), 0);
		}
		// Heading increases clockwise from north; physical world-Z rotation has the opposite sign.
		var angular = right(sub).scale(sub.vehicleConfig().surfaceLocked() ? 0 : sub.pitchRate())
				.add(new Vec3(0, 0, -sub.yawRate()));
		return linear.add(new Vec3(current.x(), current.y(), 0)).add(angular.cross(offset));
	}

	private static Vec3 linearMobility(SubmarineEntity sub, Vec3 direction) {
		// Impacts exchange physical vessel momentum. Hydrodynamic added masses remain
		// in the subsequent thrust/drag integration, rather than creating direction-dependent
		// unequal impulses between the two hulls.
		return new Vec3(direction.x(), direction.y(), sub.vehicleConfig().surfaceLocked() ? 0 : direction.z())
				.scale(1 / sub.vehicleConfig().dryMass());
	}

	private static double yawInertia(SubmarineEntity sub) {
		var cfg = sub.vehicleConfig();
		double cosP = Math.cos(sub.pitch()), sinP = Math.sin(sub.pitch());
		return cfg.collisionYawInertia() * cosP * cosP + cfg.collisionRollInertia() * sinP * sinP;
	}

	private static Vec3 separationMobility(SubmarineEntity sub, Vec3 direction) {
		var result = linearMobility(sub, direction);
		// If one hull reaches the water surface, put the remainder of the correction
		// into the other available directions rather than leaving the pair interpenetrating.
		return sub.z() >= 0 && result.z() > 0 ? new Vec3(result.x(), result.y(), 0) : result;
	}

	private static double inverseContactMass(SubmarineEntity sub, Vec3 offset, Vec3 normal) {
		var lever = offset.cross(normal);
		double result = linearMobility(sub, normal).dot(normal) + lever.z() * lever.z() / yawInertia(sub);
		if (!sub.vehicleConfig().surfaceLocked()) {
			double pitchLever = lever.dot(right(sub));
			result += pitchLever * pitchLever / sub.vehicleConfig().collisionPitchInertia();
		}
		return result;
	}

	private static void applyImpulse(SubmarineEntity sub, Vec3 offset, Vec3 impulse) {
		var velocity = sub.velocity().linear().add(linearMobility(sub, impulse));
		double sinH = Math.sin(sub.heading()), cosH = Math.cos(sub.heading());
		double speed = (velocity.x() * sinH + velocity.y() * cosH) / Math.cos(sub.pitch());
		sub.setSpeed(speed);
		sub.setSwaySpeed(velocity.x() * cosH - velocity.y() * sinH);
		sub.setVerticalSpeed(sub.vehicleConfig().surfaceLocked() ? 0 : velocity.z() - speed * Math.sin(sub.pitch()));
		var torque = offset.cross(impulse);
		sub.setYawRate(sub.yawRate() - torque.z() / yawInertia(sub));
		if (!sub.vehicleConfig().surfaceLocked()) {
			sub.setPitchRate(sub.pitchRate() + torque.dot(right(sub)) / sub.vehicleConfig().collisionPitchInertia());
		}
	}

	private static void translate(SubmarineEntity sub, Vec3 displacement) {
		sub.setX(sub.x() + displacement.x());
		sub.setY(sub.y() + displacement.y());
		sub.setZ(sub.vehicleConfig().surfaceLocked() ? 0 : Math.min(Math.max(0, sub.z()), sub.z() + displacement.z()));
	}
}
