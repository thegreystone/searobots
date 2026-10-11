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
import se.hirt.searobots.api.TerrainMap;
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

	static ContactImpulse prepare(
		SubmarineEntity a, SubmarineEntity b, HullOverlap.Contact contact, CurrentField currents) {
		// Separate support points would add spurious torque when the hulls overlap;
		// apply the opposite impulses at one shared physical contact point.
		var point = contact.pointA().add(contact.pointB()).scale(0.5);
		var offsetA = point.subtract(a.pose().position());
		var offsetB = point.subtract(b.pose().position());
		return new ContactImpulse(a, b, offsetA, offsetB, contact.normal(), currents);
	}

	/** Accumulated impulses prevent every solver iteration from becoming another impact. */
	static final class ContactImpulse {
		private final SubmarineEntity a, b;
		private final Vec3 offsetA, offsetB, normal;
		private final CurrentField currents;
		private final double inverseMass;
		private double compressionImpulse;
		private double restitutionImpulse;
		private double recoveryImpulse;

		private ContactImpulse(SubmarineEntity a, SubmarineEntity b, Vec3 offsetA, Vec3 offsetB, Vec3 normal,
				CurrentField currents) {
			this.a = a;
			this.b = b;
			this.offsetA = offsetA;
			this.offsetB = offsetB;
			this.normal = normal;
			this.currents = currents;
			inverseMass = inverseContactMass(a, offsetA, normal) + inverseContactMass(b, offsetB, normal);
		}

		/** Returns the residual normal velocity, measured before this constraint is projected. */
		double compress(boolean recovery) {
			double closing = contactVelocity(a, offsetA, currents).subtract(contactVelocity(b, offsetB, currents))
					.dot(normal);
			if (inverseMass <= 0) {
				return 0;
			}
			double accumulated = recovery ? recoveryImpulse : compressionImpulse;
			double next = Math.max(0, accumulated + closing / inverseMass);
			apply(next - accumulated);
			if (recovery) {
				recoveryImpulse = next;
			} else {
				compressionImpulse = next;
			}
			// A separating contact with no impulse is already satisfied.
			return accumulated > 0 || next > 0 ? Math.abs(closing) : Math.max(0, closing);
		}

		void rebound() {
			// Poisson restitution scales the fully converged compression impulse. Unlike
			// independent per-contact velocity targets, this cannot add net impact energy.
			restitutionImpulse = RESTITUTION * compressionImpulse;
			apply(restitutionImpulse);
		}

		void damage() {
			// Convert the final contact load back to the isolated-impact closing speed.
			// This retains 5*v^2 damage for two hulls and also counts loads transmitted
			// through an initially stationary hull in a simultaneous contact chain.
			double speed = (compressionImpulse + restitutionImpulse + recoveryImpulse) * inverseMass
					/ (1 + RESTITUTION);
			if (speed <= 1e-9) {
				return;
			}
			int damage = Math.max(1, (int) (DAMAGE_FACTOR * speed * speed + 1e-8));
			if (a.hp() > 0) {
				a.setHp(Math.max(0, a.hp() - damage));
			}
			if (b.hp() > 0) {
				b.setHp(Math.max(0, b.hp() - damage));
			}
		}

		private void apply(double magnitude) {
			if (magnitude == 0) {
				return;
			}
			var impulse = normal.scale(magnitude);
			applyImpulse(a, offsetA, impulse.scale(-1));
			applyImpulse(b, offsetB, impulse);
		}
	}

	/** Position correction is independent of impact velocity and may be repeated without damage. */
	static void separate(SubmarineEntity a, SubmarineEntity b, HullOverlap.Contact contact, TerrainMap terrain) {
		var normal = contact.normal();
		double remaining = contact.penetration() + SEPARATION_SLOP;
		for (int i = 0; i < 4 && remaining > 1e-10; i++) {
			var mobilityA = separationMobility(a, normal.scale(-1), terrain);
			var mobilityB = separationMobility(b, normal, terrain);
			double mobility = mobilityB.subtract(mobilityA).dot(normal);
			if (mobility <= 0) {
				break;
			}
			var previousA = a.pose().position();
			var previousB = b.pose().position();
			double correction = remaining / mobility;
			translate(a, mobilityA.scale(correction), terrain);
			translate(b, mobilityB.scale(correction), terrain);
			remaining -= b.pose().position().subtract(previousB).subtract(a.pose().position().subtract(previousA))
					.dot(normal);
		}
		if (remaining > 1e-8 && terrain != null && HullOverlap.overlaps(a, b)) {
			// A hull caught between the seabed and the surface cannot separate vertically.
			// Use a horizontal support plane rather than repeatedly pushing through the floor.
			var horizontal = HullOverlap.horizontalContact(a, b);
			var mobilityA = linearMobility(a, horizontal.normal().scale(-1));
			var mobilityB = linearMobility(b, horizontal.normal());
			double correction = (horizontal.penetration() + SEPARATION_SLOP)
					/ mobilityB.subtract(mobilityA).dot(horizontal.normal());
			translate(a, mobilityA.scale(correction), terrain);
			translate(b, mobilityB.scale(correction), terrain);
		}
	}

	private static Vec3 right(SubmarineEntity sub) {
		return new Vec3(Math.cos(sub.heading()), -Math.sin(sub.heading()), 0);
	}

	static Vec3 contactVelocity(SubmarineEntity sub, Vec3 offset, CurrentField currents) {
		var current = currents.currentAt(sub.z());
		var linear = sub.velocity().linear();
		if (sub.surfaceConstrained()) {
			linear = new Vec3(linear.x(), linear.y(), 0);
		}
		// Heading increases clockwise from north; physical world-Z rotation has the opposite sign.
		var angular = right(sub).scale(sub.surfaceConstrained() ? 0 : sub.pitchRate())
				.add(new Vec3(0, 0, -sub.yawRate()));
		return linear.add(new Vec3(current.x(), current.y(), 0)).add(angular.cross(offset));
	}

	private static Vec3 linearMobility(SubmarineEntity sub, Vec3 direction) {
		// Impacts exchange physical vessel momentum. Hydrodynamic added masses remain
		// in the subsequent thrust/drag integration, rather than creating direction-dependent
		// unequal impulses between the two hulls.
		return new Vec3(direction.x(), direction.y(), sub.surfaceConstrained() ? 0 : direction.z())
				.scale(1 / sub.vehicleConfig().dryMass());
	}

	private static double yawInertia(SubmarineEntity sub) {
		var cfg = sub.vehicleConfig();
		double cosP = Math.cos(sub.pitch()), sinP = Math.sin(sub.pitch());
		return cfg.collisionYawInertia() * cosP * cosP + cfg.collisionRollInertia() * sinP * sinP;
	}

	private static Vec3 separationMobility(SubmarineEntity sub, Vec3 direction, TerrainMap terrain) {
		var result = linearMobility(sub, direction);
		// If one hull reaches the water surface, put the remainder of the correction
		// into the other available directions rather than leaving the pair interpenetrating.
		if (sub.z() >= 0 && result.z() > 0) {
			return new Vec3(result.x(), result.y(), 0);
		}
		if (terrain != null && result.z() < 0) {
			var below = sub.pose().position().add(new Vec3(0, 0, -1e-7));
			if (HullGeometry.terrainPenetration(below, sub.heading(), sub.pitch(), sub.vehicleConfig(), terrain) > 0) {
				return new Vec3(result.x(), result.y(), 0);
			}
		}
		return result;
	}

	static double inverseContactMass(SubmarineEntity sub, Vec3 offset, Vec3 normal) {
		var lever = offset.cross(normal);
		double result = linearMobility(sub, normal).dot(normal) + lever.z() * lever.z() / yawInertia(sub);
		if (!sub.surfaceConstrained()) {
			double pitchLever = lever.dot(right(sub));
			result += pitchLever * pitchLever / sub.vehicleConfig().collisionPitchInertia();
		}
		return result;
	}

	static void applyImpulse(SubmarineEntity sub, Vec3 offset, Vec3 impulse) {
		setLinearVelocity(sub, sub.velocity().linear().add(linearMobility(sub, impulse)));
		var torque = offset.cross(impulse);
		sub.setYawRate(sub.yawRate() - torque.z() / yawInertia(sub));
		if (!sub.surfaceConstrained()) {
			sub.setPitchRate(sub.pitchRate() + torque.dot(right(sub)) / sub.vehicleConfig().collisionPitchInertia());
		}
	}

	/** Retains world velocity when collision detection rewinds the hull's orientation. */
	static void setLinearVelocity(SubmarineEntity sub, Vec3 velocity) {
		if (sub.z() >= 0 && velocity.z() > 0) {
			// The surface reaction removes upward motion instead of leaving hidden
			// heave in the actuator state to reappear when the hull dives next tick.
			velocity = new Vec3(velocity.x(), velocity.y(), 0);
		}
		double sinH = Math.sin(sub.heading()), cosH = Math.cos(sub.heading());
		double speed = (velocity.x() * sinH + velocity.y() * cosH) / Math.cos(sub.pitch());
		sub.setSpeed(speed);
		sub.setSwaySpeed(velocity.x() * cosH - velocity.y() * sinH);
		sub.setVerticalSpeed(sub.surfaceConstrained() ? 0 : velocity.z() - speed * Math.sin(sub.pitch()));
	}

	private static void translate(SubmarineEntity sub, Vec3 displacement, TerrainMap terrain) {
		sub.setX(sub.x() + displacement.x());
		sub.setY(sub.y() + displacement.y());
		sub.setZ(sub.surfaceConstrained() ? 0 : Math.min(Math.max(0, sub.z()), sub.z() + displacement.z()));
		if (terrain != null) {
			sub.setZ(sub.z() + Math.max(0, HullGeometry.terrainPenetration(sub.pose().position(), sub.heading(),
					sub.pitch(), sub.vehicleConfig(), terrain)));
		}
	}
}
