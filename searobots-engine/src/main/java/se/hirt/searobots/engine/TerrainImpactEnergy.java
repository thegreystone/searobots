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

import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

/** Incident normal contact energy; the existing bounce response remains a simplified model. */
final class TerrainImpactEnergy {

	// A fixed calibration mass makes heavier vehicles hurt more at equal impact speed.
	// With the default factor 5, 1 MJ costs 4 HP.
	private static final double REFERENCE_MASS_KG = 2_500_000;

	private TerrainImpactEnergy() {
	}

	/**
	 * Normal impact energy in joules, using m_eff = 1 / (J M^-1 J^T). The normal must be a unit
	 * vector; the offset is from the centre of mass in world axes. Translation includes directional
	 * added-water mass; pitch and yaw include contact lever arms and geometric hull moments. Roll
	 * is locked in this engine. Surface-locked vehicles also cannot relieve impact by heave or
	 * pitch. This is a single-contact energy estimate, not a coupled contact impulse solver.
	 */
	static double energyJoules(
		VehicleConfig cfg, Vec3 contactOffset, Vec3 normal, double heading, double pitch, double closingSpeed) {
		if (closingSpeed <= 0) {
			return 0;
		}
		double sinH = Math.sin(heading), cosH = Math.cos(heading);
		double sinP = Math.sin(pitch), cosP = Math.cos(pitch);
		var forward = new Vec3(sinH * cosP, cosH * cosP, sinP);
		var right = new Vec3(cosH, -sinH, 0);
		var up = new Vec3(-sinH * sinP, -cosH * sinP, cosP);
		double surge = normal.dot(forward);
		double sway = normal.dot(right);
		double heave = normal.dot(up);
		double inverseMass = surge * surge / cfg.massSurge() + sway * sway / cfg.massSway();
		var lever = contactOffset.cross(normal);
		double pitchLever = lever.dot(right);
		double yawLever = lever.z();
		// Heading changes rotate about world Z, which mixes the hull's principal yaw
		// and longitudinal moments when pitched. There is still no free roll degree of freedom.
		double yawInertia = cfg.collisionYawInertia() * cosP * cosP + cfg.collisionRollInertia() * sinP * sinP;
		inverseMass += yawLever * yawLever / yawInertia;
		if (!cfg.surfaceLocked()) {
			inverseMass += heave * heave / cfg.massHeave() + pitchLever * pitchLever / cfg.collisionPitchInertia();
		}
		// A wholly locked contact direction cannot have physical incoming motion.
		if (inverseMass <= 0) {
			return 0;
		}
		return 0.5 * closingSpeed * closingSpeed / inverseMass;
	}

	static int damage(double energyJoules, double damageFactor) {
		return (int) Math.max(0, 2 * damageFactor * energyJoules / REFERENCE_MASS_KG);
	}
}
