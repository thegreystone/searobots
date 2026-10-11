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

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Resolves simultaneous hull velocities and hull, seabed and surface positional constraints. */
final class HullContactSolver {

	private static final int MAX_ITERATIONS = 128;
	private static final int MAX_VELOCITY_ITERATIONS = 256;
	private static final double VELOCITY_TOLERANCE = 1e-9;

	private HullContactSolver() {
	}

	static void resolve(List<SubmarineEntity> entities, CurrentField currents, TerrainMap terrain) {
		// A destroyed vessel remains a finite-mass physical hull. Identity ordering makes
		// both iterative solvers independent of the simulation's entity iteration order.
		var active = entities.stream().filter(sub -> !sub.forfeited())
				.sorted(Comparator.comparingInt(SubmarineEntity::id)).toList();
		var impacts = new ArrayList<SubCollisionResponse.ContactImpulse>();
		var recorded = new boolean[active.size()][active.size()];
		for (int i = 0; i < active.size(); i++) {
			for (int j = i + 1; j < active.size(); j++) {
				var a = active.get(i);
				var b = active.get(j);
				var contact = HullOverlap.contact(a, b);
				if (contact != null) {
					impacts.add(SubCollisionResponse.prepare(a, b, contact, currents));
					recorded[i][j] = true;
				}
			}
		}
		if (terrain != null) {
			for (var sub : active) {
				sub.setZ(sub.z() + Math.max(0, HullGeometry.terrainPenetration(sub.pose().position(), sub.heading(),
						sub.pitch(), sub.vehicleConfig(), terrain)));
			}
		}
		for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
			boolean contactFound = false;
			for (int i = 0; i < active.size(); i++) {
				for (int j = i + 1; j < active.size(); j++) {
					var a = active.get(i);
					var b = active.get(j);
					var contact = HullOverlap.contact(a, b);
					if (contact != null) {
						contactFound = true;
						if (!recorded[i][j]) {
							// Resolving one overlap can close another gap. Retain the first
							// physical contact offsets before projection moves the hulls apart.
							impacts.add(SubCollisionResponse.prepare(a, b, contact, currents));
							recorded[i][j] = true;
						}
						SubCollisionResponse.separate(a, b, contact, terrain);
					}
				}
			}
			if (!contactFound) {
				break;
			}
		}
		// Solve every contact together, including those discovered during projection.
		// First converge compression, then return a small proportion of its stored impulse;
		// the final inelastic projection handles contacts reactivated by that rebound.
		compress(active, impacts, currents, terrain, false);
		for (var impact : impacts) {
			impact.rebound();
		}
		compress(active, impacts, currents, terrain, true);
		for (var impact : impacts) {
			impact.damage();
		}
	}

	private static void compress(
		List<SubmarineEntity> active, List<SubCollisionResponse.ContactImpulse> impacts, CurrentField currents,
		TerrainMap terrain, boolean recovery) {
		if (impacts.isEmpty()) {
			// Individual physics already resolved terrain velocity; only a hull impact
			// can reactivate it here. Avoid repeating expensive geometry queries every tick.
			return;
		}
		for (int iteration = 0; iteration < MAX_VELOCITY_ITERATIONS; iteration++) {
			double residual = 0;
			for (var impact : impacts) {
				residual = Math.max(residual, impact.compress(recovery));
			}
			if (terrain != null) {
				for (var sub : active) {
					residual = Math.max(residual, TerrainContactResponse.projectVelocity(sub, terrain, currents));
				}
			}
			if (residual <= VELOCITY_TOLERANCE) {
				break;
			}
		}
	}
}
