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

import java.util.List;

/** Resolves hull, seabed and surface positional constraints together after motion integration. */
final class HullContactSolver {

	private static final int MAX_ITERATIONS = 128;

	private HullContactSolver() {
	}

	static void resolve(List<SubmarineEntity> entities, CurrentField currents, TerrainMap terrain) {
		// Retain hulls killed by this tick's impact until their contact is separated.
		// Wrecks from previous ticks remain excluded, as in the original collision handling.
		var active = entities.stream().filter(sub -> !sub.forfeited() && sub.hp() > 0).toList();
		// Only contacts produced by motion receive an impact. Iterative position corrections
		// can create another overlap, but that overlap is not another physical collision.
		for (int i = 0; i < active.size(); i++) {
			for (int j = i + 1; j < active.size(); j++) {
				var a = active.get(i);
				var b = active.get(j);
				var contact = HullOverlap.contact(a, b);
				if (contact != null) {
					SubCollisionResponse.impact(a, b, contact, currents);
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
						SubCollisionResponse.separate(a, b, contact, terrain);
					}
				}
			}
			if (!contactFound) {
				break;
			}
		}
	}
}
