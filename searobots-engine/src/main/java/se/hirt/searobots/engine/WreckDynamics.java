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

/** Passive flooding and drag for destroyed hulls; living vehicle configuration is unchanged. */
final class WreckDynamics {

	private static final double FLOOD_TIME_CONSTANT = 20;
	private static final double SHIP_LOST_BUOYANCY_FRACTION = 0.03;
	private static final double SUBMARINE_LOST_BUOYANCY_FRACTION = 0.015;
	private static final double WATER_DENSITY = 1025;

	private WreckDynamics() {
	}

	/** Advances flooding once per physics tick and returns additional vertical force in newtons. */
	static double advance(SubmarineEntity sub, double dt) {
		if (sub.hp() > 0 || sub.forfeited()) {
			return 0;
		}

		// Destruction removes propulsion immediately, including any throttle slew state.
		// Neutral control surfaces may still generate passive hull drag and restoring forces.
		sub.setThrottle(0);
		sub.setActualThrottle(0);
		sub.setEngineClutch(false);
		sub.setRudder(0);
		sub.setActualRudder(0);
		sub.setSternPlanes(0);
		sub.setActualSternPlanes(0);
		sub.setBallast(0);

		double fraction = 1 - (1 - sub.wreckFloodFraction()) * Math.exp(-Math.max(0, dt) / FLOOD_TIME_CONSTANT);
		sub.setWreckFloodFraction(fraction);
		// Flooded submarine ballast supplies the initial sinking force. Continued hull flooding
		// adds a smaller imbalance over tens of seconds; passive water drag bounds the descent.
		double lostBuoyancy = sub.vehicleConfig().surfaceLocked() ? SHIP_LOST_BUOYANCY_FRACTION
				: SUBMARINE_LOST_BUOYANCY_FRACTION;
		return -lostBuoyancy * sub.vehicleConfig().dryMass() * 9.81 * fraction;
	}

	static double verticalDragCoefficient(SubmarineEntity sub) {
		var cfg = sub.vehicleConfig();
		if (sub.hp() <= 0 && cfg.surfaceLocked()) {
			// The live ship model needs no heave drag while locked to the surface. Once sunk,
			// its broad horizontal area resists descent and bounds its terminal sinking speed.
			double horizontalArea = 4 * cfg.hullHalfLength() * cfg.hullHalfBeam();
			return Math.max(cfg.verticalDragCoeff(), 0.5 * WATER_DENSITY * 1.2 * horizontalArea);
		}
		return cfg.verticalDragCoeff();
	}
}
