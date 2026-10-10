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

import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

/**
 * Simplified submarine physics based on Fossen's 6-DOF formulation. See docs/physics-model.md for
 * full documentation, references, and characterization results.
 */
public final class SubmarinePhysics {

	private static final double WATER_DENSITY = 1025.0; // kg/m^3 seawater
	private static final double CL_SLOPE = 2 * Math.PI; // thin airfoil theory
	private final HullPressureModel hullPressure;

	public SubmarinePhysics() {
		this(MatchConfig.withDefaults(0));
	}

	/** Creates physics with the match's depth limits and seeded pressure-hull failures. */
	public SubmarinePhysics(MatchConfig config) {
		hullPressure = new HullPressureModel(config);
	}

	/**
	 * Lift coefficient with stall. Linear below stallAngle, smooth rolloff above (drops to ~60% at
	 * full deflection).
	 */
	static double liftCoefficient(double alpha, double stallAngle) {
		double absAlpha = Math.abs(alpha);
		double cl;
		if (absAlpha <= stallAngle) {
			cl = CL_SLOPE * absAlpha;
		} else {
			double clMax = CL_SLOPE * stallAngle;
			double maxDeflection = Math.PI / 4;
			double postStallFraction = (absAlpha - stallAngle) / (maxDeflection - stallAngle);
			cl = clMax * (1.0 - 0.4 * Math.min(postStallFraction, 1.0));
		}
		return Math.copySign(cl, alpha);
	}

	public void step(
		SubmarineEntity sub, double dt, TerrainMap terrain, CurrentField currentField, BattleArea battleArea) {
		if (sub.forfeited())
			return;
		// Dead subs still get physics (sinking to the bottom)

		var cfg = sub.vehicleConfig();
		double previousX = sub.x();
		double previousY = sub.y();
		double previousZ = sub.z();
		double previousHeading = sub.heading();
		double previousPitch = sub.pitch();

		// ── Damage model: HP loss degrades performance ──
		// hpRatio: 1.0 = undamaged, 0.0 = dead
		// damageEffect: gentle at first, severe at low HP.
		// At 75% HP: 94% performance. At 50%: 75%. At 25%: 44%. At 10%: 16%.
		double hpRatio = sub.maxHp() > 0 ? (double) sub.hp() / sub.maxHp() : 1.0;
		double damageEffect = hpRatio * hpRatio; // quadratic: light damage barely matters
		double thrustFactor = 0.3 + 0.7 * damageEffect; // 30-100% thrust
		double controlFactor = 0.4 + 0.6 * damageEffect; // 40-100% rudder/planes
		double damageNoiseDb = (1.0 - hpRatio) * 10.0; // up to +10 dB from hull damage

		// 1. Thrust lag: actual throttle tracks commanded with slew limit
		double commandedThrottle = sub.throttle();
		double actualThrottle = sub.actualThrottle();
		double maxThrottleChange = cfg.thrustSlewRate() * dt;
		if (commandedThrottle > actualThrottle) {
			actualThrottle = Math.min(actualThrottle + maxThrottleChange, commandedThrottle);
		} else if (commandedThrottle < actualThrottle) {
			actualThrottle = Math.max(actualThrottle - maxThrottleChange, commandedThrottle);
		}
		sub.setActualThrottle(actualThrottle);

		// 2. Thrust and drag (with engine clutch mechanic)
		boolean clutchEngaged = sub.engineClutch();
		double thrust;
		if (!clutchEngaged) {
			// Clutch disengaged: prop freewheels, no thrust, no engine braking
			thrust = 0;
		} else if (actualThrottle >= 0) {
			thrust = cfg.maxThrust() * actualThrottle * thrustFactor;
		} else {
			thrust = cfg.maxThrust() * cfg.reverseThrustFactor() * actualThrottle * thrustFactor;
		}
		double speed = sub.speed();
		double drag = cfg.dragCoeff() * speed * Math.abs(speed);
		// Extra drag from windmilling prop when clutch engaged at zero throttle
		if (clutchEngaged && actualThrottle == 0) {
			drag += cfg.dragCoeff() * cfg.propDragFactor() * speed * Math.abs(speed);
		}
		speed += (thrust - drag) / cfg.massSurge() * dt;
		// Cap reverse speed; hull form creates enormous drag going backwards
		if (speed < -cfg.maxReverseSpeed())
			speed = -cfg.maxReverseSpeed();
		sub.setSpeed(speed);

		// 2b. Control surface slew: rudder and planes take time to move
		// Full sweep (-1 to 1) in ~3.5 seconds = slew rate 0.57/s
		double controlSlewRate = 0.57;
		double maxRudderChange = controlSlewRate * dt;
		double actualRudder = sub.actualRudder();
		double commandedRudder = sub.rudder();
		if (commandedRudder > actualRudder) {
			actualRudder = Math.min(actualRudder + maxRudderChange, commandedRudder);
		} else if (commandedRudder < actualRudder) {
			actualRudder = Math.max(actualRudder - maxRudderChange, commandedRudder);
		}
		sub.setActualRudder(actualRudder);

		double actualPlanes = sub.actualSternPlanes();
		double commandedPlanes = sub.sternPlanes();
		if (commandedPlanes > actualPlanes) {
			actualPlanes = Math.min(actualPlanes + maxRudderChange, commandedPlanes);
		} else if (commandedPlanes < actualPlanes) {
			actualPlanes = Math.max(actualPlanes - maxRudderChange, commandedPlanes);
		}
		sub.setActualSternPlanes(actualPlanes);

		// 3. Yaw: first-order filter toward steady-state yaw rate (Option A from Thune thesis)
		// The steady-state yaw rate is what the rudder moment can sustain against rotary damping.
		// The actual yaw rate exponentially approaches this with a time constant tau.
		// This gives realistic transients (gradual buildup, overshoot in zigzag) while
		// remaining unconditionally stable (no oscillation risk).
		double rudderAngle = actualRudder * Math.PI / 4; // -1..1 maps to -45..+45 deg
		double rudderCl = liftCoefficient(rudderAngle, cfg.stallAngle());
		double rudderMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed) * cfg.rudderArea() * rudderCl
				* cfg.rudderArm() * controlFactor;

		// Effective inertia increases with v² (centrifugal resistance at high speed).
		// Higher coefficient = bigger difference between slow-speed and flank-speed turning.
		double baseInertia = cfg.massSurge() * cfg.rotationalInertia();
		double speedDamping = baseInertia * 0.05 * speed * Math.abs(speed);
		double effectiveInertia = baseInertia + speedDamping;

		// Steady-state yaw rate: what the rudder can sustain
		double yawRateSteady = rudderMoment / effectiveInertia;

		// Time constant: how quickly yaw rate responds (larger = more sluggish)
		// Scales with inertia and inversely with speed (faster = quicker response)
		// At patrol speed (~10 m/s): tau ~ 8s. At low speed (~4 m/s): tau ~ 15s.
		double absSpeed = Math.max(Math.abs(speed), 0.5); // avoid division by near-zero
		double tau = effectiveInertia / (cfg.swayDragCoeff() * cfg.hullMomentArm() * absSpeed);
		tau = Math.clamp(tau, 2.0, 30.0); // keep between 2-30 seconds

		// First-order exponential approach to steady state
		double yawRate = sub.yawRate();
		yawRate += (yawRateSteady - yawRate) * (1.0 - Math.exp(-dt / tau));
		sub.setYawRate(yawRate);

		double heading = sub.heading() + yawRate * dt;
		heading = heading % (2 * Math.PI);
		if (heading < 0)
			heading += 2 * Math.PI;
		sub.setHeading(heading);

		// 4. Pitch: first-order filter toward steady-state pitch rate (same approach as yaw)
		if (!cfg.surfaceLocked() && cfg.planesArea() > 0) {
			double planesAngle = actualPlanes * Math.PI / 4;
			double planesCl = liftCoefficient(planesAngle, cfg.stallAngle());
			double pitchMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed) * cfg.planesArea() * planesCl
					* cfg.planesArm() * controlFactor;
			// Hydrostatic restoring moment (metacentric height ~1.0m)
			double restoringMoment = cfg.dryMass() * 9.81 * 1.0 * Math.sin(sub.pitch());

			// Hydrodynamic hull restoring moment: a pitched hull moving through water
			// generates a pressure distribution that resists the pitch angle (hull body
			// lift / Munk moment). Proportional to speed² * sin(pitch), this is the
			// dominant pitch-limiting force at speed, preventing unrealistic angles.
			double lateralArea = 4.0 * cfg.hullHalfLength() * cfg.hullHalfBeam();
			double hullPitchArm = cfg.hullHalfLength() / 3.0;
			double hullPitchCl = 0.10;
			restoringMoment += 0.5 * WATER_DENSITY * speed * Math.abs(speed) * lateralArea * hullPitchCl * hullPitchArm
					* Math.sin(sub.pitch());
			// Effective inertia with speed-dependent resistance
			double pitchBaseInertia = cfg.massHeave() * cfg.rotationalInertia();
			double pitchSpeedDamping = pitchBaseInertia * 0.05 * speed * Math.abs(speed);
			double pitchEffectiveInertia = pitchBaseInertia + pitchSpeedDamping;

			double pitchRateSteady = (pitchMoment - restoringMoment) / pitchEffectiveInertia;

			// First-order filter with same time constant approach as yaw
			double pitchTau = pitchEffectiveInertia / (cfg.swayDragCoeff() * cfg.hullMomentArm() * absSpeed);
			pitchTau = Math.clamp(pitchTau, 2.0, 30.0);

			double pitchRate = sub.pitchRate();
			pitchRate += (pitchRateSteady - pitchRate) * (1.0 - Math.exp(-dt / pitchTau));
			sub.setPitchRate(pitchRate);

			double pitch = sub.pitch() + pitchRate * dt;
			double pitchLimit = Math.PI / 4;
			pitch = Math.clamp(pitch, -pitchLimit, pitchLimit);
			if ((pitch == pitchLimit && pitchRate > 0) || (pitch == -pitchLimit && pitchRate < 0)) {
				// The pitch stop removes outward motion; inward control can recover immediately.
				sub.setPitchRate(0);
			}
			sub.setPitch(pitch);
		}

		// 4. Ballast: slew actual ballast toward commanded ballast (tanks take time to flood/blow)
		double actual = sub.actualBallast();
		double buoyancyForce = 0;
		if (cfg.hasBallast()) {
			double commanded = sub.ballast();
			double maxChange = cfg.ballastSlewRate() * dt;
			if (commanded > actual) {
				actual = Math.min(actual + maxChange, commanded);
			} else if (commanded < actual) {
				actual = Math.max(actual - maxChange, commanded);
			}
			sub.setPreviousActualBallast(sub.actualBallast());
			sub.setActualBallast(actual);

			// Buoyancy force from ballast: 0.5 = neutral, <0.5 = heavy (sink), >0.5 = light (rise)
			buoyancyForce = (actual - 0.5) * 2.0 * cfg.ballastForceMax();
		}

		// Current vertical speed
		double verticalSpeed = sub.verticalSpeed();

		if (!cfg.surfaceLocked()) {
			// Vertical drag (proportional to v^2, large cross-section)
			double verticalDrag = cfg.verticalDragCoeff() * verticalSpeed * Math.abs(verticalSpeed);

			// Vertical acceleration: buoyancy force minus drag, divided by heave mass
			verticalSpeed += (buoyancyForce - verticalDrag) / cfg.massHeave() * dt;
			sub.setVerticalSpeed(verticalSpeed);
		} else {
			verticalSpeed = 0;
			sub.setVerticalSpeed(0);
		}

		// Collision impulses can leave lateral motion. Quadratic water drag dissipates it;
		// the implicit update remains stable even after a large impact.
		double swaySpeed = sub.swaySpeed();
		swaySpeed /= 1 + cfg.swayDragCoeff() * Math.abs(swaySpeed) * dt / cfg.massSway();
		sub.setSwaySpeed(swaySpeed);

		// 5. Position update (surge, lateral drift and vertical motion)
		double pitch = sub.pitch();
		double vx = speed * Math.sin(heading) * Math.cos(pitch) + swaySpeed * Math.cos(heading);
		double vy = speed * Math.cos(heading) * Math.cos(pitch) - swaySpeed * Math.sin(heading);
		double vz = speed * Math.sin(pitch) + verticalSpeed;

		// Apply current
		var current = currentField.currentAt(sub.z());
		vx += current.x();
		vy += current.y();

		double newX = sub.x() + vx * dt;
		double newY = sub.y() + vy * dt;
		double newZ = sub.z() + vz * dt;
		double contactVerticalSpeed = vz;

		// 6. Clamp: can't go above water
		if (newZ > 0) {
			// Clip a real rise from underwater, but do not count an above-water terrain
			// correction as downward motion when restoring the surface constraint.
			if (previousZ <= 0) {
				contactVerticalSpeed = -previousZ / dt;
			}
			newZ = 0;
			// Remove upward heave stopped by the surface. Do not invent downward heave
			// to cancel pitched surge: leveling or diving must release the constraint.
			sub.setVerticalSpeed(Math.min(0, verticalSpeed));
		}

		// For surfaceLocked vehicles, force z=0 and zero vertical speed
		if (cfg.surfaceLocked()) {
			newZ = 0;
			sub.setVerticalSpeed(0);
			contactVerticalSpeed = 0;
		}

		// Pressure applies to the deepest centre position reached this tick. Terrain correction
		// must not rescue a hull that has already crossed its absolute crush depth.
		hullPressure.step(sub, dt, Math.min(sub.z(), newZ));

		// 7. Terrain collision: check 7 hull points (pitch-aware)
		//    center, bow, stern, port, starboard, tower top, keel
		double[][] points = hullPoints(newX, newY, heading, pitch, cfg);

		// Navigation margins belong to controllers. Only the physical contact samples
		// can ground, damage or slow a hull; neither living hulls nor wrecks get an
		// invisible safety shell that lifts them off the seabed or out of the water.
		double worstPenetration = terrainPenetration(points, newZ, terrain);
		boolean scraping = false;
		if (worstPenetration > 0) {
			scraping = true;
			// Measure the incoming motion before correcting overlap or changing the bounce pose.
			// Rotation and currents move contact points even when the centre has no surge.
			var impact = new TerrainImpact(0, 0);
			if (sub.hp() > 0) {
				var previousPoints = hullPoints(previousX, previousY, previousHeading, previousPitch, cfg);
				impact = terrainImpact(points, newX, newY, newZ, previousPoints, contactVerticalSpeed, heading, pitch,
						cfg, terrain, dt);
			}
			newZ = newZ + worstPenetration;

			if (sub.hp() <= 0) {
				// Dead sub: settle on the bottom, no bounce, no damage.
				// Bleed all speed, align pitch with terrain slope.
				sub.setVerticalSpeed(0);
				sub.setSpeed(speed * 0.90); // drag to a stop
				sub.setYawRate(sub.yawRate() * 0.9);
				sub.setPitchRate(sub.pitchRate() * 0.9);

				// Compute terrain slope along the sub's heading to set rest pitch.
				// Sample terrain at bow and stern to find the slope angle.
				double halfLen = cfg.hullHalfLength();
				double bowX = newX + Math.sin(heading) * halfLen;
				double bowY = newY + Math.cos(heading) * halfLen;
				double sternX = newX - Math.sin(heading) * halfLen;
				double sternY = newY - Math.cos(heading) * halfLen;
				double bowFloor = terrain.elevationAt(bowX, bowY);
				double sternFloor = terrain.elevationAt(sternX, sternY);
				double terrainPitch = Math.atan2(bowFloor - sternFloor, 2 * halfLen);
				// Settle toward terrain pitch (not flat)
				double currentPitch = sub.pitch();
				sub.setPitch(currentPitch + (terrainPitch - currentPitch) * 0.03);
			} else {
				// Alive: bounce, take damage from inward contact motion, lose speed.
				int damage = TerrainImpactEnergy.damage(impact.energyJoules(), cfg.collisionDamageFactor());
				sub.setHp(Math.max(0, sub.hp() - damage));
				sub.setVerticalSpeed(cfg.bounceSpeed());
				sub.setPitch(Math.max(sub.pitch(), 0));

				if (impact.closingSpeed() > 3.0) {
					sub.setSpeed(speed * 0.5);
				} else {
					sub.setSpeed(speed * 0.95);
				}
			}
			// The bounce and wreck settling can change pitch. Clear the resulting hull as well;
			// that positional correction must not be charged as another impact next tick.
			if (sub.pitch() != pitch) {
				var responsePoints = hullPoints(newX, newY, heading, sub.pitch(), cfg);
				newZ += Math.max(0, terrainPenetration(responsePoints, newZ, terrain));
			}
		}

		// Guard against NaN/Inf from numerical issues
		if (!Double.isFinite(newX) || !Double.isFinite(newY) || !Double.isFinite(newZ)) {
			System.err.printf(
					"PHYSICS NaN/Inf detected for sub %d: pos=(%.1f,%.1f,%.1f) spd=%.1f hdg=%.3f yawRate=%.3f%n",
					sub.id(), newX, newY, newZ, speed, heading, yawRate);
			return; // skip this tick, keep previous position
		}
		sub.setX(newX);
		sub.setY(newY);
		sub.setZ(newZ);

		// 8. Noise model (dB-based source level)
		double sl = cfg.baseSlDb();

		// When clutch is disengaged and throttle is zero, machinery noise drops
		if (!clutchEngaged && actualThrottle == 0) {
			sl -= cfg.clutchDisengagedSlReduction();
		}

		// Speed-dependent: flow noise and propeller noise
		sl += cfg.speedNoiseDbPerMs() * Math.abs(speed);

		// Depth-dependent cavitation
		// Deeper = higher pressure = cavitation onset at higher speed
		// At -50m: cavitate above 6 m/s. At -200m: above 9 m/s. At -500m: above 15 m/s.
		double cavitationSpeed = cfg.baseCavitationSpeed() + (-newZ) * cfg.cavitationDepthFactor();
		if (Math.abs(speed) > cavitationSpeed) {
			double excess = (Math.abs(speed) - cavitationSpeed) / cavitationSpeed;
			sl += cfg.cavitationMaxDb() * Math.min(excess, 1.0);
		}

		// Control surface flow noise: deflected rudder/planes create turbulent
		// wake. Noise scales with speed² * deflection² (dynamic pressure * area).
		// A hard turn at flank speed is very loud; gentle turns at low speed are silent.
		double rudderDeflection = Math.abs(actualRudder);
		double planesDeflection = Math.abs(actualPlanes);
		double maxDeflection = Math.max(rudderDeflection, planesDeflection);
		if (maxDeflection > 0.05 && Math.abs(speed) > 1) {
			double flowNoise = 8.0 * (speed * speed / 225.0) * maxDeflection * maxDeflection;
			sl += flowNoise; // up to ~8 dB at flank speed with full deflection
		}

		// Reverse thrust cavitation (prop wash turbulence)
		if (actualThrottle < -0.1) {
			sl += cfg.reverseCavitationDb() * Math.abs(actualThrottle);
		}

		// Hull scraping noise (metal on rock is extremely loud)
		if (scraping && Math.abs(speed) > 0.5) {
			sl += 20.0 * Math.min(Math.abs(speed) / 5.0, 1.0);
		}

		// Surface proximity noise
		if (newZ > cfg.surfaceNoiseDepth()) {
			sl += cfg.surfaceNoiseDb() * (1.0 + newZ / (-cfg.surfaceNoiseDepth()));
		}

		// Ballast change noise (flooding/blowing tanks is audible)
		if (cfg.hasBallast()) {
			double ballastChangeRate = Math.abs(actual - sub.previousActualBallast()) / dt;
			if (ballastChangeRate > 0.001) {
				sl += cfg.ballastNoiseDb() * Math.min(ballastChangeRate / cfg.ballastSlewRate(), 1.0);
			}
		}

		// Torpedo launch transient: tube flooding + ejection noise spike
		if (sub.launchTransientTicks() > 0) {
			sl = Math.max(sl, 120.0); // ~120 dB transient, louder than normal ops
		}

		// Hull damage noise: cracked hull, bent machinery, flooding sounds
		sl += damageNoiseDb;

		sub.setSourceLevelDb(sl);

		// Also set linear noise level for viewer compatibility (80 dB = 1.0)
		sub.setNoiseLevel(Math.pow(10, (sl - 80) / 20.0));

		// 9. Battle area check
		if (!battleArea.contains(newX, newY)) {
			sub.setForfeited(true);
		}
	}

	/** World X/Y and centre-relative Z for the seven terrain contact points. */
	private static double[][] hullPoints(double x, double y, double heading, double pitch, VehicleConfig cfg) {
		double sinH = Math.sin(heading), cosH = Math.cos(heading);
		double sinPt = Math.sin(pitch), cosPt = Math.cos(pitch);
		double fwdX = sinH * cosPt, fwdY = cosH * cosPt;
		double upX = -sinH * sinPt, upY = -cosH * sinPt;
		var samples = HullGeometry.terrainSamplePoints(cfg);
		var points = new double[samples.length][3];
		for (int i = 0; i < samples.length; i++) {
			var local = samples[i];
			points[i][0] = x + cosH * local.x() + fwdX * local.y() + upX * local.z();
			points[i][1] = y - sinH * local.x() + fwdY * local.y() + upY * local.z();
			points[i][2] = sinPt * local.y() + cosPt * local.z();
		}
		return points;
	}

	private static double terrainPenetration(double[][] points, double z, TerrainMap terrain) {
		double worst = Double.NEGATIVE_INFINITY;
		for (var point : points) {
			worst = Math.max(worst, terrain.elevationAt(point[0], point[1]) - (z + point[2]));
		}
		return worst;
	}

	private record TerrainImpact(double closingSpeed, double energyJoules) {
	}

	private static TerrainImpact terrainImpact(
		double[][] points, double x, double y, double z, double[][] previousPoints, double centreVerticalSpeed,
		double heading, double pitch, VehicleConfig cfg, TerrainMap terrain, double dt) {
		double closingSpeed = 0;
		double energyJoules = 0;
		double sample = Math.min(1.0, terrain.getCellSize() * 0.5);
		for (int i = 0; i < points.length; i++) {
			var point = points[i];
			if (terrain.elevationAt(point[0], point[1]) <= z + point[2]) {
				continue;
			}
			var previous = previousPoints[i];
			double vx = (point[0] - previous[0]) / dt;
			double vy = (point[1] - previous[1]) / dt;
			double vz = centreVerticalSpeed + (point[2] - previous[2]) / dt;
			double slopeX = (terrain.elevationAt(point[0] + sample, point[1])
					- terrain.elevationAt(point[0] - sample, point[1])) / (2 * sample);
			double slopeY = (terrain.elevationAt(point[0], point[1] + sample)
					- terrain.elevationAt(point[0], point[1] - sample)) / (2 * sample);
			// Project onto the outward unit normal (-slopeX, -slopeY, 1).
			double normalLength = Math.hypot(Math.hypot(slopeX, slopeY), 1.0);
			var normal = new Vec3(-slopeX / normalLength, -slopeY / normalLength, 1 / normalLength);
			double inwardSpeed = -(normal.x() * vx + normal.y() * vy + normal.z() * vz);
			closingSpeed = Math.max(closingSpeed, inwardSpeed);
			var offset = new Vec3(point[0] - x, point[1] - y, point[2]);
			// Contacts can have different effective masses. Choose the greatest energy, not
			// merely the fastest point, and do not charge the same hull seven times.
			energyJoules = Math.max(energyJoules,
					TerrainImpactEnergy.energyJoules(cfg, offset, normal, heading, pitch, inwardSpeed));
		}
		return new TerrainImpact(closingSpeed, energyJoules);
	}
}
