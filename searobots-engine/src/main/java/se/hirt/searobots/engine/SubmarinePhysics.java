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

/**
 * Simplified submarine physics based on Fossen's 6-DOF formulation. See docs/physics-model.md for
 * full documentation, references, and characterization results.
 */
public final class SubmarinePhysics {

	private static final double WATER_DENSITY = 1025.0; // kg/m^3 seawater
	private static final double CL_SLOPE = 2 * Math.PI; // thin airfoil theory
	private final HullPressureModel hullPressure;
	private final long worldSeed;
	private final long correlationId;

	public SubmarinePhysics() {
		this(MatchConfig.withDefaults(0));
	}

	/** Creates physics with the match's depth limits and seeded pressure-hull failures. */
	public SubmarinePhysics(MatchConfig config) {
		this(config, 0);
	}

	SubmarinePhysics(MatchConfig config, long correlationId) {
		hullPressure = new HullPressureModel(config);
		worldSeed = config.worldSeed();
		this.correlationId = correlationId;
	}

	/**
	 * Checks a position changed by contact projection without charging another tick of exposure.
	 */
	void enforceCrushDepth(SubmarineEntity sub) {
		hullPressure.step(sub, 0, sub.z());
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
		double wreckForce = WreckDynamics.advance(sub, dt);
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
		if (actualThrottle < 0 && speed <= -cfg.maxReverseSpeed()) {
			// The engine cannot drive a hull farther beyond its astern operating limit.
			// Collision-imparted drift still decays through water drag rather than a speed clamp.
			thrust = 0;
		}
		double freeSpeed = speed + thrust / cfg.massSurge() * dt;
		double dragCoefficient = cfg.dragCoeff();
		if (freeSpeed < 0 && cfg.maxReverseSpeed() > 0) {
			// Poor stern-first streamlining sets the full-power astern equilibrium. Use the
			// configured reverse thrust, so submarine and ship operating speeds stay familiar.
			dragCoefficient = Math.max(dragCoefficient,
					cfg.maxThrust() * cfg.reverseThrustFactor() / (cfg.maxReverseSpeed() * cfg.maxReverseSpeed()));
		}
		// Extra drag from windmilling prop when clutch engaged at zero throttle.
		if (clutchEngaged && actualThrottle == 0) {
			dragCoefficient *= 1 + cfg.propDragFactor();
		}
		if (freeSpeed < 0) {
			// Backward Euler for quadratic drag is dissipative even after a strong collision,
			// and its force balance cannot drive an initially slower hull past the astern limit.
			double magnitude = -freeSpeed;
			speed = -2 * magnitude / (1 + Math.sqrt(1 + 4 * dragCoefficient * dt * magnitude / cfg.massSurge()));
		} else {
			double drag = dragCoefficient * speed * Math.abs(speed);
			speed += (thrust - drag) / cfg.massSurge() * dt;
		}
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
		double speedDamping = baseInertia * 0.05 * speed * speed;
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
		if (!sub.surfaceConstrained() && (cfg.planesArea() > 0 || sub.hp() <= 0)) {
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
			restoringMoment += 0.5 * WATER_DENSITY * speed * speed * lateralArea * hullPitchCl * hullPitchArm
					* Math.sin(sub.pitch());
			// Effective inertia with speed-dependent resistance
			double pitchBaseInertia = cfg.massHeave() * cfg.rotationalInertia();
			double pitchSpeedDamping = pitchBaseInertia * 0.05 * speed * speed;
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
		double buoyancyForce = wreckForce;
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
			buoyancyForce += (actual - 0.5) * 2.0 * cfg.ballastForceMax();
		}

		// Current vertical speed
		double verticalSpeed = sub.verticalSpeed();

		if (!sub.surfaceConstrained()) {
			// Vertical drag (proportional to v^2, large cross-section)
			double verticalDrag = WreckDynamics.verticalDragCoefficient(sub) * verticalSpeed * Math.abs(verticalSpeed);

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
		if (sub.surfaceConstrained()) {
			newZ = 0;
			sub.setVerticalSpeed(0);
			contactVerticalSpeed = 0;
		}

		// Keep the incoming motion independent of the shortened path or position correction.
		var incomingLinear = new Vec3(vx, vy, contactVerticalSpeed);
		double incomingYawRate = Math.atan2(Math.sin(heading - previousHeading), Math.cos(heading - previousHeading))
				/ dt;
		double incomingPitchRate = (pitch - previousPitch) / dt;
		var terrainStep = moveWithTerrain(sub, dt, new Vec3(previousX, previousY, previousZ),
				new Vec3(newX, newY, newZ), previousHeading, heading, previousPitch, pitch, incomingLinear,
				incomingYawRate, incomingPitchRate, terrain, currentField);
		newX = sub.x();
		newY = sub.y();
		newZ = sub.z();
		heading = sub.heading();
		speed = sub.speed();
		boolean scraping = terrainStep.scraping();
		// Only the physically reached path counts as exposure; projection cannot rescue a hull
		// already below the limit, but a swept seabed contact can stop it before that depth.
		hullPressure.step(sub, dt, terrainStep.deepestZ());

		// Guard against NaN/Inf from numerical issues
		if (!Double.isFinite(newX) || !Double.isFinite(newY) || !Double.isFinite(newZ)) {
			System.err.printf(
					"PHYSICS NaN/Inf detected for sub %d: pos=(%.1f,%.1f,%.1f) spd=%.1f hdg=%.3f yawRate=%.3f%n",
					sub.id(), newX, newY, newZ, speed, heading, yawRate);
			sub.setX(previousX);
			sub.setY(previousY);
			sub.setZ(previousZ);
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

	private record TerrainStep(boolean scraping, double deepestZ) {
	}

	private TerrainStep moveWithTerrain(
		SubmarineEntity sub, double dt, Vec3 start, Vec3 end, double startHeading, double endHeading, double startPitch,
		double endPitch, Vec3 incomingLinear, double incomingYawRate, double incomingPitchRate, TerrainMap terrain,
		CurrentField currents) {
		var terrainEvent = SimulationJfrEvents.beginTerrainMotion();
		var initialStart = start;
		int hpAtStart = terrainEvent == null ? 0 : sub.hp();
		try {
			var cfg = sub.vehicleConfig();
			double remaining = dt, deepestZ = start.z(), impactEnergy = 0;
			boolean scraping = false;
			// Continue the unspent tick after an impact, so support on a flat floor does not
			// cancel tangential movement. A small cap bounds pathological repeated contacts.
			for (int iteration = 0; iteration < 6; iteration++) {
				if (terrainEvent != null) {
					terrainEvent.iterations++;
					terrainEvent.remainingSeconds = remaining;
				}
				var contact = SweptHullTerrain.firstContact(start, end, startHeading, endHeading, startPitch, endPitch,
						cfg, terrain);
				if (terrainEvent != null && contact != null) {
					terrainEvent.sweptContacts++;
					terrainEvent.lastContactFraction = contact.fraction();
				}
				var position = contact == null ? end : contact.position();
				double heading = contact == null ? endHeading : contact.heading();
				double pitch = contact == null ? endPitch : contact.pitch();
				deepestZ = Math.min(deepestZ, position.z());
				var samples = contact == null
						? HullGeometry.terrainContactPoints(position, heading, pitch, cfg, terrain)
						: contact.localPoints();
				if (terrainEvent != null)
					terrainEvent.contactPointCount += samples.length;
				double penetration = 0;
				for (var local : samples) {
					var world = HullTerrainGeometry.worldPoint(local, position, heading, pitch);
					penetration = Math.max(penetration, terrain.elevationAt(world.x(), world.y()) - world.z());
				}
				sub.setHeading(heading);
				sub.setPitch(pitch);
				sub.setX(position.x());
				sub.setY(position.y());
				sub.setZ(position.z());
				// Reaching absolute crush depth changes contact response immediately: an
				// imploded hull cannot receive a living hull's restitution impulse.
				hullPressure.step(sub, 0, deepestZ);
				if (penetration > 0 || contact != null) {
					scraping = true;
					var current = currents.currentAt(position.z());
					SubCollisionResponse.setLinearVelocity(sub,
							incomingLinear.subtract(new Vec3(current.x(), current.y(), 0)));
					var impact = TerrainContactResponse.apply(sub, position, samples, incomingLinear, incomingYawRate,
							incomingPitchRate, terrain, currents);
					impactEnergy = Math.max(impactEnergy, impact.energyJoules());
					var groundVelocity = sub.velocity().linear().add(new Vec3(current.x(), current.y(), 0));
					sub.setZ(position.z() + penetration);
					var correctedCurrent = currents.currentAt(sub.z());
					if (!correctedCurrent.equals(current)) {
						// An artificial overlap correction must not add a current-band velocity jump.
						SubCollisionResponse.setLinearVelocity(sub,
								groundVelocity.subtract(new Vec3(correctedCurrent.x(), correctedCurrent.y(), 0)));
					}
					if (sub.z() >= 0 && sub.verticalSpeed() > 0) {
						sub.setVerticalSpeed(0);
					}
				}
				if (contact == null) {
					if (terrainEvent != null)
						terrainEvent.remainingSeconds = 0;
					break;
				}
				remaining *= 1 - contact.fraction();
				if (terrainEvent != null)
					terrainEvent.remainingSeconds = remaining;
				if (remaining <= dt * 1e-9) {
					break;
				}
				start = sub.pose().position();
				startHeading = heading;
				startPitch = pitch;
				var current = currents.currentAt(start.z());
				incomingLinear = sub.velocity().linear().add(new Vec3(current.x(), current.y(), 0));
				incomingYawRate = sub.yawRate();
				incomingPitchRate = sub.surfaceConstrained() ? 0 : sub.pitchRate();
				end = start.add(incomingLinear.scale(remaining));
				end = new Vec3(end.x(), end.y(),
						sub.surfaceConstrained() ? 0 : Math.min(Math.max(0, start.z()), end.z()));
				endHeading = heading + incomingYawRate * remaining;
				endPitch = Math.clamp(pitch + incomingPitchRate * remaining, -Math.PI / 4, Math.PI / 4);
			}
			if (sub.hp() > 0) {
				sub.setHp(
						Math.max(0, sub.hp() - TerrainImpactEnergy.damage(impactEnergy, cfg.collisionDamageFactor())));
			}
			double finalHeading = sub.heading() % (2 * Math.PI);
			sub.setHeading(finalHeading < 0 ? finalHeading + 2 * Math.PI : finalHeading);
			return new TerrainStep(scraping, deepestZ);
		} finally {
			SimulationJfrEvents.finishTerrainMotion(terrainEvent, correlationId, worldSeed, sub, initialStart,
					hpAtStart);
		}
	}

}
