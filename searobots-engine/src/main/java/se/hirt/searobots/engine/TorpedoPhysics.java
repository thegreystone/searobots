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
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

/**
 * Physics model for torpedoes. Simplified relative to submarine physics: no ballast, no clutch, no
 * reverse thrust. Torpedoes are slightly negatively buoyant and rely on hydrodynamic lift from
 * forward motion to maintain depth. Below minimum speed (~3 m/s), lift fails and the torpedo sinks.
 */
public final class TorpedoPhysics {

	private static final double WATER_DENSITY = 1025.0; // kg/m^3
	private static final double NEGATIVE_BUOYANCY = 30.0; // N downward (slight)
	private static final double LIFT_COEFFICIENT = 0.3; // lift per m/s^2 at depth
	private static final double MAX_CONTROL_DEFLECTION = Math.PI / 4;
	private static final double FULL_DEFLECTION_LIFT_FRACTION = 0.6;
	// Game-calibrated fin drag: lift-induced loss plus profile loss at large deflections.
	private static final double INDUCED_DRAG_FACTOR = 0.002;
	private static final double FIN_PROFILE_DRAG_FACTOR = 0.02;
	private static final int MAX_TERRAIN_BROADPHASE_CORNERS = 256;

	/**
	 * Preserve the existing attached-flow lift below stall, then progressively reduce authority as
	 * flow separates. Full deflection retains 60% of peak lift; this is game calibration.
	 */
	private static double liftCoefficient(double alpha, double stallAngle) {
		double absAlpha = Math.abs(alpha);
		if (absAlpha <= stallAngle) {
			return 2 * Math.PI * Math.sin(alpha);
		}
		double peakLift = 2 * Math.PI * Math.sin(stallAngle);
		double postStallFraction = Math.clamp((absAlpha - stallAngle) / (MAX_CONTROL_DEFLECTION - stallAngle), 0, 1);
		double separation = postStallFraction * postStallFraction * (3 - 2 * postStallFraction);
		return Math.copySign(peakLift * (1 - (1 - FULL_DEFLECTION_LIFT_FRACTION) * separation), alpha);
	}

	/**
	 * Steps the torpedo physics by one tick.
	 *
	 * @param torp
	 *            torpedo entity to update
	 * @param dt
	 *            time step in seconds
	 * @param terrain
	 *            terrain map for collision checking
	 * @param currentField
	 *            ocean currents
	 * @param battleArea
	 *            battle area boundary
	 */
	public void step(
		TorpedoEntity torp, double dt, TerrainMap terrain, CurrentField currentField, BattleArea battleArea) {
		if (!torp.alive() || torp.inTube())
			return;

		var cfg = torp.vehicleConfig();
		var previousPosition = torp.pose().position();
		double previousHeading = torp.heading();
		double previousPitch = torp.pitch();

		// 1. Fuel consumption
		double cmdThrottle = torp.cmdThrottle(); // already 0 if no fuel
		if (cmdThrottle > 0 && torp.fuelRemaining() > 0) {
			torp.consumeFuel(dt);
		}
		if (torp.fuelRemaining() <= 0) {
			cmdThrottle = 0;
		}

		// 2. Throttle slew
		double actualThrottle = torp.actualThrottle();
		double maxThrottleChange = cfg.thrustSlewRate() * dt;
		if (cmdThrottle > actualThrottle) {
			actualThrottle = Math.min(actualThrottle + maxThrottleChange, cmdThrottle);
		} else {
			actualThrottle = Math.max(actualThrottle - maxThrottleChange * 2, cmdThrottle); // fast decay
		}
		torp.setActualThrottle(actualThrottle);

		// 3. Control surface slew (faster than submarine: small fins, less inertia)
		double controlSlewRate = 1.5; // faster than sub's 0.57
		double maxControlChange = controlSlewRate * dt;

		double actualRudder = torp.actualRudder();
		double cmdRudder = torp.cmdRudder();
		if (cmdRudder > actualRudder) {
			actualRudder = Math.min(actualRudder + maxControlChange, cmdRudder);
		} else {
			actualRudder = Math.max(actualRudder - maxControlChange, cmdRudder);
		}
		torp.setActualRudder(actualRudder);

		double actualPlanes = torp.actualSternPlanes();
		double cmdPlanes = torp.cmdSternPlanes();
		if (cmdPlanes > actualPlanes) {
			actualPlanes = Math.min(actualPlanes + maxControlChange, cmdPlanes);
		} else {
			actualPlanes = Math.max(actualPlanes - maxControlChange, cmdPlanes);
		}
		torp.setActualSternPlanes(actualPlanes);

		double rudderAngle = actualRudder * MAX_CONTROL_DEFLECTION;
		double rudderCl = liftCoefficient(rudderAngle, cfg.stallAngle());
		double planesAngle = actualPlanes * MAX_CONTROL_DEFLECTION;
		double planesCl = liftCoefficient(planesAngle, cfg.stallAngle());

		// 4. Thrust and drag. A hard turn must spend speed as well as angular authority;
		// stalled fins still incur profile drag even after their lift has fallen.
		double rudderSin = Math.sin(rudderAngle), planesSin = Math.sin(planesAngle);
		double finDrag = 0.5 * WATER_DENSITY * (INDUCED_DRAG_FACTOR
				* (cfg.rudderArea() * rudderCl * rudderCl + cfg.planesArea() * planesCl * planesCl)
				+ FIN_PROFILE_DRAG_FACTOR
						* (cfg.rudderArea() * rudderSin * rudderSin + cfg.planesArea() * planesSin * planesSin));
		double thrust = cfg.maxThrust() * actualThrottle;
		double speed = torp.speed();
		double drag = (cfg.dragCoeff() + finDrag) * speed * Math.abs(speed);
		speed = Math.max(0, speed + (thrust - drag) / cfg.massSurge() * dt);
		torp.setSpeed(speed);

		// 5. Steering authority. Pitch responds faster than yaw, but has comparable steady
		// authority. Both axes share one budget for changing the direction of travel.
		double rudderMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed) * cfg.rudderArea() * rudderCl
				* cfg.rudderArm();

		double baseInertia = cfg.massSurge() * cfg.rotationalInertia();
		// Torpedo: long narrow body generates enormous rotational resistance at speed.
		// Speed damping grows with v^2, making the torpedo progressively less
		// maneuverable at higher speeds. At fixed speed the optimum level turn radius
		// is about 160m at 20m/s and 44m at 5m/s; powered turns also lose speed to fin drag.
		double speedDampingCoeff = 0.4; // higher than sub's 0.05, but allows reasonable turns
		double speedDamping = baseInertia * speedDampingCoeff * speed * Math.abs(speed);
		double effectiveInertia = baseInertia + speedDamping;

		double yawRateSteady = effectiveInertia > 0 ? rudderMoment / effectiveInertia : 0;

		double absSpeed = Math.max(speed, 0.5);
		double tau = effectiveInertia / (cfg.swayDragCoeff() * cfg.hullMomentArm() * absSpeed);
		tau = Math.clamp(tau, 0.5, 10.0); // torpedoes respond faster

		double pitchMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed) * cfg.planesArea() * planesCl
				* cfg.planesArm();
		double pitchInertia = baseInertia * 1.5 + speedDamping;
		double pitchRateSteady = pitchInertia > 0 ? pitchMoment / pitchInertia : 0;
		double pitchTau = Math.clamp(pitchInertia / (cfg.swayDragCoeff() * absSpeed), 1.2, 6.0);

		double pitch = torp.pitch();
		double pitchLimit = Math.PI / 3; // max 60 deg
		double previousPitchRate = torp.pitchRate();
		if ((pitch >= pitchLimit && pitchRateSteady > 0) || (pitch <= -pitchLimit && pitchRateSteady < 0))
			pitchRateSteady = 0;
		if ((pitch >= pitchLimit && previousPitchRate > 0) || (pitch <= -pitchLimit && previousPitchRate < 0))
			previousPitchRate = 0;

		double peakCl = Math.abs(liftCoefficient(cfg.stallAngle(), cfg.stallAngle()));
		double peakYawRate = effectiveInertia > 0
				? 0.5 * WATER_DENSITY * speed * speed * cfg.rudderArea() * peakCl * cfg.rudderArm() / effectiveInertia
				: 0;
		double peakPitchRate = pitchInertia > 0
				? 0.5 * WATER_DENSITY * speed * speed * cfg.planesArea() * peakCl * cfg.planesArm() / pitchInertia : 0;
		double steeringBudget = Math.max(peakYawRate, peakPitchRate);
		double cosPitch = Math.cos(pitch);
		double demandedRate = Math.hypot(yawRateSteady * cosPitch, pitchRateSteady);
		if (demandedRate > steeringBudget) {
			double scale = steeringBudget / demandedRate;
			yawRateSteady *= scale;
			pitchRateSteady *= scale;
		}

		// Unequal response times must not briefly double the budget when switching axes.
		// Preserve naturally decaying prior motion, including rotation of a stopped hull.
		double yawRetention = Math.exp(-dt / tau), pitchRetention = Math.exp(-dt / pitchTau);
		double retainedYaw = torp.yawRate() * yawRetention;
		double retainedPitch = previousPitchRate * pitchRetention;
		double yawRate = retainedYaw + yawRateSteady * (1 - yawRetention);
		double pitchRate = retainedPitch + pitchRateSteady * (1 - pitchRetention);
		// Inward pitch motion increases yaw's contribution to spatial curvature. Bound the
		// entire pitch sweep, including a crossing of level, before advancing orientation.
		double endPitch = Math.clamp(pitch + pitchRate * dt, -pitchLimit, pitchLimit);
		double sweepCosine = pitch * endPitch <= 0 ? 1 : Math.max(Math.abs(cosPitch), Math.abs(Math.cos(endPitch)));
		double allowedRate = Math.max(steeringBudget, Math.hypot(retainedYaw * sweepCosine, retainedPitch));
		double directionRate = Math.hypot(yawRate * sweepCosine, pitchRate);
		if (directionRate > allowedRate) {
			double scale = allowedRate / directionRate;
			yawRate *= scale;
			pitchRate *= scale;
		}
		torp.setYawRate(yawRate);
		torp.setPitchRate(pitchRate);

		// 6. Advance orientation, removing only outward motion at the pitch stops.
		double heading = (torp.heading() + yawRate * dt) % (2 * Math.PI);
		if (heading < 0)
			heading += 2 * Math.PI;
		torp.setHeading(heading);
		pitch = Math.clamp(pitch + pitchRate * dt, -pitchLimit, pitchLimit);
		if ((pitch == pitchLimit && pitchRate > 0) || (pitch == -pitchLimit && pitchRate < 0))
			torp.setPitchRate(0);
		torp.setPitch(pitch);

		// 7. Buoyancy and lift
		double verticalSpeed = torp.verticalSpeed();

		if (speed >= TorpedoEntity.minimumSpeed()) {
			// Sufficient speed: hydrodynamic lift counteracts negative buoyancy
			// Lift proportional to speed^2, counters the constant negative buoyancy
			double liftForce = LIFT_COEFFICIENT * speed * speed;
			double netVertical = (-NEGATIVE_BUOYANCY + liftForce) / cfg.massHeave();
			// Damp toward zero: lift maintains depth when pitch controls are neutral
			verticalSpeed += netVertical * dt;
			verticalSpeed *= Math.exp(-2.0 * dt); // vertical damping
		} else {
			// Below minimum speed: lift fails, torpedo sinks
			verticalSpeed -= TorpedoEntity.sinkAcceleration() * dt;
		}

		// Vertical drag
		double vDrag = cfg.verticalDragCoeff() * verticalSpeed * Math.abs(verticalSpeed);
		verticalSpeed -= vDrag / cfg.massHeave() * dt;
		torp.setVerticalSpeed(verticalSpeed);

		// 8. Position update
		double sinH = Math.sin(heading);
		double cosH = Math.cos(heading);
		double cosP = Math.cos(torp.pitch());
		double sinP = Math.sin(torp.pitch());

		double vx = speed * sinH * cosP;
		double vy = speed * cosH * cosP;
		double vz = speed * sinP + verticalSpeed;

		// Apply ocean current
		double depth = torp.z();
		if (currentField != null) {
			var current = currentField.currentAt(depth);
			vx += current.x();
			vy += current.y();
		}

		double newX = torp.x() + vx * dt;
		double newY = torp.y() + vy * dt;
		double newZ = torp.z() + vz * dt;

		// 9. Surface clamp
		if (newZ > 0) {
			newZ = 0;
			// Remove upward heave without inventing a downward counter-velocity for pitched surge.
			torp.setVerticalSpeed(Math.min(0, verticalSpeed));
		}

		// 10. Terrain collision: torpedo DETONATES on impact (can damage nearby subs)
		if (terrain != null) {
			var contact = firstTerrainContact(previousPosition, new Vec3(newX, newY, newZ), previousHeading, heading,
					previousPitch, torp.pitch(), cfg, terrain);
			if (contact != null) {
				torp.detonate(); // detonate, not just kill
				torp.setX(contact.position().x());
				torp.setY(contact.position().y());
				torp.setZ(contact.position().z());
				torp.setHeading(contact.heading());
				torp.setPitch(contact.pitch());
				return;
			}
		}

		// 11. Battle area: torpedo destroyed if outside
		if (battleArea != null && !battleArea.contains(newX, newY)) {
			torp.kill();
		}

		torp.setX(newX);
		torp.setY(newY);
		torp.setZ(newZ);

		// 12. Noise model (simplified: loud constant base + speed contribution)
		double baseNoise = cfg.baseSlDb();
		double speedNoise = cfg.speedNoiseDbPerMs() * speed;
		torp.setSourceLevelDb(baseNoise + speedNoise);
	}

	private record TerrainContact(Vec3 position, double heading, double pitch) {
	}

	/**
	 * Finds the first intersecting pose, including terrain crossed between the tick's endpoints.
	 */
	private static TerrainContact firstTerrainContact(
		Vec3 start, Vec3 end, double startHeading, double endHeading, double startPitch, double endPitch,
		VehicleConfig cfg, TerrainMap terrain) {
		if (clearsTerrainBounds(start, end, cfg, terrain)) {
			return null;
		}
		var envelope = new HullGeometry.Envelope(cfg.hullHalfLength(), cfg.hullHalfBeam(), cfg.hullHalfBeam(), 0, 0);
		double headingChange = Math.atan2(Math.sin(endHeading - startHeading), Math.cos(endHeading - startHeading));
		var movement = end.subtract(start);
		double spacing = Math.min(cfg.hullHalfLength(), terrain.getCellSize() * 0.5);
		double radius = Math.max(cfg.hullHalfLength(), cfg.hullHalfBeam());
		double tipTravel = movement.length() + radius * (Math.abs(headingChange) + Math.abs(endPitch - startPitch));
		int samples = Math.max(1, (int) Math.ceil(tipTravel / spacing));
		double previousFraction = 0;
		if (terrainGap(start, startHeading, startPitch, envelope, terrain) < 0) {
			return new TerrainContact(start, startHeading, startPitch);
		}
		for (int i = 1; i <= samples; i++) {
			double fraction = (double) i / samples;
			var position = start.add(movement.scale(fraction));
			double heading = startHeading + headingChange * fraction;
			double pitch = startPitch + (endPitch - startPitch) * fraction;
			if (terrainGap(position, heading, pitch, envelope, terrain) < 0) {
				double lower = previousFraction, upper = fraction;
				for (int iteration = 0; iteration < 24; iteration++) {
					double middle = (lower + upper) * 0.5;
					if (terrainGap(start.add(movement.scale(middle)), startHeading + headingChange * middle,
							startPitch + (endPitch - startPitch) * middle, envelope, terrain) < 0) {
						upper = middle;
					} else {
						lower = middle;
					}
				}
				return new TerrainContact(start.add(movement.scale(upper)), startHeading + headingChange * upper,
						startPitch + (endPitch - startPitch) * upper);
			}
			previousFraction = fraction;
		}
		return null;
	}

	/** Bilinear elevations never exceed the greatest lattice corner in the swept hull's bounds. */
	private static boolean clearsTerrainBounds(Vec3 start, Vec3 end, VehicleConfig cfg, TerrainMap terrain) {
		double radius = Math.max(cfg.hullHalfLength(), cfg.hullHalfBeam());
		double lowestHullZ = Math.min(start.z(), end.z()) - radius;
		if (lowestHullZ >= terrain.getMaxElevation()) {
			return true;
		}
		double cell = terrain.getCellSize();
		double minX = Math.min(start.x(), end.x()) - radius, maxX = Math.max(start.x(), end.x()) + radius;
		double minY = Math.min(start.y(), end.y()) - radius, maxY = Math.max(start.y(), end.y()) + radius;
		int minCol = (int) Math.max(0, Math.floor((minX - terrain.getOriginX()) / cell));
		int maxCol = (int) Math.min(terrain.getCols() - 1, Math.floor((maxX - terrain.getOriginX()) / cell) + 1);
		int minRow = (int) Math.max(0, Math.floor((minY - terrain.getOriginY()) / cell));
		int maxRow = (int) Math.min(terrain.getRows() - 1, Math.floor((maxY - terrain.getOriginY()) / cell) + 1);
		double maximum = terrain.getMinElevation(); // Also bounds the map's out-of-grid interpolation corners.
		if (minCol > maxCol || minRow > maxRow) {
			return lowestHullZ >= maximum;
		}
		long corners = ((long) maxCol - minCol + 1) * ((long) maxRow - minRow + 1);
		if (corners > MAX_TERRAIN_BROADPHASE_CORNERS) {
			return false;
		}
		for (int row = minRow; row <= maxRow; row++) {
			for (int col = minCol; col <= maxCol; col++) {
				maximum = Math.max(maximum, terrain.elevationAtGrid(col, row));
			}
		}
		return lowestHullZ >= maximum;
	}

	/**
	 * Physical gap beneath the torpedo ellipsoid, including terrain cell edges and ridge crests.
	 */
	private static double terrainGap(
		Vec3 position, double heading, double pitch, HullGeometry.Envelope envelope, TerrainMap terrain) {
		double minimum = Double.POSITIVE_INFINITY;
		for (var local : HullTerrainGeometry.ellipsoidContactPoints(position, heading, pitch, envelope, terrain)) {
			minimum = Math.min(minimum,
					gapAt(HullTerrainGeometry.worldPoint(local, position, heading, pitch), terrain));
		}
		return minimum;
	}

	private static double gapAt(Vec3 point, TerrainMap terrain) {
		return point.z() - terrain.elevationAt(point.x(), point.y());
	}
}
