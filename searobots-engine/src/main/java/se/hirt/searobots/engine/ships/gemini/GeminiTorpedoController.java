/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoOutput;
import se.hirt.searobots.api.Vec3;

import java.util.List;

/**
 * Green-field autonomous torpedo controller for Gemini.
 * <p>
 * Features first-order lag compensation (&tau;_yaw=10s, &tau;_pitch=6s), non-stalling fin clamping
 * with shared steering-budget coordination, zero-fuel-burn coasting during re-attack turns, a 3D
 * gain-scheduled alpha-beta active tracker, per-sample 3D slope-pitch terrain clearance, and live
 * dual-return anti-friendly-fire gating.
 */
public final class GeminiTorpedoController implements TorpedoController {

	private enum Phase {
		DOGLEG, TRANSIT, HOMING, TERMINAL, REATTACK, SEARCH
	}

	private MatchConfig config;
	private TerrainMap terrain;
	private GeminiMission mission;
	private Vec3 launchPos;
	private long releaseTick = -1;

	private double launchDirX, launchDirY;
	private double initialTargetDist;

	// 3D target state estimate
	private double estX, estY, estZ;
	private double estVx, estVy, estVz;
	private boolean hasActiveLock;
	private boolean everLockedEnemy;
	private int activeFixCount;
	private long lastActiveTick = -10_000;
	private double lastActiveRange = Double.MAX_VALUE;
	private double prevActiveRange = Double.MAX_VALUE;

	// Passive bearing tracking
	private double lastPassiveBearing = Double.NaN;
	private long lastPassiveTick = -10_000;

	// Live launcher exclusion track (updated both by dead-reckoning AND dual-return pings!)
	private double launcherX, launcherY;
	private double launcherVx, launcherVy;

	// Phase & search state
	private Phase phase = Phase.TRANSIT;
	private boolean waypointCleared;
	private double searchCenterX, searchCenterY, searchCenterZ;
	private double searchAngle;
	private double searchRadius = 200.0;
	private int searchDirection = 1;

	@Override
	public void onLaunch(se.hirt.searobots.api.TorpedoLaunchContext context) {
		this.config = context.config();
		this.terrain = context.terrain();
		this.launchPos = context.launchPosition();
		this.mission = GeminiMission.decode(context.missionData(), launchPos.x(), launchPos.y(), launchPos.z(),
				context.launchHeading());

		double tubeDelay = 3.0;
		double vx = 0.0, vy = 0.0;
		if (Double.isFinite(mission.targetHeading()) && mission.targetSpeed() > 0) {
			vx = mission.targetSpeed() * Math.sin(mission.targetHeading());
			vy = mission.targetSpeed() * Math.cos(mission.targetHeading());
		}
		this.estX = mission.targetX() + vx * tubeDelay;
		this.estY = mission.targetY() + vy * tubeDelay;
		this.estZ = clampOperationalDepth(estX, estY, mission.targetZ(), 14.0);
		this.estVx = vx;
		this.estVy = vy;
		this.estVz = 0.0;

		double dx = estX - launchPos.x();
		double dy = estY - launchPos.y();
		this.initialTargetDist = Math.max(1.0, Math.hypot(dx, dy));
		this.launchDirX = dx / initialTargetDist;
		this.launchDirY = dy / initialTargetDist;

		this.launcherX = mission.launcherX();
		this.launcherY = mission.launcherY();
		this.launcherVx = Math.sin(mission.launcherEscapeBearing()) * 5.0;
		this.launcherVy = Math.cos(mission.launcherEscapeBearing()) * 5.0;

		this.waypointCleared = !mission.hasWaypoint();
		this.phase = waypointCleared ? Phase.TRANSIT : Phase.DOGLEG;
		this.searchCenterX = estX;
		this.searchCenterY = estY;
		this.searchCenterZ = estZ;
		this.searchAngle = GeminiMath.normalizeBearing(context.launchHeading());
		this.searchDirection = (GeminiMath.angleDiff(GeminiMath.bearing(launchPos.x(), launchPos.y(), estX, estY),
				context.launchHeading()) >= 0) ? 1 : -1;
	}

	@Override
	public void onTick(TorpedoInput input, TorpedoOutput output) {
		long tick = input.tick();
		double dt = input.deltaTimeSeconds();
		if (releaseTick < 0)
			releaseTick = tick;
		double flightTime = (tick - releaseTick) * dt;

		var pose = input.self();
		var pos = pose.position();
		double heading = pose.heading();
		double pitch = pose.pitch();
		double speed = input.speed();
		double yawRate = input.velocity().angular().z();
		double pitchRate = input.velocity().angular().y();

		var waterVel = input.velocity().linear();
		var groundVel = input.groundVelocity();
		double curX = groundVel.x() - waterVel.x();
		double curY = groundVel.y() - waterVel.y();

		// 1. Propagate target & launcher estimates
		estX += estVx * dt;
		estY += estVy * dt;
		estZ = clampOperationalDepth(estX, estY, estZ + estVz * dt, 5.0);
		if ((tick - lastActiveTick) * dt > 1.8) {
			estVz *= Math.exp(-0.35 * dt);
		}

		launcherX += launcherVx * dt;
		launcherY += launcherVy * dt;

		// 2. Process active and passive sonar contacts
		processActiveReturns(tick, dt, flightTime, pos, input.activeSonarReturns());
		processPassiveContacts(tick, flightTime, pos, input.sonarContacts());

		// 3. Active sonar ping schedule
		double distToEst = GeminiMath.dist3D(pos.x(), pos.y(), pos.z(), estX, estY, estZ);
		double distFromLaunch = GeminiMath.dist2D(pos.x(), pos.y(), launchPos.x(), launchPos.y());
		double distFromLauncher = GeminiMath.dist2D(pos.x(), pos.y(), launcherX, launcherY);
		double progressAlongLaunch = (pos.x() - launchPos.x()) * launchDirX + (pos.y() - launchPos.y()) * launchDirY;

		boolean safeToPing = flightTime >= 5.0 && distFromLaunch > 260.0 && distFromLauncher > 280.0
				&& (initialTargetDist < 900.0 || progressAlongLaunch > 0.25 * initialTargetDist);
		boolean inAcquisitionZone = distToEst < 2200.0 || hasActiveLock || phase == Phase.SEARCH
				|| (mission.mode() == GeminiMission.Mode.SNAP && distToEst < 2600.0);

		if (safeToPing && inAcquisitionZone && input.activeSonarCooldownTicks() <= 0) {
			output.activeSonarPing();
		}

		// 4. Phase transitions
		double activeAge = (tick - lastActiveTick) * dt;
		boolean freshActive = hasActiveLock && activeAge < 4.0;
		if (hasActiveLock && activeAge > 7.0) {
			hasActiveLock = false;
		}

		if (phase == Phase.DOGLEG) {
			double dWp = GeminiMath.dist2D(pos.x(), pos.y(), mission.waypointX(), mission.waypointY());
			boolean clearDirect = GeminiMath.maxFloorAlongSegment(terrain, pos.x(), pos.y(), estX, estY,
					40.0) < Math.max(pos.z(), estZ) - 14.0;
			if (dWp < 260.0 || freshActive || (flightTime > 8.0 && clearDirect)) {
				waypointCleared = true;
				phase = freshActive ? Phase.HOMING : Phase.TRANSIT;
			}
		}

		if (waypointCleared) {
			if (freshActive) {
				double bearingToTarget = GeminiMath.bearing(pos.x(), pos.y(), estX, estY);
				double angleOff = Math.abs(GeminiMath.angleDiff(bearingToTarget, heading));
				if (distToEst < 420.0 && angleOff > Math.toRadians(55)) {
					phase = Phase.REATTACK;
				} else if (distToEst < 600.0) {
					phase = Phase.TERMINAL;
				} else {
					phase = Phase.HOMING;
				}
			} else {
				if (phase == Phase.HOMING || phase == Phase.TERMINAL || phase == Phase.REATTACK) {
					if (activeAge > 4.0) {
						phase = Phase.SEARCH;
						searchCenterX = estX;
						searchCenterY = estY;
						searchCenterZ = estZ;
						searchRadius = 200.0;
						searchAngle = GeminiMath.normalizeBearing(heading);
					}
				} else if (phase == Phase.TRANSIT && distToEst < 300.0) {
					phase = Phase.SEARCH;
					searchCenterX = estX;
					searchCenterY = estY;
					searchCenterZ = estZ;
					searchRadius = 220.0;
					searchAngle = GeminiMath.normalizeBearing(heading);
				}
			}
		}

		// 5. Compute desired 3D intercept / aim point
		Vec3 aimPoint;
		double desiredSpeed = 23.0;

		switch (phase) {
		case DOGLEG -> {
			double wpZ = clampOperationalDepth(mission.waypointX(), mission.waypointY(), Math.min(estZ, pos.z()), 20.0);
			aimPoint = new Vec3(mission.waypointX(), mission.waypointY(), wpZ);
		}
		case TRANSIT -> {
			double leadCap = Math.min(22.0, distToEst / Math.max(speed, 14.0));
			aimPoint = GeminiMath.solveIntercept3D(pos.x(), pos.y(), pos.z(), curX, curY, Math.max(speed, 19.0), estX,
					estY, estZ, estVx, estVy, 0.0, leadCap);
		}
		case HOMING, TERMINAL -> {
			double effectiveSpeed = Math.max(speed, 15.0);
			double maxLead = (phase == Phase.TERMINAL) ? Math.min(12.0, distToEst / effectiveSpeed * 1.15)
					: Math.min(26.0, distToEst / effectiveSpeed * 1.15);
			double closeTaper = distToEst < 55.0 ? Math.clamp((distToEst - 15.0) / 40.0, 0.25, 1.0) : 1.0;
			double velTrust = (activeFixCount >= 3 ? 1.0 : (activeFixCount == 2 ? 0.80 : 0.45)) * closeTaper;
			aimPoint = GeminiMath.solveIntercept3D(pos.x(), pos.y(), pos.z(), curX, curY, effectiveSpeed, estX, estY,
					estZ, estVx * velTrust, estVy * velTrust, estVz * velTrust, maxLead);
		}
		case REATTACK -> {
			aimPoint = GeminiMath.solveIntercept3D(pos.x(), pos.y(), pos.z(), curX, curY, Math.max(speed, 11.0), estX,
					estY, estZ, estVx * 0.65, estVy * 0.65, estVz * 0.5, 8.0);
			desiredSpeed = 7.5;
		}
		case SEARCH -> {
			if (Double.isFinite(lastPassiveBearing) && (tick - lastPassiveTick) * dt < 3.0) {
				double sDist = 450.0;
				double sx = pos.x() + Math.sin(lastPassiveBearing) * sDist;
				double sy = pos.y() + Math.cos(lastPassiveBearing) * sDist;
				if (isInLauncherHemisphere(sx, sy) || GeminiMath.dist2D(sx, sy, launcherX, launcherY) < 600.0) {
					sx = searchCenterX + launchDirX * 300.0;
					sy = searchCenterY + launchDirY * 300.0;
				}
				double sz = clampOperationalDepth(sx, sy, searchCenterZ, 20.0);
				aimPoint = new Vec3(sx, sy, sz);
				desiredSpeed = 16.5;
			} else {
				searchAngle = GeminiMath.normalizeBearing(searchAngle + searchDirection * 0.09 * dt);
				searchRadius = Math.min(1100.0, searchRadius + 15.0 * dt);
				double sx = searchCenterX + Math.sin(searchAngle) * searchRadius;
				double sy = searchCenterY + Math.cos(searchAngle) * searchRadius;
				if (isInLauncherHemisphere(sx, sy) || GeminiMath.dist2D(sx, sy, launcherX, launcherY) < 650.0) {
					sx = searchCenterX + launchDirX * Math.max(300.0, searchRadius * 0.6);
					sy = searchCenterY + launchDirY * Math.max(300.0, searchRadius * 0.6);
				}
				if (config != null && config.battleArea().distanceToBoundary(sx, sy) < 300.0) {
					double toCenter = GeminiMath.bearing(sx, sy, 0.0, 0.0);
					sx += Math.sin(toCenter) * 400.0;
					sy += Math.cos(toCenter) * 400.0;
				}
				double sz = clampOperationalDepth(sx, sy, searchCenterZ, 20.0);
				aimPoint = new Vec3(sx, sy, sz);
				desiredSpeed = 15.0;
			}
		}
		default -> aimPoint = new Vec3(estX, estY, estZ);
		}

		// 6. True 3D Slope-Pitch Terrain Avoidance (ZERO ridge crashes during transit!)
		double minClearance = (phase == Phase.TERMINAL && distToEst < 180.0) ? 5.5 : 18.0;
		double desiredBearing = GeminiMath.bearing(pos.x(), pos.y(), aimPoint.x(), aimPoint.y());
		desiredBearing = avoidObstaclesHorizontally(pos, heading, desiredBearing, minClearance);

		double horizDist = Math.max(GeminiMath.dist2D(pos.x(), pos.y(), aimPoint.x(), aimPoint.y()), 1.0);
		double clampedAimZ = Math.clamp(aimPoint.z(), -650.0, -3.5);
		double targetPitch = Math.atan2(clampedAimZ - pos.z(),
				(phase == Phase.TERMINAL && distToEst < 260.0) ? Math.max(horizDist, 12.0) : Math.max(horizDist, 60.0));

		// Compute exact minimum climb pitch required to clear EVERY ridge sample along forward path!
		double minTerrainPitch = computeMinTerrainClimbPitch(pos, heading, desiredBearing, speed, minClearance);
		double desiredPitch = Math.clamp(Math.max(targetPitch, minTerrainPitch), -Math.toRadians(54),
				Math.toRadians(54));

		// Prevent broaching surface
		if (pos.z() > -12.0 && desiredPitch > 0) {
			desiredPitch *= Math.clamp((-pos.z() - 3.0) / 9.0, 0.0, 1.0);
		}

		double floorNow = terrain.elevationAt(pos.x(), pos.y());
		boolean urgentClimb = (pos.z() - floorNow) < 12.0 || minTerrainPitch > Math.toRadians(22);

		// 7. Optimal zero-overshoot rudder and planes commands
		double headingError = GeminiMath.angleDiff(desiredBearing, heading);
		double pitchError = desiredPitch - pitch;

		double maxYawRate = GeminiMath.torpPeakYawRate(speed);
		double maxPitchRate = GeminiMath.torpPeakPitchRate(speed);

		double rudderCmd = GeminiMath.computeOptimalAxisCommand(headingError, yawRate, maxYawRate,
				GeminiMath.TORP_YAW_TAU, GeminiMath.TORP_MAX_RUDDER, Math.toRadians(12));
		double planesCmd = GeminiMath.computeOptimalAxisCommand(pitchError, pitchRate, maxPitchRate,
				GeminiMath.TORP_PITCH_TAU, GeminiMath.TORP_MAX_PLANES, Math.toRadians(9));

		double cosP = Math.cos(pitch);
		double combined = Math.hypot(rudderCmd * cosP, planesCmd);
		double budget = GeminiMath.TORP_MAX_RUDDER;
		if (combined > budget) {
			if (urgentClimb) {
				double rem = Math.sqrt(Math.max(0.0, budget * budget - planesCmd * planesCmd));
				rudderCmd = Math.copySign(Math.min(Math.abs(rudderCmd), cosP > 0.1 ? rem / cosP : budget), rudderCmd);
			} else {
				double scale = budget / combined;
				rudderCmd *= scale;
				planesCmd *= scale;
			}
		}

		// 8. Speed / fuel management
		double absHeadingErr = Math.abs(headingError);
		if (phase != Phase.REATTACK && phase != Phase.SEARCH) {
			if (distToEst < 380.0 && absHeadingErr > Math.toRadians(42)) {
				desiredSpeed = 7.5;
			} else if (distToEst < 850.0 && absHeadingErr > Math.toRadians(65)) {
				desiredSpeed = 10.0;
			} else {
				desiredSpeed = 23.0;
			}
		}

		double throttleCmd;
		if (speed < 6.0) {
			throttleCmd = 1.0;
		} else if (desiredSpeed >= 22.0) {
			throttleCmd = 1.0;
		} else if (speed > desiredSpeed + 0.8) {
			throttleCmd = 0.0;
		} else if (speed < desiredSpeed - 0.5) {
			throttleCmd = 1.0;
		} else {
			throttleCmd = Math.clamp((desiredSpeed / 23.0) * (desiredSpeed / 23.0), 0.2, 1.0);
		}

		// 9. Emergency manual detonation fallback
		if (flightTime > 6.0 && distFromLauncher > 450.0 && !isInLauncherHemisphere(pos.x(), pos.y()) && freshActive) {
			boolean dyingNextToTarget = (input.fuelRemaining() < 0.6 || speed < 3.6) && lastActiveRange < 38.0;
			boolean lateMissedPass = input.fuelRemaining() < 14.0 && prevActiveRange < 18.0
					&& lastActiveRange > prevActiveRange + 1.5 && lastActiveRange < 24.0;
			if (dyingNextToTarget || lateMissedPass) {
				output.detonate();
			}
		}

		output.setRudder(rudderCmd);
		output.setSternPlanes(planesCmd);
		output.setThrottle(throttleCmd);

		double pubZ = Math.max(clampedAimZ, pos.z() + Math.tan(desiredPitch) * Math.min(horizDist, 200.0));
		output.publishTarget(aimPoint.x(), aimPoint.y(), Math.clamp(pubZ, -650.0, -3.5));
		double estHdg = Math.hypot(estVx, estVy) > 0.3 ? GeminiMath.normalizeBearing(Math.atan2(estVx, estVy))
				: Double.NaN;
		output.publishDiagnostics(estX, estY, estZ, estHdg, Math.hypot(estVx, estVy), aimPoint.x(), aimPoint.y(),
				Math.clamp(pubZ, -650.0, -3.5), phase.name());
	}

	private boolean isInLauncherHemisphere(double x, double y) {
		if (initialTargetDist < 850.0) {
			return GeminiMath.dist2D(x, y, launchPos.x(), launchPos.y()) < 340.0;
		}
		double proj = (x - launchPos.x()) * launchDirX + (y - launchPos.y()) * launchDirY;
		return proj < 0.32 * initialTargetDist;
	}

	private void processActiveReturns(long tick, double dt, double flightTime, Vec3 pos, List<SonarContact> returns) {
		if (returns == null || returns.isEmpty())
			return;

		// Convert all valid active returns to world 3D points
		record Obs(SonarContact c, double x, double y, double z, double distToPred, double distToLauncher) {
		}
		var obsList = new java.util.ArrayList<Obs>(returns.size());
		for (var c : returns) {
			if (!Double.isFinite(c.range()) || c.range() <= 0)
				continue;
			double mz = Double.isFinite(c.estimatedDepth()) ? c.estimatedDepth() : estZ;
			double dz = mz - pos.z();
			double hRange = Math.sqrt(Math.max(0.0, c.range() * c.range() - dz * dz));
			double mx = pos.x() + Math.sin(c.bearing()) * hRange;
			double my = pos.y() + Math.cos(c.bearing()) * hRange;
			obsList.add(new Obs(c, mx, my, mz, GeminiMath.dist3D(mx, my, mz, estX, estY, estZ),
					GeminiMath.dist2D(mx, my, launcherX, launcherY)));
		}
		if (obsList.isEmpty())
			return;

		// Dual-return disambiguation: if we see BOTH submarines, update launcher position directly!
		if (obsList.size() >= 2) {
			Obs a = obsList.get(0);
			Obs b = obsList.get(1);
			// Assign whichever pairing minimizes (enemyDistToPred + launcherDistToLauncher)
			double costAB = a.distToPred() + b.distToLauncher();
			double costBA = b.distToPred() + a.distToLauncher();
			Obs launcherObs = (costAB <= costBA) ? b : a;
			double elapsed = lastActiveTick >= 0 ? Math.max(0.5, (tick - lastActiveTick) * dt) : 1.0;
			double ldx = launcherObs.x() - launcherX;
			double ldy = launcherObs.y() - launcherY;
			if (Math.hypot(ldx, ldy) < 500.0) {
				launcherVx = 0.5 * launcherVx + 0.5 * Math.clamp(ldx / elapsed, -15.0, 15.0);
				launcherVy = 0.5 * launcherVy + 0.5 * Math.clamp(ldy / elapsed, -15.0, 15.0);
			}
			launcherX = 0.3 * launcherX + 0.7 * launcherObs.x();
			launcherY = 0.3 * launcherY + 0.7 * launcherObs.y();
		}

		double initialGate = Math.max(700.0, mission.uncertaintyRadius() * 1.8 + flightTime * 15.0);
		double lockGate = hasActiveLock ? Math.max(400.0, (tick - lastActiveTick) * dt * 20.0 + 160.0)
				: (everLockedEnemy ? Math.max(520.0, (tick - lastActiveTick) * dt * 18.0 + 200.0) : initialGate);

		Obs best = null;
		double bestScore = Double.POSITIVE_INFINITY;

		for (var o : obsList) {
			double distToLaunch = GeminiMath.dist2D(o.x(), o.y(), launchPos.x(), launchPos.y());
			double distToLauncher = GeminiMath.dist2D(o.x(), o.y(), launcherX, launcherY);

			// HARD ANTI-FRIENDLY-FIRE GATES:
			if (o.distToPred() > lockGate) {
				continue;
			}
			if (!everLockedEnemy && isInLauncherHemisphere(o.x(), o.y())) {
				continue;
			}
			// Never lock onto a contact sitting closer to our launcher than to the enemy track!
			if (distToLauncher < 550.0 && distToLauncher < o.distToPred() * 1.15) {
				continue;
			}
			if (flightTime < 60.0 && distToLaunch < 420.0 && o.distToPred() > distToLaunch * 1.1) {
				continue;
			}

			double score = o.distToPred() + (distToLauncher < 500.0 ? 2000.0 : 0.0);
			if (score < bestScore) {
				bestScore = score;
				best = o;
			}
		}

		if (best == null)
			return;

		prevActiveRange = lastActiveRange;
		lastActiveRange = best.c().range();

		double elapsed = lastActiveTick >= 0 ? (tick - lastActiveTick) * dt : 0.0;
		activeFixCount++;
		everLockedEnemy = true;

		if (!hasActiveLock || elapsed > 6.0 || activeFixCount == 1) {
			estX = best.x();
			estY = best.y();
			estZ = best.z();
			if (activeFixCount == 1 && Double.isFinite(mission.targetHeading()) && mission.targetSpeed() > 0) {
				double spd = best.c().estimatedSpeed() >= 0
						? 0.5 * mission.targetSpeed() + 0.5 * best.c().estimatedSpeed() : mission.targetSpeed();
				estVx = spd * Math.sin(mission.targetHeading());
				estVy = spd * Math.cos(mission.targetHeading());
				estVz = 0.0;
			}
			hasActiveLock = true;
		} else if (elapsed >= 0.4) {
			double r = best.c().range();
			double ex = best.x() - estX;
			double ey = best.y() - estY;
			double ez = best.z() - estZ;
			double errHoriz = Math.hypot(ex, ey);
			double expectedNoise = Math.max(6.0, r * 0.025);

			double maneuverBoost = errHoriz > 2.2 * expectedNoise ? 0.16 : 0.0;
			double alpha = Math.clamp((r < 500.0 ? 0.82 : (r < 1200.0 ? 0.66 : 0.50)) + maneuverBoost, 0.42, 0.92);
			double beta = Math.clamp((r < 500.0 ? 0.34 : (r < 1200.0 ? 0.26 : 0.18)) + maneuverBoost * 0.7, 0.14, 0.52);

			estX += alpha * ex;
			estY += alpha * ey;
			estZ += alpha * ez;

			if (activeFixCount == 2) {
				estVx = 0.30 * estVx + 0.70 * (ex / elapsed);
				estVy = 0.30 * estVy + 0.70 * (ey / elapsed);
				estVz = 0.65 * (ez / elapsed);
			} else {
				estVx += (beta / elapsed) * ex;
				estVy += (beta / elapsed) * ey;
				estVz += (beta * 0.75 / elapsed) * ez;
			}

			double hSpd = Math.hypot(estVx, estVy);
			if (hSpd > 16.0) {
				double s = 16.0 / hSpd;
				estVx *= s;
				estVy *= s;
			}
			estVz = Math.clamp(estVz, -5.0, 5.0);
		}

		lastActiveTick = tick;
	}

	private void processPassiveContacts(long tick, double flightTime, Vec3 pos, List<SonarContact> contacts) {
		if (contacts == null || contacts.isEmpty())
			return;
		double predBrg = GeminiMath.bearing(pos.x(), pos.y(), estX, estY);
		double launcherBrg = GeminiMath.bearing(pos.x(), pos.y(), launcherX, launcherY);
		double launchOriginBrg = GeminiMath.bearing(pos.x(), pos.y(), launchPos.x(), launchPos.y());

		SonarContact best = null;
		double bestDiff = Double.POSITIVE_INFINITY;
		for (var c : contacts) {
			if (c.classification() == SonarContact.Classification.TORPEDO)
				continue;
			if (Math.abs(GeminiMath.angleDiff(c.bearing(), launcherBrg)) < Math.toRadians(22)
					&& Math.abs(GeminiMath.angleDiff(c.bearing(), predBrg)) > Math.toRadians(28)) {
				continue;
			}
			if (flightTime < 45.0 && Math.abs(GeminiMath.angleDiff(c.bearing(), launchOriginBrg)) < Math.toRadians(22)
					&& Math.abs(GeminiMath.angleDiff(c.bearing(), predBrg)) > Math.toRadians(28)) {
				continue;
			}
			double diff = Math.abs(GeminiMath.angleDiff(c.bearing(), predBrg));
			if (diff < bestDiff) {
				bestDiff = diff;
				best = c;
			}
		}
		if (best != null && (bestDiff < Math.toRadians(38) || phase == Phase.SEARCH)) {
			lastPassiveBearing = best.bearing();
			lastPassiveTick = tick;
		}
	}

	/**
	 * Computes the steepest upward pitch angle required to clear all terrain along both current
	 * heading and desired heading out to 360m, accounting for pitch response lag.
	 */
	private double computeMinTerrainClimbPitch(
		Vec3 pos, double heading, double desiredBearing, double speed, double minClearance) {
		double maxReqPitch = -Math.PI / 2;
		double sinH = Math.sin(heading), cosH = Math.cos(heading);
		double sinD = Math.sin(desiredBearing), cosD = Math.cos(desiredBearing);
		double maxDist = Math.max(240.0, speed * 16.0);

		for (double d = 25.0; d <= maxDist; d += 30.0) {
			double fH = terrain.elevationAt(pos.x() + sinH * d, pos.y() + cosH * d);
			double fD = terrain.elevationAt(pos.x() + sinD * d, pos.y() + cosD * d);
			double worst = Math.max(fH, fD);
			// Lead distance slightly (subtract 25m) to account for 6s pitch lag!
			double effectiveD = Math.max(15.0, d - 25.0);
			double reqZ = worst + minClearance;
			if (reqZ > pos.z() - 40.0) {
				double reqPitch = Math.atan2(reqZ - pos.z(), effectiveD);
				if (reqPitch > maxReqPitch) {
					maxReqPitch = reqPitch;
				}
			}
		}
		return maxReqPitch;
	}

	private double avoidObstaclesHorizontally(Vec3 pos, double heading, double desiredBearing, double minClearance) {
		// Check if an island (floor > -12m) OR an un-climbable steep underwater cliff (> 38 deg slope) blocks desiredBearing
		boolean blocked = false;
		double sinD = Math.sin(desiredBearing), cosD = Math.cos(desiredBearing);
		for (double d = 35.0; d <= 280.0; d += 40.0) {
			double f = terrain.elevationAt(pos.x() + sinD * d, pos.y() + cosD * d);
			if (f > -12.0 || Math.atan2((f + minClearance) - pos.z(), Math.max(15.0, d - 25.0)) > Math.toRadians(38)) {
				blocked = true;
				break;
			}
		}
		if (!blocked)
			return desiredBearing;

		double bestBrg = desiredBearing;
		double bestScore = -Double.MAX_VALUE;
		for (int deg = -105; deg <= 105; deg += 15) {
			if (deg == 0)
				continue;
			double cand = GeminiMath.normalizeBearing(desiredBearing + Math.toRadians(deg));
			double sinC = Math.sin(cand), cosC = Math.cos(cand);
			double worstFloor = -1000.0;
			double worstSlope = -Math.PI / 2;
			for (double d = 35.0; d <= 320.0; d += 45.0) {
				double f = terrain.elevationAt(pos.x() + sinC * d, pos.y() + cosC * d);
				worstFloor = Math.max(worstFloor, f);
				worstSlope = Math.max(worstSlope, Math.atan2((f + minClearance) - pos.z(), Math.max(15.0, d - 25.0)));
			}
			boolean passable = worstFloor < -15.0 && worstSlope < Math.toRadians(32);
			double score = (passable ? 600.0 : -worstSlope * 400.0 - Math.max(0.0, worstFloor + 20.0) * 15.0)
					- Math.abs(deg) * 2.2 - Math.abs(Math.toDegrees(GeminiMath.angleDiff(cand, heading))) * 0.5;
			if (score > bestScore) {
				bestScore = score;
				bestBrg = cand;
			}
		}
		return bestBrg;
	}

	private double clampOperationalDepth(double x, double y, double z, double clearance) {
		double floor = terrain.elevationAt(x, y);
		return Math.clamp(Math.max(z, floor + clearance), -650.0, -5.0);
	}
}
