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
package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.*;

public final class CodexTorpedoController implements TorpedoController {
	private static final double MIN_TERRAIN_CLEARANCE = 35.0;
	private static final double MAX_CRUISE_DEPTH = -30.0;
	private static final double MAX_TARGET_DEPTH = -650.0;
	private static final double MIN_INTERCEPT_DEPTH_DISTANCE = 60.0;
	private static final double LONG_RANGE_SHALLOW_CRUISE_FACTOR = 0.40;
	private static final double LONG_RANGE_SHALLOW_MIN_DEPTH = -140.0;
	private static final double LONG_RANGE_SHALLOW_MAX_DEPTH = -55.0;
	private static final double LONG_RANGE_DEPTH_BLEND_START = 1_600.0;
	private static final double LONG_RANGE_DEPTH_BLEND_END = 450.0;
	private static final double TERMINAL_TRACK_RANGE = 650.0;
	private static final double TERMINAL_COMMIT_RANGE = 260.0;
	private static final double TERMINAL_HOLD_RANGE = 90.0;
	private static final double TERMINAL_TRACK_THROTTLE = 0.72;
	private static final double TERMINAL_COMMIT_THROTTLE = 0.52;
	private static final double TERMINAL_FINE_THROTTLE = 0.38;
	private static final long STRAIGHT_RUN_TICKS = 300L;
	private static final double STRAIGHT_RUN_DISTANCE = 450.0;
	private static final double STRAIGHT_RUN_RUDDER_LIMIT = 0.22;
	private static final double ACTIVE_SONAR_ARM_DISTANCE = 850.0;
	private static final long ACTIVE_FIX_STALE_TICKS = 750L;
	private static final double MAX_ESTIMATED_TARGET_SPEED = 14.0;
	private static final double MAX_CLOSE_ESTIMATED_TARGET_SPEED = 10.0;
	private static final double OWNER_AVOIDANCE_RADIUS = 850.0;
	private static final double OWNER_REJECTION_RUN_DISTANCE = 1_200.0;
	private static final double TARGET_GATE_RADIUS = 1_400.0;
	// Fins stall above 30 degrees; outputs are mapped to +/-45 degrees by the vehicle.
	private static final double MAX_FIN_COMMAND = 0.65;

	private TerrainMap terrain;
	private boolean hasTarget;
	private double targetX = Double.NaN;
	private double targetY = Double.NaN;
	private double targetZ = -100.0;
	private double estVelX = 0.0;
	private double estVelY = 0.0;
	private double prevFixX = Double.NaN;
	private double prevFixY = Double.NaN;
	private long prevFixTick = Long.MIN_VALUE / 4;
	private double passiveRangeEstimate = Double.NaN;
	private double launchX = Double.NaN;
	private double launchY = Double.NaN;
	private double launchHeading = Double.NaN;
	private long launchTick = Long.MIN_VALUE / 4;
	private double straightRunDistance = STRAIGHT_RUN_DISTANCE;
	private double initialTargetRange = Double.POSITIVE_INFINITY;
	private long currentTick;
	private long targetStateTick = Long.MIN_VALUE / 4;
	private double pitchErrorIntegral = 0.0;
	private double lastPitch = 0.0;
	private boolean hasLastPitchSample;
	private double detonationRange = 12.0;
	private double safeScuttleRange = 200.0;

	@Override
	public void onLaunch(TorpedoLaunchContext context) {
		this.terrain = context.terrain();
		this.safeScuttleRange = Math.max(200.0,
				context.config().blastRadius() + VehicleConfig.submarine().hullHalfLength() + 80.0);
		this.launchX = context.launchPosition().x();
		this.launchY = context.launchPosition().y();
		this.launchHeading = context.launchHeading();
		this.pitchErrorIntegral = 0.0;
		this.lastPitch = 0.0;
		this.hasLastPitchSample = false;
		String missionData = context.missionData();
		if (missionData == null || missionData.isBlank()) {
			return;
		}

		try {
			var parts = missionData.contains(";") ? missionData.split(";") : missionData.split(",");
			if (parts.length >= 2) {
				targetX = Double.parseDouble(parts[0].trim());
				targetY = Double.parseDouble(parts[1].trim());
				targetZ = parts.length >= 3 ? Double.parseDouble(parts[2].trim()) : -100.0;
				if (parts.length >= 5) {
					double heading = Double.parseDouble(parts[3].trim());
					double speed = Double.parseDouble(parts[4].trim());
					if (!Double.isNaN(heading) && speed > 0.0) {
						estVelX = Math.sin(heading) * speed;
						estVelY = Math.cos(heading) * speed;
					}
				}
				if (parts.length >= 6) {
					detonationRange = Math.clamp(Double.parseDouble(parts[5].trim()), 5.0, 20.0);
				}
				hasTarget = true;
				initialTargetRange = Math.hypot(targetX - launchX, targetY - launchY);
				straightRunDistance = Math.clamp(initialTargetRange * 0.15, 180.0, STRAIGHT_RUN_DISTANCE);
			}
		} catch (NumberFormatException ignored) {
			hasTarget = false;
		}
	}

	@Override
	public void onTick(TorpedoInput input, TorpedoOutput output) {
		currentTick = input.tick();
		if (hasTarget && targetStateTick > Long.MIN_VALUE / 8) {
			double elapsed = Math.max(0.0, (input.tick() - targetStateTick) / 50.0);
			targetX += estVelX * elapsed;
			targetY += estVelY * elapsed;
		}
		targetStateTick = input.tick();
		var pos = input.self().position();
		double heading = input.self().heading();
		if (launchTick <= Long.MIN_VALUE / 8) {
			launchTick = input.tick();
		}
		double runDistance = Math.hypot(pos.x() - launchX, pos.y() - launchY);
		boolean straightRun = inStraightRun(runDistance, input.tick());
		double missionRange = hasTarget ? Math.hypot(targetX - pos.x(), targetY - pos.y()) : Double.POSITIVE_INFINITY;
		if (input.activeSonarCooldownTicks() == 0 && !straightRun
				&& (runDistance >= ACTIVE_SONAR_ARM_DISTANCE || missionRange < LONG_RANGE_DEPTH_BLEND_START)) {
			output.activeSonarPing();
		}
		boolean lostAcquiredTarget = prevFixTick > Long.MIN_VALUE / 8
				&& input.tick() - prevFixTick > ACTIVE_FIX_STALE_TICKS;
		boolean freshPing = !input.activeSonarReturns().isEmpty() || input.activeSonarCooldownTicks() >= 49;
		SonarContact fix = pickForwardContact(input.activeSonarReturns(), pos, heading);
		if (fix == null && lostAcquiredTarget && freshPing && runDistance > safeScuttleRange
				&& !input.activeSonarReturns().isEmpty()
				&& input.activeSonarReturns().stream().noneMatch(contact -> contact.range() < safeScuttleRange)) {
			output.detonate();
			return;
		}

		if (fix != null) {
			applyActiveFix(input.tick(), pos, fix);
		} else if (hasTarget && input.tick() - prevFixTick > 100L) {
			SonarContact passive = pickForwardContact(input.sonarContacts(), pos, heading);
			if (passive != null) {
				applyPassiveContact(pos, passive);
			}
		}

		if (!hasTarget) {
			output.setRudder(0.0);
			output.setSternPlanes(0.0);
			return;
		}

		double speed = Math.max(input.speed(), 8.0);
		double targetDist = Math.hypot(targetX - pos.x(), targetY - pos.y());
		double[] intercept = predictedIntercept(pos.x(), pos.y(), speed, input.tick());
		double interceptX = intercept[0];
		double interceptY = intercept[1];
		double guidanceX = straightRun ? launchX + Math.sin(launchHeading) * straightRunDistance : interceptX;
		double guidanceY = straightRun ? launchY + Math.cos(launchHeading) * straightRunDistance : interceptY;
		double dx = guidanceX - pos.x();
		double dy = guidanceY - pos.y();
		double bearingToIntercept = normalize(Math.atan2(dx, dy));

		double headingError = bearingToIntercept - heading;
		while (headingError > Math.PI) {
			headingError -= 2.0 * Math.PI;
		}
		while (headingError < -Math.PI) {
			headingError += 2.0 * Math.PI;
		}
		double rudderGain = straightRun ? 1.2
				: targetDist < TERMINAL_COMMIT_RANGE ? 1.2 : targetDist < TERMINAL_TRACK_RANGE ? 1.45 : 1.5;
		double rudderLimit = straightRun ? STRAIGHT_RUN_RUDDER_LIMIT : MAX_FIN_COMMAND;
		output.setRudder(Math.clamp(headingError * rudderGain, -rudderLimit, rudderLimit));

		double missionDepth = Math.clamp(targetZ, MAX_TARGET_DEPTH, MAX_CRUISE_DEPTH);
		double desiredZ = desiredDepthForRange(missionDepth, targetDist);
		if (targetDist < TERMINAL_HOLD_RANGE && Math.abs(desiredZ - pos.z()) < 12.0) {
			desiredZ = pos.z();
		}

		double depthControlRange = targetDist;
		if (terrain != null) {
			double floor = terrain.elevationAt(pos.x(), pos.y());
			double safeZ = floor + MIN_TERRAIN_CLEARANCE;
			double lookAhead = Math.max(input.speed(), 5.0) * 15.0;
			for (int sample = 1; sample <= 6; sample++) {
				double sampleRange = lookAhead * sample / 6.0;
				double aheadFloor = terrain.elevationAt(pos.x() + Math.sin(heading) * sampleRange,
						pos.y() + Math.cos(heading) * sampleRange);
				double guidanceFloor = terrain.elevationAt(pos.x() + Math.sin(bearingToIntercept) * sampleRange,
						pos.y() + Math.cos(bearingToIntercept) * sampleRange);
				safeZ = Math.max(safeZ, Math.max(aheadFloor, guidanceFloor) + MIN_TERRAIN_CLEARANCE + 10.0);
			}
			if (desiredZ < safeZ) {
				desiredZ = safeZ;
				depthControlRange = Math.min(targetDist, lookAhead);
			}
		}

		desiredZ = Math.min(desiredZ, MAX_CRUISE_DEPTH);
		output.setThrottle(computeThrottle(input, targetDist, headingError, desiredZ, straightRun));
		output.setSternPlanes(computeSternPlanes(input, desiredZ, depthControlRange));

		if (fix != null && fix.range() < detonationRange) {
			output.detonate();
		}
		output.publishTarget(guidanceX, guidanceY, targetZ);
	}

	private void applyActiveFix(long tick, se.hirt.searobots.api.Vec3 pos, SonarContact fix) {
		double horizontalRange = horizontalRange(fix, pos);
		double fixX = pos.x() + Math.sin(fix.bearing()) * horizontalRange;
		double fixY = pos.y() + Math.cos(fix.bearing()) * horizontalRange;
		double acceptedX = fixX;
		double acceptedY = fixY;

		if (hasTarget && prevFixTick > Long.MIN_VALUE / 8) {
			double dt = Math.max(0.02, (tick - prevFixTick) / 50.0);
			double projectedX = targetX;
			double projectedY = targetY;
			double jump = Math.hypot(fixX - projectedX, fixY - projectedY);
			double allowedJump = allowedFixJump(fix.range(), dt);
			double blend = activeFixBlend(fix.range(), jump, allowedJump);
			acceptedX = projectedX + (fixX - projectedX) * blend;
			acceptedY = projectedY + (fixY - projectedY) * blend;
		}

		updateVelocityEstimate(tick, acceptedX, acceptedY, fix.range());
		targetX = acceptedX;
		targetY = acceptedY;
		passiveRangeEstimate = horizontalRange;
		if (!Double.isNaN(fix.estimatedDepth())) {
			double fixDepth = Math.clamp(fix.estimatedDepth(), MAX_TARGET_DEPTH, MAX_CRUISE_DEPTH);
			double depthBlend = fix.range() < TERMINAL_HOLD_RANGE ? 1.0
					: fix.range() < TERMINAL_COMMIT_RANGE ? 0.85 : 0.65;
			targetZ = hasTarget ? targetZ + (fixDepth - targetZ) * depthBlend : fixDepth;
		}
		hasTarget = true;
	}

	private void applyPassiveContact(se.hirt.searobots.api.Vec3 pos, SonarContact passive) {
		double currentRange = Math.hypot(targetX - pos.x(), targetY - pos.y());
		if (!Double.isFinite(currentRange) || currentRange < 100.0) {
			return;
		}

		double passiveRange = currentRange;
		double passiveX = pos.x() + Math.sin(passive.bearing()) * passiveRange;
		double passiveY = pos.y() + Math.cos(passive.bearing()) * passiveRange;
		double passiveBlend = 0.03;
		targetX = targetX + (passiveX - targetX) * passiveBlend;
		targetY = targetY + (passiveY - targetY) * passiveBlend;
		passiveRangeEstimate = passiveRange;

		if (!Double.isNaN(passive.estimatedDepth()) && currentRange < TERMINAL_TRACK_RANGE) {
			double passiveDepth = Math.clamp(passive.estimatedDepth(), MAX_TARGET_DEPTH, MAX_CRUISE_DEPTH);
			double depthBlend = currentRange < TERMINAL_COMMIT_RANGE ? 0.18 : 0.08;
			targetZ = targetZ + (passiveDepth - targetZ) * depthBlend;
		}

		if (!Double.isNaN(passive.estimatedHeading()) && passive.estimatedSpeed() > 0.0) {
			double passiveSpeed = Math.min(passive.estimatedSpeed(), MAX_ESTIMATED_TARGET_SPEED);
			double passiveVelX = Math.sin(passive.estimatedHeading()) * passiveSpeed;
			double passiveVelY = Math.cos(passive.estimatedHeading()) * passiveSpeed;
			double velocityBlend = currentRange < TERMINAL_COMMIT_RANGE ? 0.10
					: currentRange < TERMINAL_TRACK_RANGE ? 0.14 : 0.20;
			estVelX = estVelX * (1.0 - velocityBlend) + passiveVelX * velocityBlend;
			estVelY = estVelY * (1.0 - velocityBlend) + passiveVelY * velocityBlend;
			clampVelocityEstimate(currentRange < TERMINAL_TRACK_RANGE ? MAX_CLOSE_ESTIMATED_TARGET_SPEED
					: MAX_ESTIMATED_TARGET_SPEED);
		}
	}

	private void updateVelocityEstimate(long tick, double fixX, double fixY, double range) {
		if (!Double.isNaN(prevFixX) && prevFixTick > Long.MIN_VALUE / 8) {
			double dt = (tick - prevFixTick) / 50.0;
			if (dt > 0.25 && dt < 15.0) {
				double rawVX = (fixX - prevFixX) / dt;
				double rawVY = (fixY - prevFixY) / dt;
				double rawSpeed = Math.hypot(rawVX, rawVY);
				double maxSpeed = range < TERMINAL_TRACK_RANGE ? MAX_CLOSE_ESTIMATED_TARGET_SPEED
						: MAX_ESTIMATED_TARGET_SPEED;
				if (rawSpeed > maxSpeed) {
					double scale = maxSpeed / rawSpeed;
					rawVX *= scale;
					rawVY *= scale;
				}
				double blend = range < TERMINAL_COMMIT_RANGE ? 0.08 : range < TERMINAL_TRACK_RANGE ? 0.18 : 0.45;
				estVelX = estVelX * (1.0 - blend) + rawVX * blend;
				estVelY = estVelY * (1.0 - blend) + rawVY * blend;
			}
		}
		clampVelocityEstimate(
				range < TERMINAL_TRACK_RANGE ? MAX_CLOSE_ESTIMATED_TARGET_SPEED : MAX_ESTIMATED_TARGET_SPEED);
		prevFixX = fixX;
		prevFixY = fixY;
		prevFixTick = tick;
	}

	private SonarContact pickForwardContact(
		java.util.List<SonarContact> contacts, se.hirt.searobots.api.Vec3 pos, double heading) {
		SonarContact best = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		for (SonarContact contact : contacts) {
			if (contact.isActive() && contact.range() <= 0.0) {
				continue;
			}
			double angleDiff = contact.bearing() - heading;
			while (angleDiff > Math.PI) {
				angleDiff -= 2.0 * Math.PI;
			}
			while (angleDiff < -Math.PI) {
				angleDiff += 2.0 * Math.PI;
			}
			double forwardness = Math.cos(angleDiff);
			double rangePenalty = contact.range() > 0.0 ? contact.range() * 0.01 : 0.0;
			if (!isPlausibleContact(contact, pos, heading)) {
				continue;
			}
			double score = forwardness * 100.0 + contact.signalExcess() - rangePenalty + targetClassBias(contact);
			if (hasTarget) {
				double contactRange = contact.range() > 0.0 ? horizontalRange(contact, pos)
						: Math.max(passiveRangeEstimate, 450.0);
				double cx = targetX;
				double cy = targetY;
				if (!Double.isNaN(contactRange)) {
					cx = pos.x() + Math.sin(contact.bearing()) * contactRange;
					cy = pos.y() + Math.cos(contact.bearing()) * contactRange;
				}
				double predictedError = Math.hypot(cx - targetX, cy - targetY);
				score -= predictedError * 0.03;
			}
			if (score > bestScore) {
				bestScore = score;
				best = contact;
			}
		}
		return best;
	}

	private double targetClassBias(SonarContact contact) {
		return switch (contact.classification()) {
		case SUBMARINE -> 180.0;
		case UNKNOWN -> 0.0;
		case SURFACE_SHIP -> -120.0;
		case TORPEDO -> -260.0;
		};
	}

	private double[] predictedIntercept(double ownX, double ownY, double ownSpeed, long tick) {
		double rx = targetX - ownX;
		double ry = targetY - ownY;
		double a = estVelX * estVelX + estVelY * estVelY - ownSpeed * ownSpeed;
		double b = 2.0 * (rx * estVelX + ry * estVelY);
		double c = rx * rx + ry * ry;
		double timeToIntercept = Math.sqrt(c) / ownSpeed;
		double discriminant = b * b - 4.0 * a * c;
		if (Math.abs(a) > 0.01 && discriminant >= 0.0) {
			double root = Math.sqrt(discriminant);
			double first = (-b - root) / (2.0 * a);
			double second = (-b + root) / (2.0 * a);
			if (first > 0.0) {
				timeToIntercept = first;
			}
			if (second > 0.0 && (first <= 0.0 || second < first)) {
				timeToIntercept = second;
			}
		}
		timeToIntercept = Math.clamp(timeToIntercept, 0.0, 60.0);
		if (tick - prevFixTick > ACTIVE_FIX_STALE_TICKS) {
			timeToIntercept *= 0.5;
		}
		return new double[] {targetX + estVelX * timeToIntercept, targetY + estVelY * timeToIntercept};
	}

	private double allowedFixJump(double range, double dtSeconds) {
		return Math.max(35.0, range * 0.08 + dtSeconds * MAX_ESTIMATED_TARGET_SPEED);
	}

	private double activeFixBlend(double range, double jump, double allowedJump) {
		if (range < TERMINAL_COMMIT_RANGE) {
			return jump > allowedJump ? 0.08 : 0.28;
		}
		if (range < TERMINAL_TRACK_RANGE) {
			return jump > allowedJump ? 0.18 : 0.45;
		}
		return jump > allowedJump ? 0.35 : 0.75;
	}

	private void clampVelocityEstimate(double maxSpeed) {
		double speed = Math.hypot(estVelX, estVelY);
		if (speed > maxSpeed && speed > 0.0) {
			double scale = maxSpeed / speed;
			estVelX *= scale;
			estVelY *= scale;
		}
	}

	private double computeSternPlanes(TorpedoInput input, double desiredZ, double targetDist) {
		var pos = input.self().position();
		double currentPitch = input.self().pitch();
		double depthError = desiredZ - pos.z();
		double speed = Math.max(input.speed(), 8.0);
		double responseLeadSeconds = targetDist < TERMINAL_COMMIT_RANGE ? 3.5
				: targetDist < TERMINAL_TRACK_RANGE ? 5.5 : 8.0;
		double effectiveDist = Math.max(targetDist - speed * responseLeadSeconds, MIN_INTERCEPT_DEPTH_DISTANCE);
		double desiredPitch = Math.atan2(depthError, effectiveDist);
		double maxDesiredPitch = targetDist < TERMINAL_COMMIT_RANGE ? Math.toRadians(55.0)
				: targetDist < TERMINAL_TRACK_RANGE ? Math.toRadians(46.0) : Math.toRadians(40.0);
		desiredPitch = Math.clamp(desiredPitch, -maxDesiredPitch, maxDesiredPitch);

		double pitchError = desiredPitch - currentPitch;
		double dt = Math.max(input.deltaTimeSeconds(), 0.01);
		if (hasLastPitchSample && Math.signum(pitchError) != Math.signum(desiredPitch - lastPitch)) {
			pitchErrorIntegral *= 0.35;
		}
		pitchErrorIntegral += pitchError * dt;
		double integralLimit = targetDist < TERMINAL_COMMIT_RANGE ? 0.26
				: targetDist < TERMINAL_TRACK_RANGE ? 0.18 : 0.10;
		pitchErrorIntegral = Math.clamp(pitchErrorIntegral, -integralLimit, integralLimit);

		double measuredPitchRate = hasLastPitchSample ? (currentPitch - lastPitch) / dt : 0.0;
		lastPitch = currentPitch;
		hasLastPitchSample = true;

		double proportionalGain = targetDist < TERMINAL_COMMIT_RANGE ? 5.6
				: targetDist < TERMINAL_TRACK_RANGE ? 4.4 : 2.8;
		double integralGain = targetDist < TERMINAL_COMMIT_RANGE ? 1.8 : targetDist < TERMINAL_TRACK_RANGE ? 1.1 : 0.45;
		double derivativeGain = targetDist < TERMINAL_COMMIT_RANGE ? 0.32
				: targetDist < TERMINAL_TRACK_RANGE ? 0.52 : 0.90;
		double feedForward = targetDist < TERMINAL_COMMIT_RANGE ? Math.clamp(depthError / 120.0, -0.25, 0.25)
				: targetDist < TERMINAL_TRACK_RANGE ? Math.clamp(depthError / 180.0, -0.18, 0.18)
						: Math.clamp(depthError / 260.0, -0.10, 0.10);
		double output = pitchError * proportionalGain + pitchErrorIntegral * integralGain
				- measuredPitchRate * derivativeGain + feedForward;
		double outputLimit = MAX_FIN_COMMAND;
		return Math.clamp(output, -outputLimit, outputLimit);
	}

	private double computeThrottle(
		TorpedoInput input, double targetDist, double headingError, double desiredZ, boolean straightRun) {
		if (straightRun || targetDist > TERMINAL_TRACK_RANGE) {
			return 1.0;
		}
		if (input.tick() - prevFixTick > ACTIVE_FIX_STALE_TICKS) {
			return 1.0;
		}

		var pos = input.self().position();
		double currentPitch = input.self().pitch();
		double projectedDepthAtTarget = pos.z() + Math.sin(currentPitch) * targetDist;
		double interceptDepthError = desiredZ - projectedDepthAtTarget;
		double absHeadingErrorDeg = Math.toDegrees(Math.abs(headingError));
		double absInterceptDepthError = Math.abs(interceptDepthError);

		if (targetDist < TERMINAL_HOLD_RANGE) {
			if (absHeadingErrorDeg > 4.0 || absInterceptDepthError > 12.0) {
				return TERMINAL_FINE_THROTTLE;
			}
			return 0.82;
		}
		if (targetDist < TERMINAL_COMMIT_RANGE) {
			if (absHeadingErrorDeg > 8.0 || absInterceptDepthError > 25.0) {
				return TERMINAL_COMMIT_THROTTLE;
			}
			if (absHeadingErrorDeg > 4.0 || absInterceptDepthError > 12.0) {
				return 0.68;
			}
			return 0.88;
		}
		if (absHeadingErrorDeg > 14.0 || absInterceptDepthError > 45.0) {
			return TERMINAL_TRACK_THROTTLE;
		}
		return 1.0;
	}

	private boolean isPlausibleContact(SonarContact contact, se.hirt.searobots.api.Vec3 pos, double heading) {
		if (!hasTarget) {
			return true;
		}
		double rangeGuess = contact.range() > 0.0 ? horizontalRange(contact, pos)
				: Double.isNaN(passiveRangeEstimate) ? 500.0 : Math.max(passiveRangeEstimate, 300.0);
		double cx = pos.x() + Math.sin(contact.bearing()) * rangeGuess;
		double cy = pos.y() + Math.cos(contact.bearing()) * rangeGuess;
		double fromLaunch = Math.hypot(cx - launchX, cy - launchY);
		double fromTarget = Math.hypot(cx - targetX, cy - targetY);
		double runDistance = Math.hypot(pos.x() - launchX, pos.y() - launchY);
		double ownerExclusion = Math.min(OWNER_AVOIDANCE_RADIUS, initialTargetRange * 0.35);
		if (contact.isActive() && prevFixTick <= Long.MIN_VALUE / 8 && fromLaunch < ownerExclusion) {
			return false;
		}
		if (prevFixTick > Long.MIN_VALUE / 8) {
			double age = Math.max(0.0, (currentTick - prevFixTick) / 50.0);
			double expectedX = prevFixX + estVelX * age;
			double expectedY = prevFixY + estVelY * age;
			if (contact.isActive()) {
				double gateRadius = Math.min(600.0,
						Math.max(150.0, contact.range() * 0.10 + Math.min(age, 3.0) * MAX_ESTIMATED_TARGET_SPEED));
				if (Math.hypot(cx - expectedX, cy - expectedY) > gateRadius) {
					return false;
				}
			} else {
				double expectedBearing = Math.atan2(expectedX - pos.x(), expectedY - pos.y());
				double bearingError = Math.atan2(Math.sin(contact.bearing() - expectedBearing),
						Math.cos(contact.bearing() - expectedBearing));
				double gateAngle = Math.max(Math.toRadians(20.0), contact.bearingUncertainty() * 3.0);
				if (Math.abs(bearingError) > gateAngle) {
					return false;
				}
			}
		}
		double headingDiff = heading - launchHeading;
		while (headingDiff > Math.PI)
			headingDiff -= 2.0 * Math.PI;
		while (headingDiff < -Math.PI)
			headingDiff += 2.0 * Math.PI;
		boolean stillOutbound = Math.abs(headingDiff) < Math.toRadians(70.0);
		boolean ownerDangerZone = fromLaunch < OWNER_AVOIDANCE_RADIUS && runDistance < OWNER_REJECTION_RUN_DISTANCE;
		if ((stillOutbound || ownerDangerZone) && fromTarget > TARGET_GATE_RADIUS) {
			return false;
		}
		return fromTarget <= TARGET_GATE_RADIUS || contact.signalExcess() > 18.0;
	}

	private boolean inStraightRun(double runDistance, long tick) {
		return tick - launchTick < STRAIGHT_RUN_TICKS || runDistance < straightRunDistance;
	}

	private double horizontalRange(SonarContact contact, se.hirt.searobots.api.Vec3 pos) {
		if (!contact.isActive() || !Double.isFinite(contact.estimatedDepth())) {
			return contact.range();
		}
		double depthDifference = contact.estimatedDepth() - pos.z();
		return Math.sqrt(Math.max(0.0, contact.range() * contact.range() - depthDifference * depthDifference));
	}

	private double normalize(double bearing) {
		double result = bearing % (2.0 * Math.PI);
		if (result < 0.0) {
			result += 2.0 * Math.PI;
		}
		return result;
	}

	private double desiredDepthForRange(double missionDepth, double targetDist) {
		double shallowCruiseDepth = Math.clamp(missionDepth * LONG_RANGE_SHALLOW_CRUISE_FACTOR,
				LONG_RANGE_SHALLOW_MIN_DEPTH, LONG_RANGE_SHALLOW_MAX_DEPTH);
		if (targetDist >= LONG_RANGE_DEPTH_BLEND_START) {
			return shallowCruiseDepth;
		}
		if (targetDist <= LONG_RANGE_DEPTH_BLEND_END) {
			return missionDepth;
		}
		double blend = 1.0 - (targetDist - LONG_RANGE_DEPTH_BLEND_END)
				/ (LONG_RANGE_DEPTH_BLEND_START - LONG_RANGE_DEPTH_BLEND_END);
		blend = blend * blend * (3.0 - 2.0 * blend);
		return shallowCruiseDepth + (missionDepth - shallowCruiseDepth) * blend;
	}
}
