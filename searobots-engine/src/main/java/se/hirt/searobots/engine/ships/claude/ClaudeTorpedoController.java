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
package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.*;

/**
 * Claude's torpedo controller. Three-phase approach:
 * <ol>
 *   <li><b>Transit:</b> Head toward mission data target at full throttle.
 *       No pinging. Use passive sonar bearings to refine heading.</li>
 *   <li><b>Acquisition:</b> Near target area, start pinging. Use consecutive
 *       fixes to estimate target velocity. Steer toward intercept point.</li>
 *   <li><b>Terminal:</b> Within 800m, slow down for tight turning.
 *       True proportional navigation using line-of-sight rate.</li>
 * </ol>
 */
public class ClaudeTorpedoController implements TorpedoController {

	// Mission data from launch
	private double targetX, targetY, targetZ;
	private double targetHeading = Double.NaN, targetSpeed = 5.0;
	private boolean hasTarget;
	private TerrainMap terrain;

	// Target tracking from active pings
	private double prevFixX = Double.NaN, prevFixY = Double.NaN;
	private long prevFixTick = -1;
	private double estVelX = 0, estVelY = 0;
	private double lastActiveRange = Double.MAX_VALUE; // closest active return range

	// Phase management
	private enum Phase {TRANSIT, ACQUISITION, TERMINAL}

	private Phase phase = Phase.TRANSIT;

	private static final double MIN_TERRAIN_CLEARANCE = 30;
	private static final double ACQUISITION_RANGE = 2500;
	private static final double TERMINAL_RANGE = 800;
	// Proportional navigation: track the intercept-point bearing rate for LOS damping
	private double prevBearingToIntercept = Double.NaN;
	private long prevBearingTick = -1;

	// PID depth controller
	private double depthIntegral;
	private double prevDepthError;
	private double launchDepth = Double.NaN; // remember where we started

	@Override
	public void onLaunch(TorpedoLaunchContext context) {
		this.terrain = context.terrain();
		this.launchDepth = context.launchPosition().z();
		String data = context.missionData();
		if (data != null && !data.isBlank()) {
			try {
				var parts = data.split(",");
				if (parts.length >= 2) {
					targetX = Double.parseDouble(parts[0].trim());
					targetY = Double.parseDouble(parts[1].trim());
					targetZ = parts.length >= 3 ? Double.parseDouble(parts[2].trim()) : -100;
					if (parts.length >= 4) {
						targetHeading = Double.parseDouble(parts[3].trim());
					}
					if (parts.length >= 5) {
						targetSpeed = Double.parseDouble(parts[4].trim());
					}
					hasTarget = true;

					// Pre-compute initial velocity estimate from mission data
					if (!Double.isNaN(targetHeading) && targetSpeed > 0) {
						estVelX = targetSpeed * Math.sin(targetHeading);
						estVelY = targetSpeed * Math.cos(targetHeading);
					}
				}
			} catch (NumberFormatException e) {
				// Could not parse
			}
		}
	}

	@Override
	public void onTick(TorpedoInput input, TorpedoOutput output) {
		var pos = input.self().position();
		double heading = input.self().heading();

		if (!hasTarget) {
			output.setThrottle(1.0);
			output.setRudder(0);
			output.setSternPlanes(0);
			return;
		}

		// Distance to current target estimate
		double dx = targetX - pos.x();
		double dy = targetY - pos.y();
		double dist = Math.sqrt(dx * dx + dy * dy);

		// Phase transitions
		if (phase == Phase.TRANSIT && dist < ACQUISITION_RANGE) {
			phase = Phase.ACQUISITION;
		}
		if (phase == Phase.ACQUISITION && dist < TERMINAL_RANGE) {
			phase = Phase.TERMINAL;
		}

		// ── Throttle management ──
		// Adaptive speed: stay fast when heading is aligned (tail chase),
		// slow down when heading error is large (need to turn for crossing shots).
		// This maximizes closing speed in tail chases while enabling tight
		// turns in beam/crossing engagements.
		double headingToTarget = Math.atan2(dx, dy);
		if (headingToTarget < 0)
			headingToTarget += 2 * Math.PI;
		double headingErr = headingToTarget - heading;
		while (headingErr > Math.PI)
			headingErr -= 2 * Math.PI;
		while (headingErr < -Math.PI)
			headingErr += 2 * Math.PI;
		double absErr = Math.abs(headingErr);

		// turnNeed: 0 when on course, 1 when 45+ degrees off
		double turnNeed = Math.clamp(absErr / Math.toRadians(45), 0, 1);

		// Adaptive speed: balance turn authority against closing rate.
		double bearingX = dx / Math.max(dist, 1), bearingY = dy / Math.max(dist, 1);
		// Target velocity component along our bearing: + = fleeing, - = driving toward us.
		double alongBearing = estVelX * bearingX + estVelY * bearingY;
		double targetAwaySpeed = Math.max(alongBearing, 0);

		// 1. Turn authority: slow down when heading error is large (turn radius ~ v^2).
		//    Allow dropping to 7 m/s on hard turns for a tight terminal merge.
		double turnSpeed = 23.0 - turnNeed * 16.0; // 23 on course, 7 turning hard
		// 2. Closing floor: speed needed to keep a healthy closing rate. When the
		//    target is driving toward us (alongBearing < 0) the closing rate stays
		//    high even when we are slow, so the floor drops and we can turn tight to
		//    track late maneuvers. When it flees, the floor rises so we run it down.
		double closingFloor = Math.clamp(8.0 + alongBearing, 7.0, 23.0);
		// Take the max: never sacrifice closing rate, but slow for turns when the
		// geometry (a target closing on us) makes the speed unnecessary.
		double goalSpeed = Math.clamp(Math.max(turnSpeed, closingFloor), 7.0, 23.0);
		if (phase == Phase.TRANSIT)
			goalSpeed = 23.0;
		double speedError = goalSpeed - input.speed();
		double throttle = Math.clamp(speedError * 0.15 + 0.5, 0.1, 1.0);
		output.setThrottle(throttle);

		// ── Sonar: passive every tick, active when cooldown allows ──
		// Passive bearings update the target bearing continuously (every tick).
		// Active pings give range + precise position (every 5 seconds).
		// Between pings, passive tracking keeps the PN guidance fed with
		// fresh bearing data instead of flying on stale predictions.
		usePassiveContacts(input, pos, heading);
		double activeRange = processActiveReturns(input, pos, heading); // updates target position + velocity estimate

		// Close-range command detonation: a measured fix inside 30m means we are
		// close enough that the warhead does heavy damage now, and crossing-shot
		// geometry may keep the proximity fuse from ever tripping. Fire.
		if (phase == Phase.TERMINAL && activeRange < 30) {
			output.detonate();
			return;
		}

		// Active ping scheduling
		switch (phase) {
		case TRANSIT -> {
			// Start pinging when within 3000m of estimated target
			if (dist < 3000 && input.activeSonarCooldownTicks() == 0) {
				output.activeSonarPing();
			}
		}
		case ACQUISITION, TERMINAL -> {
			if (input.activeSonarCooldownTicks() == 0) {
				output.activeSonarPing();
			}
		}
		}

		// ── Compute lead-collision intercept point ──
		// Solve for the earliest time t>0 at which the torpedo (speed s) can reach
		// the target's future position: |P + V t| = s t, where P is the relative
		// target position and V its estimated velocity. Steering at this point is a
		// true collision course, which (unlike chasing the target's current
		// position) does not degenerate into orbiting the target at the turn radius.
		double torpSpeed = Math.max(input.speed(), 5);

		// Cap velocity magnitude to reject garbage estimates
		double velMag = Math.sqrt(estVelX * estVelX + estVelY * estVelY);
		double cappedVelX = estVelX, cappedVelY = estVelY;
		if (velMag > 15) { // no sub goes faster than 15 m/s
			double scale = 15 / velMag;
			cappedVelX *= scale;
			cappedVelY *= scale;
		}

		double interceptX, interceptY;
		double leadTime = solveInterceptTime(dx, dy, cappedVelX, cappedVelY, torpSpeed);
		if (leadTime > 0 && leadTime < 60) {
			// Bound the lead horizon: the target sub maneuvers, so a long
			// straight-line lead over-leads; the limited-agility hull is only
			// ~straight over a few seconds.
			leadTime = Math.min(leadTime, 12);
			interceptX = targetX + cappedVelX * leadTime;
			interceptY = targetY + cappedVelY * leadTime;
		} else {
			// No solution (target outrunning us, or stationary): pursue directly.
			interceptX = targetX;
			interceptY = targetY;
		}

		// ── Steering ──
		// Bearing to the lead-collision intercept point.
		double idx = interceptX - pos.x();
		double idy = interceptY - pos.y();
		double bearingToIntercept = Math.atan2(idx, idy);
		if (bearingToIntercept < 0)
			bearingToIntercept += 2 * Math.PI;

		// Lead pursuit of the intercept point in every phase. Because the aim point
		// already leads the target, pointing at it yields a converging collision
		// course rather than the tail-chase orbit that pursuing the target's current
		// position produces once inside one turn radius.
		double headingError = bearingToIntercept - heading;
		while (headingError > Math.PI)
			headingError -= 2 * Math.PI;
		while (headingError < -Math.PI)
			headingError += 2 * Math.PI;
		double rudderGain = (phase == Phase.TERMINAL) ? 3.5 : 2.5;
		double rudderCmd = headingError * rudderGain;

		// Proportional-navigation damping on the intercept line-of-sight rate:
		// settles the torpedo onto the collision course and suppresses overshoot
		// without adding raw turn authority.
		if (!Double.isNaN(prevBearingToIntercept) && prevBearingTick >= 0) {
			double dtBearing = (input.tick() - prevBearingTick) / 50.0;
			if (dtBearing > 0.01) {
				double dBearing = bearingToIntercept - prevBearingToIntercept;
				while (dBearing > Math.PI)
					dBearing -= 2 * Math.PI;
				while (dBearing < -Math.PI)
					dBearing += 2 * Math.PI;
				double losRate = dBearing / dtBearing; // rad/s
				double closingSpeed = Math.max(torpSpeed - targetAwaySpeed, 5);
				rudderCmd += 4.0 * closingSpeed * losRate * 0.08;
			}
		}
		prevBearingToIntercept = bearingToIntercept;
		prevBearingTick = input.tick();
		output.setRudder(Math.clamp(rudderCmd, -1, 1));

		// ── Depth control (PID) ──
		// Depth profile: start shallow (terrain clearance), then smoothly
		// converge to target depth as we close in. This naturally avoids
		// terrain early and reaches the target's depth for the kill.
		double minDepth = -20; // never breach

		// Depth profile: blend from cruise altitude to target depth over a long
		// range so the torpedo arrives at the right depth. Starting the descent
		// early (from 1500m) gives the PID enough distance to converge, even
		// with gentle pitch rates.
		double goalZ;
		// Cruise altitude: slightly above target depth for terrain clearance.
		// Never climb above launch depth: if we're already deep enough, stay deep.
		double cruiseZ = Math.min(targetZ + 30, -60);
		if (!Double.isNaN(launchDepth)) {
			cruiseZ = Math.min(cruiseZ, launchDepth);
		}
		if (dist > 1500) {
			goalZ = cruiseZ;
		} else {
			// Smooth blend from cruise to target depth over 1500m to 0m
			double t = dist / 1500.0; // 1 at 1500m, 0 at target
			goalZ = targetZ + (cruiseZ - targetZ) * t;
		}

		// Terrain avoidance: check multiple distances ahead so ridges well before impact
		// are detected in time for the PID to respond.
		if (terrain != null) {
			double floor = terrain.elevationAt(pos.x(), pos.y());
			double safeZ = floor + MIN_TERRAIN_CLEARANCE;
			double sinH = Math.sin(heading), cosH = Math.cos(heading);
			for (int seconds = 2; seconds <= 10; seconds += 2) {
				double d = input.speed() * seconds;
				double aheadFloor = terrain.elevationAt(pos.x() + sinH * d, pos.y() + cosH * d);
				// Extra clearance for far detections — more time to climb, but ridge can be tall
				double extra = Math.max(0, (10 - seconds) * 4); // 32m at 2s, 8m at 8s, 0 at 10s
				safeZ = Math.max(safeZ, aheadFloor + MIN_TERRAIN_CLEARANCE + extra);
			}
			if (goalZ < safeZ)
				goalZ = safeZ;
		}
		goalZ = Math.min(goalZ, minDepth);
		// Depth floor: prevent going too shallow during far approach.
		// Relaxes as we get closer so the torpedo can match shallow targets.
		if (dist > 500) {
			goalZ = Math.min(goalZ, -60);
		} else if (dist > 100) {
			double t = (dist - 100) / 400.0;
			goalZ = Math.min(goalZ, targetZ + (-60 - targetZ) * t);
		}

		// PID controller with distance-dependent gains.
		// Far away: soft P, strong D (stay smooth, go shallow for clearance).
		// Close in: aggressive P, extra planes for last-second depth corrections.
		double dt = input.deltaTimeSeconds();
		double depthError = goalZ - pos.z();
		depthIntegral += depthError * dt;
		depthIntegral = Math.clamp(depthIntegral, -20, 20); // tight anti-windup
		double depthDerivative = (depthError - prevDepthError) / Math.max(dt, 0.001);
		prevDepthError = depthError;

		// Urgency ramps from 0 (far) to 1 (close)
		double urgency = Math.clamp(1.0 - dist / 1500.0, 0, 1);
		// Vertical rate for overshoot detection and surface avoidance
		double vRate = input.speed() * Math.sin(input.self().pitch());

		// Terminal boost: scale with both range (close = more) and depth error
		// (large error = more). This avoids boosting when already at the right
		// depth (prevents oscillation) while aggressively correcting large gaps.
		double termRange = Math.clamp(1.0 - dist / 400.0, 0, 1);
		double depthNeed = Math.clamp(Math.abs(depthError) / 30.0, 0, 1);
		double termBoost = termRange * depthNeed;
		double kP = 0.0015 + urgency * 0.008 + termBoost * 0.012;
		double kI = 0.0002 + urgency * 0.0008;
		// D term: extra damping when vertical rate opposes the correction
		double basekD = 0.05 - urgency * 0.02 + termBoost * 0.02;
		boolean overshooting = (vRate > 0 && depthError < -2) || (vRate < 0 && depthError > 2);
		double kD = overshooting ? basekD * 3.0 : (depthDerivative > 0 ? basekD * 2.0 : basekD);
		// More planes authority when depth error is large at close range
		double maxPlanes = 0.10 + urgency * 0.15 + termBoost * 0.25;

		double planes = kP * depthError + kI * depthIntegral + kD * depthDerivative;
		double predictedZ = pos.z() + vRate * 4; // where we'll be in 4 seconds

		// Surface avoidance: predictive + hard limit.
		if (predictedZ > -40 && vRate > 0) {
			double brakeUrgency = Math.max(0, (predictedZ + 40) * 0.03 + vRate * 0.05);
			planes = Math.min(planes, -brakeUrgency);
			depthIntegral = Math.min(depthIntegral, 0);
		}
		if (pos.z() > -45) {
			double surfaceUrgency = (pos.z() + 45) * 0.08;
			planes = Math.min(planes, -surfaceUrgency - 0.10);
			depthIntegral = Math.min(depthIntegral, 0);
		}

		output.setSternPlanes(Math.clamp(planes, -maxPlanes, maxPlanes));

		// Publish intercept point for viewer
		output.publishTarget(interceptX, interceptY, targetZ);

		// Publish full diagnostics for analysis
		double estHdg = Math.atan2(estVelX, estVelY);
		if (estHdg < 0)
			estHdg += 2 * Math.PI;
		double estSpd = Math.sqrt(estVelX * estVelX + estVelY * estVelY);
		output.publishDiagnostics(targetX, targetY, targetZ, estHdg, estSpd, interceptX, interceptY, targetZ,
				phase.name());
	}

	/**
	 * Use passive sonar during transit to refine bearing to target.
	 */
	private void usePassiveContacts(TorpedoInput input, Vec3 pos, double heading) {
		if (input.sonarContacts().isEmpty())
			return;

		// Pick the loudest forward contact (likely our target)
		SonarContact best = pickForwardContact(input.sonarContacts(), heading);
		if (best != null && best.signalExcess() > 0) {
			// Passive gives bearing only (no range). Use it two ways:
			// 1. Gently nudge target position estimate (bearing is good, range is our guess)
			// 2. Feed the PN bearing tracker for continuous LOS rate computation
			double pdx = targetX - pos.x();
			double pdy = targetY - pos.y();
			double currentDist = Math.sqrt(pdx * pdx + pdy * pdy);

			double passiveX = pos.x() + Math.sin(best.bearing()) * currentDist;
			double passiveY = pos.y() + Math.cos(best.bearing()) * currentDist;

			// Gentle position blend (noisy bearings + estimated range = don't trust too much)
			targetX = targetX * 0.95 + passiveX * 0.05;
			targetY = targetY * 0.95 + passiveY * 0.05;

			// DO NOT overwrite the PN bearing tracker (prevBearingToTarget) with
			// passive bearings. The PN guidance computes LOS rate from consecutive
			// bearing samples; injecting noisy passive data every tick destroys the
			// rate signal and causes erratic steering. Let the PN tracker update
			// naturally from the recomputed bearingToTarget in the main loop.
		}
	}

	/**
	 * Process active sonar returns for precise targeting. Returns closest active range.
	 */
	private double processActiveReturns(TorpedoInput input, Vec3 pos, double heading) {
		if (input.activeSonarReturns().isEmpty())
			return Double.MAX_VALUE;

		SonarContact best = pickForwardContact(input.activeSonarReturns(), heading);
		if (best == null)
			return Double.MAX_VALUE;

		double closestRange = best.range();

		double fixX = pos.x() + Math.sin(best.bearing()) * best.range();
		double fixY = pos.y() + Math.cos(best.bearing()) * best.range();

		// Estimate target velocity from consecutive fixes
		if (!Double.isNaN(prevFixX) && prevFixTick >= 0) {
			double dtFix = (input.tick() - prevFixTick) / 50.0;
			if (dtFix > 0.2 && dtFix < 20) {
				double rawVX = (fixX - prevFixX) / dtFix;
				double rawVY = (fixY - prevFixY) / dtFix;
				estVelX = estVelX * 0.3 + rawVX * 0.7;
				estVelY = estVelY * 0.3 + rawVY * 0.7;
			}
		}

		prevFixX = fixX;
		prevFixY = fixY;
		prevFixTick = input.tick();

		// Update target position from active return
		targetX = fixX;
		targetY = fixY;
		if (!Double.isNaN(best.estimatedDepth())) {
			targetZ = best.estimatedDepth();
		}
		lastActiveRange = closestRange;
		return closestRange;
	}

	/**
	 * Earliest positive time at which a pursuer of the given speed, starting at the origin, can reach a target
	 * currently at (px, py) moving at velocity (vx, vy): solves |(px,py) + (vx,vy)·t| = speed·t. Returns -1 if there is
	 * no positive solution (target faster than the torpedo and opening the range).
	 */
	private static double solveInterceptTime(double px, double py, double vx, double vy, double speed) {
		double a = (vx * vx + vy * vy) - speed * speed;
		double b = 2 * (px * vx + py * vy);
		double c = px * px + py * py;
		if (Math.abs(a) < 1e-6) {
			if (Math.abs(b) < 1e-9)
				return -1;
			double t = -c / b;
			return t > 0 ? t : -1;
		}
		double disc = b * b - 4 * a * c;
		if (disc < 0)
			return -1;
		double sq = Math.sqrt(disc);
		double t1 = (-b - sq) / (2 * a);
		double t2 = (-b + sq) / (2 * a);
		double t = Double.MAX_VALUE;
		if (t1 > 0)
			t = Math.min(t, t1);
		if (t2 > 0)
			t = Math.min(t, t2);
		return t == Double.MAX_VALUE ? -1 : t;
	}

	/**
	 * Pick the best non-torpedo contact forward of our heading.
	 */
	private SonarContact pickForwardContact(java.util.List<SonarContact> returns, double heading) {
		SonarContact best = null;
		double bestScore = Double.NEGATIVE_INFINITY;
		for (var c : returns) {
			if (c.range() <= 0 && c.isActive())
				continue; // skip zero-range active
			if (c.classification() == SonarContact.Classification.TORPEDO)
				continue;
			double angleDiff = c.bearing() - heading;
			while (angleDiff > Math.PI)
				angleDiff -= 2 * Math.PI;
			while (angleDiff < -Math.PI)
				angleDiff += 2 * Math.PI;
			double forwardness = Math.cos(angleDiff);
			double score = forwardness * 100 + c.signalExcess() * 0.5;
			if (c.isActive() && c.range() > 0)
				score += 50; // prefer active fixes
			if (score > bestScore) {
				bestScore = score;
				best = c;
			}
		}
		return best;
	}
}
