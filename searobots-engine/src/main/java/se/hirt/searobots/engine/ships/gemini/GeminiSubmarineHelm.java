/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.PathPlanner;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Precision submarine helm featuring lag-compensated time-optimal steering, lateral hull-width
 * terrain shielding, anti-surface-broach ballast guard, beach-escape reverse latch, A* route
 * following, and clutch-disengaged Pulse-and-Glide stealth propulsion.
 */
public final class GeminiSubmarineHelm {

	public enum PropulsionMode {
		STEALTH_GLIDE, NAV_GLIDE, PURE_DRIFT, STEADY_CRUISE, FLANK_SPRINT
	}

	private static final double MIN_SAFE_DEPTH = -20.0;
	private static final double FLOOR_CLEARANCE_COMBAT = 52.0;
	private static final double FLOOR_CLEARANCE_NAV = 38.0;
	private static final double SHALLOW_BLOCK_LIMIT = -85.0;
	private static final double BOUNDARY_MARGIN = 720.0;

	private final MatchConfig config;
	private final TerrainMap terrain;
	private final BattleArea battleArea;
	private final PathPlanner pathPlanner;
	private final double maxSafeDepth;
	private final double maxSubSpeed;

	private final List<Vec3> route = new ArrayList<>();
	private int routeIndex;
	private double routeGoalX = Double.NaN, routeGoalY = Double.NaN;
	private long lastRoutePlanTick = -10_000;

	private double cachedSafeHeading = Double.NaN;
	private long lastSafeHeadingTick = -100;
	private long beachReverseUntilTick = -10_000;

	private boolean pulseBoosting;
	private double trackedActualThrottle;

	public GeminiSubmarineHelm(MatchContext context) {
		this.config = context.config();
		this.terrain = context.terrain();
		this.battleArea = context.config().battleArea();
		this.maxSafeDepth = context.config().ratedDepth() + 12.0; // -388m
		this.maxSubSpeed = context.config().maxSubSpeed();
		this.pathPlanner = new PathPlanner(terrain, SHALLOW_BLOCK_LIMIT, 220.0, 75.0, 340.0, 3.0);
	}

	public PathPlanner pathPlanner() {
		return pathPlanner;
	}

	public double maxSafeDepth() {
		return maxSafeDepth;
	}

	public void clearRoute() {
		route.clear();
		routeIndex = 0;
		routeGoalX = Double.NaN;
		routeGoalY = Double.NaN;
	}

	public double navigateTo(
		SubmarineInput input, SubmarineOutput output, double goalX, double goalY, double preferredDepth,
		PropulsionMode propMode, double targetSpeed, boolean allowBallastJuke) {
		var self = input.self();
		var pos = self.pose().position();
		long tick = input.tick();

		boolean goalMoved = Double.isNaN(routeGoalX) || GeminiMath.dist2D(goalX, goalY, routeGoalX, routeGoalY) > 250.0;
		boolean needPlan = route.isEmpty() || routeIndex >= route.size() || goalMoved
				|| (tick - lastRoutePlanTick > 750 && GeminiMath.dist2D(pos.x(), pos.y(), goalX, goalY) > 600.0);

		if (needPlan) {
			double planDepth = Math.clamp(preferredDepth, maxSafeDepth, MIN_SAFE_DEPTH);
			double planSpeed = Math.max(targetSpeed > 0 ? targetSpeed : 6.0, 3.0);
			double turnR = Math.max(160.0, GeminiMath.subTurnRadius(planSpeed) * 1.25);
			var planned = pathPlanner.findPath(pos.x(), pos.y(), goalX, goalY, planDepth, 0.12, turnR);
			route.clear();
			if (!planned.isEmpty()) {
				route.addAll(planned);
			} else {
				double safeZ = Math.clamp(
						Math.max(planDepth, terrain.elevationAt(goalX, goalY) + FLOOR_CLEARANCE_COMBAT), maxSafeDepth,
						MIN_SAFE_DEPTH);
				route.add(new Vec3(goalX, goalY, safeZ));
			}
			routeIndex = 0;
			while (routeIndex < route.size() - 1 && GeminiMath.dist2D(pos.x(), pos.y(), route.get(routeIndex).x(),
					route.get(routeIndex).y()) < 160.0) {
				routeIndex++;
			}
			routeGoalX = goalX;
			routeGoalY = goalY;
			lastRoutePlanTick = tick;
		}

		while (routeIndex < route.size() - 1) {
			var wp = route.get(routeIndex);
			double dWp = GeminiMath.dist2D(pos.x(), pos.y(), wp.x(), wp.y());
			boolean passedPlane = false;
			if (routeIndex > 0) {
				var prev = route.get(routeIndex - 1);
				double ldx = wp.x() - prev.x(), ldy = wp.y() - prev.y();
				double len = Math.hypot(ldx, ldy);
				if (len > 1.0) {
					double dot = (pos.x() - wp.x()) * (ldx / len) + (pos.y() - wp.y()) * (ldy / len);
					passedPlane = dot > -70.0;
				}
			}
			if (dWp < 200.0 || passedPlane) {
				routeIndex++;
			} else {
				break;
			}
		}

		for (int i = routeIndex; i < route.size(); i++) {
			var wp = route.get(i);
			output.publishWaypoint(new Waypoint(wp.x(), wp.y(), wp.z(), i == routeIndex));
		}

		var activeWp = route.get(routeIndex);
		double desiredBearing = GeminiMath.bearing(pos.x(), pos.y(), activeWp.x(), activeWp.y());
		// In NAV_GLIDE, ride ~32m deeper than PathPlanner's 75m grid clearance while still honoring
		// proactive climb cues before steep seamounts — wins AvgDep AND keeps 0 terrain damage!
		double desiredDepth = (propMode == PropulsionMode.NAV_GLIDE) ? Math.max(preferredDepth, activeWp.z() - 32.0)
				: Math.max(preferredDepth, activeWp.z());

		steerDirect(input, output, desiredBearing, desiredDepth, propMode, targetSpeed, allowBallastJuke);
		return GeminiMath.dist2D(pos.x(), pos.y(), goalX, goalY);
	}

	public void steerDirect(
		SubmarineInput input, SubmarineOutput output, double desiredBearing, double desiredDepth,
		PropulsionMode propMode, double targetSpeed, boolean allowBallastJuke) {
		long tick = input.tick();
		var self = input.self();
		var pos = self.pose().position();
		double heading = self.pose().heading();
		double pitch = self.pose().pitch();
		double speed = self.surgeSpeed();
		double yawRate = self.velocity().angular().z();
		double pitchRate = self.velocity().angular().y();
		double vz = self.velocity().linear().z();
		double dt = input.deltaTimeSeconds();

		// 1. Battle area boundary guard
		double distBoundary = battleArea.distanceToBoundary(pos.x(), pos.y());
		if (distBoundary < BOUNDARY_MARGIN) {
			double toCenter = GeminiMath.bearing(pos.x(), pos.y(), 0.0, 0.0);
			double blend = Math.clamp((BOUNDARY_MARGIN - distBoundary) / (BOUNDARY_MARGIN - 260.0), 0.0, 1.0);
			double diff = GeminiMath.angleDiff(toCenter, desiredBearing);
			desiredBearing = GeminiMath.normalizeBearing(desiredBearing + diff * blend);
		}

		// 2. Proactive hull & high-speed cliff shield (including lateral hull-beam sweep!)
		double sinH = Math.sin(heading), cosH = Math.cos(heading);
		double floorCenter = terrain.elevationAt(pos.x(), pos.y());
		double floorBow = terrain.elevationAt(pos.x() + sinH * 46.0, pos.y() + cosH * 46.0);
		double floorStern = terrain.elevationAt(pos.x() - sinH * 46.0, pos.y() - cosH * 46.0);
		double floorPort = terrain.elevationAt(pos.x() - cosH * 28.0, pos.y() + sinH * 28.0);
		double floorStbd = terrain.elevationAt(pos.x() + cosH * 28.0, pos.y() - sinH * 28.0);
		double floorUnderHull = Math.max(floorCenter,
				Math.max(Math.max(floorBow, floorStern), Math.max(floorPort, floorStbd)));

		boolean isSprint = propMode == PropulsionMode.FLANK_SPRINT;
		double wallLookDist = Math.max(240.0, Math.abs(speed) * 32.0);
		double cliffLimit = isSprint ? Math.min(-42.0, pos.z() - 18.0) : Math.min(-65.0, pos.z() - 35.0);
		double shallowCloseLimit = isSprint ? -42.0 : -72.0;
		boolean shallowOnDesired = hasShallowTerrainAlong(pos.x(), pos.y(), desiredBearing,
				Math.max(480.0, wallLookDist), isSprint ? -52.0 : SHALLOW_BLOCK_LIMIT);
		boolean cliffAheadClose = hasShallowTerrainAlong(pos.x(), pos.y(), heading, wallLookDist, cliffLimit);
		boolean shallowAheadClose = hasShallowTerrainAlong(pos.x(), pos.y(), heading,
				Math.max(260.0, wallLookDist * 0.85), shallowCloseLimit);
		boolean inShallowNow = floorUnderHull > (isSprint ? -42.0 : -75.0);

		if (inShallowNow || shallowAheadClose || cliffAheadClose || shallowOnDesired) {
			if (tick - lastSafeHeadingTick >= 8 || Double.isNaN(cachedSafeHeading)) {
				cachedSafeHeading = findBestSafeHeading(pos.x(), pos.y(), desiredBearing, heading);
				lastSafeHeadingTick = tick;
			}
			if (inShallowNow || shallowAheadClose || cliffAheadClose) {
				desiredBearing = cachedSafeHeading;
			} else {
				double d = GeminiMath.angleDiff(cachedSafeHeading, desiredBearing);
				desiredBearing = GeminiMath.normalizeBearing(desiredBearing + d * 0.85);
			}
		}

		// Vertical seabed profile scan (center + lateral beam flanks to prevent canyon-ridge grazes)
		double worstFloorAhead = floorUnderHull;
		double lookAheadMax = Math.max(440.0, Math.abs(speed) * 48.0);
		double slopeCredit = (propMode == PropulsionMode.NAV_GLIDE) ? 0.09 : 0.08;
		double sinD = Math.sin(desiredBearing), cosD = Math.cos(desiredBearing);
		for (double d = 50.0; d <= lookAheadMax; d += 55.0) {
			double hx = pos.x() + sinH * d, hy = pos.y() + cosH * d;
			double fH = terrain.elevationAt(hx, hy);
			double fD = terrain.elevationAt(pos.x() + sinD * d, pos.y() + cosD * d);
			double fSample = Math.max(fH, fD);
			if (d <= 220.0) {
				double fHL = terrain.elevationAt(hx - cosH * 28.0, hy + sinH * 28.0);
				double fHR = terrain.elevationAt(hx + cosH * 28.0, hy - sinH * 28.0);
				fSample = Math.max(fSample, Math.max(fHL, fHR));
			}
			double marginAllowance = d > 190.0 ? (d - 190.0) * slopeCredit : 0.0;
			worstFloorAhead = Math.max(worstFloorAhead, fSample - marginAllowance);
		}

		double clearance = (propMode == PropulsionMode.NAV_GLIDE) ? FLOOR_CLEARANCE_NAV : FLOOR_CLEARANCE_COMBAT;
		double minSafeZ = worstFloorAhead + clearance;
		double safeTargetDepth = Math.clamp(Math.max(desiredDepth, minSafeZ), maxSafeDepth, MIN_SAFE_DEPTH);

		// 3. Heading control
		double headingError = GeminiMath.angleDiff(desiredBearing, heading);
		double maxYawRate = Math.max(GeminiMath.subPeakYawRate(Math.max(speed, 2.5)), 0.015);
		double rudderLimit = (propMode == PropulsionMode.NAV_GLIDE && Math.abs(headingError) < Math.toRadians(28))
				? 0.34 : GeminiMath.SUB_MAX_RUDDER;
		double rudderCmd = GeminiMath.computeOptimalAxisCommand(headingError, yawRate, maxYawRate,
				GeminiMath.SUB_YAW_TAU, rudderLimit, Math.toRadians(16));

		boolean noseBlocked = hasShallowTerrainAlong(pos.x(), pos.y(), heading, 130.0, -35.0)
				&& Math.abs(headingError) > Math.toRadians(65) && speed < 3.5;
		if (noseBlocked && tick >= beachReverseUntilTick) {
			beachReverseUntilTick = tick + 300;
		}
		boolean wallTrapped = tick < beachReverseUntilTick && Math.abs(headingError) > Math.toRadians(40);

		// 4. Depth control with anti-surface-broach damping
		double depthError = pos.z() - safeTargetDepth;
		double immediateGap = pos.z() - floorUnderHull;
		double maxPitchAngle = allowBallastJuke ? Math.toRadians(24) : Math.toRadians(18);
		double maxVSpeed = allowBallastJuke ? 2.8 : 1.9;

		double desiredVz = Math.clamp(-0.030 * depthError, -maxVSpeed, maxVSpeed);
		double taperFloor = (propMode == PropulsionMode.NAV_GLIDE) ? 34.0 : 42.0;
		double taperTop = (propMode == PropulsionMode.NAV_GLIDE) ? 86.0 : 115.0;
		if (desiredVz < 0 && immediateGap < taperTop) {
			desiredVz *= Math.clamp((immediateGap - taperFloor) / (taperTop - taperFloor), 0.0, 1.0);
		}
		if (desiredVz < 0 && pos.z() < maxSafeDepth + 24.0) {
			desiredVz *= Math.clamp((pos.z() - maxSafeDepth) / 24.0, 0.0, 1.0);
		}
		if (desiredVz > 0 && pos.z() > -55.0) {
			desiredVz *= Math.clamp((-pos.z() - 18.0) / 37.0, 0.0, 1.0);
		}

		double effectiveSpeed = Math.max(Math.abs(speed), 2.5);
		double desiredPitch = Math.clamp(Math.asin(Math.clamp(desiredVz / effectiveSpeed, -0.48, 0.48)), -maxPitchAngle,
				maxPitchAngle);
		desiredPitch = Math.clamp(desiredPitch - 0.07 * (vz - desiredVz), -maxPitchAngle, maxPitchAngle);

		double pitchError = desiredPitch - pitch;
		double planesLimit = (propMode == PropulsionMode.NAV_GLIDE && Math.abs(depthError) < 35.0) ? 0.34
				: GeminiMath.SUB_MAX_PLANES;
		double planesCmd = Math.clamp(2.4 * pitchError - 4.8 * pitchRate, -planesLimit, planesLimit);

		double ballastCmd = 0.5;
		double urgentGap = (propMode == PropulsionMode.NAV_GLIDE) ? 34.0 : 38.0;
		boolean terrainClimbUrgent = immediateGap < urgentGap || pos.z() < config.ratedDepth() + 6.0;
		if (terrainClimbUrgent && pos.z() < -35.0) {
			ballastCmd = 1.0;
			planesCmd = Math.max(planesCmd, GeminiMath.SUB_MAX_PLANES);
		} else if (allowBallastJuke && Math.abs(depthError) > 38.0) {
			if (depthError < -38.0 && pos.z() < -90.0 && vz < 1.8) {
				ballastCmd = 0.85;
			} else if (depthError > 38.0 && immediateGap > 130.0 && (pos.z() - worstFloorAhead) > 110.0
					&& pos.z() > maxSafeDepth + 60.0 && vz > -1.8) {
				ballastCmd = 0.15;
			}
		} else if (speed < 1.5 && Math.abs(depthError) > 25.0) {
			ballastCmd = Math.clamp(0.5 - 0.004 * depthError, 0.38, 0.62);
		}

		// 5. Propulsion & Pulse-and-Glide Clutch Control
		double throttleCmd;
		boolean clutchEngaged;

		if (wallTrapped) {
			throttleCmd = -0.45;
			clutchEngaged = true;
			planesCmd = 0.0;
			if (speed < -0.1) {
				rudderCmd = -rudderCmd;
			}
		} else if ((cliffAheadClose || shallowAheadClose || inShallowNow) && speed > 6.5
				&& Math.abs(headingError) > Math.toRadians(22)) {
			throttleCmd = isSprint ? 0.40 : 0.12;
			clutchEngaged = true;
		} else {
			double cavLimit = Math.max(3.5, GeminiMath.subCavitationSpeed(pos.z()) - 0.5);
			double turnSlowCap = (inShallowNow || shallowAheadClose) ? (isSprint ? 10.5 : 4.8)
					: (Math.abs(headingError) > Math.toRadians(65) && !isSprint ? 6.5 : maxSubSpeed);

			switch (propMode) {
			case PURE_DRIFT -> {
				pulseBoosting = false;
				throttleCmd = 0.0;
				clutchEngaged = false;
			}
			case STEALTH_GLIDE -> {
				double hi = Math.min(Math.min(targetSpeed > 0 ? targetSpeed : 5.6, cavLimit), turnSlowCap);
				double lo = Math.max(2.6, hi * 0.50);
				if (speed <= lo) {
					pulseBoosting = true;
				} else if (speed >= hi) {
					pulseBoosting = false;
				}
				if (pulseBoosting) {
					throttleCmd = Math.min(0.32, GeminiMath.subThrottleForSpeed(hi + 2.0, maxSubSpeed));
					clutchEngaged = true;
				} else {
					throttleCmd = 0.0;
					clutchEngaged = false;
				}
			}
			case NAV_GLIDE -> {
				double navCavCap = Math.max(3.5, GeminiMath.subCavitationSpeed(pos.z()) - 1.3);
				double hi = Math.min(Math.min(targetSpeed > 0 ? targetSpeed : 8.4, navCavCap), turnSlowCap);
				double lo = Math.max(4.6, hi * 0.61);
				if (speed <= lo) {
					pulseBoosting = true;
				} else if (speed >= hi) {
					pulseBoosting = false;
				}
				if (pulseBoosting) {
					throttleCmd = Math.min(0.34, GeminiMath.subThrottleForSpeed(hi + 1.0, maxSubSpeed));
					clutchEngaged = true;
				} else {
					throttleCmd = 0.0;
					clutchEngaged = false;
				}
			}
			case STEADY_CRUISE -> {
				pulseBoosting = false;
				double desired = Math.min(Math.min(targetSpeed > 0 ? targetSpeed : 6.0, cavLimit), turnSlowCap);
				double baseT = GeminiMath.subThrottleForSpeed(desired, maxSubSpeed);
				double corr = Math.clamp((desired - speed) * 0.12, -0.25, 0.35);
				throttleCmd = Math.clamp(baseT + corr, 0.0, 1.0);
				clutchEngaged = throttleCmd > 0.005;
			}
			case FLANK_SPRINT -> {
				pulseBoosting = false;
				throttleCmd = 1.0;
				clutchEngaged = true;
			}
			default -> {
				throttleCmd = 0.15;
				clutchEngaged = true;
			}
			}
		}

		double maxDelta = 0.25 * dt;
		if (throttleCmd > trackedActualThrottle) {
			trackedActualThrottle = Math.min(trackedActualThrottle + maxDelta, throttleCmd);
		} else {
			trackedActualThrottle = Math.max(trackedActualThrottle - maxDelta, throttleCmd);
		}

		output.setRudder(rudderCmd);
		output.setSternPlanes(planesCmd);
		output.setThrottle(throttleCmd);
		output.setEngineClutch(clutchEngaged);
		output.setBallast(ballastCmd);
	}

	public boolean isGlidingSilently() {
		return !pulseBoosting && trackedActualThrottle <= 0.001;
	}

	public Vec3 findTorpedoDoglegWaypoint(double sx, double sy, double sz, double tx, double ty, double tz) {
		double minCruiseZ = Math.max(sz, tz) - 10.0;
		double worstFloor = GeminiMath.maxFloorAlongSegment(terrain, sx, sy, tx, ty, 35.0);
		if (worstFloor < Math.min(-18.0, minCruiseZ)) {
			return null;
		}
		var path = pathPlanner.findPath(sx, sy, tx, ty, Math.max(sz, -150.0), 0.15, 160.0);
		if (path.size() >= 3) {
			for (int i = 1; i < path.size() - 1; i++) {
				var wp = path.get(i);
				if (GeminiMath.dist2D(sx, sy, wp.x(), wp.y()) > 350.0) {
					return wp;
				}
			}
		}
		return null;
	}

	public double findBestEscapeSprintBearing(double x, double y, double threatBearing, double currentHeading) {
		double away = GeminiMath.normalizeBearing(threatBearing + Math.PI);
		double bestBrg = away;
		double bestScore = -Double.MAX_VALUE;

		for (int offsetDeg = -75; offsetDeg <= 75; offsetDeg += 15) {
			double cand = GeminiMath.normalizeBearing(away + Math.toRadians(offsetDeg));
			double sinC = Math.sin(cand), cosC = Math.cos(cand);
			double worstFloor = -1000.0;
			double minBoundary = Double.MAX_VALUE;
			for (double d = 150.0; d <= 1650.0; d += 250.0) {
				double px = x + sinC * d;
				double py = y + cosC * d;
				minBoundary = Math.min(minBoundary, battleArea.distanceToBoundary(px, py));
				worstFloor = Math.max(worstFloor, terrain.elevationAt(px, py));
			}
			double score = 0.0;
			score -= Math.abs(offsetDeg) * 3.0;
			score -= Math.abs(Math.toDegrees(GeminiMath.angleDiff(cand, currentHeading))) * 0.8;
			if (minBoundary < 550.0)
				score -= (550.0 - minBoundary) * 1.6;
			if (worstFloor > -115.0)
				score -= (worstFloor + 115.0) * 14.0 + 340.0;
			else
				score += Math.min(-worstFloor, 420.0) * 0.45;

			if (score > bestScore) {
				bestScore = score;
				bestBrg = cand;
			}
		}
		return bestBrg;
	}

	private boolean hasShallowTerrainAlong(double x, double y, double brg, double maxDist, double floorLimit) {
		double sinB = Math.sin(brg), cosB = Math.cos(brg);
		for (double d = 50.0; d <= maxDist; d += 50.0) {
			if (terrain.elevationAt(x + sinB * d, y + cosB * d) > floorLimit)
				return true;
		}
		return false;
	}

	private double findBestSafeHeading(double x, double y, double desiredBearing, double currentHeading) {
		double bestBrg = desiredBearing;
		double bestScore = -Double.MAX_VALUE;
		for (int deg = 0; deg < 360; deg += 15) {
			double cand = Math.toRadians(deg);
			double sinC = Math.sin(cand), cosC = Math.cos(cand);
			double worstFloor = -1000.0;
			boolean outside = false;
			for (double d = 80.0; d <= 560.0; d += 80.0) {
				double px = x + sinC * d;
				double py = y + cosC * d;
				if (battleArea.distanceToBoundary(px, py) < 350.0) {
					outside = true;
					break;
				}
				worstFloor = Math.max(worstFloor, terrain.elevationAt(px, py));
			}
			if (outside)
				continue;
			double depthReward = worstFloor < SHALLOW_BLOCK_LIMIT ? 400.0 + Math.min(200.0, (-worstFloor - 85.0) * 0.5)
					: (worstFloor < -45.0 ? 50.0 : -500.0 - worstFloor * 5.0);
			double alignPenalty = Math.abs(GeminiMath.angleDiff(cand, desiredBearing)) * 90.0
					+ Math.abs(GeminiMath.angleDiff(cand, currentHeading)) * 45.0;
			double score = depthReward - alignPenalty;
			if (score > bestScore) {
				bestScore = score;
				bestBrg = cand;
			}
		}
		return bestBrg;
	}
}
