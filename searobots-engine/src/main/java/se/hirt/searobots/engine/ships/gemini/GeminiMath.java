/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.ThermalLayer;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.util.List;

/**
 * Pure mathematics, physics models, and acoustic/geometric helpers for the Gemini submarine and
 * torpedo controllers. All formulas mirror the physical dynamics of the SeaRobots simulation.
 */
public final class GeminiMath {

	private static final double TWO_PI = 2.0 * Math.PI;
	private static final double WATER_DENSITY = 1025.0;

	// Submarine control limits: peak lift occurs at stallAngle (25 deg) / maxDeflection (45 deg) = 0.5556
	public static final double SUB_MAX_RUDDER = 0.55;
	public static final double SUB_MAX_PLANES = 0.55;
	public static final double SUB_YAW_TAU = 30.0;
	public static final double SUB_PITCH_TAU = 30.0;

	// Torpedo control limits: peak lift occurs at stallAngle (30 deg) / maxDeflection (45 deg) = 0.6667
	public static final double TORP_MAX_RUDDER = 0.665;
	public static final double TORP_MAX_PLANES = 0.665;
	public static final double TORP_YAW_TAU = 10.0;
	public static final double TORP_PITCH_TAU = 6.0;

	private static final VehicleConfig SUB_CFG = VehicleConfig.submarine();
	private static final VehicleConfig TORP_CFG = VehicleConfig.torpedo();

	private GeminiMath() {
	}

	/** Normalizes angle to [0, 2pi). */
	public static double normalizeBearing(double angle) {
		double r = angle % TWO_PI;
		return r < 0 ? r + TWO_PI : r;
	}

	/** Signed shortest angle difference (target - current) in [-pi, pi]. */
	public static double angleDiff(double target, double current) {
		double d = (target - current) % TWO_PI;
		if (d > Math.PI)
			d -= TWO_PI;
		if (d < -Math.PI)
			d += TWO_PI;
		return d;
	}

	/** Compass bearing (0 = north/+Y, clockwise) from (x0, y0) to (x1, y1). */
	public static double bearing(double x0, double y0, double x1, double y1) {
		return normalizeBearing(Math.atan2(x1 - x0, y1 - y0));
	}

	public static double dist2D(double x0, double y0, double x1, double y1) {
		return Math.hypot(x1 - x0, y1 - y0);
	}

	public static double dist3D(double x0, double y0, double z0, double x1, double y1, double z1) {
		double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	/**
	 * Steady-state cavitation threshold speed (m/s) for an attack submarine at depth z (negative).
	 */
	public static double subCavitationSpeed(double z) {
		return SUB_CFG.baseCavitationSpeed() + Math.max(0.0, -z) * SUB_CFG.cavitationDepthFactor();
	}

	/**
	 * Steady-state forward throttle needed to maintain targetSpeed (m/s) on an attack submarine.
	 */
	public static double subThrottleForSpeed(double targetSpeed, double maxSubSpeed) {
		if (targetSpeed <= 0 || maxSubSpeed <= 0)
			return 0.0;
		double ratio = Math.clamp(targetSpeed / maxSubSpeed, 0.0, 1.0);
		return ratio * ratio;
	}

	/** Peak steady-state submarine yaw rate (rad/s) at optimal rudder deflection (0.555). */
	public static double subPeakYawRate(double speed) {
		double v = Math.abs(speed);
		double clMax = 2.0 * Math.PI * SUB_CFG.stallAngle();
		double moment = 0.5 * WATER_DENSITY * v * v * SUB_CFG.rudderArea() * clMax * SUB_CFG.rudderArm();
		double baseInertia = SUB_CFG.massSurge() * SUB_CFG.rotationalInertia();
		double effInertia = baseInertia * (1.0 + 0.05 * v * v);
		return effInertia > 0 ? moment / effInertia : 0.0;
	}

	/** Steady-state turn radius (m) of an attack submarine at optimal rudder. */
	public static double subTurnRadius(double speed) {
		double v = Math.max(Math.abs(speed), 1.0);
		double r = subPeakYawRate(v);
		return r > 1e-4 ? v / r : 400.0;
	}

	/** Peak steady-state torpedo yaw rate (rad/s) at optimal rudder deflection (0.667). */
	public static double torpPeakYawRate(double speed) {
		double v = Math.max(Math.abs(speed), 0.1);
		double clMax = 2.0 * Math.PI * Math.sin(TORP_CFG.stallAngle());
		double moment = 0.5 * WATER_DENSITY * v * v * TORP_CFG.rudderArea() * clMax * TORP_CFG.rudderArm();
		double baseInertia = TORP_CFG.massSurge() * TORP_CFG.rotationalInertia();
		double effInertia = baseInertia * (1.0 + 0.4 * v * v);
		return effInertia > 0 ? moment / effInertia : 0.0;
	}

	/** Peak steady-state torpedo pitch rate (rad/s) at optimal planes deflection (0.667). */
	public static double torpPeakPitchRate(double speed) {
		double v = Math.max(Math.abs(speed), 0.1);
		double clMax = 2.0 * Math.PI * Math.sin(TORP_CFG.stallAngle());
		double moment = 0.5 * WATER_DENSITY * v * v * TORP_CFG.planesArea() * clMax * TORP_CFG.planesArm();
		double baseInertia = TORP_CFG.massSurge() * TORP_CFG.rotationalInertia();
		double effInertia = baseInertia * 1.5 + baseInertia * 0.4 * v * v;
		return effInertia > 0 ? moment / effInertia : 0.0;
	}

	/**
	 * Closed-form time-optimal braking angle for first-order rate lag: d(rate)/dt = (u - rate) /
	 * tau with {@code |u| <= maxRate}. Returns the signed angle traversed while applying full
	 * opposite authority to bring angular velocity from {@code rate} to 0.
	 */
	public static double brakingAngle(double rate, double maxRate, double tau) {
		double absRate = Math.abs(rate);
		if (absRate < 1e-6 || maxRate < 1e-5)
			return 0.0;
		double angle = tau * (absRate - maxRate * Math.log(1.0 + absRate / maxRate));
		return Math.copySign(angle, rate);
	}

	/**
	 * Computes a smooth, zero-overshoot deflection command in [-maxDeflection, +maxDeflection] for
	 * an axis governed by first-order rate lag (tau). Uses the exact time-optimal braking curve
	 * outside a linear boundary layer and critically-damped PD tracking inside it.
	 */
	public static double computeOptimalAxisCommand(
		double angleError, double angularRate, double maxRate, double tau, double maxDeflection,
		double linearErrorBand) {
		if (maxRate < 1e-4) {
			return Math.clamp(angleError * 2.0, -maxDeflection, maxDeflection);
		}
		double slewLead = Math.min(0.8, tau * 0.03);
		double predictedError = angleError - angularRate * slewLead;
		double brake = brakingAngle(angularRate, maxRate, tau);
		double switchingError = predictedError - brake;

		if (Math.abs(switchingError) >= linearErrorBand) {
			return Math.copySign(maxDeflection, switchingError);
		}

		double wn = Math.min(Math.sqrt(maxRate / (linearErrorBand * tau)), 1.6 / Math.sqrt(tau));
		double kp = wn * wn * tau;
		double kd = Math.max(0.0, 2.0 * wn * tau - 1.0);
		double desiredRate = kp * angleError - kd * angularRate;
		double normalized = desiredRate / maxRate;
		return Math.clamp(normalized * maxDeflection, -maxDeflection, maxDeflection);
	}

	/**
	 * Solves for the 3D intercept point of a mover at (ox, oy, oz) with water speed
	 * {@code ownSpeed} plus horizontal ocean current (curX, curY) chasing a target at (tx, ty, tz)
	 * moving with ground velocity (tvx, tvy, tvz). Returns the aim point (water-frame lead adjusted
	 * for current) capped at {@code maxLeadSeconds}.
	 */
	public static Vec3 solveIntercept3D(
		double ox, double oy, double oz, double curX, double curY, double ownSpeed, double tx, double ty, double tz,
		double tvx, double tvy, double tvz, double maxLeadSeconds) {
		double rvx = tvx - curX;
		double rvy = tvy - curY;
		double rvz = tvz;

		double dx = tx - ox;
		double dy = ty - oy;
		double dz = tz - oz;

		double v = Math.max(ownSpeed, 4.0);
		double a = rvx * rvx + rvy * rvy + rvz * rvz - v * v;
		double b = 2.0 * (dx * rvx + dy * rvy + dz * rvz);
		double c = dx * dx + dy * dy + dz * dz;

		double tGo;
		if (Math.abs(a) < 1e-6) {
			tGo = b < -1e-6 ? -c / b : Math.sqrt(c) / v;
		} else {
			double disc = b * b - 4.0 * a * c;
			if (disc < 0) {
				tGo = Math.sqrt(c) / v;
			} else {
				double sqrtD = Math.sqrt(disc);
				double t1 = (-b - sqrtD) / (2.0 * a);
				double t2 = (-b + sqrtD) / (2.0 * a);
				if (t1 > 0 && t2 > 0) {
					tGo = Math.min(t1, t2);
				} else if (t1 > 0) {
					tGo = t1;
				} else if (t2 > 0) {
					tGo = t2;
				} else {
					tGo = Math.sqrt(c) / v;
				}
			}
		}

		tGo = Math.clamp(tGo, 0.0, maxLeadSeconds);
		return new Vec3(tx + rvx * tGo, ty + rvy * tGo, tz + rvz * tGo);
	}

	/**
	 * Scans terrain along a 2D line segment and returns the shallowest (highest Z) floor elevation.
	 */
	public static double maxFloorAlongSegment(
		TerrainMap terrain, double x0, double y0, double x1, double y1, double stepMeters) {
		double dist = dist2D(x0, y0, x1, y1);
		int steps = Math.max(2, (int) Math.ceil(dist / Math.max(stepMeters, 10.0)));
		double maxFloor = terrain.elevationAt(x0, y0);
		for (int i = 1; i <= steps; i++) {
			double t = (double) i / steps;
			double f = terrain.elevationAt(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t);
			if (f > maxFloor)
				maxFloor = f;
		}
		return maxFloor;
	}

	/**
	 * Estimates one-way terrain acoustic occlusion in dB along the 3D segment from src to dst,
	 * matching the engine's acoustic model.
	 */
	public static double estimateTerrainOcclusionDb(TerrainMap terrain, Vec3 src, Vec3 dst) {
		double dx = dst.x() - src.x();
		double dy = dst.y() - src.y();
		double dist = Math.hypot(dx, dy);
		if (dist < 1.0)
			return 0.0;
		double step = Math.max(terrain.getCellSize(), 20.0);
		int samples = Math.max(2, (int) Math.ceil(dist / step));
		double totalDb = 0.0;
		for (int i = 1; i < samples; i++) {
			double t = (double) i / samples;
			double floor = terrain.elevationAt(src.x() + dx * t, src.y() + dy * t);
			double lineZ = src.z() + (dst.z() - src.z()) * t;
			if (floor > lineZ) {
				if (floor >= 0) {
					totalDb += 30.0 * (step / terrain.getCellSize());
				} else {
					double excess = floor - lineZ;
					totalDb += 15.0 * Math.min(excess / 50.0, 1.0) * (step / terrain.getCellSize());
				}
			}
		}
		return totalDb;
	}

	/**
	 * Returns the deepest thermocline bottom depth, or NaN if no layers exist.
	 */
	public static double deepestThermoclineBottom(List<ThermalLayer> layers) {
		if (layers == null || layers.isEmpty())
			return Double.NaN;
		double deepest = Double.POSITIVE_INFINITY;
		for (var layer : layers) {
			if (layer.bottom() < deepest)
				deepest = layer.bottom();
		}
		return deepest;
	}

	/**
	 * Returns the shallowest thermocline top depth, or NaN if no layers exist.
	 */
	public static double shallowestThermoclineTop(List<ThermalLayer> layers) {
		if (layers == null || layers.isEmpty())
			return Double.NaN;
		double shallowest = Double.NEGATIVE_INFINITY;
		for (var layer : layers) {
			if (layer.top() > shallowest)
				shallowest = layer.top();
		}
		return shallowest;
	}

	/**
	 * Computes total thermocline transmission loss (dB) between depths zA and zB.
	 */
	public static double thermoclineLossDb(double zA, double zB, List<ThermalLayer> layers) {
		if (layers == null || layers.isEmpty())
			return 0.0;
		double total = 0.0;
		for (var layer : layers) {
			double top = layer.top();
			double bottom = layer.bottom();
			boolean aAbove = zA > top;
			boolean aBelow = zA < bottom;
			boolean bAbove = zB > top;
			boolean bBelow = zB < bottom;
			if ((aAbove && bAbove) || (aBelow && bBelow)) {
				total -= 2.0;
			} else if (aAbove != bAbove && aBelow != bBelow) {
				total += Math.clamp(3.0 + 0.5 * layer.gradient(), 5.0, 12.0);
			} else {
				double maxPen = Math.clamp(3.0 + 0.5 * layer.gradient(), 5.0, 12.0);
				if (layer.thickness() < 1.0) {
					total += maxPen;
				} else {
					double ac = Math.clamp(zA, bottom, top);
					double bc = Math.clamp(zB, bottom, top);
					total += maxPen * Math.abs(ac - bc) / layer.thickness();
				}
			}
		}
		return total;
	}
}
