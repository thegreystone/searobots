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

import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

/**
 * Hull collision geometry: ellipsoid parameters and distance calculations shared between the
 * simulation loop, physics, and tests.
 */
public final class HullGeometry {

	private HullGeometry() {
	}

	// Submarine hull ellipsoid (collision + fuse check)
	public static final double SEMI_LENGTH = 38.0; // bow-to-stern half-length
	public static final double SEMI_BEAM = 5.5; // port-to-starboard half-width
	public static final double SEMI_HEIGHT = 4.5; // keel-to-deck half-height
	public static final double AFT_OFFSET = 0.0; // forward-axis offset from the submarine pose origin
	public static final double UP_OFFSET = 0.11; // generated hull's section axis above the pose origin
	private static final Envelope SUBMARINE = new Envelope(SEMI_LENGTH, SEMI_BEAM, SEMI_HEIGHT, AFT_OFFSET, UP_OFFSET);
	// Conservative fit to the submerged HullBelow/HullBoot/TransomBoot/Bulb mesh groups.
	// A feeder ship has a broad transom and bulbous bow, so its enclosing ellipsoid is longer
	// and wider than the 150 x 30 m nominal hull. Superstructure is outside this water-contact hull.
	private static final Envelope SURFACE_SHIP = new Envelope(90, 19, 7, 1, -2.5);

	/** Collision ellipsoid dimensions and its centre offset in the vehicle's local frame. */
	public record Envelope(double semiLength, double semiBeam, double semiHeight, double forwardOffset,
			double upOffset) {
		public double boundingRadius() {
			return Math.max(semiLength, Math.max(semiBeam, semiHeight));
		}

		/** Converts centre-relative local (right, forward, up) coordinates to world coordinates. */
		public Vec3 worldPoint(Vec3 local, Vec3 position, double heading, double pitch) {
			double sinH = Math.sin(heading), cosH = Math.cos(heading);
			double sinP = Math.sin(pitch), cosP = Math.cos(pitch);
			var forward = new Vec3(sinH * cosP, cosH * cosP, sinP);
			var right = new Vec3(cosH, -sinH, 0);
			var up = new Vec3(-sinH * sinP, -cosH * sinP, cosP);
			return position.add(right.scale(local.x())).add(forward.scale(forwardOffset + local.y()))
					.add(up.scale(upOffset + local.z()));
		}

		/**
		 * Vehicle-origin-relative local (right, forward, up) samples at the centre and six tips.
		 */
		public Vec3[] terrainSamplePoints() {
			return new Vec3[] {new Vec3(0, forwardOffset, upOffset), new Vec3(0, forwardOffset + semiLength, upOffset),
					new Vec3(0, forwardOffset - semiLength, upOffset), new Vec3(semiBeam, forwardOffset, upOffset),
					new Vec3(-semiBeam, forwardOffset, upOffset), new Vec3(0, forwardOffset, upOffset + semiHeight),
					new Vec3(0, forwardOffset, upOffset - semiHeight)};
		}
	}

	public static Envelope envelope(VehicleConfig config) {
		if (config.surfaceLocked()) {
			return new Envelope(SURFACE_SHIP.semiLength() * config.hullHalfLength() / 75,
					SURFACE_SHIP.semiBeam() * config.hullHalfBeam() / 15,
					SURFACE_SHIP.semiHeight() * config.hullHalfBeam() / 15,
					SURFACE_SHIP.forwardOffset() * config.hullHalfLength() / 75,
					SURFACE_SHIP.upOffset() * config.hullHalfBeam() / 15);
		}
		return new Envelope(SEMI_LENGTH * config.hullHalfLength() / 37.5, SEMI_BEAM * config.hullHalfBeam() / 6,
				SEMI_HEIGHT * config.hullHalfBeam() / 6, AFT_OFFSET, UP_OFFSET * config.hullHalfBeam() / 6);
	}

	public static Envelope envelope(SubmarineSnapshot snapshot) {
		return snapshot.surfaceLocked() ? SURFACE_SHIP : SUBMARINE;
	}

	/**
	 * Vehicle-origin-relative physical contact samples shared by terrain physics and its overlay.
	 * These approximate the hull and appendages; navigation clearance does not enlarge them.
	 */
	public static Vec3[] terrainSamplePoints(VehicleConfig config) {
		if (config.surfaceLocked()) {
			return envelope(config).terrainSamplePoints();
		}
		// Conservative coverage of the submarine's hull and appendages.
		return new Vec3[] {Vec3.ZERO, new Vec3(0, 33.5, 0), new Vec3(0, -40, 0), new Vec3(config.hullHalfBeam(), 0, 0),
				new Vec3(-config.hullHalfBeam(), 0, 0), new Vec3(0, 0, 6.5), new Vec3(0, 0, -5)};
	}

	public static Vec3[] terrainSamplePoints(boolean surfaceLocked) {
		return terrainSamplePoints(surfaceLocked ? VehicleConfig.surfaceShip() : VehicleConfig.submarine());
	}

	/**
	 * Distance from a point (px, py, pz) to the nearest point on a submarine's hull ellipsoid.
	 * Returns 0 if the point is inside the ellipsoid.
	 *
	 * @param px
	 *            point x (e.g. torpedo position)
	 * @param py
	 *            point y
	 * @param pz
	 *            point z
	 * @param subX
	 *            sub center x
	 * @param subY
	 *            sub center y
	 * @param subZ
	 *            sub center z
	 * @param subHeading
	 *            sub heading in radians
	 * @param subPitch
	 *            sub pitch in radians
	 * @return distance to hull surface, or 0 if inside
	 */
	public static double distanceToHull(
		double px, double py, double pz, double subX, double subY, double subZ, double subHeading, double subPitch) {
		return distanceToHull(px, py, pz, subX, subY, subZ, subHeading, subPitch, SUBMARINE);
	}

	public static double distanceToHull(double px, double py, double pz, SubmarineEntity vehicle) {
		return distanceToHull(px, py, pz, vehicle.x(), vehicle.y(), vehicle.z(), vehicle.heading(), vehicle.pitch(),
				envelope(vehicle.vehicleConfig()));
	}

	public static double distanceToHull(Vec3 point, SubmarineSnapshot vehicle) {
		var pose = vehicle.pose();
		var position = pose.position();
		return distanceToHull(point.x(), point.y(), point.z(), position.x(), position.y(), position.z(), pose.heading(),
				pose.pitch(), envelope(vehicle));
	}

	public static double distanceToHull(
		double px, double py, double pz, double subX, double subY, double subZ, double subHeading, double subPitch,
		Envelope envelope) {
		double sinH = Math.sin(subHeading), cosH = Math.cos(subHeading);
		double sinP = Math.sin(subPitch), cosP = Math.cos(subPitch);
		double fwdX = sinH * cosP, fwdY = cosH * cosP, fwdZ = sinP;
		double rightX = cosH, rightY = -sinH;
		double upX = -sinH * sinP, upY = -cosH * sinP, upZ = cosP;

		// Align with the generated hull axis; offsets rotate with heading and pitch.
		double cx = subX + fwdX * envelope.forwardOffset() + upX * envelope.upOffset();
		double cy = subY + fwdY * envelope.forwardOffset() + upY * envelope.upOffset();
		double cz = subZ + fwdZ * envelope.forwardOffset() + upZ * envelope.upOffset();

		// Delta from sub center to point
		double dx = px - cx, dy = py - cy, dz = pz - cz;

		// Project into sub's local frame
		double localFwd = dx * fwdX + dy * fwdY + dz * fwdZ;
		double localRight = dx * rightX + dy * rightY;
		double localUp = dx * upX + dy * upY + dz * upZ;

		return distanceToEllipsoid(localFwd, localRight, localUp, envelope);
	}

	private static double distanceToEllipsoid(double localFwd, double localRight, double localUp, Envelope envelope) {
		double semiLength = envelope.semiLength(), semiBeam = envelope.semiBeam(), semiHeight = envelope.semiHeight();
		// Normalize by ellipsoid semi-axes.
		double normFwd = localFwd / semiLength;
		double normRight = localRight / semiBeam;
		double normUp = localUp / semiHeight;
		double normDist = Math.hypot(Math.hypot(normFwd, normRight), normUp);

		if (normDist <= 1.0)
			return 0; // inside ellipsoid

		// At the nearest surface point q, the displacement is parallel to the surface normal:
		// q_i = p_i * a_i^2 / (a_i^2 + lambda). The ellipsoid constraint then gives one
		// strictly decreasing equation in lambda >= 0. Scale lambda by the longest axis squared.
		double beamRatio = semiBeam * semiBeam / (semiLength * semiLength);
		double heightRatio = semiHeight * semiHeight / (semiLength * semiLength);
		double lower = 0;
		// Every normalized coordinate shrinks by at least 1 / normDist at this bound.
		double upper = normDist - 1;
		for (int i = 0; i < 80; i++) {
			double lambda = lower + (upper - lower) * 0.5;
			if (lambda == lower || lambda == upper)
				break;
			double fwd = normFwd / (1 + lambda);
			double right = normRight * (beamRatio / (beamRatio + lambda));
			double up = normUp * (heightRatio / (heightRatio + lambda));
			if (fwd * fwd + right * right + up * up > 1)
				lower = lambda;
			else
				upper = lambda;
		}
		double lambda = lower + (upper - lower) * 0.5;
		double nearestFwd = localFwd / (1 + lambda);
		double nearestRight = localRight * (beamRatio / (beamRatio + lambda));
		double nearestUp = localUp * (heightRatio / (heightRatio + lambda));
		return Math.hypot(Math.hypot(localFwd - nearestFwd, localRight - nearestRight), localUp - nearestUp);
	}

	/**
	 * Convenience: distance from a torpedo entity to a submarine entity's hull.
	 */
	public static double distanceToHull(TorpedoEntity torp, SubmarineEntity sub) {
		return distanceToHull(torp.x(), torp.y(), torp.z(), sub);
	}

	/**
	 * Distance from a torpedo's BOW (nose tip) to a submarine's hull surface. The bow is the
	 * forward-most point of the torpedo cylinder, which is the closest part of the torpedo to the
	 * target during approach.
	 *
	 * @param torpX
	 *            torpedo center x
	 * @param torpY
	 *            torpedo center y
	 * @param torpZ
	 *            torpedo center z
	 * @param torpHeading
	 *            torpedo heading (radians)
	 * @param torpPitch
	 *            torpedo pitch (radians)
	 * @param torpHalfLength
	 *            torpedo half-length (from VehicleConfig.hullHalfLength)
	 */
	public static double bowDistanceToHull(
		double torpX, double torpY, double torpZ, double torpHeading, double torpPitch, double torpHalfLength,
		double subX, double subY, double subZ, double subHeading, double subPitch) {
		return bowDistanceToHull(torpX, torpY, torpZ, torpHeading, torpPitch, torpHalfLength, subX, subY, subZ,
				subHeading, subPitch, SUBMARINE);
	}

	public static double bowDistanceToHull(TorpedoEntity torp, SubmarineEntity sub) {
		return bowDistanceToHull(torp.x(), torp.y(), torp.z(), torp.heading(), torp.pitch(),
				torp.vehicleConfig().hullHalfLength(), sub.x(), sub.y(), sub.z(), sub.heading(), sub.pitch(),
				envelope(sub.vehicleConfig()));
	}

	private static double bowDistanceToHull(
		double torpX, double torpY, double torpZ, double torpHeading, double torpPitch, double torpHalfLength,
		double subX, double subY, double subZ, double subHeading, double subPitch, Envelope envelope) {
		// Torpedo bow = center + forward * halfLength
		double cosP = Math.cos(torpPitch), sinP = Math.sin(torpPitch);
		double bowX = torpX + Math.sin(torpHeading) * cosP * torpHalfLength;
		double bowY = torpY + Math.cos(torpHeading) * cosP * torpHalfLength;
		double bowZ = torpZ + sinP * torpHalfLength;
		return distanceToHull(bowX, bowY, bowZ, subX, subY, subZ, subHeading, subPitch, envelope);
	}

	/**
	 * Convenience: bow distance from torpedo snapshot to submarine snapshot.
	 */
	public static double bowDistanceToHull(TorpedoSnapshot torp, SubmarineSnapshot sub) {
		var tp = torp.pose();
		var sp = sub.pose();
		double halfLen = se.hirt.searobots.api.VehicleConfig.torpedo().hullHalfLength(); // 2.5m
		return bowDistanceToHull(tp.position().x(), tp.position().y(), tp.position().z(), tp.heading(), tp.pitch(),
				halfLen, sp.position().x(), sp.position().y(), sp.position().z(), sp.heading(), sp.pitch(),
				envelope(sub));
	}
}
