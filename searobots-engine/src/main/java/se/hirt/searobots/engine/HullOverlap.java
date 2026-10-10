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

/**
 * Oriented hull-ellipsoid intersection using the Perram-Wertheim contact function, J. Comput. Phys.
 * 58 (1985), 409-416, doi:10.1016/0021-9991(85)90171-8. Unlike surface samples, this also detects
 * side contacts where neither hull contains the other's centre or tips.
 */
final class HullOverlap {

	private static final double CONTACT_TOLERANCE = 1e-10;
	private static final int MAX_ITERATIONS = 48;

	private HullOverlap() {
	}

	static boolean overlaps(SubmarineEntity a, SubmarineEntity b) {
		var hullA = hull(a);
		var hullB = hull(b);
		var delta = hullB.center.subtract(hullA.center);
		double distanceSquared = delta.dot(delta);
		if (!Double.isFinite(distanceSquared)) {
			return false;
		}
		// The longest semi-axis bounds every orientation; reject distant pairs cheaply.
		double diameter = 2 * HullGeometry.SEMI_LENGTH;
		if (distanceSquared > diameter * diameter * (1.0 + CONTACT_TOLERANCE)) {
			return false;
		}
		if (distanceSquared == 0.0) {
			return true;
		}

		var difference = hullB.shape.subtract(hullA.shape);
		double lower = 0.0;
		double upper = 1.0;
		// F(t) = t(1-t) delta^T [(1-t)A+tB]^-1 delta is concave. Its maximum
		// is <= 1 exactly when the closed ellipsoids intersect. Bisect F' to find it.
		for (int i = 0; i < MAX_ITERATIONS; i++) {
			double t = (lower + upper) * 0.5;
			var solved = hullA.shape.blend(hullB.shape, t).solve(delta);
			double quadratic = delta.dot(solved);
			double contact = t * (1.0 - t) * quadratic;
			if (contact > 1.0 + CONTACT_TOLERANCE) {
				return false; // Any such value proves the maximum exceeds one.
			}
			double derivative = (1.0 - 2.0 * t) * quadratic - t * (1.0 - t) * difference.quadratic(solved);
			if (derivative == 0.0) {
				return true;
			}
			if (derivative > 0.0) {
				lower = t;
			} else {
				upper = t;
			}
		}
		return true;
	}

	private static Hull hull(SubmarineEntity sub) {
		double sinH = Math.sin(sub.heading()), cosH = Math.cos(sub.heading());
		double sinP = Math.sin(sub.pitch()), cosP = Math.cos(sub.pitch());
		var forward = new Vec3(sinH * cosP, cosH * cosP, sinP);
		var right = new Vec3(cosH, -sinH, 0);
		var up = new Vec3(-sinH * sinP, -cosH * sinP, cosP);
		var center = new Vec3(sub.x(), sub.y(), sub.z()).add(forward.scale(HullGeometry.AFT_OFFSET))
				.add(up.scale(HullGeometry.UP_OFFSET));
		return new Hull(center, Shape.axis(forward, HullGeometry.SEMI_LENGTH)
				.add(Shape.axis(right, HullGeometry.SEMI_BEAM)).add(Shape.axis(up, HullGeometry.SEMI_HEIGHT)));
	}

	private record Hull(Vec3 center, Shape shape) {
	}

	/** Six independent entries of a symmetric 3x3 shape matrix (squared semi-axes). */
	private record Shape(double xx, double xy, double xz, double yy, double yz, double zz) {
		static Shape axis(Vec3 direction, double radius) {
			double squared = radius * radius;
			return new Shape(squared * direction.x() * direction.x(), squared * direction.x() * direction.y(),
					squared * direction.x() * direction.z(), squared * direction.y() * direction.y(),
					squared * direction.y() * direction.z(), squared * direction.z() * direction.z());
		}

		Shape add(Shape other) {
			return new Shape(xx + other.xx, xy + other.xy, xz + other.xz, yy + other.yy, yz + other.yz, zz + other.zz);
		}

		Shape subtract(Shape other) {
			return new Shape(xx - other.xx, xy - other.xy, xz - other.xz, yy - other.yy, yz - other.yz, zz - other.zz);
		}

		Shape blend(Shape other, double t) {
			return new Shape((1 - t) * xx + t * other.xx, (1 - t) * xy + t * other.xy, (1 - t) * xz + t * other.xz,
					(1 - t) * yy + t * other.yy, (1 - t) * yz + t * other.yz, (1 - t) * zz + t * other.zz);
		}

		double quadratic(Vec3 v) {
			return xx * v.x() * v.x() + yy * v.y() * v.y() + zz * v.z() * v.z()
					+ 2 * (xy * v.x() * v.y() + xz * v.x() * v.z() + yz * v.y() * v.z());
		}

		Vec3 solve(Vec3 delta) {
			// Cholesky factorization: all blended shape matrices are positive definite.
			double l00 = Math.sqrt(xx);
			double l10 = xy / l00;
			double l20 = xz / l00;
			double l11 = Math.sqrt(yy - l10 * l10);
			double l21 = (yz - l20 * l10) / l11;
			double l22 = Math.sqrt(zz - l20 * l20 - l21 * l21);
			double y0 = delta.x() / l00;
			double y1 = (delta.y() - l10 * y0) / l11;
			double y2 = (delta.z() - l20 * y0 - l21 * y1) / l22;
			double z = y2 / l22;
			double y = (y1 - l21 * z) / l11;
			double x = (y0 - l10 * y - l20 * z) / l00;
			return new Vec3(x, y, z);
		}
	}
}
