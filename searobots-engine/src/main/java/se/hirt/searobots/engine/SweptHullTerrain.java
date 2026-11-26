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

import java.util.Arrays;

import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

/** First physical terrain contact along a tick's translated and rotated hull path. */
final class SweptHullTerrain {

	private static final double CONTACT_TOLERANCE = 1e-9;
	private static final int MAX_PATH_SAMPLES = 1024;
	private static final int MAX_GRID_CUTS = 8192;
	private static final int MAX_BROADPHASE_CORNERS = 8192;

	private SweptHullTerrain() {
	}

	record Contact(double fraction, Vec3 position, double heading, double pitch, Vec3[] localPoints) {
	}

	private record GeometryQuery(Contact contact, double penetration) {
	}

	static Contact firstContact(
		Vec3 start, Vec3 end, double startHeading, double endHeading, double startPitch, double endPitch,
		VehicleConfig config, TerrainMap terrain) {
		double headingChange = Math.atan2(Math.sin(endHeading - startHeading), Math.cos(endHeading - startHeading));
		var path = new Path(start, end.subtract(start), startHeading, headingChange, startPitch, endPitch - startPitch,
				config, terrain);
		if (path.movement.length() == 0 && headingChange == 0 && endPitch == startPitch) {
			return null;
		}
		// Existing penetration belongs to static projection. Truncating it to t=0 would prevent
		// an embedded hull from moving out, and would turn position correction into an impact.
		if (path.clearsBounds(0, 1)) {
			return null;
		}
		double startSignedPenetration = path.maxSignedPenetration(0);
		if (startSignedPenetration > CONTACT_TOLERANCE) {
			return null;
		}

		double upper = 1;
		// With fixed attitude each material-point path crosses bilinear cells along a line.
		// Within each cell terrain minus point height is quadratic; its exact maximum catches
		// even a very shallow, narrow crest between time samples. The conservative keel and
		// appendage vertices are included in the same physical shape used by static contact.
		if (headingChange == 0 && endPitch == startPitch) {
			double sinH = Math.sin(startHeading), cosH = Math.cos(startHeading);
			double sinP = Math.sin(startPitch), cosP = Math.cos(startPitch);
			double fwdX = sinH * cosP, fwdY = cosH * cosP;
			double upX = -sinH * sinP, upY = -cosH * sinP;
			for (var local : path.fixedPoints) {
				double sx = start.x() + cosH * local.x() + fwdX * local.y() + upX * local.z();
				double sy = start.y() - sinH * local.x() + fwdY * local.y() + upY * local.z();
				double sz = start.z() + sinP * local.y() + cosP * local.z();
				upper = Math.min(upper, pointContact(path, sx, sy, sz, upper));
			}
		}
		double spacing = Math.min(config.hullHalfBeam() * 0.5, terrain.getCellSize() * 0.25);
		double travel = path.movement.length()
				+ path.radius * (Math.abs(headingChange) + Math.abs(endPitch - startPitch));
		int samples = Math.max(1, Math.min(MAX_PATH_SAMPLES, (int) Math.ceil(travel * upper / spacing)));
		double previous = 0;
		for (int i = 1; i <= samples; i++) {
			double fraction = upper * i / samples;
			if (!path.clearsBounds(previous, fraction)) {
				var query = path.query(fraction);
				if (query.penetration() > CONTACT_TOLERANCE) {
					return path.refine(previous, query, previous == 0 ? startSignedPenetration : Double.NaN);
				}
			}
			previous = fraction;
		}
		// An exact material-point crest can be shallower than the geometry query's tolerance.
		// pointContact deliberately returns a penetrating side of its bracket.
		return upper < 1 ? path.contact(upper) : null;
	}

	private static double pointContact(Path path, double sx, double sy, double sz, double maximumFraction) {
		double mx = path.movement.x(), my = path.movement.y(), mz = path.movement.z();
		double ex = sx + mx * maximumFraction, ey = sy + my * maximumFraction, ez = sz + mz * maximumFraction;
		if (Math.min(sz, ez) >= path.terrain.getMaxElevation()) {
			return 1;
		}
		double[] cuts = new double[16];
		int cutCount = 0;
		cuts[cutCount++] = 0.0;
		cuts[cutCount++] = maximumFraction;
		var buffer = new CutBuffer(cuts, cutCount);
		addCuts(buffer, sx, ex, path.terrain.getOriginX(), path.terrain.getCellSize(), path.terrain.getCols(),
				maximumFraction);
		addCuts(buffer, sy, ey, path.terrain.getOriginY(), path.terrain.getCellSize(), path.terrain.getRows(),
				maximumFraction);
		cuts = buffer.values;
		cutCount = buffer.size;
		if (cutCount > 2) {
			Arrays.sort(cuts, 0, cutCount);
		}
		double previous = 0;
		for (int i = 1; i < cutCount; i++) {
			double fraction = cuts[i];
			if (fraction > previous) {
				double middle = (previous + fraction) * 0.5;
				double q0 = path.terrain.elevationAt(sx + mx * previous, sy + my * previous) - (sz + mz * previous);
				double qm = path.terrain.elevationAt(sx + mx * middle, sy + my * middle) - (sz + mz * middle);
				double q1 = path.terrain.elevationAt(sx + mx * fraction, sy + my * fraction) - (sz + mz * fraction);
				double quadratic = 2 * (q0 + q1 - 2 * qm), linear = q1 - q0 - quadratic;
				double testFraction = q1 > CONTACT_TOLERANCE ? fraction : Double.POSITIVE_INFINITY;
				if (quadratic < -1e-12) {
					double maximum = -linear / (2 * quadratic);
					if (maximum > 0 && maximum < 1
							&& quadratic * maximum * maximum + linear * maximum + q0 > CONTACT_TOLERANCE) {
						testFraction = previous + (fraction - previous) * maximum;
					}
				}
				if (Double.isFinite(testFraction)) {
					double lower = previous, upper = testFraction;
					for (int iter = 0; iter < 32; iter++) {
						double candidate = (lower + upper) * 0.5;
						double pen = path.terrain.elevationAt(sx + mx * candidate, sy + my * candidate)
								- (sz + mz * candidate);
						if (pen > CONTACT_TOLERANCE) {
							upper = candidate;
						} else {
							lower = candidate;
						}
					}
					return upper;
				}
			}
			previous = fraction;
		}
		return 1;
	}

	private static final class CutBuffer {
		double[] values;
		int size;

		CutBuffer(double[] values, int size) {
			this.values = values;
			this.size = size;
		}

		void add(double v) {
			if (size == values.length) {
				values = Arrays.copyOf(values, values.length * 2);
			}
			values[size++] = v;
		}
	}

	private static void addCuts(
		CutBuffer cuts, double start, double end, double origin, double cell, int count, double maximumFraction) {
		if (Math.abs(end - start) < 1e-12) {
			return;
		}
		double lower = Math.max(Math.min(start, end), origin - cell);
		double upper = Math.min(Math.max(start, end), origin + count * cell);
		double stride = cell * Math.max(1, Math.ceil((upper - lower) / cell / MAX_GRID_CUTS));
		for (double grid = origin + Math.ceil((lower - origin) / stride) * stride; grid <= upper; grid += stride) {
			double fraction = maximumFraction * (grid - start) / (end - start);
			if (fraction > 0 && fraction < maximumFraction) {
				cuts.add(fraction);
			}
		}
	}

	private static final class Path {
		final Vec3 start, movement;
		final double heading, headingChange, pitch, pitchChange, radius;
		final VehicleConfig config;
		final TerrainMap terrain;
		final HullGeometry.Envelope envelope;
		final Vec3[] fixedPoints;

		Path(Vec3 start, Vec3 movement, double heading, double headingChange, double pitch, double pitchChange,
				VehicleConfig config, TerrainMap terrain) {
			this.start = start;
			this.movement = movement;
			this.heading = heading;
			this.headingChange = headingChange;
			this.pitch = pitch;
			this.pitchChange = pitchChange;
			this.config = config;
			this.terrain = terrain;
			fixedPoints = HullGeometry.sharedTerrainSamplePoints(config);
			envelope = HullGeometry.envelope(config);
			double bound = envelope.boundingRadius() + Math.hypot(envelope.forwardOffset(), envelope.upOffset());
			for (var point : fixedPoints) {
				bound = Math.max(bound, point.length());
			}
			radius = bound;
		}

		double maxSignedPenetration(double fraction) {
			var position = start.add(movement.scale(fraction));
			double h = heading + headingChange * fraction, p = pitch + pitchChange * fraction;
			var points = HullGeometry.terrainContactPoints(position, h, p, config, terrain);
			double maxSigned = Double.NEGATIVE_INFINITY;
			if (points.length > 0) {
				double sinH = Math.sin(h), cosH = Math.cos(h), sinP = Math.sin(p), cosP = Math.cos(p);
				double fwdX = sinH * cosP, fwdY = cosH * cosP;
				double upX = -sinH * sinP, upY = -cosH * sinP;
				for (var local : points) {
					double wx = position.x() + cosH * local.x() + fwdX * local.y() + upX * local.z();
					double wy = position.y() - sinH * local.x() + fwdY * local.y() + upY * local.z();
					double wz = position.z() + sinP * local.y() + cosP * local.z();
					maxSigned = Math.max(maxSigned, terrain.elevationAt(wx, wy) - wz);
				}
			}
			return maxSigned;
		}

		Contact contact(double fraction) {
			var position = start.add(movement.scale(fraction));
			double h = heading + headingChange * fraction, p = pitch + pitchChange * fraction;
			return new Contact(fraction, position, h, p,
					HullGeometry.terrainContactPoints(position, h, p, config, terrain));
		}

		GeometryQuery query(double fraction) {
			var contact = contact(fraction);
			var points = contact.localPoints();
			double penetration = 0;
			if (points.length > 0) {
				var position = contact.position();
				double h = contact.heading(), p = contact.pitch();
				double sinH = Math.sin(h), cosH = Math.cos(h), sinP = Math.sin(p), cosP = Math.cos(p);
				double fwdX = sinH * cosP, fwdY = cosH * cosP;
				double upX = -sinH * sinP, upY = -cosH * sinP;
				for (var local : points) {
					double wx = position.x() + cosH * local.x() + fwdX * local.y() + upX * local.z();
					double wy = position.y() - sinH * local.x() + fwdY * local.y() + upY * local.z();
					double wz = position.z() + sinP * local.y() + cosP * local.z();
					penetration = Math.max(penetration, terrain.elevationAt(wx, wy) - wz);
				}
			}
			return new GeometryQuery(contact, penetration);
		}

		Contact refine(double lower, GeometryQuery upperQuery, double startSignedPenetration) {
			double upper = upperQuery.contact().fraction();
			double targetSpan = (upper - lower) * (1.0 / (1 << 28));
			// A grounded hull already touching terrain at t=0 crosses the contact tolerance
			// almost immediately; one linear probe shrinks the bracket before bisection.
			if (lower == 0 && startSignedPenetration >= -1e-6 && upperQuery.penetration() > startSignedPenetration) {
				double estimated = upper * (CONTACT_TOLERANCE - startSignedPenetration)
						/ (upperQuery.penetration() - startSignedPenetration);
				double probe = Math.clamp(2.0 * estimated, upper * 1e-8, upper * 0.25);
				if (probe > lower && probe < upper) {
					var query = query(probe);
					if (query.penetration() > CONTACT_TOLERANCE) {
						upper = probe;
						upperQuery = query;
					} else {
						lower = probe;
					}
				}
			}
			for (int i = 0; i < 28 && upper - lower > targetSpan; i++) {
				double middle = (lower + upper) * 0.5;
				if (middle == lower || middle == upper) {
					break;
				}
				var query = query(middle);
				if (query.penetration() > CONTACT_TOLERANCE) {
					upper = middle;
					upperQuery = query;
				} else {
					lower = middle;
				}
			}
			// The penetrating side already has exact material points at this pose. Reuse
			// them instead of running the full ellipsoid/appendage query once more.
			return upperQuery.contact();
		}

		/** Bounds the entire translated and rotated hull interval, including its appendages. */
		boolean clearsBounds(double lower, double upper) {
			double middle = (lower + upper) * 0.5;
			double px = start.x() + movement.x() * middle;
			double py = start.y() + movement.y() * middle;
			double pz = start.z() + movement.z() * middle;
			double h = heading + headingChange * middle, p = pitch + pitchChange * middle;
			double sinH = Math.sin(h), cosH = Math.cos(h), sinP = Math.sin(p), cosP = Math.cos(p);
			double fwdX = sinH * cosP, fwdY = cosH * cosP;
			double upX = -sinH * sinP, upY = -cosH * sinP;
			double cx = px + fwdX * envelope.forwardOffset() + upX * envelope.upOffset();
			double cy = py + fwdY * envelope.forwardOffset() + upY * envelope.upOffset();
			double cz = pz + sinP * envelope.forwardOffset() + cosP * envelope.upOffset();
			double beam = envelope.semiBeam(), length = envelope.semiLength(), height = envelope.semiHeight();
			double bx = beam * cosH, lx = length * fwdX, hx = height * upX;
			double by = beam * sinH, ly = length * fwdY, hy = height * upY;
			double extentX = Math.sqrt(bx * bx + lx * lx + hx * hx);
			double extentY = Math.sqrt(by * by + ly * ly + hy * hy);
			double extentZ = Math.hypot(length * sinP, height * cosP);
			double minX = cx - extentX, maxX = cx + extentX;
			double minY = cy - extentY, maxY = cy + extentY;
			double minZ = cz - extentZ;
			for (var local : fixedPoints) {
				double wx = px + cosH * local.x() + fwdX * local.y() + upX * local.z();
				double wy = py - sinH * local.x() + fwdY * local.y() + upY * local.z();
				double wz = pz + sinP * local.y() + cosP * local.z();
				minX = Math.min(minX, wx);
				maxX = Math.max(maxX, wx);
				minY = Math.min(minY, wy);
				maxY = Math.max(maxY, wy);
				minZ = Math.min(minZ, wz);
			}
			double half = (upper - lower) * 0.5;
			double angular = radius * (Math.abs(headingChange) + Math.abs(pitchChange)) * half;
			minX -= Math.abs(movement.x()) * half + angular;
			maxX += Math.abs(movement.x()) * half + angular;
			minY -= Math.abs(movement.y()) * half + angular;
			maxY += Math.abs(movement.y()) * half + angular;
			minZ -= Math.abs(movement.z()) * half + radius * Math.abs(pitchChange) * half;
			if (minZ >= terrain.getMaxElevation()) {
				return true;
			}
			double cell = terrain.getCellSize();
			int firstCol = (int) Math.max(0, Math.floor((minX - terrain.getOriginX()) / cell));
			int lastCol = (int) Math.min(terrain.getCols() - 1, Math.floor((maxX - terrain.getOriginX()) / cell) + 1);
			int firstRow = (int) Math.max(0, Math.floor((minY - terrain.getOriginY()) / cell));
			int lastRow = (int) Math.min(terrain.getRows() - 1, Math.floor((maxY - terrain.getOriginY()) / cell) + 1);
			if ((long) (lastCol - firstCol + 1) * (lastRow - firstRow + 1) > MAX_BROADPHASE_CORNERS) {
				return false;
			}
			double maximum = terrain.getMinElevation();
			for (int row = firstRow; row <= lastRow; row++) {
				for (int col = firstCol; col <= lastCol; col++) {
					maximum = Math.max(maximum, terrain.elevationAtGrid(col, row));
				}
			}
			return minZ >= maximum;
		}
	}
}
