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

import java.util.TreeSet;

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
		if (path.penetration(0) > CONTACT_TOLERANCE || path.clearsBounds(0, 1)) {
			return null;
		}

		double upper = 1;
		// With fixed attitude each material-point path crosses bilinear cells along a line.
		// Within each cell terrain minus point height is quadratic; its exact maximum catches
		// even a very shallow, narrow crest between time samples. The conservative keel and
		// appendage vertices are included in the same physical shape used by static contact.
		if (headingChange == 0 && endPitch == startPitch) {
			for (var local : HullGeometry.terrainSamplePoints(config)) {
				upper = Math.min(upper, pointContact(path, local, upper));
			}
		}
		double spacing = Math.min(config.hullHalfBeam() * 0.5, terrain.getCellSize() * 0.25);
		double travel = path.movement.length()
				+ path.radius * (Math.abs(headingChange) + Math.abs(endPitch - startPitch));
		int samples = Math.max(1, Math.min(MAX_PATH_SAMPLES, (int) Math.ceil(travel * upper / spacing)));
		double previous = 0;
		for (int i = 1; i <= samples; i++) {
			double fraction = upper * i / samples;
			if (!path.clearsBounds(previous, fraction) && path.penetration(fraction) > CONTACT_TOLERANCE) {
				return path.refine(previous, fraction);
			}
			previous = fraction;
		}
		// An exact material-point crest can be shallower than the geometry query's tolerance.
		// pointContact deliberately returns a penetrating side of its bracket.
		return upper < 1 ? path.contact(upper) : null;
	}

	private static double pointContact(Path path, Vec3 local, double maximumFraction) {
		var start = HullTerrainGeometry.worldPoint(local, path.start, path.heading, path.pitch);
		var end = start.add(path.movement.scale(maximumFraction));
		var cuts = new TreeSet<Double>();
		cuts.add(0.0);
		cuts.add(maximumFraction);
		addCuts(cuts, start.x(), end.x(), path.terrain.getOriginX(), path.terrain.getCellSize(), path.terrain.getCols(),
				maximumFraction);
		addCuts(cuts, start.y(), end.y(), path.terrain.getOriginY(), path.terrain.getCellSize(), path.terrain.getRows(),
				maximumFraction);
		double previous = 0;
		for (double fraction : cuts) {
			if (fraction > previous) {
				double middle = (previous + fraction) * 0.5;
				double q0 = pointPenetration(start.add(path.movement.scale(previous)), path.terrain);
				double qm = pointPenetration(start.add(path.movement.scale(middle)), path.terrain);
				double q1 = pointPenetration(start.add(path.movement.scale(fraction)), path.terrain);
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
					for (int i = 0; i < 32; i++) {
						double candidate = (lower + upper) * 0.5;
						if (pointPenetration(start.add(path.movement.scale(candidate)),
								path.terrain) > CONTACT_TOLERANCE) {
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

	private static void addCuts(
		TreeSet<Double> cuts, double start, double end, double origin, double cell, int count, double maximumFraction) {
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

	private static double pointPenetration(Vec3 world, TerrainMap terrain) {
		return terrain.elevationAt(world.x(), world.y()) - world.z();
	}

	private static final class Path {
		final Vec3 start, movement;
		final double heading, headingChange, pitch, pitchChange, radius;
		final VehicleConfig config;
		final TerrainMap terrain;
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
			fixedPoints = HullGeometry.terrainSamplePoints(config);
			var envelope = HullGeometry.envelope(config);
			double bound = envelope.boundingRadius() + Math.hypot(envelope.forwardOffset(), envelope.upOffset());
			for (var point : fixedPoints) {
				bound = Math.max(bound, point.length());
			}
			radius = bound;
		}

		double penetration(double fraction) {
			return HullGeometry.terrainPenetration(start.add(movement.scale(fraction)),
					heading + headingChange * fraction, pitch + pitchChange * fraction, config, terrain);
		}

		Contact contact(double fraction) {
			var position = start.add(movement.scale(fraction));
			double h = heading + headingChange * fraction, p = pitch + pitchChange * fraction;
			return new Contact(fraction, position, h, p,
					HullGeometry.terrainContactPoints(position, h, p, config, terrain));
		}

		Contact refine(double lower, double upper) {
			for (int i = 0; i < 28; i++) {
				double middle = (lower + upper) * 0.5;
				if (penetration(middle) > CONTACT_TOLERANCE) {
					upper = middle;
				} else {
					lower = middle;
				}
			}
			return contact(upper);
		}

		/** Bounds the entire translated and rotated hull interval, including its appendages. */
		boolean clearsBounds(double lower, double upper) {
			double middle = (lower + upper) * 0.5;
			var position = start.add(movement.scale(middle));
			double h = heading + headingChange * middle, p = pitch + pitchChange * middle;
			var envelope = HullGeometry.envelope(config);
			var center = envelope.worldPoint(Vec3.ZERO, position, h, p);
			double sinH = Math.sin(h), cosH = Math.cos(h), sinP = Math.sin(p), cosP = Math.cos(p);
			double beam = envelope.semiBeam(), length = envelope.semiLength(), height = envelope.semiHeight();
			double extentX = Math.sqrt(
					Math.pow(beam * cosH, 2) + Math.pow(length * sinH * cosP, 2) + Math.pow(height * sinH * sinP, 2));
			double extentY = Math.sqrt(
					Math.pow(beam * sinH, 2) + Math.pow(length * cosH * cosP, 2) + Math.pow(height * cosH * sinP, 2));
			double extentZ = Math.hypot(length * sinP, height * cosP);
			double minX = center.x() - extentX, maxX = center.x() + extentX;
			double minY = center.y() - extentY, maxY = center.y() + extentY;
			double minZ = center.z() - extentZ;
			for (var local : fixedPoints) {
				var world = HullTerrainGeometry.worldPoint(local, position, h, p);
				minX = Math.min(minX, world.x());
				maxX = Math.max(maxX, world.x());
				minY = Math.min(minY, world.y());
				maxY = Math.max(maxY, world.y());
				minZ = Math.min(minZ, world.z());
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
