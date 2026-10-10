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

import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.TreeSet;

/** Terrain-aware sampling of the physical hull, independent of navigation clearance. */
final class HullTerrainGeometry {

	// Local right/forward/up bounds of appendages outside the main body ellipsoid.
	static final double[][] APPENDAGES = {{-2.3, 2.3, -39, -35.4, -2.2, 2.4}, {-2.95, 2.95, -35.4, -30.6, -2.82, 3.04},
			{-2.1, 2.1, 10.4, 20.5, 3, 7.9}, {-6, -3.6, 7.4, 16.7, -0.29, 0.09}, {3.6, 6, 7.4, 16.7, -0.29, 0.09}};
	private static final int MAX_GRID_SAMPLES = 4096;

	private HullTerrainGeometry() {
	}

	static Vec3[] contactPoints(Vec3 position, double heading, double pitch, VehicleConfig config, TerrainMap terrain) {
		var points = new ArrayList<Vec3>(Arrays.asList(HullGeometry.terrainSamplePoints(config)));
		var shape = new Shape(position, heading, pitch, HullGeometry.envelope(config));
		var lowest = shape.support(new Vec3(0, 0, -1));
		points.add(shape.local(lowest));
		double lowestZ = lowest.z();
		for (var local : points) {
			lowestZ = Math.min(lowestZ, worldPoint(local, position, heading, pitch).z());
		}
		if (lowestZ >= terrain.getMaxElevation()
				|| clearsLocalBounds(shape, points, position, heading, pitch, lowestZ, terrain)) {
			return new Vec3[0];
		}
		if (terrain.getMinElevation() == terrain.getMaxElevation()) {
			return points.toArray(Vec3[]::new);
		}

		// Align to half-cells so both lattice crests and the intervening bilinear slopes are covered.
		// Limit work for exceptionally fine user maps; the ordinary 10 m map needs only tens of seeds.
		double minX = Math.max(shape.center.x() - shape.extentX, terrain.getOriginX() - terrain.getCellSize());
		double maxX = Math.min(shape.center.x() + shape.extentX,
				terrain.getOriginX() + terrain.getCols() * terrain.getCellSize());
		double minY = Math.max(shape.center.y() - shape.extentY, terrain.getOriginY() - terrain.getCellSize());
		double maxY = Math.min(shape.center.y() + shape.extentY,
				terrain.getOriginY() + terrain.getRows() * terrain.getCellSize());
		double spacing = terrain.getCellSize() * 0.5;
		double count = ((maxX - minX) / spacing + 2) * ((maxY - minY) / spacing + 2);
		if (count > MAX_GRID_SAMPLES) {
			spacing *= Math.ceil(Math.sqrt(count / MAX_GRID_SAMPLES));
		}
		double startX = terrain.getOriginX() + Math.ceil((minX - terrain.getOriginX()) / spacing) * spacing;
		double startY = terrain.getOriginY() + Math.ceil((minY - terrain.getOriginY()) / spacing) * spacing;
		for (double x = startX; x <= maxX; x += spacing) {
			for (double y = startY; y <= maxY; y += spacing) {
				var point = shape.lowerPoint(x, y);
				if (point != null) {
					points.add(shape.local(point));
					addSupports(points, point, shape, terrain);
				}
			}
		}
		addGridLineSupports(points, shape, terrain, minX, maxX, minY, maxY);
		// The global lower support also covers small footprints falling between lattice samples.
		addSupports(points, lowest, shape, terrain);
		if (!config.surfaceLocked()) {
			addAppendageSamples(points, position, heading, pitch, config, terrain);
		}
		return points.toArray(Vec3[]::new);
	}

	/** Exact normal support of each ellipsoid section along a terrain cell edge. */
	private static void addGridLineSupports(
		ArrayList<Vec3> points, Shape shape, TerrainMap terrain, double minX, double maxX, double minY, double maxY) {
		double cell = terrain.getCellSize();
		int firstCol = (int) Math.ceil((minX - terrain.getOriginX()) / cell);
		int lastCol = (int) Math.floor((maxX - terrain.getOriginX()) / cell);
		int firstRow = (int) Math.floor((minY - terrain.getOriginY()) / cell);
		int lastRow = (int) Math.floor((maxY - terrain.getOriginY()) / cell);
		long count = (long) (lastCol - firstCol + 2) * (lastRow - firstRow + 2);
		int stride = count <= MAX_GRID_SAMPLES ? 1 : (int) Math.ceil(Math.sqrt((double) count / MAX_GRID_SAMPLES));
		for (int col = firstCol; col <= lastCol; col += stride) {
			double x = terrain.getOriginX() + col * cell;
			for (int row = firstRow; row <= lastRow; row += stride) {
				double lower = Math.max(minY, terrain.getOriginY() + row * cell);
				double upper = Math.min(maxY, terrain.getOriginY() + (row + 1) * cell);
				if (upper > lower) {
					double slope = (terrain.elevationAt(x, upper) - terrain.elevationAt(x, lower)) / (upper - lower);
					var point = shape.sectionSupport(true, x, lower, upper, slope);
					if (point != null)
						points.add(shape.local(point));
				}
			}
		}
		firstCol = (int) Math.floor((minX - terrain.getOriginX()) / cell);
		firstRow = (int) Math.ceil((minY - terrain.getOriginY()) / cell);
		for (int row = firstRow; row <= lastRow; row += stride) {
			double y = terrain.getOriginY() + row * cell;
			for (int col = firstCol; col <= lastCol; col += stride) {
				double lower = Math.max(minX, terrain.getOriginX() + col * cell);
				double upper = Math.min(maxX, terrain.getOriginX() + (col + 1) * cell);
				if (upper > lower) {
					double slope = (terrain.elevationAt(upper, y) - terrain.elevationAt(lower, y)) / (upper - lower);
					var point = shape.sectionSupport(false, y, lower, upper, slope);
					if (point != null)
						points.add(shape.local(point));
				}
			}
		}
	}

	private static void addSupports(ArrayList<Vec3> points, Vec3 seed, Shape shape, TerrainMap terrain) {
		double sample = Math.min(1, terrain.getCellSize() * 0.05);
		var point = seed;
		for (int i = 0; i < 3; i++) {
			double slopeX = (terrain.elevationAt(point.x() + sample, point.y())
					- terrain.elevationAt(point.x() - sample, point.y())) / (2 * sample);
			double slopeY = (terrain.elevationAt(point.x(), point.y() + sample)
					- terrain.elevationAt(point.x(), point.y() - sample)) / (2 * sample);
			point = shape.support(new Vec3(slopeX, slopeY, -1));
			points.add(shape.local(point));
		}
	}

	private static void addAppendageSamples(
		ArrayList<Vec3> points, Vec3 position, double heading, double pitch, VehicleConfig config, TerrainMap terrain) {
		double lengthScale = config.hullHalfLength() / 37.5, beamScale = config.hullHalfBeam() / 6;
		for (var box : APPENDAGES) {
			var bounds = new double[] {box[0] * beamScale, box[1] * beamScale, box[2] * lengthScale,
					box[3] * lengthScale, box[4] * beamScale, box[5] * beamScale};
			var localCorners = new Vec3[8];
			var worldCorners = new Vec3[8];
			double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
			double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
			for (int i = 0; i < 8; i++) {
				localCorners[i] = new Vec3(bounds[(i >> 2) & 1], bounds[2 + ((i >> 1) & 1)], bounds[4 + (i & 1)]);
				worldCorners[i] = worldPoint(localCorners[i], position, heading, pitch);
				minX = Math.min(minX, worldCorners[i].x());
				maxX = Math.max(maxX, worldCorners[i].x());
				minY = Math.min(minY, worldCorners[i].y());
				maxY = Math.max(maxY, worldCorners[i].y());
			}
			// Bilinear terrain minus a planar box face has no interior strict maximum. The
			// lattice vertices and clipped face edges therefore cover contact extrema.
			minX = Math.max(minX, terrain.getOriginX() - terrain.getCellSize());
			maxX = Math.min(maxX, terrain.getOriginX() + terrain.getCols() * terrain.getCellSize());
			minY = Math.max(minY, terrain.getOriginY() - terrain.getCellSize());
			maxY = Math.min(maxY, terrain.getOriginY() + terrain.getRows() * terrain.getCellSize());
			if (minX > maxX || minY > maxY)
				continue;
			double spacing = terrain.getCellSize();
			double count = ((maxX - minX) / spacing + 2) * ((maxY - minY) / spacing + 2);
			if (count > MAX_GRID_SAMPLES)
				spacing *= Math.ceil(Math.sqrt(count / MAX_GRID_SAMPLES));
			double startX = terrain.getOriginX() + Math.ceil((minX - terrain.getOriginX()) / spacing) * spacing;
			double startY = terrain.getOriginY() + Math.ceil((minY - terrain.getOriginY()) / spacing) * spacing;
			for (double x = startX; x <= maxX; x += spacing) {
				for (double y = startY; y <= maxY; y += spacing) {
					var local = boxLowerPoint(x, y, position, heading, pitch, bounds);
					if (local != null)
						points.add(local);
				}
			}
			for (int i = 0; i < 8; i++) {
				for (int axis : new int[] {1, 2, 4}) {
					if ((i & axis) == 0) {
						addBoxEdge(points, localCorners[i], localCorners[i | axis], worldCorners[i],
								worldCorners[i | axis], terrain);
					}
				}
			}
		}
	}

	private static Vec3 boxLowerPoint(double x, double y, Vec3 position, double heading, double pitch, double[] box) {
		double sinH = Math.sin(heading), cosH = Math.cos(heading), sinP = Math.sin(pitch), cosP = Math.cos(pitch);
		double dx = x - position.x(), dy = y - position.y();
		double right = dx * cosH - dy * sinH;
		if (right < box[0] - 1e-10 || right > box[1] + 1e-10)
			return null;
		double forward = (dx * sinH + dy * cosH) * cosP, up = -(dx * sinH + dy * cosH) * sinP;
		double lower = Double.NEGATIVE_INFINITY, upper = Double.POSITIVE_INFINITY;
		for (int axis = 0; axis < 2; axis++) {
			double origin = axis == 0 ? forward : up, direction = axis == 0 ? sinP : cosP;
			double min = box[2 + axis * 2], max = box[3 + axis * 2];
			if (Math.abs(direction) < 1e-12) {
				if (origin < min - 1e-10 || origin > max + 1e-10)
					return null;
			} else {
				double t0 = (min - origin) / direction, t1 = (max - origin) / direction;
				lower = Math.max(lower, Math.min(t0, t1));
				upper = Math.min(upper, Math.max(t0, t1));
			}
		}
		if (lower > upper + 1e-10)
			return null;
		return new Vec3(right, forward + sinP * lower, up + cosP * lower);
	}

	private static void addBoxEdge(
		ArrayList<Vec3> points, Vec3 localStart, Vec3 localEnd, Vec3 worldStart, Vec3 worldEnd, TerrainMap terrain) {
		var cuts = new TreeSet<Double>();
		cuts.add(0.0);
		cuts.add(1.0);
		addGridCuts(cuts, worldStart.x(), worldEnd.x(), terrain.getOriginX(), terrain.getCellSize(), terrain.getCols());
		addGridCuts(cuts, worldStart.y(), worldEnd.y(), terrain.getOriginY(), terrain.getCellSize(), terrain.getRows());
		var localDelta = localEnd.subtract(localStart);
		var worldDelta = worldEnd.subtract(worldStart);
		double previous = 0;
		for (double cut : cuts) {
			points.add(localStart.add(localDelta.scale(cut)));
			if (cut > previous) {
				double q0 = penetrationAt(worldStart.add(worldDelta.scale(previous)), terrain);
				double qm = penetrationAt(worldStart.add(worldDelta.scale((previous + cut) * 0.5)), terrain);
				double q1 = penetrationAt(worldStart.add(worldDelta.scale(cut)), terrain);
				double quadratic = 2 * (q0 + q1 - 2 * qm), linear = q1 - q0 - quadratic;
				if (quadratic < -1e-12) {
					double maximum = -linear / (2 * quadratic);
					if (maximum > 0 && maximum < 1)
						points.add(localStart.add(localDelta.scale(previous + (cut - previous) * maximum)));
				}
			}
			previous = cut;
		}
	}

	private static void addGridCuts(
		TreeSet<Double> cuts, double start, double end, double origin, double cell, int count) {
		if (Math.abs(end - start) < 1e-12)
			return;
		double min = Math.max(Math.min(start, end), origin - cell);
		double max = Math.min(Math.max(start, end), origin + count * cell);
		double spacing = cell * Math.max(1, Math.ceil((max - min) / cell / MAX_GRID_SAMPLES));
		for (double grid = origin + Math.ceil((min - origin) / spacing) * spacing; grid <= max; grid += spacing) {
			double fraction = (grid - start) / (end - start);
			if (fraction > 0 && fraction < 1)
				cuts.add(fraction);
		}
	}

	private static double penetrationAt(Vec3 world, TerrainMap terrain) {
		return terrain.elevationAt(world.x(), world.y()) - world.z();
	}

	/** A bilinear cell's elevation cannot exceed its highest lattice corner. */
	private static boolean clearsLocalBounds(
		Shape shape, ArrayList<Vec3> points, Vec3 position, double heading, double pitch, double lowestZ,
		TerrainMap terrain) {
		double minX = shape.center.x() - shape.extentX, maxX = shape.center.x() + shape.extentX;
		double minY = shape.center.y() - shape.extentY, maxY = shape.center.y() + shape.extentY;
		for (var local : points) {
			var world = worldPoint(local, position, heading, pitch);
			minX = Math.min(minX, world.x());
			maxX = Math.max(maxX, world.x());
			minY = Math.min(minY, world.y());
			maxY = Math.max(maxY, world.y());
		}
		double cell = terrain.getCellSize();
		int minCol = (int) Math.max(0, Math.floor((minX - terrain.getOriginX()) / cell));
		int maxCol = (int) Math.min(terrain.getCols() - 1, Math.floor((maxX - terrain.getOriginX()) / cell) + 1);
		int minRow = (int) Math.max(0, Math.floor((minY - terrain.getOriginY()) / cell));
		int maxRow = (int) Math.min(terrain.getRows() - 1, Math.floor((maxY - terrain.getOriginY()) / cell) + 1);
		if ((long) (maxCol - minCol + 1) * (maxRow - minRow + 1) > MAX_GRID_SAMPLES)
			return false;
		double maximum = terrain.getMinElevation();
		for (int col = minCol; col <= maxCol; col++) {
			for (int row = minRow; row <= maxRow; row++) {
				maximum = Math.max(maximum, terrain.elevationAtGrid(col, row));
			}
		}
		return lowestZ >= maximum;
	}

	static Vec3 worldPoint(Vec3 local, Vec3 position, double heading, double pitch) {
		double sinH = Math.sin(heading), cosH = Math.cos(heading), sinP = Math.sin(pitch), cosP = Math.cos(pitch);
		return position.add(new Vec3(cosH, -sinH, 0).scale(local.x()))
				.add(new Vec3(sinH * cosP, cosH * cosP, sinP).scale(local.y()))
				.add(new Vec3(-sinH * sinP, -cosH * sinP, cosP).scale(local.z()));
	}

	private static final class Shape {
		final Vec3 center, right, forward, up;
		final HullGeometry.Envelope envelope;
		final double extentX, extentY;
		final double mxx, mxy, mxz, myy, myz, mzz;

		Shape(Vec3 position, double heading, double pitch, HullGeometry.Envelope envelope) {
			this.envelope = envelope;
			double sinH = Math.sin(heading), cosH = Math.cos(heading), sinP = Math.sin(pitch), cosP = Math.cos(pitch);
			right = new Vec3(cosH, -sinH, 0);
			forward = new Vec3(sinH * cosP, cosH * cosP, sinP);
			up = new Vec3(-sinH * sinP, -cosH * sinP, cosP);
			center = position.add(forward.scale(envelope.forwardOffset())).add(up.scale(envelope.upOffset()));
			double a2 = envelope.semiBeam() * envelope.semiBeam(), b2 = envelope.semiLength() * envelope.semiLength(),
					c2 = envelope.semiHeight() * envelope.semiHeight();
			extentX = Math.sqrt(a2 * right.x() * right.x() + b2 * forward.x() * forward.x() + c2 * up.x() * up.x());
			extentY = Math.sqrt(a2 * right.y() * right.y() + b2 * forward.y() * forward.y() + c2 * up.y() * up.y());
			mxx = right.x() * right.x() / a2 + forward.x() * forward.x() / b2 + up.x() * up.x() / c2;
			mxy = right.x() * right.y() / a2 + forward.x() * forward.y() / b2 + up.x() * up.y() / c2;
			mxz = forward.x() * forward.z() / b2 + up.x() * up.z() / c2;
			myy = right.y() * right.y() / a2 + forward.y() * forward.y() / b2 + up.y() * up.y() / c2;
			myz = forward.y() * forward.z() / b2 + up.y() * up.z() / c2;
			mzz = forward.z() * forward.z() / b2 + up.z() * up.z() / c2;
		}

		Vec3 support(Vec3 direction) {
			double x = right.dot(direction) * envelope.semiBeam(), y = forward.dot(direction) * envelope.semiLength(),
					z = up.dot(direction) * envelope.semiHeight();
			double length = Math.sqrt(x * x + y * y + z * z);
			return center.add(right.scale(x * envelope.semiBeam() / length))
					.add(forward.scale(y * envelope.semiLength() / length))
					.add(up.scale(z * envelope.semiHeight() / length));
		}

		Vec3 lowerPoint(double x, double y) {
			double dx = x - center.x(), dy = y - center.y();
			double b = mxz * dx + myz * dy, c = mxx * dx * dx + 2 * mxy * dx * dy + myy * dy * dy - 1;
			double discriminant = b * b - mzz * c;
			if (discriminant < -1e-12)
				return null;
			return new Vec3(x, y, center.z() + (-b - Math.sqrt(Math.max(0, discriminant))) / mzz);
		}

		Vec3 sectionSupport(boolean fixedX, double fixed, double lower, double upper, double slope) {
			double offset = fixed - (fixedX ? center.x() : center.y());
			double muu = fixedX ? myy : mxx, muv = fixedX ? myz : mxz, mff = fixedX ? mxx : myy;
			double lu = mxy * offset, lv = (fixedX ? mxz : myz) * offset;
			double determinant = muu * mzz - muv * muv;
			double iuu = mzz / determinant, iuv = -muv / determinant, ivv = muu / determinant;
			double centerU = -(iuu * lu + iuv * lv), centerV = -(iuv * lu + ivv * lv);
			double radiusSquared = 1 - mff * offset * offset - lu * centerU - lv * centerV;
			if (radiusSquared < -1e-12)
				return null;
			double radius = Math.sqrt(Math.max(0, radiusSquared));
			double originU = fixedX ? center.y() : center.x();
			lower = Math.max(lower, originU + centerU - radius * Math.sqrt(iuu));
			upper = Math.min(upper, originU + centerU + radius * Math.sqrt(iuu));
			if (lower > upper + 1e-10)
				return null;
			double supportU = iuu * slope - iuv, supportV = iuv * slope - ivv;
			double length = Math.sqrt(slope * supportU - supportV);
			double variable = originU + centerU + radius * supportU / length;
			if (variable < lower || variable > upper) {
				variable = Math.max(lower, Math.min(upper, variable));
				return fixedX ? lowerPoint(fixed, variable) : lowerPoint(variable, fixed);
			}
			double z = center.z() + centerV + radius * supportV / length;
			return fixedX ? new Vec3(fixed, variable, z) : new Vec3(variable, fixed, z);
		}

		Vec3 local(Vec3 world) {
			var delta = world.subtract(center);
			return new Vec3(delta.dot(right), delta.dot(forward) + envelope.forwardOffset(),
					delta.dot(up) + envelope.upOffset());
		}
	}
}
