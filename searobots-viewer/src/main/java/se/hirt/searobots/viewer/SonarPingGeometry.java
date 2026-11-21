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
package se.hirt.searobots.viewer;

import se.hirt.searobots.api.TerrainMap;

/** Geometry for a visual outgoing pulse; this does not change the engine's acoustic model. */
final class SonarPingGeometry {
	static final double SPEED = 1500;
	static final double MAX_RADIUS = 10000;
	private static final double SURFACE_LIFT = 0.8;
	private static final int ARC_STEPS = 96;
	/** Most segments in the surface circle: about a terrain cell each, up to this. */
	private static final int SURFACE_STEPS = 1440;
	private static final int MERIDIAN_COUNT = 8;
	private static final int LATITUDE_COUNT = 7;
	private static final int ARC_COUNT = MERIDIAN_COUNT + LATITUDE_COUNT;
	private final TerrainMap terrain;
	private final SonarTerrainVisibility visibility;
	private final double x, y, z;
	private final double[] directions = new double[ARC_COUNT * (ARC_STEPS + 1) * 3];
	private final double[] limits = new double[ARC_COUNT * (ARC_STEPS + 1)];
	private final double[] midpointLimits = new double[ARC_COUNT * ARC_STEPS];
	private final double[] corners = new double[16];
	private final double[] crossings = new double[6];

	SonarPingGeometry(TerrainMap terrain, double x, double y, double z) {
		this(terrain, x, y, z, SonarTerrainVisibility.forTerrain(terrain));
	}

	SonarPingGeometry(TerrainMap terrain, double x, double y, double z, SonarTerrainVisibility visibility) {
		this.terrain = terrain;
		this.visibility = visibility;
		this.x = x;
		this.y = y;
		this.z = z;
		for (int arc = 0; arc < ARC_COUNT; arc++) {
			for (int i = 0; i <= ARC_STEPS; i++) {
				double a = i * Math.PI * 2 / ARC_STEPS;
				double dx, dy, dz;
				if (arc < MERIDIAN_COUNT) {
					double heading = arc * Math.PI / MERIDIAN_COUNT;
					dx = Math.cos(a) * Math.cos(heading);
					dy = Math.cos(a) * Math.sin(heading);
					dz = Math.sin(a);
				} else {
					dz = -1 + 2.0 * (arc - MERIDIAN_COUNT + 1) / (LATITUDE_COUNT + 1);
					double ringRadius = Math.sqrt(1 - dz * dz);
					dx = Math.cos(a) * ringRadius;
					dy = Math.sin(a) * ringRadius;
				}
				int index = arc * (ARC_STEPS + 1) + i;
				directions[index * 3] = dx;
				directions[index * 3 + 1] = dy;
				directions[index * 3 + 2] = dz;
				limits[index] = firstHit(dx, dy, dz);
			}
			for (int i = 0; i < ARC_STEPS; i++) {
				int a = (arc * (ARC_STEPS + 1) + i) * 3, b = a + 3;
				double dx = (directions[a] + directions[b]) * 0.5;
				double dy = (directions[a + 1] + directions[b + 1]) * 0.5;
				double dz = (directions[a + 2] + directions[b + 2]) * 0.5;
				double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
				midpointLimits[arc * ARC_STEPS + i] = firstHit(dx / length, dy / length, dz / length) / length;
			}
		}
	}

	/** The wavefront's radius at {@code tick}, which may fall between ticks. */
	static double radius(double tick, long startTick, double tickRateHz) {
		return (tick - startTick) * SPEED / tickRateHz;
	}

	void sphere(double radius, Lines output) {
		output.clear();
		if (radius <= 0 || radius > MAX_RADIUS)
			return;
		for (int arc = 0; arc < ARC_COUNT; arc++) {
			for (int i = 0; i < ARC_STEPS; i++) {
				int a = arc * (ARC_STEPS + 1) + i, b = a + 1;
				if (radius > limits[a] || radius > limits[b] || radius > midpointLimits[arc * ARC_STEPS + i]) {
					cutAtSurface(a, b, radius, output);
					continue;
				}
				int p = a * 3, q = b * 3;
				double ax = x + directions[p] * radius, ay = y + directions[p + 1] * radius;
				double az = z + directions[p + 2] * radius;
				double bx = x + directions[q] * radius, by = y + directions[q + 1] * radius;
				double bz = z + directions[q + 2] * radius;
				output.segment(ax, ay, az, bx, by, bz);
			}
		}
	}

	/**
	 * The part below the water of a sphere segment that crosses the surface, so the sphere ends on
	 * the {@link #surface} circle instead of a whole segment short of it.
	 */
	private void cutAtSurface(int a, int b, double radius, Lines output) {
		int p = a * 3, q = b * 3;
		double az = z + directions[p + 2] * radius, bz = z + directions[q + 2] * radius;
		if ((az > 0) == (bz > 0))
			return;
		int below = az > 0 ? b : a, above = az > 0 ? a : b;
		if (radius > limits[below])
			return; // the lower end is behind terrain too
		int l = below * 3, u = above * 3;
		double lx = x + directions[l] * radius, ly = y + directions[l + 1] * radius,
				lz = z + directions[l + 2] * radius;
		double ux = x + directions[u] * radius, uy = y + directions[u + 1] * radius,
				uz = z + directions[u + 2] * radius;
		double t = -lz / (uz - lz);
		double cx = lx + (ux - lx) * t, cy = ly + (uy - ly) * t;
		if (reachesSurface(cx, cy))
			output.segment(lx, ly, lz, cx, cy, 0);
	}

	/**
	 * Where the wavefront meets the water surface: a circle above the source once the pulse has
	 * risen that far, over the open water it reaches, so not over land or in the lee of an island.
	 */
	void surface(double radius, Lines output) {
		output.clear();
		if (radius <= 0 || radius > MAX_RADIUS || z >= 0 || radius <= -z)
			return;
		double ring = Math.sqrt(radius * radius - z * z);
		int steps = (int) Math.min(SURFACE_STEPS,
				Math.max(ARC_STEPS, Math.ceil(2 * Math.PI * ring / terrain.getCellSize())));
		double step = 2 * Math.PI / steps;
		double previousX = x + ring, previousY = y;
		boolean previousOpen = reachesSurface(previousX, previousY);
		for (int i = 1; i <= steps; i++) {
			double angle = i * step;
			double px = x + Math.cos(angle) * ring, py = y + Math.sin(angle) * ring;
			boolean open = reachesSurface(px, py);
			if (previousOpen && open) {
				output.segment(previousX, previousY, SURFACE_LIFT, px, py, SURFACE_LIFT);
			} else if (previousOpen != open) {
				// A shore or the edge of a shadow: find it, so the circle ends there
				double low = angle - step, high = angle;
				for (int k = 0; k < 10; k++) {
					double middle = (low + high) * 0.5;
					if (reachesSurface(x + Math.cos(middle) * ring, y + Math.sin(middle) * ring) == previousOpen)
						low = middle;
					else
						high = middle;
				}
				double edge = previousOpen ? low : high;
				double ex = x + Math.cos(edge) * ring, ey = y + Math.sin(edge) * ring;
				if (previousOpen)
					output.segment(previousX, previousY, SURFACE_LIFT, ex, ey, SURFACE_LIFT);
				else
					output.segment(ex, ey, SURFACE_LIFT, px, py, SURFACE_LIFT);
			}
			previousX = px;
			previousY = py;
			previousOpen = open;
		}
	}

	/**
	 * Whether the pulse reaches the water surface at {@code (px, py)}: open water, in sight of the
	 * source.
	 */
	private boolean reachesSurface(double px, double py) {
		if (px < terrain.getOriginX() || px > terrain.getOriginX() + terrain.worldWidth() || py < terrain.getOriginY()
				|| py > terrain.getOriginY() + terrain.worldHeight())
			return false;
		return terrain.elevationAt(px, py) < 0 && visibility.clear(x, y, z, px, py, 0);
	}

	void bottom(double radius, Lines output) {
		output.clear();
		if (radius <= 0 || radius > MAX_RADIUS)
			return;
		double minZ = terrain.getMinElevation(), maxZ = Math.min(0, terrain.getMaxElevation());
		double nearest = z < minZ ? minZ - z : z > maxZ ? z - maxZ : 0;
		if (radius < nearest)
			return;
		double farthest = Math.max(Math.abs(z - minZ), Math.abs(z - maxZ));
		double inner = Math.sqrt(Math.max(0, radius * radius - farthest * farthest));
		// Near the source retain terrain resolution; bound work as the sphere grows.
		double step = Math.max(terrain.getCellSize(), radius / 96);
		int cells = (int) Math.ceil(radius / step) + 1;
		double squared = radius * radius;
		for (int row = -cells; row < cells; row++) {
			double y0 = y + row * step, y1 = y0 + step;
			if (y1 < terrain.getOriginY() || y0 > terrain.getOriginY() + terrain.worldHeight())
				continue;
			for (int col = -cells; col < cells; col++) {
				double x0 = x + col * step, x1 = x0 + step;
				if (x1 < terrain.getOriginX() || x0 > terrain.getOriginX() + terrain.worldWidth())
					continue;
				double cx = (col + 0.5) * step, cy = (row + 0.5) * step;
				double horizontal = Math.hypot(cx, cy);
				if (horizontal > radius + step || horizontal < inner - step)
					continue;
				corner(0, x0, y0, squared);
				corner(4, x1, y0, squared);
				corner(8, x1, y1, squared);
				corner(12, x0, y1, squared);
				triangle(0, 4, 8, squared, output);
				triangle(0, 8, 12, squared, output);
			}
		}
	}

	private void corner(int index, double px, double py, double squared) {
		double pz = terrain.elevationAt(px, py);
		corners[index] = px;
		corners[index + 1] = py;
		corners[index + 2] = pz;
		corners[index + 3] = distanceSquared(px, py, pz) - squared;
	}

	private void triangle(int a, int b, int c, double squared, Lines output) {
		int count = edge(a, b, squared, 0);
		count = edge(b, c, squared, count);
		count = edge(c, a, squared, count);
		if (count != 2)
			return;
		double ax = crossings[0], ay = crossings[1], az = crossings[2];
		double bx = crossings[3], by = crossings[4], bz = crossings[5];
		// Drape long contour segments over the original terrain so the coarse search grid
		// cannot bridge valleys or bury the line in a nearby peak at large ping radii.
		int pieces = Math.max(1, (int) Math.ceil(Math.hypot(bx - ax, by - ay) / terrain.getCellSize()));
		double previousX = ax, previousY = ay, previousZ = az;
		boolean previousVisible = az <= 0 && visibility.clear(x, y, z, ax, ay, az);
		for (int i = 1; i <= pieces; i++) {
			double t = (double) i / pieces;
			double px = ax + (bx - ax) * t, py = ay + (by - ay) * t;
			double pz = terrain.elevationAt(px, py);
			boolean visible = pz <= 0 && visibility.clear(x, y, z, px, py, pz);
			if (previousVisible && visible)
				output.segment(previousX, previousY, previousZ + SURFACE_LIFT, px, py, pz + SURFACE_LIFT);
			previousX = px;
			previousY = py;
			previousZ = pz;
			previousVisible = visible;
		}
	}

	private int edge(int a, int b, double squared, int count) {
		if (count >= 2 || (corners[a + 3] < 0) == (corners[b + 3] < 0))
			return count;
		double low = 0, high = 1;
		boolean lowInside = corners[a + 3] < 0;
		// Refine against the original bilinear terrain, not just the coarse visual grid.
		for (int i = 0; i < 10; i++) {
			double t = (low + high) * 0.5;
			double px = corners[a] + (corners[b] - corners[a]) * t;
			double py = corners[a + 1] + (corners[b + 1] - corners[a + 1]) * t;
			double pz = terrain.elevationAt(px, py);
			if ((distanceSquared(px, py, pz) < squared) == lowInside)
				low = t;
			else
				high = t;
		}
		double t = (low + high) * 0.5;
		double px = corners[a] + (corners[b] - corners[a]) * t;
		double py = corners[a + 1] + (corners[b + 1] - corners[a + 1]) * t;
		crossings[count * 3] = px;
		crossings[count * 3 + 1] = py;
		crossings[count * 3 + 2] = terrain.elevationAt(px, py);
		return count + 1;
	}

	private double distanceSquared(double px, double py, double pz) {
		return (px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z);
	}

	private double firstHit(double dx, double dy, double dz) {
		double maximum = dz > 0 ? Math.min(MAX_RADIUS, Math.max(0, -z / dz)) : MAX_RADIUS;
		double step = terrain.getCellSize() * 0.5 / Math.max(0.01, Math.hypot(dx, dy));
		step = Math.min(step, 50);
		for (double distance = step; distance <= maximum + step; distance += step) {
			double end = Math.min(maximum, distance);
			if (terrain.elevationAt(x + dx * end, y + dy * end) > z + dz * end) {
				double low = Math.max(0, distance - step), high = end;
				for (int i = 0; i < 10; i++) {
					double middle = (low + high) * 0.5;
					if (terrain.elevationAt(x + dx * middle, y + dy * middle) > z + dz * middle)
						high = middle;
					else
						low = middle;
				}
				return low;
			}
			if (end >= maximum)
				break;
		}
		return maximum;
	}

	/** Source visibility, independently of camera depth. The final bottom contact is allowed. */
	static boolean clearPath(TerrainMap terrain, double sx, double sy, double sz, double tx, double ty, double tz) {
		return SonarTerrainVisibility.forTerrain(terrain).clear(sx, sy, sz, tx, ty, tz);
	}

	/** Reusable, bounded segment buffer in JME coordinates (east, elevation, south). */
	static final class Lines {
		final float[] positions = new float[16384 * 6];
		int size;

		void clear() {
			size = 0;
		}

		void segment(double ax, double ay, double az, double bx, double by, double bz) {
			if (size + 6 > positions.length)
				return;
			positions[size++] = (float) ax;
			positions[size++] = (float) az;
			positions[size++] = (float) -ay;
			positions[size++] = (float) bx;
			positions[size++] = (float) bz;
			positions[size++] = (float) -by;
		}
	}
}
