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
import java.util.Comparator;
import java.util.PriorityQueue;

/**
 * Forms gently rising beach shelves inside suitable island coasts. The original shoreline ring
 * stays untouched: bilinear cells containing water retain all four original corner elevations.
 */
final class IslandBeachShaper {

	private static final float DIAGONAL = (float) Math.sqrt(2);
	private static final double MAX_REACH_METERS = 270;
	// Permit natural steep hills behind the shelf, while preventing carving from adding a
	// near-vertical wall. A gentler bound can raise an entire narrow beach back into the hills.
	private static final double MAX_ADDED_SLOPE = .75;

	private IslandBeachShaper() {
	}

	/** Shapes above-water elevations in place without drawing from the world's generation RNG. */
	static void shape(
		double[] elevations, int cols, int rows, double originX, double originY, double cellSize, long worldSeed) {
		float[] distances = new float[elevations.length];
		Arrays.fill(distances, Float.POSITIVE_INFINITY);
		int[] anchors = new int[elevations.length];
		Arrays.fill(anchors, -1);
		float[] suitability = new float[elevations.length];

		// Protect diagonal neighbors too: each corner affects an entire bilinear cell, including
		// its underwater portion. Edge nodes are protected but do not create artificial beaches.
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				int index = row * cols + col;
				if (elevations[index] <= 0)
					continue;
				boolean edge = col == 0 || row == 0 || col == cols - 1 || row == rows - 1;
				if (!edge && !touchesWater(elevations, cols, col, row))
					continue;
				distances[index] = 0;
				anchors[index] = index;
				if (!edge) {
					double dx = (elevations[index + 1] - elevations[index - 1]) / (2 * cellSize);
					double dy = (elevations[index + cols] - elevations[index - cols]) / (2 * cellSize);
					double slope = Math.hypot(dx, dy);
					suitability[index] = (float) ((1 - smoothstep(4, 12, elevations[index]))
							* (1 - smoothstep(.18, .35, slope)));
				}
			}
		}

		// An eight-connected chamfer field is linear in map size; retain the nearest shoreline
		// anchor so adjacent beach profiles share the same original coastal height and character.
		float maxDistance = (float) (MAX_REACH_METERS / cellSize);
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				int index = row * cols + col;
				if (elevations[index] <= 0)
					continue;
				if (col > 0)
					propagate(index, index - 1, 1, maxDistance, distances, anchors);
				if (row > 0) {
					propagate(index, index - cols, 1, maxDistance, distances, anchors);
					if (col > 0)
						propagate(index, index - cols - 1, DIAGONAL, maxDistance, distances, anchors);
					if (col < cols - 1)
						propagate(index, index - cols + 1, DIAGONAL, maxDistance, distances, anchors);
				}
			}
		}
		for (int row = rows - 1; row >= 0; row--) {
			for (int col = cols - 1; col >= 0; col--) {
				int index = row * cols + col;
				if (elevations[index] <= 0)
					continue;
				if (col < cols - 1)
					propagate(index, index + 1, 1, maxDistance, distances, anchors);
				if (row < rows - 1) {
					propagate(index, index + cols, 1, maxDistance, distances, anchors);
					if (col > 0)
						propagate(index, index + cols - 1, DIAGONAL, maxDistance, distances, anchors);
					if (col < cols - 1)
						propagate(index, index + cols + 1, DIAGONAL, maxDistance, distances, anchors);
				}
			}
		}

		var coastNoise = new PerlinNoise(worldSeed ^ 0x424541434853484cL);
		float[] lowering = new float[elevations.length];
		for (int index = 0; index < elevations.length; index++) {
			int anchor = anchors[index];
			if (anchor < 0 || distances[index] == 0 || suitability[anchor] == 0)
				continue;
			double distance = distances[index] * cellSize;
			double variation = coastVariation(coastNoise, anchor, cols, originX, originY, cellSize);
			double width = 70 + variation * 50;
			double target = beachHeight(elevations[anchor], distance, variation);
			if (target >= elevations[index])
				continue;

			// Restore the original hills behind the shelf rather than compressing the entire island.
			double amount = suitability[anchor] * (1 - smoothstep(width, width + 150, distance));
			lowering[index] = (float) ((elevations[index] - target) * amount);
		}

		// Nearest coastal anchors can change abruptly at a headland. Smooth the proposed carve,
		// not the original terrain, so the edge of a shelf does not become a one-cell quarry wall.
		// The eligibility buffer is no longer needed and can hold the horizontal filtering pass.
		float[] horizontal = suitability;
		float[] kernel = smoothingKernel(cellSize);
		int radius = kernel.length / 2;
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				int index = row * cols + col;
				horizontal[index] = 0;
				if (anchors[index] < 0 || distances[index] == 0)
					continue;
				for (int offset = -radius; offset <= radius; offset++) {
					if (col + offset >= 0 && col + offset < cols)
						horizontal[index] += lowering[index + offset] * kernel[offset + radius];
				}
			}
		}
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				int index = row * cols + col;
				int anchor = anchors[index];
				if (anchor < 0 || distances[index] == 0)
					continue;
				double carve = 0;
				for (int offset = -radius; offset <= radius; offset++) {
					if (row + offset >= 0 && row + offset < rows)
						carve += horizontal[index + offset * cols] * kernel[offset + radius];
				}
				if (carve == 0)
					continue;
				double variation = coastVariation(coastNoise, anchor, cols, originX, originY, cellSize);
				double minimum = beachHeight(elevations[anchor], distances[index] * cellSize, variation);
				// Neighboring shelves may lend their smoothing influence to low land. Keep every
				// affected node above its own gently rising coastal profile, preserving the land mask.
				// Compensate for averaging a narrow beach with its uncarved surroundings. The local
				// profile still bounds the final depth, while the Gaussian retains gradual sidewalls.
				carve = Math.min(carve * 1.6, Math.max(0, elevations[index] - minimum));
				lowering[index] = (float) carve;
			}
		}

		// Clamping against different coastal profiles can undo the Gaussian at a headland or
		// low pocket. Extend those constraints gradually into the shelf instead of leaving a
		// one-cell cut. Only reduce carving: coast nodes, water, and original relief remain fixed.
		limitTransitionSlopes(elevations, lowering, cols, rows, cellSize);
		for (int index = 0; index < elevations.length; index++)
			elevations[index] -= lowering[index];
	}

	private record CarveDepth(int index, float depth) {
	}

	/**
	 * Finds the deepest spatially coherent carve below the proposed depth field. Each neighbor
	 * constraint bounds the resulting grade by its original grade plus {@link #MAX_ADDED_SLOPE}.
	 * Propagating shallow constraints by depth also handles narrow shelves and nearby low land,
	 * without smoothing an untouched cliff or taking more material out of the island.
	 */
	private static void limitTransitionSlopes(
		double[] elevations, float[] lowering, int cols, int rows, double cellSize) {
		var pending = new PriorityQueue<CarveDepth>(
				Comparator.comparingDouble(CarveDepth::depth).thenComparingInt(CarveDepth::index));
		for (int index = 0; index < lowering.length; index++) {
			if (lowering[index] <= 0)
				continue;
			pending.add(new CarveDepth(index, lowering[index]));
			int col = index % cols;
			int row = index / cols;
			for (int dy = -1; dy <= 1; dy++) {
				for (int dx = -1; dx <= 1; dx++) {
					if ((dx == 0 && dy == 0) || col + dx < 0 || col + dx >= cols || row + dy < 0 || row + dy >= rows)
						continue;
					int neighbor = index + dy * cols + dx;
					if (lowering[neighbor] == 0)
						pending.add(new CarveDepth(neighbor, 0));
				}
			}
		}
		while (!pending.isEmpty()) {
			CarveDepth next = pending.remove();
			int index = next.index();
			if (next.depth() != lowering[index])
				continue;
			int col = index % cols;
			int row = index / cols;
			for (int dy = -1; dy <= 1; dy++) {
				for (int dx = -1; dx <= 1; dx++) {
					if ((dx == 0 && dy == 0) || col + dx < 0 || col + dx >= cols || row + dy < 0 || row + dy >= rows)
						continue;
					int neighbor = index + dy * cols + dx;
					if (lowering[neighbor] <= 0)
						continue;
					double originalRise = elevations[neighbor] - elevations[index];
					double permittedRise = Math.abs(originalRise) + MAX_ADDED_SLOPE * cellSize * Math.hypot(dx, dy);
					float depthLimit = (float) (next.depth() + originalRise + permittedRise);
					if (lowering[neighbor] > depthLimit) {
						lowering[neighbor] = depthLimit;
						pending.add(new CarveDepth(neighbor, depthLimit));
					}
				}
			}
		}
	}

	private static double coastVariation(
		PerlinNoise noise, int anchor, int cols, double originX, double originY, double cellSize) {
		double anchorX = originX + (anchor % cols) * cellSize;
		double anchorY = originY + (anchor / cols) * cellSize;
		return Math.max(0, Math.min(1, .5 + .5 * noise.fbm(anchorX / 420 + 17.3, anchorY / 420 - 29.1, 3, 2, .5)));
	}

	private static double beachHeight(double anchorHeight, double distance, double variation) {
		// Ease from the original coastal ring into the shelf over a few tens of metres. The
		// extra rise approaches two metres, retaining a strictly uphill profile without a berm.
		return anchorHeight + distance * (.035 + variation * .015) + 2 * (1 - Math.exp(-distance / 12));
	}

	private static float[] smoothingKernel(double cellSize) {
		int radius = Math.max(1, (int) Math.ceil(40 / cellSize));
		float[] kernel = new float[2 * radius + 1];
		double total = 0;
		for (int offset = -radius; offset <= radius; offset++) {
			double metres = offset * cellSize;
			kernel[offset + radius] = (float) Math.exp(-.5 * metres * metres / (20 * 20));
			total += kernel[offset + radius];
		}
		for (int index = 0; index < kernel.length; index++)
			kernel[index] /= total;
		return kernel;
	}

	private static boolean touchesWater(double[] elevations, int cols, int col, int row) {
		for (int dy = -1; dy <= 1; dy++) {
			for (int dx = -1; dx <= 1; dx++) {
				if (elevations[(row + dy) * cols + col + dx] <= 0)
					return true;
			}
		}
		return false;
	}

	private static void propagate(int index, int neighbor, float step, float limit, float[] distances, int[] anchors) {
		float candidate = distances[neighbor] + step;
		if (candidate < distances[index] && candidate <= limit) {
			distances[index] = candidate;
			anchors[index] = anchors[neighbor];
		}
	}

	private static double smoothstep(double low, double high, double value) {
		double t = Math.max(0, Math.min(1, (value - low) / (high - low)));
		return t * t * (3 - 2 * t);
	}
}
