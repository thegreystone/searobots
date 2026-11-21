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
package se.hirt.searobots.viewer.showreel;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import se.hirt.searobots.api.TerrainMap;

/**
 * How photogenic a generated world is: its islands, and the underwater cliffs and ridges a
 * submarine can pass. Also rates the scenery around any point, which {@link MatchIndex} samples
 * along each submarine's track to find the moments worth filming.
 * <p>
 * Coordinates are the simulation's: x east, y north, z up, metres, sea level at z = 0.
 */
public final class TerrainSurvey {
	/** Distance between survey samples. */
	private static final double STEP = 50;
	/** Surveyed radius around the arena centre: the battle area and a little of the margin. */
	private static final double RADIUS = 7500;
	/** Smallest island counted, in survey samples (0.04 km²). */
	private static final int MIN_ISLAND_SAMPLES = 16;
	/** A sample is a cliff when the seabed within ±100 m spans this much height... */
	private static final double CLIFF_RELIEF = 300;
	/** ...and it lies shallow enough for submarines to pass it. */
	private static final double CLIFF_MAX_DEPTH = -450;

	/** An island: its highest point and its land area. */
	public record Island(double x, double y, double peak, double areaKm2) {
	}

	/**
	 * Scenery around one position. {@code score} is 0..1; {@code featureBearing} is the compass
	 * bearing (radians, 0 north, clockwise) from the position to the tallest nearby terrain, the
	 * feature to frame behind the subject.
	 */
	public record Scenery(double score, double featureBearing, double wall, double relief, double clearance,
			double shoreDistance) {
	}

	private final TerrainMap terrain;
	private final List<Island> islands;
	private final double landFraction;
	private final double cliffFraction;
	private final double score;

	public TerrainSurvey(TerrainMap terrain) {
		this.terrain = terrain;
		int n = (int) Math.round(2 * RADIUS / STEP) + 1;
		double[] elevation = new double[n * n];
		boolean[] inside = new boolean[n * n];
		for (int r = 0; r < n; r++) {
			for (int c = 0; c < n; c++) {
				double x = -RADIUS + c * STEP, y = -RADIUS + r * STEP;
				elevation[r * n + c] = terrain.elevationAt(x, y);
				inside[r * n + c] = x * x + y * y <= RADIUS * RADIUS;
			}
		}

		int land = 0, water = 0, cliffs = 0;
		for (int r = 2; r < n - 2; r++) {
			for (int c = 2; c < n - 2; c++) {
				int i = r * n + c;
				if (!inside[i])
					continue;
				if (elevation[i] > 0) {
					land++;
					continue;
				}
				water++;
				if (elevation[i] < CLIFF_MAX_DEPTH)
					continue;
				double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
				for (int dr = -2; dr <= 2; dr++) {
					for (int dc = -2; dc <= 2; dc++) {
						double e = elevation[(r + dr) * n + c + dc];
						lo = Math.min(lo, e);
						hi = Math.max(hi, e);
					}
				}
				if (hi - lo >= CLIFF_RELIEF)
					cliffs++;
			}
		}
		landFraction = land / (double) Math.max(1, land + water);
		cliffFraction = cliffs / (double) Math.max(1, water);
		islands = findIslands(elevation, inside, n);

		// Generated seabeds are statistically much alike (about 1-2% of the shallower seabed has
		// 300 m cliffs), so islands carry most of the weight: several distinct ones, at least one big
		// and tall, but not a continent crowding the arena.
		double islandCount = Math.min(islands.size(), 8) / 8.0;
		double tallest = islands.isEmpty() ? 0 : Math.min(1, islands.getFirst().peak() / 250);
		double bigIsland = islands.stream().anyMatch(i -> i.areaKm2() >= 1) ? 1 : 0;
		double landBalance = landFraction < 0.02 ? landFraction / 0.02
				: landFraction <= 0.12 ? 1 : Math.max(0, 1 - (landFraction - 0.12) / 0.12);
		double drama = Math.min(1, cliffFraction / 0.02);
		score = 100 * (0.25 * islandCount + 0.20 * tallest + 0.10 * bigIsland + 0.20 * landBalance + 0.25 * drama);
	}

	private List<Island> findIslands(double[] elevation, boolean[] inside, int n) {
		var found = new ArrayList<Island>();
		boolean[] seen = new boolean[n * n];
		int[] queue = new int[n * n];
		for (int start = 0; start < n * n; start++) {
			if (seen[start] || elevation[start] <= 0 || !inside[start])
				continue;
			int head = 0, tail = 0, peakIndex = start;
			queue[tail++] = start;
			seen[start] = true;
			while (head < tail) {
				int i = queue[head++];
				if (elevation[i] > elevation[peakIndex])
					peakIndex = i;
				int r = i / n, c = i % n;
				for (int k = 0; k < 4; k++) {
					int rr = r + (k == 0 ? -1 : k == 1 ? 1 : 0), cc = c + (k == 2 ? -1 : k == 3 ? 1 : 0);
					if (rr < 0 || cc < 0 || rr >= n || cc >= n)
						continue;
					int j = rr * n + cc;
					if (!seen[j] && elevation[j] > 0) {
						seen[j] = true;
						queue[tail++] = j;
					}
				}
			}
			if (tail >= MIN_ISLAND_SAMPLES) {
				double x = -RADIUS + (peakIndex % n) * STEP, y = -RADIUS + (peakIndex / n) * STEP;
				found.add(new Island(x, y, elevation[peakIndex], tail * STEP * STEP / 1e6));
			}
		}
		found.sort(Comparator.comparingDouble(Island::peak).reversed());
		return List.copyOf(found);
	}

	private static final double[] RING_RADII = {120, 250, 400};
	private static final int RING_BEARINGS = 12;
	private static final double[] SHORE_RADII = {150, 300, 500, 800};

	/** Rates the scenery a camera near {@code (x, y, z)} would see. */
	public Scenery sceneryAt(double x, double y, double z) {
		double floor = terrain.elevationAt(x, y);
		double hi = floor, lo = floor, bearing = 0;
		for (double radius : RING_RADII) {
			for (int k = 0; k < RING_BEARINGS; k++) {
				double b = k * 2 * Math.PI / RING_BEARINGS;
				double e = terrain.elevationAt(x + radius * Math.sin(b), y + radius * Math.cos(b));
				if (e > hi) {
					hi = e;
					bearing = b;
				}
				lo = Math.min(lo, e);
			}
		}
		double shore = Double.POSITIVE_INFINITY;
		outer: for (double radius : SHORE_RADII) {
			for (int k = 0; k < 16; k++) {
				double b = k * 2 * Math.PI / 16;
				if (terrain.elevationAt(x + radius * Math.sin(b), y + radius * Math.cos(b)) > 0) {
					shore = radius;
					break outer;
				}
			}
		}
		double clearance = z - floor;
		double wall = Math.min(hi, 0) - z; // terrain rising beside the submarine, to the surface at most
		double relief = hi - lo;
		double score = 0.35 * clamp(relief / 250) + 0.30 * clamp((wall + 60) / 220)
				+ 0.15 * clamp((140 - clearance) / 110) + 0.20 * (shore <= 800 && z > -160 ? 1 - shore / 1600 : 0);
		return new Scenery(clamp(score), bearing, wall, relief, clearance, shore);
	}

	private static double clamp(double v) {
		return v < 0 ? 0 : v > 1 ? 1 : v;
	}

	public TerrainMap terrain() {
		return terrain;
	}

	/** Islands in the arena, tallest first. */
	public List<Island> islands() {
		return islands;
	}

	public double landFraction() {
		return landFraction;
	}

	public double cliffFraction() {
		return cliffFraction;
	}

	/** Overall photogenic rating, 0..100. */
	public double score() {
		return score;
	}

	@Override
	public String toString() {
		return String.format(Locale.ROOT, "terrain %.0f: %d islands (tallest %.0f m), land %.1f%%, cliffs %.1f%%",
				score, islands.size(), islands.isEmpty() ? 0 : islands.getFirst().peak(), 100 * landFraction,
				100 * cliffFraction);
	}
}
