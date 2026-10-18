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

import java.lang.ref.WeakReference;
import java.util.WeakHashMap;

/**
 * Conservative height bounds accelerate source rays without skipping a possible terrain blocker.
 */
final class SonarTerrainVisibility {
	private static final int[] BLOCKS = {128, 16, 4};
	private static final WeakHashMap<TerrainMap, WeakReference<SonarTerrainVisibility>> CACHE = new WeakHashMap<>();
	private final TerrainMap terrain;
	private final int[] columns = new int[BLOCKS.length], rows = new int[BLOCKS.length];
	private final double[][] maxima = new double[BLOCKS.length][];

	static synchronized SonarTerrainVisibility forTerrain(TerrainMap terrain) {
		var reference = CACHE.get(terrain);
		var cached = reference == null ? null : reference.get();
		if (cached != null)
			return cached;
		var result = new SonarTerrainVisibility(terrain);
		CACHE.put(terrain, new WeakReference<>(result));
		return result;
	}

	private SonarTerrainVisibility(TerrainMap terrain) {
		this.terrain = terrain;
		for (int level = 0; level < BLOCKS.length; level++) {
			int block = BLOCKS[level];
			columns[level] = (terrain.getCols() - 1 + block - 1) / block;
			rows[level] = (terrain.getRows() - 1 + block - 1) / block;
			maxima[level] = new double[columns[level] * rows[level]];
			for (int by = 0; by < rows[level]; by++) {
				for (int bx = 0; bx < columns[level]; bx++) {
					double maximum = -Double.MAX_VALUE;
					// Inclusive boundary vertices bound all bilinear samples inside this tile.
					for (int row = by * block; row <= Math.min((by + 1) * block, terrain.getRows() - 1); row++) {
						for (int col = bx * block; col <= Math.min((bx + 1) * block, terrain.getCols() - 1); col++)
							maximum = Math.max(maximum, terrain.elevationAtGrid(col, row));
					}
					maxima[level][by * columns[level] + bx] = maximum;
				}
			}
		}
	}

	boolean clear(double sx, double sy, double sz, double tx, double ty, double tz) {
		if (terrain.getMaxElevation() <= Math.min(sz, tz) + 0.02)
			return true;
		double dx = tx - sx, dy = ty - sy, dz = tz - sz;
		int samples = Math.max(2, (int) Math.ceil(Math.hypot(dx, dy) / (terrain.getCellSize() * 0.5)));
		int i = 1;
		while (i < samples) {
			double t = (double) i / samples;
			double px = sx + dx * t, py = sy + dy * t, pz = sz + dz * t;
			int skip = i;
			for (int level = 0; level < BLOCKS.length; level++) {
				double width = terrain.getCellSize() * BLOCKS[level];
				int bx = (int) Math.floor((px - terrain.getOriginX()) / width);
				int by = (int) Math.floor((py - terrain.getOriginY()) / width);
				if (bx < 0 || bx >= columns[level] || by < 0 || by >= rows[level])
					continue;
				double nextX = dx == 0 ? 1 : (terrain.getOriginX() + (bx + (dx > 0 ? 1 : 0)) * width - sx) / dx;
				double nextY = dy == 0 ? 1 : (terrain.getOriginY() + (by + (dy > 0 ? 1 : 0)) * width - sy) / dy;
				int last = Math.min(samples - 1, (int) Math.floor(Math.min(1, Math.min(nextX, nextY)) * samples));
				if (last >= i
						&& maxima[level][by * columns[level] + bx] <= Math.min(pz, sz + dz * last / samples) + 0.02) {
					skip = last + 1;
					break;
				}
			}
			if (skip > i) {
				i = skip;
				continue;
			}
			if (terrain.elevationAt(px, py) > pz + 0.02)
				return false;
			i++;
		}
		return true;
	}
}
