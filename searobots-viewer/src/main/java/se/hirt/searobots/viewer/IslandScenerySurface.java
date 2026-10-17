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

import com.jme3.math.Vector3f;
import com.jme3.terrain.geomipmap.TerrainQuad;

/** Immutable full-detail terrain snapshot for background scenery construction. */
final class IslandScenerySurface {

	private final float[] heights;
	private final int size, patchWidth;
	private final Vector3f origin, scale;

	IslandScenerySurface(TerrainQuad terrain) {
		// World transform getters refresh ancestors without prematurely clearing light-list flags.
		heights = terrain.getHeightMap();
		size = terrain.getTotalSize();
		patchWidth = terrain.getPatchSize() - 1;
		origin = terrain.getWorldTranslation().clone();
		scale = terrain.getWorldScale().clone();
	}

	float height(float x, float z) {
		float gx = (x - origin.x) / scale.x + (size - 1) * .5f;
		float gz = (z - origin.z) / scale.z + (size - 1) * .5f;
		int col = (int) Math.floor(gx), row = (int) Math.floor(gz);
		if (col < 0 || row < 0 || col >= size - 1 || row >= size - 1)
			return Float.NaN;
		float fx = gx - col, fz = gz - row;
		int index = row * size + col;
		float a = heights[index], b = heights[index + 1], c = heights[index + size], d = heights[index + size + 1];
		float local;
		// jME's two patch corners use the opposite diagonal from the other grid squares.
		int px = col % patchWidth, pz = row % patchWidth;
		if (px == 0 && pz == 0 || px == patchWidth - 1 && pz == patchWidth - 1) {
			local = fx < fz ? a + fx * (d - c) + fz * (c - a) : a + fx * (b - a) + fz * (d - b);
		} else {
			local = fx < 1 - fz ? c + fx * (b - a) + (1 - fz) * (a - c) : c + fx * (d - c) + (1 - fz) * (b - d);
		}
		return local * scale.y + origin.y;
	}

	float slope(float x, float z) {
		float dx = (height(x + 1, z) - height(x - 1, z)) * .5f;
		float dz = (height(x, z + 1) - height(x, z - 1)) * .5f;
		return (float) Math.hypot(dx, dz);
	}
}
