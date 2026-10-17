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
package se.hirt.searobots.viewer.shoreline;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.system.JmeSystem;
import jme3tools.optimize.GeometryBatchFactory;
import org.junit.jupiter.api.Test;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

import static org.junit.jupiter.api.Assertions.*;

class CoastalVegetationTest {

	@Test
	void batchesDetailedGeometryWithinTheShowcaseAndReproducesItFromTheSeed() {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		Node first = CoastalVegetation.create(assets, 8421L, (x, z) -> 2f);
		Map<String, Long> original = geometrySignature(first);
		int treeCount = first.getUserData("treeCount");
		assertTrue(treeCount >= 70 && treeCount <= 80,
				"The showcase needs a substantial woodland around its coastal edge");
		assertTrue((int) first.getUserData("grassCount") >= treeCount * 40,
				"Woodland floor needs visible grass pockets beneath its crowns");
		assertEquals(original, geometrySignature(CoastalVegetation.create(assets, 8421L, (x, z) -> 2f)));
		assertNotEquals(original, geometrySignature(CoastalVegetation.create(assets, 8422L, (x, z) -> 2f)));
	}

	@Test
	void skipsTerrainSamplesThatAreMissingOrUnderwater() {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		Node vegetation = CoastalVegetation.create(assets, 8421L, (x, z) -> x < 0f ? Float.NaN : -1f);
		assertEquals(0, vegetation.getQuantity());
		assertEquals(0, (int) vegetation.getUserData("treeCount"));
		assertEquals(0, (int) vegetation.getUserData("shrubCount"));
		assertEquals(0, (int) vegetation.getUserData("grassCount"));
	}

	private static Map<String, Long> geometrySignature(Node vegetation) {
		var geometries = new ArrayList<Geometry>();
		GeometryBatchFactory.gatherGeoms(vegetation, geometries);
		assertTrue(geometries.size() <= 36, "Material batches should stay local without a draw call for every plant");
		assertFalse(geometries.isEmpty());
		Map<String, Long> signatures = new TreeMap<>();
		int triangles = 0;
		for (Geometry geometry : geometries) {
			FloatBuffer positions = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
			FloatBuffer normals = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Normal);
			FloatBuffer tangents = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Tangent);
			long signature = 1L;
			for (int vertex = 0; vertex < geometry.getMesh().getVertexCount(); vertex++) {
				float x = positions.get(vertex * 3), y = positions.get(vertex * 3 + 1),
						z = positions.get(vertex * 3 + 2);
				assertTrue(Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z));
				assertTrue(Math.abs(x) <= 128f && Math.abs(z) <= 128f, "Plants must stay inside the terrain domain");
				assertTrue(y >= 1.85f && y <= 20f, "Plants must sit on the supplied terrain height");
				float nx = normals.get(vertex * 3), ny = normals.get(vertex * 3 + 1), nz = normals.get(vertex * 3 + 2);
				assertEquals(1f, nx * nx + ny * ny + nz * nz, 0.0001f, "PBR lighting needs finite unit normals");
				if (tangents != null) {
					float tx = tangents.get(vertex * 4), ty = tangents.get(vertex * 4 + 1),
							tz = tangents.get(vertex * 4 + 2);
					assertEquals(1f, tx * tx + ty * ty + tz * tz, 0.0001f,
							"Bark normal maps need finite unit tangents");
					assertEquals(0f, tx * nx + ty * ny + tz * nz, 0.0001f);
				}
				signature = signature * 31 + Float.floatToIntBits(x);
				signature = signature * 31 + Float.floatToIntBits(y);
				signature = signature * 31 + Float.floatToIntBits(z);
			}
			triangles += geometry.getMesh().getTriangleCount();
			String key = geometry.getParent().getName() + "/" + geometry.getMaterial().getName();
			assertNull(signatures.put(key, signature), "A spatial cell needs only one batch for each material");
		}
		assertTrue(triangles > 100_000, "The showcase needs individual foliage geometry, not whole-tree cards");
		assertTrue(triangles < 2_500_000, "Dense individual canopies must keep a bounded showcase geometry budget");
		return signatures;
	}
}
