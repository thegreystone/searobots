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
import org.junit.jupiter.api.Test;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.ModeledTree;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeDetail;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeKind;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class IslandTreeLibraryTest {

	@Test
	void sharesArchetypesAndBatchesRealGeometryAtTwoDeterministicBudgets() {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		var library = CoastalVegetation.treeLibrary(assets);
		assertSame(library, CoastalVegetation.treeLibrary(assets),
				"World rebuilds reuse one bounded archetype library");
		var trees = new ArrayList<ModeledTree>();
		for (TreeKind kind : TreeKind.values()) {
			assertTrue(library.variantCount(kind) >= 2, "Repeated crowns need asymmetric variants");
			for (int variant = 0; variant < library.variantCount(kind); variant++) {
				int near = library.triangleCount(kind, TreeDetail.NEAR, variant);
				int middle = library.triangleCount(kind, TreeDetail.MIDDLE, variant);
				assertTrue(near > middle);
				assertTrue(middle > 100, "Middle distance keeps branch and leaf geometry, not whole-tree cards");
				assertTrue(near < 18000, "Game trees must be cheaper than full showcase crowns");
				trees.add(new ModeledTree(kind, variant, variant * 19f, 10f, kind.ordinal() * 35f, 7f, 15f,
						variant * .7f, .8f, .95f, 1.05f));
			}
		}
		Node near = library.batch(trees, TreeDetail.NEAR);
		Node middle = library.batch(trees, TreeDetail.MIDDLE);
		assertEquals(2, near.getQuantity());
		assertEquals(2, middle.getQuantity());
		assertEquals(signature(near), signature(library.batch(trees, TreeDetail.NEAR)));
		assertTrue(triangles(near) > triangles(middle));
		assertEquals(trees.size(), (int) near.getUserData("treeCount"));
		verifyDirectionsAndPlacement(near);
		verifyDirectionsAndPlacement(middle);
	}

	private static int triangles(Node batch) {
		return batch.getChildren().stream().mapToInt(child -> ((Geometry) child).getMesh().getTriangleCount()).sum();
	}

	private static long signature(Node batch) {
		long hash = 1;
		for (var child : batch.getChildren()) {
			FloatBuffer positions = ((Geometry) child).getMesh().getFloatBuffer(VertexBuffer.Type.Position);
			for (int index = 0; index < positions.limit(); index++)
				hash = hash * 31 + Float.floatToIntBits(positions.get(index));
		}
		return hash;
	}

	private static void verifyDirectionsAndPlacement(Node batch) {
		for (var child : batch.getChildren()) {
			var mesh = ((Geometry) child).getMesh();
			FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
			FloatBuffer normals = mesh.getFloatBuffer(VertexBuffer.Type.Normal);
			FloatBuffer tangents = mesh.getFloatBuffer(VertexBuffer.Type.Tangent);
			for (int vertex = 0; vertex < mesh.getVertexCount(); vertex++) {
				assertTrue(Float.isFinite(positions.get(vertex * 3)));
				assertTrue(Float.isFinite(positions.get(vertex * 3 + 2)));
				assertTrue(positions.get(vertex * 3 + 1) >= 9.5f && positions.get(vertex * 3 + 1) < 26f);
				float nx = normals.get(vertex * 3), ny = normals.get(vertex * 3 + 1), nz = normals.get(vertex * 3 + 2);
				assertEquals(1f, nx * nx + ny * ny + nz * nz, .0001f, "Anisotropic size transforms keep unit normals");
				if (tangents != null) {
					float tx = tangents.get(vertex * 4), ty = tangents.get(vertex * 4 + 1),
							tz = tangents.get(vertex * 4 + 2);
					assertEquals(1f, tx * tx + ty * ty + tz * tz, .0001f);
					assertEquals(0f, tx * nx + ty * ny + tz * nz, .0001f);
				}
			}
		}
	}
}
