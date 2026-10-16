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

import com.jme3.math.FastMath;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.terrain.geomipmap.TerrainQuad;
import org.junit.jupiter.api.Test;

import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class TreeScatterTest {

	@Test
	void distantAndModeledTreesKeepTheExistingSeededArchetypes() {
		var tree = new TreeScatter.TreeInstance(13f, 7f, -23f, 9f, 20f, .31f, .7f, .8f, 1.1f);
		int[] originalVariants = {-470384705, 76944575, -1427069885, 565198661};
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			assertEquals(originalVariants[type.ordinal()], TreeImpostors.variant(42L, tree, type),
					"Moving archetype selection must preserve existing worlds");
			assertEquals(1, TreeImpostors.variantIndex(42L, tree, type));
		}
	}

	@Test
	void modeledImpostorSidesPreserveTintAndUseMatchingFrontAndRearAtlasViews() {
		var tree = new TreeScatter.TreeInstance(13f, 7f, -23f, 9f, 20f, .31f, .7f, .8f, 1.1f);
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			Mesh mesh = TreeScatter.buildImpostorCrossMesh(List.of(tree), type, 42L);
			assertEquals(8, mesh.getVertexCount());
			assertEquals(4, mesh.getTriangleCount(), "Regional visibility masks retain 12 side indices per tree");
			FloatBuffer uv = mesh.getFloatBuffer(VertexBuffer.Type.TexCoord);
			FloatBuffer rear = mesh.getFloatBuffer(VertexBuffer.Type.TexCoord2);
			FloatBuffer colors = mesh.getFloatBuffer(VertexBuffer.Type.Color);
			FloatBuffer normals = mesh.getFloatBuffer(VertexBuffer.Type.Normal);
			FloatBuffer tangents = mesh.getFloatBuffer(VertexBuffer.Type.Tangent);
			FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
			for (int vertex = 0; vertex < 8; vertex++) {
				int view = vertex / 4;
				assertEquals(.75f, uv.get(vertex * 2) + rear.get(vertex * 2), .00001f,
						"Variant-one front and rear views mirror U within the same atlas column");
				assertEquals(.5f, uv.get(vertex * 2 + 1) - rear.get(vertex * 2 + 1), .00001f);
				assertTrue(uv.get(vertex * 2 + 1) >= (3f - view) / 4f);
				assertTrue(uv.get(vertex * 2 + 1) <= (4f - view) / 4f);
				assertEquals(tree.r(), colors.get(vertex * 4), .00001f, "Baked albedo must not be shaded twice");
				assertEquals(tree.g(), colors.get(vertex * 4 + 1), .00001f);
				assertEquals(tree.b(), colors.get(vertex * 4 + 2), .00001f);
				Vector3f normal = vector(normals, vertex);
				Vector3f tangent = new Vector3f(tangents.get(vertex * 4), tangents.get(vertex * 4 + 1),
						tangents.get(vertex * 4 + 2));
				assertEquals(1f, normal.length(), .00001f);
				assertEquals(1f, tangent.length(), .00001f);
				assertEquals(0f, tangent.dot(normal), .00001f);
				assertEquals(1f, normal.cross(tangent).y, .00001f, "The captured bitangent is upward on both planes");
				float height = vertex % 4 >= 2 ? tree.h() * TreeImpostors.SIDE_HEIGHT_SCALE : 0f;
				assertEquals(tree.y() + height, positions.get(vertex * 3 + 1), .00001f);
			}
		}
	}

	@Test
	void modeledImpostorCrownKeepsVariantYawTintAndRegionalIndexBudget() {
		var tree = new TreeScatter.TreeInstance(13f, 7f, -23f, 9f, 20f, .31f, .7f, .8f, 1.1f);
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			Mesh mesh = TreeScatter.buildImpostorTopMesh(List.of(tree), type, 42L);
			assertEquals(9, mesh.getVertexCount());
			assertEquals(8, mesh.getTriangleCount(), "Regional visibility masks retain 24 crown indices per tree");
			FloatBuffer uv = mesh.getFloatBuffer(VertexBuffer.Type.TexCoord);
			FloatBuffer colors = mesh.getFloatBuffer(VertexBuffer.Type.Color);
			FloatBuffer normals = mesh.getFloatBuffer(VertexBuffer.Type.Normal);
			FloatBuffer tangents = mesh.getFloatBuffer(VertexBuffer.Type.Tangent);
			assertEquals(.375f, uv.get(0), .00001f, "Crown uses the same variant-one atlas column");
			for (int vertex = 0; vertex < 9; vertex++) {
				assertEquals(Vector3f.UNIT_Y, vector(normals, vertex));
				assertEquals(FastMath.cos(tree.rot()), tangents.get(vertex * 4), .00001f);
				assertEquals(FastMath.sin(tree.rot()), tangents.get(vertex * 4 + 2), .00001f);
				assertEquals(tree.r(), colors.get(vertex * 4), .00001f);
				assertEquals(tree.g(), colors.get(vertex * 4 + 1), .00001f);
				assertEquals(tree.b(), colors.get(vertex * 4 + 2), .00001f);
				assertTrue(uv.get(vertex * 2) >= .25f && uv.get(vertex * 2) <= .5f);
			}
		}
	}

	@Test
	void crownTextureRotatesWithTreeRatherThanStayingLockedToWorld() {
		Mesh original = TreeScatter.buildTopMesh(List.of(tree(0f)), TreeScatter.Type.BROADLEAF);
		Mesh rotated = TreeScatter.buildTopMesh(List.of(tree(FastMath.HALF_PI)), TreeScatter.Type.BROADLEAF);
		FloatBuffer uv = original.getFloatBuffer(VertexBuffer.Type.TexCoord);
		FloatBuffer rotatedUv = rotated.getFloatBuffer(VertexBuffer.Type.TexCoord);
		for (int p = 0; p < uv.limit(); p++)
			assertEquals(uv.get(p), rotatedUv.get(p), 0.00001f);
		FloatBuffer positions = original.getFloatBuffer(VertexBuffer.Type.Position);
		FloatBuffer rotatedPositions = rotated.getFloatBuffer(VertexBuffer.Type.Position);
		// First rim vertex keeps its UV while its geometry turns a quarter revolution.
		assertTrue(positions.get(3) > 0f);
		assertEquals(0f, rotatedPositions.get(3), 0.00001f);
		assertEquals(positions.get(3), rotatedPositions.get(5), 0.00001f);
	}

	@Test
	void coniferCrownReachesTheTipAndStaysInsideItsTaperedSideSilhouette() {
		var tree = new TreeScatter.TreeInstance(13f, 7f, -23f, 9f, 20f, 0.31f, 1f, 1f, 1f);
		Mesh crown = TreeScatter.buildTopMesh(List.of(tree), TreeScatter.Type.CONIFER);
		assertEquals(9, crown.getVertexCount(), "A taller crown must keep the existing vertex budget");
		assertEquals(8, crown.getTriangleCount());
		FloatBuffer positions = crown.getFloatBuffer(VertexBuffer.Type.Position);
		Vector3f apex = vector(positions, 0);
		assertEquals(tree.x(), apex.x, 0.00001f);
		assertEquals(tree.z(), apex.z, 0.00001f);
		assertTrue(apex.y > tree.y() + tree.h() * 0.85f, "The cone must reach the upper needle crown");
		assertTrue(apex.y < tree.y() + tree.h(), "The cap must not extend above the side cutout");
		for (int i = 1; i < crown.getVertexCount(); i++) {
			Vector3f rim = vector(positions, i);
			float relativeHeight = (rim.y - tree.y()) / tree.h();
			float radius = (float) Math.hypot(rim.x - tree.x(), rim.z - tree.z());
			assertTrue(relativeHeight > 0.25f && relativeHeight < 0.45f,
					"The rim must sit in the wider lower needle crown");
			assertTrue(radius > tree.w() * 0.3f, "The cone must fill out the lower crown from above");
			// Allow a small margin for the irregular needle tips beyond the idealized side triangle.
			float sideSilhouetteRadius = tree.w() * 0.5f * (1f - relativeHeight);
			assertTrue(radius <= sideSilhouetteRadius + tree.w() * 0.02f,
					"The rim must fit the tapered side silhouette instead of forming a broad skirt");
		}
	}

	@Test
	void coniferNormalsFollowTheConeAtAnyTreeRotation() {
		for (float yaw : new float[] {0f, 0.37f, FastMath.HALF_PI}) {
			var tree = new TreeScatter.TreeInstance(13f, 7f, -23f, 9f, 20f, yaw, 1f, 1f, 1f);
			Mesh crown = TreeScatter.buildTopMesh(List.of(tree), TreeScatter.Type.CONIFER);
			FloatBuffer positions = crown.getFloatBuffer(VertexBuffer.Type.Position);
			FloatBuffer normals = crown.getFloatBuffer(VertexBuffer.Type.Normal);
			Vector3f apex = vector(positions, 0);
			for (int i = 0; i < crown.getVertexCount(); i++) {
				Vector3f normal = vector(normals, i);
				assertEquals(1f, normal.length(), 0.00001f);
				assertTrue(normal.y > 0f);
				if (i == 0)
					continue;
				Vector3f rim = vector(positions, i);
				Vector3f radial = new Vector3f(rim.x - tree.x(), 0f, rim.z - tree.z());
				Vector3f alongRim = new Vector3f(radial.z, 0f, -radial.x);
				assertEquals(0f, normal.dot(apex.subtract(rim)), 0.0001f,
						"Rim normal must be perpendicular to the cone generator");
				assertEquals(0f, normal.dot(alongRim), 0.0001f);
				assertTrue(normal.dot(radial) > 0f, "Rim normal must face out from the trunk");
			}
		}
	}

	@Test
	void crownTrianglesFaceUpAndOutForEveryTreeType() {
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			Mesh crown = TreeScatter.buildTopMesh(List.of(tree(0.37f)), type);
			Vector3f a = new Vector3f(), b = new Vector3f(), c = new Vector3f();
			for (int triangle = 0; triangle < crown.getTriangleCount(); triangle++) {
				crown.getTriangle(triangle, a, b, c);
				Vector3f normal = b.subtract(a).cross(c.subtract(a));
				assertTrue(normal.y > 0f, type + " must remain visible from above with back-face culling");
				Vector3f radial = new Vector3f((a.x + b.x + c.x) / 3f, 0f, (a.z + b.z + c.z) / 3f);
				assertTrue(normal.dot(radial) > 0f, type + " faces must point away from the trunk");
			}
		}
	}

	@Test
	void shadowHeightSamplesWorldCoordinatesAndIncludesVerticalTranslation() {
		float[] heights = new float[9 * 9];
		Arrays.fill(heights, 4f);
		TerrainQuad terrain = new TerrainQuad("translated-terrain", 5, 9, heights);
		terrain.setLocalTranslation(100f, 7f, -50f);
		terrain.setLocalScale(3f, 2f, 3f);
		terrain.updateGeometricState();
		assertEquals(15f, TreeContactShadows.surfaceHeight(terrain, new Vector2f(103f, -47f)), 0.00001f);
		assertTrue(Float.isNaN(TreeContactShadows.surfaceHeight(terrain, new Vector2f(0f, 0f))));
	}

	@Test
	void contactFootprintDoesNotBridgeHollowsOrClipIntoRidges() {
		for (float deviation : new float[] {-0.15f, 0.15f}) {
			float[] heights = new float[9 * 9];
			heights[4 * 9 + 4] = deviation;
			TerrainQuad terrain = new TerrainQuad("curved-ground", 5, 9, heights);
			terrain.updateGeometricState();
			// A 15 cm ridge or hollow cannot be covered by a footprint with only a 2.5 cm lift.
			// Rejection happens before material loading, so no renderer or asset manager is needed.
			assertNull(TreeContactShadows.create(List.of(List.of(tree(0))), terrain, null));
		}
	}

	private static TreeScatter.TreeInstance tree(float yaw) {
		return new TreeScatter.TreeInstance(0f, 0f, 0f, 10f, 15f, yaw, 1f, 1f, 1f);
	}

	private static Vector3f vector(FloatBuffer buffer, int vertex) {
		return new Vector3f(buffer.get(vertex * 3), buffer.get(vertex * 3 + 1), buffer.get(vertex * 3 + 2));
	}
}
