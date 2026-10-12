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
}
