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

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector2f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.List;

/** Small, soft ambient contact shadows under crowns, rendered in one forest-wide batch. */
final class TreeContactShadows {

	private static final float OPACITY = 0.10f;
	private static final float MAX_SURFACE_DEVIATION = 0.015f;

	private TreeContactShadows() {
	}

	static Geometry create(List<List<TreeScatter.TreeInstance>> perType, TerrainQuad terrain, AssetManager assets) {
		int maxCount = perType.stream().mapToInt(List::size).sum();
		float[] positions = new float[maxCount * 12], texcoords = new float[maxCount * 8];
		int[] indices = new int[maxCount * 6];
		Vector2f sample = new Vector2f();
		float[] heights = new float[4];
		int count = 0;
		for (var trees : perType) {
			for (var tree : trees) {
				float radius = Math.max(0.6f, Math.min(2.5f, tree.w() * 0.28f));
				boolean valid = true;
				for (int i = 0; i < 4; i++) {
					float x = tree.x() + ((i == 0 || i == 3) ? -radius : radius);
					float z = tree.z() + (i < 2 ? -radius : radius);
					heights[i] = surfaceHeight(terrain, sample.set(x, z));
					valid &= Float.isFinite(heights[i]);
				}
				// Check both triangles and their edges. Curvature must stay below the 2.5 cm lift,
				// otherwise a footprint could disappear into a ridge or float across a hollow.
				if (!valid || !fitsSurface(terrain, sample, tree, radius, heights))
					continue;
				int vertex = count * 4;
				for (int i = 0; i < 4; i++) {
					int p = (vertex + i) * 3, t = (vertex + i) * 2;
					positions[p] = tree.x() + ((i == 0 || i == 3) ? -radius : radius);
					positions[p + 1] = heights[i] + 0.025f;
					positions[p + 2] = tree.z() + (i < 2 ? -radius : radius);
					texcoords[t] = (i == 0 || i == 3) ? 0f : 1f;
					texcoords[t + 1] = i < 2 ? 0f : 1f;
				}
				int p = count * 6;
				indices[p] = vertex;
				indices[p + 1] = vertex + 2;
				indices[p + 2] = vertex + 1;
				indices[p + 3] = vertex;
				indices[p + 4] = vertex + 3;
				indices[p + 5] = vertex + 2;
				count++;
			}
		}
		if (count == 0)
			return null;
		Mesh mesh = new Mesh();
		mesh.setBuffer(VertexBuffer.Type.Position, 3,
				BufferUtils.createFloatBuffer(Arrays.copyOf(positions, count * 12)));
		mesh.setBuffer(VertexBuffer.Type.TexCoord, 2,
				BufferUtils.createFloatBuffer(Arrays.copyOf(texcoords, count * 8)));
		mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(Arrays.copyOf(indices, count * 6)));
		mesh.setStatic();
		mesh.updateBound();
		Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setTexture("ColorMap", softDisk());
		material.setColor("Color", new ColorRGBA(0f, 0f, 0f, OPACITY));
		RenderState state = material.getAdditionalRenderState();
		state.setBlendMode(RenderState.BlendMode.Alpha);
		state.setDepthWrite(false);
		state.setPolyOffset(-1f, -1f);
		Geometry geometry = new Geometry("tree_contact_shadows", mesh);
		geometry.setMaterial(material);
		geometry.setQueueBucket(RenderQueue.Bucket.Transparent);
		geometry.setShadowMode(RenderQueue.ShadowMode.Off);
		return geometry;
	}

	private static boolean fitsSurface(
		TerrainQuad terrain, Vector2f sample, TreeScatter.TreeInstance tree, float radius, float[] h) {
		float x = tree.x(), z = tree.z();
		return matchesSurface(terrain, sample, x, z, (h[0] + h[2]) * 0.5f)
				&& matchesSurface(terrain, sample, x + radius / 3, z - radius / 3, (h[0] + h[1] + h[2]) / 3)
				&& matchesSurface(terrain, sample, x - radius / 3, z + radius / 3, (h[0] + h[2] + h[3]) / 3)
				&& matchesSurface(terrain, sample, x, z - radius, (h[0] + h[1]) * 0.5f)
				&& matchesSurface(terrain, sample, x + radius, z, (h[1] + h[2]) * 0.5f)
				&& matchesSurface(terrain, sample, x, z + radius, (h[2] + h[3]) * 0.5f)
				&& matchesSurface(terrain, sample, x - radius, z, (h[3] + h[0]) * 0.5f);
	}

	private static boolean matchesSurface(TerrainQuad terrain, Vector2f sample, float x, float z, float expected) {
		float actual = surfaceHeight(terrain, sample.set(x, z));
		return Float.isFinite(actual) && Math.abs(actual - expected) <= MAX_SURFACE_DEVIATION;
	}

	/** getHeight consumes world X/Z and applies Y scale, but does not add world Y translation. */
	static float surfaceHeight(TerrainQuad terrain, Vector2f worldXZ) {
		return terrain.getHeight(worldXZ) + terrain.getWorldTranslation().y;
	}

	private static Texture softDisk() {
		int size = 32;
		ByteBuffer pixels = BufferUtils.createByteBuffer(size * size * 4);
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				float dx = (2f * x + 1f) / size - 1f, dy = (2f * y + 1f) / size - 1f;
				float strength = Math.max(0f, 1f - dx * dx - dy * dy);
				pixels.put((byte) 255).put((byte) 255).put((byte) 255)
						.put((byte) Math.round(strength * strength * 255f));
			}
		}
		pixels.flip();
		Texture texture = new Texture2D(new Image(Image.Format.RGBA8, size, size, pixels, ColorSpace.Linear));
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		texture.setWrap(Texture.WrapMode.EdgeClamp);
		return texture;
	}
}
