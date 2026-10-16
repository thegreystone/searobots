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

import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.Random;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.util.BufferUtils;

/** Small original foliage meshes and direct material batches; all construction is CPU only. */
final class IslandSceneryMeshes {

	private IslandSceneryMeshes() {
	}

	static Mesh grass(long seed) {
		Random random = new Random(seed);
		Builder builder = new Builder(false);
		for (int blade = 0; blade < 12; blade++) {
			float angle = random.nextFloat() * (float) (Math.PI * 2), dx = (float) Math.cos(angle),
					dz = (float) Math.sin(angle);
			float rootX = dx * random.nextFloat() * .22f, rootZ = dz * random.nextFloat() * .22f;
			float height = .23f + random.nextFloat() * .47f, bend = .08f + random.nextFloat() * .25f;
			float width = .022f + random.nextFloat() * .028f;
			Vector3f normal = new Vector3f(-dx * .58f, .8f, -dz * .58f).normalizeLocal();
			int previousLeft = -1, previousRight = -1;
			for (int level = 0; level < 4; level++) {
				float t = level / 3f, half = width * (1 - t) * .5f;
				float x = rootX + dx * bend * t * t, z = rootZ + dz * bend * t * t;
				ColorRGBA color = new ColorRGBA(.055f + .10f * t, .15f + .19f * t, .025f + .045f * t, 1);
				int left = builder.vertex(x - dz * half, height * t, z + dx * half, normal, 0, t, color);
				int right = builder.vertex(x + dz * half, height * t, z - dx * half, normal, 1, t, color);
				if (level > 0) {
					builder.triangle(previousLeft, previousRight, left);
					if (level < 3)
						builder.triangle(previousRight, right, left);
				}
				previousLeft = left;
				previousRight = right;
			}
		}
		return builder.mesh();
	}

	static Mesh shrub(long seed) {
		Random random = new Random(seed);
		Builder builder = new Builder(false);
		for (int spray = 0; spray < 5; spray++) {
			float angle = random.nextFloat() * (float) (Math.PI * 2), dx = (float) Math.cos(angle),
					dz = (float) Math.sin(angle), height = .35f + random.nextFloat() * .42f;
			Vector3f end = new Vector3f(dx * .32f, height, dz * .32f);
			Vector3f side = new Vector3f(-dz * .018f, 0, dx * .018f);
			ColorRGBA wood = new ColorRGBA(.14f, .085f, .035f, 1);
			int a = builder.vertex(-side.x, 0, -side.z, Vector3f.UNIT_Y, 0, 0, wood);
			int b = builder.vertex(side.x, 0, side.z, Vector3f.UNIT_Y, 1, 0, wood);
			int c = builder.vertex(end.x, end.y, end.z, Vector3f.UNIT_Y, .5f, 1, wood);
			builder.triangle(a, b, c);
			for (int leaf = 0; leaf < 9; leaf++) {
				float fraction = .28f + random.nextFloat() * .72f;
				Vector3f center = end.mult(fraction).addLocal((random.nextFloat() - .5f) * .38f,
						(random.nextFloat() - .5f) * .18f, (random.nextFloat() - .5f) * .38f);
				float yaw = random.nextFloat() * (float) (Math.PI * 2), length = .12f + random.nextFloat() * .14f;
				Vector3f axis = new Vector3f((float) Math.cos(yaw), .30f, (float) Math.sin(yaw)).normalizeLocal();
				Vector3f across = new Vector3f(-axis.z, 0, axis.x).multLocal(length * .33f);
				Vector3f normal = new Vector3f(-axis.x * .3f, 1, -axis.z * .3f).normalizeLocal();
				ColorRGBA color = new ColorRGBA(.065f + random.nextFloat() * .04f, .19f + random.nextFloat() * .11f,
						.025f + random.nextFloat() * .035f, 1);
				int root = builder.vertex(center.subtract(axis.mult(length * .4f)), normal, .5f, 0, color);
				int left = builder.vertex(center.subtract(across), normal, 0, .45f, color);
				int tip = builder.vertex(center.add(axis.mult(length)), normal, .5f, 1, color);
				int right = builder.vertex(center.add(across), normal, 1, .45f, color);
				builder.triangle(root, left, tip);
				builder.triangle(root, tip, right);
			}
		}
		return builder.mesh();
	}

	static final class Builder {
		private float[] positions = new float[768], normals = new float[768], colors = new float[1024],
				uv = new float[512], tangents;
		private int[] indices = new int[768];
		private int vertices, indexCount;

		Builder(boolean tangent) {
			if (tangent)
				tangents = new float[1024];
		}

		int vertex(Vector3f point, Vector3f normal, float u, float v, ColorRGBA color) {
			return vertex(point.x, point.y, point.z, normal, u, v, color);
		}

		int vertex(float x, float y, float z, Vector3f normal, float u, float v, ColorRGBA color) {
			ensureVertexCapacity();
			int p = vertices * 3, c = vertices * 4, t = vertices * 2;
			positions[p] = x;
			positions[p + 1] = y;
			positions[p + 2] = z;
			normals[p] = normal.x;
			normals[p + 1] = normal.y;
			normals[p + 2] = normal.z;
			colors[c] = color.r;
			colors[c + 1] = color.g;
			colors[c + 2] = color.b;
			colors[c + 3] = color.a;
			uv[t] = u;
			uv[t + 1] = v;
			return vertices++;
		}

		void triangle(int a, int b, int c) {
			if (indexCount + 3 > indices.length)
				indices = Arrays.copyOf(indices, indices.length * 2);
			indices[indexCount++] = a;
			indices[indexCount++] = b;
			indices[indexCount++] = c;
		}

		void append(
			Mesh mesh, IslandScenerySurface terrain, float x, float y, float z, float sx, float sy, float sz, float yaw,
			float tint, boolean conform) {
			FloatBuffer p = mesh.getFloatBuffer(VertexBuffer.Type.Position),
					n = mesh.getFloatBuffer(VertexBuffer.Type.Normal),
					texture = mesh.getFloatBuffer(VertexBuffer.Type.TexCoord),
					c = mesh.getFloatBuffer(VertexBuffer.Type.Color),
					t = mesh.getFloatBuffer(VertexBuffer.Type.Tangent);
			float cos = (float) Math.cos(yaw), sin = (float) Math.sin(yaw);
			int base = vertices;
			Vector3f normal = new Vector3f(), tangent = new Vector3f();
			ColorRGBA color = new ColorRGBA();
			for (int vertex = 0; vertex < mesh.getVertexCount(); vertex++) {
				float lx = p.get(vertex * 3) * sx, lz = p.get(vertex * 3 + 2) * sz;
				float wx = x + cos * lx + sin * lz, wz = z - sin * lx + cos * lz;
				float ground = conform ? terrain.height(wx, wz) : y;
				if (!Float.isFinite(ground))
					ground = y;
				normal.set(n.get(vertex * 3) / sx, n.get(vertex * 3 + 1) / sy, n.get(vertex * 3 + 2) / sz);
				rotate(normal, cos, sin).normalizeLocal();
				color.set(c == null ? 1 : c.get(vertex * 4), c == null ? 1 : c.get(vertex * 4 + 1),
						c == null ? 1 : c.get(vertex * 4 + 2), 1).multLocal(tint);
				color.a = 1;
				int added = vertex(wx, ground + p.get(vertex * 3 + 1) * sy, wz, normal, texture.get(vertex * 2),
						texture.get(vertex * 2 + 1), color);
				if (tangents != null) {
					tangent.set(t.get(vertex * 4) * sx, t.get(vertex * 4 + 1) * sy, t.get(vertex * 4 + 2) * sz);
					rotate(tangent, cos, sin);
					tangent.subtractLocal(normal.mult(tangent.dot(normal))).normalizeLocal();
					tangents[added * 4] = tangent.x;
					tangents[added * 4 + 1] = tangent.y;
					tangents[added * 4 + 2] = tangent.z;
					tangents[added * 4 + 3] = t.get(vertex * 4 + 3);
				}
			}
			IndexBuffer elements = mesh.getIndicesAsList();
			for (int index = 0; index < elements.size(); index += 3)
				triangle(base + elements.get(index), base + elements.get(index + 1), base + elements.get(index + 2));
		}

		private static Vector3f rotate(Vector3f vector, float cos, float sin) {
			float x = vector.x;
			return vector.set(cos * x + sin * vector.z, vector.y, -sin * x + cos * vector.z);
		}

		private void ensureVertexCapacity() {
			if ((vertices + 1) * 3 <= positions.length)
				return;
			positions = Arrays.copyOf(positions, positions.length * 2);
			normals = Arrays.copyOf(normals, normals.length * 2);
			colors = Arrays.copyOf(colors, colors.length * 2);
			uv = Arrays.copyOf(uv, uv.length * 2);
			if (tangents != null)
				tangents = Arrays.copyOf(tangents, tangents.length * 2);
		}

		Mesh mesh() {
			Mesh mesh = new Mesh();
			mesh.setBuffer(VertexBuffer.Type.Position, 3,
					BufferUtils.createFloatBuffer(Arrays.copyOf(positions, vertices * 3)));
			mesh.setBuffer(VertexBuffer.Type.Normal, 3,
					BufferUtils.createFloatBuffer(Arrays.copyOf(normals, vertices * 3)));
			mesh.setBuffer(VertexBuffer.Type.TexCoord, 2,
					BufferUtils.createFloatBuffer(Arrays.copyOf(uv, vertices * 2)));
			mesh.setBuffer(VertexBuffer.Type.Color, 4,
					BufferUtils.createFloatBuffer(Arrays.copyOf(colors, vertices * 4)));
			if (tangents != null)
				mesh.setBuffer(VertexBuffer.Type.Tangent, 4,
						BufferUtils.createFloatBuffer(Arrays.copyOf(tangents, vertices * 4)));
			mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(Arrays.copyOf(indices, indexCount)));
			mesh.updateBound();
			mesh.setStatic();
			return mesh;
		}

		boolean isEmpty() {
			return indexCount == 0;
		}
	}
}
