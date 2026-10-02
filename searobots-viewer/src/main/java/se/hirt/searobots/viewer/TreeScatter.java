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
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Scatters cutout trees across vegetated terrain in natural-looking clusters. Every tree is two
 * crossed vertical quads (the side cutout) plus one horizontal quad near the top of the crown (the
 * top-down cutout), so trees still read as crowns when the camera looks down at an island instead
 * of dissolving into crossed cards.
 * <p>
 * All quads of one tree type share a mesh and a lit material, so the whole forest is eight draw
 * calls regardless of tree count. The material is {@code Lighting.j3md} with alpha testing: trees
 * darken with the sun and at night like the terrain does. Vertex normals are "puffed" (up plus a
 * little outwards from the trunk) rather than the card's face normal, so a tree shades like a
 * rounded crown lit from above and the two crossed quads never disagree. Vertex colours carry a
 * per-tree tint, which breaks up the repetition of four textures, and darken the base of each tree
 * to fake the occlusion under the canopy.
 */
public final class TreeScatter {

	private static final int MAX_TREES = 500000;
	private static final float MIN_ELEVATION = 8f;
	private static final float MAX_SLOPE_DEG = 35f;
	private static final float BASE_SPACING = 4f;

	/**
	 * Tree archetypes. {@code widthRatio} must match the aspect ratio of the side cutout painted by
	 * {@code TextureGenerator}; {@code topHeight} is where the horizontal crown quad sits as a
	 * fraction of tree height.
	 */
	public enum Type {
		BROADLEAF("tree1", 8f, 15f, 0.75f, 0.74f, 0.9f),
		CONIFER("tree2", 10f, 18f, 0.45f, 0.62f, 0.8f),
		PALM("tree3", 10f, 20f, 0.62f, 0.86f, 0.9f),
		BUSH("tree4", 3f, 7f, 1.1f, 0.7f, 0.9f);

		final String texture;
		final float minHeight, maxHeight, widthRatio, topHeight, topScale;

		Type(String texture, float minHeight, float maxHeight, float widthRatio, float topHeight, float topScale) {
			this.texture = texture;
			this.minHeight = minHeight;
			this.maxHeight = maxHeight;
			this.widthRatio = widthRatio;
			this.topHeight = topHeight;
			this.topScale = topScale;
		}

		public String sidePath() {
			return "Textures/Terrain/custom/" + texture + ".png";
		}

		public String topPath() {
			return "Textures/Terrain/custom/" + texture + "_top.png";
		}
	}

	/** A tree to be placed: jME position, size, rotation about Y and colour tint. */
	public record TreeInstance(float x, float y, float z, float w, float h, float rot, float r, float g, float b) {
	}

	private TreeScatter() {
	}

	static Node create(TerrainMap terrain, TerrainQuad terrainQuad, AssetManager assetManager, long seed) {
		Node treeNode = new Node("trees");
		var rng = new Random(seed);
		long startTime = System.currentTimeMillis();
		Type[] types = Type.values();

		List<List<TreeInstance>> perType = new ArrayList<>();
		for (int i = 0; i < types.length; i++)
			perType.add(new ArrayList<>());

		double cellSize = terrain.getCellSize();
		double originX = terrain.getOriginX();
		double originY = terrain.getOriginY();

		int placed = 0;
		int gridCols = (int) (terrain.worldWidth() / BASE_SPACING);
		int gridRows = (int) (terrain.worldHeight() / BASE_SPACING);

		for (int gr = 0; gr < gridRows && placed < MAX_TREES; gr++) {
			for (int gc = 0; gc < gridCols && placed < MAX_TREES; gc++) {
				double wx = originX + (gc + rng.nextFloat()) * BASE_SPACING;
				double wy = originY + (gr + rng.nextFloat()) * BASE_SPACING;

				float elev = (float) terrain.elevationAt(wx, wy);
				if (elev < MIN_ELEVATION)
					continue;

				double e1 = terrain.elevationAt(wx + cellSize, wy);
				double e2 = terrain.elevationAt(wx, wy + cellSize);
				double dzdx = (e1 - elev) / cellSize;
				double dzdy = (e2 - elev) / cellSize;
				float slopeDeg = (float) Math.toDegrees(Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy)));
				if (slopeDeg > MAX_SLOPE_DEG)
					continue;

				double density = noiseDensity(wx, wy, seed);
				if (rng.nextFloat() > density)
					continue;

				// Elevation-based type selection
				Type type;
				if (elev < 20) {
					// Coastal zone: palms and bushes along the beach edge
					type = rng.nextFloat() < 0.65f ? Type.PALM : Type.BUSH;
				} else if (elev < 50) {
					float roll = rng.nextFloat();
					if (roll < 0.40f)
						type = Type.BROADLEAF;
					else if (roll < 0.75f)
						type = Type.CONIFER;
					else if (roll < 0.85f)
						type = Type.PALM;
					else
						type = Type.BUSH;
				} else {
					type = rng.nextFloat() < 0.7f ? (rng.nextBoolean() ? Type.BROADLEAF : Type.CONIFER) : Type.BUSH;
				}

				// Raycast for ground height
				float jmeX = (float) wx;
				float jmeZ = (float) -wy;
				var ray = new com.jme3.math.Ray(new Vector3f(jmeX, 1000, jmeZ), new Vector3f(0, -1, 0));
				var hits = new com.jme3.collision.CollisionResults();
				terrainQuad.collideWith(ray, hits);
				float groundY = hits.size() > 0 ? hits.getClosestCollision().getContactPoint().y : elev;

				perType.get(type.ordinal())
						.add(instance(type, rng, jmeX, groundY, jmeZ, 1f - Math.min(elev / 300f, 0.3f)));
				placed++;
			}
		}

		for (Type type : types) {
			var trees = perType.get(type.ordinal());
			if (trees.isEmpty())
				continue;
			treeNode.attachChild(geometry("trees_" + type.name(), buildCrossMesh(trees),
					createTreeMaterial(assetManager, type.sidePath())));
			treeNode.attachChild(geometry("trees_top_" + type.name(), buildTopMesh(trees, type),
					createTopMaterial(assetManager, type.topPath())));
		}

		long elapsed = System.currentTimeMillis() - startTime;
		System.out.printf("TreeScatter: placed %d trees in %dms (%d draw calls, direct mesh)%n", placed, elapsed,
				treeNode.getQuantity());
		return treeNode;
	}

	/**
	 * A random tree of the given type at a position; {@code heightScale} shrinks trees with
	 * altitude.
	 */
	public static TreeInstance instance(Type type, Random rng, float x, float y, float z, float heightScale) {
		float height = (type.minHeight + rng.nextFloat() * (type.maxHeight - type.minHeight)) * heightScale;
		float width = height * type.widthRatio * (0.88f + rng.nextFloat() * 0.24f);
		float rotation = rng.nextFloat() * FastMath.PI;
		// Tint: mostly brightness, with a little hue drift towards yellow or blue-green
		float bright = 0.78f + rng.nextFloat() * 0.34f;
		float warm = (rng.nextFloat() - 0.5f) * 0.2f;
		return new TreeInstance(x, y, z, width, height, rotation, bright * (1f + warm), bright,
				bright * (1f - warm * 0.8f));
	}

	private static Geometry geometry(String name, Mesh mesh, Material material) {
		Geometry geom = new Geometry(name, mesh);
		geom.setMaterial(material);
		geom.setQueueBucket(RenderQueue.Bucket.Opaque); // alpha-tested, so no sorting needed
		return geom;
	}

	/** Two crossed vertical quads per tree, baked into one mesh. */
	public static Mesh buildCrossMesh(List<TreeInstance> trees) {
		var b = new MeshBuffers(trees.size() * 2);
		for (var tree : trees) {
			for (int q = 0; q < 2; q++) {
				float ang = tree.rot + q * FastMath.HALF_PI;
				float cos = FastMath.cos(ang), sin = FastMath.sin(ang);
				float hw = tree.w / 2;
				float[] dx = {-hw, hw, hw, -hw}, dy = {0, 0, tree.h, tree.h};
				float[] u = {0, 1, 1, 0}, v = {0, 0, 1, 1};
				for (int i = 0; i < 4; i++) {
					float rx = dx[i] * cos, rz = dx[i] * sin;
					// Puffed normal: up, leaning outwards from the trunk
					float nx = rx / hw * 0.5f, nz = rz / hw * 0.5f;
					float nl = FastMath.sqrt(nx * nx + 1 + nz * nz);
					float ao = dy[i] > 0 ? 1f : 0.55f;
					b.vertex(tree.x + rx, tree.y + dy[i], tree.z + rz, nx / nl, 1 / nl, nz / nl, u[i], v[i],
							tree.r * ao, tree.g * ao, tree.b * ao);
				}
				b.quad();
			}
		}
		return b.build();
	}

	/**
	 * One low eight-sided dome per tree over the upper crown, textured with the top-down cutout.
	 * From above it reads as a full crown; from the side its sloped faces sit inside the silhouette
	 * of the crossed quads instead of cutting across them like a flat plate would. Wound so the
	 * face normals point up and out: the material culls back faces, so nothing renders when the
	 * camera is below the rim.
	 */
	public static Mesh buildTopMesh(List<TreeInstance> trees, Type type) {
		int sides = 8;
		var b = new MeshBuffers(trees.size() * (sides + 1), trees.size() * sides);
		for (var tree : trees) {
			float hw = tree.w / 2 * type.topScale;
			float rimY = tree.y + tree.h * (type.topHeight - 0.14f), apexY = tree.y + tree.h * (type.topHeight + 0.06f);
			float apexShade = 0.95f, rimShade = 0.72f; // darker towards the rim, like the shaded underside of a crown
			int apex = b.vertex(tree.x, apexY, tree.z, 0, 1, 0, 0.5f, 0.5f, tree.r * apexShade, tree.g * apexShade,
					tree.b * apexShade);
			int[] rim = new int[sides];
			for (int i = 0; i < sides; i++) {
				float a = tree.rot + i * FastMath.TWO_PI / sides;
				float cx = FastMath.cos(a), cz = FastMath.sin(a);
				float nl = FastMath.sqrt(0.6f * 0.6f + 1f);
				rim[i] = b.vertex(tree.x + cx * hw, rimY, tree.z + cz * hw, cx * 0.6f / nl, 1f / nl, cz * 0.6f / nl,
						0.5f + 0.5f * cx, 0.5f - 0.5f * cz, tree.r * rimShade, tree.g * rimShade, tree.b * rimShade);
			}
			for (int i = 0; i < sides; i++)
				b.tri(apex, rim[(i + 1) % sides], rim[i]);
		}
		return b.build();
	}

	/** Material for the crown quads: as {@link #createTreeMaterial} but only visible from above. */
	public static Material createTopMaterial(AssetManager am, String texturePath) {
		Material mat = createTreeMaterial(am, texturePath);
		mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Back);
		return mat;
	}

	/**
	 * Lit, alpha-tested cutout material: the texture supplies the colour, vertex colours the
	 * per-tree tint.
	 */
	public static Material createTreeMaterial(AssetManager am, String texturePath) {
		Material mat = new Material(am, "Common/MatDefs/Light/Lighting.j3md");
		Texture tex = am.loadTexture(texturePath);
		tex.setMinFilter(Texture.MinFilter.Trilinear);
		tex.setAnisotropicFilter(4);
		mat.setTexture("DiffuseMap", tex);
		mat.setBoolean("UseMaterialColors", true);
		mat.setColor("Diffuse", ColorRGBA.White);
		mat.setColor("Ambient", ColorRGBA.White);
		mat.setColor("Specular", ColorRGBA.Black);
		mat.setFloat("Shininess", 1f);
		mat.setBoolean("UseVertexColor", true);
		mat.setFloat("AlphaDiscardThreshold", 0.45f);
		mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
		return mat;
	}

	/** Accumulates triangles into position / normal / texcoord / colour / index buffers. */
	private static final class MeshBuffers {
		final float[] pos, norm, tc, col;
		final int[] idx;
		int vi, ii;

		MeshBuffers(int quads) {
			this(quads * 4, quads * 2);
		}

		MeshBuffers(int vertices, int triangles) {
			pos = new float[vertices * 3];
			norm = new float[vertices * 3];
			tc = new float[vertices * 2];
			col = new float[vertices * 4];
			idx = new int[triangles * 3];
		}

		/** Appends a vertex and returns its index. */
		int vertex(
			float x, float y, float z, float nx, float ny, float nz, float u, float v, float r, float g, float b) {
			int p = vi * 3, t = vi * 2, c = vi * 4;
			pos[p] = x;
			pos[p + 1] = y;
			pos[p + 2] = z;
			norm[p] = nx;
			norm[p + 1] = ny;
			norm[p + 2] = nz;
			tc[t] = u;
			tc[t + 1] = v;
			col[c] = r;
			col[c + 1] = g;
			col[c + 2] = b;
			col[c + 3] = 1f;
			return vi++;
		}

		void tri(int a, int b, int c) {
			idx[ii++] = a;
			idx[ii++] = b;
			idx[ii++] = c;
		}

		/** Closes the quad formed by the last four vertices. */
		void quad() {
			int base = vi - 4;
			tri(base, base + 1, base + 2);
			tri(base, base + 2, base + 3);
		}

		Mesh build() {
			Mesh mesh = new Mesh();
			mesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(pos));
			mesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(norm));
			mesh.setBuffer(VertexBuffer.Type.TexCoord, 2, BufferUtils.createFloatBuffer(tc));
			mesh.setBuffer(VertexBuffer.Type.Color, 4, BufferUtils.createFloatBuffer(col));
			mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(idx));
			mesh.updateBound();
			return mesh;
		}
	}

	private static double noiseDensity(double wx, double wy, long seed) {
		double n1 = Math.sin(wx * 0.007 + seed * 0.001) * Math.cos(wy * 0.008 + seed * 0.002);
		double n2 = Math.sin(wx * 0.025 + wy * 0.02 + seed * 0.003) * 0.4;
		double n3 = Math.sin(wx * 0.06 + wy * 0.05) * 0.15;
		double density = 0.8 + n1 * 0.18 + n2 * 0.10 + n3 * 0.08;
		return Math.max(0, Math.min(1, density));
	}
}
