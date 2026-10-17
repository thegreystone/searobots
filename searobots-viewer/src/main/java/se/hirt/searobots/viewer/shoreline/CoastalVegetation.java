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

import com.jme3.asset.AssetManager;
import com.jme3.asset.AssetKey;
import com.jme3.bounding.BoundingBox;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.mikktspace.MikktspaceTangentGenerator;
import jme3tools.optimize.GeometryBatchFactory;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.BiFunction;

/**
 * Original procedural plants for the small coastal showcase. Trunks and forked branches are tapered
 * tubes; foliage consists of individual folded leaves, needles and palm leaflets. No whole-tree
 * billboard or third-party vegetation asset is used. Seven reusable tree templates are transformed
 * into instances and then batched by material in 64-metre spatial cells.
 */
public final class CoastalVegetation {

	private static final int TREE_COUNT = 76;
	private static final float TREE_SPACING = 8.5f;
	private static final ColorRGBA BARK = new ColorRGBA(0.94f, 0.92f, 0.88f, 1f);
	private static final ColorRGBA BROADLEAF = new ColorRGBA(0.18f, 0.34f, 0.07f, 1f);
	private static final ColorRGBA PINE = new ColorRGBA(0.095f, 0.23f, 0.11f, 1f);
	private static final ColorRGBA PALM = new ColorRGBA(0.22f, 0.37f, 0.075f, 1f);
	private static final ColorRGBA GRASS = new ColorRGBA(0.20f, 0.27f, 0.065f, 1f);

	private CoastalVegetation() {
	}

	/**
	 * Creates trees, shrubs and grass inside the showcase's -128..128-metre X/Z domain.
	 * {@code heightAt} supplies the rendered terrain height in world metres at an X/Z point. All
	 * plants are deterministic for the same seed and terrain. The returned node includes
	 * {@code treeCount}, {@code shrubCount} and {@code grassCount} integer user data.
	 */
	public static Node create(AssetManager assets, long seed, BiFunction<Float, Float, Float> heightAt) {
		Material bark = material(assets, "coastal bark", 0.97f, false);
		Material leaves = material(assets, "coastal foliage", 0.76f, true);
		Material grass = material(assets, "coastal groundcover", 0.91f, true);
		BarkMaps barkMaps = barkMaps();
		bark.setTexture("BaseColorMap", barkMaps.color);
		bark.setTexture("RoughnessMap", barkMaps.roughness);
		bark.setTexture("NormalMap", barkMaps.normal);
		bark.setFloat("NormalType", 1f);
		bark.setFloat("NormalScale", 0.85f);
		bark.setFloat("Roughness", 1f);
		leaves.setTexture("BaseColorMap", leafVeins());
		Random templateRandom = new Random(seed ^ 0x5EEDC0A57L);
		Template[] broadleaf = {broadleaf(templateRandom), broadleaf(templateRandom), broadleaf(templateRandom)};
		Template[] pines = {pine(templateRandom), pine(templateRandom)};
		Template[] palms = {palm(templateRandom), palm(templateRandom)};
		Template[] shrubs = {shrub(templateRandom), shrub(templateRandom)};
		Mesh[] grasses = {grass(templateRandom), grass(templateRandom), grass(templateRandom)};

		Node vegetation = new Node("detailed coastal vegetation");
		vegetation.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
		Map<Integer, Node> cells = new LinkedHashMap<>();
		List<Vector3f> treePositions = new ArrayList<>();
		Random random = new Random(seed);
		for (int attempt = 0; attempt < 4000 && treePositions.size() < TREE_COUNT; attempt++) {
			boolean coastal = treePositions.size() < 12;
			float x, z;
			if (coastal) {
				x = between(random, -104f, 104f);
				z = between(random, 17f, 38f);
			} else {
				// Overlapping groves form a coherent inland woodland, leaving irregular clearings.
				int grove = random.nextInt(4);
				x = -74f + grove * 49f + (float) random.nextGaussian() * 19f;
				z = 66f + (grove % 2) * 21f + (float) random.nextGaussian() * 14f;
				if (x < -109f || x > 109f || z < 40f || z > 106f)
					continue;
			}
			Float y = heightAt.apply(x, z);
			if (y == null || !Float.isFinite(y) || y < 0.3f
					|| tooClose(treePositions, x, z, coastal ? 11f : TREE_SPACING))
				continue;
			Template tree;
			float selection = random.nextFloat();
			if (coastal && selection < 0.70f)
				tree = palms[random.nextInt(palms.length)];
			else if (selection > 0.86f)
				tree = pines[random.nextInt(pines.length)];
			else
				tree = broadleaf[random.nextInt(broadleaf.length)];
			float scale = between(random, 0.82f, 1.15f), yaw = random.nextFloat() * FastMath.TWO_PI;
			Node cell = cell(cells, x, z);
			instance(cell, "tree trunk", tree.wood, bark, x, y, z, scale, yaw);
			instance(cell, "individual tree leaves", tree.foliage, leaves, x, y, z, scale, yaw);
			treePositions.add(new Vector3f(x, y, z));
		}

		int shrubCount = 0;
		for (int attempt = 0; attempt < 380 && shrubCount < 110; attempt++) {
			float x = between(random, -118f, 118f), z = between(random, -5f, 117f);
			if (z < coastline(x) + 17f)
				continue;
			Float y = heightAt.apply(x, z);
			if (y == null || !Float.isFinite(y) || y < 0.25f)
				continue;
			Template shrub = shrubs[random.nextInt(shrubs.length)];
			float scale = between(random, 0.7f, 1.3f), yaw = random.nextFloat() * FastMath.TWO_PI;
			Node cell = cell(cells, x, z);
			instance(cell, "shrub branches", shrub.wood, bark, x, y, z, scale, yaw);
			instance(cell, "shrub leaves", shrub.foliage, leaves, x, y, z, scale, yaw);
			shrubCount++;
		}
		for (Vector3f tree : treePositions) {
			for (int shrubIndex = 0; shrubIndex < 2; shrubIndex++) {
				float angle = random.nextFloat() * FastMath.TWO_PI, radius = between(random, 2.0f, 5.0f);
				float x = tree.x + FastMath.cos(angle) * radius, z = tree.z + FastMath.sin(angle) * radius;
				Float y = heightAt.apply(x, z);
				if (y == null || !Float.isFinite(y) || y < 0.25f)
					continue;
				Template shrub = shrubs[random.nextInt(shrubs.length)];
				float scale = between(random, 0.85f, 1.35f), yaw = random.nextFloat() * FastMath.TWO_PI;
				Node cell = cell(cells, x, z);
				instance(cell, "forest understory branches", shrub.wood, bark, x, y, z, scale, yaw);
				instance(cell, "forest understory leaves", shrub.foliage, leaves, x, y, z, scale, yaw);
				shrubCount++;
			}
		}

		int grassCount = 0;
		for (int attempt = 0; attempt < 1600 && grassCount < 820; attempt++) {
			float x = between(random, -120f, 120f), z = between(random, -12f, 118f);
			if (z < coastline(x) + 12f)
				continue;
			// Irregular pockets leave visible soil between small grass clumps.
			float density = 0.62f + 0.28f * FastMath.sin(x * 0.09f) * FastMath.cos(z * 0.13f);
			if (random.nextFloat() > density)
				continue;
			Float y = heightAt.apply(x, z);
			if (y == null || !Float.isFinite(y) || y < 0.15f)
				continue;
			instance(cell(cells, x, z), "grass blades", grasses[random.nextInt(grasses.length)], grass, x, y, z,
					between(random, 0.65f, 1.25f), random.nextFloat() * FastMath.TWO_PI);
			grassCount++;
		}
		// Dense, irregular pockets beneath crowns make the woodland read as a layered habitat.
		for (Vector3f tree : treePositions) {
			for (int clump = 0; clump < 44; clump++) {
				float angle = random.nextFloat() * FastMath.TWO_PI;
				float radius = 0.65f + 5.4f * FastMath.sqrt(random.nextFloat());
				float x = tree.x + FastMath.cos(angle) * radius, z = tree.z + FastMath.sin(angle) * radius;
				Float y = heightAt.apply(x, z);
				if (y == null || !Float.isFinite(y) || y < 0.15f)
					continue;
				instance(cell(cells, x, z), "woodland grass pocket", grasses[random.nextInt(grasses.length)], grass, x,
						y, z, between(random, 0.85f, 1.5f), random.nextFloat() * FastMath.TWO_PI);
				grassCount++;
			}
		}

		for (Node cell : cells.values()) {
			cell.updateGeometricState();
			GeometryBatchFactory.optimize(cell);
			for (var spatial : cell.getChildren()) {
				if (spatial instanceof Geometry geometry) {
					normalizeDirections(geometry.getMesh(), VertexBuffer.Type.Normal, 3);
					normalizeDirections(geometry.getMesh(), VertexBuffer.Type.Tangent, 4);
				}
			}
			cell.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
			vegetation.attachChild(cell);
		}
		vegetation.setUserData("treeCount", treePositions.size());
		vegetation.setUserData("shrubCount", shrubCount);
		vegetation.setUserData("grassCount", grassCount);
		vegetation.updateGeometricState();
		return vegetation;
	}

	/** Tree kinds shared by the showcase and the game forest's distance-based detail. */
	public enum TreeKind {
		BROADLEAF, CONIFER, PALM, BUSH
	}

	/** Fully modeled crowns at two geometry budgets; neither tier contains whole-tree cards. */
	public enum TreeDetail {
		NEAR, MIDDLE
	}

	/** A stable tree placement supplied by the game, independently of the active detail tier. */
	public record ModeledTree(TreeKind kind, int variant, float x, float y, float z, float width, float height,
			float yaw, float red, float green, float blue) {
	}

	/**
	 * Reuses a small immutable archetype library per asset manager. World seeds select variants and
	 * placements, so rebuilding worlds never accumulates another library of native meshes.
	 */
	public static TreeLibrary treeLibrary(AssetManager assets) {
		AssetKey<TreeLibrary> key = new AssetKey<>("SeaRobots/IslandTreeLibrary-v1");
		synchronized (assets) {
			TreeLibrary library = assets.getFromCache(key);
			if (library == null) {
				library = new TreeLibrary(assets);
				assets.addToCache(key, library);
			}
			return library;
		}
	}

	/** Immutable archetypes with forked wood, asymmetric crowns and individually folded foliage. */
	public static final class TreeLibrary {
		private final Material bark, leaves;
		private final Archetype[][][] trees = new Archetype[TreeKind.values().length][2][];

		private TreeLibrary(AssetManager assets) {
			bark = material(assets, "island modeled bark", 1f, false);
			leaves = material(assets, "island modeled foliage", 0.76f, true);
			BarkMaps maps = barkMaps();
			bark.setTexture("BaseColorMap", maps.color);
			bark.setTexture("RoughnessMap", maps.roughness);
			bark.setTexture("NormalMap", maps.normal);
			bark.setFloat("NormalType", 1f);
			bark.setFloat("NormalScale", 0.85f);
			leaves.setTexture("BaseColorMap", leafVeins());
			for (TreeKind kind : TreeKind.values()) {
				int variants = kind == TreeKind.BROADLEAF ? 3 : 2;
				for (TreeDetail detail : TreeDetail.values())
					trees[kind.ordinal()][detail.ordinal()] = new Archetype[variants];
				for (int variant = 0; variant < variants; variant++) {
					long seed = 0x75a95f07382L + kind.ordinal() * 104729L + variant * 1699L;
					Template near = compactTree(kind, new Random(seed), TreeDetail.NEAR);
					Template middle = compactTree(kind, new Random(seed), TreeDetail.MIDDLE);
					// A common scale basis keeps the trunk and branches fixed when changing detail.
					BoundingBox bounds = (BoundingBox) near.wood.getBound().clone();
					bounds.mergeLocal(near.foliage.getBound());
					float width = Math.max(bounds.getXExtent(), bounds.getZExtent()) * 2f;
					float height = bounds.getCenter().y + bounds.getYExtent();
					trees[kind.ordinal()][TreeDetail.NEAR.ordinal()][variant] = new Archetype(near, width, height);
					trees[kind.ordinal()][TreeDetail.MIDDLE.ordinal()][variant] = new Archetype(middle, width, height);
				}
			}
		}

		public int variantCount(TreeKind kind) {
			return trees[kind.ordinal()][0].length;
		}

		public int triangleCount(TreeKind kind, TreeDetail detail, int variant) {
			Archetype tree = archetype(kind, detail, variant);
			return tree.meshes.wood.getTriangleCount() + tree.meshes.foliage.getTriangleCount();
		}

		/**
		 * Bakes only the requested local trees into two material batches, retaining per-tree tint.
		 */
		public Node batch(List<ModeledTree> instances, TreeDetail detail) {
			int woodVertices = 0, woodIndices = 0, leafVertices = 0, leafIndices = 0;
			for (ModeledTree instance : instances) {
				Template tree = archetype(instance.kind, detail, instance.variant).meshes;
				woodVertices += tree.wood.getVertexCount();
				woodIndices += tree.wood.getTriangleCount() * 3;
				leafVertices += tree.foliage.getVertexCount();
				leafIndices += tree.foliage.getTriangleCount() * 3;
			}
			Builder wood = new Builder(woodVertices, woodIndices), foliage = new Builder(leafVertices, leafIndices);
			FloatBuffer tangents = BufferUtils.createFloatBuffer(woodVertices * 4);
			for (ModeledTree instance : instances) {
				checkCancelled();
				Archetype tree = archetype(instance.kind, detail, instance.variant);
				append(wood, tree.meshes.wood, instance, tree.width, tree.height, tangents);
				append(foliage, tree.meshes.foliage, instance, tree.width, tree.height, null);
			}
			Node batch = new Node("modeled forest " + detail.name().toLowerCase());
			if (instances.isEmpty())
				return batch;
			Mesh woodMesh = wood.mesh();
			tangents.flip();
			woodMesh.setBuffer(VertexBuffer.Type.Tangent, 4, tangents);
			Geometry trunk = new Geometry("forked trunks and branches", woodMesh);
			trunk.setMaterial(bark);
			Geometry crown = new Geometry("individual modeled foliage", foliage.mesh());
			crown.setMaterial(leaves);
			batch.attachChild(trunk);
			batch.attachChild(crown);
			batch.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
			batch.setUserData("treeCount", instances.size());
			return batch;
		}

		private Archetype archetype(TreeKind kind, TreeDetail detail, int variant) {
			Archetype[] choices = trees[kind.ordinal()][detail.ordinal()];
			return choices[Math.floorMod(variant, choices.length)];
		}
	}

	private record Archetype(Template meshes, float width, float height) {
	}

	private static Template compactTree(TreeKind kind, Random random, TreeDetail detail) {
		boolean near = detail == TreeDetail.NEAR;
		int sides = near ? 7 : 4;
		return switch (kind) {
		case BROADLEAF -> broadleaf(random, near ? .35f : .12f, near ? 1.4f : 2.3f, sides, near);
		case CONIFER -> pine(random, near ? .25f : .08f, near ? 1.6f : 2.7f, sides, near);
		case PALM -> palm(random, near ? .8f : .30f, near ? 1.1f : 1.8f, sides, near);
		case BUSH -> shrub(random, near ? .45f : .14f, near ? 1.3f : 1.9f, sides, near);
		};
	}

	private static void append(
		Builder builder, Mesh mesh, ModeledTree tree, float width, float height, FloatBuffer transformedTangents) {
		FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
		FloatBuffer normals = mesh.getFloatBuffer(VertexBuffer.Type.Normal);
		FloatBuffer uv = mesh.getFloatBuffer(VertexBuffer.Type.TexCoord);
		FloatBuffer colors = mesh.getFloatBuffer(VertexBuffer.Type.Color);
		FloatBuffer tangents = transformedTangents == null ? null : mesh.getFloatBuffer(VertexBuffer.Type.Tangent);
		float horizontal = tree.width / width, vertical = tree.height / height;
		float cosine = FastMath.cos(tree.yaw), sine = FastMath.sin(tree.yaw);
		int base = builder.vertexCount;
		for (int vertex = 0; vertex < mesh.getVertexCount(); vertex++) {
			float x = positions.get(vertex * 3) * horizontal, y = positions.get(vertex * 3 + 1) * vertical,
					z = positions.get(vertex * 3 + 2) * horizontal;
			float nx = normals.get(vertex * 3) / horizontal, ny = normals.get(vertex * 3 + 1) / vertical,
					nz = normals.get(vertex * 3 + 2) / horizontal;
			float length = FastMath.sqrt(nx * nx + ny * ny + nz * nz);
			if (transformedTangents != null) {
				// Tangents transform with positions; normals use the inverse transpose. Rebuild
				// orthogonality after anisotropic height/width scaling without regenerating Mikk.
				float tx = tangents.get(vertex * 4) * horizontal, ty = tangents.get(vertex * 4 + 1) * vertical,
						tz = tangents.get(vertex * 4 + 2) * horizontal;
				float projection = (tx * nx + ty * ny + tz * nz) / (length * length);
				tx -= nx * projection;
				ty -= ny * projection;
				tz -= nz * projection;
				float tangentLength = FastMath.sqrt(tx * tx + ty * ty + tz * tz);
				transformedTangents.put((tx * cosine - tz * sine) / tangentLength).put(ty / tangentLength)
						.put((tx * sine + tz * cosine) / tangentLength).put(tangents.get(vertex * 4 + 3));
			}
			builder.vertex(tree.x + x * cosine - z * sine, tree.y - .015f + y, tree.z + x * sine + z * cosine,
					(nx * cosine - nz * sine) / length, ny / length, (nx * sine + nz * cosine) / length,
					uv.get(vertex * 2), uv.get(vertex * 2 + 1), colors.get(vertex * 4) * tree.red,
					colors.get(vertex * 4 + 1) * tree.green, colors.get(vertex * 4 + 2) * tree.blue, 1f);
		}
		IndexBuffer indices = mesh.getIndicesAsList();
		for (int index = 0; index < indices.size(); index += 3)
			builder.triangle(base + indices.get(index), base + indices.get(index + 1), base + indices.get(index + 2));
	}

	private static float coastline(float x) {
		return -25f + 10f * FastMath.sin(x * 0.035f) + 5f * FastMath.sin(x * 0.075f);
	}

	private static Material material(AssetManager assets, String name, float roughness, boolean twoSided) {
		Material material = new Material(assets, "Common/MatDefs/Light/PBRLighting.j3md");
		material.setName(name);
		material.setColor("BaseColor", ColorRGBA.White);
		material.setFloat("Metallic", 0f);
		material.setFloat("Roughness", roughness);
		material.setBoolean("UseVertexColor", true);
		if (twoSided) {
			material.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
			material.setBoolean("BackfaceShadows", true);
		}
		return material;
	}

	private record BarkMaps(Texture color, Texture roughness, Texture normal) {
	}

	/**
	 * Seamless, original bark: long wandering fissures, smaller grain and subdued brown variation.
	 */
	private static BarkMaps barkMaps() {
		int width = 128, height = 512;
		float[] relief = new float[width * height];
		for (int y = 0; y < height; y++) {
			float v = y * FastMath.TWO_PI / height;
			for (int x = 0; x < width; x++) {
				float u = x * FastMath.TWO_PI / width;
				float warp = 0.45f * FastMath.sin(v * 3f) + 0.19f * FastMath.sin(v * 7f + u * 2f);
				float ridge = (float) Math.pow(Math.abs(FastMath.sin(u * 8f + warp)), 0.42);
				float grain = 0.04f * FastMath.sin(u * 47f + v * 61f) + 0.025f * FastMath.sin(u * 31f - v * 83f);
				relief[y * width + x] = 0.20f + 0.70f * ridge + grain;
			}
		}
		ByteBuffer colors = BufferUtils.createByteBuffer(width * height * 3);
		ByteBuffer roughness = BufferUtils.createByteBuffer(width * height * 3);
		ByteBuffer normals = BufferUtils.createByteBuffer(width * height * 3);
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				float h = relief[y * width + x];
				float mottling = 4f
						* FastMath.sin(x * FastMath.TWO_PI / width * 3f + y * FastMath.TWO_PI / height * 5f);
				colors.put(channel((58f + 85f * h + mottling) / 255f));
				colors.put(channel((43f + 66f * h + mottling) / 255f));
				colors.put(channel((28f + 45f * h + mottling) / 255f));
				byte rough = channel(0.86f + 0.12f * (1f - h));
				roughness.put(rough).put(rough).put(rough);
				float dx = relief[y * width + (x + 1) % width] - relief[y * width + (x + width - 1) % width];
				float dy = relief[((y + 1) % height) * width + x] - relief[((y + height - 1) % height) * width + x];
				Vector3f normal = new Vector3f(-dx * 1.8f, -dy * 1.8f, 1f).normalizeLocal();
				normals.put(channel(normal.x * 0.5f + 0.5f)).put(channel(normal.y * 0.5f + 0.5f))
						.put(channel(normal.z * 0.5f + 0.5f));
			}
		}
		return new BarkMaps(texture(colors, width, height, ColorSpace.sRGB),
				texture(roughness, width, height, ColorSpace.Linear),
				texture(normals, width, height, ColorSpace.Linear));
	}

	/**
	 * A near-white tint map adds restrained midribs and veins to the individual green leaf meshes.
	 */
	private static Texture leafVeins() {
		int width = 64, height = 128;
		ByteBuffer colors = BufferUtils.createByteBuffer(width * height * 3);
		for (int y = 0; y < height; y++) {
			float v = y / (float) (height - 1);
			for (int x = 0; x < width; x++) {
				float distance = Math.abs(x / (float) (width - 1) - 0.5f);
				float midrib = Math.max(0f, 1f - distance * 70f);
				float veinPhase = (v - distance * 0.8f) * 9f;
				float veinDistance = Math.abs(veinPhase - Math.round(veinPhase));
				float veins = Math.max(0f, 1f - veinDistance * 35f) * (1f - distance * 2f);
				byte color = channel(0.91f + 0.06f * midrib + 0.035f * veins);
				colors.put(color).put(color).put(color);
			}
		}
		return texture(colors, width, height, ColorSpace.sRGB);
	}

	private static Texture texture(ByteBuffer pixels, int width, int height, ColorSpace colorSpace) {
		pixels.flip();
		Texture texture = new Texture2D(new Image(Image.Format.RGB8, width, height, pixels, colorSpace));
		texture.setWrap(Texture.WrapMode.Repeat);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setAnisotropicFilter(8);
		return texture;
	}

	private static byte channel(float value) {
		return (byte) Math.round(Math.max(0f, Math.min(1f, value)) * 255f);
	}

	private static void normalizeDirections(Mesh mesh, VertexBuffer.Type type, int components) {
		FloatBuffer directions = mesh.getFloatBuffer(type);
		if (directions == null)
			return;
		FloatBuffer normals = type == VertexBuffer.Type.Tangent ? mesh.getFloatBuffer(VertexBuffer.Type.Normal) : null;
		for (int offset = 0; offset < directions.limit(); offset += components) {
			float x = directions.get(offset), y = directions.get(offset + 1), z = directions.get(offset + 2);
			if (normals != null) {
				// Remove tangent-generator and batching roundoff before rebuilding the PBR tangent frame.
				int normalOffset = offset / components * 3;
				float nx = normals.get(normalOffset), ny = normals.get(normalOffset + 1),
						nz = normals.get(normalOffset + 2);
				float projection = (x * nx + y * ny + z * nz) / (nx * nx + ny * ny + nz * nz);
				x -= nx * projection;
				y -= ny * projection;
				z -= nz * projection;
			}
			float length = FastMath.sqrt(x * x + y * y + z * z);
			directions.put(offset, x / length).put(offset + 1, y / length).put(offset + 2, z / length);
		}
		mesh.getBuffer(type).setUpdateNeeded();
	}

	private static Node cell(Map<Integer, Node> cells, float x, float z) {
		int key = (int) Math.floor((x + 128f) / 64f) + 4 * (int) Math.floor((z + 128f) / 64f);
		return cells.computeIfAbsent(key, k -> new Node("vegetation cell " + k));
	}

	private static void instance(
		Node cell, String name, Mesh mesh, Material material, float x, float y, float z, float scale, float yaw) {
		Geometry geometry = new Geometry(name, mesh);
		geometry.setMaterial(material);
		geometry.setLocalTranslation(x, y - 0.015f, z);
		geometry.setLocalScale(scale);
		geometry.setLocalRotation(new Quaternion().fromAngleAxis(yaw, Vector3f.UNIT_Y));
		geometry.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
		cell.attachChild(geometry);
	}

	private static boolean tooClose(List<Vector3f> positions, float x, float z, float spacing) {
		for (Vector3f p : positions) {
			float dx = p.x - x, dz = p.z - z;
			if (dx * dx + dz * dz < spacing * spacing)
				return true;
		}
		return false;
	}

	private record Template(Mesh wood, Mesh foliage) {
	}

	private static Template broadleaf(Random random) {
		return broadleaf(random, 1f, 1f, 12, true);
	}

	private static Template broadleaf(Random random, float density, float leafScale, int sides, boolean caps) {
		Builder wood = new Builder(1f, 1f, sides, caps), foliage = new Builder(density, leafScale, sides, caps);
		float height = between(random, 6.2f, 7.5f), leanX = between(random, -1.25f, 1.25f),
				leanZ = between(random, -0.9f, 0.9f), crownAngle = random.nextFloat() * FastMath.TWO_PI;
		Vector3f[] trunk = new Vector3f[6];
		for (int i = 0; i < trunk.length; i++) {
			float t = i / 5f;
			trunk[i] = trunkPoint(height, leanX, leanZ, t);
			if (i > 0)
				tube(wood, trunk[i - 1], trunk[i], 0.33f - 0.23f * ((i - 1) / 5f), 0.33f - 0.23f * t, 12, BARK);
		}
		for (int root = 0; root < 5; root++) {
			float angle = root * FastMath.TWO_PI / 5f + between(random, -0.4f, 0.4f);
			tube(wood, new Vector3f(0f, 0.25f, 0f), new Vector3f(FastMath.cos(angle), -0.025f, FastMath.sin(angle)),
					0.14f, 0.025f, 7, BARK);
		}
		for (int branch = 0; branch < 6; branch++) {
			float angle = crownAngle + branch * 2.399963f + between(random, -0.55f, 0.55f);
			Vector3f direction = new Vector3f(FastMath.cos(angle), 0f, FastMath.sin(angle));
			Vector3f fork = trunkPoint(height, leanX, leanZ, between(random, 0.34f, 0.80f));
			Vector3f bend = fork.add(direction.mult(between(random, 0.85f, 1.85f))).addLocal(0f,
					between(random, 0.7f, 1.6f), 0f);
			Vector3f end = fork.add(direction.mult(between(random, 1.9f, 4.3f))).addLocal(leanX * 0.35f,
					between(random, 1.7f, 3.5f), leanZ * 0.35f);
			tube(wood, fork, bend, 0.13f, 0.072f, 9, BARK);
			tube(wood, bend, end, 0.072f, 0.027f, 7, BARK);
			Vector3f innerCrown = fork.clone().interpolateLocal(end, 0.70f).addLocal(0f, between(random, 0.5f, 1.0f),
					0f);
			leafCloud(foliage, random, innerCrown, between(random, 1.25f, 1.75f), between(random, 1.0f, 1.4f), 235,
					BROADLEAF, 0.56f, 0.29f);
			for (int twig = 0; twig < 3; twig++) {
				float twigAngle = angle + between(random, -1.0f, 1.0f);
				Vector3f twigEnd = end.add(FastMath.cos(twigAngle) * between(random, 0.3f, 1.15f),
						between(random, 0.05f, 1.15f), FastMath.sin(twigAngle) * between(random, 0.3f, 1.15f));
				tube(wood, bend.clone().interpolateLocal(end, 0.35f + twig * 0.2f), twigEnd, 0.035f, 0.009f, 5, BARK);
				leafCloud(foliage, random, twigEnd, between(random, 0.9f, 1.35f), between(random, 0.8f, 1.3f), 118,
						BROADLEAF, 0.55f, 0.29f);
			}
		}
		leafCloud(foliage, random, trunk[5].add(leanX * 0.45f, 0.45f, leanZ * 0.45f), 2.1f, 1.7f, 620, BROADLEAF, 0.58f,
				0.31f);
		return new Template(woodMesh(wood), foliage.mesh());
	}

	private static Vector3f trunkPoint(float height, float leanX, float leanZ, float t) {
		return new Vector3f(leanX * t * t + 0.15f * FastMath.sin(t * FastMath.PI), height * t,
				leanZ * t * t - 0.10f * FastMath.sin(t * FastMath.TWO_PI));
	}

	private static Template pine(Random random) {
		return pine(random, 1f, 1f, 12, true);
	}

	private static Template pine(Random random, float density, float leafScale, int sides, boolean caps) {
		Builder wood = new Builder(1f, 1f, sides, caps), foliage = new Builder(density, leafScale, sides, caps);
		float height = between(random, 11.0f, 13.5f), leanX = between(random, -0.9f, 0.9f),
				leanZ = between(random, -0.7f, 0.7f), crownAngle = random.nextFloat() * FastMath.TWO_PI;
		Vector3f previous = Vector3f.ZERO;
		for (int segment = 1; segment <= 5; segment++) {
			float t = segment / 5f;
			Vector3f point = trunkPoint(height, leanX, leanZ, t);
			tube(wood, previous, point, 0.27f - 0.245f * ((segment - 1) / 5f), 0.27f - 0.245f * t, 12, BARK);
			previous = point;
		}
		for (int level = 0; level < 10; level++) {
			float t = (level + between(random, -0.25f, 0.25f)) / 10f;
			float y = 2.1f + (height - 2.8f) * t, baseReach = 3.15f * (1f - t) + 0.12f;
			int branches = 5 + random.nextInt(4);
			for (int branch = 0; branch < branches; branch++) {
				float angle = crownAngle + branch * 2.399963f + level * 0.67f + between(random, -0.45f, 0.45f);
				float reach = baseReach * between(random, 0.72f, 1.23f);
				Vector3f root = trunkPoint(height, leanX, leanZ, (y + between(random, -0.24f, 0.24f)) / height);
				Vector3f end = root.add(FastMath.cos(angle) * reach, between(random, -0.65f, 0.25f) + 0.18f * t,
						FastMath.sin(angle) * reach);
				tube(wood, root, end, 0.052f * (1f - t) + 0.01f, 0.008f, 6, BARK);
				for (int cluster = 0; cluster < 5; cluster++) {
					Vector3f center = root.clone().interpolateLocal(end, 0.20f + cluster * 0.19f);
					int needles = 24 + (int) (reach * 3f);
					leafCloud(foliage, random, center, reach * 0.13f + 0.18f, between(random, 0.32f, 0.48f), needles,
							PINE, 0.48f, 0.095f);
				}
			}
			leafCloud(foliage, random, trunkPoint(height, leanX, leanZ, (y + 0.15f) / height),
					baseReach * 0.35f + 0.15f, 0.48f, 110, PINE, 0.48f, 0.095f);
		}
		leafCloud(foliage, random, trunkPoint(height, leanX, leanZ, (height - 0.1f) / height), 0.28f, 0.6f, 95, PINE,
				0.36f, 0.075f);
		return new Template(woodMesh(wood), foliage.mesh());
	}

	private static Template palm(Random random) {
		return palm(random, 1f, 1f, 12, true);
	}

	private static Template palm(Random random, float density, float leafScale, int sides, boolean caps) {
		Builder wood = new Builder(1f, 1f, sides, caps), foliage = new Builder(density, leafScale, sides, caps);
		float height = between(random, 8.5f, 11.3f), leanX = between(random, 0.8f, 1.85f),
				leanZ = between(random, -1.2f, 1.2f), crownAngle = random.nextFloat() * FastMath.TWO_PI;
		Vector3f previous = Vector3f.ZERO;
		for (int ring = 1; ring <= 16; ring++) {
			float t = ring / 16f;
			Vector3f point = trunkPoint(height, leanX, leanZ, t);
			float radius = 0.23f - 0.08f * t;
			tube(wood, previous, point, radius + 0.023f, radius, 12, tint(BARK, 0.83f + 0.16f * (ring % 2)));
			previous = point;
		}
		Vector3f crown = previous;
		int fronds = 11 + random.nextInt(3);
		for (int frond = 0; frond < fronds; frond++) {
			float angle = crownAngle + frond * 2.399963f + between(random, -0.32f, 0.32f);
			Vector3f forward = new Vector3f(FastMath.cos(angle), 0f, FastMath.sin(angle));
			Vector3f sideways = new Vector3f(-forward.z, 0f, forward.x);
			float length = between(random, 2.8f, 5.2f), droop = between(random, 0.45f, 2.6f),
					lift = between(random, 0.55f, 1.75f), curve = between(random, -0.45f, 0.45f);
			Vector3f start = crown.add(between(random, -0.15f, 0.15f), between(random, 0.1f, 0.45f),
					between(random, -0.15f, 0.15f)), last = start;
			for (int segment = 1; segment <= 6; segment++) {
				float t = segment / 6f;
				Vector3f point = frondPoint(start, forward, sideways, length, droop, lift, curve, t);
				tube(wood, last, point, 0.035f * (1f - (segment - 1) / 6f) + 0.006f, 0.035f * (1f - t) + 0.006f, 5,
						tint(BARK, 0.9f));
				last = point;
			}
			int leaflets = 19 + random.nextInt(8);
			for (int leaflet = 0; leaflet < leaflets; leaflet++) {
				float t = 0.10f + leaflet * 0.78f / (leaflets - 1) + between(random, -0.008f, 0.008f);
				Vector3f point = frondPoint(start, forward, sideways, length, droop, lift, curve, t);
				float bladeLength = (0.35f + 0.78f * FastMath.sin(t * FastMath.PI)) * between(random, 0.83f, 1.16f);
				for (int sign : new int[] {-1, 1}) {
					Vector3f bladeDirection = sideways.mult(sign).addLocal(forward.mult(0.37f))
							.addLocal(0f, -0.16f - 0.37f * t, 0f).normalizeLocal();
					leaf(foliage, point.add(bladeDirection.mult(bladeLength * 0.48f)), bladeDirection,
							new Vector3f(forward.x * 0.13f, 1f, forward.z * 0.13f), bladeLength,
							0.085f * (1f - 0.4f * t), tint(PALM, between(random, 0.85f, 1.18f)));
				}
			}
		}
		return new Template(woodMesh(wood), foliage.mesh());
	}

	private static Vector3f frondPoint(
		Vector3f start, Vector3f forward, Vector3f sideways, float length, float droop, float lift, float curve,
		float t) {
		return start.add(forward.mult(length * t)).addLocal(sideways.mult(curve * t * t)).addLocal(0f,
				lift * FastMath.sin(t * FastMath.PI) - droop * t * t, 0f);
	}

	private static Template shrub(Random random) {
		return shrub(random, 1f, 1f, 12, true);
	}

	private static Template shrub(Random random, float density, float leafScale, int sides, boolean caps) {
		Builder wood = new Builder(1f, 1f, sides, caps), foliage = new Builder(density, leafScale, sides, caps);
		for (int branch = 0; branch < 7; branch++) {
			float angle = branch * 2.399963f + between(random, -0.4f, 0.4f);
			Vector3f end = new Vector3f(FastMath.cos(angle) * between(random, 0.25f, 0.55f),
					between(random, 0.5f, 1.15f), FastMath.sin(angle) * between(random, 0.25f, 0.55f));
			tube(wood, new Vector3f(0f, 0.02f, 0f), end, 0.025f, 0.006f, 5, BARK);
			leafCloud(foliage, random, end, 0.45f, 0.37f, 42, BROADLEAF, 0.30f, 0.155f);
		}
		return new Template(woodMesh(wood), foliage.mesh());
	}

	private static Mesh woodMesh(Builder builder) {
		Mesh mesh = builder.mesh();
		MikktspaceTangentGenerator.generate(mesh);
		return mesh;
	}

	private static void leafCloud(
		Builder builder, Random random, Vector3f center, float radius, float verticalRadius, int count, ColorRGBA color,
		float length, float width) {
		float aspect = between(random, 0.75f, 1.3f), cloudAngle = random.nextFloat() * FastMath.TWO_PI;
		float cosine = FastMath.cos(cloudAngle), sine = FastMath.sin(cloudAngle);
		float warmth = between(random, -0.12f, 0.14f);
		ColorRGBA cloudColor = new ColorRGBA(color.r * (1f + warmth), color.g * (1f + warmth * 0.15f),
				color.b * (1f - warmth * 0.65f), 1f);
		for (int i = 0; i < count; i++) {
			float angle = random.nextFloat() * FastMath.TWO_PI, elevation = between(random, -1f, 1f);
			float distance = (float) Math.cbrt(random.nextFloat()),
					horizontal = FastMath.sqrt(1f - elevation * elevation);
			float localX = FastMath.cos(angle) * horizontal * distance * radius * aspect;
			float localZ = FastMath.sin(angle) * horizontal * distance * radius / aspect;
			Vector3f position = center.add(localX * cosine - localZ * sine, elevation * distance * verticalRadius,
					localX * sine + localZ * cosine);
			float leafAngle = random.nextFloat() * FastMath.TWO_PI;
			Vector3f direction = new Vector3f(FastMath.cos(leafAngle), between(random, -0.6f, 0.6f),
					FastMath.sin(leafAngle)).normalizeLocal();
			Vector3f normal = new Vector3f(between(random, -0.6f, 0.6f), 1f, between(random, -0.6f, 0.6f));
			float shade = between(random, 0.82f, 1.22f) * (0.92f + 0.08f * elevation);
			leaf(builder, position, direction, normal, length * between(random, 0.72f, 1.35f),
					width * between(random, 0.75f, 1.25f), tint(cloudColor, shade));
		}
	}

	/**
	 * Five-vertex folded leaf; its four triangles have an actual outline rather than an alpha mask.
	 */
	private static void leaf(
		Builder builder, Vector3f center, Vector3f direction, Vector3f normal, float length, float width,
		ColorRGBA color) {
		if (!builder.emitLeaf())
			return;
		length *= builder.leafScale;
		width *= builder.leafScale;
		Vector3f side = normal.cross(direction).normalizeLocal();
		normal = direction.cross(side).normalizeLocal();
		Vector3f[] rim = {direction.mult(length * 0.5f), side.mult(width * 0.5f), direction.mult(-length * 0.5f),
				side.mult(-width * 0.5f)};
		float fold = width * 0.12f;
		int middle = builder.vertex(center.add(normal.mult(fold)), normal, 0.5f, 0.5f, color);
		int[] indices = new int[4];
		for (int i = 0; i < rim.length; i++) {
			Vector3f edgeNormal = normal.add(rim[i].normalize().mult(0.18f)).normalizeLocal();
			indices[i] = builder.vertex(center.add(rim[i]), edgeNormal, 0.5f + (i == 1 ? 0.5f : i == 3 ? -0.5f : 0f),
					0.5f + (i == 0 ? 0.5f : i == 2 ? -0.5f : 0f), color);
		}
		for (int i = 0; i < 4; i++)
			builder.triangle(middle, indices[i], indices[(i + 1) % 4]);
	}

	private static Mesh grass(Random random) {
		Builder builder = new Builder();
		for (int blade = 0; blade < 17; blade++) {
			float angle = random.nextFloat() * FastMath.TWO_PI, height = between(random, 0.28f, 0.68f);
			Vector3f forward = new Vector3f(FastMath.cos(angle), 0f, FastMath.sin(angle));
			Vector3f side = new Vector3f(-forward.z, 0f, forward.x);
			Vector3f root = forward.mult(between(random, 0f, 0.34f));
			float width = between(random, 0.025f, 0.055f), bend = between(random, 0.10f, 0.32f);
			int[] left = new int[4], right = new int[4];
			for (int segment = 0; segment < 4; segment++) {
				float t = segment / 3f, halfWidth = width * (1f - t) * 0.5f;
				Vector3f center = root.add(forward.mult(bend * t * t)).addLocal(0f, height * t, 0f);
				Vector3f normal = new Vector3f(-forward.x * 0.65f, 0.76f, -forward.z * 0.65f).normalizeLocal();
				ColorRGBA color = tint(GRASS, (0.63f + 0.5f * t) * between(random, 0.85f, 1.1f));
				left[segment] = builder.vertex(center.subtract(side.mult(halfWidth)), normal, 0f, t, color);
				right[segment] = builder.vertex(center.add(side.mult(halfWidth)), normal, 1f, t, color);
				if (segment > 0) {
					builder.triangle(left[segment - 1], right[segment - 1], left[segment]);
					if (segment < 3)
						builder.triangle(right[segment - 1], right[segment], left[segment]);
				}
			}
		}
		return builder.mesh();
	}

	/** Tapered wood with subtle longitudinal ridges, surface normals, and closed cut ends. */
	private static void tube(
		Builder builder, Vector3f start, Vector3f end, float baseRadius, float tipRadius, int sides, ColorRGBA color) {
		sides = Math.min(sides, builder.maxTubeSides);
		Vector3f axis = end.subtract(start);
		float length = axis.length();
		axis.divideLocal(length);
		Vector3f reference = Math.abs(axis.y) > 0.9f ? Vector3f.UNIT_X : Vector3f.UNIT_Y;
		Vector3f u = axis.cross(reference).normalizeLocal(), v = axis.cross(u).normalizeLocal();
		int[][] ring = new int[2][sides + 1];
		for (int level = 0; level < 2; level++) {
			float radius = level == 0 ? baseRadius : tipRadius;
			Vector3f center = level == 0 ? start : end;
			for (int i = 0; i <= sides; i++) {
				float angle = i * FastMath.TWO_PI / sides;
				Vector3f radial = u.mult(FastMath.cos(angle)).addLocal(v.mult(FastMath.sin(angle)));
				float ridge = 1f + 0.065f * FastMath.sin(angle * 5f + 0.3f);
				Vector3f normal = radial.add(axis.mult((baseRadius - tipRadius) / length)).normalizeLocal();
				ring[level][i] = builder.vertex(center.add(radial.mult(radius * ridge)), normal, i / (float) sides,
						level * length, tint(color, 0.88f + 0.12f * ridge));
			}
		}
		for (int i = 0; i < sides; i++) {
			int next = i + 1;
			builder.triangle(ring[0][i], ring[0][next], ring[1][i]);
			builder.triangle(ring[0][next], ring[1][next], ring[1][i]);
		}
		if (!builder.capEnds)
			return;
		// Cut ends have flat normals and circular UVs; sharing the side-wall vertices creates
		// degenerate cap UVs and can make their generated tangent parallel to their normal.
		for (int level = 0; level < 2; level++) {
			Vector3f center = level == 0 ? start : end, normal = level == 0 ? axis.negate() : axis;
			float radius = level == 0 ? baseRadius : tipRadius;
			int middle = builder.vertex(center, normal, 0.5f, 0.5f, color);
			int[] cap = new int[sides];
			for (int i = 0; i < sides; i++) {
				float angle = i * FastMath.TWO_PI / sides, cosine = FastMath.cos(angle), sine = FastMath.sin(angle);
				float ridge = 1f + 0.065f * FastMath.sin(angle * 5f + 0.3f);
				Vector3f radial = u.mult(cosine).addLocal(v.mult(sine));
				cap[i] = builder.vertex(center.add(radial.mult(radius * ridge)), normal, 0.5f + cosine * 0.5f,
						0.5f + sine * 0.5f, color);
			}
			for (int i = 0; i < sides; i++) {
				int next = (i + 1) % sides;
				if (level == 0)
					builder.triangle(middle, cap[next], cap[i]);
				else
					builder.triangle(middle, cap[i], cap[next]);
			}
		}
	}

	private static void checkCancelled() {
		if (Thread.currentThread().isInterrupted())
			throw new java.util.concurrent.CancellationException("Forest preparation interrupted");
	}

	private static float between(Random random, float minimum, float maximum) {
		return minimum + random.nextFloat() * (maximum - minimum);
	}

	private static ColorRGBA tint(ColorRGBA color, float brightness) {
		return new ColorRGBA(color.r * brightness, color.g * brightness, color.b * brightness, 1f);
	}

	private static final class Builder {
		private float[] vertices = new float[4096];
		private int[] indices = new int[2048];
		private int vertexCount, indexCount;
		private final float leafDensity, leafScale;
		private final int maxTubeSides;
		private final boolean capEnds;
		private float leafAccumulator;

		Builder() {
			this(1f, 1f, 12, true);
		}

		Builder(float leafDensity, float leafScale, int maxTubeSides, boolean capEnds) {
			this.leafDensity = leafDensity;
			this.leafScale = leafScale;
			this.maxTubeSides = maxTubeSides;
			this.capEnds = capEnds;
		}

		Builder(int vertexCapacity, int indexCapacity) {
			this();
			vertices = new float[Math.max(12, vertexCapacity * 12)];
			indices = new int[Math.max(3, indexCapacity)];
		}

		boolean emitLeaf() {
			leafAccumulator += leafDensity;
			if (leafAccumulator < 0.99999f)
				return false;
			leafAccumulator -= 1f;
			return true;
		}

		int vertex(Vector3f position, Vector3f normal, float u, float v, ColorRGBA color) {
			return vertex(position.x, position.y, position.z, normal.x, normal.y, normal.z, u, v, color.r, color.g,
					color.b, color.a);
		}

		int vertex(
			float x, float y, float z, float nx, float ny, float nz, float u, float v, float r, float g, float b,
			float alpha) {
			if ((vertexCount & 511) == 0)
				checkCancelled();
			int offset = vertexCount * 12;
			if (offset + 12 > vertices.length)
				vertices = Arrays.copyOf(vertices, vertices.length * 2);
			vertices[offset] = x;
			vertices[offset + 1] = y;
			vertices[offset + 2] = z;
			vertices[offset + 3] = nx;
			vertices[offset + 4] = ny;
			vertices[offset + 5] = nz;
			vertices[offset + 6] = u;
			vertices[offset + 7] = v;
			vertices[offset + 8] = r;
			vertices[offset + 9] = g;
			vertices[offset + 10] = b;
			vertices[offset + 11] = alpha;
			return vertexCount++;
		}

		void triangle(int a, int b, int c) {
			if (indexCount + 3 > indices.length)
				indices = Arrays.copyOf(indices, indices.length * 2);
			indices[indexCount++] = a;
			indices[indexCount++] = b;
			indices[indexCount++] = c;
		}

		Mesh mesh() {
			FloatBuffer positions = BufferUtils.createFloatBuffer(vertexCount * 3);
			FloatBuffer normals = BufferUtils.createFloatBuffer(vertexCount * 3);
			FloatBuffer texcoords = BufferUtils.createFloatBuffer(vertexCount * 2);
			FloatBuffer colors = BufferUtils.createFloatBuffer(vertexCount * 4);
			for (int i = 0; i < vertexCount; i++) {
				positions.put(vertices, i * 12, 3);
				normals.put(vertices, i * 12 + 3, 3);
				texcoords.put(vertices, i * 12 + 6, 2);
				colors.put(vertices, i * 12 + 8, 4);
			}
			positions.flip();
			normals.flip();
			texcoords.flip();
			colors.flip();
			Mesh mesh = new Mesh();
			mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
			mesh.setBuffer(VertexBuffer.Type.Normal, 3, normals);
			mesh.setBuffer(VertexBuffer.Type.TexCoord, 2, texcoords);
			mesh.setBuffer(VertexBuffer.Type.Color, 4, colors);
			mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(Arrays.copyOf(indices, indexCount)));
			mesh.setStatic();
			mesh.updateBound();
			return mesh;
		}
	}
}
