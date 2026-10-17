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

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import com.jme3.asset.AssetKey;
import com.jme3.asset.AssetManager;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.material.Material;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.mesh.IndexBuffer;
import com.jme3.scene.shape.Sphere;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.TextureArray;
import com.jme3.texture.image.ColorSpace;
import com.jme3.texture.image.ImageRaster;
import com.jme3.util.BufferUtils;
import com.jme3.util.mikktspace.MikktspaceTangentGenerator;

/** A small, metre-scale coastal landscape for the standalone shoreline scene. */
public final class ShorelineTerrain {

	private static final int GRID_SIZE = 513;
	private static final int PATCH_SIZE = 65;
	private static final float CELL_SIZE = 0.5f;
	private static final int TEXTURE_SIZE = 512;
	private static final String PBR_TEXTURES = "Textures/Terrain/PBR/";

	private ShorelineTerrain() {
	}

	/** Sand colour and OpenGL normal textures for the game's TerrainLighting material. */
	public record SandTextures(Texture2D color, Texture2D normal) {
	}

	/**
	 * Reuses the original coastal sand images with the asset manager across game world rebuilds.
	 */
	public static SandTextures sandTextures(AssetManager assets) {
		AssetKey<Image> colorKey = new AssetKey<>("procedural/coastal-sand-color-v1");
		AssetKey<Image> normalKey = new AssetKey<>("procedural/coastal-sand-normal-opengl-v1");
		Image color = assets.getFromCache(colorKey), normal = assets.getFromCache(normalKey);
		if (color == null || normal == null) {
			Image[] generated = sandImages(false);
			color = generated[0];
			normal = generated[1];
			// PBR terrain uses DirectX normals; TerrainLighting decodes OpenGL tangent-space normals.
			ByteBuffer pixels = normal.getData(0);
			for (int green = 1; green < pixels.limit(); green += 4)
				pixels.put(green, (byte) (255 - (pixels.get(green) & 255)));
			assets.addToCache(colorKey, color);
			assets.addToCache(normalKey, normal);
		}
		return new SandTextures(sandTexture(color, "Coastal sand"), sandTexture(normal, "Coastal sand normal"));
	}

	private static Texture2D sandTexture(Image image, String name) {
		Texture2D texture = new Texture2D(image);
		texture.setName(name);
		texture.setWrap(Texture.WrapMode.Repeat);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setAnisotropicFilter(8);
		return texture;
	}

	/** Returns the fixed landscape's ground height, in metres, over [-128,128] on both axes. */
	public static float heightAt(float x, float z) {
		float distance = z - coastAt(x);
		float beach = distance >= 0f ? distance * 0.075f : distance * 0.12f - distance * distance * 0.0032f;
		float inland = smooth(12f, 65f, distance);
		float rolling = 3f + 2.2f * FastMath.sin(x * 0.037f + z * 0.045f)
				+ 1.5f * FastMath.cos(x * 0.061f - z * 0.029f);
		float hills = 14f * gaussian((x + 55f) / 46f, (z - 75f) / 38f)
				+ 19f * gaussian((x - 65f) / 43f, (z - 92f) / 44f) + 4f * gaussian((x + 18f) / 27f, (z - 43f) / 22f);
		float headland = smooth(55f, 88f, x) * gaussian(0f, (distance - 13f) / 24f)
				* (10f + 2.2f * FastMath.sin(x * 0.085f + z * 0.055f) + 1.1f * FastMath.sin(x * 0.19f - z * 0.11f));
		float microrelief = smooth(2f, 10f, distance) * 0.12f * FastMath.sin(x * 0.71f)
				* FastMath.sin(z * 0.43f + x * 0.14f);
		float height = beach + inland * (rolling + hills) + headland + microrelief;
		// Sink the mesh perimeter below the water so its square extent never forms an exposed cliff.
		// The forest placement domain ends at |x|=109 and z=106, before this coastal falloff starts.
		float side = smooth(110f, 128f, Math.abs(x));
		float back = smooth(108f, 128f, z);
		float border = 1f - (1f - side) * (1f - back);
		return (1f - border) * height + border * Math.min(height, -64f);
	}

	/** Builds the terrain and a restrained cluster of seeded shoreline boulders. */
	public static Node create(AssetManager assets, long seed) {
		Node landscape = new Node("Shoreline landscape");
		float[] heights = new float[GRID_SIZE * GRID_SIZE];
		for (int row = 0; row < GRID_SIZE; row++) {
			float z = -128f + row * CELL_SIZE;
			for (int col = 0; col < GRID_SIZE; col++)
				heights[row * GRID_SIZE + col] = heightAt(-128f + col * CELL_SIZE, z) / CELL_SIZE;
		}
		TerrainQuad terrain = new TerrainQuad("Sandy cove and headland", PATCH_SIZE, GRID_SIZE, heights);
		// Triplanar projections use patch-local positions: all axes must share one physical unit.
		terrain.setLocalScale(CELL_SIZE);
		terrain.setMaterial(terrainMaterial(assets));
		terrain.setShadowMode(ShadowMode.CastAndReceive);
		landscape.attachChild(terrain);
		addBoulders(landscape, assets, seed);
		return landscape;
	}

	private static Material terrainMaterial(AssetManager assets) {
		return islandMaterial(assets, coverageMap(), CELL_SIZE);
	}

	/**
	 * Shares the showcase's four coastal PBR surfaces and submerged gravel with generated islands.
	 * Local terrain units are converted to metre-scale texture tiles; integer repeat counts keep
	 * patch boundaries seamless.
	 */
	public static Material islandMaterial(AssetManager assets, Texture2D coverage, float cellSize) {
		Material material = ShorelineMaterials.terrain(assets);
		// Keep gravel below water independent of the rock used on exposed cliffs.
		material.setBoolean("SeabedGravel", true);
		material.setFloat("SeabedAlbedoScale", 1.2f);
		float seabedRepeats = Math.max(1, Math.round((PATCH_SIZE - 1) * cellSize / 6f));
		material.setFloat("SeabedMapScale", seabedRepeats / (PATCH_SIZE - 1));
		AssetKey<TextureArray> colorKey = new AssetKey<>("procedural/coastal-pbr-albedo-v2");
		AssetKey<TextureArray> normalKey = new AssetKey<>("procedural/coastal-pbr-normal-v2");
		AssetKey<TextureArray> packedKey = new AssetKey<>("procedural/coastal-pbr-surface-v2");
		TextureArray color = assets.getFromCache(colorKey), normal = assets.getFromCache(normalKey),
				packed = assets.getFromCache(packedKey);
		if (color == null || normal == null || packed == null) {
			Image[] sand = sandImages(false), wetSand = sandImages(true);
			Image[] floor = groundImages(assets, "Ground037", "Ground036");
			Image[] rock = groundImages(assets, "Rock035", null);
			Image[] gravel = groundImages(assets, "Gravel015", null);
			color = textureArray(sand[0], floor[0], rock[0], wetSand[0], gravel[0]);
			normal = textureArray(sand[1], floor[1], rock[1], gravel[1]);
			packed = textureArray(sand[2], floor[2], rock[2], gravel[2]);
			assets.addToCache(colorKey, color);
			assets.addToCache(normalKey, normal);
			assets.addToCache(packedKey, packed);
		}
		material.setTexture("AlbedoTextureArray", color);
		material.setTexture("NormalParallaxTextureArray", normal);
		material.setTexture("MetallicRoughnessAoEiTextureArray", packed);
		material.setTexture("AlphaMap", coverage);
		material.setBoolean("useTriPlanarMapping", true);
		material.setFloat("NormalType", -1f);
		for (int layer = 0; layer < 4; layer++) {
			int source = layer == 3 ? 0 : layer;
			material.setInt("AlbedoMap_" + layer, layer);
			material.setInt("NormalMap_" + layer, source);
			material.setInt("MetallicRoughnessMap_" + layer, source);
			material.setFloat("Roughness_" + layer, layer == 3 ? 0.45f : 1f);
			material.setFloat("Metallic_" + layer, 0f);
			float tileMetres = layer == 1 || layer == 2 ? 4f : 2f;
			float repeats = Math.max(1, Math.round((PATCH_SIZE - 1) * cellSize / tileMetres));
			material.setFloat("AlbedoMap_" + layer + "_scale", repeats / (PATCH_SIZE - 1));
		}
		return material;
	}

	private static Texture2D coverageMap() {
		ByteBuffer pixels = BufferUtils.createByteBuffer(GRID_SIZE * GRID_SIZE * 4);
		for (int row = 0; row < GRID_SIZE; row++) {
			// Terrain UV v increases towards -Z, opposite the heightmap row order.
			float z = 128f - row * CELL_SIZE;
			for (int col = 0; col < GRID_SIZE; col++) {
				float x = -128f + col * CELL_SIZE;
				float distance = z - coastAt(x);
				float height = heightAt(x, z);
				float dx = (heightAt(x + 0.5f, z) - heightAt(x - 0.5f, z));
				float dz = (heightAt(x, z + 0.5f) - heightAt(x, z - 0.5f));
				float slope = FastMath.sqrt(dx * dx + dz * dz);
				float headland = smooth(55f, 90f, x) * (1f - smooth(24f, 48f, distance));
				float rock = headland * smooth(-0.3f, 2.5f, height) * (0.5f + 0.5f * smooth(0.12f, 0.45f, slope));
				float border = Math.max(smooth(110f, 124f, Math.abs(x)), smooth(108f, 124f, z));
				rock = Math.max(rock, border * smooth(0.35f, 0.85f, slope));
				float variation = 2f * FastMath.sin(x * 0.11f) * FastMath.cos(z * 0.095f);
				float forest = smooth(4.5f, 18f, distance + variation) * (1f - rock);
				float wet = (1f - smooth(0.25f, 1.2f, height + variation * 0.06f)) * (1f - rock);
				// PBR terrain blends layers in order; channels are overlay coverage, not weights.
				pixels.put((byte) 255).put(channel(forest)).put(channel(rock)).put(channel(wet));
			}
		}
		pixels.flip();
		Texture2D texture = new Texture2D(
				new Image(Image.Format.RGBA8, GRID_SIZE, GRID_SIZE, pixels, ColorSpace.Linear));
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		texture.setMinFilter(Texture.MinFilter.BilinearNoMipMaps);
		return texture;
	}

	/** Original periodic sand grain and shallow ripple detail; the tile represents two metres. */
	private static Image[] sandImages(boolean wet) {
		ByteBuffer albedo = BufferUtils.createByteBuffer(TEXTURE_SIZE * TEXTURE_SIZE * 4);
		ByteBuffer normal = BufferUtils.createByteBuffer(TEXTURE_SIZE * TEXTURE_SIZE * 4);
		ByteBuffer packed = BufferUtils.createByteBuffer(TEXTURE_SIZE * TEXTURE_SIZE * 4);
		for (int y = 0; y < TEXTURE_SIZE; y++) {
			for (int x = 0; x < TEXTURE_SIZE; x++) {
				float u = x / (float) TEXTURE_SIZE, v = y / (float) TEXTURE_SIZE;
				float phase = FastMath.TWO_PI * (v * 12f + 0.25f * FastMath.sin(u * FastMath.TWO_PI * 2f));
				int grain = x * 73428767 ^ y * 912931;
				grain = (grain ^ grain >>> 13) * 1274126177;
				float detail = ((grain >>> 16) & 255) / 255f - 0.5f;
				float light = 1f + 0.06f * FastMath.sin(phase) + 0.10f * detail;
				float darkening = wet ? 0.65f : 1f;
				albedo.put(channel(0.76f * light * darkening)).put(channel(0.69f * light * darkening))
						.put(channel(0.54f * light * darkening)).put((byte) 255);
				Vector3f n = new Vector3f(-0.025f * FastMath.cos(phase), 0.09f * FastMath.cos(phase), 1f)
						.normalizeLocal();
				normal.put(channel(n.x * 0.5f + 0.5f)).put(channel(n.y * 0.5f + 0.5f)).put(channel(n.z * 0.5f + 0.5f))
						.put((byte) 0);
				// AdvancedPBRTerrain packs AO, roughness, metallic and emission into RGBA.
				packed.put((byte) 255).put(channel(0.82f + detail * 0.10f)).put((byte) 0).put((byte) 0);
			}
		}
		return new Image[] {image(albedo, ColorSpace.sRGB), image(normal, ColorSpace.Linear),
				image(packed, ColorSpace.Linear)};
	}

	private static Image[] groundImages(AssetManager assets, String name, String secondary) {
		ImageRaster color = raster(assets, name, "Color");
		ImageRaster normal = raster(assets, name, "Normal");
		ImageRaster roughness = raster(assets, name, "Roughness");
		ImageRaster ao = raster(assets, name, "AmbientOcclusion");
		ImageRaster otherColor = secondary == null ? null : raster(assets, secondary, "Color");
		ImageRaster otherNormal = secondary == null ? null : raster(assets, secondary, "Normal");
		ImageRaster otherRoughness = secondary == null ? null : raster(assets, secondary, "Roughness");
		ByteBuffer colors = BufferUtils.createByteBuffer(TEXTURE_SIZE * TEXTURE_SIZE * 4);
		ByteBuffer normals = BufferUtils.createByteBuffer(TEXTURE_SIZE * TEXTURE_SIZE * 4);
		ByteBuffer packed = BufferUtils.createByteBuffer(TEXTURE_SIZE * TEXTURE_SIZE * 4);
		ColorRGBA sample = new ColorRGBA(), other = new ColorRGBA();
		for (int y = 0; y < TEXTURE_SIZE; y++) {
			for (int x = 0; x < TEXTURE_SIZE; x++) {
				int px = x * color.getWidth() / TEXTURE_SIZE, py = y * color.getHeight() / TEXTURE_SIZE;
				color.getPixel(px, py, sample);
				if (otherColor != null) {
					otherColor.getPixel(px, py, other);
					sample.interpolateLocal(other, 0.22f);
					// Retain the CC0 moss/litter detail, with a deeper coastal green instead of dry olive.
					sample.r = tintSrgb(sample.r, 0.40f);
					sample.g = tintSrgb(sample.g, 0.75f);
					sample.b = tintSrgb(sample.b, 0.50f);
				}
				colors.put(channel(sample.r)).put(channel(sample.g)).put(channel(sample.b)).put((byte) 255);
				normal.getPixel(px, py, sample);
				if (otherNormal != null) {
					otherNormal.getPixel(px, py, other);
					sample.interpolateLocal(other, 0.22f);
				}
				normals.put(channel(sample.r)).put(channel(sample.g)).put(channel(sample.b)).put((byte) 0);
				roughness.getPixel(px, py, sample);
				float rough = sample.r;
				if (otherRoughness != null) {
					otherRoughness.getPixel(px, py, other);
					rough = 0.78f * rough + 0.22f * other.r;
				}
				ao.getPixel(px, py, sample);
				packed.put(channel(sample.r)).put(channel(rough)).put((byte) 0).put((byte) 0);
			}
		}
		return new Image[] {image(colors, ColorSpace.sRGB), image(normals, ColorSpace.Linear),
				image(packed, ColorSpace.Linear)};
	}

	private static ImageRaster raster(AssetManager assets, String name, String kind) {
		return ImageRaster.create(assets.loadTexture(PBR_TEXTURES + name + "_1K_" + kind + ".png").getImage(), 0, 0,
				false);
	}

	private static float tintSrgb(float encoded, float reflectanceScale) {
		float linear = encoded <= 0.04045f ? encoded / 12.92f : (float) Math.pow((encoded + 0.055f) / 1.055f, 2.4);
		linear *= reflectanceScale;
		return linear <= 0.0031308f ? linear * 12.92f : 1.055f * (float) Math.pow(linear, 1.0 / 2.4) - 0.055f;
	}

	private static Image image(ByteBuffer data, ColorSpace colorSpace) {
		data.flip();
		return new Image(Image.Format.RGBA8, TEXTURE_SIZE, TEXTURE_SIZE, data, colorSpace);
	}

	private static TextureArray textureArray(Image ... images) {
		TextureArray texture = new TextureArray(List.of(images));
		texture.setWrap(Texture.WrapMode.Repeat);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		// Generate array mipmaps on the GPU, retaining the sRGB texture format for colour filtering.
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setAnisotropicFilter(8);
		return texture;
	}

	private static void addBoulders(Node landscape, AssetManager assets, long seed) {
		Random random = new Random(seed ^ 0x57a0e5b091cL);
		Material dry = rockMaterial(assets, false), wet = rockMaterial(assets, true);
		Node rocks = new Node("Headland and tide rocks");
		for (int i = 0; i < 58; i++) {
			boolean headland = i < 34;
			float x = headland ? 59f + random.nextFloat() * 60f : -107f + random.nextFloat() * 166f;
			float z = waterlineAt(x) + (headland ? -7f + random.nextFloat() * 27f : -6f + random.nextFloat() * 9f);
			float radius = headland ? 1.1f + random.nextFloat() * 3.1f : 0.35f + random.nextFloat() * 1.2f;
			float ground = heightAt(x, z);
			Geometry rock = new Geometry("Boulder " + i, boulderMesh(random.nextFloat() * FastMath.TWO_PI));
			rock.setLocalScale(radius * (0.8f + random.nextFloat() * 0.7f), radius * (0.5f + random.nextFloat() * 0.5f),
					radius);
			rock.setLocalTranslation(x, ground + radius * 0.25f, z);
			rock.rotate(random.nextFloat() * 0.25f, random.nextFloat() * FastMath.TWO_PI, random.nextFloat() * 0.3f);
			rock.setMaterial(ground < 0.9f ? wet : dry);
			rock.setShadowMode(ShadowMode.CastAndReceive);
			rocks.attachChild(rock);
		}
		landscape.attachChild(rocks);
	}

	/** Shares the showcase's CC0 rock surfaces with streamed game scenery. */
	public static Material rockMaterial(AssetManager assets, boolean wet) {
		Material material = new Material(assets, "Common/MatDefs/Light/PBRLighting.j3md");
		material.setTexture("BaseColorMap", rockTexture(assets, "Color", ColorSpace.sRGB));
		material.setTexture("NormalMap", rockTexture(assets, "Normal", ColorSpace.Linear));
		material.setTexture("RoughnessMap", rockTexture(assets, "Roughness", ColorSpace.Linear));
		material.setFloat("Metallic", 0f);
		material.setFloat("NormalType", -1f);
		material.setFloat("Roughness", wet ? 0.42f : 0.95f);
		material.setColor("BaseColor",
				wet ? new ColorRGBA(0.56f, 0.59f, 0.60f, 1f) : new ColorRGBA(0.9f, 0.92f, 0.92f, 1f));
		return material;
	}

	private static Texture rockTexture(AssetManager assets, String kind, ColorSpace colorSpace) {
		Texture texture = assets.loadTexture(PBR_TEXTURES + "Rock035_1K_" + kind + ".png");
		texture.getImage().setColorSpace(colorSpace);
		texture.setWrap(Texture.WrapMode.Repeat);
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setAnisotropicFilter(8);
		return texture;
	}

	/** Seeded irregular, flattened boulder geometry with smooth normals and PBR tangents. */
	public static Mesh boulderMesh(float phase) {
		Mesh mesh = new Sphere(18, 26, 1f);
		FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
		for (int i = 0; i < positions.limit(); i += 3) {
			float x = positions.get(i), y = positions.get(i + 1), z = positions.get(i + 2);
			float radial = 0.9f + 0.11f * FastMath.sin(x * 5f + phase) * FastMath.sin(y * 4f + phase * 0.7f)
					+ 0.08f * FastMath.cos(x * 7f - z * 4f + phase);
			positions.put(i, x * radial);
			positions.put(i + 1, Math.max(-0.72f, Math.min(0.79f, y * radial)));
			positions.put(i + 2, z * radial);
		}
		mesh.getBuffer(VertexBuffer.Type.Position).updateData(positions);
		updateRockNormals(mesh);
		MikktspaceTangentGenerator.generate(mesh);
		mesh.updateBound();
		mesh.setStatic();
		return mesh;
	}

	private static void updateRockNormals(Mesh mesh) {
		FloatBuffer positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
		IndexBuffer indices = mesh.getIndicesAsList();
		Map<String, Vector3f> sharedNormals = new HashMap<>();
		List<String> keys = new ArrayList<>();
		for (int vertex = 0; vertex < mesh.getVertexCount(); vertex++) {
			Vector3f position = positionAt(positions, vertex);
			String key = Math.round(position.x * 10000f) + ":" + Math.round(position.y * 10000f) + ":"
					+ Math.round(position.z * 10000f);
			keys.add(key);
			sharedNormals.putIfAbsent(key, new Vector3f());
		}
		for (int i = 0; i < indices.size(); i += 3) {
			int ia = indices.get(i), ib = indices.get(i + 1), ic = indices.get(i + 2);
			Vector3f a = positionAt(positions, ia);
			Vector3f b = positionAt(positions, ib);
			Vector3f c = positionAt(positions, ic);
			Vector3f normal = b.subtract(a).crossLocal(c.subtract(a));
			sharedNormals.get(keys.get(ia)).addLocal(normal);
			sharedNormals.get(keys.get(ib)).addLocal(normal);
			sharedNormals.get(keys.get(ic)).addLocal(normal);
		}
		FloatBuffer normals = BufferUtils.createVector3Buffer(mesh.getVertexCount());
		for (String key : keys) {
			Vector3f normal = sharedNormals.get(key).normalizeLocal();
			normals.put(normal.x).put(normal.y).put(normal.z);
		}
		normals.flip();
		mesh.setBuffer(VertexBuffer.Type.Normal, 3, normals);
	}

	private static Vector3f positionAt(FloatBuffer positions, int vertex) {
		Vector3f position = new Vector3f();
		BufferUtils.populateFromBuffer(position, positions, vertex);
		return position;
	}

	private static float waterlineAt(float x) {
		float low = -110f, high = 30f;
		for (int i = 0; i < 20; i++) {
			float mid = (low + high) * 0.5f;
			if (heightAt(x, mid) < 0f)
				low = mid;
			else
				high = mid;
		}
		return (low + high) * 0.5f;
	}

	private static float coastAt(float x) {
		return -25f + 10f * FastMath.sin(x * 0.035f) + 5f * FastMath.sin(x * 0.075f);
	}

	private static float gaussian(float x, float y) {
		return (float) Math.exp(-x * x - y * y);
	}

	private static float smooth(float low, float high, float value) {
		float t = FastMath.clamp((value - low) / (high - low), 0f, 1f);
		return t * t * (3f - 2f * t);
	}

	private static byte channel(float value) {
		return (byte) Math.round(FastMath.clamp(value, 0f, 1f) * 255f);
	}
}
