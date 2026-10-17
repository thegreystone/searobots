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
import com.jme3.texture.image.ColorSpace;
import com.jme3.renderer.Camera;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.engine.PerlinNoise;
import se.hirt.searobots.viewer.shoreline.ShorelineTerrain;

import java.nio.ByteBuffer;
import java.util.function.BiConsumer;

/**
 * Builds a jMonkeyEngine {@link TerrainQuad} from a {@link TerrainMap}, with lit texture splatting
 * via procedural alpha maps, normal maps, and tri-planar texturing for steep slopes.
 * <p>
 * Four PBR surface materials are blended by elevation and slope:
 * <ol>
 * <li>Dry coastal sand</li>
 * <li>Forest floor</li>
 * <li>Cliff rock, blended with gravel and sediment below water</li>
 * <li>Wet sand around the waterline</li>
 * </ol>
 * <p>
 * TerrainQuad requires a (2^n + 1) square heightmap. The simulation's 1600x1600 grid is resampled
 * to 2049x2049 with bilinear interpolation, retaining the coastline's source detail.
 */
final class TerrainQuadBuilder {

	/** Terrain quad size: 2049 = 2^11 + 1, about 8m per sample in the default world. */
	private static final int QUAD_SIZE = 2049;
	/** Patch size for LOD tiles: 65 = 2^6 + 1. */
	private static final int PATCH_SIZE = 65;
	private static final PerlinNoise SURFACE_NOISE = new PerlinNoise(0x534f494c50415443L);
	private static final BiConsumer<Float, String> NO_PROGRESS = (fraction, stage) -> {
	};

	private TerrainQuadBuilder() {
	}

	/**
	 * Build a TerrainQuad from the simulation terrain map.
	 *
	 * @param terrain
	 *            the simulation heightmap
	 * @param assetManager
	 *            jME asset manager for loading textures
	 * @param camera
	 *            main view camera
	 * @return a TerrainQuad positioned and scaled to match world coordinates
	 */
	static TerrainQuad build(TerrainMap terrain, AssetManager assetManager, Camera camera) {
		return build(terrain, assetManager, camera, NO_PROGRESS);
	}

	/**
	 * Builds detached terrain, reporting completed preparation work on the calling thread. The
	 * callback receives a fraction from zero to one and the current stage. No camera or attached
	 * scene state is accessed, so a loading worker may pass a null camera.
	 */
	static TerrainQuad build(
		TerrainMap terrain, AssetManager assetManager, Camera camera, BiConsumer<Float, String> progress) {
		progress.accept(0f, "Sampling terrain");
		double worldW = terrain.worldWidth();
		float scaleXZ = (float) (worldW / (QUAD_SIZE - 1));
		float[] sampled = resample(terrain, progress);
		progress.accept(.25f, "Smoothing terrain");
		float[] heightmap = smoothDryRelief(sampled, QUAD_SIZE, scaleXZ);
		progress.accept(.4f, "Building terrain geometry");
		TerrainQuad quad = createGeometry(heightmap, PATCH_SIZE, QUAD_SIZE, scaleXZ);
		progress.accept(.6f, "Painting terrain surfaces");
		Texture2D coverage = generateAlphaMap(heightmap, QUAD_SIZE, scaleXZ, progress);
		progress.accept(.9f, "Preparing coastal materials");
		quad.setMaterial(ShorelineTerrain.islandMaterial(assetManager, coverage, scaleXZ));

		// Position: TerrainQuad is centred at origin. The sim terrain has
		// its origin at (originX, originY). We need to shift so the centre
		// of the quad matches the centre of the sim terrain.
		//
		// Sim coords: X=east, Y=north, Z=up
		// jME coords: X=east, Y=up, Z=-north (south)
		double worldH = terrain.worldHeight();
		float centreX = (float) (terrain.getOriginX() + worldW / 2);
		float centreZ = (float) -(terrain.getOriginY() + worldH / 2);
		quad.setLocalTranslation(centreX, 0, centreZ);

		// Keep the sampled surface used for tree/scenery grounding at every distance. Coarse
		// geomipmap levels can erase small islands beneath their fixed-height vegetation.
		// Terrain patches still use ordinary frustum culling; vegetation retains its own LOD.

		System.out.printf(
				"TerrainQuad: %dx%d resampled from %dx%d, scale=%.2f, " + "world=%.0fx%.0f, elev=[%.0f, %.0f]%n",
				QUAD_SIZE, QUAD_SIZE, terrain.getCols(), terrain.getRows(), scaleXZ, worldW, worldH,
				terrain.getMinElevation(), terrain.getMaxElevation());

		progress.accept(1f, "Terrain ready");
		return quad;
	}

	/**
	 * jME's terrain tri-planar shader samples patch-local positions and normals. Use the same unit
	 * on every local axis so vertical projections have the same physical tile size as horizontal
	 * projections, and the normals describe the actual slope. Uniform scaling restores metres.
	 */
	static TerrainQuad createGeometry(float[] heightmap, int patchSize, int quadSize, float cellSize) {
		float[] localHeights = new float[heightmap.length];
		for (int i = 0; i < heightmap.length; i++) {
			localHeights[i] = heightmap[i] / cellSize;
		}
		TerrainQuad quad = new TerrainQuad("terrain", patchSize, quadSize, localHeights);
		quad.setLocalScale(cellSize);
		return quad;
	}

	/**
	 * Resample the simulation terrain to a (QUAD_SIZE x QUAD_SIZE) float array using bilinear
	 * interpolation via {@link TerrainMap#elevationAt}.
	 */
	private static float[] resample(TerrainMap terrain, BiConsumer<Float, String> progress) {
		float[] heightmap = new float[QUAD_SIZE * QUAD_SIZE];

		double worldW = terrain.worldWidth();
		double worldH = terrain.worldHeight();
		double originX = terrain.getOriginX();
		double originY = terrain.getOriginY();

		for (int row = 0; row < QUAD_SIZE; row++) {
			// TerrainQuad row 0 corresponds to jME -Z = sim +Y (north).
			// Map rows to sim Y from north (originY + worldH) to south (originY).
			double simY = originY + worldH - row * (worldH / (QUAD_SIZE - 1));
			for (int col = 0; col < QUAD_SIZE; col++) {
				double simX = originX + col * (worldW / (QUAD_SIZE - 1));
				heightmap[row * QUAD_SIZE + col] = (float) terrain.elevationAt(simX, simY);
			}
			if (row % 32 == 0 || row == QUAD_SIZE - 1)
				progress.accept(.25f * (row + 1) / QUAD_SIZE, "Sampling terrain");
		}

		return heightmap;
	}

	/**
	 * Rounds small inland height spikes over a roughly 25m footprint. This is visual relief only:
	 * water, beaches and every vertex touching a water-bearing cell keep their original heights. A
	 * six-metre displacement bound retains headlands and larger hills; tree/scenery placement uses
	 * the resulting rendered surface.
	 */
	static float[] smoothDryRelief(float[] source, int size, float cellSize) {
		int radius = Math.max(1, (int) Math.ceil(24f / cellSize));
		float[] kernel = new float[radius * 2 + 1];
		float total = 0;
		for (int offset = -radius; offset <= radius; offset++) {
			kernel[offset + radius] = (float) Math.exp(-0.5 * Math.pow(offset * cellSize / 12f, 2));
			total += kernel[offset + radius];
		}
		for (int i = 0; i < kernel.length; i++)
			kernel[i] /= total;
		float[] horizontal = new float[source.length], result = source.clone();
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++) {
				int index = row * size + col;
				if (source[index] <= 12f) {
					horizontal[index] = source[index];
					continue;
				}
				for (int offset = -radius; offset <= radius; offset++) {
					int x = Math.max(0, Math.min(size - 1, col + offset));
					horizontal[index] += source[row * size + x] * kernel[offset + radius];
				}
			}
		}
		for (int row = 1; row < size - 1; row++) {
			for (int col = 1; col < size - 1; col++) {
				int index = row * size + col;
				if (source[index] <= 20f)
					continue;
				boolean coastal = false;
				for (int dz = -1; dz <= 1; dz++) {
					for (int dx = -1; dx <= 1; dx++)
						coastal |= source[index + dz * size + dx] <= 0f;
				}
				if (coastal)
					continue;
				float average = 0;
				for (int offset = -radius; offset <= radius; offset++) {
					int z = Math.max(0, Math.min(size - 1, row + offset));
					average += horizontal[z * size + col] * kernel[offset + radius];
				}
				float displacement = Math.max(-6f, Math.min(6f, average - source[index]));
				result[index] += displacement * smoothstep(20f, 40f, source[index]) * 0.8f;
			}
		}
		return result;
	}

	// ── Material and alpha maps ────────────────────────────────────────

	/**
	 * Create a PBR terrain material with four shared texture layers and procedural coverage derived
	 * from elevation and slope.
	 */
	static Material createMaterial(AssetManager assetManager, float[] heightmap, float cellSize) {
		int size = (int) Math.sqrt(heightmap.length);
		return ShorelineTerrain.islandMaterial(assetManager, generateAlphaMap(heightmap, size, cellSize), cellSize);
	}

	/**
	 * Generate an RGBA map for jME's ordered layer blending:
	 *
	 * <pre>
	 * R = 1 (dry sand base), G = forest coverage, B = exposed rock coverage, A = wet sand coverage
	 * </pre>
	 *
	 * AdvancedPBRTerrain mixes each successive layer over the result. Its channels are therefore
	 * coverages, not normalized weights. A fully covered base prevents dark seams, including when
	 * the map is filtered between biomes.
	 */
	static Texture2D generateAlphaMap(float[] heightmap, int size, float cellSize) {
		return generateAlphaMap(heightmap, size, cellSize, NO_PROGRESS);
	}

	private static Texture2D generateAlphaMap(
		float[] heightmap, int size, float cellSize, BiConsumer<Float, String> progress) {
		ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);

		for (int row = 0; row < size; row++) {
			// TerrainQuad UV (0,0) is at the bottom of the alpha map texture,
			// but our heightmap row 0 is the top (north). Flip vertically.
			int hRow = size - 1 - row;
			for (int col = 0; col < size; col++) {
				float elev = heightmap[hRow * size + col];

				// Slope from central finite differences
				int cl = Math.max(0, col - 1);
				int cr = Math.min(size - 1, col + 1);
				int ru = Math.max(0, hRow - 1);
				int rd = Math.min(size - 1, hRow + 1);
				float dzdx = (heightmap[hRow * size + cr] - heightmap[hRow * size + cl]) / ((cr - cl) * cellSize);
				float dzdy = (heightmap[rd * size + col] - heightmap[ru * size + col]) / ((rd - ru) * cellSize);
				float slopeDeg = (float) Math.toDegrees(Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy)));

				float x = (col - (size - 1) * 0.5f) * cellSize;
				float z = (hRow - (size - 1) * 0.5f) * cellSize;
				writeCoverage(buf, elev, slopeDeg, x, z);
			}
			if (row % 32 == 0 || row == size - 1)
				progress.accept(.6f + .3f * (row + 1) / size, "Painting terrain surfaces");
		}
		buf.flip();
		return bufferToTexture(buf, size);
	}

	private static void writeCoverage(ByteBuffer buf, float elev, float slopeDeg, float x, float z) {
		float variation = (float) SURFACE_NOISE.noise(x / 85f, z / 85f);
		float forest = smoothstep(10, 18, elev + variation * 2f);
		float sand = smoothstep(-20, -5, elev) * (1 - forest);

		// Sandy shallows and beaches remain flat; vegetation grows on steeper land.
		float removedSand = sand * smoothstep(8, 25, slopeDeg);
		sand -= removedSand;
		if (elev >= 0) {
			forest += removedSand;
		}

		if (elev > 0) {
			// Expose rock before the slope is too steep to support a canopy, avoiding bare green walls.
			float cliff = smoothstep(35, 60, slopeDeg);
			float peakExposure = smoothstep(100, 150, elev) * 0.3f;
			float biomeScale = 1 - Math.max(cliff, peakExposure);
			sand *= biomeScale;
			forest *= biomeScale;
			// Broad weathered pockets break up uninterrupted moss; detail tiles remain metre-sized.
			float weathering = smoothstep(0.10f, 0.45f, Math.abs(variation)) * 0.22f;
			forest *= 1 - weathering;
		}

		float rock = Math.max(0, 1 - sand - forest);
		float wetSand = sand * (1 - smoothstep(0.25f, 1.7f, elev));
		float drySand = sand - wetSand;
		// Invert the ordered GLSL mixes so wet sand never paints over cliff rock or forest.
		float forestCoverage = drySand + forest > 0 ? forest / (drySand + forest) : 0;
		float rockCoverage = wetSand < 1 ? rock / (1 - wetSand) : 0;
		buf.put((byte) 255).put(toByte(forestCoverage)).put(toByte(rockCoverage)).put(toByte(wetSand));
	}

	private static byte toByte(float coverage) {
		return (byte) Math.round(Math.max(0, Math.min(1, coverage)) * 255);
	}

	private static Texture2D bufferToTexture(ByteBuffer buf, int size) {
		Image img = new Image(Image.Format.RGBA8, size, size, buf, ColorSpace.Linear);
		Texture2D tex = new Texture2D(img);
		tex.setMinFilter(Texture.MinFilter.BilinearNoMipMaps);
		tex.setMagFilter(Texture.MagFilter.Bilinear);
		return tex;
	}

	/** Hermite smoothstep: 0 at lo, 1 at hi, smooth S-curve between. */
	private static float smoothstep(float lo, float hi, float val) {
		float t = Math.max(0, Math.min(1, (val - lo) / (hi - lo)));
		return t * t * (3 - 2 * t);
	}

}
