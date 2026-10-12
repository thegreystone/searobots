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
import com.jme3.terrain.geomipmap.TerrainLodControl;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.geomipmap.lodcalc.DistanceLodCalculator;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;

import java.nio.ByteBuffer;

/**
 * Builds a jMonkeyEngine {@link TerrainQuad} from a {@link TerrainMap}, with LOD, lit texture
 * splatting via procedural alpha maps, normal maps, and tri-planar texturing for steep slopes.
 * <p>
 * Three surface materials are blended by elevation and slope:
 * <ol>
 * <li>Rock for the seabed and exposed cliffs</li>
 * <li>Beach / coastal sand</li>
 * <li>Forest / vegetation</li>
 * </ol>
 * <p>
 * TerrainQuad requires a (2^n + 1) square heightmap. The simulation's 1600x1600 grid is resampled
 * to 1025x1025 with bilinear interpolation.
 */
final class TerrainQuadBuilder {

	/** Terrain quad size: 1025 = 2^10 + 1. */
	private static final int QUAD_SIZE = 1025;
	/** Patch size for LOD tiles: 65 = 2^6 + 1. */
	private static final int PATCH_SIZE = 65;

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
	 *            camera for LOD control
	 * @return a TerrainQuad positioned and scaled to match world coordinates
	 */
	static TerrainQuad build(TerrainMap terrain, AssetManager assetManager, Camera camera) {
		float[] heightmap = resample(terrain);
		double worldW = terrain.worldWidth();
		float scaleXZ = (float) (worldW / (QUAD_SIZE - 1));
		TerrainQuad quad = createGeometry(heightmap, PATCH_SIZE, QUAD_SIZE, scaleXZ);
		quad.setMaterial(createMaterial(assetManager, heightmap, scaleXZ));

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

		// LOD control: reduce triangle count for distant patches.
		// High multiplier (5) keeps detail visible further from the camera,
		// preventing distant terrain from appearing flat.
		TerrainLodControl lodControl = new TerrainLodControl(quad, camera);
		lodControl.setLodCalculator(new DistanceLodCalculator(PATCH_SIZE, 5f));
		quad.addControl(lodControl);

		System.out.printf(
				"TerrainQuad: %dx%d resampled from %dx%d, scale=%.2f, " + "world=%.0fx%.0f, elev=[%.0f, %.0f]%n",
				QUAD_SIZE, QUAD_SIZE, terrain.getCols(), terrain.getRows(), scaleXZ, worldW, worldH,
				terrain.getMinElevation(), terrain.getMaxElevation());

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
	private static float[] resample(TerrainMap terrain) {
		float[] heightmap = new float[QUAD_SIZE * QUAD_SIZE];

		double worldW = terrain.worldWidth();
		double worldH = terrain.worldHeight();
		double originX = terrain.getOriginX();
		double originY = terrain.getOriginY();

		for (int row = 0; row < QUAD_SIZE; row++) {
			// TerrainQuad row 0 corresponds to jME -Z = sim +Y (north).
			// Map row 0..1024 to sim Y from north (originY + worldH) to south (originY).
			double simY = originY + worldH - row * (worldH / (QUAD_SIZE - 1));
			for (int col = 0; col < QUAD_SIZE; col++) {
				double simX = originX + col * (worldW / (QUAD_SIZE - 1));
				heightmap[row * QUAD_SIZE + col] = (float) terrain.elevationAt(simX, simY);
			}
		}

		return heightmap;
	}

	// ── Material and alpha maps ────────────────────────────────────────

	/**
	 * Create a lit terrain material with 3 texture layers blended by procedural coverage derived
	 * from elevation and slope.
	 */
	private static Material createMaterial(AssetManager assetManager, float[] heightmap, float cellSize) {
		Material mat = new Material(assetManager, "Common/MatDefs/Terrain/TerrainLighting.j3md");
		mat.setBoolean("useTriPlanarMapping", true);

		// No specular: avoids washed-out highlights from strong sun
		mat.setFloat("Shininess", 0f);

		mat.setTexture("AlphaMap", generateAlphaMap(heightmap, QUAD_SIZE, cellSize));

		// Tri-planar scales are repeats per local grid unit, not UV repeats across the whole terrain.
		// Keep ground detail at believable scales (~17m rock, 11m sand, 27m vegetation on the
		// 17km map) on vertical faces too. Every 64-cell patch contains an integer number of
		// tiles, keeping patch-local projections continuous across patch boundaries.
		// Base rock covers all depths and exposed cliffs; water and fog darken the deep seabed.
		mat.setTexture("DiffuseMap", loadWrap(assetManager, "Textures/Terrain/PBR/Gravel015_1K_Color.png"));
		mat.setTexture("NormalMap", loadWrap(assetManager, "Textures/Terrain/PBR/Gravel015_1K_Normal.png"));
		mat.setFloat("DiffuseMap_0_scale", 1024f / (QUAD_SIZE - 1));

		// First overlay (AlphaMap.G): beach sand.
		mat.setTexture("DiffuseMap_1", loadWrap(assetManager, "Textures/Terrain/PBR/Ground037_1K_Color.png"));
		mat.setTexture("NormalMap_1", loadWrap(assetManager, "Textures/Terrain/PBR/Ground037_1K_Normal.png"));
		mat.setFloat("DiffuseMap_1_scale", 1536f / (QUAD_SIZE - 1));

		// Second overlay (AlphaMap.B): lush green vegetation.
		mat.setTexture("DiffuseMap_2", loadWrap(assetManager, "Textures/Terrain/custom/vegetation.png"));
		mat.setTexture("NormalMap_2", loadWrap(assetManager, "Textures/Terrain/custom/vegetation_normal.png"));
		mat.setFloat("DiffuseMap_2_scale", 640f / (QUAD_SIZE - 1));

		return mat;
	}

	/**
	 * Generate an RGBA map for jME's ordered layer blending:
	 *
	 * <pre>
	 * R = 1 (full base rock), G = sand coverage before vegetation, B = vegetation coverage, A = 0
	 * </pre>
	 *
	 * TerrainLighting multiplies the base by R, then mixes each later layer over the result. Its
	 * channels are therefore coverages, not normalized weights. A fully covered base prevents dark
	 * seams, including when the map is filtered between biomes.
	 */
	static Texture2D generateAlphaMap(float[] heightmap, int size, float cellSize) {
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

				writeCoverage(buf, elev, slopeDeg);
			}
		}
		buf.flip();
		return bufferToTexture(buf, size);
	}

	private static void writeCoverage(ByteBuffer buf, float elev, float slopeDeg) {
		float forest = smoothstep(10, 13, elev);
		float sand = smoothstep(-20, -5, elev) * (1 - forest);

		// Sandy shallows and beaches remain flat; vegetation grows on steeper land.
		float removedSand = sand * smoothstep(8, 25, slopeDeg);
		sand -= removedSand;
		if (elev >= 0) {
			forest += removedSand;
		}

		if (elev > 0) {
			// Vegetation clings to slopes up to 50 degrees, with exposed rock on steep cliffs.
			float cliff = smoothstep(50, 70, slopeDeg);
			float peakExposure = smoothstep(100, 150, elev) * 0.3f;
			float biomeScale = 1 - Math.max(cliff, peakExposure);
			sand *= biomeScale;
			forest *= biomeScale;
		}

		// mix(mix(rock, sand, G), forest, B) has final sand weight G * (1 - B).
		// Invert that equation rather than writing normalized weights into the coverage map.
		float sandCoverage = forest < 1 ? sand / (1 - forest) : 0;
		buf.put((byte) 255).put(toByte(sandCoverage)).put(toByte(forest)).put((byte) 0);
	}

	private static byte toByte(float coverage) {
		return (byte) Math.round(Math.max(0, Math.min(1, coverage)) * 255);
	}

	private static Texture2D bufferToTexture(ByteBuffer buf, int size) {
		Image img = new Image(Image.Format.RGBA8, size, size, buf, ColorSpace.Linear);
		Texture2D tex = new Texture2D(img);
		tex.setMinFilter(Texture.MinFilter.BilinearNearestMipMap);
		tex.setMagFilter(Texture.MagFilter.Bilinear);
		return tex;
	}

	/** Hermite smoothstep: 0 at lo, 1 at hi, smooth S-curve between. */
	private static float smoothstep(float lo, float hi, float val) {
		float t = Math.max(0, Math.min(1, (val - lo) / (hi - lo)));
		return t * t * (3 - 2 * t);
	}

	private static Texture loadWrap(AssetManager am, String path) {
		Texture t = am.loadTexture(path);
		t.setWrap(Texture.WrapMode.Repeat);
		t.setMinFilter(Texture.MinFilter.Trilinear);
		t.setMagFilter(Texture.MagFilter.Bilinear);
		t.setAnisotropicFilter(8);
		return t;
	}
}
