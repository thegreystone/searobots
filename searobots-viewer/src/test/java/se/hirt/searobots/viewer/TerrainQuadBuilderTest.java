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
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package se.hirt.searobots.viewer;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.viewer.shoreline.ShorelineTerrain;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TerrainQuadBuilderTest {

	private static final double QUANTIZATION_TOLERANCE = 1.0 / 255;

	@Test
	void islandMaterialSharesCoastalAndSeabedPbrSurfacesAtMetreScale() {
		var assets = new DesktopAssetManager(true);
		var material = TerrainQuadBuilder.createMaterial(assets, new float[9], 16f);
		var color = material.getTextureParam("AlbedoTextureArray").getTextureValue().getImage();
		var normal = material.getTextureParam("NormalParallaxTextureArray").getTextureValue().getImage();
		var packed = material.getTextureParam("MetallicRoughnessAoEiTextureArray").getTextureValue().getImage();
		assertEquals(5, color.getData().size());
		assertEquals(4, normal.getData().size());
		assertEquals(4, packed.getData().size());
		assertEquals(ColorSpace.sRGB, color.getColorSpace());
		assertEquals(ColorSpace.Linear, normal.getColorSpace());
		assertTrue(color.isGeneratedMipmapsRequired());
		assertTrue(normal.isGeneratedMipmapsRequired());
		ByteBuffer colorPixels = color.getData(0), normalPixels = normal.getData(0);
		for (int pixel = 0; pixel < colorPixels.limit(); pixel += 4 * 127) {
			// A beach must have warm sand grain, rather than the green moss previously bound here.
			assertTrue((colorPixels.get(pixel) & 255) > (colorPixels.get(pixel + 1) & 255));
			assertTrue((colorPixels.get(pixel + 1) & 255) > (colorPixels.get(pixel + 2) & 255));
			float x = (normalPixels.get(pixel) & 255) / 127.5f - 1f;
			float y = (normalPixels.get(pixel + 1) & 255) / 127.5f - 1f;
			float z = (normalPixels.get(pixel + 2) & 255) / 127.5f - 1f;
			assertEquals(1f, Math.sqrt(x * x + y * y + z * z), 0.01);
		}
		// The array uses DirectX normals, decoded by NormalType=-1 in AdvancedPBRTerrain.
		assertTrue((normalPixels.get(1) & 255) > 127);
		assertEquals(-1f, material.getParam("NormalType").getValue());
		assertEquals(ColorSpace.Linear, packed.getColorSpace());
		var rebuilt = ShorelineTerrain.islandMaterial(assets,
				(Texture2D) material.getTextureParam("AlphaMap").getTextureValue(), 16f);
		assertSame(color, rebuilt.getTextureParam("AlbedoTextureArray").getTextureValue().getImage());
		assertSame(normal, rebuilt.getTextureParam("NormalParallaxTextureArray").getTextureValue().getImage());
		for (int layer = 0; layer < 4; layer++) {
			float scale = (float) material.getParam("AlbedoMap_" + layer + "_scale").getValue();
			assertEquals(layer == 0 || layer == 3 ? 2f : 4f, 16f / scale, 0.02);
			assertEquals(Math.round(scale * 64f), scale * 64f, 0.0001);
		}
		float seabedScale = (float) material.getParam("SeabedMapScale").getValue();
		assertEquals(6f, 16f / seabedScale, 0.02);
		assertEquals(Math.round(seabedScale * 64f), seabedScale * 64f, 0.0001);
		ByteBuffer wet = color.getData(3);
		assertTrue((wet.get(0) & 255) < (colorPixels.get(0) & 255));
		assertTrue((packed.getData(0).get(1) & 255) > 180); // Rough, nonmetallic dry sand.
		assertEquals(0, packed.getData(0).get(2));
		assertTrue((float) material.getParam("Roughness_3").getValue() < (float) material.getParam("Roughness_0")
				.getValue());
	}

	@Test
	void bundledTerrainShadersUseOrderedCoverage() throws IOException {
		String source = shaderSource("Modular/PBRTerrainUtils.glsllib");
		assertTrue(source.contains("surface.albedo = mix(surface.albedo, layer.albedo.rgb, layer.blendValue);"));
	}

	@Test
	void matchingSurfaceColorsStayBrightAcrossBiomesAndSlopes() {
		for (float elevation : new float[] {-500, -300, -225, -150, -20, -12.5f, -5, 0, 5, 10, 11.5f, 13, 50, 125,
				150}) {
			for (float slope : new float[] {0, 8, 16.5f, 25, 50, 60, 70}) {
				float[] coverage = coverageAt(elevation, slope);
				assertEquals(1, shaderBlend(1, 1, 1, coverage), 0.000001,
						"A transition must not darken identical textures at " + elevation + "m / " + slope
								+ " degrees");
			}
		}
	}

	@Test
	void differentSurfaceColorsRetainTheirIntendedShares() {
		assertSurfaceShares(-12.5f, 0, 0.5, 0.5, 0); // Seabed to sandy shallows.
		assertSurfaceShares(0, 0, 0, 1, 0); // Beach.
		assertSurfaceShares(14, 0, 0, 0.5, 0.5); // Halfway through the treeline.
		assertSurfaceShares(50, 0, 0, 0, 1); // Forest.
		assertSurfaceShares(50, 47.5f, 0.5, 0, 0.5); // Vegetation gives way to cliff rock.
		assertSurfaceShares(50, 60, 1, 0, 0); // Exposed cliff.
		assertSurfaceShares(150, 0, 0.3, 0, 0.7); // Peak rock exposure.
	}

	@Test
	void weatheredGroundVariesAcrossFlatLandWithoutChangingBeachCoverage() {
		int size = 65;
		float[] flat = new float[size * size];
		java.util.Arrays.fill(flat, 40f);
		ByteBuffer land = TerrainQuadBuilder.generateAlphaMap(flat, size, 8f).getImage().getData(0);
		int minimum = 255, maximum = 0;
		for (int i = 2; i < land.limit(); i += 4) {
			minimum = Math.min(minimum, land.get(i) & 255);
			maximum = Math.max(maximum, land.get(i) & 255);
		}
		assertTrue(maximum - minimum > 20, "Large weathered pockets must interrupt a uniform green hillside");
		java.util.Arrays.fill(flat, 4f);
		ByteBuffer beach = TerrainQuadBuilder.generateAlphaMap(flat, size, 8f).getImage().getData(0);
		for (int i = 0; i < beach.limit(); i += 4) {
			assertEquals(255, beach.get(i) & 255);
			assertEquals(0, beach.get(i + 1) & 255);
			assertEquals(0, beach.get(i + 2) & 255);
			assertEquals(0, beach.get(i + 3) & 255);
		}
	}

	@Test
	void filteredCoverageDoesNotIntroduceDarkBoundaryBands() {
		float[] shallows = coverageAt(-12.5f, 0);
		float[] forest = coverageAt(50, 0);
		for (int step = 0; step <= 100; step++) {
			float t = step / 100f;
			float[] filtered = new float[4];
			for (int channel = 0; channel < filtered.length; channel++) {
				filtered[channel] = shallows[channel] * (1 - t) + forest[channel] * t;
			}
			assertEquals(1, shaderBlend(1, 1, 1, filtered), 0.000001);
		}
	}

	@Test
	void wetSandStaysAtTheWaterlineAndDoesNotReplaceCliffRock() {
		assertEquals(1, shaderBlend(0, 0, 0, 1, coverageAt(0, 0)), QUANTIZATION_TOLERANCE);
		assertEquals(0.5, shaderBlend(0, 0, 0, 1, coverageAt(0.975f, 0)), QUANTIZATION_TOLERANCE);
		assertEquals(0, shaderBlend(0, 0, 0, 1, coverageAt(5, 0)), QUANTIZATION_TOLERANCE);
		assertEquals(0, shaderBlend(0, 0, 0, 1, coverageAt(0, 70)), QUANTIZATION_TOLERANCE);
		assertEquals(1, shaderBlend(1, 0, 0, 0, coverageAt(-500, 0)), QUANTIZATION_TOLERANCE);
	}

	@Test
	void uniformLocalUnitsPreserveWorldHeightsAndSlopeNormals() {
		int size = 9;
		float cellSize = 20;
		float[] heightmap = new float[size * size];
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++) {
				heightmap[row * size + col] = 200 + (col - 4) * cellSize;
			}
		}
		TerrainQuad terrain = TerrainQuadBuilder.createGeometry(heightmap, 5, size, cellSize);
		terrain.setLocalTranslation(1000, 0, -2000);
		terrain.updateGeometricState();

		// Include a fractional grid position, as the tree placement code queries interpolated height.
		Vector2f location = new Vector2f(1017, -2011);
		assertEquals(217, terrain.getHeight(location), 0.001);
		assertEquals(140, terrain.getHeight(new Vector2f(940, -2040)), 0.001);
		Vector3f normal = terrain.getNormal(location);
		assertEquals(-Math.sqrt(0.5), normal.x, 0.0001);
		assertEquals(Math.sqrt(0.5), normal.y, 0.0001);
		assertEquals(0, normal.z, 0.0001);
	}

	@Test
	void visualSmoothingRoundsInlandSpikesAndPreservesEveryWaterBearingCell() {
		int size = 17;
		float[] source = new float[size * size];
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++)
				source[row * size + col] = col < 4 ? -10f : col == 4 ? 30f : col < 8 ? 10f : 50f;
		}
		source[8 * size + 12] = 66f;
		float[] untouched = source.clone();
		float[] rendered = TerrainQuadBuilder.smoothDryRelief(source, size, 8f);
		org.junit.jupiter.api.Assertions.assertArrayEquals(untouched, source);
		assertTrue(rendered[8 * size + 12] < 63f, "A pointed inland spike must become a rounder ridge");
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++) {
				int i = row * size + col;
				if (col <= 7 || row == 0 || row == size - 1 || col == size - 1)
					assertEquals(source[i], rendered[i], "Water, coast corners, beaches and map edges stay exact");
				assertTrue(Math.abs(rendered[i] - source[i]) <= 4.801f);
			}
		}
	}

	private static void assertSurfaceShares(float elevation, float slope, double rock, double sand, double forest) {
		float[] coverage = coverageAt(elevation, slope);
		// Three distinct primary colors expose the rendered share of each material independently.
		assertEquals(rock, shaderBlend(1, 0, 0, coverage), QUANTIZATION_TOLERANCE);
		assertEquals(sand, shaderBlend(0, 1, 0, coverage), QUANTIZATION_TOLERANCE);
		assertEquals(forest, shaderBlend(0, 0, 1, coverage), QUANTIZATION_TOLERANCE);
	}

	private static float[] coverageAt(float elevation, float slopeDegrees) {
		float rise = (float) Math.tan(Math.toRadians(slopeDegrees));
		float[] heightmap = new float[9];
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				heightmap[row * 3 + col] = elevation + (col - 1) * rise;
			}
		}
		Texture2D alphaMap = TerrainQuadBuilder.generateAlphaMap(heightmap, 3, 1);
		ByteBuffer data = alphaMap.getImage().getData(0);
		return new float[] {(data.get(16) & 0xff) / 255f, (data.get(17) & 0xff) / 255f, (data.get(18) & 0xff) / 255f,
				(data.get(19) & 0xff) / 255f};
	}

	private static double shaderBlend(float rock, float sand, float forest, float[] coverage) {
		return shaderBlend(rock, sand, forest, sand, coverage);
	}

	private static double shaderBlend(float rock, float drySand, float forest, float wetSand, float[] coverage) {
		double result = drySand * coverage[0];
		result = result * (1 - coverage[1]) + forest * coverage[1];
		result = result * (1 - coverage[2]) + rock * coverage[2];
		return result * (1 - coverage[3]) + wetSand * coverage[3];
	}

	private static String shaderSource(String name) throws IOException {
		try (InputStream stream = TerrainQuadBuilderTest.class.getResourceAsStream("/Common/MatDefs/Terrain/" + name)) {
			assertNotNull(stream);
			return new String(stream.readAllBytes(), StandardCharsets.UTF_8);
		}
	}
}
