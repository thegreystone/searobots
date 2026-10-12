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

import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture2D;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TerrainQuadBuilderTest {

	private static final double QUANTIZATION_TOLERANCE = 1.0 / 255;

	@Test
	void bundledTerrainShadersUseOrderedCoverage() throws IOException {
		// Pin the convention of the dependency we actually render with. The output checks below
		// apply its base multiply and successive GLSL mix operations, rather than a weighted sum.
		for (String shader : new String[] {"TerrainLighting.frag", "SPTerrainLighting.frag"}) {
			String source = shaderSource(shader);
			assertTrue(source.contains("vec4 diffuseColor = tex0 * alphaBlend.r;"), shader);
			assertTrue(source.contains("diffuseColor = mix( diffuseColor, tex1, alphaBlend.g );"), shader);
			assertTrue(source.contains("diffuseColor = mix( diffuseColor, tex2, alphaBlend.b );"), shader);
		}
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
		assertSurfaceShares(11.5f, 0, 0, 0.5, 0.5); // Halfway through the treeline.
		assertSurfaceShares(50, 0, 0, 0, 1); // Forest.
		assertSurfaceShares(50, 60, 0.5, 0, 0.5); // Vegetation gives way to cliff rock.
		assertSurfaceShares(50, 70, 1, 0, 0); // Exposed cliff.
		assertSurfaceShares(150, 0, 0.3, 0, 0.7); // Peak rock exposure.
	}

	@Test
	void filteredCoverageDoesNotIntroduceDarkBoundaryBands() {
		float[] shallows = coverageAt(-12.5f, 0);
		float[] forest = coverageAt(50, 0);
		for (int step = 0; step <= 100; step++) {
			float t = step / 100f;
			float[] filtered = new float[3];
			for (int channel = 0; channel < filtered.length; channel++) {
				filtered[channel] = shallows[channel] * (1 - t) + forest[channel] * t;
			}
			assertEquals(1, shaderBlend(1, 1, 1, filtered), 0.000001);
		}
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
		return new float[] {(data.get(16) & 0xff) / 255f, (data.get(17) & 0xff) / 255f, (data.get(18) & 0xff) / 255f};
	}

	private static double shaderBlend(float rock, float sand, float forest, float[] coverage) {
		double result = rock * coverage[0];
		result = result * (1 - coverage[1]) + sand * coverage[1];
		return result * (1 - coverage[2]) + forest * coverage[2];
	}

	private static String shaderSource(String name) throws IOException {
		try (InputStream stream = TerrainQuadBuilderTest.class.getResourceAsStream("/Common/MatDefs/Terrain/" + name)) {
			assertNotNull(stream);
			return new String(stream.readAllBytes(), StandardCharsets.UTF_8);
		}
	}
}
