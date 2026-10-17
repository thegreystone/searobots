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

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

import com.jme3.asset.AssetManager;
import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.material.Material;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.texture.Image;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AtmosphereWaterFilterTest {
	private static final String STOCK_MATERIAL_DEF = "Common/MatDefs/Water/Water.j3md";

	@Test
	void defaultsToClearWaterAndPreservesWaterSettingsAcrossRepeatedAtmosphereToggles() {
		TestWaterFilter water = new TestWaterFilter();
		assertNull(water.renderMaterial(), "Construction must not require an initialized render context");
		water.setUnderWaterFogDistance(275f);
		Material material = water.installMaterial();
		water.setWaveScale(0.0073f);
		water.setCausticsIntensity(0.72f);

		assertSame(material, water.renderMaterial());
		assertEquals(100_000f, uniform(material, "UnderWaterFogDistance"));
		assertEquals(275f, water.getUnderWaterFogDistance());
		for (boolean enabled : new boolean[] {true, false, true, false}) {
			water.setFogEnabled(enabled);
			assertSame(material, water.renderMaterial());
			assertEquals(enabled ? 275f : 100_000f, uniform(material, "UnderWaterFogDistance"));
			assertEquals(275f, water.getUnderWaterFogDistance(), "The panel's fog-distance setting must remain intact");
			assertEquals(0.0073f, water.getWaveScale());
			assertEquals(0.0073f, uniform(material, "WaveScale"));
			assertEquals(0.72f, water.getCausticsIntensity());
			assertEquals(0.72f, uniform(material, "CausticsIntensity"));
		}
	}

	@Test
	void restoresTheLatestFogDistanceEditedWhileAtmosphereIsOff() {
		TestWaterFilter water = new TestWaterFilter();
		water.setFogEnabled(false);
		water.setUnderWaterFogDistance(180f);
		Material material = water.installMaterial();
		assertEquals(100_000f, uniform(water.renderMaterial(), "UnderWaterFogDistance"));

		water.setUnderWaterFogDistance(420f);
		assertEquals(420f, water.getUnderWaterFogDistance());
		assertEquals(100_000f, uniform(water.renderMaterial(), "UnderWaterFogDistance"));
		water.setFogEnabled(true);
		assertSame(material, water.renderMaterial());
		assertEquals(420f, uniform(material, "UnderWaterFogDistance"));
		water.setFogEnabled(false);
		assertEquals(100_000f, uniform(water.renderMaterial(), "UnderWaterFogDistance"));
		assertEquals(420f, water.getUnderWaterFogDistance());
	}

	@Test
	void rendersWithTheSurfaceAwareCausticsShaderKeepingEveryStockParameter() throws IOException {
		AssetManager assets = assets();
		Material stock = new Material(assets, STOCK_MATERIAL_DEF);
		Texture2D caustics = new Texture2D(
				new Image(Image.Format.RGB8, 1, 1, BufferUtils.createByteBuffer(3), ColorSpace.sRGB));
		stock.setTexture("CausticsMap", caustics);
		stock.setBoolean("UseCaustics", true);
		stock.setFloat("CausticsIntensity", 0.3f);
		stock.setVector3("LightDir", new Vector3f(0.2f, -0.9f, 0.1f));
		stock.setColor("WaterColor", new ColorRGBA(0.1f, 0.2f, 0.3f, 1f));

		Material water = AtmosphereWaterFilter.withSurfaceAwareCaustics(assets, stock);

		assertEquals(AtmosphereWaterFilter.MATERIAL_DEF, water.getMaterialDef().getAssetName());
		assertEquals(stock.getParams().size(), water.getParams().size());
		assertSame(caustics, water.getTextureParam("CausticsMap").getTextureValue());
		assertEquals(true, water.getParam("UseCaustics").getValue());
		assertEquals(0.3f, uniform(water, "CausticsIntensity"));
		assertEquals(new Vector3f(0.2f, -0.9f, 0.1f), water.getParam("LightDir").getValue());
		assertEquals(new ColorRGBA(0.1f, 0.2f, 0.3f, 1f), water.getParam("WaterColor").getValue());
		// The stock definition and ours expose the same uniforms, so every filter setter still applies.
		assertEquals(stock.getMaterialDef().getMaterialParams().size(),
				water.getMaterialDef().getMaterialParams().size());

		String shader = resource("/Shaders/Water/CausticWater.frag");
		assertTrue(shader.contains("refract(normalize(m_LightDir)"), "caustics must follow the refracted sun");
		assertTrue(shader.contains("m_CausticsIntensity * facingSun"), "caustics must be weighted by facing");
		assertTrue(resource("/Shaders/Water/CausticWater.j3md").contains("Shaders/Water/CausticWater.frag"));
	}

	private static float uniform(Material material, String name) {
		return (Float) material.getParam(name).getValue();
	}

	private static String resource(String path) throws IOException {
		try (InputStream in = AtmosphereWaterFilterTest.class.getResourceAsStream(path)) {
			assertNotNull(in, path + " must be on the classpath");
			return new String(in.readAllBytes(), StandardCharsets.UTF_8);
		}
	}

	private static AssetManager assets() {
		var assets = new DesktopAssetManager(false);
		assets.registerLoader(J3MLoader.class, "j3m", "j3md");
		assets.registerLocator("/", ClasspathLocator.class);
		return assets;
	}

	private static final class TestWaterFilter extends AtmosphereWaterFilter {
		TestWaterFilter() {
			super(new Node("reflection scene"), new Vector3f(0f, -1f, 0f));
		}

		/**
		 * Installs the material as {@code initFilter} does: the stock one, rebuilt on our
		 * definition.
		 */
		Material installMaterial() {
			AssetManager assets = assets();
			material = withSurfaceAwareCaustics(assets, new Material(assets, STOCK_MATERIAL_DEF));
			return material;
		}

		Material renderMaterial() {
			return getMaterial();
		}
	}
}
