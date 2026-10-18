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

import com.jme3.asset.DesktopAssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.post.filters.FogFilter;
import com.jme3.system.JmeSystem;
import com.jme3.water.WaterFilter;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class WakeMaterialTest {
	@Test
	void usesAlphaFadingAndSceneOcclusionWithoutWritingParticleQuadDepth() {
		Material material = material();
		assertEquals(WakeMaterial.MATERIAL_DEF, material.getMaterialDef().getAssetName());
		assertEquals(RenderState.BlendMode.AlphaAdditive, material.getAdditionalRenderState().getBlendMode());
		assertTrue(material.getAdditionalRenderState().isDepthTest());
		assertFalse(material.getAdditionalRenderState().isDepthWrite());
		assertEquals(RenderState.FaceCullMode.Off, material.getAdditionalRenderState().getFaceCullMode());
		assertNotNull(material.getTextureParam("Texture"));
	}

	@Test
	void followsWaterPanelSettingsAndAtmosphereWithoutChangingSavedSettings() {
		Material material = material();
		WaterFilter water = new WaterFilter();
		water.setUnderWaterFogDistance(400f);
		water.setWaterTransparency(0.16f);
		water.setWaterHeight(7f);
		FogFilter fog = new FogFilter();
		fog.setFogDensity(1.5f);
		fog.setFogDistance(2000f);

		WakeMaterial.update(material, water, fog, true);
		assertEquals(1f / 400f, value(material, "WaterDensity"), 1e-7f);
		assertEquals(1.5f / 2000f, value(material, "FogDensity"), 1e-7f);
		assertEquals(7f, value(material, "WaterHeight"));
		assertEquals(1f / 60f, value(material, "SurfaceDensity"), 1e-7f);
		assertEquals(1f, value(material, "SurfaceEnabled"));

		WakeMaterial.update(material, water, fog, false);
		assertEquals(0f, value(material, "WaterDensity"));
		assertEquals(0f, value(material, "FogDensity"));
		assertEquals(1f, value(material, "SurfaceEnabled"), "The water surface remains in atmosphere-off views");
		assertEquals(1f / 60f, value(material, "SurfaceDensity"), 1e-7f);
		assertEquals(400f, water.getUnderWaterFogDistance());
		assertEquals(0.16f, water.getWaterTransparency());

		water.setUnderWaterFogDistance(200f);
		WakeMaterial.update(material, water, fog, true);
		assertEquals(1f / 200f, value(material, "WaterDensity"), 1e-7f);
	}

	@Test
	void tacticalViewsClearWaterAndFogAttenuationIndependently() {
		Material material = material();
		WaterFilter water = new WaterFilter();
		FogFilter fog = new FogFilter();

		water.setEnabled(false);
		WakeMaterial.update(material, water, fog, true);
		assertEquals(0f, value(material, "WaterDensity"));
		assertEquals(0f, value(material, "SurfaceEnabled"));
		assertEquals(0f, value(material, "SurfaceDensity"));
		assertTrue(value(material, "FogDensity") > 0f);

		fog.setEnabled(false);
		WakeMaterial.update(material, water, fog, true);
		assertEquals(0f, value(material, "FogDensity"));

		assertDoesNotThrow(() -> WakeMaterial.update(material, null, null, true));
		assertEquals(0f, value(material, "WaterDensity"));
		assertEquals(0f, value(material, "FogDensity"));
	}

	private static Material material() {
		return WakeMaterial.create(new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL()));
	}

	private static float value(Material material, String name) {
		return (Float) material.getParam(name).getValue();
	}
}
