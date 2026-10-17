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

import java.io.IOException;
import java.nio.charset.StandardCharsets;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.material.MatParamTexture;
import com.jme3.material.MaterialDef;
import com.jme3.material.TechniqueDef;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ShorelineMaterialsTest {

	@Test
	void terrainShaderOverrideKeepsSharedDefinitionAndShadowTechniquesIntact() {
		var assets = new DesktopAssetManager(true);
		var stock = (MaterialDef) assets.loadAsset("Common/MatDefs/Terrain/AdvancedPBRTerrain.j3md");
		var custom = ShorelineMaterials.terrain(assets).getMaterialDef();
		assertNotSame(stock, custom);
		assertSame(stock, assets.loadAsset("Common/MatDefs/Terrain/AdvancedPBRTerrain.j3md"));
		assertEquals(stock.getMaterialParams().size() + 3, custom.getMaterialParams().size());
		assertNull(stock.getMaterialParam("SeabedAlbedoScale"));
		assertEquals(1f, custom.getMaterialParam("SeabedAlbedoScale").getValue());
		assertNull(stock.getMaterialParam("SeabedGravel"));
		assertEquals(false, custom.getMaterialParam("SeabedGravel").getValue());
		for (var parameter : stock.getMaterialParams()) {
			var copy = custom.getMaterialParam(parameter.getName());
			assertNotSame(parameter, copy);
			assertEquals(parameter.getVarType(), copy.getVarType());
			assertEquals(parameter.getValue(), copy.getValue());
			if (parameter instanceof MatParamTexture texture)
				assertEquals(texture.getColorSpace(), ((MatParamTexture) copy).getColorSpace());
		}
		for (String name : stock.getTechniqueDefsNames()) {
			var originals = stock.getTechniqueDefs(name);
			var copies = custom.getTechniqueDefs(name);
			assertEquals(originals.size(), copies.size());
			for (int i = 0; i < originals.size(); i++) {
				var original = originals.get(i);
				var copy = copies.get(i);
				assertNotSame(original, copy);
				assertEquals(original.getVertexShaderName(), copy.getVertexShaderName());
				assertEquals(original.getVertexShaderLanguage(), copy.getVertexShaderLanguage());
				assertEquals(original.getFragmentShaderLanguage(), copy.getFragmentShaderLanguage());
				assertEquals(original.getLightMode(), copy.getLightMode());
				assertNull(original.getShaderParamDefine("SeabedGravel"));
				if (TechniqueDef.DEFAULT_TECHNIQUE_NAME.equals(name)) {
					assertEquals("SEABED_GRAVEL", copy.getShaderParamDefine("SeabedGravel"));
					assertEquals("Common/MatDefs/Terrain/AdvancedPBRTerrain.frag", original.getFragmentShaderName());
					assertEquals("Shaders/Shoreline/AdvancedPBRTerrain.frag", copy.getFragmentShaderName());
				} else {
					assertEquals(original.getFragmentShaderName(), copy.getFragmentShaderName());
					assertEquals(original.getForcedRenderState(), copy.getForcedRenderState());
				}
			}
		}
	}

	@Test
	void terrainShaderInitializesLightingStateBeforeAccumulation() throws IOException {
		try (var resource = getClass().getResourceAsStream("/Shaders/Shoreline/AdvancedPBRTerrain.frag")) {
			assertNotNull(resource);
			String shader = new String(resource.readAllBytes(), StandardCharsets.UTF_8);
			int directLighting = shader.indexOf("PBRLightingUtils_computeDirectLightContribution(");
			for (String initialization : new String[] {"surface.bakedLightContribution = vec3(0.0);",
					"surface.directLightContribution = vec3(0.0);", "surface.envLightContribution = vec3(0.0);",
					"surface.brightestNonGlobalLightStrength = 0.0;",
					"surface.NdotV = clamp(abs(dot(surface.normal, surface.viewDir)), 0.001, 1.0);"}) {
				int initialized = shader.indexOf(initialization);
				assertTrue(initialized >= 0 && initialized < directLighting, initialization);
			}
		}
	}
}
