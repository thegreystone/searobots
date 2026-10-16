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
import com.jme3.material.MatParamTexture;
import com.jme3.material.Material;
import com.jme3.material.MaterialDef;
import com.jme3.material.TechniqueDef;
import com.jme3.shader.VarType;

/** Creates shoreline materials without changing jME's shared asset definitions. */
public final class ShorelineMaterials {

	private static final String STOCK_TERRAIN = "Common/MatDefs/Terrain/AdvancedPBRTerrain.j3md";
	private static final String TERRAIN_FRAGMENT = "Shaders/Shoreline/AdvancedPBRTerrain.frag";

	private ShorelineMaterials() {
	}

	/**
	 * Keeps jME's PBR terrain parameters and shadow techniques, with initialized lighting state.
	 */
	public static Material terrain(AssetManager assets) {
		MaterialDef stock = (MaterialDef) assets.loadAsset(STOCK_TERRAIN);
		MaterialDef definition = new MaterialDef(assets, "Shoreline PBR Terrain");
		for (var parameter : stock.getMaterialParams()) {
			if (parameter instanceof MatParamTexture texture) {
				definition.addMaterialParamTexture(texture.getVarType(), texture.getName(), texture.getColorSpace(),
						texture.getTextureValue());
			} else {
				definition.addMaterialParam(parameter.getVarType(), parameter.getName(), parameter.getValue());
			}
		}
		definition.addMaterialParam(VarType.Float, "SeabedAlbedoScale", 1f);
		definition.addMaterialParam(VarType.Boolean, "SeabedGravel", false);
		definition.addMaterialParam(VarType.Float, "SeabedMapScale", 1f);
		for (String name : stock.getTechniqueDefsNames()) {
			for (TechniqueDef original : stock.getTechniqueDefs(name)) {
				try {
					TechniqueDef technique = original.clone();
					if (TechniqueDef.DEFAULT_TECHNIQUE_NAME.equals(name)) {
						technique.addShaderParamDefine("SeabedGravel", VarType.Boolean, "SEABED_GRAVEL");
						technique.setShaderFile(original.getVertexShaderName(), TERRAIN_FRAGMENT,
								original.getVertexShaderLanguage(), original.getFragmentShaderLanguage());
					}
					definition.addTechniqueDef(technique);
				} catch (CloneNotSupportedException e) {
					throw new IllegalStateException("Cannot isolate shoreline terrain material", e);
				}
			}
		}
		return new Material(definition);
	}
}
