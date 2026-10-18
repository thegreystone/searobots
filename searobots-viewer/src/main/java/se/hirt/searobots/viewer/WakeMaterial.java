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
import com.jme3.material.RenderState;
import com.jme3.post.filters.FogFilter;
import com.jme3.water.WaterFilter;

/**
 * Cavitation particles drawn after the water filter. Their shader attenuates each fragment using
 * its own position, since the scene depth texture contains the seabed or sky behind a wake. A
 * material may be shared by every wake and updated once per frame, including while paused.
 */
public final class WakeMaterial {
	static final String MATERIAL_DEF = "Shaders/Effects/Wake.j3md";
	private static final float REFERENCE_TRANSPARENCY = 0.16f;
	private static final float SURFACE_FADE_DISTANCE = 60f;

	private WakeMaterial() {
	}

	public static Material create(AssetManager assets) {
		Material material = new Material(assets, MATERIAL_DEF);
		material.setTexture("Texture", assets.loadTexture("Effects/Explosion/smoketrail.png"));
		// Plain Additive ignores particle alpha, including the texture edge and lifetime fade.
		material.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
		material.getAdditionalRenderState().setDepthWrite(false);
		material.getAdditionalRenderState().setDepthTest(true);
		material.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
		return material;
	}

	/**
	 * Copies the current scene settings without modifying either filter. Disabled atmosphere clears
	 * the haze; a disabled water filter also removes surface attenuation for tactical views.
	 */
	public static void update(Material material, WaterFilter water, FogFilter fog, boolean atmosphereEnabled) {
		boolean waterVisible = water != null && water.isEnabled();
		material.setFloat("WaterHeight", water == null ? 0f : water.getWaterHeight());
		material.setFloat("SurfaceEnabled", waterVisible ? 1f : 0f);
		float opacity = waterVisible ? Math.max(0f, water.getWaterTransparency()) / REFERENCE_TRANSPARENCY : 0f;
		material.setFloat("SurfaceDensity", opacity / SURFACE_FADE_DISTANCE);
		float waterDensity = 0f;
		if (atmosphereEnabled && waterVisible) {
			waterDensity = Math.max(0f, water.getWaterTransparency())
					/ (REFERENCE_TRANSPARENCY * Math.max(1f, water.getUnderWaterFogDistance()));
		}
		material.setFloat("WaterDensity", waterDensity);
		float fogDensity = atmosphereEnabled && fog != null && fog.isEnabled()
				? Math.max(0f, fog.getFogDensity()) / Math.max(1f, fog.getFogDistance()) : 0f;
		material.setFloat("FogDensity", fogDensity);
	}
}
