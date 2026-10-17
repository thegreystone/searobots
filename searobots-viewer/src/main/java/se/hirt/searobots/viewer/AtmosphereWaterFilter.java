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
import com.jme3.material.MatParam;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;
import com.jme3.water.WaterFilter;

/**
 * The stock water filter with two changes. The atmosphere toggle switches underwater fog off
 * without touching the water panel's saved distance. Rendering uses {@value #MATERIAL_DEF}, the
 * stock water shader with caustics that fall only on surfaces facing the refracted sun: the stock
 * shader paints them on every submerged pixel, hull undersides included.
 */
class AtmosphereWaterFilter extends WaterFilter {
	static final String MATERIAL_DEF = "Shaders/Water/CausticWater.j3md";
	private static final float CLEAR_FOG_DISTANCE = 100_000f;
	private boolean fogEnabled;

	AtmosphereWaterFilter(Node reflectionScene, Vector3f lightDirection) {
		super(reflectionScene, lightDirection);
	}

	void setFogEnabled(boolean enabled) {
		fogEnabled = enabled;
	}

	@Override
	protected void initFilter(AssetManager manager, RenderManager renderManager, ViewPort vp, int w, int h) {
		super.initFilter(manager, renderManager, vp, w, h);
		material = withSurfaceAwareCaustics(manager, material);
	}

	/**
	 * Rebuilds a stock water material on {@value #MATERIAL_DEF}, carrying over every parameter the
	 * stock filter set, textures included, so the filter's setters keep working unchanged.
	 */
	static Material withSurfaceAwareCaustics(AssetManager assets, Material stockWater) {
		Material water = new Material(assets, MATERIAL_DEF);
		for (MatParam param : stockWater.getParams())
			water.setParam(param.getName(), param.getVarType(), param.getValue());
		return water;
	}

	@Override
	protected Material getMaterial() {
		Material water = super.getMaterial();
		if (water != null)
			water.setFloat("UnderWaterFogDistance", fogEnabled ? getUnderWaterFogDistance() : CLEAR_FOG_DISTANCE);
		return water;
	}
}
