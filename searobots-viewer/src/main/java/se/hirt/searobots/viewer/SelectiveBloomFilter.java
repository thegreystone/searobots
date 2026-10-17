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

import java.util.function.Predicate;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.post.filters.BloomFilter;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;

/** Renders emitters and vehicle occluders without redrawing the landscape in the glow pass. */
final class SelectiveBloomFilter extends BloomFilter {
	private static final String OCCLUDER = "selectiveBloomOccluder";
	private static final String[] EMISSION_MAPS = {"GlowMap", "EmissiveMap"};
	private static final String[] EMISSION_COLORS = {"GlowColor", "Emissive"};
	private RenderManager manager;

	SelectiveBloomFilter() {
		super(GlowMode.Objects);
	}

	@Override
	protected void initFilter(
		AssetManager assets, RenderManager renderManager, ViewPort viewPort, int width, int height) {
		manager = renderManager;
		super.initFilter(assets, renderManager, viewPort, width, height);
	}

	@Override
	protected void postQueue(RenderQueue queue) {
		Predicate<Geometry> previous = manager.getRenderFilter();
		manager.setRenderFilter(
				geometry -> contributesToGlow(geometry) && (previous == null || previous.test(geometry)));
		try {
			super.postQueue(queue);
		} finally {
			manager.setRenderFilter(previous);
		}
	}

	/** Vehicle hulls must write depth in the glow pass to hide lights on their far side. */
	static void markOccluder(Spatial root) {
		root.setUserData(OCCLUDER, true);
	}

	static boolean contributesToGlow(Geometry geometry) {
		Material material = geometry.getMaterial();
		if (emitsLight(material))
			return true;
		if (material == null || geometry.getQueueBucket() != RenderQueue.Bucket.Opaque
				|| !material.getAdditionalRenderState().isDepthWrite())
			return false;
		for (Spatial ancestor = geometry; ancestor != null; ancestor = ancestor.getParent()) {
			if (Boolean.TRUE.equals(ancestor.getUserData(OCCLUDER)))
				return true;
		}
		return false;
	}

	static boolean emitsLight(Material material) {
		if (material == null)
			return false;
		for (String name : EMISSION_MAPS) {
			if (material.getTextureParam(name) != null && material.getTextureParam(name).getTextureValue() != null)
				return true;
		}
		for (String name : EMISSION_COLORS) {
			var parameter = material.getParam(name);
			if (parameter != null && parameter.getValue() instanceof ColorRGBA color
					&& (color.r > 0 || color.g > 0 || color.b > 0))
				return true;
		}
		return false;
	}
}
