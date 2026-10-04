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

import java.nio.FloatBuffer;
import java.util.Set;

import com.jme3.asset.AssetManager;
import com.jme3.material.MatParamTexture;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.texture.Texture;
import com.jme3.util.BufferUtils;
import com.jme3.util.mikktspace.MikktspaceTangentGenerator;

/**
 * Load-time additions to the generated submarine model (SubmarineModelGenerator) that its OBJ and
 * MTL files cannot express: tangents for the normal-mapped tiles, the large-scale weathering light
 * map, which needs a second set of texture coordinates, and blending for the painted markings.
 */
public final class SubmarineModelSupport {
	// As in SubmarineModelGenerator: one repeat of the tile texture covers TILE_REPEAT metres of hull, and the
	// weathering texture WEATHERING_ALONG metres along the hull (its width) by WEATHERING_AROUND metres round it,
	// centred on the top centreline
	private static final float TILE_REPEAT = 8f, WEATHERING_ALONG = 80f, WEATHERING_AROUND = 40f;
	private static final String TILES = "submarine-tiles.png", WEATHERING = "models/submarine-weathering.png";
	// The untiled parts of the hull (nose cap, keel strip): weathered like the tiles, so they match them
	private static final String PLAIN_HULL = "HullPlain";
	// Model groups of painted markings (SubmarineModelGenerator)
	private static final Set<String> MARKINGS = Set.of("DraftMarks", "RescueRings");

	private SubmarineModelSupport() {
	}

	/** Prepares a freshly loaded submarine model for rendering. */
	public static void prepare(AssetManager assets, Spatial model) {
		model.depthFirstTraversal(s -> {
			if (!(s instanceof Geometry g) || g.getMaterial() == null)
				return;
			Material m = g.getMaterial();
			if (m.getTextureParam("NormalMap") != null)
				MikktspaceTangentGenerator.generate(g);
			MatParamTexture diffuse = m.getTextureParam("DiffuseMap");
			if (diffuse != null && diffuse.getTextureValue().getKey() != null
					&& diffuse.getTextureValue().getKey().getName().endsWith(TILES) || PLAIN_HULL.equals(g.getName()))
				addWeathering(assets, g);
			// The fixed markings are paint with transparent surroundings: blended over the hull, just in front of it
			if (MARKINGS.contains(g.getName())) {
				RenderState state = m.getAdditionalRenderState();
				state.setBlendMode(RenderState.BlendMode.Alpha);
				state.setPolyOffset(-2, -2);
				m.setFloat("AlphaDiscardThreshold", 0.02f);
				g.setQueueBucket(RenderQueue.Bucket.Transparent);
			}
			// The decal patches stay hidden until SubmarineDecals paints them
			if (SubmarineDecals.PATCHES.contains(g.getName()))
				g.setCullHint(Spatial.CullHint.Always);
		});
	}

	/**
	 * Spreads the weathering texture over a tiled mesh: its second texture coordinates are the
	 * first (in repeats of the tile texture) rescaled to the weathering texture's extent, so the
	 * weathering does not repeat along the hull.
	 */
	private static void addWeathering(AssetManager assets, Geometry g) {
		Mesh mesh = g.getMesh();
		FloatBuffer uv = mesh.getFloatBuffer(VertexBuffer.Type.TexCoord);
		if (uv == null)
			return;
		FloatBuffer uv2 = BufferUtils.createFloatBuffer(uv.limit());
		for (int i = 0; i + 1 < uv.limit(); i += 2) {
			float around = uv.get(i) * TILE_REPEAT, along = uv.get(i + 1) * TILE_REPEAT;
			uv2.put(along / WEATHERING_ALONG).put(0.5f + around / WEATHERING_AROUND);
		}
		uv2.flip();
		mesh.setBuffer(VertexBuffer.Type.TexCoord2, 2, uv2);
		Texture weathering = assets.loadTexture(WEATHERING);
		weathering.setWrap(Texture.WrapMode.Repeat);
		g.getMaterial().setTexture("LightMap", weathering);
		g.getMaterial().setBoolean("SeparateTexCoord", true);
	}
}
