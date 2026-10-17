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
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector2f;

import java.util.Locale;

/** Model-derived, relit distant crowns sharing the modeled trees' variant selection and tint. */
public final class TreeImpostors {

	public static final float SIDE_WIDTH_SCALE = 1.12f;
	public static final float SIDE_HEIGHT_SCALE = 1.04f;
	public static final int ATLAS_COLUMNS = 4;
	public static final int SIDE_VIEWS = 4;
	// Leave ground coverage inside the scene's minimum 6 km far plane, including tall crowns.
	public static final float FADE_START = 5400f;
	public static final float FADE_END = 5750f;

	private TreeImpostors() {
	}

	/** The same stable hash selects a tree's modeled and distant archetype. */
	public static int variant(long seed, TreeScatter.TreeInstance tree, TreeScatter.Type type) {
		long value = seed ^ ((long) Float.floatToIntBits(tree.x()) << 32) ^ Float.floatToIntBits(tree.z())
				^ type.ordinal() * 0x9e3779b97f4a7c15L;
		value = (value ^ (value >>> 30)) * 0xbf58476d1ce4e5b9L;
		value = (value ^ (value >>> 27)) * 0x94d049bb133111ebL;
		return (int) (value ^ (value >>> 31));
	}

	public static int variantIndex(long seed, TreeScatter.TreeInstance tree, TreeScatter.Type type) {
		return Math.floorMod(variant(seed, tree, type), type == TreeScatter.Type.BROADLEAF ? 3 : 2);
	}

	/** Cutouts contain unlit albedo and leaf normals; sun and probes are evaluated at runtime. */
	static Material material(AssetManager assets, TreeScatter.Type type, boolean top) {
		String prefix = "Textures/Terrain/impostors/" + type.name().toLowerCase(Locale.ROOT)
				+ (top ? "-top-" : "-side-");
		int tileHeight = top ? 128 : 256;
		Material material = new Material(assets, "Shaders/Shoreline/TreeImpostor.j3md");
		material.setName("model-derived " + type.name().toLowerCase(Locale.ROOT) + (top ? " crown" : " sides"));
		material.setTexture("BaseColorMap",
				FoliageMipmaps.loadAtlas(assets, prefix + "albedo.png", 128, tileHeight, false));
		material.setTexture("NormalMap",
				FoliageMipmaps.loadAtlas(assets, prefix + "normal.png", 128, tileHeight, true));
		material.setColor("BaseColor", ColorRGBA.White);
		material.setFloat("Metallic", 0f);
		material.setFloat("Roughness", .76f);
		material.setFloat("NormalType", 1f);
		material.setBoolean("UseVertexColor", true);
		material.setBoolean("ImpostorBackViews", !top);
		material.setBoolean("ImpostorTopViews", top);
		material.setVector2("ImpostorDistanceFadeRange", new Vector2f(FADE_START, FADE_END));
		material.setVector2("ImpostorTopFadeRange",
				type == TreeScatter.Type.CONIFER ? new Vector2f(.65f, .85f) : new Vector2f(.38f, .62f));
		material.setFloat("AlphaDiscardThreshold", FoliageMipmaps.ALPHA_DISCARD);
		material.getAdditionalRenderState()
				.setFaceCullMode(top ? RenderState.FaceCullMode.Back : RenderState.FaceCullMode.Off);
		return material;
	}

}
