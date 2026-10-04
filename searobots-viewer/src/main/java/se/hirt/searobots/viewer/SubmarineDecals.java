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

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.util.Locale;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.plugins.AWTLoader;

/**
 * Paints a submarine's markings on the decal patches of the generated model
 * (SubmarineModelGenerator): its code, two letters and a two-digit number, on both sides of the
 * sail, and its name along both upper flanks, in a low-visibility grey a little lighter than the
 * hull.
 */
public final class SubmarineDecals {
	private static final String SAIL_CODE = "SailCode", HULL_NAME = "HullName";
	// Texture sizes, in the proportions of the model's patches
	private static final int CODE_W = 768, CODE_H = 256, NAME_W = 4096, NAME_H = 256;
	// Letters fill this much of the patch's height, and the text at most this much of its width
	private static final float LETTER_HEIGHT = 0.72f, MAX_WIDTH = 0.94f;
	private static final ColorRGBA PAINT = new ColorRGBA(0.3f, 0.3f, 0.3f, 1f);

	private SubmarineDecals() {
	}

	/**
	 * The code on the sail for a submarine: its two-letter short name and its engine id plus one,
	 * for example "CL 01".
	 */
	public static String code(String shortName, int id) {
		return String.format(Locale.ROOT, "%s %02d", shortName, id + 1);
	}

	/**
	 * Paints {@code code} on the sail and {@code name} along the flanks of a submarine model. Does
	 * nothing for models without the patches (the surface ship).
	 */
	public static void paint(AssetManager assets, Node model, String code, String name) {
		paint(assets, model, SAIL_CODE, code, CODE_W, CODE_H);
		paint(assets, model, HULL_NAME, name.toUpperCase(Locale.ROOT), NAME_W, NAME_H);
	}

	private static void paint(AssetManager assets, Node model, String patch, String text, int w, int h) {
		Spatial target = model.getChild(patch);
		if (target == null)
			return;
		var texture = new Texture2D(new AWTLoader().load(textImage(text, w, h), true));
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setAnisotropicFilter(8);
		Material m = new Material(assets, "Common/MatDefs/Light/Lighting.j3md");
		m.setTexture("DiffuseMap", texture);
		m.setBoolean("UseMaterialColors", true);
		m.setColor("Diffuse", PAINT);
		m.setColor("Ambient", PAINT);
		m.setColor("Specular", ColorRGBA.Black);
		m.setFloat("Shininess", 1f);
		RenderState state = m.getAdditionalRenderState();
		state.setBlendMode(RenderState.BlendMode.Alpha);
		state.setFaceCullMode(RenderState.FaceCullMode.Off);
		state.setPolyOffset(-2, -2); // stay in front of the surface underneath
		m.setFloat("AlphaDiscardThreshold", 0.02f);
		target.setMaterial(m);
		target.setQueueBucket(RenderQueue.Bucket.Transparent);
	}

	/**
	 * White text centred on a transparent image; the material's colour tints it. The transparent
	 * pixels are white too, so the edges stay clean in the smaller mipmaps.
	 */
	static BufferedImage textImage(String text, int w, int h) {
		var img = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
				img.setRGB(x, y, 0x00FFFFFF);
		Graphics2D g = img.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
		g.setComposite(java.awt.AlphaComposite.Src);
		g.setColor(Color.WHITE);
		// Size the font so capitals are LETTER_HEIGHT of the image, then shrink it if the text is too wide
		Font font = new Font(Font.SANS_SERIF, Font.BOLD, 100);
		float capHeight = (float) font.createGlyphVector(g.getFontRenderContext(), "H").getVisualBounds().getHeight();
		font = font.deriveFont(100f * LETTER_HEIGHT * h / capHeight);
		FontMetrics fm = g.getFontMetrics(font);
		if (fm.stringWidth(text) > MAX_WIDTH * w) {
			font = font.deriveFont(font.getSize2D() * MAX_WIDTH * w / fm.stringWidth(text));
			fm = g.getFontMetrics(font);
		}
		g.setFont(font);
		float cap = (float) font.createGlyphVector(g.getFontRenderContext(), "H").getVisualBounds().getHeight();
		g.drawString(text, (w - fm.stringWidth(text)) / 2f, (h + cap) / 2f);
		g.dispose();
		return img;
	}
}
