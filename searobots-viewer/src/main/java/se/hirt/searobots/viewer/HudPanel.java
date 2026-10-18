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
import com.jme3.font.BitmapText;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Quad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.ToDoubleFunction;

/** A quiet, size-bounded backdrop for screen-space information. */
final class HudPanel extends Node {

	static final float PADDING = 10;
	private final BitmapText text;
	private final Geometry backing;
	private float backgroundOpacity = .78f;
	private String source = "";
	private String rendered = "";

	HudPanel(AssetManager assets, BitmapText text) {
		super("HudPanel");
		this.text = text;
		setQueueBucket(RenderQueue.Bucket.Gui);
		backing = new Geometry("HudBackdrop", new Quad(1, 1));
		Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setColor("Color", new ColorRGBA(.018f, .035f, .055f, .78f));
		material.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
		material.getAdditionalRenderState().setDepthTest(false);
		material.getAdditionalRenderState().setDepthWrite(false);
		backing.setMaterial(material);
		attachChild(backing);
		attachChild(text);
	}

	void setBackgroundOpacity(float opacity) {
		backgroundOpacity = opacity;
	}

	BitmapText text() {
		return text;
	}

	/** Places the panel by its top-left corner, returning the occupied height. */
	float layout(float x, float top, float width, float maxHeight, float alpha) {
		if (!text.getText().equals(rendered))
			source = text.getText();
		float lineHeight = text.getLineHeight();
		int lines = (int) ((maxHeight - 2 * PADDING) / lineHeight);
		if (source.isBlank() || lines <= 0 || width <= 2 * PADDING) {
			setCullHint(CullHint.Always);
			return 0;
		}
		float scale = text.getSize() / text.getFont().getCharSet().getRenderedSize();
		List<String> fitted = fit(source, width - 2 * PADDING, lines,
				value -> text.getFont().getLineWidth(value) * scale);
		rendered = String.join("\n", fitted);
		text.setText(rendered);
		float height = fitted.size() * lineHeight + 2 * PADDING;
		setCullHint(CullHint.Inherit);
		setLocalTranslation(x, top, getLocalTranslation().z);
		setBackgroundSize(width, height);
		backing.getMaterial().setColor("Color", new ColorRGBA(.018f, .035f, .055f, backgroundOpacity * alpha));
		text.setLocalTranslation(PADDING, -PADDING, 0);
		ColorRGBA color = text.getColor() == null ? ColorRGBA.White.clone() : text.getColor().clone();
		color.a = alpha;
		text.setColor(color);
		return height;
	}

	/** Matches another panel's backdrop without changing text wrapping or visibility. */
	void setBackgroundSize(float width, float height) {
		backing.setLocalTranslation(0, -height, -.1f);
		backing.setLocalScale(width, height, 1);
	}

	/** Wraps at word boundaries, including long identifiers, and marks hidden rows. */
	static List<String> fit(String source, float width, int maxLines, ToDoubleFunction<String> measure) {
		if (maxLines <= 0 || width <= 0)
			return List.of();
		List<String> lines = new ArrayList<>();
		for (String original : source.stripTrailing().split("\n", -1)) {
			String remaining = original.stripTrailing();
			while (measure.applyAsDouble(remaining) > width && remaining.length() > 1) {
				int end = remaining.length() - 1;
				while (end > 1 && measure.applyAsDouble(remaining.substring(0, end)) > width)
					end--;
				int space = remaining.lastIndexOf(' ', end);
				if (space > 0)
					end = space;
				lines.add(remaining.substring(0, end).stripTrailing());
				remaining = remaining.substring(end).stripLeading();
			}
			lines.add(remaining);
		}
		if (lines.size() > maxLines) {
			int hidden = lines.size() - maxLines + 1;
			lines = new ArrayList<>(lines.subList(0, maxLines));
			String more = "... " + hidden + " more lines";
			while (more.length() > 1 && measure.applyAsDouble(more) > width)
				more = more.substring(0, more.length() - 1);
			lines.set(maxLines - 1, more);
		}
		return lines;
	}
}
