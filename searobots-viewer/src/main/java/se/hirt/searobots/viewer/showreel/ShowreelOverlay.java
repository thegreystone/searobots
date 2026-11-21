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
package se.hirt.searobots.viewer.showreel;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.GradientPaint;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.RadialGradientPaint;
import java.awt.RenderingHints;
import java.awt.font.TextAttribute;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import javax.imageio.ImageIO;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Quad;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.texture.plugins.AWTLoader;

/**
 * The showreel's graphics over the 3D view: title and end cards with the logo, lower-third
 * captions, a time-lapse speed badge, fades to black and the white flash of a detonation. Text is
 * set with Java2D into textures, so it stays crisp at any size.
 */
public final class ShowreelOverlay {
	static final Color MINT = new Color(0x5ff0c8);
	private static final List<String> TITLE_FONTS = List.of("Montserrat", "Avenir Next", "Bahnschrift",
			"Segoe UI Semibold", "Helvetica Neue", "Arial");
	private static final List<String> TEXT_FONTS = List.of("Montserrat", "Avenir Next", "Segoe UI", "Helvetica Neue",
			"Arial");

	private final AssetManager assets;
	private final Node root = new Node("showreel-overlay");
	private final int width, height;
	private final Geometry fade, flash;
	private final Geometry title, end;
	private final Geometry caption;
	private final Geometry badge;
	private final Map<String, Texture2D> captions = new HashMap<>();
	private final Map<String, Texture2D> badges = new HashMap<>();
	private final String titleFamily, textFamily;
	private String shownCaption, shownBadge;

	public ShowreelOverlay(AssetManager assets, int width, int height, String url) {
		this.assets = assets;
		this.width = width;
		this.height = height;
		var families = Set.of(GraphicsEnvironment.getLocalGraphicsEnvironment().getAvailableFontFamilyNames());
		titleFamily = TITLE_FONTS.stream().filter(families::contains).findFirst().orElse(Font.SANS_SERIF);
		textFamily = TEXT_FONTS.stream().filter(families::contains).findFirst().orElse(Font.SANS_SERIF);

		title = picture("title-card", card(null), 0, 0, 10);
		end = picture("end-card", card(url), 0, 0, 10);
		caption = picture("caption", null, 0, 0, 5);
		badge = picture("badge", null, 0, 0, 5);
		flash = solid("flash", ColorRGBA.White, 20);
		fade = solid("fade", ColorRGBA.Black, 30);
		for (var g : List.of(title, end, caption, badge, flash, fade))
			root.attachChild(g);
		setFade(1);
		setFlash(0);
		showCard(null, 0);
		showCaption(null, 0, 0);
		showSpeed(0, 0);
	}

	public Node root() {
		return root;
	}

	/** Black over everything: 1 is fully black. */
	public void setFade(float alpha) {
		setAlpha(fade, alpha);
	}

	/** White over the scene, under the cards and captions. */
	public void setFlash(float alpha) {
		setAlpha(flash, alpha);
	}

	/**
	 * Shows the title card ({@code end} false) or the end card, or neither if {@code which} is
	 * null.
	 */
	public void showCard(Boolean end, float alpha) {
		setAlpha(title, end != null && !end ? alpha : 0);
		setAlpha(this.end, end != null && end ? alpha : 0);
		float rise = 14 * (1 - alpha); // drifts up a little as it fades in
		title.setLocalTranslation(0, -rise, 10);
		this.end.setLocalTranslation(0, -rise, 10);
	}

	/**
	 * Shows a lower-third caption, "TITLE|subtitle", sliding in from the left as {@code slide} goes
	 * from 0 to 1.
	 */
	public void showCaption(String text, float alpha, float slide) {
		if (text == null || alpha <= 0) {
			setAlpha(caption, 0);
			return;
		}
		if (!text.equals(shownCaption)) {
			shownCaption = text;
			var texture = captions.computeIfAbsent(text, this::captionTexture);
			setTexture(caption, texture);
		}
		float eased = 1 - (1 - slide) * (1 - slide);
		float margin = Math.round(height * 0.075f);
		caption.setLocalTranslation(margin - 24 * (1 - eased), margin * 0.9f, 5);
		setAlpha(caption, alpha);
	}

	/** Shows the time-lapse badge, "48×", for speeds of at least 2; hides it otherwise. */
	public void showSpeed(double speed, float alpha) {
		if (speed < 2 || alpha <= 0) {
			setAlpha(badge, 0);
			return;
		}
		String text = (speed >= 10 ? Long.toString(Math.round(speed / 5) * 5) : Long.toString(Math.round(speed))) + "×";
		if (!text.equals(shownBadge)) {
			shownBadge = text;
			setTexture(badge, badges.computeIfAbsent(text, this::badgeTexture));
		}
		var texture = (Texture2D) badge.getMaterial().getTextureParam("Texture").getTextureValue();
		float margin = Math.round(height * 0.06f);
		badge.setLocalTranslation(width - margin - texture.getImage().getWidth(),
				height - margin - texture.getImage().getHeight(), 5);
		setAlpha(badge, alpha);
	}

	// ── Graphics ──

	private BufferedImage card(String url) {
		var image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
		var g = graphics(image);
		float unit = height / 960f;
		int logoSize = Math.round(150 * unit);
		// The block sits in the sky, above the horizon and the island
		int block = logoSize + Math.round((78 + 52 + (url != null ? 64 : 0)) * unit);
		float centre = height * 0.34f;
		// A soft dark pool behind the type keeps it readable over bright water and sky
		g.setPaint(new RadialGradientPaint(width / 2f, centre, width * 0.36f, new float[] {0f, 1f},
				new Color[] {new Color(0, 10, 24, 165), new Color(0, 10, 24, 0)}));
		g.fillRect(0, 0, width, height);
		int y = Math.round(centre - block / 2f);
		try (var in = ShowreelOverlay.class.getResourceAsStream("/icons/searobots-512.png")) {
			if (in != null) {
				var logo = ImageIO.read(in);
				g.drawImage(logo, (width - logoSize) / 2, y, logoSize, logoSize, null);
			}
		} catch (IOException e) {
			// The card still works without its logo
		}
		y += logoSize + Math.round(78 * unit);
		var name = tracked(new Font(titleFamily, Font.BOLD, Math.round(86 * unit)), 0.22f);
		centered(g, "SEAROBOTS", name, Color.WHITE, y);
		y += Math.round(52 * unit);
		var tagline = tracked(new Font(textFamily, Font.BOLD, Math.round(25 * unit)), 0.34f);
		centered(g, "AUTONOMOUS UNDERWATER COMBAT", tagline, new Color(0x8ff7dc), y);
		if (url != null) {
			y += Math.round(64 * unit);
			centered(g, url, tracked(new Font(textFamily, Font.PLAIN, Math.round(26 * unit)), 0.06f),
					new Color(235, 242, 250, 230), y);
		}
		g.dispose();
		return image;
	}

	private Texture2D captionTexture(String text) {
		String[] parts = text.split("\\|", 2);
		String head = parts[0], sub = parts.length > 1 && !parts[1].isBlank() ? parts[1] : null;
		float unit = height / 960f;
		var headFont = tracked(new Font(titleFamily, Font.BOLD, Math.round(38 * unit)), 0.12f);
		var subFont = tracked(new Font(textFamily, Font.PLAIN, Math.round(23 * unit)), 0.04f);
		var probe = graphics(new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB));
		FontMetrics hm = probe.getFontMetrics(headFont), sm = probe.getFontMetrics(subFont);
		int bar = Math.round(5 * unit), gap = Math.round(18 * unit), pad = Math.round(8 * unit);
		int w = bar + gap + Math.max(hm.stringWidth(head), sub == null ? 0 : sm.stringWidth(sub)) + 2 * pad;
		int h = hm.getHeight() + (sub == null ? 0 : sm.getHeight() + Math.round(4 * unit)) + 2 * pad;
		probe.dispose();

		var image = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
		var g = graphics(image);
		g.setPaint(new GradientPaint(0, 0, MINT, 0, h, new Color(MINT.getRed(), MINT.getGreen(), MINT.getBlue(), 120)));
		g.fillRect(pad, pad, bar, h - 2 * pad);
		int x = pad + bar + gap;
		int baseline = pad + hm.getAscent();
		shadowed(g, head, headFont, Color.WHITE, x, baseline);
		if (sub != null)
			shadowed(g, sub, subFont, new Color(225, 236, 245, 235), x,
					baseline + hm.getDescent() + Math.round(4 * unit) + sm.getAscent());
		g.dispose();
		return texture(image);
	}

	private Texture2D badgeTexture(String text) {
		float unit = height / 960f;
		var font = tracked(new Font(textFamily, Font.BOLD, Math.round(30 * unit)), 0.05f);
		var probe = graphics(new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB));
		var fm = probe.getFontMetrics(font);
		probe.dispose();
		int arrow = Math.round(15 * unit), pad = Math.round(6 * unit), gap = Math.round(12 * unit);
		int w = pad * 2 + arrow * 2 + gap + fm.stringWidth(text), h = pad * 2 + fm.getHeight();
		var image = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
		var g = graphics(image);
		int mid = h / 2;
		var shape = new Path2D.Float();
		for (int i = 0; i < 2; i++) {
			int x0 = pad + i * arrow;
			shape.moveTo(x0, mid - arrow * 0.8f);
			shape.lineTo(x0 + arrow, mid);
			shape.lineTo(x0, mid + arrow * 0.8f);
			shape.closePath();
		}
		g.setColor(new Color(0, 0, 0, 90));
		g.translate(1.5, 1.5);
		g.fill(shape);
		g.translate(-1.5, -1.5);
		g.setColor(MINT);
		g.fill(shape);
		shadowed(g, text, font, Color.WHITE, pad + arrow * 2 + gap, pad + fm.getAscent());
		g.dispose();
		return texture(image);
	}

	private static Graphics2D graphics(BufferedImage image) {
		var g = image.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
		g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
		g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BICUBIC);
		g.setStroke(new BasicStroke(1f));
		return g;
	}

	private static Font tracked(Font font, float tracking) {
		return font.deriveFont(Map.of(TextAttribute.TRACKING, tracking));
	}

	private void centered(Graphics2D g, String text, Font font, Color color, int baseline) {
		int w = g.getFontMetrics(font).stringWidth(text);
		shadowed(g, text, font, color, (width - w) / 2, baseline);
	}

	/** Text with a soft drop shadow, so it reads over bright water. */
	private static void shadowed(Graphics2D g, String text, Font font, Color color, int x, int y) {
		g.setFont(font);
		for (int r = 3; r >= 1; r--) {
			g.setColor(new Color(0, 6, 16, 40));
			for (int dx = -r; dx <= r; dx += r)
				for (int dy = -r; dy <= r; dy += r)
					g.drawString(text, x + dx + 1, y + dy + 2);
		}
		g.setColor(color);
		g.drawString(text, x, y);
	}

	private Texture2D texture(BufferedImage image) {
		Image converted = new AWTLoader().load(image, true);
		converted.setColorSpace(ColorSpace.sRGB);
		var texture = new Texture2D(converted);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		texture.setMinFilter(Texture.MinFilter.BilinearNoMipMaps);
		return texture;
	}

	private Geometry picture(String name, BufferedImage image, float x, float y, float z) {
		var geometry = new Geometry(name, new Quad(1, 1));
		var material = new Material(assets, "Common/MatDefs/Gui/Gui.j3md");
		material.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
		material.setColor("Color", new ColorRGBA(1, 1, 1, 0));
		geometry.setMaterial(material);
		geometry.setLocalTranslation(x, y, z);
		if (image != null)
			setTexture(geometry, texture(image));
		else
			geometry.setCullHint(Spatial.CullHint.Always);
		return geometry;
	}

	private static void setTexture(Geometry geometry, Texture2D texture) {
		geometry.getMaterial().setTexture("Texture", texture);
		geometry.setLocalScale(texture.getImage().getWidth(), texture.getImage().getHeight(), 1);
	}

	private Geometry solid(String name, ColorRGBA color, float z) {
		var geometry = new Geometry(name, new Quad(width, height));
		var material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setColor("Color", color.clone());
		material.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
		geometry.setMaterial(material);
		geometry.setLocalTranslation(0, 0, z);
		return geometry;
	}

	private static void setAlpha(Geometry geometry, float alpha) {
		alpha = Math.max(0, Math.min(1, alpha));
		var param = geometry.getMaterial().getParam("Color");
		var color = ((ColorRGBA) param.getValue()).clone();
		color.a = alpha;
		geometry.getMaterial().setColor("Color", color);
		boolean hasTexture = geometry.getMaterial().getTextureParam("Texture") != null
				|| geometry.getMaterial().getMaterialDef().getName().contains("Unshaded");
		geometry.setCullHint(alpha > 0.001f && hasTexture ? Spatial.CullHint.Never : Spatial.CullHint.Always);
	}

	@Override
	public String toString() {
		return String.format(Locale.ROOT, "overlay %dx%d (%s / %s)", width, height, titleFamily, textFamily);
	}
}
