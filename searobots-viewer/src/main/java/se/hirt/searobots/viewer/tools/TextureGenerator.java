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
package se.hirt.searobots.viewer.tools;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.imageio.ImageIO;

/**
 * Generates the custom terrain textures and the tree cutouts under {@code Textures/Terrain/custom}.
 * Pure Java2D, no dependencies; run from the repository root.
 * <p>
 * Each tree type gets a side cutout ({@code treeN.png}, used on the two crossed billboards) and a
 * top-down cutout ({@code treeN_top.png}, used on the horizontal canopy quad so trees read as
 * crowns rather than crossed cards when the camera looks down at an island). Foliage is painted as
 * leaf clouds over a handful of overlapping ellipsoidal clumps: every leaf is shaded from the
 * clump's surface normal against a fixed upper-left light, back clumps sit in shadow, and the union
 * of the clumps gives an irregular silhouette with sky showing through. Cutouts keep soft
 * anti-aliased alpha; the colour of transparent pixels is bled outwards from the opaque edge so
 * mip-mapped edges do not pick up dark fringes.
 * <p>
 * Side cutout aspect ratios must match {@code TreeScatter.Type.widthRatio}, or the billboards
 * stretch the art.
 */
public class TextureGenerator {
	static final String DIR = "searobots-viewer/src/main/resources/Textures/Terrain/custom/";

	/**
	 * Screen-space light for the cutouts: upper left, a little towards the viewer (image y grows
	 * downwards).
	 */
	private static final double[] LIGHT = normalize(-0.45, -0.75, 0.55);

	public static void main(String[] args) throws Exception {
		new File(DIR).mkdirs();
		genBroadleaf();
		genConifer();
		genPalm();
		genBush();
		genVegetation();
		genDeepSediment();
		System.out.println("All custom textures generated in " + DIR);
	}

	// ── Foliage primitives ──────────────────────────────────────────────

	/**
	 * An ellipsoidal leaf clump in image space; {@code depth} runs 0 (front, lit) to 1 (back,
	 * shaded).
	 */
	private record Clump(double cx, double cy, double rx, double ry, double depth) {
	}

	/**
	 * Paints {@code count} leaves distributed over the clumps (back to front, proportionally to
	 * area). Brightness follows the clump's surface normal against {@link #LIGHT}, so each clump
	 * reads as a rounded mass, and the whole cloud darkens towards the back and the underside.
	 */
	private static void leafCloud(
		Graphics2D g, Random rng, List<Clump> clumps, int count, float[] base, double leafSize) {
		var sorted = new ArrayList<>(clumps);
		sorted.sort((a, b) -> Double.compare(b.depth, a.depth));
		double total = 0;
		for (Clump c : clumps)
			total += c.rx * c.ry;
		for (Clump c : sorted) {
			int n = (int) Math.round(count * c.rx * c.ry / total);
			for (int i = 0; i < n; i++) {
				double a = rng.nextDouble() * 2 * Math.PI;
				double r = Math.pow(rng.nextDouble(), 0.62); // denser towards the rim so the outline is crisp
				double nx = r * Math.cos(a), ny = r * Math.sin(a);
				double nz = Math.sqrt(Math.max(0, 1 - r * r));
				double lit = Math.max(0, nx * LIGHT[0] + ny * LIGHT[1] + nz * LIGHT[2]);
				double shade = (0.28 + 0.72 * lit) * (1 - 0.5 * c.depth) * (0.82 + 0.36 * rng.nextDouble());
				double x = c.cx + nx * c.rx, y = c.cy + ny * c.ry;
				leaf(g, x, y, leafSize * (0.65 + 0.7 * rng.nextDouble()), rng.nextDouble() * Math.PI,
						tint(base, shade));
			}
		}
	}

	/** One leaf: a small rotated lozenge. */
	private static void leaf(Graphics2D g, double x, double y, double size, double angle, Color c) {
		var t = g.getTransform();
		g.translate(x, y);
		g.rotate(angle);
		g.setColor(c);
		var p = new Path2D.Double();
		p.moveTo(-size, 0);
		p.lineTo(0, -size * 0.45);
		p.lineTo(size, 0);
		p.lineTo(0, size * 0.45);
		p.closePath();
		g.fill(p);
		g.setTransform(t);
	}

	/**
	 * Base colour scaled by {@code shade}; bright leaves drift towards yellow, dark ones towards
	 * blue.
	 */
	private static Color tint(float[] base, double shade) {
		double s = Math.max(0, Math.min(1.6, shade));
		double r = base[0] * s + Math.max(0, s - 0.9) * 0.35;
		double gg = base[1] * s + Math.max(0, s - 0.9) * 0.12;
		double b = base[2] * s + Math.max(0, 0.5 - s) * 0.10;
		return new Color(clampF(r), clampF(gg), clampF(b));
	}

	/**
	 * Tapered trunk or branch drawn pixel by pixel between two centre points, shaded across its
	 * width so the lit side (left) is lighter and the far edge falls into shadow; bark is broken up
	 * with horizontal streaks.
	 */
	private static void limb(
		BufferedImage img, Random rng, double x0, double y0, double x1, double y1, double w0, double w1, float[] bark) {
		double len = Math.hypot(x1 - x0, y1 - y0);
		int steps = Math.max(2, (int) (len * 1.5));
		double px = -(y1 - y0) / len, py = (x1 - x0) / len; // unit perpendicular
		for (int s = 0; s <= steps; s++) {
			double t = (double) s / steps;
			double cx = x0 + (x1 - x0) * t, cy = y0 + (y1 - y0) * t, w = w0 + (w1 - w0) * t;
			double streak = 0.9 + 0.2 * Math.sin(t * len * 0.7 + rng.nextDouble() * 0.4);
			for (double d = -w / 2; d <= w / 2; d += 0.5) {
				double u = d / (w / 2); // -1 lit edge .. +1 shadow edge
				double shade = (0.95 - 0.45 * (u + 1) / 2) * (1 - 0.35 * u * u) * streak;
				int x = (int) Math.round(cx + px * d), y = (int) Math.round(cy + py * d);
				if (x < 0 || y < 0 || x >= img.getWidth() || y >= img.getHeight())
					continue;
				img.setRGB(x, y, 0xFF000000 | rgb(bark[0] * shade, bark[1] * shade, bark[2] * shade));
			}
		}
	}

	private static void genBroadleaf() throws Exception {
		int w = 384, h = 512;
		var img = clear(w, h);
		var rng = new Random(101);
		float[] bark = {0.30f, 0.22f, 0.14f}, green = {0.20f, 0.44f, 0.11f};
		// Trunk with a gentle lean, forking into the crown
		limb(img, rng, 196, 512, 190, 300, 24, 13, bark);
		limb(img, rng, 190, 300, 120, 200, 13, 4, bark);
		limb(img, rng, 190, 300, 262, 190, 12, 4, bark);
		limb(img, rng, 190, 300, 200, 150, 11, 3, bark);
		limb(img, rng, 150, 250, 92, 270, 6, 2, bark);
		limb(img, rng, 236, 240, 300, 280, 6, 2, bark);
		var g = g(img);
		List<Clump> clumps = List.of(new Clump(196, 215, 150, 120, 0.95), // back mass
				new Clump(120, 220, 90, 78, 0.35), new Clump(275, 225, 92, 76, 0.30),
				new Clump(200, 140, 100, 72, 0.15), new Clump(150, 300, 82, 58, 0.55),
				new Clump(250, 300, 78, 56, 0.50), new Clump(200, 235, 105, 82, 0.05), // front lit mass
				new Clump(62, 270, 34, 26, 0.40), new Clump(335, 190, 40, 30, 0.35), new Clump(205, 84, 44, 30, 0.20),
				new Clump(300, 320, 36, 26, 0.45));
		leafCloud(g, rng, clumps, 9500, green, 6.5);
		g.dispose();
		bleed(img);
		write(img, "tree1.png");

		// Top-down crown: clumps ring a lit centre, edges fall away into shadow
		int s = 256;
		var top = clear(s, s);
		var rt = new Random(102);
		var gt = g(top);
		List<Clump> tc = new ArrayList<>();
		tc.add(new Clump(128, 128, 95, 95, 0.9));
		for (int i = 0; i < 7; i++) {
			double a = i * 2 * Math.PI / 7 + rt.nextDouble() * 0.5;
			tc.add(new Clump(128 + 52 * Math.cos(a), 128 + 52 * Math.sin(a), 48 + rt.nextInt(16), 44 + rt.nextInt(14),
					0.5 + 0.3 * rt.nextDouble()));
		}
		tc.add(new Clump(122, 124, 62, 60, 0.05));
		leafCloud(gt, rt, tc, 5200, green, 6.0);
		gt.dispose();
		bleed(top);
		write(top, "tree1_top.png");
	}

	private static void genConifer() throws Exception {
		int w = 232, h = 512;
		var img = clear(w, h);
		var rng = new Random(202);
		float[] bark = {0.28f, 0.20f, 0.13f}, needle = {0.09f, 0.30f, 0.15f};
		double cx = 116;
		limb(img, rng, cx + 2, 512, cx, 36, 15, 2, bark);
		var g = g(img);
		// Whorls of drooping branches from the top down; every branch is a fringe of short needle strokes
		for (double y = 52; y < 476; y += 14 + rng.nextInt(6)) {
			double t = (y - 52) / 424.0;
			double len = 12 + 92 * Math.pow(t, 0.8);
			for (int side = -1; side <= 1; side += 2) {
				double bl = len * (0.8 + 0.4 * rng.nextDouble());
				conBranch(g, rng, cx, y, side, bl, 0.30 + 0.25 * t, needle, side < 0 ? 1.0 : 0.72);
			}
			// A few branches angled towards the viewer, drawn shorter and darker to give depth
			if (rng.nextDouble() < 0.7)
				conBranch(g, rng, cx, y + 4, rng.nextBoolean() ? -1 : 1, len * 0.45, 0.6, needle, 0.55);
		}
		// Leader: short tufts around the top
		for (int i = 0; i < 14; i++) {
			double y = 30 + i * 3.0;
			conBranch(g, rng, cx, y, i % 2 == 0 ? -1 : 1, 4 + i * 1.2, 0.2, needle, 0.9);
		}
		g.dispose();
		bleed(img);
		write(img, "tree2.png");

		// Top-down: a star of branches radiating from the leader, lit centre, shaded rim
		int s = 256;
		var top = clear(s, s);
		var gt = g(top);
		var rt = new Random(203);
		for (int ring = 0; ring < 3; ring++) {
			int n = 9 + ring * 3;
			for (int i = 0; i < n; i++) {
				double a = i * 2 * Math.PI / n + rt.nextDouble() * 0.4 + ring * 0.3;
				double len = 62 + ring * 26 + rt.nextInt(18);
				double lit = 0.9 - 0.25 * ring + 0.25 * Math.cos(a - Math.atan2(LIGHT[1], LIGHT[0]));
				radialBranch(gt, rt, 128, 128, a, len, needle, lit);
			}
		}
		gt.dispose();
		bleed(top);
		write(top, "tree2_top.png");
	}

	/**
	 * Side-view conifer branch: a rachis drooping outwards, clothed in a dense mass of needle
	 * strokes and small leaf lozenges so the branch reads as a solid drooping bough rather than a
	 * bare rib.
	 */
	private static void conBranch(
		Graphics2D g, Random rng, double x0, double y0, int side, double len, double droop, float[] needle,
		double lit) {
		int steps = Math.max(4, (int) (len / 2.5));
		double px = x0, py = y0;
		for (int s = 1; s <= steps; s++) {
			double t = (double) s / steps;
			double x = x0 + side * len * t, y = y0 + droop * len * t * t;
			g.setStroke(new BasicStroke((float) (2.4 - 1.6 * t)));
			g.setColor(new Color(0.22f, 0.16f, 0.10f));
			g.drawLine((int) px, (int) py, (int) x, (int) y);
			double mass = 7 + 9 * (1 - 0.35 * t); // vertical extent of the needle mass under the rachis
			for (int k = 0; k < 7; k++) {
				double ox = (rng.nextDouble() - 0.5) * 6, oy = rng.nextDouble() * mass;
				double shade = lit * (0.5 + 0.7 * rng.nextDouble()) * (0.8 + 0.3 * t) * (1 - 0.35 * oy / mass);
				leaf(g, x + ox, y + oy, 2.2 + 2.2 * rng.nextDouble(), Math.PI / 2 + (rng.nextDouble() - 0.5) * 1.2,
						tint(needle, shade));
			}
			for (int k = 0; k < 3; k++) {
				double nl = 4 + mass * rng.nextDouble();
				double ang = Math.PI / 2 + (rng.nextDouble() - 0.5) * 1.6 - side * 0.35;
				g.setColor(tint(needle, lit * (0.6 + 0.6 * rng.nextDouble())));
				g.setStroke(new BasicStroke(1.2f));
				g.drawLine((int) x, (int) y, (int) (x + Math.cos(ang) * nl), (int) (y + Math.sin(ang) * nl));
			}
			px = x;
			py = y;
		}
	}

	/** Top-view branch: needle fringe on both sides of a straight rachis from the centre. */
	private static void radialBranch(
		Graphics2D g, Random rng, double cx, double cy, double a, double len, float[] needle, double lit) {
		double dx = Math.cos(a), dy = Math.sin(a);
		g.setStroke(new BasicStroke(2f));
		g.setColor(new Color(0.20f, 0.15f, 0.09f));
		g.drawLine((int) cx, (int) cy, (int) (cx + dx * len), (int) (cy + dy * len));
		for (double d = 8; d < len; d += 2.2) {
			double x = cx + dx * d, y = cy + dy * d;
			double nl = 5 + 11 * (1 - d / len) + rng.nextDouble() * 5;
			for (int side = -1; side <= 1; side += 2) {
				leaf(g, x + (rng.nextDouble() - 0.5) * 4, y + (rng.nextDouble() - 0.5) * 4, 2 + 2 * rng.nextDouble(),
						rng.nextDouble() * Math.PI, tint(needle, lit * (0.5 + 0.6 * rng.nextDouble())));
				double ang = a + side * (0.9 + rng.nextDouble() * 0.5);
				double shade = lit * (0.6 + 0.5 * rng.nextDouble()) * (1 - 0.3 * d / len);
				g.setColor(tint(needle, shade));
				g.setStroke(new BasicStroke(1.2f));
				g.drawLine((int) x, (int) y, (int) (x + Math.cos(ang) * nl), (int) (y + Math.sin(ang) * nl));
			}
		}
	}

	private static void genPalm() throws Exception {
		int w = 320, h = 512;
		var img = clear(w, h);
		var rng = new Random(303);
		float[] bark = {0.42f, 0.33f, 0.20f}, frond = {0.14f, 0.42f, 0.12f}, dead = {0.46f, 0.36f, 0.16f};
		// Curved trunk (quadratic Bezier) drawn as short limbs, with ring bands from the leaf scars
		double[] p0 = {150, 512}, p1 = {112, 330}, p2 = {178, 150};
		double prevX = p0[0], prevY = p0[1];
		int segs = 34;
		for (int i = 1; i <= segs; i++) {
			double t = (double) i / segs;
			double x = (1 - t) * (1 - t) * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0];
			double y = (1 - t) * (1 - t) * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1];
			double band = i % 2 == 0 ? 1.0 : 0.86;
			float[] b = {(float) (bark[0] * band), (float) (bark[1] * band), (float) (bark[2] * band)};
			double w0 = 26 - 13 * (i - 1.0) / segs, w1 = 26 - 13 * (double) i / segs;
			if (i < 4) { // flared base
				w0 += (4 - i) * 3;
				w1 += (3 - i) * 3;
			}
			limb(img, new Random(i), prevX, prevY, x, y, w0, w1, b);
			prevX = x;
			prevY = y;
		}
		var g = g(img);
		double crX = p2[0], crY = p2[1];
		// Dead fronds hanging under the crown, then live fronds back to front
		for (int i = 0; i < 4; i++)
			palmFrond(g, rng, crX, crY, Math.toRadians(55 + i * 22), 95 + rng.nextInt(30), 150, dead, 0.55);
		int n = 15;
		List<double[]> fronds = new ArrayList<>();
		for (int i = 0; i < n; i++) {
			double a = -Math.PI * 0.95 + i * (Math.PI * 1.9 / (n - 1)) + (rng.nextDouble() - 0.5) * 0.25;
			double depth = rng.nextDouble();
			fronds.add(new double[] {a, 118 + rng.nextInt(38), depth});
		}
		fronds.sort((x, y) -> Double.compare(y[2], x[2]));
		for (double[] f : fronds)
			palmFrond(g, rng, crX, crY, f[0], f[1], 105, frond, 1.0 - 0.45 * f[2]);
		// Coconuts and the crown boss
		for (int i = 0; i < 7; i++) {
			double shade = 0.7 + 0.5 * rng.nextDouble();
			g.setColor(new Color(clampF(0.35 * shade), clampF(0.30 * shade), clampF(0.12 * shade)));
			g.fill(new Ellipse2D.Double(crX - 14 + rng.nextInt(20), crY + 2 + rng.nextInt(16), 11, 12));
		}
		g.dispose();
		bleed(img);
		write(img, "tree3.png");

		// Top-down: fronds radiating from the centre, lit side brighter
		int s = 256;
		var top = clear(s, s);
		var gt = g(top);
		var rt = new Random(304);
		for (int i = 0; i < 14; i++) {
			double a = i * 2 * Math.PI / 14 + rt.nextDouble() * 0.3;
			double lit = 0.95 + 0.3 * Math.cos(a - Math.atan2(LIGHT[1], LIGHT[0]));
			palmFrondTop(gt, rt, 128, 128, a, 96 + rt.nextInt(22), frond, lit);
		}
		gt.setColor(new Color(0.30f, 0.24f, 0.10f));
		gt.fill(new Ellipse2D.Double(118, 118, 20, 20));
		gt.dispose();
		bleed(top);
		write(top, "tree3_top.png");
	}

	/** Side-view frond: an arching rachis with drooping leaflets, tips lighter than the base. */
	private static void palmFrond(
		Graphics2D g, Random rng, double x0, double y0, double angle, double len, double droop, float[] col,
		double lit) {
		int steps = 26;
		double dx = Math.cos(angle), dy = Math.sin(angle);
		double px = x0, py = y0;
		for (int s = 1; s <= steps; s++) {
			double t = (double) s / steps;
			double x = x0 + dx * len * t, y = y0 + dy * len * t * (1 - 0.35 * t) + droop * t * t;
			g.setStroke(new BasicStroke((float) (3.0 - 2.2 * t)));
			g.setColor(tint(new float[] {col[0] * 0.9f, col[1] * 0.75f, col[2] * 0.7f}, lit * 0.8));
			g.drawLine((int) px, (int) py, (int) x, (int) y);
			if (t > 0.12) {
				double tx = x - px, ty = y - py, tl = Math.hypot(tx, ty);
				double nx = -ty / tl, ny = tx / tl;
				double ll = 20 * (1 - 0.55 * t);
				for (int side = -1; side <= 1; side += 2) {
					double shade = lit * (0.7 + 0.5 * rng.nextDouble()) * (0.85 + 0.35 * t);
					g.setColor(tint(col, shade));
					g.setStroke(new BasicStroke(1.6f));
					double ex = x + nx * ll * side + tx / tl * 6, ey = y + ny * ll * side + 7 + rng.nextInt(4);
					g.drawLine((int) x, (int) y, (int) ex, (int) ey);
				}
			}
			px = x;
			py = y;
		}
	}

	/** Top-view frond: straight rachis with leaflets swept back towards the tip. */
	private static void palmFrondTop(
		Graphics2D g, Random rng, double cx, double cy, double a, double len, float[] col, double lit) {
		double dx = Math.cos(a), dy = Math.sin(a);
		g.setStroke(new BasicStroke(2.5f));
		g.setColor(tint(new float[] {col[0] * 0.9f, col[1] * 0.75f, col[2] * 0.7f}, lit * 0.8));
		g.drawLine((int) cx, (int) cy, (int) (cx + dx * len), (int) (cy + dy * len));
		for (double d = 12; d < len; d += 2.5) {
			double x = cx + dx * d, y = cy + dy * d;
			double ll = 5 + 16 * (1 - d / len) * (0.7 + 0.4 * rng.nextDouble());
			for (int side = -1; side <= 1; side += 2) {
				double ang = a + side * 1.15;
				g.setColor(tint(col, lit * (0.7 + 0.5 * rng.nextDouble())));
				g.setStroke(new BasicStroke(1.4f));
				g.drawLine((int) x, (int) y, (int) (x + Math.cos(ang) * ll + dx * 4),
						(int) (y + Math.sin(ang) * ll + dy * 4));
			}
		}
	}

	private static void genBush() throws Exception {
		int w = 352, h = 320;
		var img = clear(w, h);
		var rng = new Random(404);
		float[] bark = {0.26f, 0.19f, 0.12f}, green = {0.19f, 0.40f, 0.12f};
		limb(img, rng, 150, 320, 128, 220, 7, 2, bark);
		limb(img, rng, 178, 320, 196, 205, 7, 2, bark);
		limb(img, rng, 206, 320, 250, 225, 6, 2, bark);
		var g = g(img);
		List<Clump> clumps = List.of(new Clump(176, 190, 150, 92, 0.95), new Clump(95, 215, 78, 60, 0.4),
				new Clump(258, 210, 84, 62, 0.35), new Clump(176, 150, 92, 58, 0.15), new Clump(140, 235, 70, 50, 0.1),
				new Clump(222, 240, 68, 48, 0.2), new Clump(40, 250, 32, 24, 0.5), new Clump(318, 240, 30, 24, 0.5));
		leafCloud(g, rng, clumps, 7000, green, 5.5);
		// A scatter of pale flowers on the lit side
		for (int i = 0; i < 90; i++) {
			double a = rng.nextDouble() * 2 * Math.PI, r = Math.sqrt(rng.nextDouble());
			double x = 150 + r * 120 * Math.cos(a), y = 185 + r * 70 * Math.sin(a);
			g.setColor(new Color(0.85f, 0.85f, 0.55f));
			g.fill(new Ellipse2D.Double(x - 1.5, y - 1.5, 3, 3));
		}
		g.dispose();
		bleed(img);
		write(img, "tree4.png");

		int s = 256;
		var top = clear(s, s);
		var rt = new Random(405);
		var gt = g(top);
		List<Clump> tc = new ArrayList<>();
		tc.add(new Clump(128, 128, 88, 80, 0.9));
		for (int i = 0; i < 6; i++) {
			double a = i * Math.PI / 3 + rt.nextDouble() * 0.6;
			tc.add(new Clump(128 + 46 * Math.cos(a), 128 + 42 * Math.sin(a), 42 + rt.nextInt(14), 38 + rt.nextInt(12),
					0.3 + 0.4 * rt.nextDouble()));
		}
		tc.add(new Clump(124, 122, 52, 50, 0.05));
		leafCloud(gt, rt, tc, 4200, green, 5.5);
		gt.dispose();
		bleed(top);
		write(top, "tree4_top.png");
	}

	// ── Terrain textures ───────────────────────────────────────────────

	/**
	 * Grass: three tones (moss, lush, dry olive) mixed by slow noise, fine-grained brightness,
	 * thousands of short blade strokes, tufts, and a few bare-earth patches. The normal map is
	 * built from the same structure so the lit terrain shader picks up the tufts and clumps.
	 */
	static void genVegetation() throws Exception {
		int size = 512;
		float[][] n1 = valueNoise(size, 4, 11), n2 = valueNoise(size, 12, 12), n3 = valueNoise(size, 40, 13),
				n4 = valueNoise(size, 110, 14);
		float[] moss = {0.09f, 0.22f, 0.07f}, lush = {0.17f, 0.38f, 0.09f}, olive = {0.31f, 0.36f, 0.12f};
		float[][] col = new float[size * size][];
		float[] height = new float[size * size];
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				int i = y * size + x;
				float patch = n1[y][x] * 0.6f + n2[y][x] * 0.4f;
				float fine = n3[y][x] * 0.55f + n4[y][x] * 0.45f;
				float[] c = mix(mix(moss, lush, smoothstep(0.30f, 0.58f, patch)), olive,
						smoothstep(0.58f, 0.86f, patch));
				float b = 0.86f + 0.24f * (fine - 0.5f);
				col[i] = new float[] {c[0] * b, c[1] * b, c[2] * b};
				height[i] = 0.45f * n2[y][x] + 0.35f * n3[y][x] + 0.20f * n4[y][x];
			}
		}
		var rng = new Random(15);
		// Bare earth patches (wrap-around)
		for (int p = 0; p < 12; p++) {
			int cx = rng.nextInt(size), cy = rng.nextInt(size), r = 10 + rng.nextInt(22);
			for (int dy = -r; dy <= r; dy++)
				for (int dx = -r; dx <= r; dx++) {
					float d = (float) Math.hypot(dx, dy) / r;
					if (d > 1)
						continue;
					float f = (1 - d * d) * 0.45f
							* (0.6f + 0.4f * n4[(cy + dy + size) % size][(cx + dx + size) % size]);
					int i = ((cy + dy + size) % size) * size + (cx + dx + size) % size;
					col[i] = mix(col[i], new float[] {0.32f, 0.25f, 0.14f}, f);
					height[i] -= 0.25f * f;
				}
		}
		// Blade strokes: short, mostly vertical, lighter or darker than the ground they cross
		for (int k = 0; k < 14000; k++) {
			int x0 = rng.nextInt(size), y0 = rng.nextInt(size);
			double ang = -Math.PI / 2 + (rng.nextDouble() - 0.5) * 1.1;
			int len = 4 + rng.nextInt(9);
			float delta = rng.nextBoolean() ? 0.08f + 0.12f * rng.nextFloat() : -(0.10f + 0.14f * rng.nextFloat());
			stroke(col, height, size, x0, y0, ang, len, delta);
		}
		// Tufts: fans of blades from a common base, dark at the base and light at the tips
		for (int t = 0; t < 320; t++) {
			int x0 = rng.nextInt(size), y0 = rng.nextInt(size);
			int blades = 7 + rng.nextInt(8);
			for (int b = 0; b < blades; b++) {
				double ang = -Math.PI / 2 + (rng.nextDouble() - 0.5) * 1.6;
				stroke(col, height, size, x0, y0, ang, 8 + rng.nextInt(10), 0.08f + 0.14f * rng.nextFloat());
			}
			for (int dy = -2; dy <= 2; dy++)
				for (int dx = -2; dx <= 2; dx++) {
					int i = ((y0 + dy + size) % size) * size + (x0 + dx + size) % size;
					col[i] = mix(col[i], moss, 0.5f);
				}
		}
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				float[] c = col[y * size + x];
				img.setRGB(x, y, rgb(c[0], c[1], c[2]));
			}
		ImageIO.write(img, "PNG", new File(DIR + "vegetation.png"));
		ImageIO.write(normalMap(height, size, 2.2f), "PNG", new File(DIR + "vegetation_normal.png"));
		System.out.println("  vegetation.png + normal");
	}

	/** Adds a blade stroke to the colour and height fields, wrapping at the tile edges. */
	private static void stroke(
		float[][] col, float[] height, int size, int x0, int y0, double ang, int len, float delta) {
		for (int s = 0; s < len; s++) {
			int x = (int) Math.round(x0 + Math.cos(ang) * s), y = (int) Math.round(y0 + Math.sin(ang) * s);
			int i = ((y % size + size) % size) * size + ((x % size + size) % size);
			float f = delta * (0.4f + 0.6f * s / len);
			float[] c = col[i];
			col[i] = new float[] {c[0] * (1 + f), c[1] * (1 + f * 1.1f), c[2] * (1 + f * 0.6f)};
			height[i] += Math.abs(delta) * 0.35f;
		}
	}

	/** Deep sea floor: dark sediment with faint current ripples. */
	static void genDeepSediment() throws Exception {
		int size = 512;
		float[][] n1 = valueNoise(size, 10, 21), n2 = valueNoise(size, 40, 22), n3 = valueNoise(size, 80, 23);
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				float v = n1[y][x] * 0.5f + n2[y][x] * 0.3f + n3[y][x] * 0.2f;
				float ripple = 0.5f + 0.5f * (float) Math.sin(x * 2 * Math.PI * 9 / size + n1[y][x] * 5.0);
				v = v * 0.85f + ripple * 0.15f;
				img.setRGB(x, y, rgb(0.10f + v * 0.07f, 0.11f + v * 0.07f, 0.14f + v * 0.08f));
			}
		}
		ImageIO.write(img, "PNG", new File(DIR + "deep_sediment.png"));
		System.out.println("  deep_sediment.png");
	}

	// ── Helpers ─────────────────────────────────────────────────────────

	/** Tangent-space normal map from a tileable height field. */
	private static BufferedImage normalMap(float[] height, int size, float strength) {
		var nimg = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				int xp = (x + 1) % size, xm = (x - 1 + size) % size, yp = (y + 1) % size, ym = (y - 1 + size) % size;
				float dx = (height[y * size + xp] - height[y * size + xm]) * strength;
				float dy = (height[yp * size + x] - height[ym * size + x]) * strength;
				float len = (float) Math.sqrt(dx * dx + dy * dy + 1);
				nimg.setRGB(x, y, rgb(0.5f - dx / len * 0.5f, 0.5f - dy / len * 0.5f, 0.5f + 0.5f / len));
			}
		}
		return nimg;
	}

	/**
	 * Copies colour outwards from opaque pixels into their transparent neighbours (alpha
	 * untouched), so bilinear and mip-map filtering across the cutout edge blends leaf colour with
	 * leaf colour instead of with black.
	 */
	static void bleed(BufferedImage img) {
		int w = img.getWidth(), h = img.getHeight();
		int[] px = img.getRGB(0, 0, w, h, null, 0, w);
		boolean[] filled = new boolean[w * h];
		for (int i = 0; i < px.length; i++)
			filled[i] = (px[i] >>> 24) >= 8;
		for (int pass = 0; pass < 12; pass++) {
			boolean[] next = filled.clone();
			int[] out = px.clone();
			for (int y = 0; y < h; y++) {
				for (int x = 0; x < w; x++) {
					int i = y * w + x;
					if (filled[i])
						continue;
					int r = 0, g = 0, b = 0, n = 0;
					for (int dy = -1; dy <= 1; dy++)
						for (int dx = -1; dx <= 1; dx++) {
							int xx = x + dx, yy = y + dy;
							if (xx < 0 || yy < 0 || xx >= w || yy >= h)
								continue;
							int j = yy * w + xx;
							if (!filled[j])
								continue;
							r += (px[j] >> 16) & 0xFF;
							g += (px[j] >> 8) & 0xFF;
							b += px[j] & 0xFF;
							n++;
						}
					if (n > 0) {
						out[i] = (px[i] & 0xFF000000) | ((r / n) << 16) | ((g / n) << 8) | (b / n);
						next[i] = true;
					}
				}
			}
			px = out;
			filled = next;
		}
		img.setRGB(0, 0, w, h, px, 0, w);
	}

	private static void write(BufferedImage img, String file) throws Exception {
		ImageIO.write(img, "PNG", new File(DIR + file));
		System.out.println("  " + file);
	}

	static float[][] valueNoise(int size, int freq, long seed) {
		float[][] grid = new float[freq + 1][freq + 1];
		var r = new Random(seed * 7919 + freq);
		for (int y = 0; y <= freq; y++)
			for (int x = 0; x <= freq; x++)
				grid[y][x] = r.nextFloat();
		for (int y = 0; y <= freq; y++)
			grid[y][freq] = grid[y][0];
		for (int x = 0; x <= freq; x++)
			grid[freq][x] = grid[0][x];
		float[][] result = new float[size][size];
		for (int y = 0; y < size; y++) {
			float gy = (float) y / size * freq;
			int iy = (int) gy;
			float fy = gy - iy;
			fy = fy * fy * (3 - 2 * fy);
			for (int x = 0; x < size; x++) {
				float gx = (float) x / size * freq;
				int ix = (int) gx;
				float fx = gx - ix;
				fx = fx * fx * (3 - 2 * fx);
				float top = grid[iy][ix] + (grid[iy][ix + 1] - grid[iy][ix]) * fx;
				float bot = grid[iy + 1][ix] + (grid[iy + 1][ix + 1] - grid[iy + 1][ix]) * fx;
				result[y][x] = top + (bot - top) * fy;
			}
		}
		return result;
	}

	private static float[] mix(float[] a, float[] b, float t) {
		return new float[] {a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t};
	}

	private static float smoothstep(float lo, float hi, float v) {
		float t = Math.max(0, Math.min(1, (v - lo) / (hi - lo)));
		return t * t * (3 - 2 * t);
	}

	private static double[] normalize(double x, double y, double z) {
		double l = Math.sqrt(x * x + y * y + z * z);
		return new double[] {x / l, y / l, z / l};
	}

	private static float clampF(double v) {
		return (float) Math.max(0, Math.min(1, v));
	}

	private static int rgb(double r, double g, double b) {
		return (clamp((int) Math.round(r * 255)) << 16) | (clamp((int) Math.round(g * 255)) << 8)
				| clamp((int) Math.round(b * 255));
	}

	static BufferedImage clear(int w, int h) {
		var i = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
		var g = i.createGraphics();
		g.setComposite(AlphaComposite.Clear);
		g.fillRect(0, 0, w, h);
		g.dispose();
		return i;
	}

	static Graphics2D g(BufferedImage i) {
		var g = i.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
		g.setTransform(new AffineTransform());
		return g;
	}

	static int clamp(int v) {
		return Math.max(0, Math.min(255, v));
	}
}
