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

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

import javax.imageio.ImageIO;

/**
 * Generates {@code models/surface-ship.obj}, {@code surface-ship.mtl} and the two procedural
 * textures they reference: the 150 m x 30 m feeder container ship used for the surface-ship drone.
 * Pure Java, no dependencies.
 * <p>
 * The OBJ follows the same convention as {@code submarine-hybrid.obj} so the viewer treats both
 * templates identically: X is starboard, Y is fore-aft with the <b>bow at -Y</b> (the submarine's
 * propeller is at +Y), Z is up, units are metres and the waterline is at Z = 0. The viewer rotates
 * the template -90 degrees about X so the bow faces +Z in jME; no scaling is needed.
 * <p>
 * Faces are emitted with outward winding, explicit normals and texture coordinates. jME's OBJ
 * loader merges vertices with equal position/uv/normal, and the viewer only generates normals for
 * meshes that lack them, so the hull gets smooth per-vertex normals shared across its paint groups
 * (no crease at the boot-top) and everything box-like gets one flat normal per face for crisp
 * edges. Materials multiply their diffuse colour with a greyscale texture: a plating/grime tile for
 * hull, deck and deckhouse, and a corrugation atlas for the containers (side in u [0, 0.75), door
 * end in u [0.75, 1]). The propeller is its own group named {@code Propeller} and the rudder
 * {@code Rudder} so the viewer can animate them; their positions are fixed (see SHIP_PROP_LOCAL /
 * SHIP_RUDDER_LOCAL in the viewer).
 * <p>
 * Usage: {@code ShipModelGenerator <out-dir>}, for example
 * {@code java -cp target/classes se.hirt.searobots.viewer.tools.ShipModelGenerator src/main/resources/models}.
 * Use {@link ModelRenderCheck} to look at the result under jME lighting.
 */
public final class ShipModelGenerator {

	// Principal dimensions (metres): length, beam, draft, freeboard to the main deck amidships
	private static final double L = 150.0, B = 30.0, T = 7.5, D = 7.5;
	// Boot-topping band around the waterline
	private static final double BOOT_LO = -1.0, BOOT_HI = 1.2;
	// ISO 40 ft container
	private static final double TEU_L = 12.19, TEU_W = 2.44, TEU_H = 2.59;
	// Plating texture tile size in metres (u along the ship, v up) and texture file names
	private static final double PLATE_U = 18.75, PLATE_V = 20.0;
	private static final String HULL_TEX = "surface-ship-hull.png", CONTAINER_TEX = "surface-ship-container.png";
	private static final double CONTAINER_SIDE_U = 0.75;
	// Hull section levels (constant so consecutive stations can be quad-stripped)
	private static final int LEVELS = 13;
	// Deck layout along y (bow at -L/2)
	private static final int BAYS = 7;
	private static final double BAY_GAP = 1.4, BAY0_Y = -55.5;
	// Cranes stand in the gaps ahead of these bays; those gaps are wide enough for the pedestal to
	// clear the hatch covers on both sides
	private static final int[] CRANE_BAYS = {2, 5};
	private static final double CRANE_GAP = 4.4, CRANE_PEDESTAL_R = 1.6;
	private static final double HOUSE_Y0 = 45.0, HOUSE_Y1 = 60.0, HOUSE_HALF = 11.0;
	private static final int HOUSE_TIERS = 5; // accommodation tiers below the bridge deck
	private static final double TIER_H = 2.8, BRIDGE_H = 3.0;
	private static final double COAMING_H = 1.6, HATCH_COVER_H = 0.3;
	private static final double CONTAINER_BASE = D + COAMING_H + HATCH_COVER_H;
	private static final double RAIL_H = 1.05;

	private record Mat(double r, double g, double b, double ks, double ns, String map) {
	}

	private static final Map<String, Mat> MATERIALS = new LinkedHashMap<>();

	static {
		MATERIALS.put("hull_dark", new Mat(0.10, 0.12, 0.17, 0.35, 24, HULL_TEX));
		MATERIALS.put("hull_boot", new Mat(0.80, 0.80, 0.77, 0.25, 16, HULL_TEX));
		MATERIALS.put("hull_red", new Mat(0.52, 0.16, 0.12, 0.20, 12, HULL_TEX));
		MATERIALS.put("deck", new Mat(0.30, 0.35, 0.31, 0.10, 8, HULL_TEX));
		MATERIALS.put("hatch", new Mat(0.44, 0.46, 0.45, 0.15, 10, HULL_TEX));
		MATERIALS.put("superstructure", new Mat(0.88, 0.89, 0.87, 0.25, 16, HULL_TEX));
		MATERIALS.put("glass", new Mat(0.07, 0.10, 0.14, 0.90, 90, null));
		MATERIALS.put("funnel", new Mat(0.58, 0.13, 0.11, 0.25, 16, HULL_TEX));
		MATERIALS.put("funnel_cap", new Mat(0.10, 0.10, 0.10, 0.20, 12, null));
		MATERIALS.put("metal_dark", new Mat(0.17, 0.17, 0.18, 0.30, 20, null));
		MATERIALS.put("metal_light", new Mat(0.72, 0.73, 0.72, 0.40, 30, null));
		MATERIALS.put("Metal_Chrome", new Mat(0.55, 0.52, 0.42, 0.90, 40, null));
		MATERIALS.put("lifeboat", new Mat(0.92, 0.42, 0.06, 0.35, 24, null));
		MATERIALS.put("crane", new Mat(0.82, 0.74, 0.30, 0.25, 16, HULL_TEX));
		MATERIALS.put("c_blue", new Mat(0.10, 0.22, 0.50, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_red", new Mat(0.60, 0.14, 0.10, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_green", new Mat(0.10, 0.38, 0.22, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_grey", new Mat(0.55, 0.56, 0.56, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_rust", new Mat(0.48, 0.28, 0.14, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_white", new Mat(0.86, 0.86, 0.84, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_orange", new Mat(0.78, 0.36, 0.08, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_navy", new Mat(0.08, 0.12, 0.30, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_maroon", new Mat(0.40, 0.10, 0.12, 0.15, 10, CONTAINER_TEX));
		MATERIALS.put("c_teal", new Mat(0.10, 0.40, 0.42, 0.15, 10, CONTAINER_TEX));
	}

	private static final String[] CONTAINER_COLOURS = {"c_blue", "c_red", "c_green", "c_grey", "c_rust", "c_white",
			"c_orange", "c_navy", "c_maroon", "c_teal", "c_blue", "c_red", "c_grey", "c_navy"};

	private final List<double[]> verts = new ArrayList<>();
	private final List<double[]> uvs = new ArrayList<>();
	private final List<Group> groups = new ArrayList<>();
	private final Map<String, Group> groupByName = new HashMap<>();
	private final Random rng = new Random(20260909L);

	public static void main(String[] args) throws IOException {
		if (args.length < 1) {
			System.err.println("Usage: ShipModelGenerator <out-dir>");
			System.exit(2);
		}
		Path out = Path.of(args[0]);
		Files.createDirectories(out);
		var gen = new ShipModelGenerator();
		gen.buildHull();
		gen.buildBulb();
		gen.buildForecastleAndStern();
		gen.buildRailings();
		gen.buildCargo();
		gen.buildSuperstructure();
		gen.buildCranes();
		gen.buildSternGear();
		int tris = gen.write(out.resolve("surface-ship.obj"), out.resolve("surface-ship.mtl"));
		ImageIO.write(hullTexture(), "png", out.resolve(HULL_TEX).toFile());
		ImageIO.write(containerTexture(), "png", out.resolve(CONTAINER_TEX).toFile());
		System.out.printf(Locale.ROOT, "vertices=%d triangles=%d -> %s%n", gen.verts.size(), tris, out);
	}

	// ── Mesh primitives ──────────────────────────────────────────────────────

	/**
	 * A named OBJ group with one material. Smooth groups share per-vertex normals; others get flat
	 * normals.
	 */
	private final class Group {
		final String name;
		final String material;
		final boolean smooth;
		final List<int[]> tris = new ArrayList<>();

		Group(String name, String material, boolean smooth) {
			this.name = name;
			this.material = material;
			this.smooth = smooth;
			groups.add(this);
			groupByName.put(name, this);
		}

		/**
		 * Adds a triangle of 1-based vertex indices. If {@code inside} is a point known to be
		 * inside the part, the winding is flipped so the face normal points away from it.
		 * Degenerate faces are dropped.
		 */
		void tri(int a, int b, int c, double[] inside) {
			double[] p0 = verts.get(a - 1), p1 = verts.get(b - 1), p2 = verts.get(c - 1);
			double[] n = cross(sub(p1, p0), sub(p2, p0));
			if (dot(n, n) < 1e-8)
				return;
			if (inside != null) {
				double[] centroid = {(p0[0] + p1[0] + p2[0]) / 3, (p0[1] + p1[1] + p2[1]) / 3,
						(p0[2] + p1[2] + p2[2]) / 3};
				if (dot(n, sub(centroid, inside)) < 0) {
					int t = a;
					a = c;
					c = t;
				}
			}
			tris.add(new int[] {a, b, c});
		}

		void quad(int a, int b, int c, int d, double[] inside) {
			tri(a, b, c, inside);
			tri(a, c, d, inside);
		}
	}

	private Group group(String name, String material, boolean smooth) {
		Group g = groupByName.get(name);
		return g != null ? g : new Group(name, material, smooth);
	}

	private int v(double x, double y, double z) {
		return v(x, y, z, 0.0, 0.0);
	}

	private int v(double x, double y, double z, double u, double w) {
		verts.add(new double[] {x, y, z});
		uvs.add(new double[] {u, w});
		return verts.size();
	}

	private static double[] sub(double[] a, double[] b) {
		return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
	}

	private static double[] add(double[] a, double[] b) {
		return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
	}

	private static double[] scale(double[] a, double s) {
		return new double[] {a[0] * s, a[1] * s, a[2] * s};
	}

	private static double[] cross(double[] a, double[] b) {
		return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
	}

	private static double dot(double[] a, double[] b) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	private static double[] normalize(double[] a) {
		double len = Math.sqrt(dot(a, a));
		return len > 0 ? new double[] {a[0] / len, a[1] / len, a[2] / len} : new double[] {0, 0, 0};
	}

	private static double clamp01(double x) {
		return Math.max(0.0, Math.min(1.0, x));
	}

	/** Quad from four explicit points with explicit texture coordinates. */
	private void quadP(Group g, double[] inside, double[][] p, double[][] uv) {
		g.quad(v(p[0][0], p[0][1], p[0][2], uv[0][0], uv[0][1]), v(p[1][0], p[1][1], p[1][2], uv[1][0], uv[1][1]),
				v(p[2][0], p[2][1], p[2][2], uv[2][0], uv[2][1]), v(p[3][0], p[3][1], p[3][2], uv[3][0], uv[3][1]),
				inside);
	}

	/**
	 * Plating texture coordinates for a point on a face whose normal is dominated by {@code axis}
	 * (0 = x, 1 = y, 2 = z): world metres divided by the tile size, so seams line up across
	 * adjacent parts.
	 */
	private static double[] platedUv(double[] p, int axis) {
		return switch (axis) {
		case 0 -> new double[] {p[1] / PLATE_U, p[2] / PLATE_V};
		case 1 -> new double[] {p[0] / PLATE_U, p[2] / PLATE_V};
		default -> new double[] {p[1] / PLATE_U, p[0] / PLATE_V};
		};
	}

	private static int dominantAxis(double[] n) {
		double ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2]);
		return ax >= ay && ax >= az ? 0 : (ay >= az ? 1 : 2);
	}

	/**
	 * Quad from four points with plating texture coordinates projected along its dominant normal
	 * axis.
	 */
	private void quadPlated(Group g, double[] inside, double[][] p) {
		int axis = dominantAxis(cross(sub(p[1], p[0]), sub(p[2], p[0])));
		quadP(g, inside, p, new double[][] {platedUv(p[0], axis), platedUv(p[1], axis), platedUv(p[2], axis),
				platedUv(p[3], axis)});
	}

	/**
	 * Hexahedron from eight corners: {@code c[0..3]} the bottom ring (x0y0, x1y0, x1y1, x0y1 for a
	 * box), {@code c[4..7]} the top ring in the same order. Unshared vertices so edges stay crisp;
	 * plated UVs when {@code plated}.
	 */
	private void hexa(Group g, double[][] c, boolean plated) {
		double[] centre = {0, 0, 0};
		for (double[] p : c)
			centre = add(centre, p);
		centre = scale(centre, 1.0 / 8);
		int[][] faces = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 2, 6, 7}, {0, 3, 7, 4}, {1, 2, 6, 5}};
		for (int[] f : faces) {
			double[][] p = {c[f[0]], c[f[1]], c[f[2]], c[f[3]]};
			if (plated)
				quadPlated(g, centre, p);
			else
				quadP(g, centre, p, new double[][] {{0, 0}, {0, 0}, {0, 0}, {0, 0}});
		}
	}

	private static double[][] boxCorners(double x0, double x1, double y0, double y1, double z0, double z1) {
		return new double[][] {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}, {x0, y0, z1}, {x1, y0, z1},
				{x1, y1, z1}, {x0, y1, z1}};
	}

	/** Axis-aligned box without texture coordinates. */
	private void box(Group g, double x0, double x1, double y0, double y1, double z0, double z1) {
		hexa(g, boxCorners(x0, x1, y0, y1, z0, z1), false);
	}

	/** Axis-aligned box with plating texture coordinates. */
	private void boxPlated(Group g, double x0, double x1, double y0, double y1, double z0, double z1) {
		hexa(g, boxCorners(x0, x1, y0, y1, z0, z1), true);
	}

	/**
	 * Box of width {@code w} and height {@code h} running from {@code a} to {@code b}; {@code up}
	 * hints which way the height axis points. Used for crane jibs, davits and railings that are not
	 * axis-aligned.
	 */
	private void beam(Group g, double[] a, double[] b, double w, double h, double[] up) {
		double[] dir = normalize(sub(b, a));
		double[] side = normalize(cross(dir, up));
		if (dot(side, side) < 1e-9)
			side = normalize(cross(dir, new double[] {1, 0, 0}));
		double[] upv = normalize(cross(side, dir));
		double[] sw = scale(side, w / 2), uh = scale(upv, h / 2);
		double[][] c = {sub(sub(a, sw), uh), sub(add(a, sw), uh), sub(add(b, sw), uh), sub(sub(b, sw), uh),
				add(sub(a, sw), uh), add(add(a, sw), uh), add(add(b, sw), uh), add(sub(b, sw), uh)};
		hexa(g, c, false);
	}

	/** Vertical elliptical prism (pedestals, pipes). */
	private void prismZ(Group g, double cx, double cy, double rx, double ry, double z0, double z1, int n) {
		double[] c = {cx, cy, (z0 + z1) / 2};
		int[] ring0 = new int[n], ring1 = new int[n];
		for (int i = 0; i < n; i++) {
			double a = 2 * Math.PI * i / n;
			ring0[i] = v(cx + rx * Math.cos(a), cy + ry * Math.sin(a), z0);
			ring1[i] = v(cx + rx * Math.cos(a), cy + ry * Math.sin(a), z1);
		}
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			g.quad(ring0[i], ring0[j], ring1[j], ring1[i], c);
		}
		int top = v(cx, cy, z1), bottom = v(cx, cy, z0);
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			g.tri(top, ring1[i], ring1[j], c);
			g.tri(bottom, ring0[j], ring0[i], c);
		}
	}

	/** Fore-aft cylinder (shaft, propeller hub, lifeboats). */
	private void prismY(Group g, double cx, double cz, double r, double y0, double y1, int n) {
		double[] c = {cx, (y0 + y1) / 2, cz};
		int[] ring0 = new int[n], ring1 = new int[n];
		for (int i = 0; i < n; i++) {
			double a = 2 * Math.PI * i / n;
			ring0[i] = v(cx + r * Math.cos(a), y0, cz + r * Math.sin(a));
			ring1[i] = v(cx + r * Math.cos(a), y1, cz + r * Math.sin(a));
		}
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			g.quad(ring0[i], ring0[j], ring1[j], ring1[i], c);
		}
		int fore = v(cx, y0, cz), aft = v(cx, y1, cz);
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			g.tri(fore, ring0[j], ring0[i], c);
			g.tri(aft, ring1[i], ring1[j], c);
		}
	}

	// ── Hull lines ───────────────────────────────────────────────────────────
	// t runs 0 (stem) to 1 (transom).

	/** Half-breadth at deck level: fine entry, parallel midbody, tapered run. */
	private static double halfBreadthAtDeck(double t) {
		if (t < 0.40) {
			double u = t / 0.40;
			return B / 2 * (1 - Math.pow(1 - u, 2.0));
		}
		if (t < 0.74)
			return B / 2;
		double u = (t - 0.74) / 0.26;
		return B / 2 * (1 - 0.5 * u * u);
	}

	private static double keelZ(double t) {
		// Forefoot rises to the waterline at the stem. Linear: any steeper start (a power below 1) drops
		// the keel so fast that the raked sections fold back on themselves next to the stem.
		if (t < 0.15)
			return -T * t / 0.15;
		if (t <= 0.70)
			return -T;
		return -T + (T - 1.0) * Math.pow((t - 0.70) / 0.30, 1.2); // run rises to the transom (-1 m)
	}

	private static double deckZ(double t) {
		double bowSheer = clamp01((0.30 - t) / 0.30), sternSheer = clamp01((t - 0.80) / 0.20);
		return D + 2.5 * bowSheer * bowSheer + 0.8 * sternSheer * sternSheer;
	}

	/**
	 * Fraction of the deck half-breadth that is flat bottom: V-shaped forward, narrow deadwood aft.
	 */
	private static double keelFlat(double t) {
		if (t < 0.30)
			return 0.45 * Math.pow(t / 0.30, 1.5);
		if (t <= 0.70)
			return 0.45;
		return 0.45 * (1 - 0.8 * Math.pow((t - 0.70) / 0.30, 1.2));
	}

	private static double flare(double t) {
		return 0.04 + 0.10 * clamp01((0.25 - t) / 0.25);
	}

	/** Stem leans forward: the deck point is 9 m ahead of the forefoot. */
	private static double rake(double t) {
		return 9.0 * clamp01(1 - t / 0.10);
	}

	private static double yBase(double t) {
		return -L / 2 + L * t;
	}

	/** Half-breadth of the hull at station {@code t} and height {@code z}. */
	private static double halfBreadth(double t, double z) {
		double zk = keelZ(t), zd = deckZ(t);
		double k = keelFlat(t), hbDeck = halfBreadthAtDeck(t), fl = flare(t);
		double hbWl = hbDeck / (1 + fl);
		if (z <= 0) {
			double u = zk < -1e-6 ? clamp01((z - zk) / (0 - zk)) : 1.0;
			double frac = k + (1 - k) * Math.pow(1 - Math.pow(1 - u, 2.2), 1 / 2.2);
			return hbWl * frac;
		}
		double w = zd > 1e-6 ? clamp01(z / zd) : 0.0;
		return hbWl * (1 + fl * w);
	}

	/**
	 * Fore-aft position of the hull surface at station {@code t} and height {@code z} (the stem is
	 * raked).
	 */
	private static double yAt(double t, double z) {
		double zk = keelZ(t), zd = deckZ(t);
		double frac = zd > zk ? clamp01((z - zk) / (zd - zk)) : 1.0;
		return yBase(t) + rake(t) * (1 - frac);
	}

	/**
	 * The section heights used at every station: six steps from the keel to the lower boot-top
	 * edge, then the waterline, the upper boot-top edge, and four steps to the deck. Clamped so
	 * levels collapse (and their quads degenerate away) near the stem where the keel rises above
	 * the band.
	 */
	private static double[] levels(double t) {
		double zk = keelZ(t), zd = deckZ(t);
		double[] zs = new double[LEVELS];
		double zb0 = Math.min(Math.max(zk, BOOT_LO), zd);
		for (int i = 0; i <= 6; i++)
			zs[i] = zk + (zb0 - zk) * i / 6;
		zs[7] = Math.min(Math.max(0.0, zb0), zd);
		zs[8] = Math.min(Math.max(BOOT_HI, zs[7]), zd);
		for (int i = 1; i <= 4; i++)
			zs[8 + i] = zs[8] + (zd - zs[8]) * i / 4;
		return zs;
	}

	private int hullVertex(Map<String, Integer> shared, double x, double y, double z) {
		double u = (y + L / 2) / PLATE_U, w = (z + T) / PLATE_V;
		if (Math.abs(x) < 1e-6) {
			String key = String.format(Locale.ROOT, "%.4f,%.4f", y, z);
			return shared.computeIfAbsent(key, kk -> v(0.0, y, z, u, w));
		}
		return v(x, y, z, u, w);
	}

	/**
	 * One hull section as a ring of vertex indices: port deck edge down to the keel, then up the
	 * starboard side. Centreline points are shared between the two sides so the smooth normal there
	 * points forward instead of leaving a mirrored crease.
	 */
	private int[] stationRing(double t) {
		double[] zs = levels(t);
		double[][] pts = new double[LEVELS][];
		for (int i = 0; i < LEVELS; i++)
			pts[i] = new double[] {halfBreadth(t, zs[i]), yAt(t, zs[i]), zs[i]};
		Map<String, Integer> shared = new HashMap<>();
		int[] ring = new int[2 * LEVELS - 1];
		int idx = 0;
		for (int i = LEVELS - 1; i >= 1; i--)
			ring[idx++] = hullVertex(shared, -pts[i][0], pts[i][1], pts[i][2]);
		ring[idx++] = hullVertex(shared, 0.0, pts[0][1], pts[0][2]);
		for (int i = 1; i < LEVELS; i++)
			ring[idx++] = hullVertex(shared, pts[i][0], pts[i][1], pts[i][2]);
		return ring;
	}

	private static double[] sectionCentre(double t) {
		return new double[] {0.0, yBase(t), (keelZ(t) + deckZ(t)) / 2};
	}

	private Group paintFor(double z, Group red, Group boot, Group dark) {
		return z < BOOT_LO ? red : (z < BOOT_HI ? boot : dark);
	}

	private void buildHull() {
		Group hullRed = new Group("HullBelow", "hull_red", true);
		Group hullBoot = new Group("HullBoot", "hull_boot", true);
		Group hullDark = new Group("HullAbove", "hull_dark", true);
		Group deck = new Group("Deck", "deck", false);
		Group bulwark = new Group("Bulwark", "hull_dark", false);
		// Cosine-spaced stations: dense at the bow and stern where the curvature is high.
		int stations = 80;
		int[][] rings = new int[stations + 1][];
		double[] ts = new double[stations + 1];
		for (int i = 0; i <= stations; i++) {
			ts[i] = 0.5 * (1 - Math.cos(Math.PI * i / stations));
			rings[i] = stationRing(ts[i]);
		}
		int m = rings[0].length;
		for (int i = 0; i < stations; i++) {
			int[] r0 = rings[i], r1 = rings[i + 1];
			double[] c0 = sectionCentre(ts[i]), c1 = sectionCentre(ts[i + 1]);
			double[] inside = {0.0, (c0[1] + c1[1]) / 2, (c0[2] + c1[2]) / 2};
			// Rings run port deck -> keel -> starboard deck and stations run bow -> stern, so this order is
			// outward everywhere. No inside-point test: with the raked stem the plating near the forefoot lies
			// aft of the station centre, which would flip those faces inward. The starboard half splits its quads
			// along the mirrored diagonal so the smoothed normals on the centreline come out symmetric.
			for (int j = 0; j < m - 1; j++) {
				double zc = (verts.get(r0[j] - 1)[2] + verts.get(r0[j + 1] - 1)[2] + verts.get(r1[j] - 1)[2]
						+ verts.get(r1[j + 1] - 1)[2]) / 4;
				Group paint = paintFor(zc, hullRed, hullBoot, hullDark);
				if (j < LEVELS - 1)
					paint.quad(r0[j], r1[j], r1[j + 1], r0[j + 1], null);
				else
					paint.quad(r1[j], r1[j + 1], r0[j + 1], r0[j], null);
			}
			// Deck strip between the two deck edges (own vertices: plated in plan, flat shaded)
			double[] p0 = verts.get(r0[0] - 1), p1 = verts.get(r0[m - 1] - 1), p2 = verts.get(r1[m - 1] - 1),
					p3 = verts.get(r1[0] - 1);
			quadPlated(deck, new double[] {0.0, inside[1], inside[2] - 5}, new double[][] {p0, p1, p2, p3});
			// Bulwark on the forecastle and the aft mooring deck: a thin plate rising from the deck edge
			double tm = (ts[i] + ts[i + 1]) / 2;
			if (tm < 0.31 || tm > 0.86) {
				double h = 1.15;
				for (double[][] side : new double[][][] {{p0, p3}, {p1, p2}}) {
					double[] a = side[0], b = side[1];
					double sign = a[0] < 0 ? -1 : 1;
					double[][] q = {a, b, {b[0], b[1], b[2] + h}, {a[0], a[1], a[2] + h}};
					quadPlated(bulwark, new double[] {a[0] - sign * 3, (a[1] + b[1]) / 2, a[2]}, q);
					// Cap rail along the top edge
					beam(group("BulwarkRail", "metal_light", false), new double[] {a[0], a[1], a[2] + h},
							new double[] {b[0], b[1], b[2] + h}, 0.16, 0.10, new double[] {0, 0, 1});
				}
			}
		}
		// Transom: fan from its centre. It is flat and meets the sides at a hard corner, so it gets its own
		// copies of the last ring's vertices in flat groups; sharing them would smooth the corner into a gradient.
		Group transomRed = new Group("TransomBelow", "hull_red", false);
		Group transomBoot = new Group("TransomBoot", "hull_boot", false);
		Group transomDark = new Group("TransomAbove", "hull_dark", false);
		int[] last = new int[rings[stations].length];
		for (int j = 0; j < last.length; j++) {
			int i = rings[stations][j];
			double[] p = verts.get(i - 1), uv = uvs.get(i - 1);
			last[j] = v(p[0], p[1], p[2], uv[0], uv[1]);
		}
		double cx = 0, cy = 0, cz = 0;
		for (int i : last) {
			cx += verts.get(i - 1)[0];
			cy += verts.get(i - 1)[1];
			cz += verts.get(i - 1)[2];
		}
		cx /= last.length;
		cy /= last.length;
		cz /= last.length;
		int centre = hullVertex(new HashMap<>(), cx, cy, cz);
		double[] inside = {0.0, cy - 5.0, cz};
		for (int j = 0; j < last.length - 1; j++) {
			double zc = (verts.get(last[j] - 1)[2] + verts.get(last[j + 1] - 1)[2]) / 2;
			paintFor(zc, transomRed, transomBoot, transomDark).tri(centre, last[j], last[j + 1], inside);
		}
		transomDark.tri(centre, last[last.length - 1], last[0], inside); // close across the deck edge
		// Transom bulwark across the stern
		double[] pl = verts.get(last[0] - 1), pr = verts.get(last[last.length - 1] - 1);
		quadPlated(bulwark, new double[] {0, pl[1] - 3, pl[2]},
				new double[][] {pl, pr, {pr[0], pr[1], pr[2] + 1.15}, {pl[0], pl[1], pl[2] + 1.15}});
	}

	/** Bulbous bow: a spindle of revolution below the waterline, protruding ahead of the stem. */
	private void buildBulb() {
		Group g = new Group("Bulb", "hull_red", true);
		double tipY = -L / 2 - 5.5, rootY = -L / 2 + 15.0, zc = -4.2, R = 2.3;
		int along = 12, around = 16;
		int[][] rings = new int[along + 1][around];
		for (int i = 0; i <= along; i++) {
			double s = (double) i / along; // 0 tip .. 1 root
			double r = R * Math.sin(Math.PI * Math.pow(s, 0.55));
			double y = tipY + (rootY - tipY) * s;
			double zOff = 0.6 * s; // drifts up slightly as it merges into the forefoot
			for (int j = 0; j < around; j++) {
				double a = 2 * Math.PI * j / around;
				rings[i][j] = v(r * Math.cos(a), y, zc + zOff + r * Math.sin(a), (y + L / 2) / PLATE_U,
						(zc + r * Math.sin(a) + T) / PLATE_V);
			}
		}
		for (int i = 0; i < along; i++) {
			double[] inside = {0, tipY + (rootY - tipY) * (i + 0.5) / along, zc};
			for (int j = 0; j < around; j++) {
				int k = (j + 1) % around;
				g.quad(rings[i][j], rings[i][k], rings[i + 1][k], rings[i + 1][j], inside);
			}
		}
	}

	/** Anchors, windlass, mooring winches and the small masts fore and aft. */
	private void buildForecastleAndStern() {
		Group dark = group("DeckGear", "metal_dark", false);
		Group white = group("DeckHouses", "superstructure", false);
		// Hawse pipes and anchors: dark plates proud of the bow plating on both sides
		double t = 0.045, z = deckZ(t) - 2.4;
		double hb = halfBreadth(t, z), y = yAt(t, z);
		for (double s : new double[] {-1, 1}) {
			box(dark, s * (hb - 0.05) - 0.2, s * (hb + 0.25) + 0.2, y - 0.6, y + 0.6, z - 0.7, z + 0.7);
			box(dark, s * (hb - 0.05) - 0.35, s * (hb + 0.35) + 0.35, y - 0.4, y + 0.4, z - 2.2, z - 0.8);
		}
		// Windlass house and mooring winches on the forecastle
		boxPlated(white, -3.2, 3.2, -66.5, -61.0, deckZ(0.06), deckZ(0.06) + 2.6);
		for (double s : new double[] {-1, 1}) {
			prismY(dark, s * 5.0, deckZ(0.09) + 0.9, 0.8, -60.0, -57.0, 10);
			box(dark, s * 5.0 - 1.2, s * 5.0 + 1.2, -60.4, -56.6, deckZ(0.09), deckZ(0.09) + 0.5);
		}
		// Foremast with a yard and a radar reflector
		Group mast = group("Masts", "metal_dark", false);
		box(mast, -0.3, 0.3, -58.3, -57.7, deckZ(0.09), deckZ(0.09) + 13.0);
		box(mast, -3.0, 3.0, -58.15, -57.85, deckZ(0.09) + 10.5, deckZ(0.09) + 10.8);
		box(mast, -0.5, 0.5, -58.5, -57.5, deckZ(0.09) + 13.0, deckZ(0.09) + 13.6);
		// Aft mooring deck: winches and a stern mast
		double za = deckZ(0.95);
		for (double s : new double[] {-1, 1}) {
			prismY(dark, s * 6.0, za + 0.9, 0.8, 67.0, 70.0, 10);
			box(dark, s * 6.0 - 1.2, s * 6.0 + 1.2, 66.6, 70.4, za, za + 0.5);
			box(dark, s * 10.0 - 0.6, s * 10.0 + 0.6, 71.0, 72.2, za, za + 1.2); // fairleads
		}
		box(mast, -0.25, 0.25, 71.7, 72.3, za, za + 8.0);
		box(mast, -2.2, 2.2, 71.85, 72.15, za + 6.5, za + 6.8);
	}

	/**
	 * Railings with stanchions along the main deck edges between the forecastle and the deckhouse.
	 */
	private void buildRailings() {
		Group rail = group("Railings", "metal_light", false);
		double[] up = {0, 0, 1};
		for (double s : new double[] {-1, 1}) {
			double[] prev = null;
			for (double y = -49.0; y <= 63.0; y += 2.4) {
				double t = (y + L / 2) / L;
				double zd = deckZ(t);
				double x = s * (halfBreadth(t, zd) - 0.35);
				double[] p = {x, yAt(t, zd), zd};
				box(rail, x - 0.05, x + 0.05, p[1] - 0.05, p[1] + 0.05, zd, zd + RAIL_H);
				if (prev != null) {
					beam(rail, new double[] {prev[0], prev[1], prev[2] + RAIL_H},
							new double[] {p[0], p[1], p[2] + RAIL_H}, 0.08, 0.08, up);
					beam(rail, new double[] {prev[0], prev[1], prev[2] + RAIL_H * 0.55},
							new double[] {p[0], p[1], p[2] + RAIL_H * 0.55}, 0.05, 0.05, up);
				}
				prev = p;
			}
		}
	}

	// ── Cargo ────────────────────────────────────────────────────────────────

	private static boolean isCraneBay(int bay) {
		for (int b : CRANE_BAYS)
			if (b == bay)
				return true;
		return false;
	}

	/** Width of the gap ahead of {@code bay} (bay 0 has none). */
	private static double gapBefore(int bay) {
		return bay == 0 ? 0.0 : (isCraneBay(bay) ? CRANE_GAP : BAY_GAP);
	}

	/** Fore end of {@code bay}'s container stack. */
	private static double bayY0(int bay) {
		double y = BAY0_Y;
		for (int b = 1; b <= bay; b++)
			y += TEU_L + gapBefore(b);
		return y;
	}

	private int rowsAt(double y) {
		double t = (y + L / 2) / L;
		double hb = halfBreadth(t, deckZ(t));
		return Math.min(11, (int) Math.floor(2 * (hb - 2.4) / (TEU_W + 0.08)));
	}

	/**
	 * A 40 ft container with the corrugation atlas: sides and roof from the side region, doors on
	 * the aft end.
	 */
	private void container(
		Group g, double x0, double x1, double y0, double y1, double z0, double z1, boolean[] visible) {
		double[] c = {(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2};
		double su = CONTAINER_SIDE_U, du = 1.0 - CONTAINER_SIDE_U;
		// Faces: 0 bottom, 1 top, 2 fore, 3 aft (doors), 4 port, 5 starboard
		if (visible[4])
			quadP(g, c, new double[][] {{x0, y0, z0}, {x0, y1, z0}, {x0, y1, z1}, {x0, y0, z1}},
					new double[][] {{0, 0}, {su, 0}, {su, 1}, {0, 1}});
		if (visible[5])
			quadP(g, c, new double[][] {{x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}},
					new double[][] {{0, 0}, {su, 0}, {su, 1}, {0, 1}});
		if (visible[1])
			quadP(g, c, new double[][] {{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}},
					new double[][] {{0, 0}, {0, 1}, {su, 1}, {su, 0}});
		if (visible[0])
			quadP(g, c, new double[][] {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}},
					new double[][] {{0, 0}, {0, 1}, {su, 1}, {su, 0}});
		if (visible[2])
			quadP(g, c, new double[][] {{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}},
					new double[][] {{0, 0}, {0.15, 0}, {0.15, 1}, {0, 1}});
		if (visible[3])
			quadP(g, c, new double[][] {{x0, y1, z0}, {x1, y1, z0}, {x1, y1, z1}, {x0, y1, z1}},
					new double[][] {{su, 0}, {su + du, 0}, {su + du, 1}, {su, 1}});
	}

	private void buildCargo() {
		Group coaming = group("HatchCoamings", "hatch", false);
		Group cover = group("HatchCovers", "hatch", false);
		Group lashing = group("LashingBridges", "metal_dark", false);
		int[] bayTiers = {3, 4, 5, 5, 5, 4, 4};
		double pitch = TEU_W + 0.08;
		double[] up = {0, 0, 1};
		double prevHalf = 0;
		for (int bay = 0; bay < BAYS; bay++) {
			double y0 = bayY0(bay), y1 = y0 + TEU_L;
			int rows = rowsAt((y0 + y1) / 2);
			double half = rows * pitch / 2 + 0.35;
			double zDeck = D;
			boxPlated(coaming, -half, half, y0 - 0.3, y1 + 0.3, zDeck, zDeck + COAMING_H);
			boxPlated(cover, -half - 0.25, half + 0.25, y0 - 0.45, y1 + 0.45, zDeck + COAMING_H,
					zDeck + COAMING_H + HATCH_COVER_H);
			// Stack plan: bay-wide tier count, outer columns a step lower, random top-tier holes.
			int tiers = bayTiers[bay];
			int[] height = new int[rows];
			for (int r = 0; r < rows; r++) {
				int h = tiers;
				if (r == 0 || r == rows - 1)
					h -= 1;
				if (rng.nextDouble() < 0.18)
					h -= 1;
				if (rng.nextDouble() < 0.06)
					h -= 1;
				height[r] = Math.max(1, h);
			}
			String[][] colour = new String[rows][tiers];
			for (int r = 0; r < rows; r++)
				for (int k = 0; k < height[r]; k++)
					colour[r][k] = CONTAINER_COLOURS[rng.nextInt(CONTAINER_COLOURS.length)];
			for (int r = 0; r < rows; r++) {
				double x0 = -rows * pitch / 2 + r * pitch + 0.04, x1 = x0 + TEU_W;
				for (int k = 0; k < height[r]; k++) {
					double z0 = CONTAINER_BASE + k * TEU_H;
					boolean[] visible = {k == 0, k == height[r] - 1, true, true, r == 0 || height[r - 1] <= k,
							r == rows - 1 || height[r + 1] <= k};
					visible[0] = false; // bottoms sit on the hatch cover
					container(group("Containers_" + colour[r][k], colour[r][k], false), x0, x1, y0 + 0.05, y1 - 0.05,
							z0, z0 + TEU_H, visible);
				}
			}
			// Lashing bridge in the gap ahead of this bay (from bay 1 on, not where a crane stands): platform two
			// tiers up, posts, hand rail
			if (bay > 0 && !isCraneBay(bay)) {
				double gy = y0 - BAY_GAP / 2, w = Math.max(prevHalf, half) - 0.3;
				double zp = CONTAINER_BASE + 2 * TEU_H - 0.25;
				box(lashing, -w, w, gy - 0.35, gy + 0.35, zp, zp + 0.25);
				for (double x = -w + 1.0; x < w; x += 4.0)
					box(lashing, x - 0.15, x + 0.15, gy - 0.15, gy + 0.15, D + COAMING_H, zp);
				beam(lashing, new double[] {-w, gy, zp + 1.0}, new double[] {w, gy, zp + 1.0}, 0.06, 0.06, up);
				for (double x = -w; x <= w; x += 2.5)
					box(lashing, x - 0.04, x + 0.04, gy - 0.04, gy + 0.04, zp + 0.25, zp + 1.0);
			}
			prevHalf = half;
		}
	}

	// ── Deckhouse ────────────────────────────────────────────────────────────

	/**
	 * Row of dark window panes proud of a face. {@code along} is 0 for a face spanning x, 1 for one
	 * spanning y.
	 */
	private void windows(
		Group glass, int along, double a0, double a1, double fixed, double z0, double z1, double paneW, double gap,
		boolean plusSide) {
		double span = a1 - a0, step = paneW + gap;
		int n = (int) Math.floor((span - gap) / step);
		if (n <= 0)
			return;
		double start = a0 + (span - (n * step - gap)) / 2;
		double f0 = plusSide ? fixed : fixed - 0.08, f1 = plusSide ? fixed + 0.08 : fixed;
		for (int i = 0; i < n; i++) {
			double p0 = start + i * step, p1 = p0 + paneW;
			if (along == 0)
				box(glass, p0, p1, f0, f1, z0, z1);
			else
				box(glass, f0, f1, p0, p1, z0, z1);
		}
	}

	/** Railing (top rail + stanchions) along a straight run between two points. */
	private void railRun(Group rail, double[] a, double[] b) {
		double[] up = {0, 0, 1};
		beam(rail, new double[] {a[0], a[1], a[2] + RAIL_H}, new double[] {b[0], b[1], b[2] + RAIL_H}, 0.07, 0.07, up);
		beam(rail, new double[] {a[0], a[1], a[2] + RAIL_H * 0.5}, new double[] {b[0], b[1], b[2] + RAIL_H * 0.5}, 0.04,
				0.04, up);
		double len = Math.sqrt(dot(sub(b, a), sub(b, a)));
		int n = Math.max(1, (int) Math.round(len / 1.8));
		for (int i = 0; i <= n; i++) {
			double[] p = add(a, scale(sub(b, a), (double) i / n));
			box(rail, p[0] - 0.04, p[0] + 0.04, p[1] - 0.04, p[1] + 0.04, p[2], p[2] + RAIL_H);
		}
	}

	private void buildSuperstructure() {
		Group white = group("Superstructure", "superstructure", false);
		Group glass = group("Windows", "glass", false);
		Group rail = group("HouseRailings", "metal_light", false);
		Group mast = group("Masts", "metal_dark", false);
		double hx = HOUSE_HALF, y0 = HOUSE_Y0, y1 = HOUSE_Y1;
		// Accommodation tiers with external walkways and railings on the tiers above the main deck
		for (int tier = 0; tier < HOUSE_TIERS; tier++) {
			double z0 = D + tier * TIER_H, z1 = z0 + TIER_H;
			boxPlated(white, -hx, hx, y0, y1, z0, z1);
			windows(glass, 0, -hx + 1.0, hx - 1.0, y0, z0 + 1.2, z0 + 2.2, 0.8, 0.9, false);
			windows(glass, 1, y0 + 1.0, y1 - 1.0, hx, z0 + 1.2, z0 + 2.2, 0.8, 1.2, true);
			windows(glass, 1, y0 + 1.0, y1 - 1.0, -hx, z0 + 1.2, z0 + 2.2, 0.8, 1.2, false);
			windows(glass, 0, -hx + 1.0, hx - 1.0, y1, z0 + 1.2, z0 + 2.2, 0.8, 1.4, true);
			if (tier > 0) {
				double wx = hx + 1.0, wy0 = y0 - 1.0, wy1 = y1 + 1.0;
				boxPlated(white, -wx, wx, wy0, wy1, z0 - 0.18, z0);
				railRun(rail, new double[] {-wx, wy0, z0}, new double[] {wx, wy0, z0});
				railRun(rail, new double[] {-wx, wy1, z0}, new double[] {wx, wy1, z0});
				railRun(rail, new double[] {-wx, wy0, z0}, new double[] {-wx, wy1, z0});
				railRun(rail, new double[] {wx, wy0, z0}, new double[] {wx, wy1, z0});
			}
		}
		// Bridge deck: body plus wings out to the full beam, continuous window band on the front and wing faces
		double zb0 = D + HOUSE_TIERS * TIER_H, zb1 = zb0 + BRIDGE_H;
		double wingX = B / 2 + 1.0, wingY1 = y0 + 4.5;
		boxPlated(white, -wingX, wingX, y0 - 1.0, y1 + 1.0, zb0 - 0.18, zb0); // walkway/wing deck
		boxPlated(white, -hx, hx, y0, y1, zb0, zb1);
		boxPlated(white, hx, wingX, y0, wingY1, zb0, zb1);
		boxPlated(white, -wingX, -hx, y0, wingY1, zb0, zb1);
		windows(glass, 0, -wingX + 0.4, wingX - 0.4, y0, zb0 + 1.0, zb0 + 2.4, 1.3, 0.25, false);
		windows(glass, 1, y0 + 0.4, wingY1 - 0.4, wingX, zb0 + 1.0, zb0 + 2.4, 1.3, 0.25, true);
		windows(glass, 1, y0 + 0.4, wingY1 - 0.4, -wingX, zb0 + 1.0, zb0 + 2.4, 1.3, 0.25, false);
		windows(glass, 1, wingY1 + 0.6, y1 - 0.6, hx, zb0 + 1.0, zb0 + 2.2, 0.9, 1.0, true);
		windows(glass, 1, wingY1 + 0.6, y1 - 0.6, -hx, zb0 + 1.0, zb0 + 2.2, 0.9, 1.0, false);
		windows(glass, 0, -hx + 1.0, hx - 1.0, y1, zb0 + 1.0, zb0 + 2.2, 0.9, 1.2, true);
		railRun(rail, new double[] {-wingX, y0 - 1.0, zb0}, new double[] {wingX, y0 - 1.0, zb0});
		railRun(rail, new double[] {-wingX, y0 - 1.0, zb0}, new double[] {-wingX, y1 + 1.0, zb0});
		railRun(rail, new double[] {wingX, y0 - 1.0, zb0}, new double[] {wingX, y1 + 1.0, zb0});
		railRun(rail, new double[] {-wingX, y1 + 1.0, zb0}, new double[] {wingX, y1 + 1.0, zb0});
		// Monkey island: rails around the bridge roof, mast with radar scanners, crosstree and lights
		railRun(rail, new double[] {-hx, y0, zb1}, new double[] {hx, y0, zb1});
		railRun(rail, new double[] {-hx, y0, zb1}, new double[] {-hx, y1, zb1});
		railRun(rail, new double[] {hx, y0, zb1}, new double[] {hx, y1, zb1});
		railRun(rail, new double[] {-hx, y1, zb1}, new double[] {hx, y1, zb1});
		double my = y0 + 6.0;
		box(mast, -0.35, 0.35, my - 0.35, my + 0.35, zb1, zb1 + 12.0);
		box(white, -1.6, 1.6, my - 1.6, my + 1.6, zb1 + 3.0, zb1 + 3.3); // radar platform
		box(mast, -2.2, 2.2, my - 0.25, my + 0.25, zb1 + 3.6, zb1 + 4.0); // X-band scanner
		box(mast, -2.8, 2.8, my - 0.3, my + 0.3, zb1 + 6.0, zb1 + 6.5); // S-band scanner
		box(mast, -4.0, 4.0, my - 0.2, my + 0.2, zb1 + 9.5, zb1 + 9.9); // crosstree
		box(mast, -0.15, 0.15, my - 0.15, my + 0.15, zb1 + 12.0, zb1 + 14.5); // top light pole
		for (double s : new double[] {-1, 1}) // magnetic compass / signal light boxes on the wings
			box(white, s * (hx - 1.5) - 0.5, s * (hx - 1.5) + 0.5, y0 + 1.0, y0 + 2.0, zb1, zb1 + 1.2);
		// Engine casing and funnel aft of the house: raked top, dark cap, twin exhaust pipes
		Group funnel = group("Funnel", "funnel", false);
		Group cap = group("FunnelCap", "funnel_cap", false);
		double fx = 3.0, fy0 = 61.0, fy1 = 67.0, fz1 = zb1 + 4.5;
		boxPlated(white, -hx + 2.0, hx - 2.0, y1, fy1 + 0.5, D, D + TIER_H); // engine room casing base
		hexa(funnel,
				new double[][] {{-fx, fy0, D + TIER_H}, {fx, fy0, D + TIER_H}, {fx, fy1, D + TIER_H},
						{-fx, fy1, D + TIER_H}, {-fx, fy0 + 1.5, fz1 - 3.0}, {fx, fy0 + 1.5, fz1 - 3.0}, {fx, fy1, fz1},
						{-fx, fy1, fz1}},
				true);
		hexa(cap, new double[][] {{-fx - 0.1, fy0 + 1.5, fz1 - 3.0}, {fx + 0.1, fy0 + 1.5, fz1 - 3.0},
				{fx + 0.1, fy1 + 0.1, fz1}, {-fx - 0.1, fy1 + 0.1, fz1}, {-fx - 0.1, fy0 + 2.2, fz1 - 1.6},
				{fx + 0.1, fy0 + 2.2, fz1 - 1.6}, {fx + 0.1, fy1 + 0.1, fz1 + 1.4}, {-fx - 0.1, fy1 + 0.1, fz1 + 1.4}},
				false);
		prismZ(cap, -1.3, fy1 - 1.8, 0.55, 0.55, fz1, fz1 + 3.2, 10);
		prismZ(cap, 1.3, fy1 - 1.8, 0.55, 0.55, fz1, fz1 + 3.2, 10);
		// Lifeboats in davits on both sides at the third tier, plus a rescue boat platform aft
		Group boats = group("Lifeboats", "lifeboat", false);
		double bz = D + 2 * TIER_H + 1.4, bx = hx + 2.6, by0 = 51.0, by1 = 58.5;
		for (double s : new double[] {-1, 1}) {
			prismY(boats, s * bx, bz, 1.25, by0, by1, 10);
			box(white, s * bx - 1.0, s * bx + 1.0, by0 + 2.0, by1 - 2.0, bz + 0.9, bz + 1.5); // canopy ridge
			for (double y : new double[] {by0 + 1.2, by1 - 1.2}) { // davit arms from the house wall
				beam(mast, new double[] {s * hx, y, bz + 1.8}, new double[] {s * (bx + 0.6), y, bz + 3.2}, 0.25, 0.25,
						new double[] {0, 0, 1});
				box(mast, s * (bx + 0.6) - 0.15, s * (bx + 0.6) + 0.15, y - 0.15, y + 0.15, bz - 0.2, bz + 3.2);
			}
		}
		// Doors on the aft face of tier 0 and a stair tower on the port side
		box(glass, -hx + 0.5, -hx + 0.58, y0 + 3.0, y0 + 4.0, D + 0.1, D + 2.1);
		boxPlated(white, -hx - 1.5, -hx, y1 - 3.5, y1, D, zb0);
	}

	/** Two deck cranes on the port side, jibs stowed forward over the hatches. */
	private void buildCranes() {
		Group body = group("Cranes", "crane", false);
		Group dark = group("CraneGear", "metal_dark", false);
		Group glass = group("Windows", "glass", false);
		for (int bay : CRANE_BAYS) {
			double y = bayY0(bay) - CRANE_GAP / 2, x = -(B / 2 - 2.6);
			double zTop = D + 12.0;
			prismZ(body, x, y, CRANE_PEDESTAL_R, CRANE_PEDESTAL_R, D, zTop, 16);
			// Machinery house, no deeper than the gap between the container stacks
			boxPlated(body, x - 1.9, x + 1.9, y - 2.0, y + 2.0, zTop, zTop + 3.0);
			box(glass, x - 1.6, x + 1.6, y - 2.1, y - 2.02, zTop + 1.2, zTop + 2.4); // cab windows facing forward
			box(dark, x - 0.9, x + 0.9, y - 1.4, y + 1.4, zTop + 3.0, zTop + 3.6); // slew ring / hoist drum
			double angle = Math.toRadians(24);
			double[] root = {x, y - 1.6, zTop + 3.2};
			double[] tip = {x, root[1] - 30.0 * Math.cos(angle), root[2] + 30.0 * Math.sin(angle)};
			beam(body, root, tip, 1.1, 1.3, new double[] {0, 0, 1});
			beam(dark, new double[] {x, y - 0.8, zTop + 3.6},
					new double[] {x, root[1] - 12.0 * Math.cos(angle), root[2] + 12.0 * Math.sin(angle) + 0.4}, 0.12,
					0.12, new double[] {0, 0, 1}); // topping wire
			box(dark, x - 0.4, x + 0.4, tip[1] - 0.6, tip[1] + 0.6, tip[2] - 2.0, tip[2] - 0.6); // hook block
		}
	}

	private void buildSternGear() {
		// Skeg (deadwood) enclosing the shaft, then shaft, propeller and rudder
		box(new Group("Skeg", "hull_red", false), -0.8, 0.8, 50.0, 66.0, -5.9, -2.4);
		prismY(new Group("Shaft", "metal_dark", false), 0.0, -5.0, 0.35, 60.0, 66.6, 10);

		Group prop = new Group("Propeller", "Metal_Chrome", false);
		double hubY0 = 66.6, hubY1 = 69.2, hubZ = -5.0;
		prismY(prop, 0.0, hubZ, 0.75, hubY0, hubY1, 12);
		double hubY = (hubY0 + hubY1) / 2;
		double pitch = 4.2, rootRadius = 0.72, tipRadius = 2.75;
		int blades = 4, segments = 6;
		for (int b = 0; b < blades; b++) {
			double baseAngle = 2 * Math.PI * b / blades;
			int prevLeading = -1, prevTrailing = -1;
			for (int s = 0; s <= segments; s++) {
				double f = (double) s / segments;
				double r = rootRadius + (tipRadius - rootRadius) * f;
				double chord = 1.35 * (1 - 0.25 * f) * (1 - 0.65 * Math.max(0.0, f - 0.8) / 0.2); // rounded tip
				double angle = baseAngle + 0.18 * f; // slight sweep-back
				double radialX = Math.cos(angle), radialZ = Math.sin(angle);
				double tangentialX = -Math.sin(angle), tangentialZ = Math.cos(angle);
				double phi = Math.atan(pitch / (2 * Math.PI * r));
				double ex = tangentialX * Math.cos(phi), ey = Math.sin(phi), ez = tangentialZ * Math.cos(phi);
				double cx = radialX * r, cz = hubZ + radialZ * r;
				int leading = v(cx + ex * chord / 2, hubY + ey * chord / 2, cz + ez * chord / 2);
				int trailing = v(cx - ex * chord / 2, hubY - ey * chord / 2, cz - ez * chord / 2);
				if (prevLeading >= 0)
					prop.quad(prevLeading, prevTrailing, trailing, leading, null);
				prevLeading = leading;
				prevTrailing = trailing;
			}
		}

		// Semi-balanced rudder: about a quarter of the chord ahead of the stock, the rest trailing aft, so the
		// blade visibly swings when the viewer rotates the group about the stock (SubmarineScene3D.SHIP_RUDDER_LOCAL).
		Group rudder = new Group("Rudder", "metal_dark", false);
		box(rudder, -0.35, 0.35, 70.6, 74.2, -6.6, -1.2);
		box(rudder, -0.3, 0.3, 71.2, 72.0, -1.2, 1.0); // stock into the hull, axis at y = 71.6
	}

	// ── Textures ─────────────────────────────────────────────────────────────

	/** Smooth value noise in [0, 1] on a coarse lattice, bilinearly interpolated. */
	private static float[] valueNoise(int w, int h, int cell, long seed) {
		Random r = new Random(seed);
		int gw = w / cell + 2, gh = h / cell + 2;
		float[] lattice = new float[gw * gh];
		for (int i = 0; i < lattice.length; i++)
			lattice[i] = r.nextFloat();
		float[] out = new float[w * h];
		for (int y = 0; y < h; y++) {
			int gy = y / cell;
			float fy = (float) (y % cell) / cell;
			fy = fy * fy * (3 - 2 * fy);
			for (int x = 0; x < w; x++) {
				int gx = x / cell;
				float fx = (float) (x % cell) / cell;
				fx = fx * fx * (3 - 2 * fx);
				float a = lattice[gy * gw + gx], b = lattice[gy * gw + gx + 1];
				float c = lattice[(gy + 1) * gw + gx], d = lattice[(gy + 1) * gw + gx + 1];
				out[y * w + x] = (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
			}
		}
		return out;
	}

	private static BufferedImage toImage(float[] lum, int w, int h) {
		var img = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				int c = (int) Math.round(255 * clamp01(lum[y * w + x]));
				img.setRGB(x, y, (c << 16) | (c << 8) | c);
			}
		}
		return img;
	}

	/**
	 * Plating tile: one tile is {@link #PLATE_U} x {@link #PLATE_V} metres. Horizontal strake seams
	 * every 2 m, staggered vertical butt welds, slow mottling, rust/grime streaks running down from
	 * the top edge and a scum band at the waterline (the image's row 0 is the top of the tile: jME
	 * flips textures on load).
	 */
	static BufferedImage hullTexture() {
		int w = 1024, h = 512;
		float[] lum = new float[w * h];
		float[] mottle = valueNoise(w, h, 64, 1L), fine = valueNoise(w, h, 6, 2L);
		for (int i = 0; i < lum.length; i++)
			lum[i] = 0.90f + 0.08f * (mottle[i] - 0.5f) + 0.03f * (fine[i] - 0.5f);
		double pxPerM = h / PLATE_V;
		int strake = (int) Math.round(2.0 * pxPerM), butt = (int) Math.round(9.0 * (w / PLATE_U));
		for (int s = 0; s * strake < h; s++) {
			int y = s * strake;
			for (int x = 0; x < w; x++) {
				lum[y * w + x] *= 0.80f;
				if (y + 1 < h)
					lum[(y + 1) * w + x] *= 0.90f;
				if (y + 2 < h)
					lum[(y + 2) * w + x] = Math.min(1f, lum[(y + 2) * w + x] * 1.05f);
			}
			int offset = (s % 2) * butt / 2 + (s % 3) * 37;
			for (int x = offset; x < w; x += butt) {
				for (int yy = y; yy < Math.min(h, y + strake); yy++) {
					lum[yy * w + x % w] *= 0.82f;
					lum[yy * w + (x + 1) % w] *= 0.93f;
				}
			}
		}
		Random r = new Random(3L);
		for (int i = 0; i < 70; i++) { // streaks from the top (deck edge, scuppers, anchor pocket)
			int x = r.nextInt(w), width = 2 + r.nextInt(5), len = 40 + r.nextInt(220);
			float dark = 0.45f + 0.35f * r.nextFloat();
			for (int y = 0; y < len && y < h; y++) {
				float fade = 1f - (float) y / len;
				for (int dx = 0; dx < width; dx++) {
					float edge = 1f - Math.abs((dx + 0.5f) / width - 0.5f) * 1.6f;
					int idx = y * w + (x + dx) % w;
					lum[idx] *= 1f - (1f - dark) * fade * Math.max(0f, edge);
				}
			}
		}
		int wl = (int) Math.round(h - T * pxPerM); // waterline row
		for (int y = wl - 8; y < wl + 4; y++) {
			float f = 1f - Math.abs(y - (wl - 2)) / 6f;
			if (y < 0 || y >= h || f <= 0)
				continue;
			for (int x = 0; x < w; x++)
				lum[y * w + x] *= 1f - 0.18f * f * (0.6f + 0.4f * fine[y * w + x]);
		}
		return toImage(lum, w, h);
	}

	/**
	 * Container atlas: u in [0, 0.75) is a corrugated side (40 ft long), u in [0.75, 1] a door end
	 * with lock bars. Top and bottom rails and corner posts are darker so each box reads as a
	 * framed unit.
	 */
	static BufferedImage containerTexture() {
		int w = 1024, h = 256;
		int sideW = (int) (w * CONTAINER_SIDE_U);
		float[] lum = new float[w * h];
		float[] dirt = valueNoise(w, h, 40, 5L), fine = valueNoise(w, h, 5, 6L);
		int period = 18;
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				boolean door = x >= sideW;
				int lx = door ? x - sideW : x;
				float phase = (float) (lx % period) / period;
				float ridge = 1f - Math.abs(phase * 2f - 1f); // triangle wave: one lit and one shaded flank per fold
				float shade = 0.78f + 0.22f * (phase < 0.5f ? 0.55f + 0.45f * ridge : 0.85f + 0.15f * ridge);
				float v = shade * (0.93f + 0.07f * dirt[y * w + x]) + 0.02f * (fine[y * w + x] - 0.5f);
				boolean frame = y < 7 || y >= h - 7 || lx < 8 || (door ? lx >= w - sideW - 8 : lx >= sideW - 8);
				if (frame)
					v = 0.52f + 0.05f * fine[y * w + x];
				if (door) {
					int dw = w - sideW;
					int centre = dw / 2;
					if (Math.abs(lx - centre) <= 1)
						v = 0.35f;
					for (int bar : new int[] {dw * 15 / 100, dw * 31 / 100, dw * 69 / 100, dw * 85 / 100}) {
						if (Math.abs(lx - bar) <= 2 && y > 10 && y < h - 10)
							v = 0.42f + 0.06f * fine[y * w + x];
						if (Math.abs(lx - bar) <= 4 && (Math.abs(y - h / 3) < 4 || Math.abs(y - 2 * h / 3) < 4))
							v = 0.30f; // handles
					}
					for (int hinge : new int[] {12, 30, dw - 12, dw - 30})
						if (Math.abs(lx - hinge) <= 2 && (y % 40) < 8 && y > 10 && y < h - 10)
							v = 0.40f;
				}
				lum[y * w + x] = v;
			}
		}
		return toImage(lum, w, h);
	}

	// ── Output ───────────────────────────────────────────────────────────────

	private int write(Path objPath, Path mtlPath) throws IOException {
		try (PrintWriter m = new PrintWriter(Files.newBufferedWriter(mtlPath, StandardCharsets.US_ASCII))) {
			m.print("# Surface ship (feeder container ship) materials. Generated by ShipModelGenerator.\n#\n");
			for (var e : MATERIALS.entrySet()) {
				String name = e.getKey();
				Mat c = e.getValue();
				// Ka feeds jME's ambient term; zero would leave the shadow side of the hull pitch black.
				m.printf(Locale.ROOT, "newmtl %s\nKa  %.3f %.3f %.3f\nKd  %.3f %.3f %.3f\nKs  %.2f %.2f %.2f\n", name,
						c.r(), c.g(), c.b(), c.r(), c.g(), c.b(), c.ks(), c.ks(), c.ks());
				m.printf(Locale.ROOT, "d  1.0\nNs  %.1f\nillum 2\n", c.ns());
				if (c.map() != null)
					m.print("map_Kd " + c.map() + "\n");
				m.print("#\n");
			}
			m.print("# EOF\n");
		}

		// Smooth groups: per-vertex normals accumulated across all smooth groups
		Map<Integer, double[]> accumulated = new LinkedHashMap<>();
		for (Group g : groups) {
			if (!g.smooth)
				continue;
			for (int[] t : g.tris) {
				double[] p0 = verts.get(t[0] - 1), p1 = verts.get(t[1] - 1), p2 = verts.get(t[2] - 1);
				double[] n = cross(sub(p1, p0), sub(p2, p0));
				for (int i : t) {
					double[] acc = accumulated.computeIfAbsent(i, k -> new double[3]);
					acc[0] += n[0];
					acc[1] += n[1];
					acc[2] += n[2];
				}
			}
		}
		List<double[]> normals = new ArrayList<>();
		Map<Integer, Integer> normalOfVertex = new HashMap<>();
		for (var e : accumulated.entrySet()) {
			normals.add(normalize(e.getValue()));
			normalOfVertex.put(e.getKey(), normals.size());
		}
		// Flat groups: one normal per face direction
		Map<String, Integer> flatNormals = new HashMap<>();
		StringBuilder faces = new StringBuilder();
		int triangleCount = 0;
		for (Group g : groups) {
			if (g.tris.isEmpty())
				continue;
			faces.append("g ").append(g.name).append('\n').append("usemtl ").append(g.material).append('\n');
			for (int[] t : g.tris) {
				triangleCount++;
				if (g.smooth) {
					faces.append(String.format(Locale.ROOT, "f %d/%d/%d %d/%d/%d %d/%d/%d\n", t[0], t[0],
							normalOfVertex.get(t[0]), t[1], t[1], normalOfVertex.get(t[1]), t[2], t[2],
							normalOfVertex.get(t[2])));
				} else {
					double[] p0 = verts.get(t[0] - 1), p1 = verts.get(t[1] - 1), p2 = verts.get(t[2] - 1);
					double[] n = normalize(cross(sub(p1, p0), sub(p2, p0)));
					String key = String.format(Locale.ROOT, "%.3f,%.3f,%.3f", n[0], n[1], n[2]);
					Integer ni = flatNormals.get(key);
					if (ni == null) {
						normals.add(n);
						ni = normals.size();
						flatNormals.put(key, ni);
					}
					faces.append(String.format(Locale.ROOT, "f %d/%d/%d %d/%d/%d %d/%d/%d\n", t[0], t[0], ni, t[1],
							t[1], ni, t[2], t[2], ni));
				}
			}
		}

		try (PrintWriter o = new PrintWriter(Files.newBufferedWriter(objPath, StandardCharsets.US_ASCII))) {
			o.print("# Surface ship: 150 m feeder container ship for the surface-ship drone\n");
			o.print("# Generated by se.hirt.searobots.viewer.tools.ShipModelGenerator. Units: metres."
					+ " X=starboard, Y=aft (bow at -Y), Z=up, waterline Z=0.\n");
			o.printf(Locale.ROOT, "# %d vertices, %d normals, %d triangles\n", verts.size(), normals.size(),
					triangleCount);
			o.print("mtllib " + mtlPath.getFileName() + "\n#\n");
			for (double[] p : verts)
				o.printf(Locale.ROOT, "v %.4f %.4f %.4f\n", p[0], p[1], p[2]);
			for (double[] uv : uvs)
				o.printf(Locale.ROOT, "vt %.4f %.4f\n", uv[0], uv[1]);
			for (double[] n : normals)
				o.printf(Locale.ROOT, "vn %.4f %.4f %.4f\n", n[0], n[1], n[2]);
			o.print(faces);
			o.print("# EOF\n");
		}
		return triangleCount;
	}
}
