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

/**
 * Generates {@code models/surface-ship.obj} and {@code surface-ship.mtl}: the 150 m x 30 m
 * container ship used for the surface-ship drone. Pure Java, no dependencies.
 * <p>
 * The OBJ follows the same convention as {@code submarine-hybrid.obj} so the viewer treats both
 * templates identically: X is starboard, Y is fore-aft with the <b>bow at -Y</b> (the submarine's
 * propeller is at +Y), Z is up, units are metres and the waterline is at Z = 0. The viewer rotates
 * the template -90 degrees about X so the bow faces +Z in jME; no scaling is needed.
 * <p>
 * Faces are emitted with outward winding and explicit normals. jME's OBJ loader merges vertices
 * with equal position unless their normals differ, and the viewer only generates normals for meshes
 * that lack them, so the hull gets smooth per-vertex normals shared across its groups (no seam at
 * the waterline paint line) and everything box-like gets one flat normal per face for crisp edges.
 * The propeller is its own group named {@code Propeller} so the viewer can spin it.
 * <p>
 * Usage: {@code ShipModelGenerator <out-dir>}, for example
 * {@code java -cp target/classes se.hirt.searobots.viewer.tools.ShipModelGenerator src/main/resources/models}.
 * Use {@link ModelRenderCheck} to look at the result under jME lighting.
 */
public final class ShipModelGenerator {

	// Length, beam, draft, deck height above the waterline (metres)
	private static final double L = 150.0, B = 30.0, T = 7.5, D = 9.5;

	private static final Map<String, double[]> MATERIALS = new LinkedHashMap<>();

	static {
		MATERIALS.put("hull_dark", new double[] {0.10, 0.13, 0.22});
		MATERIALS.put("hull_red", new double[] {0.55, 0.14, 0.10});
		MATERIALS.put("deck", new double[] {0.30, 0.34, 0.30});
		MATERIALS.put("hatch", new double[] {0.36, 0.38, 0.36});
		MATERIALS.put("superstructure", new double[] {0.88, 0.88, 0.86});
		MATERIALS.put("glass", new double[] {0.05, 0.08, 0.12});
		MATERIALS.put("funnel", new double[] {0.72, 0.14, 0.12});
		MATERIALS.put("metal_dark", new double[] {0.15, 0.15, 0.15});
		MATERIALS.put("Metal_Chrome", new double[] {0.55, 0.52, 0.42});
		MATERIALS.put("lifeboat", new double[] {0.90, 0.45, 0.08});
		MATERIALS.put("container_red", new double[] {0.62, 0.16, 0.12});
		MATERIALS.put("container_blue", new double[] {0.12, 0.25, 0.55});
		MATERIALS.put("container_green", new double[] {0.12, 0.42, 0.25});
		MATERIALS.put("container_grey", new double[] {0.55, 0.56, 0.55});
		MATERIALS.put("container_rust", new double[] {0.55, 0.32, 0.14});
	}

	private final List<double[]> verts = new ArrayList<>();
	private final List<Group> groups = new ArrayList<>();

	public static void main(String[] args) throws IOException {
		if (args.length < 1) {
			System.err.println("Usage: ShipModelGenerator <out-dir>");
			System.exit(2);
		}
		Path out = Path.of(args[0]);
		Files.createDirectories(out);
		var gen = new ShipModelGenerator();
		gen.buildHull();
		gen.buildSuperstructure();
		gen.buildCargo();
		gen.buildSternGear();
		int tris = gen.write(out.resolve("surface-ship.obj"), out.resolve("surface-ship.mtl"));
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

	private int v(double x, double y, double z) {
		verts.add(new double[] {x, y, z});
		return verts.size();
	}

	private static double[] sub(double[] a, double[] b) {
		return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
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

	/** Axis-aligned box with unshared vertices so its edges stay crisp. */
	private void box(Group g, double x0, double x1, double y0, double y1, double z0, double z1) {
		double[] c = {(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2};
		face(g, c, x0, y0, z0, x1, y0, z0, x1, y1, z0, x0, y1, z0); // bottom
		face(g, c, x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1); // top
		face(g, c, x0, y0, z0, x1, y0, z0, x1, y0, z1, x0, y0, z1); // fore
		face(g, c, x0, y1, z0, x1, y1, z0, x1, y1, z1, x0, y1, z1); // aft
		face(g, c, x0, y0, z0, x0, y1, z0, x0, y1, z1, x0, y0, z1); // port
		face(g, c, x1, y0, z0, x1, y1, z0, x1, y1, z1, x1, y0, z1); // starboard
	}

	private void face(Group g, double[] inside, double ... p) {
		g.quad(v(p[0], p[1], p[2]), v(p[3], p[4], p[5]), v(p[6], p[7], p[8]), v(p[9], p[10], p[11]), inside);
	}

	/** Vertical elliptical prism (funnel). */
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

	/** Fore-aft cylinder (shaft, propeller hub). */
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

	/**
	 * Half-breadth at deck level: fine entry (about 25 degrees half-angle), parallel midbody,
	 * tapered run.
	 */
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
		if (t < 0.15)
			return -T * Math.pow(t / 0.15, 0.6); // forefoot rises to the waterline at the stem
		if (t <= 0.70)
			return -T;
		return -T + (T - 1.0) * Math.pow((t - 0.70) / 0.30, 1.2); // run rises to the transom (-1 m)
	}

	private static double deckZ(double t) {
		double bowSheer = clamp01((0.30 - t) / 0.30), sternSheer = clamp01((t - 0.80) / 0.20);
		return D + 2.0 * bowSheer * bowSheer + 0.8 * sternSheer * sternSheer;
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

	/**
	 * One hull section as a ring of vertex indices: port deck edge down to the keel, then up the
	 * starboard side. Level 6 is exactly the waterline so the paint can change there. Centreline
	 * points (keel, and the whole stem where the half-breadth is zero) are shared between the two
	 * sides so the smooth normal there points forward instead of leaving a mirrored crease.
	 */
	private int[] stationRing(double t) {
		double zk = keelZ(t), zd = deckZ(t);
		double vWl = zd > zk ? (0 - zk) / (zd - zk) : 0.0;
		double[] levels = new double[11];
		for (int i = 0; i <= 6; i++)
			levels[i] = vWl * i / 6;
		for (int i = 1; i <= 4; i++)
			levels[6 + i] = vWl + (1 - vWl) * i / 4;
		double k = keelFlat(t), hbDeck = halfBreadthAtDeck(t), fl = flare(t);
		double hbWl = hbDeck / (1 + fl);
		double yBase = -L / 2 + L * t;

		double[][] pts = new double[levels.length][];
		for (int i = 0; i < levels.length; i++) {
			double vv = levels[i];
			double z = zk + vv * (zd - zk);
			double x;
			if (vWl > 1e-6 && vv <= vWl + 1e-9) {
				double u = clamp01(vv / vWl);
				double frac = k + (1 - k) * Math.pow(1 - Math.pow(1 - u, 2.2), 1 / 2.2);
				x = hbWl * frac;
			} else {
				double w = vWl < 1 ? (vv - vWl) / (1 - vWl) : 1;
				x = hbWl * (1 + fl * w);
			}
			double y = yBase + rake(t) * (1 - vv);
			pts[i] = new double[] {x, y, z};
		}

		Map<String, Integer> shared = new HashMap<>();
		int[] ring = new int[2 * pts.length - 1];
		int idx = 0;
		for (int i = pts.length - 1; i >= 1; i--)
			ring[idx++] = ringVertex(shared, -pts[i][0], pts[i][1], pts[i][2]);
		ring[idx++] = ringVertex(shared, 0.0, pts[0][1], pts[0][2]);
		for (int i = 1; i < pts.length; i++)
			ring[idx++] = ringVertex(shared, pts[i][0], pts[i][1], pts[i][2]);
		return ring;
	}

	private int ringVertex(Map<String, Integer> shared, double x, double y, double z) {
		if (Math.abs(x) < 1e-6) {
			String key = String.format(Locale.ROOT, "%.4f,%.4f", y, z);
			return shared.computeIfAbsent(key, kk -> v(0.0, y, z));
		}
		return v(x, y, z);
	}

	private static double[] sectionCentre(double t) {
		return new double[] {0.0, -L / 2 + L * t, (keelZ(t) + deckZ(t)) / 2};
	}

	private void buildHull() {
		Group hullRed = new Group("HullBelow", "hull_red", true);
		Group hullDark = new Group("HullAbove", "hull_dark", true);
		Group deck = new Group("Deck", "deck", false);
		// Cosine-spaced stations: dense at the bow and stern where the curvature
		// is high, so the smooth shading interpolates cleanly across the entry.
		int stations = 80;
		int[][] rings = new int[stations + 1][];
		double[][] centres = new double[stations + 1][];
		for (int i = 0; i <= stations; i++) {
			double t = 0.5 * (1 - Math.cos(Math.PI * i / stations));
			rings[i] = stationRing(t);
			centres[i] = sectionCentre(t);
		}
		int m = rings[0].length;
		for (int i = 0; i < stations; i++) {
			int[] r0 = rings[i], r1 = rings[i + 1];
			double[] inside = {0.0, (centres[i][1] + centres[i + 1][1]) / 2, (centres[i][2] + centres[i + 1][2]) / 2};
			for (int j = 0; j < m - 1; j++) {
				double zc = (verts.get(r0[j] - 1)[2] + verts.get(r0[j + 1] - 1)[2] + verts.get(r1[j] - 1)[2]
						+ verts.get(r1[j + 1] - 1)[2]) / 4;
				Group g = zc < -1e-6 ? hullRed : hullDark;
				g.quad(r0[j], r0[j + 1], r1[j + 1], r1[j], inside);
			}
			// Deck strip between the two deck edges
			deck.quad(r0[0], r0[m - 1], r1[m - 1], r1[0], new double[] {0.0, inside[1], inside[2] - 5});
		}
		// Transom: fan from its centre
		int[] last = rings[stations];
		double cx = 0, cy = 0, cz = 0;
		for (int i : last) {
			cx += verts.get(i - 1)[0];
			cy += verts.get(i - 1)[1];
			cz += verts.get(i - 1)[2];
		}
		cx /= last.length;
		cy /= last.length;
		cz /= last.length;
		int centre = v(cx, cy, cz);
		double[] inside = {0.0, cy - 5.0, cz};
		for (int j = 0; j < last.length - 1; j++) {
			double zc = (verts.get(last[j] - 1)[2] + verts.get(last[j + 1] - 1)[2]) / 2;
			(zc < 0 ? hullRed : hullDark).tri(centre, last[j], last[j + 1], inside);
		}
		hullDark.tri(centre, last[last.length - 1], last[0], inside); // close across the deck edge
	}

	// ── Superstructure, cargo, stern gear ────────────────────────────────────

	private void buildSuperstructure() {
		Group white = new Group("Superstructure", "superstructure", false);
		Group glass = new Group("Windows", "glass", false);
		// Accommodation block aft, three tiers
		box(white, -10.0, 10.0, 42.0, 62.0, 9.0, 15.5);
		box(white, -9.0, 9.0, 43.0, 61.0, 15.5, 20.5);
		box(white, -8.0, 8.0, 44.0, 54.0, 20.5, 24.0);
		// Bridge wings out to the full beam
		box(white, 8.0, 15.0, 44.0, 48.0, 20.5, 23.5);
		box(white, -15.0, -8.0, 44.0, 48.0, 20.5, 23.5);
		// Window bands (thin dark slabs proud of the faces)
		box(glass, -7.6, 7.6, 43.85, 44.05, 21.8, 23.2); // bridge front
		box(glass, -8.6, 8.6, 42.85, 43.05, 17.0, 18.6); // tier 2 front
		box(glass, 8.0, 14.8, 43.85, 44.05, 21.6, 22.8); // wing fronts
		box(glass, -14.8, -8.0, 43.85, 44.05, 21.6, 22.8);
		box(glass, -9.6, 9.6, 41.85, 42.05, 11.0, 12.6); // tier 1 front
		for (double y0 : new double[] {46.0, 52.0, 58.0}) { // side windows tier 1
			box(glass, 9.95, 10.15, y0, y0 + 4.0, 11.0, 12.4);
			box(glass, -10.15, -9.95, y0, y0 + 4.0, 11.0, 12.4);
		}
		// Funnel on the tier-2 roof, with a dark cap
		prismZ(new Group("Funnel", "funnel", false), 0.0, 58.0, 2.2, 3.0, 20.5, 27.5, 14);
		prismZ(new Group("FunnelCap", "metal_dark", false), 0.0, 58.0, 2.3, 3.1, 27.5, 29.0, 14);
		// Main mast on the bridge roof, with a radar bar and a yard
		Group mast = new Group("Mast", "metal_dark", false);
		box(mast, -0.4, 0.4, 46.0, 46.8, 24.0, 35.0);
		box(mast, -3.0, 3.0, 46.2, 46.6, 30.0, 30.4);
		box(mast, -2.6, 2.6, 45.6, 47.2, 34.4, 35.0);
		// Lifeboats on either side of the accommodation
		Group boats = new Group("Lifeboats", "lifeboat", false);
		box(boats, 10.2, 11.6, 49.5, 56.0, 16.2, 18.0);
		box(boats, -11.6, -10.2, 49.5, 56.0, 16.2, 18.0);
		// Forecastle: windlass house and foremast
		box(white, -3.0, 3.0, -66.0, -60.5, 9.0, 12.9);
		box(mast, -0.35, 0.35, -58.0, -57.3, 9.0, 22.0);
		box(mast, -2.5, 2.5, -57.85, -57.45, 19.0, 19.4);
	}

	private void buildCargo() {
		Group coaming = new Group("HatchCoamings", "hatch", false);
		String[] colours = {"container_red", "container_blue", "container_green", "container_grey", "container_rust",
				"container_blue", "container_red"};
		Map<String, Group> containerGroups = new LinkedHashMap<>();
		for (String c : new String[] {"container_blue", "container_green", "container_grey", "container_red",
				"container_rust"}) {
			containerGroups.put(c, new Group("Containers_" + c, c, false));
		}
		double hatchLength = 17.0, gap = 2.0, rowWidth = 21.0 / 4;
		// Forward end sits where the deck is ~9.3 m wide each side; the block ends 1 m short of the house.
		double y = -52.0;
		int colourIndex = 0;
		for (int h = 0; h < 5; h++) {
			double y0 = y, y1 = y + hatchLength;
			// The forward hatch sits in the bow taper, so it carries three rows on a narrower coaming.
			int rows = h == 0 ? 3 : 4;
			double halfWidth = h == 0 ? 8.5 : 11.0;
			box(coaming, -halfWidth, halfWidth, y0, y1, 9.0, 11.5);
			int tiers = (h == 1 || h == 3) ? 2 : (h == 2 ? 3 : 1);
			for (int tier = 0; tier < tiers; tier++) {
				double z0 = 11.5 + tier * 2.6;
				for (int r = 0; r < rows; r++) {
					double x0 = -rows * rowWidth / 2 + r * rowWidth + 0.15;
					double x1 = x0 + rowWidth - 0.3;
					Group g = containerGroups.get(colours[colourIndex++ % colours.length]);
					box(g, x0, x1, y0 + 0.6, y1 - 0.6, z0, z0 + 2.6);
				}
			}
			y = y1 + gap;
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

	// ── Output ───────────────────────────────────────────────────────────────

	private int write(Path objPath, Path mtlPath) throws IOException {
		try (PrintWriter m = new PrintWriter(Files.newBufferedWriter(mtlPath, StandardCharsets.US_ASCII))) {
			m.print("# Surface ship (container ship) materials\n#\n");
			for (var e : MATERIALS.entrySet()) {
				String name = e.getKey();
				double[] c = e.getValue();
				double specular = name.equals("Metal_Chrome") ? 0.9 : 0.25;
				double shininess = name.equals("Metal_Chrome") ? 40.0 : (name.equals("glass") ? 60.0 : 12.0);
				// Ka feeds jME's ambient term; zero would leave the shadow side of the hull pitch black.
				m.printf(Locale.ROOT, "newmtl %s\nKa  %.3f %.3f %.3f\nKd  %.3f %.3f %.3f\nKs  %.2f %.2f %.2f\n", name,
						c[0], c[1], c[2], c[0], c[1], c[2], specular, specular, specular);
				m.printf(Locale.ROOT, "d  1.0\nNs  %.1f\nillum 2\n#\n", shininess);
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
					faces.append(String.format(Locale.ROOT, "f %d//%d %d//%d %d//%d\n", t[0], normalOfVertex.get(t[0]),
							t[1], normalOfVertex.get(t[1]), t[2], normalOfVertex.get(t[2])));
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
					faces.append(String.format(Locale.ROOT, "f %d//%d %d//%d %d//%d\n", t[0], ni, t[1], ni, t[2], ni));
				}
			}
		}

		try (PrintWriter o = new PrintWriter(Files.newBufferedWriter(objPath, StandardCharsets.US_ASCII))) {
			o.print("# Surface ship: 150 m container ship for the surface-ship drone\n");
			o.print("# Generated by se.hirt.searobots.viewer.tools.ShipModelGenerator. Units: metres."
					+ " X=starboard, Y=aft (bow at -Y), Z=up, waterline Z=0.\n");
			o.printf(Locale.ROOT, "# %d vertices, %d normals, %d triangles\n", verts.size(), normals.size(),
					triangleCount);
			o.print("mtllib " + mtlPath.getFileName() + "\n#\n");
			for (double[] p : verts)
				o.printf(Locale.ROOT, "v %.4f %.4f %.4f\n", p[0], p[1], p[2]);
			for (double[] n : normals)
				o.printf(Locale.ROOT, "vn %.4f %.4f %.4f\n", n[0], n[1], n[2]);
			o.print(faces);
			o.print("# EOF\n");
		}
		return triangleCount;
	}
}
