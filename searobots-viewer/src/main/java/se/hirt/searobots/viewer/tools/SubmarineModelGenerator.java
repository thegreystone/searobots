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
 * Generates parts of {@code models/submarine-hybrid.obj}. For now it retrofits a pump-jet propulsor
 * onto the existing hull: it reads the OBJ, replaces the {@code Propeller} and
 * {@code PropellerMount} groups with generated geometry, tucks the tail spike that used to stick
 * out behind the old propeller inside the new hub, closes a sliver hole along the top of the hull's
 * tail cone, and recomputes normals for the groups it touched. Running it again on its own output
 * gives the same result. The plan is to grow this into a generator for the whole submarine.
 * <p>
 * Model conventions as in the viewer: X across, Y fore-aft with the stern at +Y, Z up, metres. The
 * propeller shaft runs along Y through x = 0, z = {@link #AXIS_Z}; the viewer spins the
 * {@code Propeller} group about that axis (SUB_PROP_LOCAL in SubmarineScene3D), so the rotor and
 * hub must be symmetric about it.
 * <ul>
 * <li>{@code Propeller} (rotating): seven skewed, raked blades with rounded tips on a slim hub that
 * tapers to a point just behind the duct exit.</li>
 * <li>{@code PropellerMount} (fixed): a duct with an aerofoil section that narrows towards the
 * exit, and five stator vanes ahead of the rotor that hold it to the hull. The vanes avoid the
 * rudder planes at the top and bottom, and the duct stays clear of the crescent rudders.</li>
 * </ul>
 * Normals are area weighted and split at {@link #CREASE_DEG}, so curved surfaces are smooth and
 * real edges stay crisp.
 * <p>
 * Usage: {@code SubmarineModelGenerator <in.obj> <out.obj>}, for example
 * {@code java -cp target/classes se.hirt.searobots.viewer.tools.SubmarineModelGenerator src/main/resources/models/submarine-hybrid.obj src/main/resources/models/submarine-hybrid.obj}.
 */
public final class SubmarineModelGenerator {

	private static final double AXIS_Z = 0.11;
	private static final double CREASE_DEG = 50;
	// Tail: hull vertices aft of TAIL_START are pulled forward so the tip ends at TAIL_END, inside the hub cap
	private static final double TAIL_START = 38.0, TAIL_END = 38.35;
	// Duct: aerofoil chord along Y, mean radius narrowing towards the exit (a mild nozzle)
	private static final double DUCT_Y0 = 36.7, DUCT_Y1 = 38.5, DUCT_R_LE = 2.08, DUCT_R_TE = 1.98;
	private static final double DUCT_THICKNESS = 0.133; // fraction of the chord (NACA 4-digit)
	// Stator vanes just inside the duct entry and ahead of the (rotating) hub, rotated off the rudder
	// planes (90 and 270 degrees)
	private static final int STATORS = 5;
	private static final double STATOR_Y0 = 36.74, STATOR_Y1 = 36.98, STATOR_ANGLE0 = 36;
	// Rotor
	private static final int BLADES = 7;
	private static final double ROTOR_Y = 37.55, BLADE_ROOT_R = 0.38, BLADE_TIP_R = 1.86, PITCH = 2.6;
	private static final double SKEW = 0.55, RAKE = 0.12; // tip skew (radians) and aft rake (metres)
	// Hub profile (y, radius), front to back, just covering the hull's tail cone; the tail cone
	// tapers from the last point to a tip at HUB_TIP_Y as r = R (1 - s^HUB_TAPER)
	private static final double[][] HUB = {{37.0, 0.37}, {37.12, 0.42}, {37.25, 0.46}, {37.85, 0.46}};
	private static final double HUB_TIP_Y = 39.1, HUB_TAPER = 1.8;

	private final List<double[]> verts = new ArrayList<>();
	private final List<double[]> uvs = new ArrayList<>();
	private final List<double[]> normals = new ArrayList<>();
	private final LinkedHashMap<String, Group> groups = new LinkedHashMap<>();
	private final List<String> header = new ArrayList<>();

	/** A face corner: 1-based vertex, uv and normal indices (0 = absent). */
	private record Corner(int v, int t, int n) {
	}

	private static final class Group {
		final String name;
		String material;
		final List<Corner[]> faces = new ArrayList<>();

		Group(String name) {
			this.name = name;
		}
	}

	public static void main(String[] args) throws IOException {
		if (args.length < 2) {
			System.err.println("Usage: SubmarineModelGenerator <in.obj> <out.obj>");
			System.exit(2);
		}
		var gen = new SubmarineModelGenerator();
		gen.read(Path.of(args[0]));
		gen.tuckTail();
		gen.closeTriangularHoles(gen.groups.get("Body"));
		Group rotor = gen.replace("Propeller", "Metal_Chrome");
		Group mount = gen.replace("PropellerMount", "Metal_Black_Plain");
		gen.buildDuct(mount);
		gen.buildStators(mount);
		gen.buildRotor(rotor);
		gen.buildHub(rotor);
		for (String g : List.of("Body", "Propeller", "PropellerMount"))
			gen.creaseNormals(gen.groups.get(g));
		gen.write(Path.of(args[1]));
	}

	// ── OBJ in and out ───────────────────────────────────────────────────────

	private void read(Path in) throws IOException {
		Group current = null;
		for (String line : Files.readAllLines(in)) {
			String[] t = line.trim().split("\\s+");
			switch (t[0]) {
			case "v" -> verts.add(new double[] {num(t[1]), num(t[2]), num(t[3])});
			case "vt" -> uvs.add(new double[] {num(t[1]), num(t[2])});
			case "vn" -> normals.add(new double[] {num(t[1]), num(t[2]), num(t[3])});
			case "g" -> current = groups.computeIfAbsent(t[1], Group::new);
			case "usemtl" -> current.material = t[1];
			case "f" -> {
				Corner[] f = new Corner[t.length - 1];
				for (int i = 0; i < f.length; i++) {
					String[] p = t[i + 1].split("/", -1);
					f[i] = new Corner(Integer.parseInt(p[0]),
							p.length > 1 && !p[1].isEmpty() ? Integer.parseInt(p[1]) : 0,
							p.length > 2 && !p[2].isEmpty() ? Integer.parseInt(p[2]) : 0);
				}
				current.faces.add(f);
			}
			default -> {
				if (current == null && !line.isBlank() && !line.startsWith("# EOF"))
					header.add(line);
			}
			}
		}
	}

	/**
	 * Writes all groups in their original order, keeping only the vertices, uvs and normals in use.
	 */
	private void write(Path out) throws IOException {
		int[] vMap = new int[verts.size() + 1], tMap = new int[uvs.size() + 1], nMap = new int[normals.size() + 1];
		StringBuilder v = new StringBuilder(), vt = new StringBuilder(), vn = new StringBuilder(),
				f = new StringBuilder();
		int[] counts = new int[4];
		for (Group g : groups.values()) {
			f.append("g ").append(g.name).append('\n');
			if (g.material != null)
				f.append("usemtl ").append(g.material).append('\n');
			for (Corner[] face : g.faces) {
				counts[3]++;
				f.append('f');
				for (Corner c : face) {
					if (vMap[c.v] == 0) {
						double[] p = verts.get(c.v - 1);
						v.append(String.format(Locale.ROOT, "v %.6f %.6f %.6f\n", p[0], p[1], p[2]));
						vMap[c.v] = ++counts[0];
					}
					f.append(' ').append(vMap[c.v]);
					if (c.t == 0 && c.n == 0)
						continue;
					f.append('/');
					if (c.t > 0) {
						if (tMap[c.t] == 0) {
							double[] p = uvs.get(c.t - 1);
							vt.append(String.format(Locale.ROOT, "vt %.6f %.6f\n", p[0], p[1]));
							tMap[c.t] = ++counts[1];
						}
						f.append(tMap[c.t]);
					}
					if (c.n > 0) {
						if (nMap[c.n] == 0) {
							double[] p = normals.get(c.n - 1);
							vn.append(String.format(Locale.ROOT, "vn %.6f %.6f %.6f\n", p[0], p[1], p[2]));
							nMap[c.n] = ++counts[2];
						}
						f.append('/').append(nMap[c.n]);
					}
				}
				f.append('\n');
			}
		}
		StringBuilder sb = new StringBuilder();
		header.forEach(h -> sb.append(h).append('\n'));
		sb.append(v).append(vt).append(vn).append(f).append("# EOF\n");
		Files.writeString(out, sb, StandardCharsets.US_ASCII);
		System.out.printf(Locale.ROOT, "vertices=%d uvs=%d normals=%d faces=%d -> %s%n", counts[0], counts[1],
				counts[2], counts[3], out);
	}

	/** Empties (or creates) a group in place so it keeps its position in the file. */
	private Group replace(String name, String material) {
		Group g = groups.computeIfAbsent(name, Group::new);
		g.faces.clear();
		g.material = material;
		return g;
	}

	// ── Hull tail ────────────────────────────────────────────────────────────

	/**
	 * Pulls the hull vertices aft of {@link #TAIL_START} forward so the tail ends inside the hub
	 * cap.
	 */
	private void tuckTail() {
		double maxY = TAIL_START;
		List<Integer> tail = new ArrayList<>();
		for (Corner[] face : groups.get("Body").faces)
			for (Corner c : face) {
				double y = verts.get(c.v - 1)[1];
				if (y > TAIL_START && !tail.contains(c.v)) {
					tail.add(c.v);
					maxY = Math.max(maxY, y);
				}
			}
		if (maxY <= TAIL_END)
			return;
		double k = (TAIL_END - TAIL_START) / (maxY - TAIL_START);
		for (int i : tail) {
			double[] p = verts.get(i - 1);
			p[1] = TAIL_START + (p[1] - TAIL_START) * k;
		}
		System.out.printf(Locale.ROOT, "tail: %d vertices pulled from y <= %.2f to y <= %.2f%n", tail.size(), maxY,
				TAIL_END);
	}

	/**
	 * Closes three-edge holes in a group: the original hull has a long sliver open along the top of
	 * the tail cone. Each hole gets one triangle, wound against its border so it faces the same way
	 * as its neighbours, reusing texture coordinates its vertices already have.
	 */
	private void closeTriangularHoles(Group g) {
		Map<Long, Boolean> directed = new HashMap<>();
		Map<Integer, Integer> uvOf = new HashMap<>();
		for (Corner[] f : g.faces)
			for (int k = 0; k < f.length; k++) {
				directed.put(edgeKey(f[k].v, f[(k + 1) % f.length].v), true);
				if (f[k].t > 0)
					uvOf.putIfAbsent(f[k].v, f[k].t);
			}
		Map<Integer, List<Integer>> border = new HashMap<>(); // a -> b for edges a->b without a b->a twin
		for (long key : directed.keySet()) {
			int a = (int) (key >> 32), b = (int) key;
			if (!directed.containsKey(edgeKey(b, a)))
				border.computeIfAbsent(a, k -> new ArrayList<>()).add(b);
		}
		int closed = 0;
		for (var e : new ArrayList<>(border.entrySet()))
			for (int b : e.getValue())
				for (int c : border.getOrDefault(b, List.of()))
					if (border.getOrDefault(c, List.of()).contains(e.getKey()) && e.getKey() < b && e.getKey() < c) {
						int a = e.getKey();
						g.faces.add(new Corner[] {new Corner(a, uvOf.getOrDefault(a, 0), 0),
								new Corner(c, uvOf.getOrDefault(c, 0), 0), new Corner(b, uvOf.getOrDefault(b, 0), 0)});
						closed++;
					}
		if (closed > 0)
			System.out.printf(Locale.ROOT, "%s: closed %d triangular hole(s)%n", g.name, closed);
	}

	private static long edgeKey(int a, int b) {
		return ((long) a << 32) | (b & 0xffffffffL);
	}

	// ── Propulsor ────────────────────────────────────────────────────────────

	/**
	 * Point at radius {@code r}, angle {@code a} (0 = starboard, 90 degrees = up) and station
	 * {@code y}.
	 */
	private static double[] onAxis(double r, double a, double y) {
		return new double[] {r * Math.cos(a), y, AXIS_Z + r * Math.sin(a)};
	}

	/** NACA 4-digit half thickness at chord fraction {@code s}, closed trailing edge. */
	private static double nacaHalf(double s, double thickness) {
		return 5 * thickness
				* (0.2969 * Math.sqrt(s) - 0.1260 * s - 0.3516 * s * s + 0.2843 * s * s * s - 0.1036 * s * s * s * s);
	}

	private static double ductMeanR(double s) {
		return DUCT_R_LE + (DUCT_R_TE - DUCT_R_LE) * s;
	}

	/** Inner duct radius at station {@code y}. */
	private static double ductInnerR(double y) {
		double s = Math.max(0, Math.min(1, (y - DUCT_Y0) / (DUCT_Y1 - DUCT_Y0)));
		return ductMeanR(s) - nacaHalf(s, DUCT_THICKNESS) * (DUCT_Y1 - DUCT_Y0);
	}

	/**
	 * The duct: an aerofoil section (outer surface LE to TE, inner surface back) revolved about the
	 * shaft.
	 */
	private void buildDuct(Group g) {
		int chord = 12, around = 48;
		double c = DUCT_Y1 - DUCT_Y0;
		List<double[]> profile = new ArrayList<>(); // {y, r}
		for (int j = 0; j <= chord; j++) {
			double s = 0.5 * (1 - Math.cos(Math.PI * j / chord));
			profile.add(new double[] {DUCT_Y0 + s * c, ductMeanR(s) + nacaHalf(s, DUCT_THICKNESS) * c});
		}
		for (int j = chord - 1; j >= 1; j--) {
			double s = 0.5 * (1 - Math.cos(Math.PI * j / chord));
			profile.add(new double[] {DUCT_Y0 + s * c, ductMeanR(s) - nacaHalf(s, DUCT_THICKNESS) * c});
		}
		// The section is convex, so one interior point (on the mean line at its thickest) orients every face
		double[] core = {DUCT_Y0 + 0.3 * c, ductMeanR(0.3)};
		int m = profile.size();
		int[][] ring = new int[around][m];
		for (int i = 0; i < around; i++) {
			double a = 2 * Math.PI * i / around;
			for (int j = 0; j < m; j++)
				ring[i][j] = vertex(onAxis(profile.get(j)[1], a, profile.get(j)[0]));
		}
		for (int i = 0; i < around; i++) {
			int i1 = (i + 1) % around;
			double a = 2 * Math.PI * (i + 0.5) / around;
			double[] inside = onAxis(core[1], a, core[0]);
			for (int j = 0; j < m; j++) {
				int j1 = (j + 1) % m;
				quad(g, ring[i][j], ring[i1][j], ring[i1][j1], ring[i][j1], inside);
			}
		}
	}

	/**
	 * Stator vanes from inside the hull to inside the duct wall, turned slightly against the rotor
	 * swirl.
	 */
	private void buildStators(Group g) {
		double half = 0.035, twist = Math.toRadians(8);
		for (int k = 0; k < STATORS; k++) {
			double a = Math.toRadians(STATOR_ANGLE0) + 2 * Math.PI * k / STATORS;
			// Root buried in the hull, tip buried in the duct wall
			double r0 = 0.25, r1 = Math.max(ductInnerR(STATOR_Y0), ductInnerR(STATOR_Y1)) + 0.03;
			double[][] c = new double[8][];
			int idx = 0;
			for (double r : new double[] {r0, r1}) {
				for (double[] e : new double[][] {{STATOR_Y0, -1}, {STATOR_Y1, -1}, {STATOR_Y1, 1}, {STATOR_Y0, 1}}) {
					// Chord along Y, turned by the twist about the radial direction; thickness tangential
					double yc = (STATOR_Y0 + STATOR_Y1) / 2, dy = e[0] - yc;
					double tang = dy * Math.sin(twist) + e[1] * half;
					c[idx++] = onAxis(r, a + tang / r, yc + dy * Math.cos(twist));
				}
			}
			hexa(g, c);
		}
	}

	/**
	 * Seven blades: helical sections at {@link #PITCH}, skewed back and raked aft, rounded at the
	 * tip.
	 */
	private void buildRotor(Group g) {
		int span = 14, chord = 8;
		for (int blade = 0; blade < BLADES; blade++) {
			double a0 = 2 * Math.PI * blade / BLADES;
			int[][][] side = new int[2][span + 1][chord + 1];
			double[][][] mid = new double[span + 1][chord + 1][];
			for (int i = 0; i <= span; i++) {
				double u = Math.sin(Math.PI / 2 * i / span); // rows bunch up towards the rounded tip
				double r = BLADE_ROOT_R + (BLADE_TIP_R - BLADE_ROOT_R) * u;
				double tip = u > 0.7 ? Math.sqrt(Math.max(0, 1 - Math.pow((u - 0.7) / 0.3, 2))) : 1;
				double c = (0.42 + 0.36 * Math.sin(Math.PI / 2 * Math.min(u / 0.7, 1))) * tip;
				double t = 0.05 * (1 - 0.6 * u) * Math.sqrt(tip);
				double phi = Math.atan(PITCH / (2 * Math.PI * r));
				double ac = a0 + SKEW * Math.pow(u, 1.6), yc = ROTOR_Y + RAKE * u;
				for (int j = 0; j <= chord; j++) {
					double x = c * (0.5 * (1 - Math.cos(Math.PI * j / chord)) - 0.5); // -c/2 .. c/2, dense at edges
					double h = c > 1e-9 ? t * Math.sqrt(Math.max(0, 1 - Math.pow(2 * x / c, 2))) : 0;
					mid[i][j] = onAxis(r, ac + x * Math.cos(phi) / r, yc + x * Math.sin(phi));
					for (int sgn = 0; sgn < 2; sgn++) {
						if (sgn == 1 && (j == 0 || j == chord)) {
							side[1][i][j] = side[0][i][j]; // leading and trailing edges are shared
							continue;
						}
						double hs = sgn == 0 ? h : -h;
						double tang = x * Math.cos(phi) - hs * Math.sin(phi);
						side[sgn][i][j] = vertex(onAxis(r, ac + tang / r, yc + x * Math.sin(phi) + hs * Math.cos(phi)));
					}
				}
			}
			// Wind by grid order: near the thin edges an inside-point test is unreliable. The sign comes from
			// the middle of the blade, where the surface is well clear of the mid-surface.
			int im = span / 2, jm = chord / 2;
			double[] p00 = verts.get(side[0][im][jm] - 1), p10 = verts.get(side[0][im + 1][jm] - 1),
					p11 = verts.get(side[0][im + 1][jm + 1] - 1);
			boolean gridOutward = dot(cross(sub(p10, p00), sub(p11, p00)), sub(p00, mid[im][jm])) > 0;
			for (int sgn = 0; sgn < 2; sgn++) {
				boolean forward = gridOutward == (sgn == 0);
				for (int i = 0; i < span; i++)
					for (int j = 0; j < chord; j++) {
						int a = side[sgn][i][j], b = side[sgn][i + 1][j], c = side[sgn][i + 1][j + 1],
								d = side[sgn][i][j + 1];
						if (forward)
							quad(g, a, b, c, d, null);
						else
							quad(g, d, c, b, a, null);
					}
			}
		}
	}

	/**
	 * Hub: a short cylinder that meets the hull at the front and tapers to a point behind the duct.
	 */
	private void buildHub(Group g) {
		int around = 32, capSteps = 10;
		List<double[]> profile = new ArrayList<>(List.of(HUB));
		double[] last = HUB[HUB.length - 1];
		for (int k = 1; k < capSteps; k++) {
			double s = (double) k / capSteps;
			profile.add(new double[] {last[0] + (HUB_TIP_Y - last[0]) * s, last[1] * (1 - Math.pow(s, HUB_TAPER))});
		}
		int[][] ring = new int[profile.size()][around];
		for (int j = 0; j < profile.size(); j++)
			for (int i = 0; i < around; i++)
				ring[j][i] = vertex(onAxis(profile.get(j)[1], 2 * Math.PI * i / around, profile.get(j)[0]));
		int tip = vertex(onAxis(0, 0, HUB_TIP_Y));
		for (int j = 0; j < profile.size(); j++) {
			double[] inside = onAxis(0, 0, profile.get(j)[0]);
			for (int i = 0; i < around; i++) {
				int i1 = (i + 1) % around;
				if (j + 1 < profile.size())
					quad(g, ring[j][i], ring[j][i1], ring[j + 1][i1], ring[j + 1][i], inside);
				else
					tri(g, ring[j][i], ring[j][i1], tip, inside);
			}
		}
	}

	// ── Mesh helpers ─────────────────────────────────────────────────────────

	private int vertex(double[] p) {
		verts.add(p.clone());
		return verts.size();
	}

	/**
	 * Adds a triangle, wound so its normal points away from {@code inside}, or as given when
	 * {@code inside} is null; degenerate ones are dropped.
	 */
	private void tri(Group g, int a, int b, int c, double[] inside) {
		double[] p0 = verts.get(a - 1), p1 = verts.get(b - 1), p2 = verts.get(c - 1);
		double[] n = cross(sub(p1, p0), sub(p2, p0));
		if (dot(n, n) < 1e-14)
			return;
		double[] centre = {(p0[0] + p1[0] + p2[0]) / 3, (p0[1] + p1[1] + p2[1]) / 3, (p0[2] + p1[2] + p2[2]) / 3};
		if (inside != null && dot(n, sub(centre, inside)) < 0) {
			int t = b;
			b = c;
			c = t;
		}
		g.faces.add(new Corner[] {new Corner(a, 0, 0), new Corner(b, 0, 0), new Corner(c, 0, 0)});
	}

	private void quad(Group g, int a, int b, int c, int d, double[] inside) {
		tri(g, a, b, c, inside);
		tri(g, a, c, d, inside);
	}

	/**
	 * Hexahedron from eight corners: c[0..3] one end ring, c[4..7] the other, in the same order.
	 */
	private void hexa(Group g, double[][] c) {
		double[] centre = new double[3];
		for (double[] p : c)
			for (int k = 0; k < 3; k++)
				centre[k] += p[k] / 8;
		int[][] faces = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
		for (int[] f : faces)
			quad(g, vertex(c[f[0]]), vertex(c[f[1]]), vertex(c[f[2]]), vertex(c[f[3]]), centre);
	}

	/**
	 * Replaces the group's normals: each corner gets the area-weighted sum of the normals of the
	 * faces around its vertex that lie within {@link #CREASE_DEG} of its own face.
	 */
	private void creaseNormals(Group g) {
		double cosCrease = Math.cos(Math.toRadians(CREASE_DEG));
		Map<Integer, List<double[]>> around = new HashMap<>();
		List<double[]> faceN = new ArrayList<>();
		for (Corner[] f : g.faces) {
			double[] n = cross(sub(verts.get(f[1].v - 1), verts.get(f[0].v - 1)),
					sub(verts.get(f[2].v - 1), verts.get(f[0].v - 1)));
			faceN.add(n);
			for (Corner c : f)
				around.computeIfAbsent(c.v, k -> new ArrayList<>()).add(n);
		}
		Map<String, Integer> shared = new HashMap<>();
		for (int fi = 0; fi < g.faces.size(); fi++) {
			Corner[] f = g.faces.get(fi);
			double[] own = unit(faceN.get(fi));
			for (int k = 0; k < f.length; k++) {
				double[] sum = new double[3];
				for (double[] n : around.get(f[k].v))
					if (dot(own, unit(n)) >= cosCrease)
						for (int a = 0; a < 3; a++)
							sum[a] += n[a];
				double[] n = unit(sum);
				String key = String.format(Locale.ROOT, "%.5f %.5f %.5f", n[0], n[1], n[2]);
				Integer idx = shared.get(key);
				if (idx == null) {
					normals.add(n);
					idx = normals.size();
					shared.put(key, idx);
				}
				f[k] = new Corner(f[k].v, f[k].t, idx);
			}
		}
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

	private static double[] unit(double[] a) {
		double len = Math.sqrt(dot(a, a));
		return len > 0 ? new double[] {a[0] / len, a[1] / len, a[2] / len} : new double[] {0, 0, 1};
	}

	private static double num(String s) {
		return Double.parseDouble(s);
	}
}
