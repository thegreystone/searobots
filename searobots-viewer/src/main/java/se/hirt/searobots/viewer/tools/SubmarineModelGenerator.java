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
 * Generates parts of {@code models/submarine-hybrid.obj}. For now it retrofits a new tail and a
 * pump-jet propulsor onto the existing hull: it reads the OBJ, replaces the hull aft of
 * {@link #TAIL_CUT_Y} with a smooth tail cone that runs into the spinner, replaces the
 * {@code Propeller} and {@code PropellerMount} groups with generated geometry, and recomputes
 * normals for the groups it touched. Running it again on its own output gives the same result. The
 * plan is to grow this into a generator for the whole submarine.
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
	// Tail: the hull aft of TAIL_CUT_Y is regenerated. Its radius follows a cubic that leaves the cut with
	// the old hull's own taper and reaches TAIL_JOIN_R at TAIL_JOIN_Y with slope TAIL_JOIN_SLOPE (metres
	// of radius per metre), where the spinner (a hair wider) takes over
	private static final double TAIL_CUT_Y = 20.0, TAIL_JOIN_Y = 37.0, TAIL_JOIN_R = 0.46, TAIL_JOIN_SLOPE = -0.06;
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
	// Hub profile (y, radius), front to back: it starts where the hull's tail ends, 5 mm wider so the
	// seam is hidden, and tapers from the last point to a tip at HUB_TIP_Y as r = R (1 - s^HUB_TAPER)
	private static final double[][] HUB = {{TAIL_JOIN_Y, TAIL_JOIN_R + 0.005}, {37.85, TAIL_JOIN_R + 0.005}};
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
		gen.rebuildTail(gen.groups.get("Body"));
		Group rotor = gen.replace("Propeller", "rubber"); // dark matte, like the fins, for a stealthier look
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
	 * Replaces the hull aft of {@link #TAIL_CUT_Y} with a smooth tail cone. The original tail is a
	 * fan of long, partly twisted sliver triangles that shade as streaks, and it narrowed below the
	 * spinner's radius so the hub bulged out of it. The hull is clipped at the plane, splitting the
	 * triangles that cross it so the cut edge stays watertight. The new tail starts from that edge
	 * with the hull's own taper (measured 3 m further forward), turns round over its first third
	 * and narrows to {@link #TAIL_JOIN_R} where it meets the spinner, then ends just inside the
	 * hub. The rudder roots reach almost to the shaft, so they stay buried. On the generator's own
	 * output the clip removes exactly the previous tail, so reruns are stable.
	 */
	private void rebuildTail(Group body) {
		double ahead = meanRadiusAt(body, TAIL_CUT_Y - 3);
		List<Integer> edge = clipAft(body, TAIL_CUT_Y);
		int n0 = edge.size();
		double[] ang0 = new double[n0], rad0 = new double[n0];
		double r0 = 0;
		for (int i = 0; i < n0; i++) {
			double[] p = verts.get(edge.get(i) - 1);
			ang0[i] = Math.atan2(p[2] - AXIS_Z, p[0]);
			rad0[i] = Math.hypot(p[0], p[2] - AXIS_Z);
			r0 += rad0[i] / n0;
		}
		double length = TAIL_JOIN_Y - TAIL_CUT_Y, startSlope = (r0 - ahead) / 3;
		int around = 40, rings = 20;
		double[] angB = new double[around];
		for (int j = 0; j < around; j++)
			angB[j] = -Math.PI + 2 * Math.PI * j / around;
		int[][] ring = new int[rings + 1][around];
		for (int k = 1; k <= rings; k++) {
			double s = (double) k / rings, y = TAIL_CUT_Y + length * s;
			// Cubic Hermite: value and slope at the cut, value and slope at the join
			double s2 = s * s, s3 = s2 * s;
			double r = (2 * s3 - 3 * s2 + 1) * r0 + (s3 - 2 * s2 + s) * length * startSlope
					+ (-2 * s3 + 3 * s2) * TAIL_JOIN_R + (s3 - s2) * length * TAIL_JOIN_SLOPE;
			double w = smoothstep(Math.min(1, s / 0.35)); // 0 = the cut's own outline, 1 = round
			for (int j = 0; j < around; j++) {
				double shape = (1 - w) * interpolateAround(ang0, rad0, angB[j]) / r0 + w;
				ring[k][j] = vertex(onAxis(r * shape, angB[j], y));
			}
		}
		int first = body.faces.size();
		// Zip the irregular cut edge to the first round ring, walking both by angle
		int p = 0, q = 0;
		while (p < n0 || q < around) {
			double nextA = p < n0 ? (p + 1 < n0 ? ang0[p + 1] : ang0[0] + 2 * Math.PI) : Double.MAX_VALUE;
			double nextB = q < around ? (q + 1 < around ? angB[q + 1] : angB[0] + 2 * Math.PI) : Double.MAX_VALUE;
			int a = edge.get(p % n0), b = ring[1][q % around];
			if (nextB <= nextA) {
				triAboutAxis(body, a, b, ring[1][(q + 1) % around]);
				q++;
			} else {
				triAboutAxis(body, a, b, edge.get((p + 1) % n0));
				p++;
			}
		}
		for (int k = 1; k < rings; k++)
			for (int j = 0; j < around; j++) {
				int j1 = (j + 1) % around;
				triAboutAxis(body, ring[k][j], ring[k][j1], ring[k + 1][j1]);
				triAboutAxis(body, ring[k][j], ring[k + 1][j1], ring[k + 1][j]);
			}
		// A short step inside the hub, closed with a flat cap nobody sees
		int[] inner = new int[around];
		double yIn = TAIL_JOIN_Y + 0.15;
		for (int j = 0; j < around; j++)
			inner[j] = vertex(onAxis(TAIL_JOIN_R - 0.02, angB[j], yIn));
		int centre = vertex(onAxis(0, 0, yIn));
		for (int j = 0; j < around; j++) {
			int j1 = (j + 1) % around;
			triAboutAxis(body, ring[rings][j], ring[rings][j1], inner[j1]);
			triAboutAxis(body, ring[rings][j], inner[j1], inner[j]);
			tri(body, inner[j], inner[j1], centre, onAxis(0, 0, yIn - 1));
		}
		// The hull's other faces carry texture coordinates; give the tail one too so the group is uniform
		uvs.add(new double[] {0, 0});
		int uv = uvs.size();
		for (int i = first; i < body.faces.size(); i++) {
			Corner[] f = body.faces.get(i);
			for (int k = 0; k < f.length; k++)
				f[k] = new Corner(f[k].v, uv, f[k].n);
		}
		System.out.printf(Locale.ROOT,
				"tail: cut at y=%.1f (%d edge vertices, mean radius %.2f, taper %.3f), %d faces to y=%.2f%n",
				TAIL_CUT_Y, n0, r0, startSlope, body.faces.size() - first, TAIL_JOIN_Y);
	}

	/**
	 * Clips the group to y <= {@code yc}. Faces entirely aft are dropped, faces crossing the plane
	 * are cut, sharing the new vertex on each cut edge with the neighbouring face. Returns the
	 * vertices on the cut edge, sorted by angle about the shaft.
	 */
	private List<Integer> clipAft(Group g, double yc) {
		Map<String, Corner> cuts = new HashMap<>();
		List<Corner[]> kept = new ArrayList<>();
		for (Corner[] f : g.faces) {
			double[] d = new double[3];
			boolean front = false, aft = false;
			for (int k = 0; k < 3; k++) {
				d[k] = verts.get(f[k].v - 1)[1] - yc;
				if (Math.abs(d[k]) < 1e-6)
					d[k] = 0;
				front |= d[k] < 0;
				aft |= d[k] > 0;
			}
			if (!aft) {
				kept.add(f);
				continue;
			}
			if (!front)
				continue;
			List<Corner> poly = new ArrayList<>();
			for (int k = 0; k < 3; k++) {
				Corner a = f[k], b = f[(k + 1) % 3];
				double da = d[k], db = d[(k + 1) % 3];
				if (da <= 0)
					poly.add(a);
				if (da * db < 0)
					poly.add(cutEdge(a, b, da / (da - db), cuts));
			}
			for (int k = 1; k + 1 < poly.size(); k++)
				kept.add(new Corner[] {poly.get(0), poly.get(k), poly.get(k + 1)});
		}
		g.faces.clear();
		g.faces.addAll(kept);
		List<Integer> edge = new ArrayList<>();
		for (Corner[] f : kept)
			for (Corner c : f)
				if (Math.abs(verts.get(c.v - 1)[1] - yc) < 1e-6 && !edge.contains(c.v))
					edge.add(c.v);
		edge.sort((a, b) -> Double.compare(angleAbout(a), angleAbout(b)));
		return edge;
	}

	/**
	 * Mean distance from the shaft of the points where the group's edges cross the plane at
	 * {@code y}.
	 */
	private double meanRadiusAt(Group g, double y) {
		double sum = 0;
		int n = 0;
		for (Corner[] f : g.faces)
			for (int k = 0; k < f.length; k++) {
				double[] a = verts.get(f[k].v - 1), b = verts.get(f[(k + 1) % f.length].v - 1);
				if ((a[1] - y) * (b[1] - y) >= 0)
					continue;
				double t = (y - a[1]) / (b[1] - a[1]);
				sum += Math.hypot(a[0] + (b[0] - a[0]) * t, a[2] + (b[2] - a[2]) * t - AXIS_Z);
				n++;
			}
		return sum / n;
	}

	/** The point a fraction {@code t} along edge a-b, created once per edge and shared. */
	private Corner cutEdge(Corner a, Corner b, double t, Map<String, Corner> cuts) {
		String key = Math.min(a.v, b.v) + "-" + Math.max(a.v, b.v);
		Corner c = cuts.get(key);
		if (c == null) {
			double[] p = verts.get(a.v - 1), q = verts.get(b.v - 1);
			int v = vertex(new double[] {p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, p[2] + (q[2] - p[2]) * t});
			int uv = 0;
			if (a.t > 0 && b.t > 0) {
				double[] ua = uvs.get(a.t - 1), ub = uvs.get(b.t - 1);
				uvs.add(new double[] {ua[0] + (ub[0] - ua[0]) * t, ua[1] + (ub[1] - ua[1]) * t});
				uv = uvs.size();
			}
			c = new Corner(v, uv, 0);
			cuts.put(key, c);
		}
		return c;
	}

	private double angleAbout(int v) {
		double[] p = verts.get(v - 1);
		return Math.atan2(p[2] - AXIS_Z, p[0]);
	}

	/**
	 * Linear interpolation of {@code values} at angle {@code a}, around a sorted list of angles.
	 */
	private static double interpolateAround(double[] angles, double[] values, double a) {
		int n = angles.length;
		for (int i = 0; i < n; i++) {
			double a0 = angles[i], a1 = i + 1 < n ? angles[i + 1] : angles[0] + 2 * Math.PI;
			double x = a < a0 ? a + 2 * Math.PI : a;
			if (x >= a0 && x <= a1)
				return values[i] + (values[(i + 1) % n] - values[i]) * (x - a0) / (a1 - a0);
		}
		return values[0];
	}

	private static double smoothstep(double x) {
		return x * x * (3 - 2 * x);
	}

	/** Triangle on a surface around the shaft, wound to face away from it. */
	private void triAboutAxis(Group g, int a, int b, int c) {
		double y = (verts.get(a - 1)[1] + verts.get(b - 1)[1] + verts.get(c - 1)[1]) / 3;
		tri(g, a, b, c, onAxis(0, 0, y));
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

	/**
	 * Adds a vertex, rounded to the precision the OBJ is written with so that geometry derived from
	 * it comes out the same when the generator reads its own output.
	 */
	private int vertex(double[] p) {
		verts.add(new double[] {round6(p[0]), round6(p[1]), round6(p[2])});
		return verts.size();
	}

	private static double round6(double x) {
		return Math.round(x * 1e6) / 1e6;
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
