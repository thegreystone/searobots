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
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.Path2D;
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
 * Generates the torpedo model: a white heavyweight torpedo with a blunt seeker nose, a long
 * cylindrical body, a boat-tail afterbody with four fins inside the body's diameter (it has to fit
 * a tube), and contra-rotating propellers. The paint is one texture unrolled over the whole body:
 * section joints with their screws, and fictional warning and identification stencils.
 * <p>
 * Model units and axes are those of the hand-made model it replaces, so the viewer's scale (0.19,
 * about 5 m long) and the team band (TorpedoRing) carry over: the axis along +Y from the nose (y =
 * {@link #NOSE_Y}) to the tail, Z up, +X to port, body radius {@link #R}. The propellers are the
 * groups Propeller and PropellerAft, each round the axis so they can spin in place.
 * <p>
 * Usage: {@code TorpedoModelGenerator <out-dir> [basename]}, default basename {@code torpedo}.
 */
public final class TorpedoModelGenerator {
	// Body profile, in model units
	private static final double R = 1.41421, NOSE_Y = -11.58, NOSE_END_Y = -9.6;
	private static final double TAPER_Y0 = 4.0, TAPER_Y1 = 11.8, HUB_R = 0.55, CONE_Y = 13.6, TAIL_Y = 14.75;
	// The seeker's acoustic window: the front of the nose, out to this radius
	private static final double WINDOW_R = 0.95;
	// Section joints (painted, with screws) and the team band the viewer adds (kept clear of stencils)
	private static final double[] JOINTS = {-7.5, -1.0, TAPER_Y0};
	// Fins: root and tip, leading and trailing edge stations, and the tip's radius (inside R)
	private static final double FIN_ROOT_LE = 8.3, FIN_ROOT_TE = 11.0, FIN_TIP_LE = 9.6, FIN_TIP_TE = 11.0;
	private static final double FIN_TIP_R = 1.36, FIN_THICKNESS = 0.1;
	// Contra-rotating propellers: station, blade count, tip radius and pitch hand
	private static final double PROP_Y = 12.25, PROP_AFT_Y = 12.95;
	// Each propeller sits on its own bronze hub block, BLOCK_HALF either side of it and flush with the hub (which leaves
	// those stretches to the blocks), turning with it; thin bands of bare metal on the fixed hub show before, between
	// and after the blocks
	private static final double BLOCK_HALF = 0.3, BAND = 0.08;
	private static final double[][] BANDS = {{PROP_Y - BLOCK_HALF - BAND, PROP_Y - BLOCK_HALF},
			{PROP_Y + BLOCK_HALF, PROP_AFT_Y - BLOCK_HALF}, {PROP_AFT_Y + BLOCK_HALF, PROP_AFT_Y + BLOCK_HALF + BAND}};
	// Hoist lugs on the top, at these stations
	private static final double[] LUGS = {-5.6, 1.5};
	// The paint texture: the whole body unrolled, along it (nose to tail) by round it
	private static final int TEX_W = 3072, TEX_H = 1024;
	private static final double PX_PER_UNIT = TEX_W / (TAIL_Y - NOSE_Y);
	private static final String PAINT = "torpedo-paint.png";

	private record Mat(double[] kd, double ks, double ns, String map, String comment) {
	}

	private static final Map<String, Mat> MATERIALS = new LinkedHashMap<>();

	static {
		MATERIALS.put("Torpedo_Paint", new Mat(new double[] {0.9, 0.9, 0.9}, 0.3, 25, PAINT,
				"Body: white satin paint with the joints and stencils painted on"));
		MATERIALS.put("Torpedo_White",
				new Mat(new double[] {0.82, 0.82, 0.82}, 0.3, 25, null, "Fins: the same white paint, plain"));
		MATERIALS.put("Seeker_Window", new Mat(new double[] {0.55, 0.56, 0.57}, 0.45, 40, null,
				"Acoustic window over the seeker: unpainted grey composite, a little glossier"));
		MATERIALS.put("Propeller_Bronze",
				new Mat(new double[] {0.55, 0.40, 0.22}, 0.65, 45, null, "Propellers: nickel-aluminium bronze"));
		MATERIALS.put("Fittings_Steel",
				new Mat(new double[] {0.32, 0.33, 0.35}, 0.5, 30, null, "Hoist lugs: dark steel"));
		MATERIALS.put("Metal_Silver", new Mat(new double[] {0.42, 0.44, 0.47}, 0.95, 90, null,
				"Band on the hub between the propellers: bare polished metal, darker than the white paint with a sharp "
						+ "highlight, so it reads as metal against it"));
	}

	private record Corner(int v, int t) {
	}

	private static final class Group {
		final String name, material;
		final List<Corner[]> faces = new ArrayList<>();

		Group(String name, String material) {
			this.name = name;
			this.material = material;
		}
	}

	private final List<double[]> verts = new ArrayList<>();
	private final List<double[]> uvs = new ArrayList<>();
	private final Map<String, Group> groups = new LinkedHashMap<>();

	public static void main(String[] args) throws IOException {
		Path out = Path.of(args.length > 0 ? args[0] : ".");
		String base = args.length > 1 ? args[1] : "torpedo";
		Files.createDirectories(out);
		var gen = new TorpedoModelGenerator();
		gen.buildBody(gen.group("Body", "Torpedo_Paint"), gen.group("SeekerWindow", "Seeker_Window"),
				gen.group("HubBand", "Metal_Silver"));
		gen.buildFins(gen.group("Fins", "Torpedo_White"));
		gen.buildFittings(gen.group("Fittings", "Fittings_Steel"));
		gen.buildPropeller(gen.group("Propeller", "Propeller_Bronze"), PROP_Y, 6, 1.42, 1);
		gen.buildPropeller(gen.group("PropellerAft", "Propeller_Bronze"), PROP_AFT_Y, 5, 1.3, -1);
		ImageIO.write(paintTexture(), "png", out.resolve(PAINT).toFile());
		gen.write(out.resolve(base + ".obj"), out.resolve(base + ".mtl"), base + ".mtl");
	}

	private Group group(String name, String material) {
		return groups.computeIfAbsent(name, n -> new Group(n, material));
	}

	// ── Body ─────────────────────────────────────────────────────────────────

	/** Radius of the body at station {@code y}. */
	static double radius(double y) {
		if (y <= NOSE_Y)
			return 0;
		if (y < NOSE_END_Y) { // blunt semi-ellipse
			double t = (NOSE_END_Y - y) / (NOSE_END_Y - NOSE_Y);
			return R * Math.sqrt(Math.max(0, 1 - t * t));
		}
		if (y <= TAPER_Y0)
			return R;
		if (y < TAPER_Y1) { // boat tail, joining the cylinder and the hub without a kink
			double s = (y - TAPER_Y0) / (TAPER_Y1 - TAPER_Y0);
			return HUB_R + (R - HUB_R) * (1 - s * s * (3 - 2 * s));
		}
		if (y <= CONE_Y)
			return HUB_R;
		double s = Math.min(1, (y - CONE_Y) / (TAIL_Y - CONE_Y)); // rounded tail cone, closing to a point
		return HUB_R * Math.sqrt(1 - s * s);
	}

	/** Angle round the axis from the bottom, positive towards starboard (-X), for the paint. */
	private static double[] onBody(double y, double r, double theta) {
		return new double[] {-r * Math.sin(theta), y, -r * Math.cos(theta)};
	}

	/**
	 * The body as a surface of revolution, with the paint texture unrolled over it (u along, v
	 * round), the front of the nose split off as the seeker window, the hub between the propellers
	 * as thin bands of bare metal round the propellers' blocks, and a rounded tail closing to a
	 * point.
	 */
	private void buildBody(Group paint, Group window, Group band) {
		int around = 40;
		List<Double> stations = new ArrayList<>();
		for (int k = 1; k <= 14; k++) // bunched up towards the tip
			stations.add(NOSE_Y + (NOSE_END_Y - NOSE_Y) * (1 - Math.cos(Math.PI / 2 * k / 14)));
		for (double y = NOSE_END_Y + 1.0; y < TAPER_Y0; y += 1.0)
			stations.add(y);
		for (int k = 0; k <= 20; k++)
			stations.add(TAPER_Y0 + (TAPER_Y1 - TAPER_Y0) * k / 20);
		for (double[] b : BANDS) {
			stations.add(b[0]);
			stations.add(b[1]);
		}
		stations.add(CONE_Y);
		for (int k = 1; k <= 8; k++)
			stations.add(CONE_Y + (TAIL_Y - CONE_Y) * k / 8);
		int[][] ring = new int[stations.size()][around + 1];
		for (int k = 0; k < stations.size(); k++) {
			double y = stations.get(k), r = radius(y);
			for (int j = 0; j <= around; j++) {
				double theta = 2 * Math.PI * j / around;
				ring[k][j] = vertex(onBody(y, r, theta),
						new double[] {(y - NOSE_Y) / (TAIL_Y - NOSE_Y), 1 - theta / (2 * Math.PI)});
			}
		}
		int tip = vertex(new double[] {0, NOSE_Y, 0}, new double[] {0, 0.5});
		for (int j = 0; j < around; j++)
			tri(window, tip, ring[0][j + 1], ring[0][j]);
		for (int k = 0; k + 1 < stations.size(); k++) {
			double y0 = stations.get(k), y1 = stations.get(k + 1);
			boolean inBand = false;
			for (double[] b : BANDS)
				inBand |= y0 >= b[0] - 1e-9 && y1 <= b[1] + 1e-9;
			// The propellers' blocks replace the hub's surface under them
			if (Math.abs(y0 - BANDS[0][1]) < 1e-9 && Math.abs(y1 - BANDS[1][0]) < 1e-9
					|| Math.abs(y0 - BANDS[1][1]) < 1e-9 && Math.abs(y1 - BANDS[2][0]) < 1e-9)
				continue;
			Group g = y1 <= NOSE_END_Y && radius(y1) <= WINDOW_R ? window : inBand ? band : paint;
			for (int j = 0; j < around; j++)
				quad(g, ring[k][j], ring[k][j + 1], ring[k + 1][j + 1], ring[k + 1][j]);
		}
	}

	// ── Fins ─────────────────────────────────────────────────────────────────

	/**
	 * Four fins in a cross (up, down, port, starboard): swept leading edge, straight trailing edge,
	 * a 10% symmetric section, roots buried in the boat tail, tips inside the body's radius.
	 */
	private void buildFins(Group g) {
		int span = 5, chord = 10;
		for (int f = 0; f < 4; f++) {
			double a = Math.PI / 2 * f;
			double[] out = {Math.cos(a), 0, Math.sin(a)}, across = {-Math.sin(a), 0, Math.cos(a)};
			int[][] grid = new int[span + 1][2 * chord];
			for (int i = 0; i <= span; i++) {
				double t = (double) i / span;
				double le = FIN_ROOT_LE + (FIN_TIP_LE - FIN_ROOT_LE) * t,
						te = FIN_ROOT_TE + (FIN_TIP_TE - FIN_ROOT_TE) * t;
				double rootR = radius((le + te) / 2) - 0.05, r = rootR + (FIN_TIP_R - rootR) * t, c = te - le;
				for (int j = 0; j < 2 * chord; j++) {
					int k = j < chord ? j : 2 * chord - j; // round the section: upper side, then lower
					double s = 0.5 * (1 - Math.cos(Math.PI * k / chord));
					double half = (j == 0 || k == chord) ? 0 : naca(s, FIN_THICKNESS) * c * (j < chord ? 1 : -1);
					double y = le + s * c;
					grid[i][j] = vertex(new double[] {out[0] * r + across[0] * half, y, out[2] * r + across[2] * half},
							new double[] {0, 0});
				}
			}
			for (int i = 0; i < span; i++)
				for (int j = 0; j < 2 * chord; j++) {
					int j1 = (j + 1) % (2 * chord);
					quadOut(g, grid[i][j], grid[i][j1], grid[i + 1][j1], grid[i + 1][j]);
				}
			// Flat tip
			int[] tipLoop = grid[span];
			for (int j = 1; j + 1 < 2 * chord; j++)
				triOut(g, tipLoop[0], tipLoop[j], tipLoop[j + 1], out);
		}
	}

	private static double naca(double s, double t) {
		return 5 * t
				* (0.2969 * Math.sqrt(s) - 0.1260 * s - 0.3516 * s * s + 0.2843 * s * s * s - 0.1036 * s * s * s * s);
	}

	// ── Fittings ─────────────────────────────────────────────────────────────

	/** Hoist lugs on the top: low rounded blocks with a slot, flush to the curve of the body. */
	private void buildFittings(Group g) {
		for (double y : LUGS) {
			int n = 24;
			double hx = 0.32, hy = 0.5, corner = 0.15, h = 0.12;
			List<double[]> outline = new ArrayList<>();
			for (int q = 0; q < 4; q++) {
				double cx = (q == 0 || q == 3 ? 1 : -1) * (hx - corner), cy = (q < 2 ? 1 : -1) * (hy - corner);
				for (int i = 0; i <= n / 4; i++) {
					double a = Math.PI / 2 * q + Math.PI / 2 * i / (n / 4);
					outline.add(new double[] {cx + corner * Math.cos(a), y + cy + corner * Math.sin(a)});
				}
			}
			int m = outline.size();
			int[] top = new int[m], bottom = new int[m];
			for (int i = 0; i < m; i++) {
				double x = outline.get(i)[0], yy = outline.get(i)[1], z = Math.sqrt(R * R - x * x);
				top[i] = vertex(new double[] {x, yy, z + h}, new double[] {0, 0});
				bottom[i] = vertex(new double[] {x, yy, z - 0.05}, new double[] {0, 0});
			}
			int hub = vertex(new double[] {0, y, R + h}, new double[] {0, 0});
			for (int i = 0; i < m; i++) {
				int j = (i + 1) % m;
				triOut(g, hub, top[i], top[j], new double[] {0, 0, 1});
				quadAway(g, top[i], top[j], bottom[j], bottom[i], new double[] {0, y, R - 0.2});
			}
		}
	}

	// ── Propellers ───────────────────────────────────────────────────────────

	/**
	 * A propeller of {@code blades} skewed blades round the hub at station {@code y}, of the given
	 * hand (+1 or -1, the two of a contra-rotating pair turn opposite ways and are pitched opposite
	 * ways). Each blade: a thin elliptical section on a helix, wide in the middle and rounded at
	 * the tip, skewed back against the rotation; the root buried in the hub.
	 */
	private void buildPropeller(Group g, double y, int blades, double tipR, int hand) {
		buildHubBlock(g, y);
		int span = 8, chord = 6;
		double pitch = 3.2 * hand, skew = 0.6 * hand;
		for (int b = 0; b < blades; b++) {
			double a0 = 2 * Math.PI * b / blades + (hand < 0 ? Math.PI / blades : 0);
			int[][][] side = new int[2][span + 1][chord + 1];
			for (int i = 0; i <= span; i++) {
				double u = Math.sin(Math.PI / 2 * i / span);
				double r = HUB_R - 0.05 + (tipR - HUB_R + 0.05) * u;
				double tipRound = u > 0.75 ? Math.sqrt(Math.max(0, 1 - Math.pow((u - 0.75) / 0.25, 2))) : 1;
				double c = 1.15 * (0.42 - 0.04 * u + 0.08 * Math.sin(Math.PI * Math.min(1, u / 0.85) * 0.9)) * tipRound;
				double thick = 0.045 * (1 - 0.7 * u) * Math.sqrt(tipRound);
				double phi = Math.atan(pitch / (2 * Math.PI * r)), ac = a0 + skew * u * u;
				for (int j = 0; j <= chord; j++) {
					double x = c * (0.5 * (1 - Math.cos(Math.PI * j / chord)) - 0.5);
					double h = c > 1e-9 ? thick * Math.sqrt(Math.max(0, 1 - Math.pow(2 * x / c, 2))) : 0;
					for (int sgn = 0; sgn < 2; sgn++) {
						if (sgn == 1 && (j == 0 || j == chord)) {
							side[1][i][j] = side[0][i][j];
							continue;
						}
						double hs = sgn == 0 ? h : -h;
						double tang = x * Math.cos(phi) - hs * Math.sin(phi);
						double along = x * Math.sin(phi) + hs * Math.cos(phi);
						double ang = ac + tang / r;
						side[sgn][i][j] = vertex(new double[] {r * Math.cos(ang), y + along, r * Math.sin(ang)},
								new double[] {0, 0});
					}
				}
			}
			// Each side faces away from the blade's mid-surface; decide from the middle of the blade
			for (int sgn = 0; sgn < 2; sgn++)
				for (int i = 0; i < span; i++)
					for (int j = 0; j < chord; j++) {
						int p = side[sgn][i][j], q = side[sgn][i + 1][j], s = side[sgn][i + 1][j + 1],
								t = side[sgn][i][j + 1];
						int other = side[1 - sgn][i][j];
						double[] mid = verts.get(other - 1);
						quadAway(g, p, q, s, t, mid);
					}
		}
	}

	/**
	 * The block a propeller's blades are set in: a short cylinder of the hub's radius at station
	 * {@code y}, between two of the bands. Part of the propeller's group, so it turns with the
	 * blades.
	 */
	private void buildHubBlock(Group g, double y) {
		int around = 40;
		int[][] ring = new int[2][around + 1];
		for (int k = 0; k < 2; k++)
			for (int j = 0; j <= around; j++)
				ring[k][j] = vertex(onBody(y + (k == 0 ? -BLOCK_HALF : BLOCK_HALF), HUB_R, 2 * Math.PI * j / around),
						new double[] {0, 0});
		for (int j = 0; j < around; j++)
			quad(g, ring[0][j], ring[0][j + 1], ring[1][j + 1], ring[1][j]);
	}

	// ── Mesh helpers ─────────────────────────────────────────────────────────

	private int vertex(double[] p, double[] uv) {
		verts.add(p);
		uvs.add(uv);
		return verts.size();
	}

	private double[] v(int i) {
		return verts.get(i - 1);
	}

	/** Triangle wound away from the axis (the body is convex about it). */
	private void tri(Group g, int a, int b, int c) {
		double[] centre = {0, (v(a)[1] + v(b)[1] + v(c)[1]) / 3, 0};
		triAway(g, a, b, c, centre);
	}

	private void quad(Group g, int a, int b, int c, int d) {
		tri(g, a, b, c);
		tri(g, a, c, d);
	}

	private void triAway(Group g, int a, int b, int c, double[] inside) {
		double[] n = normal(a, b, c), toFace = sub(centroid(a, b, c), inside);
		if (dot(n, toFace) >= 0)
			g.faces.add(new Corner[] {new Corner(a, a), new Corner(b, b), new Corner(c, c)});
		else
			g.faces.add(new Corner[] {new Corner(a, a), new Corner(c, c), new Corner(b, b)});
	}

	private void quadAway(Group g, int a, int b, int c, int d, double[] inside) {
		triAway(g, a, b, c, inside);
		triAway(g, a, c, d, inside);
	}

	/** Triangle facing along {@code dir}. */
	private void triOut(Group g, int a, int b, int c, double[] dir) {
		double[] n = normal(a, b, c);
		if (dot(n, dir) >= 0)
			g.faces.add(new Corner[] {new Corner(a, a), new Corner(b, b), new Corner(c, c)});
		else
			g.faces.add(new Corner[] {new Corner(a, a), new Corner(c, c), new Corner(b, b)});
	}

	/** Quad on a fin, facing away from the fin's mid-plane at its root. */
	private void quadOut(Group g, int a, int b, int c, int d) {
		double[] ctr = centroid(a, b, c);
		double r = Math.hypot(ctr[0], ctr[2]);
		// The mid-plane point: the same radius and station, on the fin's own spanwise line
		double ang = Math.round(Math.atan2(ctr[2], ctr[0]) / (Math.PI / 2)) * (Math.PI / 2);
		double[] inside = {r * Math.cos(ang), ctr[1], r * Math.sin(ang)};
		quadAway(g, a, b, c, d, inside);
	}

	private double[] normal(int a, int b, int c) {
		return cross(sub(v(b), v(a)), sub(v(c), v(a)));
	}

	private double[] centroid(int a, int b, int c) {
		double[] p = v(a), q = v(b), s = v(c);
		return new double[] {(p[0] + q[0] + s[0]) / 3, (p[1] + q[1] + s[1]) / 3, (p[2] + q[2] + s[2]) / 3};
	}

	private static double[] sub(double[] a, double[] b) {
		return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
	}

	private static double dot(double[] a, double[] b) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	private static double[] cross(double[] a, double[] b) {
		return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
	}

	// ── Output ───────────────────────────────────────────────────────────────

	/**
	 * Writes the OBJ and MTL. Normals are averaged per group over the faces round each position
	 * that lie within 50 degrees of each other, so the texture seam does not show and edges stay
	 * sharp.
	 */
	private void write(Path objPath, Path mtlPath, String mtlName) throws IOException {
		try (PrintWriter m = new PrintWriter(Files.newBufferedWriter(mtlPath, StandardCharsets.US_ASCII))) {
			m.print("# Torpedo materials. Generated by TorpedoModelGenerator.\n#\n");
			for (var e : MATERIALS.entrySet()) {
				Mat c = e.getValue();
				m.printf(Locale.ROOT, "# %s\nnewmtl %s\nKa  %.2f %.2f %.2f\nKd  %.2f %.2f %.2f\nKs  %.2f %.2f %.2f\n",
						c.comment(), e.getKey(), c.kd()[0] * 0.8, c.kd()[1] * 0.8, c.kd()[2] * 0.8, c.kd()[0],
						c.kd()[1], c.kd()[2], c.ks(), c.ks(), c.ks());
				m.printf(Locale.ROOT, "d  1.0\nNs  %.1f\nillum 2\n", c.ns());
				if (c.map() != null)
					m.print("map_Kd " + c.map() + "\n");
				m.print("#\n");
			}
		}
		double cosCrease = Math.cos(Math.toRadians(50));
		StringBuilder sv = new StringBuilder(), st = new StringBuilder(), sn = new StringBuilder(),
				sf = new StringBuilder();
		for (double[] p : verts)
			sv.append(String.format(Locale.ROOT, "v %.6f %.6f %.6f\n", p[0], p[1], p[2]));
		for (double[] t : uvs)
			st.append(String.format(Locale.ROOT, "vt %.5f %.5f\n", t[0], t[1]));
		int normalCount = 0;
		for (Group g : groups.values()) {
			if (g.faces.isEmpty())
				continue;
			// Face normals, and the faces round each position
			Map<String, List<double[]>> round = new HashMap<>();
			List<double[]> faceNormals = new ArrayList<>();
			for (Corner[] f : g.faces) {
				double[] n = normal(f[0].v, f[1].v, f[2].v);
				faceNormals.add(n);
				for (Corner c : f)
					round.computeIfAbsent(key(v(c.v)), k -> new ArrayList<>()).add(n);
			}
			sf.append("g ").append(g.name).append('\n').append("usemtl ").append(g.material).append('\n');
			for (int fi = 0; fi < g.faces.size(); fi++) {
				Corner[] f = g.faces.get(fi);
				double[] own = unit(faceNormals.get(fi));
				sf.append('f');
				for (Corner c : f) {
					double[] sum = new double[3];
					for (double[] n : round.get(key(v(c.v))))
						if (dot(unit(n), own) >= cosCrease)
							for (int k = 0; k < 3; k++)
								sum[k] += n[k];
					double[] n = unit(sum);
					sn.append(String.format(Locale.ROOT, "vn %.4f %.4f %.4f\n", n[0], n[1], n[2]));
					normalCount++;
					sf.append(' ').append(c.v).append('/').append(c.t).append('/').append(normalCount);
				}
				sf.append('\n');
			}
		}
		Files.writeString(objPath,
				"# Torpedo. Generated by TorpedoModelGenerator.\nmtllib " + mtlName + "\n" + sv + st + sn + sf,
				StandardCharsets.US_ASCII);
		int faces = groups.values().stream().mapToInt(g -> g.faces.size()).sum();
		System.out.println("vertices=" + verts.size() + " faces=" + faces + " -> " + objPath);
	}

	private static String key(double[] p) {
		return String.format(Locale.ROOT, "%.5f %.5f %.5f", p[0], p[1], p[2]);
	}

	private static double[] unit(double[] a) {
		double l = Math.sqrt(dot(a, a));
		return l < 1e-12 ? new double[] {0, 0, 1} : new double[] {a[0] / l, a[1] / l, a[2] / l};
	}

	// ── Paint ────────────────────────────────────────────────────────────────

	/** Column of the paint texture at station {@code y}. */
	private static double col(double y) {
		return (y - NOSE_Y) * PX_PER_UNIT;
	}

	/**
	 * The paint: off-white with faint mottling, the section joints as dark lines with screws, a
	 * hazard band ahead of the propellers, and stencils on both flanks and on the top. Rows run
	 * round the body from the bottom (row 0) towards starboard; the port flank is at 3/4 of the
	 * height, the top at 1/2, the starboard flank at 1/4. Text reads towards the tail on the port
	 * side and towards the nose on the starboard side, upright on both, as on the submarine.
	 */
	static BufferedImage paintTexture() {
		var img = new BufferedImage(TEX_W, TEX_H, BufferedImage.TYPE_INT_RGB);
		Random r = new Random(41);
		float[] noise = smoothNoise(TEX_W, TEX_H, 48, r);
		for (int y = 0; y < TEX_H; y++)
			for (int x = 0; x < TEX_W; x++) {
				double v = 0.93 + 0.035 * (noise[y * TEX_W + x] - 0.5) + 0.01 * (r.nextDouble() - 0.5);
				int c = (int) Math.round(255 * v);
				img.setRGB(x, y, (c << 16) | (c << 8) | Math.min(255, c + 2));
			}
		Graphics2D g = img.createGraphics();
		g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
		g.setRenderingHint(RenderingHints.KEY_FRACTIONALMETRICS, RenderingHints.VALUE_FRACTIONALMETRICS_ON);
		Color stencil = new Color(28, 30, 32), red = new Color(178, 28, 22), yellow = new Color(232, 190, 40);
		// Section joints: a fine dark line round the body, screws either side every 30 degrees
		for (double j : JOINTS) {
			int x = (int) Math.round(col(j));
			g.setColor(new Color(90, 92, 95));
			g.fillRect(x - 1, 0, 3, TEX_H);
			g.setColor(new Color(150, 152, 155));
			for (int k = 0; k < 12; k++) {
				int yy = (int) Math.round((k + 0.5) * TEX_H / 12.0);
				g.fillOval(x - 15, yy - 5, 10, 10);
				g.fillOval(x + 6, yy - 5, 10, 10);
			}
		}
		// Hazard band ahead of the propellers, on the boat tail behind the fins
		int hx0 = (int) Math.round(col(11.2)), hx1 = (int) Math.round(col(11.65));
		g.setColor(yellow);
		g.fillRect(hx0, 0, hx1 - hx0, TEX_H);
		g.setColor(stencil);
		g.setClip(hx0, 0, hx1 - hx0, TEX_H);
		for (int y = -TEX_H; y < 2 * TEX_H; y += 64) {
			Path2D stripe = new Path2D.Double();
			stripe.moveTo(hx0, y);
			stripe.lineTo(hx0, y + 32);
			stripe.lineTo(hx1, y + 32 + (hx1 - hx0));
			stripe.lineTo(hx1, y + (hx1 - hx0));
			stripe.closePath();
			g.fill(stripe);
		}
		g.setClip(null);
		for (boolean starboard : new boolean[] {false, true}) {
			// Warhead: DANGER panel in red
			box(g, red, starboard, -5.6, 0.05, 3.2, 1.25, 7);
			text(g, "DANGER", 0.5, red, starboard, -5.6, 0.32);
			text(g, "HIGH EXPLOSIVE", 0.26, red, starboard, -5.6, -0.22);
			// Seeker section
			text(g, "ACOUSTIC WINDOW - DO NOT PAINT", 0.13, stencil, starboard, -9.15, -0.55);
			text(g, "SECT 1  SEEKER", 0.17, stencil, starboard, -8.4, 0.0);
			// Identification on the after body
			text(g, "SR-28 HEAVYWEIGHT", 0.3, stencil, starboard, 1.5, 0.32);
			text(g, "MOD 2   LOT 0417-B   AUTONOMOUS", 0.15, stencil, starboard, 1.5, -0.08);
			text(g, "DO NOT DROP  -  SHOCK SENSITIVE", 0.15, red, starboard, 1.5, -0.38);
			// Propeller warning on the boat tail, ahead of the fins
			text(g, "KEEP CLEAR OF PROPELLERS", 0.13, stencil, starboard, 6.0, -0.15);
			arrow(g, stencil, starboard, 6.0, 0.16, 7.2);
		}
		// Top: hoist marks by the lugs, and the alignment line
		for (double lug : LUGS) {
			topText(g, "HOIST", 0.17, stencil, lug - 0.95);
			topText(g, "HOIST", 0.17, stencil, lug + 0.95);
		}
		topText(g, "TOP", 0.2, stencil, -8.55);
		g.setColor(stencil);
		int top = TEX_H / 2;
		g.fillRect((int) Math.round(col(-9.2)), top - 2, (int) Math.round(col(-7.9) - col(-9.2)), 4);
		g.dispose();
		return img;
	}

	/** Row of a point {@code up} units above the flank's centreline. */
	private static double row(boolean starboard, double up) {
		return starboard ? TEX_H / 4.0 + up * PX_PER_UNIT : 3 * TEX_H / 4.0 - up * PX_PER_UNIT;
	}

	/** Text of cap height {@code height} units centred at a station on a flank. */
	private static void text(
		Graphics2D g, String s, double height, Color c, boolean starboard, double station, double up) {
		drawCentred(g, s, height, c, col(station), row(starboard, up), starboard ? Math.PI : 0);
	}

	/** Text on the top, reading towards the tail as seen from the port side. */
	private static void topText(Graphics2D g, String s, double height, Color c, double station) {
		drawCentred(g, s, height, c, col(station), TEX_H / 2.0 + 0.3 * PX_PER_UNIT, 0);
	}

	private static void drawCentred(Graphics2D g, String s, double height, Color c, double cx, double cy, double rot) {
		var saved = g.getTransform();
		g.rotate(rot, cx, cy);
		Font font = new Font(Font.SANS_SERIF, Font.BOLD, 100);
		double cap = font.createGlyphVector(g.getFontRenderContext(), "H").getVisualBounds().getHeight();
		font = font.deriveFont((float) (100 * height * PX_PER_UNIT / cap));
		g.setFont(font);
		FontMetrics fm = g.getFontMetrics();
		g.setColor(c);
		g.drawString(s, (float) (cx - fm.stringWidth(s) / 2.0), (float) (cy + height * PX_PER_UNIT / 2));
		g.setTransform(saved);
	}

	/** An outlined box centred at a station on a flank, {@code w} by {@code h} units. */
	private static void box(
		Graphics2D g, Color c, boolean starboard, double station, double up, double w, double h, float stroke) {
		g.setColor(c);
		g.setStroke(new BasicStroke(stroke));
		double cx = col(station), cy = row(starboard, up);
		g.drawRect((int) Math.round(cx - w / 2 * PX_PER_UNIT), (int) Math.round(cy - h / 2 * PX_PER_UNIT),
				(int) Math.round(w * PX_PER_UNIT), (int) Math.round(h * PX_PER_UNIT));
	}

	/**
	 * An arrow on a flank from a station towards {@code toStation}, {@code up} units above the
	 * centreline.
	 */
	private static void arrow(Graphics2D g, Color c, boolean starboard, double station, double up, double toStation) {
		double y = row(starboard, up), x0 = col(station - 0.9), x1 = col(toStation);
		g.setColor(c);
		g.setStroke(new BasicStroke(5));
		g.drawLine((int) x0, (int) y, (int) x1, (int) y);
		Path2D head = new Path2D.Double();
		head.moveTo(x1 + 22, y);
		head.lineTo(x1 - 4, y - 14);
		head.lineTo(x1 - 4, y + 14);
		head.closePath();
		g.fill(head);
		g.setComposite(AlphaComposite.SrcOver);
	}

	/** Smooth value noise in [0, 1], bilinearly interpolated from a coarse random lattice. */
	private static float[] smoothNoise(int w, int h, int cell, Random r) {
		int gw = w / cell + 2, gh = h / cell + 2;
		float[] lattice = new float[gw * gh];
		for (int i = 0; i < lattice.length; i++)
			lattice[i] = r.nextFloat();
		float[] out = new float[w * h];
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int gx = x / cell, gy = y / cell;
				float fx = (float) (x % cell) / cell, fy = (float) (y % cell) / cell;
				fx = fx * fx * (3 - 2 * fx);
				fy = fy * fy * (3 - 2 * fy);
				float a = lattice[gy * gw + gx], b = lattice[gy * gw + gx + 1], c = lattice[(gy + 1) * gw + gx],
						d = lattice[(gy + 1) * gw + gx + 1];
				out[y * w + x] = (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
			}
		return out;
	}
}
