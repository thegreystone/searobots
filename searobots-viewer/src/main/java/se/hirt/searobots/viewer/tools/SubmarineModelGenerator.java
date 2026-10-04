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
 * Generates {@code models/submarine-hybrid.obj} and {@code submarine-hybrid.mtl}: the 75 m attack
 * submarine. Pure Java, no dependencies.
 * <p>
 * Model conventions as in the viewer: X across, Y fore-aft with the bow at -Y and the stern at +Y,
 * Z up, metres. The shaft runs along Y through x = 0, z = {@link #AXIS_Z}, which is also the
 * centreline of the hull's sections. The viewer animates named groups about fixed pivots
 * (SubmarineScene3D): {@code Propeller} spins about the shaft, the four X-tail flaps
 * ({@code tailflap_pu/su/sl/pl}) turn about the hinge lines this generator prints, and
 * {@code elevatorl}/{@code elevatorr} (starboard/port bow planes) tilt about an athwartships axis
 * through (+-4.3, -10, 0). The parts are built around those pivots, so the viewer needs no changes.
 * <ul>
 * <li>{@code Body}: elliptical sections on a superellipse forebody, a parallel midbody and an
 * afterbody that narrows into the spinner without a step.</li>
 * <li>{@code Tower}: the sail, an aerofoil section with a raked leading edge and a rounded
 * top.</li>
 * <li>Bow planes and rudders: aerofoil fins with rounded tips; their roots are buried deep enough
 * to stay inside the hull through their full travel.</li>
 * <li>{@code Propeller} (rotating) and {@code PropellerMount} (fixed): a pump-jet with seven skewed
 * blades on a slim tapered spinner, inside a duct held by five stator vanes.</li>
 * </ul>
 * Normals are area weighted and split at {@link #CREASE_DEG}, so curved surfaces are smooth and
 * real edges stay crisp.
 * <p>
 * Usage: {@code SubmarineModelGenerator <out-dir>}, for example
 * {@code java -cp target/classes se.hirt.searobots.viewer.tools.SubmarineModelGenerator src/main/resources/models}.
 */
public final class SubmarineModelGenerator {

	private static final double AXIS_Z = 0.11;
	private static final double CREASE_DEG = 50;

	// Hull: superellipse forebody from the nose to the widest point, parallel midbody, then an afterbody
	// r = R + (W - R) (1 - s^a)^b that meets the spinner (radius TAIL_JOIN_R) with zero slope
	private static final double NOSE_Y = -34.15, MID_Y0 = -14.0, MID_Y1 = -8.0, HULL_HALF_WIDTH = 4.47;
	private static final double FORE_EXPONENT = 2.5, AFT_A = 2.4, AFT_B = 1.05;
	private static final double TAIL_JOIN_Y = 37.0, TAIL_JOIN_R = 0.75;
	// Sections are ellipses this much taller than wide; they turn round towards the tail to meet the spinner
	private static final double HEIGHT_RATIO = 0.83, ROUND_FROM_Y = 12.0, ROUND_BY_Y = 28.0;

	// Sail: leading and trailing edge at the hull top (z = SAIL_BASE_Z) and at SAIL_TOP_Z, where a small rounded
	// edge of radius SAIL_EDGE turns into a flat top (room for the bridge, hatches and masts)
	private static final double SAIL_BASE_Z = 3.82, SAIL_TOP_Z = 6.26, SAIL_EDGE = 0.12, SAIL_HALF_WIDTH = 0.85;
	private static final double[] SAIL_BASE = {-18.5, -10.84}, SAIL_TOP = {-16.21, -11.98};
	// The sail's foot curves into the hull with a fillet of radius SAIL_FILLET_R, whose arc would be tangent to the hull
	// SAIL_FILLET_DIP below it, so it meets the hull at an angle (about 38 degrees) instead of running flat into it
	private static final double SAIL_FILLET_R = 1.2, SAIL_FILLET_DIP = 0.25;
	private static final int SAIL_CHORD_STEPS = 14; // outline points per side; the cockpit is built on them
	// Bridge cockpit: a well sunk WELL_DEPTH into the sail top, WELL_WALL in from the deck edge, from WELL_FROM to
	// WELL_TO of the deck's length (chord fractions); its walls are the windscreen. The hatch (coaming ring + lid,
	// 0.66 m clear opening) sits on its floor; the masts stand behind it.
	private static final double WELL_FROM = 0.03, WELL_TO = 0.5, WELL_WALL = 0.12, WELL_DEPTH = 0.5;
	private static final double HATCH_AT = 0.3, HATCH_R_IN = 0.33, HATCH_R_OUT = 0.40;

	// Hull fittings on the top centreline: escape hatches (station y, lid radius) and the weapons-loading hatch
	// (station y, half-width, half-length); retractable mooring bollards in pairs (station y, offset from the
	// centreline); the towed-array fairing along the starboard flank (from y, to y, radius, angle round the hull
	// from +X; model +X is to port, so 195 degrees is starboard, a little below the widest point)
	private static final double[][] ESCAPE_HATCHES = {{-26.0, 0.42}, {6.0, 0.42}};
	private static final double[] LOADING_HATCH = {-22.5, 0.45, 1.2};
	private static final double[][] BOLLARDS = {{-29.0, 0.9}, {11.0, 0.9}};
	private static final double[] TOWED_ARRAY = {-4.0, 26.0, 0.14, Math.toRadians(195)};

	// Torpedo tubes, in the frame the engine uses (TorpedoTubes): {right, up} in metres from the submarine's
	// position, right being starboard. Model +X is to port, so a tube sits at model x = -right, z = up. The tubes run
	// parallel to the hull's axis and open where they meet the hull; each muzzle has a shutter door (TubeDoor1..4, in
	// this order) that opens by turning about the hull's centreline. All tubes sit below the widest point, so turning
	// a door towards it slides it under the skin (the hull's section is wider than tall). The engine's table must match.
	private static final double[][] TUBES = {{-1.7, -0.6}, {1.7, -0.6}, {-1.25, -1.45}, {1.25, -1.45}};
	private static final double TUBE_R = 0.3, DOOR_DEPTH = 0.03, DOOR_OVERLAP = 0.08; // shutter: under the skin, overlapping the hole
	private static final double CUT_MARGIN = 0.05; // hull triangles this close to a tube's circle are cut away

	// Sensors: the flank arrays, panels on both sides (from y, to y) centred FLANK_ARRAY_THETA above the widest point, clear of the bow planes below
	// and the towed-array fairing. Each sits in a gunmetal frame SENSOR_FRAME wider all round.
	private static final double SENSOR_FRAME = 0.06;
	private static final double[][] FLANK_ARRAYS = {{-27.0, -19.5}, {-6.0, 3.0}, {8.0, 17.0}};
	private static final double FLANK_ARRAY_THETA = Math.toRadians(10), FLANK_ARRAY_HALF_HEIGHT = 0.4;
	// Team lights: one long light on each flank fore and aft (centred over the fore and aft flank arrays), TEAM_LIGHT_THETA above the
	// widest point (a little above the flank arrays); half-length and half-height of each light, and the width of
	// its frame
	private static final double[] TEAM_LIGHT_Y = {-23.25, 12.5};
	private static final double TEAM_LIGHT_THETA = Math.toRadians(23);
	private static final double[] TEAM_LIGHT_HALF = {0.9, 0.06};
	private static final double TEAM_LIGHT_FRAME = 0.025;
	// Decals, painted by the viewer: the code on both sides of the sail, on a patch {from y, to y} by {from z, to z}, and the
	// name along both upper flanks, on a patch {from y, to y} centred HULL_NAME_THETA above the widest point and
	// HULL_NAME_HALF_HEIGHT metres either side of it, both DECAL_LIFT off the surface. The viewer's textures have the
	// patches' proportions (3:1 and 16:1).
	private static final double[] SAIL_CODE_Y = {-16.0, -13.15}, SAIL_CODE_Z = {4.6, 5.55};
	private static final double[] HULL_NAME_Y = {-9.0, 7.0};
	private static final double HULL_NAME_THETA = Math.toRadians(45), HULL_NAME_HALF_HEIGHT = 0.5, DECAL_LIFT = 0.02;

	// Bow planes (starboard side; the port plane is mirrored): leading and trailing edge at the root and tip
	private static final double PLANE_ROOT_X = 3.6, PLANE_TIP_X = 5.88, PLANE_Z = -0.1, PLANE_CAP = 0.12;
	private static final double[] PLANE_ROOT = {-16.7, -9.36}, PLANE_TIP = {-13.37, -7.42};
	private static final double PLANE_THICKNESS = 0.05;

	// X-tail fins (all four alike): radius from the shaft and leading/trailing edge at root and tip. The fixed fin
	// runs to RUDDER_HINGE of the chord; the flap behind it swings about the hinge line. The trailing edge passes
	// ahead of the duct's leading edge wherever it is inside the duct's radius.
	private static final double RUDDER_ROOT_R = 0.25, RUDDER_TIP_R = 4.0, RUDDER_EDGE = 0.08, RUDDER_THICKNESS = 0.14;
	private static final double[] RUDDER_ROOT = {30.6, 35.1}, RUDDER_TIP = {33.8, 35.4};
	private static final double RUDDER_HINGE = 0.6, HINGE_GAP = 0.04;
	// Flap group names (port/starboard, upper/lower) and the angle each fin stands out at round the hull's axis (from
	// +X, port, towards +Z, up). The viewer drives each flap with a mix of the rudder and stern-plane commands.
	private static final Map<String, Double> X_TAIL = new LinkedHashMap<>();

	static {
		X_TAIL.put("tailflap_pu", Math.PI / 4);
		X_TAIL.put("tailflap_su", 3 * Math.PI / 4);
		X_TAIL.put("tailflap_sl", -3 * Math.PI / 4);
		X_TAIL.put("tailflap_pl", -Math.PI / 4);
	}

	// Duct: aerofoil chord along Y, mean radius narrowing towards the exit (a mild nozzle)
	private static final double DUCT_Y0 = 35.4, DUCT_Y1 = 38.3, DUCT_R_LE = 2.12, DUCT_R_TE = 2.0;
	private static final double DUCT_THICKNESS = 0.133; // fraction of the chord (NACA 4-digit)
	// Stator vanes just inside the duct entry and ahead of the (rotating) hub, rotated off the rudder
	// planes (90 and 270 degrees): aerofoil sections STATOR_THICKNESS of the chord, turned STATOR_TWIST
	// against the rotor swirl
	private static final int STATORS = 9;
	private static final double STATOR_Y0 = 35.8, STATOR_Y1 = 36.5, STATOR_ANGLE0 = 20;
	private static final double STATOR_THICKNESS = 0.12, STATOR_TWIST = Math.toRadians(8);
	// Rotor
	private static final int BLADES = 7;
	private static final double ROTOR_Y = 37.55, BLADE_ROOT_R = 0.72, BLADE_TIP_R = 1.86, PITCH = 2.6;
	private static final double SKEW = 0.55, RAKE = 0.12; // tip skew (radians) and aft rake (metres)
	// Hub profile (y, radius), front to back: it starts where the hull ends, 5 mm wider so the seam is
	// hidden, and tapers from the last point to a tip at HUB_TIP_Y as r = R (1 - s^HUB_TAPER)
	private static final double[][] HUB = {{TAIL_JOIN_Y, TAIL_JOIN_R + 0.005}, {37.85, TAIL_JOIN_R + 0.005}};
	// The tip is rounded off with a sphere where the cone has narrowed to HUB_TIP_ROUND
	private static final double HUB_TIP_Y = 39.0, HUB_TAPER = 1.8, HUB_TIP_ROUND = 0.1;
	// Satin metal rings about the shaft: {station y, radius of the ring's centre line, half-width along y,
	// half-thickness radially}. Round about the shaft, so the one on the spinning hub can stay in a static group.
	private static final double[][] ACCENT_RINGS = {{TAIL_JOIN_Y, TAIL_JOIN_R + 0.008, 0.06, 0.025}, // hull/spinner seam
			{HUB[1][0] - 0.06, HUB[1][1] + 0.003, 0.05, 0.02}}; // just ahead of where the spinner's cone starts

	/**
	 * A material: grey levels, shininess, and optionally a diffuse texture, a normal map and a
	 * specular map.
	 */
	private record Mat(double kd, double ka, double ks, double ns, String comment, String map, String bump,
			String specular) {
		Mat(double kd, double ka, double ks, double ns, String comment) {
			this(kd, ka, ks, ns, comment, null, null, null);
		}
	}

	// Anechoic tiles: TILE metres square, TILES_PER_TEXTURE of them along each side of one repeat of the texture
	private static final double TILE = 0.5;
	// Untiled parts of the hull: a cap over the bow sonar dome out to where the hull's half-width reaches NOSE_CAP_R
	// (the tiles would bunch up towards the nose), and a strip along the keel KEEL_STRIP either side of it (where
	// the tiles' rows meet round the hull, and where the boat sits on docking blocks)
	private static final double NOSE_CAP_R = 1.0, KEEL_STRIP = Math.toRadians(7.5);
	private static final int TILES_PER_TEXTURE = 16, TILE_PX = 64;
	private static final String TILES_MAP = "submarine-tiles.png", TILES_NORMALS = "submarine-tiles-normal.png";
	private static final String TILES_SHEEN = "submarine-tiles-spec.png";
	// Stealth coating on the fins: one repeat of its textures covers COATING_REPEAT metres, at COATING_PX pixels
	private static final double COATING_REPEAT = 4, COATING_UNROLL_R = 2.0; // surfaces round the axis unroll at this radius
	private static final int COATING_PX = 1024;
	private static final String COATING_MAP = "submarine-coating.png", COATING_NORMALS = "submarine-coating-normal.png",
			COATING_SHEEN = "submarine-coating-spec.png";
	// Weathering light map, spread over the hull by the viewer (SubmarineModelSupport, which has the same extents):
	// metres of hull along the texture's width and round the hull along its height
	private static final String WEATHERING = "submarine-weathering.png";
	private static final double WEATHERING_ALONG = 80, WEATHERING_AROUND = 40;
	// The sail's texture coordinates are shifted by whole repeats of the tile texture (so the tiles do not move) to keep
	// it clear of the hull's part of the weathering map: its sides (height along the map) beyond the hull's tail, past
	// 72.8 m, and its top (station along the map) over the hull's nose, where no contact shadow falls
	private static final double SAIL_SIDES_SHIFT = 8, SAIL_TOP_SHIFT = 16;

	private static final Map<String, Mat> MATERIALS = new LinkedHashMap<>();

	static {
		MATERIALS.put("Hull_Tiles", new Mat(0.11, 0.10, 0.55, 30,
				"Hull and sail: near-black anechoic tiles; the normal map breaks the highlights up at the seams, the "
						+ "specular map varies the sheen from tile to tile, and the viewer adds large-scale weathering",
				TILES_MAP, TILES_NORMALS, TILES_SHEEN));
		MATERIALS.put("Hull_Plain", new Mat(0.083, 0.075, 0.4, 30,
				"Untiled hull (bow sonar dome, keel strip): the tiles' average grey, without the grid"));
		MATERIALS.put("Metal_Black_Plain",
				new Mat(0.07, 0.07, 0.4, 30, "Fittings: near-black satin, with enough specular to show the shape"));
		MATERIALS.put("Metal_Chrome", new Mat(0.6, 0.4, 0.9, 60, "Light polished metal for accents"));
		MATERIALS.put("Metal_Satin", new Mat(0.25, 0.18, 0.7, 60,
				"Rings round the shaft at the tail: satin metal, darker than the chrome so they do not draw the eye"));
		MATERIALS.put("Metal_Gunmetal", new Mat(0.09, 0.06, 0.9, 80,
				"Hatches and frames: dark polished gunmetal, mostly highlight and little diffuse grey"));
		MATERIALS.put("rubber", new Mat(0.13, 0.1, 0.15, 10, "Rotor: dark matte grey"));
		MATERIALS.put("Stealth_Coating", new Mat(0.05, 0.045, 0.3, 8,
				"Fins, tail flaps, bow planes, duct and stators: near-black absorbent coating laid in panels, with a broad, soft "
						+ "satin sheen",
				COATING_MAP, COATING_NORMALS, COATING_SHEEN));
		MATERIALS.put("Sensor_Window",
				new Mat(0.03, 0.03, 0.7, 90, "Flank arrays: glossy black, so they read as a different surface"));
		MATERIALS.put("glow_team", new Mat(0.6, 0.6, 0.0, 1,
				"Team lights: the viewer replaces this with a glowing material in the submarine's team colour"));
		MATERIALS.put("Decal_Code", new Mat(0.3, 0.25, 0.05, 4,
				"Sail code (two letters and a number): the viewer paints the submarine's own code over this"));
		MATERIALS.put("Decal_Name", new Mat(0.3, 0.25, 0.05, 4,
				"Name along the upper flanks: the viewer paints the submarine's own name over this"));
		MATERIALS.put("Void", new Mat(0.01, 0.0, 0.0, 1, "Inside of the torpedo tubes: no light comes back"));
	}

	private final List<double[]> verts = new ArrayList<>();
	private final List<double[]> normals = new ArrayList<>();
	private final LinkedHashMap<String, Group> groups = new LinkedHashMap<>();
	// fin() leaves a flat top open when asked (the sail closes its own around the cockpit) and records the top
	// outline for that
	private boolean leaveTopOpen;
	// fin() spaces its levels evenly along the span unless given this map from even spacing (0 to 1) to the levels' own
	private java.util.function.DoubleUnaryOperator spanLevels;
	private int[] lastTopOutline;
	// ...and all its section loops, root first
	private int[][] lastLoops;
	// Where each vertex on the sail's sides is round its section: arc length from the leading edge along the port and the
	// starboard side, and the station of the leading edge at its level
	private final Map<Integer, double[]> sailArc = new HashMap<>();
	// Texture coordinates of the decal patches' vertices, 0 to 1 across each patch in the direction the text reads
	private final Map<Integer, double[]> decalUv = new HashMap<>();
	// Vertices of the rings round the torpedo tube openings (open edges of the hull once cut)

	/** A face corner: 1-based vertex and normal indices (0 = not assigned yet). */
	private record Corner(int v, int n) {
	}

	private static final class Group {
		final String name;
		final String material;
		final List<Corner[]> faces = new ArrayList<>();
		/** How the group's texture coordinates are made (none for untextured materials). */
		UvMap uv = UvMap.NONE;
		/** Vertices that take the hull's own normal instead of one averaged from the faces. */
		final java.util.Set<Integer> onHull = new java.util.HashSet<>();

		Group(String name, String material) {
			this.name = name;
			this.material = material;
		}
	}

	/**
	 * Texture coordinates, in repeats of the tile texture: none; the hull unrolled (arc length
	 * round its section, and along its length), so that every tile is the same size; the sail
	 * unrolled likewise (arc length round its section from the leading edge, and height), with its
	 * flat top seen from above; projected along whichever axis a face most nearly faces; or, for
	 * the fins, which all stand out from the hull's axis, position along the hull and distance from
	 * the axis; or, for the decals, given with each vertex (0 to 1 across the patch).
	 */
	private enum UvMap {
		NONE, HULL, SAIL, BOX, RADIAL, DECAL
	}

	public static void main(String[] args) throws IOException {
		if (args.length != 1) {
			System.err.println("Usage: SubmarineModelGenerator <out-dir>");
			System.exit(2);
		}
		Path out = Path.of(args[0]);
		Files.createDirectories(out);
		var gen = new SubmarineModelGenerator();
		gen.buildHull(gen.group("Body", "Hull_Tiles"));
		Group mount = gen.group("PropellerMount", "Stealth_Coating");
		gen.buildDuct(mount);
		gen.buildStators(mount);
		gen.buildAccents(gen.group("TailRings", "Metal_Satin"));
		gen.buildSail(gen.group("Tower", "Hull_Tiles"));
		gen.buildSailFittings(gen.group("SailFittings", "Metal_Black_Plain"), gen.group("Accents", "Metal_Chrome"),
				gen.group("BridgeHatch", "Metal_Gunmetal"));
		gen.buildHullFittings(gen.group("HullFittings", "Metal_Black_Plain"), gen.group("Hatches", "Metal_Gunmetal"));
		gen.buildTorpedoTubes(gen.group("Body", "Hull_Tiles"), gen.group("TubeBores", "Void"));
		gen.splitUntiled(gen.group("Body", "Hull_Tiles"), gen.group("HullPlain", "Hull_Plain"));
		gen.buildSensors(gen.group("Sensors", "Sensor_Window"), gen.group("SensorFrames", "Metal_Gunmetal"));
		gen.buildTeamLights(gen.group("TeamLights", "glow_team"), gen.group("SensorFrames", "Metal_Gunmetal"));
		gen.buildSailCode(gen.group("SailCode", "Decal_Code"));
		gen.buildHullName(gen.group("HullName", "Decal_Name"));
		Group fins = gen.group("Fins", "Stealth_Coating");
		gen.buildPlane(gen.group("elevatorr", "Stealth_Coating"), -1);
		gen.buildPlane(gen.group("elevatorl", "Stealth_Coating"), 1);
		Group rotor = gen.group("Propeller", "rubber"); // dark matte, for a stealthier look
		gen.buildRotor(rotor);
		// The spinner is round about the shaft, so it can stay in a static group, in the coating like the duct
		gen.buildHub(gen.group("Spinner", "Stealth_Coating"));
		Map<String, double[][]> hinges = new LinkedHashMap<>();
		for (var fin : X_TAIL.entrySet())
			hinges.put(fin.getKey(),
					gen.buildTailFin(fins, gen.group(fin.getKey(), "Stealth_Coating"), fin.getValue()));
		gen.groups.get("Body").uv = UvMap.HULL;
		gen.groups.get("Tower").uv = UvMap.SAIL;
		for (Group g : gen.groups.values())
			if (g.material.equals("Stealth_Coating"))
				g.uv = UvMap.RADIAL;
		for (Group g : gen.groups.values())
			gen.creaseNormals(g);
		ImageIO.write(tileTexture(), "png", out.resolve(TILES_MAP).toFile());
		ImageIO.write(tileNormals(), "png", out.resolve(TILES_NORMALS).toFile());
		ImageIO.write(tileSheen(), "png", out.resolve(TILES_SHEEN).toFile());
		ImageIO.write(coatingTexture(), "png", out.resolve(COATING_MAP).toFile());
		ImageIO.write(coatingNormals(), "png", out.resolve(COATING_NORMALS).toFile());
		ImageIO.write(coatingSheen(), "png", out.resolve(COATING_SHEEN).toFile());
		ImageIO.write(weatheringTexture(), "png", out.resolve(WEATHERING).toFile());
		gen.write(out.resolve("submarine-hybrid.obj"), out.resolve("submarine-hybrid.mtl"));
		hinges.forEach(SubmarineModelGenerator::printHinge);
	}

	/**
	 * Prints a tail flap's hinge for SubmarineScene3D: a point on the hinge line and the line's
	 * direction from root to tip.
	 */
	private static void printHinge(String name, double[][] line) {
		double[] axis = unit(sub(line[1], line[0]));
		System.out.printf(Locale.ROOT, "%s hinge: point (%.3f, %.3f, %.3f), axis (%.4f, %.4f, %.4f)%n", name,
				line[0][0], line[0][1], line[0][2], axis[0], axis[1], axis[2]);
	}

	private Group group(String name, String material) {
		return groups.computeIfAbsent(name, n -> new Group(n, material));
	}

	// ── Output ───────────────────────────────────────────────────────────────

	private void write(Path objPath, Path mtlPath) throws IOException {
		try (PrintWriter m = new PrintWriter(Files.newBufferedWriter(mtlPath, StandardCharsets.US_ASCII))) {
			m.print("# Submarine materials. Generated by SubmarineModelGenerator.\n#\n");
			for (var e : MATERIALS.entrySet()) {
				Mat c = e.getValue();
				m.printf(Locale.ROOT, "# %s\nnewmtl %s\nKa  %.2f %.2f %.2f\nKd  %.2f %.2f %.2f\nKs  %.2f %.2f %.2f\n",
						c.comment(), e.getKey(), c.ka(), c.ka(), c.ka(), c.kd(), c.kd(), c.kd(), c.ks(), c.ks(),
						c.ks());
				m.printf(Locale.ROOT, "d  1.0\nNs  %.1f\nillum 2\n", c.ns());
				if (c.map() != null)
					m.print("map_Kd " + c.map() + "\n");
				if (c.bump() != null)
					m.print("map_Bump " + c.bump() + "\n");
				if (c.specular() != null)
					m.print("map_Ks " + c.specular() + "\n");
				m.print("#\n");
			}
			m.print("# EOF\n");
		}
		int[] vMap = new int[verts.size() + 1], nMap = new int[normals.size() + 1];
		StringBuilder v = new StringBuilder(), vn = new StringBuilder(), vt = new StringBuilder(),
				f = new StringBuilder();
		Map<String, Integer> vtIndex = new HashMap<>();
		int[] counts = new int[3];
		for (Group g : groups.values()) {
			f.append("g ").append(g.name).append('\n').append("usemtl ").append(g.material).append('\n');
			for (Corner[] face : g.faces) {
				counts[2]++;
				f.append('f');
				double[][] uvs = g.uv == UvMap.NONE ? null : faceUvs(g.uv, face);
				for (int k = 0; k < face.length; k++) {
					Corner c = face[k];
					if (vMap[c.v] == 0) {
						double[] p = verts.get(c.v - 1);
						v.append(String.format(Locale.ROOT, "v %.6f %.6f %.6f\n", p[0], p[1], p[2]));
						vMap[c.v] = ++counts[0];
					}
					if (nMap[c.n] == 0) {
						double[] p = normals.get(c.n - 1);
						vn.append(String.format(Locale.ROOT, "vn %.6f %.6f %.6f\n", p[0], p[1], p[2]));
						nMap[c.n] = ++counts[1];
					}
					f.append(' ').append(vMap[c.v]).append('/');
					if (uvs != null) {
						String key = String.format(Locale.ROOT, "%.5f %.5f", uvs[k][0], uvs[k][1]);
						Integer t = vtIndex.get(key);
						if (t == null) {
							t = vtIndex.size() + 1;
							vtIndex.put(key, t);
							vt.append("vt ").append(key).append('\n');
						}
						f.append(t);
					}
					f.append('/').append(nMap[c.n]);
				}
				f.append('\n');
			}
		}
		StringBuilder sb = new StringBuilder();
		sb.append(
				"# Submarine: 75 m attack submarine. Generated by se.hirt.searobots.viewer.tools.SubmarineModelGenerator.\n");
		sb.append("# Units: metres. X across, Y aft (bow at -Y), Z up.\n");
		sb.append("mtllib ").append(mtlPath.getFileName()).append("\n#\n");
		sb.append(v).append(vt).append(vn).append(f).append("# EOF\n");
		Files.writeString(objPath, sb, StandardCharsets.US_ASCII);
		System.out.printf(Locale.ROOT, "vertices=%d normals=%d uvs=%d faces=%d -> %s%n", counts[0], counts[1],
				vtIndex.size(), counts[2], objPath);
	}

	// ── Texture coordinates and the tile textures ────────────────────────────

	/** Texture coordinates of a face's corners, in repeats of the group's texture. */
	private double[][] faceUvs(UvMap map, Corner[] face) {
		double[][] uv = new double[face.length][];
		double[][] p = new double[face.length][];
		for (int k = 0; k < face.length; k++)
			p[k] = verts.get(face[k].v - 1);
		double[] normal = cross(sub(p[1], p[0]), sub(p[2], p[0]));
		if (map == UvMap.DECAL) {
			for (int k = 0; k < face.length; k++)
				uv[k] = decalUv.get(face[k].v).clone();
			return uv;
		}
		// Faces of the sail's sides (recorded by recordSailArcs, the fillet included however much it leans), or any other face of
		// it that stands more upright than flat
		boolean recorded = map == UvMap.SAIL;
		for (Corner corner : face)
			recorded &= sailArc.containsKey(corner.v);
		if (map == UvMap.SAIL && (recorded || Math.abs(normal[2]) < 0.7 * Math.sqrt(dot(normal, normal)))) {
			// Arc length round the section plus where the leading edge is at that height: on the flat sides that is
			// about the station along the hull, so the columns of tiles stand upright under the raked leading edge.
			// Each side runs its own way (the face's centre tells which), so the tiles are cut where the two sides
			// meet at the edges, as on a real sail.
			// The arc is measured along the sail as built (recordSailArcs), so it follows the fillet too.
			double side = Math.signum(p[0][0] + p[1][0] + p[2][0]);
			for (int k = 0; k < face.length; k++) {
				double[] arc = recorded ? sailArc.get(face[k].v) : null;
				double around = arc != null ? (side > 0 ? arc[0] : arc[1]) + arc[2] : aroundSail(p[k])
						+ edgeAt(SAIL_BASE, SAIL_TOP, SAIL_BASE_Z, SAIL_TOP_Z, Math.min(p[k][2], SAIL_TOP_Z))[0];
				uv[k] = new double[] {side * around, p[k][2] - SAIL_SIDES_SHIFT};
			}
		} else if (map == UvMap.SAIL) {
			for (int k = 0; k < face.length; k++)
				uv[k] = new double[] {p[k][0], p[k][1] + SAIL_TOP_SHIFT};
		} else if (map == UvMap.RADIAL) {
			double[] c = scale(add(add(p[0], p[1]), p[2]), 1.0 / 3);
			double[] out = unit(new double[] {c[0], 0, c[2] - AXIS_Z});
			if (Math.abs(dot(normal, out)) > 0.7 * Math.sqrt(dot(normal, normal))) {
				// Facing away from the axis (the duct, the fins' tips): unrolled round the axis instead
				double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
				for (int k = 0; k < face.length; k++) {
					double angle = Math.atan2(p[k][2] - AXIS_Z, p[k][0]);
					uv[k] = new double[] {p[k][1], angle};
					lo = Math.min(lo, angle);
					hi = Math.max(hi, angle);
				}
				for (int k = 0; k < face.length; k++) {
					if (hi - lo > Math.PI && uv[k][1] < 0)
						uv[k][1] += 2 * Math.PI;
					uv[k][1] *= COATING_UNROLL_R;
				}
			} else
				for (int k = 0; k < face.length; k++)
					uv[k] = new double[] {p[k][1], Math.hypot(p[k][0], p[k][2] - AXIS_Z)};
		} else if (map == UvMap.BOX) {
			double[] n = cross(sub(p[1], p[0]), sub(p[2], p[0]));
			double ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2]);
			for (int k = 0; k < face.length; k++)
				uv[k] = ax >= ay && ax >= az ? new double[] {p[k][1], p[k][2]}
						: az >= ay ? new double[] {p[k][0], p[k][1]} : new double[] {p[k][0], p[k][2]};
		} else {
			double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
			for (int k = 0; k < face.length; k++) {
				uv[k] = new double[] {aroundHull(p[k]), alongHull(p[k][1])};
				lo = Math.min(lo, uv[k][0]);
				hi = Math.max(hi, uv[k][0]);
			}
			// The unrolled hull's seam runs along the keel: a face across it takes its corners to the port side
			if (hi - lo > Math.PI)
				for (int k = 0; k < face.length; k++)
					if (uv[k][0] < 0)
						uv[k][0] += arcRound(p[k][1], 2 * Math.PI);
		}
		double repeat = map == UvMap.RADIAL ? COATING_REPEAT : TILE * TILES_PER_TEXTURE;
		for (double[] t : uv) {
			t[0] /= repeat;
			t[1] /= repeat;
		}
		return uv;
	}

	/**
	 * Arc length round the sail's section at the height of {@code p}, from the leading edge to
	 * {@code p}, on either side.
	 */
	private static double aroundSail(double[] p) {
		double[] edges = edgeAt(SAIL_BASE, SAIL_TOP, SAIL_BASE_Z, SAIL_TOP_Z, Math.min(p[2], SAIL_TOP_Z));
		double c = edges[1] - edges[0], to = Math.max(0, Math.min(1, (p[1] - edges[0]) / c));
		int n = 48;
		double arc = 0, prev = sailHalf(0, c);
		for (int i = 1; i <= n; i++) {
			double si = to * i / n, half = sailHalf(si, c);
			arc += Math.hypot(c * to / n, half - prev);
			prev = half;
		}
		return arc;
	}

	/**
	 * Arc length round the hull's section from the top of it to {@code p}, positive to port; the
	 * seam is at the keel.
	 */
	private static double aroundHull(double[] p) {
		double w = hullHalfWidth(p[1]), h = hullHalfHeight(p[1]);
		if (w < 1e-9)
			return 0;
		return arcRound(p[1], Math.atan2(p[0] / w, (p[2] - AXIS_Z) / h));
	}

	/**
	 * Arc length along the hull's elliptical section at station y, from the top round to the
	 * parametric angle {@code t} (x = w sin t, z = h cos t).
	 */
	private static double arcRound(double y, double t) {
		double w = hullHalfWidth(y), h = hullHalfHeight(y);
		int n = 64;
		double sum = 0;
		for (int i = 0; i < n; i++) {
			double tau = t * (i + 0.5) / n;
			sum += Math.hypot(w * Math.cos(tau), h * Math.sin(tau));
		}
		return sum * t / n;
	}

	private static double[] meridian;

	/**
	 * Distance along the hull's surface from the nose to station y (on the mean of its half-width
	 * and half-height), so tiles keep their size where the hull curves in at the ends.
	 */
	private static double alongHull(double y) {
		double step = 0.01;
		if (meridian == null) {
			int n = (int) Math.ceil((TAIL_JOIN_Y - NOSE_Y) / step) + 2;
			meridian = new double[n];
			for (int i = 1; i < n; i++) {
				double y0 = NOSE_Y + (i - 1) * step, y1 = y0 + step;
				double r0 = (hullHalfWidth(y0) + hullHalfHeight(y0)) / 2,
						r1 = (hullHalfWidth(y1) + hullHalfHeight(y1)) / 2;
				meridian[i] = meridian[i - 1] + Math.hypot(step, r1 - r0);
			}
		}
		double x = Math.max(0, Math.min(meridian.length - 1.001, (y - NOSE_Y) / step));
		int i = (int) x;
		return meridian[i] + (meridian[i + 1] - meridian[i]) * (x - i);
	}

	/**
	 * Height of the tile surface at each texel: grout between the tiles, a bevel, then a face
	 * tilted very slightly, differently for each tile.
	 */
	private static double[] tileHeights() {
		Random r = new Random(12);
		int size = TILES_PER_TEXTURE * TILE_PX;
		double[] tiltU = new double[TILES_PER_TEXTURE * TILES_PER_TEXTURE], tiltV = new double[tiltU.length];
		for (int i = 0; i < tiltU.length; i++) {
			tiltU[i] = (r.nextDouble() - 0.5) * 2.5;
			tiltV[i] = (r.nextDouble() - 0.5) * 2.5;
		}
		double[] height = new double[size * size];
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				int lx = x % TILE_PX, ly = y % TILE_PX, tile = (y / TILE_PX) * TILES_PER_TEXTURE + x / TILE_PX;
				double d = Math.min(Math.min(lx, ly), Math.min(TILE_PX - 1 - lx, TILE_PX - 1 - ly));
				double face = 3 + tiltU[tile] * (lx - TILE_PX / 2.0) / TILE_PX
						+ tiltV[tile] * (ly - TILE_PX / 2.0) / TILE_PX;
				height[y * size + x] = d < 1 ? 0 : face * smoothstep(Math.min(1, (d - 1) / 3));
			}
		return height;
	}

	/**
	 * The tiles' diffuse texture: a grey level per tile with a slight grain, a few replacement
	 * tiles from a different batch, and dark grout. The material's colour multiplies it.
	 */
	static BufferedImage tileTexture() {
		int size = TILES_PER_TEXTURE * TILE_PX;
		Random r = new Random(11);
		double[] tone = new double[TILES_PER_TEXTURE * TILES_PER_TEXTURE];
		for (int i = 0; i < tone.length; i++)
			tone[i] = r.nextDouble() < 0.04 ? 0.84 + 0.04 * r.nextDouble() : 0.92 + 0.08 * (r.nextDouble() - 0.5);
		double[] height = tileHeights();
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		Random grain = new Random(13);
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				double h = height[y * size + x];
				int tile = (y / TILE_PX) * TILES_PER_TEXTURE + x / TILE_PX;
				double lum = h == 0 ? 0.74
						: tone[tile] * (0.95 + 0.05 * Math.min(1, h / 3)) + 0.02 * (grain.nextDouble() - 0.5);
				int c = (int) Math.round(255 * Math.max(0, Math.min(1, lum)));
				img.setRGB(x, y, (c << 16) | (c << 8) | c);
			}
		return img;
	}

	/**
	 * The tiles' specular map: how much each tile shines. Tiles weather unevenly, so some are
	 * glossier than others and a few are worn matte; the grout is matte. The material's specular
	 * colour multiplies it.
	 */
	static BufferedImage tileSheen() {
		int size = TILES_PER_TEXTURE * TILE_PX;
		Random r = new Random(14);
		double[] sheen = new double[TILES_PER_TEXTURE * TILES_PER_TEXTURE];
		for (int i = 0; i < sheen.length; i++)
			sheen[i] = r.nextDouble() < 0.08 ? 0.5 + 0.1 * r.nextDouble() : 0.75 + 0.25 * r.nextDouble();
		double[] height = tileHeights();
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		float[] grain = valueNoise(size, size, 4, 15); // smooth, so the PNG stays small
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				double h = height[y * size + x];
				int tile = (y / TILE_PX) * TILES_PER_TEXTURE + x / TILE_PX;
				double v = h == 0 ? 0.15
						: sheen[tile] * (0.7 + 0.3 * Math.min(1, h / 3)) * (0.92 + 0.16 * grain[y * size + x]);
				int c = (int) Math.round(255 * Math.max(0, Math.min(1, v)));
				img.setRGB(x, y, (c << 16) | (c << 8) | c);
			}
		return img;
	}

	/**
	 * Large-scale weathering over the whole hull, as a light map that darkens colour and sheen
	 * alike. The viewer spreads it over the hull with a second set of texture coordinates, so
	 * unlike the tiles it does not repeat: {@link #WEATHERING_ALONG} metres of hull along its
	 * width, {@link #WEATHERING_AROUND} metres round the hull along its height, with the top
	 * centreline across the middle. Broad blotches, finer mottling, a salt-faded deck, patches of
	 * newer tiles and faint streaks running down the sides. It averages about 0.82; the tiles'
	 * colours are raised to make up for that.
	 */
	static BufferedImage weatheringTexture() {
		int w = 1024, h = 512;
		double pxPerM = w / WEATHERING_ALONG;
		float[] broad = valueNoise(w, h, 80, 21), mid = valueNoise(w, h, 20, 22), fine = valueNoise(w, h, 4, 23);
		double[] v = new double[w * h];
		for (int y = 0; y < h; y++) {
			double around = (y - h / 2.0) / pxPerM; // metres round the hull from the top centreline
			double deck = Math.exp(-Math.pow(around / 3.0, 2));
			for (int x = 0; x < w; x++) {
				int i = y * w + x;
				v[i] = 0.82 + 0.10 * (broad[i] - 0.5) + 0.05 * (mid[i] - 0.5) + 0.02 * (fine[i] - 0.5) + 0.06 * deck;
			}
		}
		Random r = new Random(24);
		// Patches of newer, darker tiles, on the tile grid
		for (int p = 0; p < 40; p++) {
			double along0 = TILE * Math.floor((8 + 66 * r.nextDouble()) / TILE);
			double around0 = TILE * Math.floor((r.nextDouble() - 0.5) * 24 / TILE);
			double along1 = along0 + TILE * (2 + r.nextInt(5)), around1 = around0 + TILE * (1 + r.nextInt(4));
			double dark = 0.9 + 0.05 * r.nextDouble();
			for (int y = (int) Math.round(h / 2.0 + around0 * pxPerM); y < Math.round(h / 2.0 + around1 * pxPerM); y++)
				for (int x = (int) Math.round(along0 * pxPerM); x < Math.round(along1 * pxPerM); x++)
					if (x >= 0 && x < w && y >= 0 && y < h)
						v[y * w + x] *= dark;
		}
		// Streaks running down the sides from below the deck (aft of the sail's part of the map, where they would
		// run sideways)
		for (int s = 0; s < 90; s++) {
			double along = 8 + 66 * r.nextDouble();
			int side = r.nextBoolean() ? 1 : -1;
			double start = 2 + 3 * r.nextDouble(), length = 1 + 4 * r.nextDouble();
			int width = 1 + r.nextInt(3);
			double amount = (r.nextDouble() < 0.6 ? -0.07 : 0.05) * (0.5 + 0.5 * r.nextDouble());
			int x0 = (int) Math.round(along * pxPerM);
			for (int t = 0; t < length * pxPerM; t++) {
				int y = (int) Math.round(h / 2.0 + side * (start * pxPerM + t));
				double fade = 1 - t / (length * pxPerM);
				for (int dx = 0; dx < width; dx++)
					if (y >= 0 && y < h && x0 + dx < w)
						v[y * w + x0 + dx] += amount * fade;
			}
		}
		contactShadows(v, w, h, pxPerM);
		var img = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int c = (int) Math.round(255 * Math.max(0, Math.min(1, v[y * w + x])));
				img.setRGB(x, y, (c << 16) | (c << 8) | c);
			}
		return img;
	}

	/**
	 * Darkens the weathering map where parts meet the hull, as ambient occlusion would: round the
	 * foot of the sail on the hull, up the foot of the sail itself, and along the roots of the bow
	 * planes and the tail fins. Each texel of the hull's part of the map is taken back to its point
	 * on the hull (station from the length along the hull, position round the section from the arc
	 * length); the sail's band past the hull's tail is darkened by height above the hull. The hull
	 * and all these parts are mirror images port and starboard, so the map's sign round the hull
	 * does not matter.
	 */
	private static void contactShadows(double[] v, int w, int h, double pxPerM) {
		double hullLength = alongHull(TAIL_JOIN_Y);
		int samples = 256;
		for (int col = 0; col < w; col++) {
			double along = (col + 0.5) / pxPerM;
			if (along > hullLength + 1) {
				// The sail's sides: height along the map, shifted by SAIL_SIDES_SHIFT and wrapped round
				double z = along - WEATHERING_ALONG + SAIL_SIDES_SHIFT;
				double shade = 1 - 0.3 * Math.exp(-Math.max(0, z - hullTop(-14)) / 0.55);
				for (int row = 0; row < h; row++)
					v[row * w + col] *= shade;
				continue;
			}
			// Station: invert the length along the hull
			double lo = NOSE_Y, hi = TAIL_JOIN_Y;
			for (int i = 0; i < 40; i++) {
				double mid = (lo + hi) / 2;
				if (alongHull(mid) < along)
					lo = mid;
				else
					hi = mid;
			}
			double y = (lo + hi) / 2, hw = hullHalfWidth(y), hh = hullHalfHeight(y);
			// Arc length round the section from the top, sampled to invert it
			double[] arc = new double[samples + 1];
			for (int i = 0; i <= samples; i++)
				arc[i] = arcRound(y, Math.PI * i / samples);
			for (int row = 0; row < h; row++) {
				double around = Math.abs((row + 0.5) / pxPerM - h / 2.0 / pxPerM);
				if (around > arc[samples])
					continue;
				int i = 0;
				while (i < samples - 1 && arc[i + 1] < around)
					i++;
				double t = Math.PI * (i + (around - arc[i]) / Math.max(1e-9, arc[i + 1] - arc[i])) / samples;
				double x = hw * Math.sin(t), z = AXIS_Z + hh * Math.cos(t);
				v[row * w + col] *= (z > AXIS_Z ? sailShadow(x, y) : 1) * planeShadow(x, y, z) * finShadow(x, y, z);
			}
		}
	}

	/**
	 * Shade on the hull round the foot of the sail (with its fillet), at (x, y) on the hull top.
	 */
	private static double sailShadow(double x, double y) {
		double le = SAIL_BASE[0], te = SAIL_BASE[1], c = te - le, flare = filletOut(0);
		if (y < le - flare - 3 || y > te + 3 || x > SAIL_HALF_WIDTH + flare + 3)
			return 1;
		// Distance to the foot's outline, from points along it
		double d = Double.MAX_VALUE;
		boolean inside = false;
		for (int i = 0; i <= 200; i++) {
			double s = i / 200.0;
			double out = sailHalf(s, c) + flare * (1 - smoothstep(Math.max(0, Math.min(1, (s - 0.65) / 0.35))));
			double ys = le + s * c - (i == 0 ? flare : 0);
			d = Math.min(d, Math.hypot(x - out, y - ys));
			if (Math.abs(y - (le + s * c)) < c / 400 && x < out)
				inside = true;
		}
		return inside ? 1 - 0.3 : 1 - 0.3 * Math.exp(-d / 0.7);
	}

	/** Shade on the hull along the roots of the bow planes. */
	private static double planeShadow(double x, double y, double z) {
		if (Math.abs(z - PLANE_Z) > 2 || x < 2)
			return 1;
		double f = (x - PLANE_ROOT_X) / (PLANE_TIP_X - PLANE_ROOT_X);
		double le = lerp(PLANE_ROOT[0], PLANE_TIP[0], f), te = lerp(PLANE_ROOT[1], PLANE_TIP[1], f), c = te - le;
		double s = Math.max(0, Math.min(1, (y - le) / c));
		double across = Math.max(0, Math.abs(z - PLANE_Z) - nacaHalf(s, PLANE_THICKNESS) * c);
		double d = Math.hypot(across, Math.max(0, Math.max(le - y, y - te)));
		return 1 - 0.28 * Math.exp(-d / 0.45);
	}

	/** Shade on the hull along the roots of the X-tail's fins (at 45 degrees above and below). */
	private static double finShadow(double x, double y, double z) {
		if (y < RUDDER_ROOT[0] - 3)
			return 1;
		double r = Math.hypot(x, z - AXIS_Z), phi = Math.atan2(z - AXIS_Z, x);
		double f = (r - RUDDER_ROOT_R) / (RUDDER_TIP_R - RUDDER_ROOT_R);
		double le = lerp(RUDDER_ROOT[0], RUDDER_TIP[0], f), te = lerp(RUDDER_ROOT[1], RUDDER_TIP[1], f), c = te - le;
		double s = Math.max(0, Math.min(1, (y - le) / c));
		double across = Math.max(0, r * (Math.abs(Math.abs(phi) - Math.PI / 4)) - nacaHalf(s, RUDDER_THICKNESS) * c);
		double d = Math.hypot(across, Math.max(0, Math.max(le - y, y - te)));
		return 1 - 0.28 * Math.exp(-d / 0.35);
	}

	// ── Stealth coating (fins, tail flaps, bow planes) ───────────────────────

	/**
	 * Height of the coating's surface at each texel: panels laid in staggered rows, with shallow
	 * grooves at the seams, and a fine orange-peel texture from spraying.
	 */
	private static double[] coatingHeights() {
		int size = COATING_PX, panelU = size / 2, panelV = size / 4; // 2 m by 1 m panels in a 4 m repeat
		float[] peel = valueNoise(size, size, 3, 31);
		double[] height = new double[size * size];
		for (int y = 0; y < size; y++) {
			int row = y / panelV, ly = y % panelV;
			for (int x = 0; x < size; x++) {
				int lx = (x + (row % 2) * panelU / 2) % panelU;
				double d = Math.min(Math.min(lx, panelU - 1 - lx), Math.min(ly, panelV - 1 - ly));
				double groove = d < 1 ? 0 : smoothstep(Math.min(1, (d - 1) / 2));
				height[y * size + x] = 0.8 * groove + 0.35 * peel[y * size + x];
			}
		}
		return height;
	}

	/**
	 * The coating's colour: slight mottling and grain, the seams a little darker. The material's
	 * colour multiplies it.
	 */
	static BufferedImage coatingTexture() {
		int size = COATING_PX;
		float[] mottle = valueNoise(size, size, 96, 32), grain = valueNoise(size, size, 4, 33);
		double[] height = coatingHeights();
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				int i = y * size + x;
				double seam = height[i] < 0.4 ? 0.93 : 1;
				double v = seam * (0.92 + 0.08 * (mottle[i] - 0.5) + 0.04 * (grain[i] - 0.5));
				int c = (int) Math.round(255 * Math.max(0, Math.min(1, v)));
				img.setRGB(x, y, (c << 16) | (c << 8) | c);
			}
		return img;
	}

	/** The coating's normal map (tangent space, green along +v), from its heights. */
	static BufferedImage coatingNormals() {
		return normalMap(coatingHeights(), COATING_PX, 0.5);
	}

	/**
	 * The coating's specular map: a soft sheen that varies a little across the panels, duller in
	 * the seams.
	 */
	static BufferedImage coatingSheen() {
		int size = COATING_PX;
		float[] patches = valueNoise(size, size, 64, 34);
		double[] height = coatingHeights();
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				int i = y * size + x;
				double v = height[i] < 0.4 ? 0.6 : 0.75 + 0.25 * patches[i];
				int c = (int) Math.round(255 * Math.max(0, Math.min(1, v)));
				img.setRGB(x, y, (c << 16) | (c << 8) | c);
			}
		return img;
	}

	/**
	 * A tangent-space normal map (green along +v) from a square, wrapping height field, the slopes
	 * scaled by {@code strength}.
	 */
	private static BufferedImage normalMap(double[] height, int size, double strength) {
		var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < size; y++)
			for (int x = 0; x < size; x++) {
				double dx = height[y * size + (x + 1) % size] - height[y * size + (x + size - 1) % size];
				double dy = height[((y + 1) % size) * size + x] - height[((y + size - 1) % size) * size + x];
				// Image rows run down while v runs up (jME flips images on load)
				double[] n = unit(new double[] {-dx * strength / 2, dy * strength / 2, 1});
				int red = (int) Math.round(255 * (n[0] * 0.5 + 0.5));
				int green = (int) Math.round(255 * (n[1] * 0.5 + 0.5));
				int blue = (int) Math.round(255 * (n[2] * 0.5 + 0.5));
				img.setRGB(x, y, (red << 16) | (green << 8) | blue);
			}
		return img;
	}

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

	/**
	 * The tiles' normal map (tangent space, green along +v): the bevels round each tile and the
	 * tilt of its face, so highlights break up into the tile grid under grazing light.
	 */
	static BufferedImage tileNormals() {
		return normalMap(tileHeights(), TILES_PER_TEXTURE * TILE_PX, 0.35);
	}

	// ── Hull ─────────────────────────────────────────────────────────────────

	/** Half-width of the hull at station {@code y}. */
	static double hullHalfWidth(double y) {
		if (y <= NOSE_Y)
			return 0;
		if (y < MID_Y0) {
			double u = 1 - (y - NOSE_Y) / (MID_Y0 - NOSE_Y);
			return HULL_HALF_WIDTH * Math.pow(1 - Math.pow(u, FORE_EXPONENT), 1 / FORE_EXPONENT);
		}
		if (y <= MID_Y1)
			return HULL_HALF_WIDTH;
		double s = Math.min(1, (y - MID_Y1) / (TAIL_JOIN_Y - MID_Y1));
		return TAIL_JOIN_R + (HULL_HALF_WIDTH - TAIL_JOIN_R) * Math.pow(1 - Math.pow(s, AFT_A), AFT_B);
	}

	/** Half-height of the hull at station {@code y}. */
	static double hullHalfHeight(double y) {
		double round = smoothstep(Math.max(0, Math.min(1, (y - ROUND_FROM_Y) / (ROUND_BY_Y - ROUND_FROM_Y))));
		return hullHalfWidth(y) * (HEIGHT_RATIO + (1 - HEIGHT_RATIO) * round);
	}

	/** Top of the hull at station {@code y} (on the centreline). */
	static double hullTop(double y) {
		return AXIS_Z + hullHalfHeight(y);
	}

	/**
	 * The hull: rings of elliptical sections, dense where the curvature is high, closed with a
	 * single vertex at the nose and with a short step inside the spinner at the tail.
	 */
	private void buildHull(Group g) {
		int around = 48;
		List<Double> stations = new ArrayList<>();
		int nose = 24;
		for (int k = 1; k <= nose; k++) // bunched up at the nose
			stations.add(NOSE_Y + (MID_Y0 - NOSE_Y) * (1 - Math.cos(Math.PI / 2 * k / nose)));
		for (double y = MID_Y0 + 2; y < MID_Y1; y += 2)
			stations.add(y);
		int aft = 36;
		for (int k = 0; k <= aft; k++)
			stations.add(MID_Y1 + (TAIL_JOIN_Y - MID_Y1) * k / aft);
		int[][] ring = new int[stations.size()][around];
		for (int k = 0; k < stations.size(); k++) {
			double y = stations.get(k), w = hullHalfWidth(y), h = hullHalfHeight(y);
			for (int j = 0; j < around; j++) {
				double a = 2 * Math.PI * j / around;
				ring[k][j] = vertex(new double[] {w * Math.cos(a), y, AXIS_Z + h * Math.sin(a)});
			}
		}
		int noseTip = vertex(new double[] {0, NOSE_Y, AXIS_Z});
		for (int j = 0; j < around; j++)
			triAboutAxis(g, noseTip, ring[0][j], ring[0][(j + 1) % around]);
		for (int k = 0; k + 1 < stations.size(); k++)
			for (int j = 0; j < around; j++) {
				int j1 = (j + 1) % around;
				triAboutAxis(g, ring[k][j], ring[k][j1], ring[k + 1][j1]);
				triAboutAxis(g, ring[k][j], ring[k + 1][j1], ring[k + 1][j]);
			}
		// A short step inside the hub, closed with a flat cap nobody sees
		int[] last = ring[stations.size() - 1], inner = new int[around];
		double yIn = TAIL_JOIN_Y + 0.15;
		for (int j = 0; j < around; j++)
			inner[j] = vertex(onAxis(TAIL_JOIN_R - 0.02, 2 * Math.PI * j / around, yIn));
		int centre = vertex(onAxis(0, 0, yIn));
		for (int j = 0; j < around; j++) {
			int j1 = (j + 1) % around;
			triAboutAxis(g, last[j], last[j1], inner[j1]);
			triAboutAxis(g, last[j], inner[j1], inner[j]);
			tri(g, inner[j], inner[j1], centre, onAxis(0, 0, yIn - 1));
		}
	}

	/**
	 * Moves the hull's untiled parts from {@code body} to {@code plain}: the cap over the bow sonar
	 * dome and the strip along the keel. Both follow the hull's rings and lines, so their edges are
	 * clean; the vertices the two groups share take the hull's own normal in both, so the shading
	 * runs on across the edge.
	 */
	private void splitUntiled(Group body, Group plain) {
		double capY = NOSE_Y;
		while (hullHalfWidth(capY) < NOSE_CAP_R)
			capY += 0.001;
		for (var it = body.faces.iterator(); it.hasNext();) {
			Corner[] f = it.next();
			double[] c = new double[3];
			for (Corner corner : f)
				c = add(c, scale(verts.get(corner.v - 1), 1.0 / f.length));
			double w = hullHalfWidth(c[1]), h = hullHalfHeight(c[1]);
			// Angle round the section as the rings are built: 0 to port, a quarter turn up; the keel is at -90 degrees
			double angle = Math.atan2((c[2] - AXIS_Z) / h, c[0] / w);
			if (c[1] < capY || Math.abs(angle + Math.PI / 2) < KEEL_STRIP) {
				plain.faces.add(f);
				it.remove();
			}
		}
		java.util.Set<Integer> inBody = new java.util.HashSet<>();
		for (Corner[] f : body.faces)
			for (Corner corner : f)
				inBody.add(corner.v);
		for (Corner[] f : plain.faces)
			for (Corner corner : f)
				if (inBody.contains(corner.v)) {
					body.onHull.add(corner.v);
					plain.onHull.add(corner.v);
				}
	}

	// ── Sail, planes and rudders ─────────────────────────────────────────────

	/**
	 * The sail: from below the hull top to SAIL_TOP_Z, leading edge raked back, with a flat top
	 * (around the bridge cockpit) behind a small rounded edge.
	 */
	private void buildSail(Group g) {
		double rootZ = SAIL_BASE_Z - 0.8; // buried in the hull
		double[] le = edgeAt(SAIL_BASE, SAIL_TOP, SAIL_BASE_Z, SAIL_TOP_Z, rootZ);
		leaveTopOpen = true; // the top is closed around the bridge cockpit instead
		// Levels close together where the fillet curves into the hull, further apart above it
		double[][] levels = {{0, rootZ}, {2.0 / 24, SAIL_BASE_Z - 0.17},
				{16.0 / 24, SAIL_BASE_Z + SAIL_FILLET_R + 0.13}, {1, SAIL_TOP_Z}};
		spanLevels = t -> {
			int i = 0;
			while (t > levels[i + 1][0])
				i++;
			double z = lerp(levels[i][1], levels[i + 1][1], (t - levels[i][0]) / (levels[i + 1][0] - levels[i][0]));
			return (z - rootZ) / (SAIL_TOP_Z - rootZ);
		};
		fin(g, new double[] {0, le[0], rootZ}, new double[] {0, le[1], rootZ},
				new double[] {0, SAIL_TOP[0], SAIL_TOP_Z}, new double[] {0, SAIL_TOP[1], SAIL_TOP_Z},
				new double[] {1, 0, 0}, new double[] {0, 0, 1}, SubmarineModelGenerator::sailHalf, SAIL_EDGE, true,
				Part.WHOLE, 0, 24, SAIL_CHORD_STEPS);
		spanLevels = null;
		int[][] loops = lastLoops;
		leaveTopOpen = false;
		buildCockpit(g, lastTopOutline, SAIL_CHORD_STEPS);
		filletSail(g);
		recordSailArcs(loops);
	}

	/**
	 * Records, for every vertex of the sail's section loops (as finally placed, fillet and all),
	 * the arc length from the leading edge round each side and the leading edge's station at its
	 * level, for the tiles' texture coordinates. Each loop starts at the leading edge and runs
	 * along the port side first.
	 */
	private void recordSailArcs(int[][] loops) {
		for (int[] loop : loops) {
			int m = loop.length;
			double[] cum = new double[m];
			for (int i = 1; i < m; i++)
				cum[i] = cum[i - 1] + distance(verts.get(loop[i - 1] - 1), verts.get(loop[i] - 1));
			double perimeter = cum[m - 1] + distance(verts.get(loop[m - 1] - 1), verts.get(loop[0] - 1));
			double le = verts.get(loop[0] - 1)[1];
			for (int i = 0; i < m; i++)
				sailArc.put(loop[i], new double[] {cum[i], i == 0 ? 0 : perimeter - cum[i], le});
		}
	}

	private static double distance(double[] a, double[] b) {
		return Math.sqrt(dot(sub(a, b), sub(a, b)));
	}

	/**
	 * Flares the foot of the sail into the hull with a fillet of radius {@link #SAIL_FILLET_R}:
	 * each point on the sail closer to the hull top than that moves out, level with itself and
	 * square to the sail's surface, so the sides and leading edge curve smoothly into the hull. The
	 * fillet fades out over the tapering trailing part, which is too thin for it.
	 */
	private void filletSail(Group g) {
		java.util.Set<Integer> done = new java.util.HashSet<>();
		for (Corner[] face : g.faces)
			for (Corner corner : face) {
				if (!done.add(corner.v))
					continue;
				double[] p = verts.get(corner.v - 1);
				if (p[2] - hullTop(p[1]) >= SAIL_FILLET_R)
					continue;
				double[] edges = edgeAt(SAIL_BASE, SAIL_TOP, SAIL_BASE_Z, SAIL_TOP_Z, p[2]);
				double c = edges[1] - edges[0], s = Math.max(0, Math.min(1, (p[1] - edges[0]) / c));
				double fade = 1 - smoothstep(Math.max(0, Math.min(1, (s - 0.65) / 0.35)));
				double nx, ny;
				if (Math.abs(p[0]) < 1e-9) { // on the leading or trailing edge
					nx = 0;
					ny = s < 0.5 ? -1 : 1;
				} else { // square to the section: the gradient of |x| - half(y)
					double ds = 0.002, slope = (sailHalf(Math.min(1, s + ds), c) - sailHalf(Math.max(0, s - ds), c))
							/ ((Math.min(1, s + ds) - Math.max(0, s - ds)) * c);
					nx = Math.signum(p[0]);
					ny = -slope;
				}
				double len = Math.hypot(nx, ny);
				nx /= len;
				ny /= len;
				// Height above the hull right under where the point ends up (the hull falls away to the sides), so
				// the fillet meets it flush; a few rounds settle it
				double out = 0;
				for (int i = 0; i < 4; i++)
					out = filletOut(p[2] - hullSurfaceZ(p[0] + out * nx, p[1] + out * ny)) * fade;
				p[0] += out * nx;
				p[1] += out * ny;
			}
	}

	/**
	 * How far the fillet moves a point at height {@code h} above the hull: a quarter circle, its
	 * bottom SAIL_FILLET_DIP below the hull.
	 */
	private static double filletOut(double height) {
		double r = SAIL_FILLET_R, h = height + SAIL_FILLET_DIP;
		return h <= 0 ? r : h >= r ? 0 : r - Math.sqrt(r * r - (r - h) * (r - h));
	}

	/**
	 * Leading and trailing edge at {@code z}, on the straight lines through the edges at z0 and z1.
	 */
	private static double[] edgeAt(double[] at0, double[] at1, double z0, double z1, double z) {
		double t = (z - z0) / (z1 - z0);
		return new double[] {at0[0] + (at1[0] - at0[0]) * t, at0[1] + (at1[1] - at0[1]) * t};
	}

	/** A bow plane on the starboard ({@code side} = 1) or port ({@code side} = -1) side. */
	private void buildPlane(Group g, int side) {
		fin(g, new double[] {side * PLANE_ROOT_X, PLANE_ROOT[0], PLANE_Z},
				new double[] {side * PLANE_ROOT_X, PLANE_ROOT[1], PLANE_Z},
				new double[] {side * PLANE_TIP_X, PLANE_TIP[0], PLANE_Z},
				new double[] {side * PLANE_TIP_X, PLANE_TIP[1], PLANE_Z}, new double[] {0, 0, 1},
				new double[] {side, 0, 0}, naca(PLANE_THICKNESS), PLANE_CAP, false, Part.WHOLE, 0, 6, 12);
	}

	/**
	 * A tail fin standing out from the hull's axis at {@code angle} round it (from +X, port,
	 * towards +Z, up): the fixed fin into {@code fixed}, the flap behind its hinge into
	 * {@code flap}. Returns the flap's hinge line, root first.
	 */
	private double[][] buildTailFin(Group fixed, Group flap, double angle) {
		double[] span = {Math.cos(angle), 0, Math.sin(angle)}, thick = {Math.sin(angle), 0, -Math.cos(angle)};
		double[] rootLE = add(new double[] {0, RUDDER_ROOT[0], AXIS_Z}, scale(span, RUDDER_ROOT_R));
		double[] rootTE = add(new double[] {0, RUDDER_ROOT[1], AXIS_Z}, scale(span, RUDDER_ROOT_R));
		double[] tipLE = add(new double[] {0, RUDDER_TIP[0], AXIS_Z}, scale(span, RUDDER_TIP_R));
		double[] tipTE = add(new double[] {0, RUDDER_TIP[1], AXIS_Z}, scale(span, RUDDER_TIP_R));
		fin(fixed, rootLE, rootTE, tipLE, tipTE, thick, span, naca(RUDDER_THICKNESS), RUDDER_EDGE, true, Part.FRONT,
				RUDDER_HINGE, 8, 10);
		return fin(flap, rootLE, rootTE, tipLE, tipTE, thick, span, naca(RUDDER_THICKNESS), RUDDER_EDGE, true,
				Part.FLAP, RUDDER_HINGE, 8, 8);
	}

	/**
	 * Which part of a fin section to build: all of it, the fixed part ahead of a hinge, or the
	 * hinged part.
	 */
	private enum Part {
		WHOLE, FRONT, FLAP
	}

	/**
	 * A fin's symmetric cross-section: the half-thickness at chord fraction {@code s} of a chord
	 * {@code c}.
	 */
	private interface Section {
		double half(double s, double c);
	}

	/** NACA 4-digit section, {@code thickness} of the chord. */
	private static Section naca(double thickness) {
		return (s, c) -> nacaHalf(s, thickness) * c;
	}

	/**
	 * The sail's section: a constant {@link #SAIL_HALF_WIDTH} whatever the chord (slab sides, like
	 * a real sail), with an elliptical front over the first 20% of the chord and a smooth taper
	 * over the last 40%.
	 */
	private static double sailHalf(double s, double c) {
		if (s < 0.2)
			return SAIL_HALF_WIDTH * Math.sqrt(Math.max(0, 1 - Math.pow((0.2 - s) / 0.2, 2)));
		if (s <= 0.6)
			return SAIL_HALF_WIDTH;
		return SAIL_HALF_WIDTH * (1 - smoothstep(Math.min(1, (s - 0.6) / 0.4)));
	}

	/**
	 * A tapered fin with a symmetric {@code section}: straight leading and trailing edges from the
	 * root chord to the tip chord, then a cap of height {@code cap} along {@code spanDir}. The cap
	 * either rounds the section off to a line or, with {@code flatTop}, rounds just the edge with
	 * radius {@code cap} and closes the top with a flat face. The root is left open; it is meant to
	 * be buried in the hull.
	 * <p>
	 * {@code part} cuts the section at chord fraction {@code hinge}: FRONT keeps the part ahead of
	 * it, ending in a flat face, and FLAP the part behind it, with a rounded leading edge centred
	 * on the hinge line so it can swing in place; a gap of {@link #HINGE_GAP} separates the two.
	 * Only flat-topped fins can be cut. Returns the hinge line as {root point, tip point} for FLAP,
	 * null otherwise.
	 */
	private double[][] fin(
		Group g, double[] rootLE, double[] rootTE, double[] tipLE, double[] tipTE, double[] thickDir, double[] spanDir,
		Section section, double cap, boolean flatTop, Part part, double hinge, int spanSteps, int chordSteps) {
		int capSteps = 6, levels = spanSteps + capSteps;
		int[][] loop = new int[levels + 1][];
		double[] tipChord = sub(tipTE, tipLE);
		double tipC = Math.sqrt(dot(tipChord, tipChord));
		if (flatTop) {
			// A rounded edge thicker than the section would turn the flat top inside out: keep it below half of
			// the part's greatest half-thickness at the tip
			double from = part == Part.FLAP ? flapNoseCentre(tipC, section, hinge) / tipC : 0;
			double to = part == Part.FRONT ? hinge : 1, thickest = 0;
			for (int i = 0; i <= 50; i++)
				thickest = Math.max(thickest, section.half(from + (to - from) * i / 50, tipC));
			cap = Math.min(cap, 0.45 * thickest);
		}
		double[][] hingeLine = new double[2][];
		int midUpper = 0;
		for (int k = 0; k <= levels; k++) {
			double[] le, te;
			double c, scale = 1, inset = 0;
			if (k <= spanSteps) {
				double u = spanLevels != null ? spanLevels.applyAsDouble((double) k / spanSteps)
						: (double) k / spanSteps;
				le = lerp(rootLE, tipLE, u);
				te = lerp(rootTE, tipTE, u);
				double[] chord = sub(te, le);
				c = Math.sqrt(dot(chord, chord));
			} else {
				double phi = Math.PI / 2 * (k - spanSteps) / capSteps;
				double lift = cap * Math.sin(phi);
				le = add(tipLE, scale(spanDir, lift));
				te = add(tipTE, scale(spanDir, lift));
				c = tipC;
				if (flatTop)
					inset = cap * (1 - Math.cos(phi)); // quarter-round edge: the outline moves in as it rises
				else
					scale = Math.cos(phi);
			}
			double[] chordDir = unit(sub(te, le));
			List<double[]> outline = new ArrayList<>();
			midUpper = sectionOutline(outline, part, c, section, hinge, scale, inset, chordSteps);
			boolean ridge = !flatTop && k == levels; // zero thickness: the two sides share one line of vertices
			Map<String, Integer> shared = new HashMap<>();
			int[] l = new int[outline.size()];
			for (int i = 0; i < outline.size(); i++) {
				double[] p = add(add(le, scale(chordDir, outline.get(i)[0])), scale(thickDir, outline.get(i)[1]));
				l[i] = ridge ? shared.computeIfAbsent(String.format(Locale.ROOT, "%.6f %.6f %.6f", p[0], p[1], p[2]),
						key -> vertex(p)) : vertex(p);
			}
			loop[k] = l;
			if (part == Part.FLAP && (k == 0 || k == spanSteps))
				hingeLine[k == 0 ? 0 : 1] = add(le, scale(chordDir, flapNoseCentre(c, section, hinge)));
		}
		lastTopOutline = loop[levels];
		lastLoops = loop;
		if (flatTop && !leaveTopOpen) {
			// Close the top: the outline is convex, so a fan from its centre covers it
			int[] top = loop[levels];
			double[] centre = new double[3];
			for (int v : top)
				centre = add(centre, scale(verts.get(v - 1), 1.0 / top.length));
			int hub = vertex(centre);
			double[] below = sub(centre, spanDir);
			for (int j = 0; j < top.length; j++)
				tri(g, hub, top[j], top[(j + 1) % top.length], below);
		}
		// Every section loop runs the same way round, so the winding follows from the grid order. Near the thin
		// edges and in the cap no inside point is reliable; the sign comes from a well-shaped face on the upper
		// side at mid-span and mid-chord, whose outward normal must point along thickDir.
		int m = loop[0].length, km = spanSteps / 2;
		double[] p0 = verts.get(loop[km][midUpper] - 1), p1 = verts.get(loop[km][midUpper + 1] - 1),
				p2 = verts.get(loop[km + 1][midUpper + 1] - 1);
		boolean forward = dot(cross(sub(p1, p0), sub(p2, p0)), thickDir) > 0;
		for (int k = 0; k < levels; k++)
			for (int j = 0; j < m; j++) {
				int a = loop[k][j], b = loop[k][(j + 1) % m], c = loop[k + 1][(j + 1) % m], d = loop[k + 1][j];
				if (forward)
					quad(g, a, b, c, d, null);
				else
					quad(g, d, c, b, a, null);
			}
		return part == Part.FLAP ? hingeLine : null;
	}

	/**
	 * Where along the chord the centre of a flap's rounded leading edge (and so its hinge) lies.
	 */
	private static double flapNoseCentre(double c, Section section, double hinge) {
		return hinge * c + HINGE_GAP / 2 + section.half(hinge, c);
	}

	/**
	 * Fills {@code out} with one section outline in metres ({x along the chord from the leading
	 * edge, y across it}), going round the upper side first. {@code inset} shrinks it for a rounded
	 * flat-top edge. Returns the index of a point in the middle of the upper side.
	 */
	private static int sectionOutline(
		List<double[]> out, Part part, double c, Section section, double hinge, double scale, double inset, int steps) {
		double x0 = 0, x1 = c; // extent of the aerofoil surfaces
		int mid;
		if (part == Part.FLAP) {
			// Rounded nose: a half circle centred on the hinge line, then the aerofoil from there to the TE
			double xc = flapNoseCentre(c, section, hinge);
			double r = Math.max(0.005, section.half(xc / c, c) * scale - inset); // meets the surface behind it
			int arc = 6;
			for (int i = 0; i <= arc; i++) {
				double phi = -Math.PI / 2 + Math.PI * i / arc;
				out.add(new double[] {xc - r * Math.cos(phi), r * Math.sin(phi)});
			}
			x0 = xc;
			mid = arc + steps / 2;
		} else {
			mid = steps / 2;
		}
		if (part == Part.FRONT)
			x1 = hinge * c - HINGE_GAP / 2;
		// Upper side from x0 to x1 (bunched up at both ends), then the lower side back
		boolean closedTE = part != Part.FRONT;
		int first = part == Part.FLAP ? 1 : 0; // the flap's nose arc already ends at x0
		for (int j = first; j <= steps; j++) {
			double x = x0 + (x1 - x0) * 0.5 * (1 - Math.cos(Math.PI * j / steps));
			boolean edge = (j == 0 && part != Part.FLAP) || (j == steps && closedTE);
			out.add(new double[] {insetX(x, x0, x1, inset, part),
					edge ? 0 : halfThickness(x / c, section, c, scale, inset)});
		}
		int last = part == Part.FRONT ? steps : steps - 1; // the FRONT part's flat end has both corners
		for (int j = last; j >= 1; j--) { // x0 itself is covered by the LE point or the nose arc
			double x = x0 + (x1 - x0) * 0.5 * (1 - Math.cos(Math.PI * j / steps));
			out.add(new double[] {insetX(x, x0, x1, inset, part), -halfThickness(x / c, section, c, scale, inset)});
		}
		return mid;
	}

	/**
	 * Pulls x in from the free ends of the surfaces by {@code inset} (the flap's nose keeps its own
	 * radius).
	 */
	private static double insetX(double x, double x0, double x1, double inset, Part part) {
		double lo = part == Part.FLAP ? x0 : x0 + inset, hi = x1 - inset;
		return lo + (x - x0) * (hi - lo) / (x1 - x0);
	}

	// ── Propulsor ────────────────────────────────────────────────────────────

	/**
	 * Point at radius {@code r}, angle {@code a} (0 = starboard, 90 degrees = up) and station
	 * {@code y}.
	 */
	private static double[] onAxis(double r, double a, double y) {
		return new double[] {r * Math.cos(a), y, AXIS_Z + r * Math.sin(a)};
	}

	/**
	 * Fin section half-thickness. Inside a flat top's rounded edge ({@code inset} > 0) it never
	 * quite reaches zero, so the two sides stay apart and the surface stays closed.
	 */
	private static double halfThickness(double s, Section section, double c, double scale, double inset) {
		double h = section.half(s, c) * scale;
		return inset > 0 ? Math.max(0.005, h - inset) : h;
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
		int chord = 16, around = 96;
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
		int span = 6, chord = 10;
		double c = STATOR_Y1 - STATOR_Y0, yc = (STATOR_Y0 + STATOR_Y1) / 2;
		double cos = Math.cos(STATOR_TWIST), sin = Math.sin(STATOR_TWIST);
		// Root buried in the hull, tip buried in the duct wall, so both ends stay open
		double r0 = 0.25, r1 = Math.max(ductInnerR(STATOR_Y0), ductInnerR(STATOR_Y1)) + 0.03;
		// Section outline {x along the chord from mid-chord, y across it}: upper side LE to TE, lower side back
		List<double[]> outline = new ArrayList<>();
		for (int j = 0; j <= chord; j++) {
			double s = 0.5 * (1 - Math.cos(Math.PI * j / chord));
			outline.add(new double[] {(s - 0.5) * c, j == 0 || j == chord ? 0 : nacaHalf(s, STATOR_THICKNESS) * c});
		}
		for (int j = chord - 1; j >= 1; j--) {
			double s = 0.5 * (1 - Math.cos(Math.PI * j / chord));
			outline.add(new double[] {(s - 0.5) * c, -nacaHalf(s, STATOR_THICKNESS) * c});
		}
		int m = outline.size();
		for (int k = 0; k < STATORS; k++) {
			double a = Math.toRadians(STATOR_ANGLE0) + 2 * Math.PI * k / STATORS;
			int[][] loop = new int[span + 1][m];
			for (int i = 0; i <= span; i++) {
				double r = r0 + (r1 - r0) * i / span;
				for (int j = 0; j < m; j++)
					loop[i][j] = vertex(statorPoint(a, r, yc, outline.get(j)[0], outline.get(j)[1], cos, sin));
			}
			for (int i = 0; i < span; i++)
				for (int j = 0; j < m; j++) {
					int j1 = (j + 1) % m;
					// Inside: the chord line at this face's position along the chord
					double x = (outline.get(j)[0] + outline.get(j1)[0]) / 2;
					double[] inside = statorPoint(a, r0 + (r1 - r0) * (i + 0.5) / span, yc, x, 0, cos, sin);
					quad(g, loop[i][j], loop[i][j1], loop[i + 1][j1], loop[i + 1][j], inside);
				}
		}
	}

	/**
	 * A point on the stator vane at angle {@code a} round the shaft and radius {@code r}: {@code x}
	 * along the chord from mid-chord (at station {@code yc}) and {@code t} across it, the chord
	 * turned by the twist (cosine and sine given) about the radial direction.
	 */
	private static double[] statorPoint(double a, double r, double yc, double x, double t, double cos, double sin) {
		double along = x * cos - t * sin, tang = x * sin + t * cos;
		return onAxis(r, a + tang / r, yc + along);
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
	 * Hub: a short cylinder that meets the hull at the front and tapers to a rounded tip behind the
	 * duct.
	 */
	private void buildHub(Group g) {
		int around = 96, coneSteps = 16, capSteps = 6;
		List<double[]> profile = new ArrayList<>(List.of(HUB));
		double[] last = HUB[HUB.length - 1];
		// The cone, rings bunching up towards the tip, until it is thin enough for the rounded tip
		for (int k = 1;; k++) {
			double s = Math.sin(Math.PI / 2 * k / coneSteps);
			double[] p = {last[0] + (HUB_TIP_Y - last[0]) * s, last[1] * (1 - Math.pow(s, HUB_TAPER))};
			if (p[1] < HUB_TIP_ROUND)
				break;
			profile.add(p);
		}
		// The rounded tip: a sphere centred on the axis, tangent to the cone at its last ring
		double[] end = profile.get(profile.size() - 1), before = profile.get(profile.size() - 2);
		double slope = (before[1] - end[1]) / (end[0] - before[0]); // how fast the radius falls, per metre
		double radius = end[1] * Math.sqrt(1 + slope * slope), centreY = end[0] - end[1] * slope;
		double angle0 = Math.atan2(1, slope); // from the axis, at the last ring
		for (int k = 1; k < capSteps; k++) {
			double a = angle0 * (1 - (double) k / capSteps);
			profile.add(new double[] {centreY + radius * Math.cos(a), radius * Math.sin(a)});
		}
		int[][] ring = new int[profile.size()][around];
		for (int j = 0; j < profile.size(); j++)
			for (int i = 0; i < around; i++)
				ring[j][i] = vertex(onAxis(profile.get(j)[1], 2 * Math.PI * i / around, profile.get(j)[0]));
		int tip = vertex(onAxis(0, 0, centreY + radius));
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

	// ── Hull fittings ────────────────────────────────────────────────────────

	/** Height of the hull surface above (x, y), on its upper half. */
	private static double hullSurfaceZ(double x, double y) {
		double w = hullHalfWidth(y);
		return AXIS_Z + hullHalfHeight(y) * Math.sqrt(Math.max(0, 1 - (x / w) * (x / w)));
	}

	/**
	 * Hatches, bollards and the towed-array fairing. Hatches are a gunmetal lid on a slightly
	 * larger black rim, both following the curve of the hull a few centimetres above it.
	 */
	private void buildHullFittings(Group black, Group metal) {
		for (double[] h : ESCAPE_HATCHES) {
			conformalPatch(black, circle(0, h[0], h[1] + 0.07, 40), 0.03);
			conformalPatch(metal, circle(0, h[0], h[1], 40), 0.06);
		}
		double[] lh = LOADING_HATCH;
		conformalPatch(black, roundedRect(0, lh[0], lh[1] + 0.07, lh[2] + 0.07, 0.25, 8), 0.03);
		conformalPatch(metal, roundedRect(0, lh[0], lh[1], lh[2], 0.2, 8), 0.055);
		for (double[] b : BOLLARDS)
			for (int side : new int[] {-1, 1})
				conformalPatch(metal, roundedRect(side * b[1], b[0], 0.12, 0.25, 0.08, 4), 0.04);
		towedArrayFairing(black);
	}

	private static List<double[]> circle(double cx, double cy, double r, int n) {
		List<double[]> out = new ArrayList<>();
		for (int i = 0; i < n; i++) {
			double a = 2 * Math.PI * i / n;
			out.add(new double[] {cx + r * Math.cos(a), cy + r * Math.sin(a)});
		}
		return out;
	}

	/**
	 * A rectangle with rounded corners, {x, y} points going round counter-clockwise seen from
	 * above.
	 */
	private static List<double[]> roundedRect(double cx, double cy, double hx, double hy, double r, int perCorner) {
		List<double[]> out = new ArrayList<>();
		double[][] corners = {{hx - r, hy - r}, {-(hx - r), hy - r}, {-(hx - r), -(hy - r)}, {hx - r, -(hy - r)}};
		for (int c = 0; c < 4; c++)
			for (int i = 0; i <= perCorner; i++) {
				double a = Math.PI / 2 * (c + (double) i / perCorner);
				out.add(new double[] {cx + corners[c][0] + r * Math.cos(a), cy + corners[c][1] + r * Math.sin(a)});
			}
		return out;
	}

	/**
	 * A slab on the upper hull with the given outline (convex, {x, y} points): its top follows the
	 * hull {@code raise} above it and its sides run 0.1 m into the hull, where they are left open.
	 */
	private void conformalPatch(Group g, List<double[]> outline, double raise) {
		int n = outline.size();
		int[] top = new int[n], bottom = new int[n];
		double cx = 0, cy = 0;
		for (int i = 0; i < n; i++) {
			double[] p = outline.get(i);
			top[i] = vertex(new double[] {p[0], p[1], hullSurfaceZ(p[0], p[1]) + raise});
			bottom[i] = vertex(new double[] {p[0], p[1], hullSurfaceZ(p[0], p[1]) - 0.1});
			cx += p[0] / n;
			cy += p[1] / n;
		}
		double[] inside = {cx, cy, hullSurfaceZ(cx, cy) - 0.05};
		int hub = vertex(new double[] {cx, cy, hullSurfaceZ(cx, cy) + raise});
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			tri(g, hub, top[i], top[j], inside);
			quad(g, top[i], top[j], bottom[j], bottom[i], inside);
		}
	}

	/**
	 * The towed-array fairing: a tube half sunk into the starboard flank, following the hull at a
	 * fixed angle below its widest point, tapering to points at both ends.
	 */
	private void towedArrayFairing(Group g) {
		double y0 = TOWED_ARRAY[0], y1 = TOWED_ARRAY[1], radius = TOWED_ARRAY[2], theta = TOWED_ARRAY[3];
		int along = 60, around = 12;
		double taper = 1.5;
		int[][] ring = new int[along + 1][around];
		int[] tips = new int[2];
		for (int k = 0; k <= along; k++) {
			double y = y0 + (y1 - y0) * k / along;
			double w = hullHalfWidth(y), h = hullHalfHeight(y);
			double[] surface = {w * Math.cos(theta), y, AXIS_Z + h * Math.sin(theta)};
			double[] normal = unit(new double[] {Math.cos(theta) / w, 0, Math.sin(theta) / h});
			double[] across = unit(cross(normal, new double[] {0, 1, 0}));
			double ends = Math.min(Math.min(y - y0, y1 - y) / taper, 1);
			double r = radius * Math.sqrt(Math.max(0, ends * (2 - ends))); // rounded taper to a point
			double[] centre = add(surface, scale(normal, 0.3 * radius)); // a little over half proud of the hull
			if (k == 0 || k == along) {
				tips[k == 0 ? 0 : 1] = vertex(centre);
				continue;
			}
			for (int j = 0; j < around; j++) {
				double phi = 2 * Math.PI * j / around;
				ring[k][j] = vertex(
						add(centre, add(scale(normal, r * Math.cos(phi)), scale(across, r * Math.sin(phi)))));
			}
		}
		for (int k = 1; k < along; k++) {
			double y = y0 + (y1 - y0) * k / along;
			double[] inside = {hullHalfWidth(y) * Math.cos(theta), y, AXIS_Z + hullHalfHeight(y) * Math.sin(theta)};
			for (int j = 0; j < around; j++) {
				int j1 = (j + 1) % around;
				if (k + 1 < along)
					quad(g, ring[k][j], ring[k][j1], ring[k + 1][j1], ring[k + 1][j], inside);
				else
					tri(g, ring[k][j], ring[k][j1], tips[1], inside);
				if (k == 1)
					tri(g, tips[0], ring[k][j1], ring[k][j], inside);
			}
		}
	}

	// ── Sensors ──────────────────────────────────────────────────────────────

	/**
	 * The flank arrays: long panels along both sides, a centimetre proud of the hull, each in a
	 * gunmetal frame a little larger and lower. The bow array sits behind the nose cap, unseen.
	 */
	private void buildSensors(Group windows, Group frames) {
		for (double[] panel : FLANK_ARRAYS)
			for (double theta : new double[] {FLANK_ARRAY_THETA, Math.PI - FLANK_ARRAY_THETA}) {
				hullStrip(frames, theta, panel[0] - SENSOR_FRAME, panel[1] + SENSOR_FRAME,
						FLANK_ARRAY_HALF_HEIGHT + SENSOR_FRAME, FLANK_ARRAY_HALF_HEIGHT + SENSOR_FRAME, 0.006, 24, 4);
				hullStrip(windows, theta, panel[0], panel[1], FLANK_ARRAY_HALF_HEIGHT, FLANK_ARRAY_HALF_HEIGHT, 0.012,
						24, 4);
			}
	}

	/**
	 * Point on the hull at angle {@code theta} round its section (from +X, towards +Z) at station
	 * y.
	 */
	private static double[] hullAt(double theta, double y) {
		return new double[] {hullHalfWidth(y) * Math.cos(theta), y, AXIS_Z + hullHalfHeight(y) * Math.sin(theta)};
	}

	/**
	 * A strip on the hull centred on the line at angle {@code theta} round it, from station y0 to
	 * y1, {@code half} wide either side of that line (measured round the hull), its ends rounded
	 * with radius {@code round} (up to {@code half}, which makes them semicircles). Its top is
	 * {@code raise} proud of the hull, its sides run 0.1 m into it. {@code along} and
	 * {@code across} set how finely it follows the hull.
	 */
	private void hullStrip(
		Group g, double theta, double y0, double y1, double half, double round, double raise, int along, int across) {
		int[][] top = new int[along + 1][across + 1], skirt = new int[along + 1][across + 1];
		double[][] inside = new double[along + 1][];
		for (int k = 0; k <= along; k++) {
			double y = y0 + (y1 - y0) * (1 - Math.cos(Math.PI * k / along)) / 2; // dense at the rounded ends
			double end = Math.max(0, round - Math.min(y - y0, y1 - y)) / round;
			double h = half - round + round * Math.sqrt(Math.max(0, 1 - end * end));
			// Angle per metre round the hull here
			double w = hullHalfWidth(y), hh = hullHalfHeight(y);
			double perMetre = 1 / Math.hypot(w * Math.sin(theta), hh * Math.cos(theta));
			double[] centre = hullAt(theta, y);
			inside[k] = add(centre, scale(hullNormal(centre), -0.05));
			for (int j = 0; j <= across; j++) {
				double[] q = hullAt(theta + (2.0 * j / across - 1) * h * perMetre, y);
				double[] n = hullNormal(q);
				top[k][j] = vertex(add(q, scale(n, raise)));
				if (j == 0 || j == across || k == 0 || k == along)
					skirt[k][j] = vertex(add(q, scale(n, -0.1)));
			}
		}
		for (int k = 0; k < along; k++) {
			for (int j = 0; j < across; j++)
				quad(g, top[k][j], top[k][j + 1], top[k + 1][j + 1], top[k + 1][j], inside[k]);
			quad(g, top[k][0], top[k + 1][0], skirt[k + 1][0], skirt[k][0], inside[k]);
			quad(g, top[k][across], top[k + 1][across], skirt[k + 1][across], skirt[k][across], inside[k]);
		}
		// Square-ish ends have width: close them too (rounded ones end in a point, and these faces drop out)
		for (int k : new int[] {0, along})
			for (int j = 0; j < across; j++)
				quad(g, top[k][j], top[k][j + 1], skirt[k][j + 1], skirt[k][j], inside[k == 0 ? 1 : along - 1]);
	}

	/**
	 * The team lights: a long, narrow rectangular light on each flank fore and aft, above the flank
	 * arrays, flush with the hull, each in a gunmetal frame. The viewer lights them in the
	 * submarine's team colour.
	 */
	private void buildTeamLights(Group lights, Group frames) {
		for (double y : TEAM_LIGHT_Y)
			for (double theta : new double[] {TEAM_LIGHT_THETA, Math.PI - TEAM_LIGHT_THETA}) {
				hullStrip(frames, theta, y - TEAM_LIGHT_HALF[0] - TEAM_LIGHT_FRAME,
						y + TEAM_LIGHT_HALF[0] + TEAM_LIGHT_FRAME, TEAM_LIGHT_HALF[1] + TEAM_LIGHT_FRAME, 0.03, 0.006,
						12, 1);
				hullStrip(lights, theta, y - TEAM_LIGHT_HALF[0], y + TEAM_LIGHT_HALF[0], TEAM_LIGHT_HALF[1], 0.015,
						0.012, 12, 1);
			}
	}

	// ── Decals ───────────────────────────────────────────────────────────────

	/**
	 * Patches on both sides of the sail for the submarine's code, following the sail's surface
	 * DECAL_LIFT out from it. Texture coordinates run 0 to 1 across each patch, along the hull the
	 * way the text reads from that side (towards the stern on the port side, towards the bow on the
	 * starboard side) and upwards; the viewer paints the text.
	 */
	private void buildSailCode(Group g) {
		g.uv = UvMap.DECAL;
		int along = 16, up = 6;
		for (int side : new int[] {1, -1}) { // model +X is to port
			int[][] grid = new int[along + 1][up + 1];
			for (int i = 0; i <= along; i++)
				for (int j = 0; j <= up; j++) {
					double u = (double) i / along, v = (double) j / up;
					double y = side > 0 ? lerp(SAIL_CODE_Y[0], SAIL_CODE_Y[1], u)
							: lerp(SAIL_CODE_Y[1], SAIL_CODE_Y[0], u);
					double z = lerp(SAIL_CODE_Z[0], SAIL_CODE_Z[1], v);
					double[] edges = edgeAt(SAIL_BASE, SAIL_TOP, SAIL_BASE_Z, SAIL_TOP_Z, z);
					double c = edges[1] - edges[0];
					double x = side * (sailHalf((y - edges[0]) / c, c) + DECAL_LIFT);
					grid[i][j] = vertex(new double[] {x, y, z});
					decalUv.put(grid[i][j], new double[] {u, v});
				}
			decalGrid(g, grid, (i, j) -> {
				double[] p = verts.get(grid[i][j] - 1);
				return new double[] {0, p[1], p[2]};
			});
		}
	}

	/**
	 * Patches along the upper flanks for the submarine's name, HULL_NAME_HALF_HEIGHT either side of
	 * HULL_NAME_THETA, standing DECAL_LIFT off the hull. Texture coordinates as for
	 * {@link #buildSailCode}.
	 */
	private void buildHullName(Group g) {
		g.uv = UvMap.DECAL;
		int along = 64, up = 4;
		for (int side : new int[] {1, -1}) {
			int[][] grid = new int[along + 1][up + 1];
			for (int i = 0; i <= along; i++) {
				double u = (double) i / along;
				double y = side > 0 ? lerp(HULL_NAME_Y[0], HULL_NAME_Y[1], u) : lerp(HULL_NAME_Y[1], HULL_NAME_Y[0], u);
				double w = hullHalfWidth(y), h = hullHalfHeight(y), t = HULL_NAME_THETA;
				// Metres round the section per radian, so the letters keep their height as the hull narrows
				double spread = HULL_NAME_HALF_HEIGHT / Math.hypot(w * Math.sin(t), h * Math.cos(t));
				for (int j = 0; j <= up; j++) {
					double v = (double) j / up, theta = t + (2 * v - 1) * spread;
					double a = side > 0 ? theta : Math.PI - theta;
					double[] p = {w * Math.cos(a), y, AXIS_Z + h * Math.sin(a)};
					grid[i][j] = vertex(add(p, scale(hullNormal(p), DECAL_LIFT)));
					decalUv.put(grid[i][j], new double[] {u, v});
				}
			}
			decalGrid(g, grid, (i, j) -> onAxis(0, 0, verts.get(grid[i][j] - 1)[1]));
		}
	}

	/**
	 * Faces of a decal patch from its grid of vertices, turned away from the given inside points.
	 */
	private void decalGrid(Group g, int[][] grid, java.util.function.BiFunction<Integer, Integer, double[]> inside) {
		for (int i = 0; i + 1 < grid.length; i++)
			for (int j = 0; j + 1 < grid[i].length; j++)
				quad(g, grid[i][j], grid[i + 1][j], grid[i + 1][j + 1], grid[i][j + 1], inside.apply(i, j));
	}

	// ── Torpedo tubes ────────────────────────────────────────────────────────

	/**
	 * Station y where the line parallel to the hull's axis through (x, z) leaves the hull at the
	 * bow.
	 */
	private static double bowExitY(double x, double z) {
		double lo = NOSE_Y, hi = MID_Y0; // outside at the nose, inside at the widest point
		for (int i = 0; i < 60; i++) {
			double mid = (lo + hi) / 2;
			double w = hullHalfWidth(mid), h = hullHalfHeight(mid);
			double f = (x / w) * (x / w) + ((z - AXIS_Z) / h) * ((z - AXIS_Z) / h);
			if (f > 1)
				lo = mid;
			else
				hi = mid;
		}
		return (lo + hi) / 2;
	}

	/**
	 * Outward unit normal of the hull at a point on it (from the gradient of its implicit form).
	 */
	private static double[] hullNormal(double[] p) {
		double e = 1e-4;
		double[] grad = new double[3];
		for (int k = 0; k < 3; k++) {
			double[] a = p.clone(), b = p.clone();
			a[k] += e;
			b[k] -= e;
			grad[k] = (hullImplicit(a) - hullImplicit(b)) / (2 * e);
		}
		return unit(grad);
	}

	/** Below 1 inside the hull, above 1 outside. */
	private static double hullImplicit(double[] p) {
		double w = hullHalfWidth(p[1]), h = hullHalfHeight(p[1]);
		return (p[0] / w) * (p[0] / w) + ((p[2] - AXIS_Z) / h) * ((p[2] - AXIS_Z) / h);
	}

	/**
	 * The torpedo tubes. Each tube gets a real opening in the hull: the hull's triangles near the
	 * muzzle are cut away and the ragged edge left behind is stitched to a smooth ring where the
	 * tube meets the hull. Behind the opening a dark bore runs back into the hull, and a shutter
	 * sits {@link #DOOR_DEPTH} inside the skin, seen through the opening when closed. The shutter
	 * opens by turning about the hull's centreline: since the hull is wider than tall and the tubes
	 * sit below its widest point, turning it that way slides it further under the skin. The
	 * generator finds the turn that clears the opening and prints it with the muzzle positions for
	 * the engine and the viewer.
	 */
	private void buildTorpedoTubes(Group body, Group bores) {
		cutOpenings(body);
		int[][] muzzles = stitchOpenings(body);
		for (int t = 0; t < TUBES.length; t++) {
			double x0 = -TUBES[t][0], z0 = TUBES[t][1];
			int[] muzzle = muzzles[t];
			buildBore(bores, muzzle, x0, z0);
			Group door = group("TubeDoor" + (t + 1), "Metal_Black_Plain");
			int firstFace = door.faces.size();
			hullSkinPatch(door, x0, z0, TUBE_R + DOOR_OVERLAP, -DOOR_DEPTH);
			// The shutter must turn far enough that none of it is left under its own opening, nor parked behind another
			List<double[]> plate = new ArrayList<>();
			for (int f = firstFace; f < door.faces.size(); f++)
				for (Corner c : door.faces.get(f))
					plate.add(verts.get(c.v - 1));
			double open = Double.NaN;
			for (int step = 1; step <= 120 && Double.isNaN(open); step++)
				for (double sign : new double[] {1, -1}) {
					double angle = sign * Math.toRadians(0.5 * step);
					boolean clear = true;
					for (double[] p : plate) {
						double[] q = rotateAboutAxis(p, angle);
						if (underAnyOpening(q) || depthBelowHull(q) < DOOR_DEPTH * 0.9) {
							clear = false;
							break;
						}
					}
					if (clear) {
						open = angle;
						break;
					}
				}
			if (Double.isNaN(open))
				throw new IllegalStateException("TubeDoor" + (t + 1) + " cannot clear its opening");
			System.out.printf(Locale.ROOT,
					"TubeDoor%d: tube right %.2f up %.2f, muzzle at forward %.3f (opening %.1f times as long as wide); opens by %.4f rad about the centreline%n",
					t + 1, TUBES[t][0], TUBES[t][1], -bowExitY(x0, z0), elongation(muzzle), open);
		}
	}

	/**
	 * Removes every hull triangle that comes within {@link #CUT_MARGIN} of a tube's circle (seen
	 * along the hull's axis), for all tubes at once, so each opening is cut before any is stitched.
	 * Where cuts meet at a single vertex, the triangles round it go too, so every cut leaves a
	 * clean loop of edges.
	 */
	private void cutOpenings(Group body) {
		body.faces.removeIf(f -> {
			for (double[] tube : TUBES)
				for (int k = 0; k < 3; k++) {
					double[] a = verts.get(f[k].v - 1), b = verts.get(f[(k + 1) % 3].v - 1);
					if (a[1] < MID_Y0 && segmentDistance(a, b, -tube[0], tube[1]) < TUBE_R + CUT_MARGIN)
						return true;
				}
			return false;
		});
		while (true) {
			java.util.Set<Long> directed = new java.util.HashSet<>();
			for (Corner[] f : body.faces)
				for (int k = 0; k < 3; k++)
					directed.add(edgeKey(f[k].v, f[(k + 1) % 3].v));
			Map<Integer, Integer> outgoing = new HashMap<>();
			for (long key : directed)
				if (!directed.contains(edgeKey((int) key, (int) (key >> 32))))
					outgoing.merge((int) (key >> 32), 1, Integer::sum);
			java.util.Set<Integer> pinched = new java.util.HashSet<>();
			outgoing.forEach((v, n) -> {
				if (n > 1)
					pinched.add(v);
			});
			if (pinched.isEmpty())
				return;
			body.faces.removeIf(f -> pinched.contains(f[0].v) || pinched.contains(f[1].v) || pinched.contains(f[2].v));
		}
	}

	/**
	 * Closes the cuts: each ragged edge left by {@link #cutOpenings} is joined to smooth rings of
	 * new vertices on the hull at the radius of the tubes inside it. Cuts that ran into each other
	 * leave one edge round several tubes; that is triangulated with a hole per tube. Returns the
	 * ring of each tube.
	 */
	private int[][] stitchOpenings(Group body) {
		// The edges of the cuts: directed edges without a twin (the hull is otherwise closed)
		java.util.Set<Long> directed = new java.util.HashSet<>();
		for (Corner[] f : body.faces)
			for (int k = 0; k < 3; k++)
				directed.add(edgeKey(f[k].v, f[(k + 1) % 3].v));
		Map<Integer, Integer> next = new HashMap<>();
		for (long key : directed) {
			int a = (int) (key >> 32), b = (int) key;
			if (!directed.contains(edgeKey(b, a)) && next.put(a, b) != null)
				throw new IllegalStateException("the edge of a tube cut touches itself at vertex " + a);
		}
		int[][] rings = new int[TUBES.length][];
		while (!next.isEmpty()) {
			List<Integer> edge = new ArrayList<>();
			int start = next.keySet().iterator().next();
			Integer v = start;
			do {
				edge.add(v);
				v = next.remove(v);
				if (v == null)
					throw new IllegalStateException("the edge of a tube cut is not closed");
			} while (v != start);
			// Smooth rings on the hull at the radius of each tube inside this edge
			List<List<Integer>> holes = new ArrayList<>();
			for (int t = 0; t < TUBES.length; t++) {
				double x0 = -TUBES[t][0], z0 = TUBES[t][1];
				if (!encloses(edge, x0, z0))
					continue;
				int n = 32;
				rings[t] = new int[n];
				List<Integer> hole = new ArrayList<>();
				for (int i = 0; i < n; i++) {
					double phi = 2 * Math.PI * i / n;
					double x = x0 + TUBE_R * Math.cos(phi), z = z0 + TUBE_R * Math.sin(phi);
					rings[t][i] = vertex(new double[] {x, bowExitY(x, z), z});
					hole.add(rings[t][i]);
				}
				holes.add(hole);
			}
			if (holes.isEmpty())
				throw new IllegalStateException("a tube cut has no tube inside it");
			// On the hull unrolled round its axis the gap between edge and rings is a flat polygon with holes
			for (int[] tri : triangulateWithHoles(edge, holes))
				triOnHull(body, tri[0], tri[1], tri[2]);
			// Its slivers would average to streaky normals: the hull's own are known exactly
			body.onHull.addAll(edge);
			for (List<Integer> hole : holes)
				body.onHull.addAll(hole);
		}
		for (int t = 0; t < TUBES.length; t++)
			if (rings[t] == null)
				throw new IllegalStateException("no cut round tube " + (t + 1));
		return rings;
	}

	/** True if (x, z) lies inside {@code loop}, seen along the hull's axis. */
	private boolean encloses(List<Integer> loop, double x, double z) {
		boolean inside = false;
		for (int i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
			double[] a = verts.get(loop.get(i) - 1), b = verts.get(loop.get(j) - 1);
			if ((a[2] > z) != (b[2] > z) && x < a[0] + (z - a[2]) * (b[0] - a[0]) / (b[2] - a[2]))
				inside = !inside;
		}
		return inside;
	}

	/**
	 * The tube's bore: a dark cylinder from the opening's ring back into the hull, facing inwards,
	 * closed at the far end.
	 */
	private void buildBore(Group g, int[] ring, double x0, double z0) {
		double back = bowExitY(x0, z0) + 3.0;
		int n = ring.length;
		int[] deep = new int[n];
		for (int i = 0; i < n; i++) {
			double[] p = verts.get(ring[i] - 1);
			deep[i] = vertex(new double[] {p[0], back, p[2]});
		}
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			double[] a = verts.get(ring[i] - 1), b = verts.get(ring[j] - 1);
			double mx = (a[0] + b[0]) / 2, mz = (a[2] + b[2]) / 2, my = (a[1] + b[1] + 2 * back) / 4;
			// Seen from inside the tube: "inside" for the winding test is outside the cylinder
			double[] beyond = {x0 + (mx - x0) * 2, my, z0 + (mz - z0) * 2};
			quad(g, ring[i], ring[j], deep[j], deep[i], beyond);
		}
		int end = vertex(new double[] {x0, back, z0});
		for (int i = 0; i < n; i++)
			tri(g, end, deep[i], deep[(i + 1) % n], new double[] {x0, back + 1, z0});
	}

	/**
	 * True if {@code p} lies within a few centimetres of any tube's opening, seen along the hull's
	 * axis.
	 */
	private static boolean underAnyOpening(double[] p) {
		for (double[] tube : TUBES)
			if (Math.hypot(p[0] + tube[0], p[2] - tube[1]) < TUBE_R + 0.03)
				return true;
		return false;
	}

	/**
	 * How much longer than the tube's diameter an opening is, measured along the hull's surface.
	 */
	private double elongation(int[] ring) {
		double longest = 0;
		for (int a : ring)
			for (int b : ring)
				longest = Math.max(longest, distance(a, b));
		return longest / (2 * TUBE_R);
	}

	private double distance(int a, int b) {
		double[] d = sub(verts.get(a - 1), verts.get(b - 1));
		return Math.sqrt(dot(d, d));
	}

	/** Shortest distance, seen along the hull's axis, from (x0, z0) to the segment a-b. */
	private static double segmentDistance(double[] a, double[] b, double x0, double z0) {
		double ax = a[0] - x0, az = a[2] - z0, dx = b[0] - a[0], dz = b[2] - a[2];
		double len2 = dx * dx + dz * dz;
		double t = len2 > 0 ? Math.max(0, Math.min(1, -(ax * dx + az * dz) / len2)) : 0;
		return Math.hypot(ax + t * dx, az + t * dz);
	}

	/**
	 * Triangulates the region between {@code outer} and {@code holes} (vertex loops), working on
	 * the hull unrolled round its axis: each hole is joined to the polygon by a bridge (the hole
	 * furthest along u first), then ears are clipped off the resulting simple polygon. Works
	 * whatever the shape of the ragged outer edge.
	 */
	private List<int[]> triangulateWithHoles(List<Integer> outer, List<List<Integer>> holes) {
		List<Integer> poly = new ArrayList<>(outer);
		if (signedArea(poly) < 0)
			java.util.Collections.reverse(poly); // outer counter-clockwise
		List<List<Integer>> pending = new ArrayList<>();
		for (List<Integer> hole : holes) {
			List<Integer> h = new ArrayList<>(hole);
			if (signedArea(h) > 0)
				java.util.Collections.reverse(h); // holes clockwise
			pending.add(h);
		}
		pending.sort(java.util.Comparator.comparingDouble(h -> -maxU(h)));
		for (int done = 0; done < pending.size(); done++) {
			List<Integer> h = pending.get(done);
			// Bridge: the hole vertex furthest along u to the closest polygon vertex that can see it
			int hi = 0;
			for (int i = 1; i < h.size(); i++)
				if (uv(h.get(i))[0] > uv(h.get(hi))[0])
					hi = i;
			double[] hp = uv(h.get(hi));
			int oi = -1;
			double best = Double.MAX_VALUE;
			for (int i = 0; i < poly.size(); i++) {
				double[] op = uv(poly.get(i));
				double d = Math.hypot(op[0] - hp[0], op[1] - hp[1]);
				if (d >= best || crossesAny(hp, op, poly))
					continue;
				boolean blocked = false;
				for (int k = done; k < pending.size() && !blocked; k++)
					blocked = crossesAny(hp, op, pending.get(k));
				if (!blocked) {
					best = d;
					oi = i;
				}
			}
			if (oi < 0)
				throw new IllegalStateException("no bridge into a tube opening");
			List<Integer> joined = new ArrayList<>(poly.subList(0, oi + 1));
			for (int k = 0; k <= h.size(); k++)
				joined.add(h.get((hi + k) % h.size()));
			joined.addAll(poly.subList(oi, poly.size()));
			poly = joined;
		}
		// Ear clipping
		List<int[]> tris = new ArrayList<>();
		int guard = 0;
		while (poly.size() > 3 && guard++ < 100000) {
			boolean clipped = false;
			for (int i = 0; i < poly.size() && !clipped; i++) {
				int a = poly.get((i + poly.size() - 1) % poly.size()), b = poly.get(i),
						c = poly.get((i + 1) % poly.size());
				double[] pa = uv(a), pb = uv(b), pc = uv(c);
				if (cross2(pa, pb, pc) <= 1e-12)
					continue; // reflex or flat
				boolean empty = true;
				for (int w : poly) {
					if (w == a || w == b || w == c)
						continue;
					if (inTriangle(uv(w), pa, pb, pc)) {
						empty = false;
						break;
					}
				}
				if (empty) {
					tris.add(new int[] {a, b, c});
					poly.remove(i);
					clipped = true;
				}
			}
			if (!clipped)
				throw new IllegalStateException("could not triangulate round a tube opening");
		}
		tris.add(new int[] {poly.get(0), poly.get(1), poly.get(2)});
		return tris;
	}

	/** Furthest a loop reaches along u on the unrolled hull. */
	private double maxU(List<Integer> loop) {
		double u = -Double.MAX_VALUE;
		for (int v : loop)
			u = Math.max(u, uv(v)[0]);
		return u;
	}

	/**
	 * Position on the hull unrolled round its axis: u the angle from straight down (at the radius
	 * of the tubes), v along the hull. Unlike a projection along the axis, this stays one-to-one
	 * where the hull runs nearly parallel to its axis.
	 */
	private double[] uv(int v) {
		double[] p = verts.get(v - 1);
		return new double[] {2 * Math.atan2(p[0], AXIS_Z - p[2]), p[1]};
	}

	private double signedArea(List<Integer> loop) {
		double a = 0;
		for (int i = 0; i < loop.size(); i++) {
			double[] p = uv(loop.get(i)), q = uv(loop.get((i + 1) % loop.size()));
			a += p[0] * q[1] - q[0] * p[1];
		}
		return a / 2;
	}

	private static double cross2(double[] a, double[] b, double[] c) {
		return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
	}

	private static boolean inTriangle(double[] p, double[] a, double[] b, double[] c) {
		return cross2(a, b, p) >= -1e-12 && cross2(b, c, p) >= -1e-12 && cross2(c, a, p) >= -1e-12;
	}

	/**
	 * True if segment p-q properly crosses an edge of {@code loop} (edges sharing an end point
	 * don't count).
	 */
	private boolean crossesAny(double[] p, double[] q, List<Integer> loop) {
		for (int i = 0; i < loop.size(); i++) {
			double[] a = uv(loop.get(i)), b = uv(loop.get((i + 1) % loop.size()));
			if (same(a, p) || same(a, q) || same(b, p) || same(b, q))
				continue;
			double d1 = cross2(p, q, a), d2 = cross2(p, q, b), d3 = cross2(a, b, p), d4 = cross2(a, b, q);
			if (d1 * d2 < 0 && d3 * d4 < 0)
				return true;
		}
		return false;
	}

	private static boolean same(double[] a, double[] b) {
		return Math.abs(a[0] - b[0]) < 1e-12 && Math.abs(a[1] - b[1]) < 1e-12;
	}

	private static long edgeKey(int a, int b) {
		return ((long) a << 32) | (b & 0xffffffffL);
	}

	/**
	 * Point {@code p} turned by {@code angle} about the hull's centreline (along +y through z =
	 * AXIS_Z).
	 */
	private static double[] rotateAboutAxis(double[] p, double angle) {
		double c = Math.cos(angle), s = Math.sin(angle), dz = p[2] - AXIS_Z;
		return new double[] {p[0] * c + dz * s, p[1], AXIS_Z - p[0] * s + dz * c};
	}

	/**
	 * How far {@code p} lies inside the hull surface, measured from the centreline (negative
	 * outside).
	 */
	private static double depthBelowHull(double[] p) {
		double w = hullHalfWidth(p[1]), h = hullHalfHeight(p[1]);
		double dz = p[2] - AXIS_Z, a = Math.atan2(dz, p[0]);
		double surface = 1 / Math.sqrt(Math.pow(Math.cos(a) / w, 2) + Math.pow(Math.sin(a) / h, 2));
		return surface - Math.hypot(p[0], dz);
	}

	/**
	 * A slab on the hull over the patch a tube of radius {@code r} along the axis through (x0, z0)
	 * cuts out at the bow: its top is {@code raise} proud of the hull along the surface normal, its
	 * sides run 0.1 m into the hull. The top is built in concentric rings so that it follows the
	 * bow's curvature instead of cutting under it. Returns the outline of its top.
	 */
	private double[][] hullSkinPatch(Group g, double x0, double z0, double r, double raise) {
		return hullSkinPatch(g, x0, z0, r, raise, 28, 5);
	}

	private double[][] hullSkinPatch(Group g, double x0, double z0, double r, double raise, int n, int rings) {
		int[][] top = new int[rings + 1][n];
		int[] bottom = new int[n];
		double[][] rimTop = new double[n][];
		// The centre's normal is the mean of the first ring's: at the very tip of the nose the hull's own is undefined
		double[] cn = new double[3];
		for (int k = 1; k <= rings; k++)
			for (int i = 0; i < n; i++) {
				double phi = 2 * Math.PI * i / n, rr = r * k / rings;
				double x = x0 + rr * Math.cos(phi), z = z0 + rr * Math.sin(phi);
				double[] p = {x, bowExitY(x, z), z};
				double[] nrm = hullNormal(p);
				if (k == 1)
					cn = add(cn, nrm);
				double[] q = add(p, scale(nrm, raise));
				top[k][i] = vertex(q);
				if (k == rings) {
					rimTop[i] = q;
					bottom[i] = vertex(add(p, scale(nrm, -0.1)));
				}
			}
		double[] c = {x0, bowExitY(x0, z0), z0};
		cn = unit(cn);
		int hub = vertex(add(c, scale(cn, raise)));
		double[] inside = add(c, scale(cn, -0.05));
		for (int i = 0; i < n; i++) {
			int j = (i + 1) % n;
			tri(g, hub, top[1][i], top[1][j], inside);
			for (int k = 1; k < rings; k++)
				quad(g, top[k][i], top[k][j], top[k + 1][j], top[k + 1][i], inside);
			quad(g, top[rings][i], top[rings][j], bottom[j], bottom[i], inside);
		}
		return rimTop;
	}

	// ── Sail fittings ────────────────────────────────────────────────────────

	/** Height of the sail's flat top. */
	private static double sailDeckZ() {
		return SAIL_TOP_Z + SAIL_EDGE;
	}

	/**
	 * The sail's flat top at chord fraction {@code s}: {station y, half-width of the deck there}.
	 */
	private static double[] sailDeck(double s) {
		double c = SAIL_TOP[1] - SAIL_TOP[0];
		return new double[] {SAIL_TOP[0] + SAIL_EDGE + s * (c - 2 * SAIL_EDGE),
				Math.max(0.005, sailHalf(s, c) - SAIL_EDGE)};
	}

	/**
	 * Fittings on the sail top: the bridge cockpit, a well sunk {@link #WELL_DEPTH} into the front
	 * of the top whose own walls are the windscreen, with the bridge hatch on its floor (a polished
	 * coaming ring and a lid in its own group, so it can open later); behind it a periscope
	 * fairing, a radio mast fairing and a raised sensor mast with a radome.
	 */
	private void buildSailFittings(Group fittings, Group accents, Group hatch) {
		double deck = sailDeckZ(), floor = deck - WELL_DEPTH;
		double hy = sailDeck(HATCH_AT)[0];
		annulusZ(accents, 0, hy, floor - 0.02, floor + 0.07, HATCH_R_IN, HATCH_R_OUT, 40);
		cylinderZ(hatch, 0, hy, floor + 0.07, floor + 0.105, HATCH_R_IN + 0.02, 40);
		// Masts: two streamlined fairings over retracted masts and one raised sensor mast, all behind the well
		mastFairing(fittings, sailDeck(0.62)[0], 0.75, 0.6, deck + 0.35);
		mastFairing(fittings, sailDeck(0.76)[0], 0.5, 0.5, deck + 0.5);
		double my = sailDeck(0.87)[0];
		cylinderZ(fittings, 0, my, deck - 0.05, deck + 1.3, 0.055, 16);
		cylinderZ(fittings, 0, my, deck + 1.25, deck + 1.5, 0.11, 20);
	}

	/**
	 * Closes the sail's open top around the bridge cockpit. {@code top} is the top outline as the
	 * fin builder made it: {@code steps + 1} points along the starboard side from the leading to
	 * the trailing edge at chord fractions s_j = (1 - cos(pi j / steps)) / 2, then the port side
	 * back. The well's rim uses the same fractions, {@link #WELL_WALL} in from the edge, from the
	 * first one at or after {@link #WELL_FROM} to the last one at or before {@link #WELL_TO}, so
	 * the deck splits into quad strips along both sides plus a convex cap in front of the well and
	 * one behind it. Then the well's walls (facing in) and its floor.
	 */
	private void buildCockpit(Group g, int[] top, int steps) {
		double deck = sailDeckZ(), floor = deck - WELL_DEPTH;
		int from = 0, to = steps;
		while (chordStep(from, steps) < WELL_FROM)
			from++;
		while (chordStep(to, steps) > WELL_TO)
			to--;
		// Rim points per side: index 0 = starboard, 1 = port
		int span = to - from + 1;
		int[][] rimUp = new int[2][span], rimDown = new int[2][span];
		for (int k = 0; k < span; k++) {
			double[] d = sailDeck(chordStep(from + k, steps));
			for (int side = 0; side < 2; side++) {
				double x = (side == 0 ? 1 : -1) * (d[1] - WELL_WALL);
				rimUp[side][k] = vertex(new double[] {x, d[0], deck});
				rimDown[side][k] = vertex(new double[] {x, d[0], floor});
			}
		}
		double[] below = {0, sailDeck(0.5)[0], deck - 1};
		// Side strips between the deck edge and the rim
		for (int k = 0; k + 1 < span; k++) {
			int j = from + k;
			quad(g, top[j], top[j + 1], rimUp[0][k + 1], rimUp[0][k], below);
			quad(g, top[portIndex(j, steps)], top[portIndex(j + 1, steps)], rimUp[1][k + 1], rimUp[1][k], below);
		}
		// Caps in front of and behind the well: convex, so fans from their centres
		List<Integer> frontCap = new ArrayList<>(), backCap = new ArrayList<>();
		frontCap.add(rimUp[1][0]);
		for (int j = from; j >= 1; j--)
			frontCap.add(top[portIndex(j, steps)]);
		for (int j = 0; j <= from; j++)
			frontCap.add(top[j]);
		frontCap.add(rimUp[0][0]);
		backCap.add(rimUp[0][span - 1]);
		for (int j = to; j <= steps; j++)
			backCap.add(top[j]);
		for (int j = steps - 1; j >= to; j--)
			backCap.add(top[portIndex(j, steps)]);
		backCap.add(rimUp[1][span - 1]);
		fan(g, frontCap, below);
		fan(g, backCap, below);
		// Walls, facing into the well, round the rim: port side front to back, across, starboard back to front
		List<int[]> wall = new ArrayList<>(); // {up, down}
		for (int k = 0; k < span; k++)
			wall.add(new int[] {rimUp[1][k], rimDown[1][k]});
		for (int k = span - 1; k >= 0; k--)
			wall.add(new int[] {rimUp[0][k], rimDown[0][k]});
		double[] wellCentre = {0, (sailDeck(chordStep(from, steps))[0] + sailDeck(chordStep(to, steps))[0]) / 2, 0};
		List<Integer> floorLoop = new ArrayList<>();
		for (int i = 0; i < wall.size(); i++) {
			int[] a = wall.get(i), b = wall.get((i + 1) % wall.size());
			double[] pa = verts.get(a[0] - 1), pb = verts.get(b[0] - 1);
			double mx = (pa[0] + pb[0]) / 2, my = (pa[1] + pb[1]) / 2;
			double[] solid = {mx + (mx - wellCentre[0]) * 0.5, my + (my - wellCentre[1]) * 0.5, (deck + floor) / 2};
			quad(g, a[0], b[0], b[1], a[1], solid);
			floorLoop.add(a[1]);
		}
		fan(g, floorLoop, new double[] {0, wellCentre[1], floor - 1});
	}

	/** Chord fraction of outline point {@code j}. */
	private static double chordStep(int j, int steps) {
		return 0.5 * (1 - Math.cos(Math.PI * j / steps));
	}

	/**
	 * Index in a fin's top outline of the port-side point at step {@code j} (0 and steps are on the
	 * centreline).
	 */
	private static int portIndex(int j, int steps) {
		return j == 0 ? 0 : 2 * steps - j;
	}

	/**
	 * Closes a convex loop of vertices with a fan from its centre, facing away from {@code inside}.
	 */
	private void fan(Group g, List<Integer> loop, double[] inside) {
		double[] c = new double[3];
		for (int v : loop)
			c = add(c, scale(verts.get(v - 1), 1.0 / loop.size()));
		int hub = vertex(c);
		for (int i = 0; i < loop.size(); i++)
			tri(g, hub, loop.get(i), loop.get((i + 1) % loop.size()), inside);
	}

	/**
	 * A streamlined fairing on the sail top: chord {@code chord} centred at {@code y}, up to
	 * {@code top}.
	 */
	private void mastFairing(Group g, double y, double chord, double thickness, double top) {
		double base = sailDeckZ() - 0.15;
		fin(g, new double[] {0, y - chord / 2, base}, new double[] {0, y + chord / 2, base},
				new double[] {0, y - chord / 2, top - 0.05}, new double[] {0, y + chord / 2, top - 0.05},
				new double[] {1, 0, 0}, new double[] {0, 0, 1}, naca(thickness), 0.05, true, Part.WHOLE, 0, 3, 10);
	}

	/** A closed vertical cylinder at (x, y) from z0 to z1. */
	private void cylinderZ(Group g, double x, double y, double z0, double z1, double r, int n) {
		int[] lo = new int[n], hi = new int[n];
		for (int i = 0; i < n; i++) {
			double a = 2 * Math.PI * i / n;
			lo[i] = vertex(new double[] {x + r * Math.cos(a), y + r * Math.sin(a), z0});
			hi[i] = vertex(new double[] {x + r * Math.cos(a), y + r * Math.sin(a), z1});
		}
		int bottom = vertex(new double[] {x, y, z0}), top = vertex(new double[] {x, y, z1});
		double[] mid = {x, y, (z0 + z1) / 2};
		for (int i = 0; i < n; i++) {
			int i1 = (i + 1) % n;
			quad(g, lo[i], lo[i1], hi[i1], hi[i], mid);
			tri(g, top, hi[i], hi[i1], mid);
			tri(g, bottom, lo[i1], lo[i], mid);
		}
	}

	/**
	 * A closed vertical ring (rectangular section) at (x, y) from z0 to z1 between rIn and rOut.
	 */
	private void annulusZ(Group g, double x, double y, double z0, double z1, double rIn, double rOut, int n) {
		int[][] v = new int[n][4]; // outer bottom, outer top, inner top, inner bottom
		for (int i = 0; i < n; i++) {
			double a = 2 * Math.PI * i / n, ca = Math.cos(a), sa = Math.sin(a);
			v[i][0] = vertex(new double[] {x + rOut * ca, y + rOut * sa, z0});
			v[i][1] = vertex(new double[] {x + rOut * ca, y + rOut * sa, z1});
			v[i][2] = vertex(new double[] {x + rIn * ca, y + rIn * sa, z1});
			v[i][3] = vertex(new double[] {x + rIn * ca, y + rIn * sa, z0});
		}
		double rMid = (rIn + rOut) / 2, zMid = (z0 + z1) / 2;
		for (int i = 0; i < n; i++) {
			int i1 = (i + 1) % n;
			double a = 2 * Math.PI * (i + 0.5) / n;
			double[] inside = {x + rMid * Math.cos(a), y + rMid * Math.sin(a), zMid};
			for (int k = 0; k < 4; k++)
				quad(g, v[i][k], v[i1][k], v[i1][(k + 1) % 4], v[i][(k + 1) % 4], inside);
		}
	}

	// ── Accents ──────────────────────────────────────────────────────────────

	/** The accent rings: closed tubes with an elliptical section, centred on the shaft. */
	private void buildAccents(Group g) {
		int section = 12;
		for (double[] spec : ACCENT_RINGS) {
			double y = spec[0], r = spec[1], halfWidth = spec[2], halfThickness = spec[3];
			int around = (int) Math.max(48, Math.min(160, Math.round(r * 120))); // segments scale with the ring's size
			int[][] ring = new int[around][section];
			for (int i = 0; i < around; i++) {
				double a = 2 * Math.PI * i / around;
				for (int j = 0; j < section; j++) {
					double phi = 2 * Math.PI * j / section;
					ring[i][j] = vertex(onAxis(r + halfThickness * Math.cos(phi), a, y + halfWidth * Math.sin(phi)));
				}
			}
			for (int i = 0; i < around; i++) {
				int i1 = (i + 1) % around;
				double[] inside = onAxis(r, 2 * Math.PI * (i + 0.5) / around, y); // the tube's centre line
				for (int j = 0; j < section; j++) {
					int j1 = (j + 1) % section;
					quad(g, ring[i][j], ring[i1][j], ring[i1][j1], ring[i][j1], inside);
				}
			}
		}
	}

	// ── Mesh helpers ─────────────────────────────────────────────────────────

	/** Adds a vertex, rounded to the precision the OBJ is written with. */
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
		g.faces.add(new Corner[] {new Corner(a, 0), new Corner(b, 0), new Corner(c, 0)});
	}

	private void quad(Group g, int a, int b, int c, int d, double[] inside) {
		tri(g, a, b, c, inside);
		tri(g, a, c, d, inside);
	}

	/**
	 * Triangle on the hull from one counter-clockwise on the unrolled hull ({@link #uv}): that
	 * winding faces into the hull, so it is turned round to face out. Degenerate slivers are kept,
	 * since leaving them out would open the hull.
	 */
	private void triOnHull(Group g, int a, int b, int c) {
		g.faces.add(new Corner[] {new Corner(a, 0), new Corner(c, 0), new Corner(b, 0)});
	}

	/** Triangle on a surface around the shaft, wound to face away from it. */
	private void triAboutAxis(Group g, int a, int b, int c) {
		double y = (verts.get(a - 1)[1] + verts.get(b - 1)[1] + verts.get(c - 1)[1]) / 3;
		tri(g, a, b, c, onAxis(0, 0, y));
	}

	/**
	 * Assigns the group's normals: each corner gets the area-weighted sum of the normals of the
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
				double[] n = g.onHull.contains(f[k].v) ? hullNormal(verts.get(f[k].v - 1)) : unit(sum);
				String key = String.format(Locale.ROOT, "%.5f %.5f %.5f", n[0], n[1], n[2]);
				Integer idx = shared.get(key);
				if (idx == null) {
					normals.add(n);
					idx = normals.size();
					shared.put(key, idx);
				}
				f[k] = new Corner(f[k].v, idx);
			}
		}
	}

	private static double smoothstep(double x) {
		return x * x * (3 - 2 * x);
	}

	private static double lerp(double a, double b, double t) {
		return a + (b - a) * t;
	}

	private static double[] lerp(double[] a, double[] b, double t) {
		return new double[] {a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t};
	}

	private static double[] add(double[] a, double[] b) {
		return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
	}

	private static double[] scale(double[] a, double s) {
		return new double[] {a[0] * s, a[1] * s, a[2] * s};
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
}
