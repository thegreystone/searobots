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
package se.hirt.searobots.engine.replay;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.awt.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static se.hirt.searobots.engine.replay.ReplayFormat.*;

/**
 * Single source of truth for the on-disk shape of every replay record. Both {@link ReplayWriter} (encode) and
 * {@link ReplayReader} (decode) go through this class, so the two directions cannot drift apart.
 * <p>
 * Each record tag has an ordered list of {@link Col}s with a name and a unit. The writer emits these as {@code COLS}
 * lines in the header, making the file fully self-describing: any reader (Java, JavaScript, or an LLM) can interpret
 * the columns and their units from the header alone, with nothing external.
 * <p>
 * Decoding is <b>by column name</b>, resolved against the schema declared in the file (not against hardcoded
 * positions). That is what lets a reader built for one version still read the common columns of a file written by a
 * later version that appended new columns: {@link #VERSION} gates incompatible changes, but additive columns degrade
 * gracefully.
 * <p>
 * Conceptually each tag is its own rectangular table; the file interleaves them in tick order for streaming replay. A
 * separate analysis pass can split them back into per-tag tables joined on {@code (seed, tick)} for SQL/DuckDB.
 */
public final class ReplayCodec {

	private ReplayCodec() {
	}

	/** A column: a name and a unit (unit may be empty for dimensionless ids/counts). */
	public record Col(String name, String unit) {
	}

	private static Col c(String name, String unit) {
		return new Col(name, unit);
	}

	// --- Column schemas (the canonical contract). Free-text columns last. ---

	/** Submarine definition line ({@code S}); identity that is fixed for the match. */
	public static final List<Col> SUBDEF_COLS = List.of(c("id", ""), c("name", "text"), c("colorArgb", ""),
			c("spawnX", "m"), c("spawnY", "m"), c("spawnZ", "m"));

	/** Per-tick submarine state line ({@code s}). */
	public static final List<Col> SUB_COLS = List.of(c("id", ""), c("x", "m"), c("y", "m"), c("z", "m"),
			c("heading", "rad"), c("pitch", "rad"), c("roll", "rad"), c("velX", "m/s"), c("velY", "m/s"),
			c("velZ", "m/s"), c("angVelX", "rad/s"), c("angVelY", "rad/s"), c("angVelZ", "rad/s"), c("speed", "m/s"),
			c("hp", "hp"), c("noise", "linear"), c("sourceLevel", "dB"), c("throttle", "frac"), c("rudder", "frac"),
			c("sternPlanes", "frac"), c("torpedoesRemaining", ""), c("forfeited", "bool"), c("pingRequested", "bool"),
			c("status", "text"));

	/** Per-tick contact-estimate line ({@code c}); child of the preceding {@code s}. */
	public static final List<Col> CONTACT_COLS = List.of(c("x", "m"), c("y", "m"), c("confidence", "frac"),
			c("contactAlive", "frac"), c("uncertaintyRadius", "m"), c("estHeading", "rad"), c("estSpeed", "m/s"),
			c("label", "text"));

	/** Per-tick waypoint line ({@code w}); child of the preceding {@code s}. */
	public static final List<Col> WAYPOINT_COLS = List.of(c("x", "m"), c("y", "m"), c("z", "m"), c("active", "bool"),
			c("reverse", "bool"));

	/** Per-tick torpedo state line ({@code p}). */
	public static final List<Col> TORPEDO_COLS = List.of(c("id", ""), c("ownerId", ""), c("colorArgb", ""), c("x", "m"),
			c("y", "m"), c("z", "m"), c("heading", "rad"), c("pitch", "rad"), c("roll", "rad"), c("velX", "m/s"),
			c("velY", "m/s"), c("velZ", "m/s"), c("angVelX", "rad/s"), c("angVelY", "rad/s"), c("angVelZ", "rad/s"),
			c("speed", "m/s"), c("fuelRemaining", "s"), c("detonated", "bool"), c("alive", "bool"),
			c("sourceLevel", "dB"), c("pingRequested", "bool"), c("targetX", "m"), c("targetY", "m"), c("targetZ", "m"),
			c("diagEstX", "m"), c("diagEstY", "m"), c("diagEstZ", "m"), c("diagEstHeading", "rad"),
			c("diagEstSpeed", "m/s"), c("diagIntX", "m"), c("diagIntY", "m"), c("diagIntZ", "m"),
			c("diagPhase", "text"));

	/** All data tags that get a {@code COLS} declaration, keyed by record tag. */
	public static final Map<String, List<Col>> SCHEMAS = Map.of(ReplayFormat.TAG_SUBDEF, SUBDEF_COLS,
			ReplayFormat.TAG_SUB, SUB_COLS, ReplayFormat.TAG_CONTACT, CONTACT_COLS, ReplayFormat.TAG_WAYPOINT,
			WAYPOINT_COLS, ReplayFormat.TAG_TORPEDO, TORPEDO_COLS);

	// ---- COLS header line ----

	/** Builds a {@code COLS <tag> name:unit name:unit ...} header line for a tag. */
	public static String colsLine(String tag, List<Col> cols) {
		var sb = new StringBuilder();
		sb.append("COLS").append(DELIM).append(tag);
		for (Col col : cols) {
			sb.append(DELIM).append(col.name()).append(':').append(col.unit());
		}
		return sb.toString();
	}

	// ---- Schema (resolved from a file's COLS line) ----

	/**
	 * A column layout resolved from a file, mapping column name to its absolute index in a tab-split data line (the
	 * leading record tag is index 0, so the first declared column is index 1). Reading by name through this is what
	 * gives forward compatibility with additive schema changes.
	 */
	public static final class Schema {
		private final Map<String, Integer> indexByName = new HashMap<>();

		public static Schema fromColsTokens(String[] tokens) {
			// tokens: ["COLS", tag, "name:unit", ...]
			Schema s = new Schema();
			for (int i = 2; i < tokens.length; i++) {
				String t = tokens[i];
				int colon = t.indexOf(':');
				String name = colon >= 0 ? t.substring(0, colon) : t;
				s.indexByName.put(name, i - 1); // first column lands at data index 1
			}
			return s;
		}

		/** Schema matching this build's canonical column order for a tag. */
		public static Schema canonical(List<Col> cols) {
			Schema s = new Schema();
			for (int i = 0; i < cols.size(); i++) {
				s.indexByName.put(cols.get(i).name(), i + 1);
			}
			return s;
		}

		public boolean has(String name) {
			return indexByName.containsKey(name);
		}

		double d(String[] f, String name, double dflt) {
			Integer i = indexByName.get(name);
			return (i != null && i < f.length && !f[i].isEmpty()) ? parseD(f[i]) : dflt;
		}

		int i(String[] f, String name, int dflt) {
			Integer i = indexByName.get(name);
			return (i != null && i < f.length && !f[i].isEmpty()) ? Integer.parseInt(f[i]) : dflt;
		}

		boolean b(String[] f, String name) {
			Integer i = indexByName.get(name);
			return i != null && i < f.length && parseBool(f[i]);
		}

		String s(String[] f, String name) {
			Integer i = indexByName.get(name);
			return (i != null && i < f.length) ? f[i] : "";
		}
	}

	// ---- Encode ----

	public static void encodeSubDef(StringBuilder sb, SubmarineSnapshot sub, Vec3 spawn) {
		sb.append(ReplayFormat.TAG_SUBDEF);
		f(sb, Integer.toString(sub.id()));
		f(sb, sanitize(sub.name()));
		f(sb, Integer.toString(sub.color().getRGB()));
		f(sb, num(spawn.x()));
		f(sb, num(spawn.y()));
		f(sb, num(spawn.z()));
	}

	public static void encodeSub(StringBuilder sb, SubmarineSnapshot sub) {
		Pose pose = sub.pose();
		Vec3 pos = pose.position();
		Vec3 lin = sub.velocity().linear();
		Vec3 ang = sub.velocity().angular();
		sb.append(ReplayFormat.TAG_SUB);
		f(sb, Integer.toString(sub.id()));
		f(sb, num(pos.x()));
		f(sb, num(pos.y()));
		f(sb, num(pos.z()));
		f(sb, num(pose.heading()));
		f(sb, num(pose.pitch()));
		f(sb, num(pose.roll()));
		f(sb, num(lin.x()));
		f(sb, num(lin.y()));
		f(sb, num(lin.z()));
		f(sb, num(ang.x()));
		f(sb, num(ang.y()));
		f(sb, num(ang.z()));
		f(sb, num(sub.speed()));
		f(sb, Integer.toString(sub.hp()));
		f(sb, num(sub.noiseLevel()));
		f(sb, num(sub.sourceLevelDb()));
		f(sb, num(sub.throttle()));
		f(sb, num(sub.rudder()));
		f(sb, num(sub.sternPlanes()));
		f(sb, Integer.toString(sub.torpedoesRemaining()));
		f(sb, bool(sub.forfeited()));
		f(sb, bool(sub.pingRequested()));
		f(sb, sanitize(sub.status()));
	}

	public static void encodeContact(StringBuilder sb, ContactEstimate ce) {
		sb.append(ReplayFormat.TAG_CONTACT);
		f(sb, num(ce.x()));
		f(sb, num(ce.y()));
		f(sb, num(ce.confidence()));
		f(sb, num(ce.contactAlive()));
		f(sb, num(ce.uncertaintyRadius()));
		f(sb, num(ce.estimatedHeading()));
		f(sb, num(ce.estimatedSpeed()));
		f(sb, sanitize(ce.label()));
	}

	public static void encodeWaypoint(StringBuilder sb, Waypoint wp) {
		sb.append(ReplayFormat.TAG_WAYPOINT);
		f(sb, num(wp.x()));
		f(sb, num(wp.y()));
		f(sb, num(wp.z()));
		f(sb, bool(wp.active()));
		f(sb, bool(wp.reverse()));
	}

	public static void encodeTorpedo(StringBuilder sb, TorpedoSnapshot t) {
		Pose pose = t.pose();
		Vec3 pos = pose.position();
		Vec3 lin = t.velocity().linear();
		Vec3 ang = t.velocity().angular();
		sb.append(ReplayFormat.TAG_TORPEDO);
		f(sb, Integer.toString(t.id()));
		f(sb, Integer.toString(t.ownerId()));
		f(sb, Integer.toString(t.color().getRGB()));
		f(sb, num(pos.x()));
		f(sb, num(pos.y()));
		f(sb, num(pos.z()));
		f(sb, num(pose.heading()));
		f(sb, num(pose.pitch()));
		f(sb, num(pose.roll()));
		f(sb, num(lin.x()));
		f(sb, num(lin.y()));
		f(sb, num(lin.z()));
		f(sb, num(ang.x()));
		f(sb, num(ang.y()));
		f(sb, num(ang.z()));
		f(sb, num(t.speed()));
		f(sb, num(t.fuelRemaining()));
		f(sb, bool(t.detonated()));
		f(sb, bool(t.alive()));
		f(sb, num(t.sourceLevelDb()));
		f(sb, bool(t.pingRequested()));
		f(sb, num(t.targetX()));
		f(sb, num(t.targetY()));
		f(sb, num(t.targetZ()));
		f(sb, num(t.diagEstX()));
		f(sb, num(t.diagEstY()));
		f(sb, num(t.diagEstZ()));
		f(sb, num(t.diagEstHeading()));
		f(sb, num(t.diagEstSpeed()));
		f(sb, num(t.diagIntX()));
		f(sb, num(t.diagIntY()));
		f(sb, num(t.diagIntZ()));
		f(sb, sanitize(t.diagPhase()));
	}

	// ---- Decode (by name, via the file's resolved Schema) ----

	public static ContactEstimate decodeContact(String[] f, Schema s) {
		return new ContactEstimate(s.d(f, "x", 0), s.d(f, "y", 0), s.d(f, "confidence", 0), s.d(f, "contactAlive", 0),
				s.d(f, "uncertaintyRadius", 0), s.d(f, "estHeading", Double.NaN), s.d(f, "estSpeed", -1),
				s.s(f, "label"));
	}

	public static Waypoint decodeWaypoint(String[] f, Schema s) {
		return new Waypoint(s.d(f, "x", 0), s.d(f, "y", 0), s.d(f, "z", 0), s.b(f, "active"), s.b(f, "reverse"));
	}

	public static TorpedoSnapshot decodeTorpedo(String[] f, Schema s) {
		Pose pose = new Pose(new Vec3(s.d(f, "x", 0), s.d(f, "y", 0), s.d(f, "z", 0)), s.d(f, "heading", 0),
				s.d(f, "pitch", 0), s.d(f, "roll", 0));
		Velocity vel = new Velocity(new Vec3(s.d(f, "velX", 0), s.d(f, "velY", 0), s.d(f, "velZ", 0)),
				new Vec3(s.d(f, "angVelX", 0), s.d(f, "angVelY", 0), s.d(f, "angVelZ", 0)));
		return new TorpedoSnapshot(s.i(f, "id", 0), s.i(f, "ownerId", 0), pose, vel, s.d(f, "speed", 0),
				new Color(s.i(f, "colorArgb", Color.GRAY.getRGB()), true), s.d(f, "fuelRemaining", 0),
				s.b(f, "detonated"), s.b(f, "alive"), s.d(f, "sourceLevel", 0), s.b(f, "pingRequested"),
				s.d(f, "targetX", 0), s.d(f, "targetY", 0), s.d(f, "targetZ", 0), s.d(f, "diagEstX", Double.NaN),
				s.d(f, "diagEstY", Double.NaN), s.d(f, "diagEstZ", Double.NaN), s.d(f, "diagEstHeading", Double.NaN),
				s.d(f, "diagEstSpeed", Double.NaN), s.d(f, "diagIntX", Double.NaN), s.d(f, "diagIntY", Double.NaN),
				s.d(f, "diagIntZ", Double.NaN), s.s(f, "diagPhase"));
	}

	private static void f(StringBuilder sb, String field) {
		sb.append(DELIM).append(field);
	}
}
