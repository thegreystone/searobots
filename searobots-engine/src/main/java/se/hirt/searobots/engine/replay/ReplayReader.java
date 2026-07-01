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
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;
import se.hirt.searobots.engine.replay.ReplayCodec.Schema;

import java.awt.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static se.hirt.searobots.engine.replay.ReplayFormat.DELIM;
import static se.hirt.searobots.engine.replay.ReplayFormat.parseD;

/**
 * Reads a {@link ReplayFormat} file and replays it through a {@link SimulationListener}, reconstructing the per-tick
 * snapshots that were recorded. Because the viewers are {@code SimulationListener}s, this lets the existing 2D/3D
 * viewers play back a recorded match unchanged, and lets tests verify a recording is lossless against the live match.
 * <p>
 * All record decoding is delegated to {@link ReplayCodec}, resolving columns by the names declared in the file's
 * {@code COLS} header lines, so the reader is tolerant of additive schema changes within a format version.
 * <p>
 * The header is parsed eagerly in the constructor; tick data is streamed in {@link #replay(SimulationListener)}, so
 * memory use is bounded by one frame.
 */
public final class ReplayReader {

	private final Path file;
	private final ReplayHeader header;
	private final Map<Integer, ReplayHeader.SubDef> subDefsById = new HashMap<>();
	private final Map<String, Schema> schemas = new HashMap<>();

	public ReplayReader(Path file) throws IOException {
		this.file = file;
		this.header = parseHeader();
		for (var def : header.submarines()) {
			subDefsById.put(def.id(), def);
		}
	}

	public ReplayHeader header() {
		return header;
	}

	private ReplayHeader parseHeader() throws IOException {
		try (BufferedReader r = Files.newBufferedReader(file)) {
			String magicLine = r.readLine();
			if (magicLine == null) {
				throw new IOException("Empty replay file: " + file);
			}
			String[] magic = magicLine.split(DELIM, -1);
			if (magic.length < 2 || !ReplayFormat.MAGIC.equals(magic[0])) {
				throw new IOException("Not a SeaRobots replay file: " + file);
			}
			int version = Integer.parseInt(magic[1]);
			if (version != ReplayFormat.VERSION) {
				throw new IOException(
						"Unsupported replay format version " + version + " (this build reads version " + ReplayFormat.VERSION + ")");
			}

			long seed = 0;
			int tickRate = 0;
			long durationTicks = 0;
			int startingHp = 0;
			double crushDepth = 0;
			double ratedDepth = 0;
			BattleArea area = new BattleArea.Circular(7000.0);
			List<ReplayHeader.SubDef> subs = new ArrayList<>();

			String lineStr;
			while ((lineStr = r.readLine()) != null) {
				String[] f = lineStr.split(DELIM, -1);
				String tag = f[0];
				if (ReplayFormat.TAG_HEADER.equals(tag)) {
					Map<String, String> kv = new HashMap<>();
					for (int i = 1; i < f.length; i++) {
						int eq = f[i].indexOf('=');
						if (eq > 0) {
							kv.put(f[i].substring(0, eq), f[i].substring(eq + 1));
						}
					}
					seed = Long.parseLong(kv.getOrDefault("seed", "0"));
					tickRate = Integer.parseInt(kv.getOrDefault("tickRate", "0"));
					durationTicks = Long.parseLong(kv.getOrDefault("durationTicks", "0"));
					startingHp = Integer.parseInt(kv.getOrDefault("startingHp", "0"));
					crushDepth = parseD(kv.getOrDefault("crushDepth", "0"));
					ratedDepth = parseD(kv.getOrDefault("ratedDepth", "0"));
					area = parseArena(kv.getOrDefault("arena", "circle:7000"));
				} else if ("COLS".equals(tag)) {
					schemas.put(f[1], Schema.fromColsTokens(f));
				} else if (ReplayFormat.TAG_SUBDEF.equals(tag)) {
					Schema s = schemaFor(ReplayFormat.TAG_SUBDEF, ReplayCodec.SUBDEF_COLS);
					subs.add(new ReplayHeader.SubDef(s.i(f, "id", 0), s.s(f, "name"),
							s.i(f, "colorArgb", Color.GRAY.getRGB()), s.d(f, "spawnX", 0), s.d(f, "spawnY", 0),
							s.d(f, "spawnZ", 0)));
				} else if (ReplayFormat.TAG_TICK.equals(tag) || ReplayFormat.TAG_END.equals(tag)) {
					break; // header section ends at the first frame
				}
			}
			return new ReplayHeader(version, seed, tickRate, durationTicks, startingHp, crushDepth, ratedDepth, area,
					subs);
		}
	}

	private static BattleArea parseArena(String spec) {
		String[] parts = spec.split(":");
		if (parts.length >= 3 && "rect".equals(parts[0])) {
			return new BattleArea.Rectangular(parseD(parts[1]), parseD(parts[2]));
		}
		double radius = parts.length >= 2 ? parseD(parts[1]) : 7000.0;
		return new BattleArea.Circular(radius);
	}

	/** Schema declared in the file for a tag, or the canonical one if absent. */
	private Schema schemaFor(String tag, List<ReplayCodec.Col> canonical) {
		return schemas.computeIfAbsent(tag, t -> Schema.canonical(canonical));
	}

	/**
	 * Streams the recorded match through {@code listener}, calling {@link SimulationListener#onTick} once per recorded
	 * tick and {@link SimulationListener#onMatchEnd} at the end.
	 *
	 * @return the number of ticks replayed
	 */
	public long replay(SimulationListener listener) throws IOException {
		Schema subSchema = schemaFor(ReplayFormat.TAG_SUB, ReplayCodec.SUB_COLS);
		Schema contactSchema = schemaFor(ReplayFormat.TAG_CONTACT, ReplayCodec.CONTACT_COLS);
		Schema waypointSchema = schemaFor(ReplayFormat.TAG_WAYPOINT, ReplayCodec.WAYPOINT_COLS);
		Schema torpedoSchema = schemaFor(ReplayFormat.TAG_TORPEDO, ReplayCodec.TORPEDO_COLS);

		try (BufferedReader r = Files.newBufferedReader(file)) {
			Frame frame = new Frame();
			long ticks = 0;
			String lineStr;
			while ((lineStr = r.readLine()) != null) {
				String[] f = lineStr.split(DELIM, -1);
				switch (f[0]) {
				case ReplayFormat.TAG_TICK -> {
					if (frame.hasTick) {
						frame.emit(listener);
						ticks++;
					}
					frame.start(Long.parseLong(f[1]));
				}
				case ReplayFormat.TAG_SUB -> frame.startSub(parseSub(f, subSchema));
				case ReplayFormat.TAG_CONTACT -> frame.addContact(ReplayCodec.decodeContact(f, contactSchema));
				case ReplayFormat.TAG_WAYPOINT -> frame.addWaypoint(ReplayCodec.decodeWaypoint(f, waypointSchema));
				case ReplayFormat.TAG_TORPEDO -> frame.addTorpedo(ReplayCodec.decodeTorpedo(f, torpedoSchema));
				case ReplayFormat.TAG_END -> {
					if (frame.hasTick) {
						frame.emit(listener);
						ticks++;
					}
					listener.onMatchEnd();
					return ticks;
				}
				default -> {
					// magic, H, COLS, S, or unknown future tags: skip
				}
				}
			}
			if (frame.hasTick) {
				frame.emit(listener);
				ticks++;
			}
			listener.onMatchEnd();
			return ticks;
		}
	}

	private PartialSub parseSub(String[] f, Schema s) {
		var p = new PartialSub();
		p.id = s.i(f, "id", 0);
		ReplayHeader.SubDef def = subDefsById.get(p.id);
		p.name = def != null ? def.name() : "sub" + p.id;
		p.color = new Color(def != null ? def.colorRgb() : Color.GRAY.getRGB(), true);
		p.pose = new Pose(new Vec3(s.d(f, "x", 0), s.d(f, "y", 0), s.d(f, "z", 0)), s.d(f, "heading", 0),
				s.d(f, "pitch", 0), s.d(f, "roll", 0));
		p.velocity = new Velocity(new Vec3(s.d(f, "velX", 0), s.d(f, "velY", 0), s.d(f, "velZ", 0)),
				new Vec3(s.d(f, "angVelX", 0), s.d(f, "angVelY", 0), s.d(f, "angVelZ", 0)));
		p.speed = s.d(f, "speed", 0);
		p.hp = s.i(f, "hp", 0);
		p.noiseLevel = s.d(f, "noise", 0);
		p.sourceLevelDb = s.d(f, "sourceLevel", 0);
		p.throttle = s.d(f, "throttle", 0);
		p.rudder = s.d(f, "rudder", 0);
		p.sternPlanes = s.d(f, "sternPlanes", 0);
		p.torpedoesRemaining = s.i(f, "torpedoesRemaining", 0);
		p.forfeited = s.b(f, "forfeited");
		p.pingRequested = s.b(f, "pingRequested");
		p.status = s.s(f, "status");
		return p;
	}

	// --- frame assembly ---

	private static final class Frame {
		boolean hasTick;
		long tick;
		List<SubmarineSnapshot> subs = new ArrayList<>();
		List<TorpedoSnapshot> torps = new ArrayList<>();
		PartialSub pending;

		void start(long t) {
			finishSub();
			hasTick = true;
			tick = t;
			subs = new ArrayList<>();
			torps = new ArrayList<>();
			pending = null;
		}

		void startSub(PartialSub s) {
			finishSub();
			pending = s;
		}

		void addContact(ContactEstimate ce) {
			if (pending != null) {
				pending.contacts.add(ce);
			}
		}

		void addWaypoint(Waypoint wp) {
			if (pending != null) {
				pending.waypoints.add(wp);
			}
		}

		void addTorpedo(TorpedoSnapshot t) {
			finishSub();
			torps.add(t);
		}

		void finishSub() {
			if (pending != null) {
				subs.add(pending.build());
				pending = null;
			}
		}

		void emit(SimulationListener listener) {
			finishSub();
			listener.onTick(tick, subs, torps);
		}
	}

	/** Mutable accumulator for a submarine and its child contact/waypoint lines. */
	private static final class PartialSub {
		int id;
		String name;
		Pose pose;
		Velocity velocity;
		double speed;
		Color color;
		boolean forfeited;
		int hp;
		double noiseLevel;
		double sourceLevelDb;
		double throttle;
		double rudder;
		double sternPlanes;
		String status;
		boolean pingRequested;
		int torpedoesRemaining;
		final List<ContactEstimate> contacts = new ArrayList<>();
		final List<Waypoint> waypoints = new ArrayList<>();

		SubmarineSnapshot build() {
			return new SubmarineSnapshot(id, name, pose, velocity, speed, color, forfeited, hp, noiseLevel,
					sourceLevelDb, throttle, rudder, sternPlanes, status, pingRequested, torpedoesRemaining,
					List.copyOf(contacts), List.copyOf(waypoints), List.of(), null);
		}
	}
}
