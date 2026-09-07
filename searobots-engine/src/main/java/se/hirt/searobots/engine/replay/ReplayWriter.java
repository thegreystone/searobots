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

import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static se.hirt.searobots.engine.replay.ReplayFormat.DELIM;
import static se.hirt.searobots.engine.replay.ReplayFormat.num;

/**
 * A {@link SimulationListener} that writes a complete, replayable record of a match to a file in the
 * {@link ReplayFormat}, delegating all record encoding to {@link ReplayCodec} (the single source of truth shared with
 * the reader).
 * <p>
 * Unlike {@code MatchRecorder} (which downsamples and omits torpedoes for human-readable analysis), this captures
 * <em>every</em> tick at full fidelity, including torpedoes, so the match replays directly from the log with no
 * re-simulation. That sidesteps determinism entirely: controllers may use randomness, but the replay reproduces what
 * actually happened.
 * <p>
 * The {@code SimulationLoop} accepts only one listener, so to record a live match while rendering it, wrap this and the
 * viewer with {@link se.hirt.searobots.engine.SimulationListeners#composite}.
 * <p><b>v1 limitations (documented extension points):</b> strategic-waypoint
 * visualization and firing solutions are not yet captured; they reconstruct as empty/null. Adding them is an additive
 * column change plus a {@link ReplayFormat#VERSION} bump.
 */
public final class ReplayWriter implements SimulationListener, AutoCloseable {

	private final BufferedWriter writer;
	private final Path file;
	private final List<Vec3> spawnPoints;
	private boolean wroteSubDefs;

	public ReplayWriter(MatchConfig config, List<Vec3> spawnPoints, Path file) throws IOException {
		Path parent = file.getParent();
		if (parent != null) {
			Files.createDirectories(parent);
		}
		this.file = file;
		this.spawnPoints = spawnPoints;
		this.writer = Files.newBufferedWriter(file);
		writeHeader(config);
	}

	public Path file() {
		return file;
	}

	private void writeHeader(MatchConfig config) throws IOException {
		// Line 1: magic + format version; the reader dispatches on this.
		writeLine(ReplayFormat.MAGIC + DELIM + ReplayFormat.VERSION);

		// Match parameters as self-describing key=value tokens.
		String arena = switch (config.battleArea()) {
			case BattleArea.Circular c -> "circle:" + num(c.radius());
			case BattleArea.Rectangular r -> "rect:" + num(r.halfWidth()) + ":" + num(r.halfHeight());
		};
		writeLine(String.join(DELIM, ReplayFormat.TAG_HEADER, "seed=" + config.worldSeed(),
				"tickRate=" + config.tickRateHz(), "durationTicks=" + config.matchDurationTicks(),
				"startingHp=" + config.startingHp(), "crushDepth=" + num(config.crushDepth()),
				"ratedDepth=" + num(config.ratedDepth()), "arena=" + arena));

		// Column schemas (names + units) make the file self-describing.
		writeLine(ReplayCodec.colsLine(ReplayFormat.TAG_SUBDEF, ReplayCodec.SUBDEF_COLS));
		writeLine(ReplayCodec.colsLine(ReplayFormat.TAG_SUB, ReplayCodec.SUB_COLS));
		writeLine(ReplayCodec.colsLine(ReplayFormat.TAG_CONTACT, ReplayCodec.CONTACT_COLS));
		writeLine(ReplayCodec.colsLine(ReplayFormat.TAG_WAYPOINT, ReplayCodec.WAYPOINT_COLS));
		writeLine(ReplayCodec.colsLine(ReplayFormat.TAG_FIRING, ReplayCodec.FIRING_COLS));
		writeLine(ReplayCodec.colsLine(ReplayFormat.TAG_TORPEDO, ReplayCodec.TORPEDO_COLS));
	}

	@Override
	public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
		try {
			if (!wroteSubDefs) {
				for (int i = 0; i < submarines.size(); i++) {
					Vec3 spawn = i < spawnPoints.size() ? spawnPoints.get(i) : Vec3.ZERO;
					var sb = new StringBuilder(64);
					ReplayCodec.encodeSubDef(sb, submarines.get(i), spawn);
					writeLine(sb.toString());
				}
				wroteSubDefs = true;
			}
			writeLine(ReplayFormat.TAG_TICK + DELIM + tick);
			for (var sub : submarines) {
				var sb = new StringBuilder(160);
				ReplayCodec.encodeSub(sb, sub);
				writeLine(sb.toString());
				if (sub.contactEstimates() != null) {
					for (var ce : sub.contactEstimates()) {
						var cb = new StringBuilder(80);
						ReplayCodec.encodeContact(cb, ce);
						writeLine(cb.toString());
					}
				}
				if (sub.waypoints() != null) {
					for (var wp : sub.waypoints()) {
						var wb = new StringBuilder(48);
						ReplayCodec.encodeWaypoint(wb, wp);
						writeLine(wb.toString());
					}
				}
				if (sub.firingSolution() != null) {
					var fb = new StringBuilder(48);
					ReplayCodec.encodeFiring(fb, sub.firingSolution());
					writeLine(fb.toString());
				}
			}
			if (torpedoes != null) {
				for (var t : torpedoes) {
					var tb = new StringBuilder(220);
					ReplayCodec.encodeTorpedo(tb, t);
					writeLine(tb.toString());
				}
			}
		} catch (IOException e) {
			throw new UncheckedIOException("Replay write failed at tick " + tick, e);
		}
	}

	@Override
	public void onMatchEnd() {
		try {
			writeLine(ReplayFormat.TAG_END);
			writer.flush();
			writer.close();
		} catch (IOException e) {
			throw new UncheckedIOException("Replay close failed", e);
		}
	}

	@Override
	public void close() {
		try {
			writer.close();
		} catch (IOException ignored) {
			// best effort
		}
	}

	private void writeLine(String line) throws IOException {
		writer.write(line);
		writer.write('\n');
	}
}
