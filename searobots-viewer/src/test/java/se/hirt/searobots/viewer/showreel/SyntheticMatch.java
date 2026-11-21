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
package se.hirt.searobots.viewer.showreel;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import se.hirt.searobots.api.Pose;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Velocity;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

/**
 * A hand-made match for the showreel tests. Seabed 400 m deep with one island and an underwater
 * ridge. "Claude Sub" (id 0) cruises east past the ridge while "Codex Sub" (id 1) waits; both ping
 * now and then. At {@link #LAUNCH} Claude fires a torpedo that runs Codex down and sinks it at
 * {@link #BLAST}; the match runs on for twenty seconds.
 */
final class SyntheticMatch {
	static final int RATE = 50;
	static final long LAUNCH = 8L * 60 * RATE;
	static final long BLAST = LAUNCH + 60L * RATE;
	static final long END = BLAST + 20L * RATE;
	static final int TORPEDO = 1000;
	static final double ISLAND_X = 2500, ISLAND_Y = 2500;

	private SyntheticMatch() {
	}

	/** 17 km of seabed at -400 m, a 150 m island, and a ridge rising to -60 m along y = 300. */
	static TerrainMap terrain() {
		int n = 341;
		double cell = 50, origin = -(n / 2) * cell;
		double[] e = new double[n * n];
		for (int r = 0; r < n; r++) {
			for (int c = 0; c < n; c++) {
				double x = origin + c * cell, y = origin + r * cell;
				double island = 150 - 0.25 * Math.hypot(x - ISLAND_X, y - ISLAND_Y); // a cone, 1.1 km wide at sea level
				double ridge = Math.abs(y - 300) < 100 && Math.abs(x) < 2500 ? -60 : -400;
				e[r * n + c] = Math.max(-400, Math.max(island, ridge));
			}
		}
		return new TerrainMap(e, n, n, origin, origin, cell);
	}

	/** Feeds the whole match to {@code index}. */
	static void play(MatchIndex index) {
		var codex = new double[] {3000, -2000, -150};
		for (long tick = 0; tick <= END; tick++) {
			double t = tick / (double) RATE;
			double cx = -3000 + 5 * t; // Claude cruises east along the ridge
			boolean dead = tick >= BLAST;
			var subs = List.of(submarine(0, "Claude Sub", cx, 200, -150, 1000, tick % 400 == 0),
					submarine(1, "Codex Sub", codex[0], codex[1], dead ? -150 - 2 * (t - BLAST / (double) RATE) : -150,
							dead ? 0 : 1000, tick % 300 == 150));
			var torpedoes = new ArrayList<TorpedoSnapshot>();
			if (tick >= LAUNCH && tick <= BLAST) {
				double f = (tick - LAUNCH) / (double) (BLAST - LAUNCH);
				double launchX = -3000 + 5 * LAUNCH / (double) RATE;
				double x = launchX + (codex[0] - launchX) * f, y = 200 + (codex[1] + 30 - 200) * f;
				torpedoes.add(torpedo(x, y, tick == BLAST));
			}
			index.onTick(tick, subs, torpedoes);
		}
		index.onMatchEnd();
	}

	private static SubmarineSnapshot submarine(
		int id, String name, double x, double y, double z, int hp, boolean ping) {
		return new SubmarineSnapshot(id, name, name.substring(0, 2), Pose.at(x, y, z, Math.PI / 2), Velocity.ZERO, 5,
				Color.CYAN, false, hp, 1, 100, 0.5, 0, 0, "", ping, 8, List.of(), List.of(), List.of(), null, false);
	}

	private static TorpedoSnapshot torpedo(double x, double y, boolean detonated) {
		return new TorpedoSnapshot(TORPEDO, 0, Pose.at(x, y, -150, Math.PI / 2), Velocity.ZERO, 20, Color.CYAN, 100,
				detonated, !detonated, 150, false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
				Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, "");
	}
}
