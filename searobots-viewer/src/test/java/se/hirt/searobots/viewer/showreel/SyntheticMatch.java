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
 * now and then. At {@link #LAUNCH} Claude fires a pair: {@link #TORPEDO} runs Codex down and sinks
 * it at {@link #BLAST}, and {@link #STRAGGLER} arrives at the wreck a moment later. The match runs
 * on for twenty seconds. Optionally, Codex has already fired back, and its {@link #COUNTER} torpedo
 * kills Claude too.
 */
final class SyntheticMatch {
	static final int RATE = 50;
	static final long LAUNCH = 8L * 60 * RATE;
	static final long BLAST = LAUNCH + 60L * RATE;
	static final long STRAGGLER_BLAST = BLAST + 30;
	static final long END = BLAST + 20L * RATE;
	static final int TORPEDO = 1000, STRAGGLER = 1001, COUNTER = 1002;
	static final long COUNTER_LAUNCH = LAUNCH + 20L * RATE;
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

	/** Feeds the whole match to {@code index}; only Codex dies. */
	static void play(MatchIndex index) {
		play(index, Double.NaN);
	}

	/**
	 * Feeds the whole match to {@code index}. Unless {@code counterKillAfter} is NaN, Codex's
	 * counter torpedo kills Claude that many seconds after {@link #BLAST}.
	 */
	static void play(MatchIndex index, double counterKillAfter) {
		boolean mutual = !Double.isNaN(counterKillAfter);
		long claudeDies = mutual ? BLAST + Math.round(counterKillAfter * RATE) : Long.MAX_VALUE;
		long end = mutual ? Math.max(END, claudeDies + 20L * RATE) : END;
		var codex = new double[] {3000, -2000, -150};
		double launchX = claudeX(LAUNCH);
		for (long tick = 0; tick <= end; tick++) {
			double t = tick / (double) RATE;
			boolean codexDead = tick >= BLAST, claudeDead = tick >= claudeDies;
			var subs = List.of(
					submarine(0, "Claude Sub", claudeX(Math.min(tick, claudeDies)), 200,
							claudeDead ? -150 - 2 * (tick - claudeDies) / (double) RATE : -150, claudeDead ? 0 : 1000,
							tick % 400 == 0),
					submarine(1, "Codex Sub", codex[0], codex[1],
							codexDead ? -150 - 2 * (t - BLAST / (double) RATE) : -150, codexDead ? 0 : 1000,
							tick % 300 == 150));
			var torpedoes = new ArrayList<TorpedoSnapshot>();
			if (tick >= LAUNCH && tick <= BLAST)
				torpedoes.add(run(TORPEDO, 0, tick, LAUNCH, BLAST, launchX, 200, codex[0], codex[1] + 30));
			if (tick >= LAUNCH && tick <= STRAGGLER_BLAST) // the pair's second weapon, 35 m off the wreck
				torpedoes.add(run(STRAGGLER, 0, tick, LAUNCH, STRAGGLER_BLAST, launchX, 240, codex[0] + 35, codex[1]));
			if (mutual && tick >= COUNTER_LAUNCH && tick <= claudeDies)
				torpedoes.add(run(COUNTER, 1, tick, COUNTER_LAUNCH, claudeDies, codex[0], codex[1], claudeX(claudeDies),
						225));
			index.onTick(tick, subs, torpedoes);
		}
		index.onMatchEnd();
	}

	/** Claude cruises east along the ridge. */
	private static double claudeX(long tick) {
		return -3000 + 5 * tick / (double) RATE;
	}

	/** A torpedo running straight from launch to where it detonates. */
	private static TorpedoSnapshot run(
		int id, int owner, long tick, long launch, long blast, double x0, double y0, double x1, double y1) {
		double f = (tick - launch) / (double) (blast - launch);
		boolean detonated = tick == blast;
		return new TorpedoSnapshot(id, owner, Pose.at(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, -150, Math.PI / 2),
				Velocity.ZERO, 20, Color.CYAN, 100, detonated, !detonated, 150, false, Double.NaN, Double.NaN,
				Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
				Double.NaN, "");
	}

	private static SubmarineSnapshot submarine(
		int id, String name, double x, double y, double z, int hp, boolean ping) {
		return new SubmarineSnapshot(id, name, name.substring(0, 2), Pose.at(x, y, z, Math.PI / 2), Velocity.ZERO, 5,
				Color.CYAN, false, hp, 1, 100, 0.5, 0, 0, "", ping, 8, List.of(), List.of(), List.of(), null, false);
	}
}
