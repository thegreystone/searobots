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

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/**
 * Records a full-fidelity {@code .srl} replay of a headless combat match (no rendering), for feeding to the viewer's
 * replay playback. Runs two torpedo-firing controllers (Claude vs Codex) at maximum speed so the log exercises the
 * interesting cases: contacts, firing solutions, torpedo launches and detonations.
 * <p>
 * Usage: {@code HeadlessRecorder [seed] [durationTicks] [outputPath]}. Defaults: seed 13 (a known fast engagement),
 * {@value #DEFAULT_DURATION} ticks, and {@code replays/headless-<seedHex>.srl}. The match also ends early once a
 * submarine is destroyed.
 */
public final class HeadlessRecorder {

	private static final int DEFAULT_DURATION = 30_000; // ~10 min at 50 Hz (enough for an engagement)

	private HeadlessRecorder() {
	}

	public static void main(String[] args) throws IOException {
		long seed = args.length > 0 ? Long.parseLong(args[0]) : 13L;
		int duration = args.length > 1 ? Integer.parseInt(args[1]) : DEFAULT_DURATION;
		Path out =
				args.length > 2 ? Path.of(args[2]) : Path.of("replays", "headless-" + Long.toHexString(seed) + ".srl");

		MatchConfig config = withDuration(MatchConfig.withDefaults(seed), duration);
		var world = new WorldGenerator().generate(config);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9); // no real-time pacing; run as fast as the CPU allows

		// Codex (id 0) vs Claude (id 1): the pairing/spawn order FastestKillScan finds reliable engagements with.
		List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
		List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		long[] frames = {0};
		boolean[] sawFiring = {false};
		boolean[] sawTorpedo = {false};
		var monitor = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				frames[0]++;
				for (var s : subs) {
					if (s.firingSolution() != null) {
						sawFiring[0] = true;
					}
				}
				if (torps != null && !torps.isEmpty()) {
					sawTorpedo[0] = true;
				}
				boolean anyDead = subs.stream().anyMatch(s -> s.hp() <= 0);
				if (anyDead) {
					sim.stop(); // no point recording the post-kill drift
				}
			}

			@Override
			public void onMatchEnd() {
			}
		};

		System.out.printf("Recording headless match: seed=%s (%s), up to %d ticks...%n", seed, Long.toHexString(seed),
				duration);
		long t0 = System.currentTimeMillis();
		try (var writer = new ReplayWriter(config, world.spawnPoints(), out)) {
			sim.run(world, controllers, vehicles, SimulationListeners.composite(monitor, writer));
		}
		System.out.printf("Recorded %d frames in %d ms to %s%n", frames[0], System.currentTimeMillis() - t0,
				out.toAbsolutePath());
		System.out.printf("  firingSolution seen=%b, torpedo seen=%b%n", sawFiring[0], sawTorpedo[0]);
		System.out.printf("Replay it with: SubmarineScene3D %s%n", out);
	}

	private static MatchConfig withDuration(MatchConfig b, int durationTicks) {
		return new MatchConfig(b.worldSeed(), b.tickRateHz(), durationTicks, b.submarineCount(), b.torpedoCount(),
				b.startingHp(), b.blastRadius(), b.minFuseRadius(), b.maxFuseRadius(), b.ratedDepth(), b.crushDepth(),
				b.battleArea(), b.terrainMarginMeters(), b.gridCellMeters(), b.minSeaFloorZ(), b.maxSeaFloorZ(),
				b.maxSubSpeed(), b.startTime());
	}
}
