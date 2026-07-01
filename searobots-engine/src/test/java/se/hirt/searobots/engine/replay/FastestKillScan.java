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

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfSystemProperty;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Development scan (not an assertion-heavy unit test): runs several seeds with the torpedo-firing combat controllers
 * (Codex vs Claude) and reports which produces the first torpedo detonation soonest. Each match is recorded with
 * {@link ReplayWriter}, so it also confirms torpedoes are captured in the replay, and it leaves behind the fastest
 * seed's replay file for inspection.
 * <p>
 * Assertion-free dev tool: skipped on normal builds, run on demand with
 * {@code mvn test -Dsearobots.diag=true -Dtest=FastestKillScan}.
 */
@EnabledIfSystemProperty(named = "searobots.diag", matches = "true")
class FastestKillScan {

	private static final long[] SEEDS = {1, 2, 3, 7, 11, 13, 42, 99};
	private static final int MAX_TICKS = 30_000; // ~10 min sim cap per seed

	private record Result(long seed, long firstHitTick, boolean torpedoSeen, Path file) {
	}

	@Test
	void findFastestTorpedoHit(@TempDir Path dir) throws IOException {
		List<Result> results = new ArrayList<>();
		for (long seed : SEEDS) {
			results.add(runSeed(seed, dir));
		}

		results.sort((a, b) -> Long.compare(a.firstHitTick() < 0 ? Long.MAX_VALUE : a.firstHitTick(),
				b.firstHitTick() < 0 ? Long.MAX_VALUE : b.firstHitTick()));

		System.out.println("=== Fastest torpedo hit by seed ===");
		for (Result r : results) {
			String when = r.firstHitTick() < 0 ? "no hit within " + MAX_TICKS + " ticks"
					: String.format("tick %d (%.1fs)", r.firstHitTick(), r.firstHitTick() / 50.0);
			System.out.printf("  seed %-4d  %-30s  torpedoCaptured=%s%n", r.seed(), when, r.torpedoSeen());
		}
		Result best = results.get(0);
		if (best.firstHitTick() >= 0) {
			System.out.printf("FASTEST: seed %d at tick %d (%.1fs), replay: %s%n", best.seed(), best.firstHitTick(),
					best.firstHitTick() / 50.0, best.file());
		}
	}

	private Result runSeed(long seed, Path dir) throws IOException {
		MatchConfig base = MatchConfig.withDefaults(seed);
		MatchConfig config = new MatchConfig(base.worldSeed(), base.tickRateHz(), MAX_TICKS, base.submarineCount(),
				base.torpedoCount(), base.startingHp(), base.blastRadius(), base.minFuseRadius(), base.maxFuseRadius(),
				base.ratedDepth(), base.crushDepth(), base.battleArea(), base.terrainMarginMeters(),
				base.gridCellMeters(), base.minSeaFloorZ(), base.maxSeaFloorZ(), base.maxSubSpeed(), base.startTime());

		var world = new WorldGenerator().generate(config);
		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);

		List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
		List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		Path file = dir.resolve("scan-" + seed + ".srl");
		long[] firstHit = {-1};
		boolean[] torpedoSeen = {false};

		var detector = new SimulationListener() {
			@Override
			public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
				if (!torps.isEmpty()) {
					torpedoSeen[0] = true;
				}
				if (firstHit[0] < 0 && torps.stream().anyMatch(TorpedoSnapshot::detonated)) {
					firstHit[0] = tick; // first torpedo detonation
					sim.stop();         // early-exit this seed once it lands
				}
			}

			@Override
			public void onMatchEnd() {
			}
		};

		try (var writer = new ReplayWriter(config, world.spawnPoints(), file)) {
			sim.run(world, controllers, vehicles, SimulationListeners.composite(detector, writer));
		}
		return new Result(seed, firstHit[0], torpedoSeen[0], file);
	}
}
