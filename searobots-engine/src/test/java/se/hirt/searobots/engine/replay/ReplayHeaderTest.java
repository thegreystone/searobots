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
import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.MatchConfig;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests {@link ReplayHeader#toMatchConfig()}: header-captured parameters must override the defaults so a match
 * recorded with a non-default config regenerates the same world, while zero/null fields (older or partial files)
 * fall back to the defaults for the recorded seed.
 */
class ReplayHeaderTest {

	private static final long SEED = 0xCAFEBABEL;

	@Test
	void capturedFieldsOverrideDefaults() {
		var area = new BattleArea.Rectangular(3500, 2500);
		var header = new ReplayHeader(ReplayFormat.VERSION, SEED, 20, 9000, 750, -900, -600, area, List.of());

		MatchConfig config = header.toMatchConfig();

		assertEquals(SEED, config.worldSeed());
		assertEquals(20, config.tickRateHz());
		assertEquals(9000, config.matchDurationTicks());
		assertEquals(750, config.startingHp());
		assertEquals(-900, config.crushDepth());
		assertEquals(-600, config.ratedDepth());
		assertEquals(area, config.battleArea());
	}

	@Test
	void zeroAndNullFieldsFallBackToDefaults() {
		var header = new ReplayHeader(ReplayFormat.VERSION, SEED, 0, 0, 0, 0, 0, null, List.of());

		MatchConfig config = header.toMatchConfig();
		MatchConfig defaults = MatchConfig.withDefaults(SEED);

		assertEquals(defaults, config);
	}

	@Test
	void uncapturedParametersKeepDefaults() {
		var header = new ReplayHeader(ReplayFormat.VERSION, SEED, 20, 9000, 750, -900, -600,
				new BattleArea.Circular(4000), List.of());

		MatchConfig config = header.toMatchConfig();
		MatchConfig defaults = MatchConfig.withDefaults(SEED);

		assertEquals(defaults.submarineCount(), config.submarineCount());
		assertEquals(defaults.torpedoCount(), config.torpedoCount());
		assertEquals(defaults.blastRadius(), config.blastRadius());
		assertEquals(defaults.minFuseRadius(), config.minFuseRadius());
		assertEquals(defaults.maxFuseRadius(), config.maxFuseRadius());
		assertEquals(defaults.terrainMarginMeters(), config.terrainMarginMeters());
		assertEquals(defaults.gridCellMeters(), config.gridCellMeters());
		assertEquals(defaults.minSeaFloorZ(), config.minSeaFloorZ());
		assertEquals(defaults.maxSeaFloorZ(), config.maxSeaFloorZ());
		assertEquals(defaults.maxSubSpeed(), config.maxSubSpeed());
		assertEquals(defaults.startTime(), config.startTime());
	}

	@Test
	void withMatchDurationTicksChangesOnlyDuration() {
		MatchConfig base = MatchConfig.withDefaults(SEED);
		MatchConfig shortened = base.withMatchDurationTicks(1234);

		assertEquals(1234, shortened.matchDurationTicks());
		assertEquals(base, shortened.withMatchDurationTicks(base.matchDurationTicks()));
	}
}
