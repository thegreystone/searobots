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

import java.util.List;

/**
 * Match-level metadata parsed from a replay file's header lines.
 *
 * @param formatVersion
 * 		the format version from line 1 of the file
 * @param seed
 * 		world seed
 * @param tickRateHz
 * 		simulation tick rate
 * @param durationTicks
 * 		configured match duration in ticks
 * @param startingHp
 * 		starting hull points
 * @param crushDepth
 * 		crush depth (meters, negative)
 * @param ratedDepth
 * 		rated depth (meters, negative)
 * @param battleArea
 * 		arena shape
 * @param submarines
 * 		submarine definitions (id, name, color, spawn)
 */
public record ReplayHeader(int formatVersion, long seed, int tickRateHz, long durationTicks, int startingHp,
                           double crushDepth, double ratedDepth, BattleArea battleArea, List<SubDef> submarines) {

	/**
	 * A submarine's identity, fixed for the whole match.
	 *
	 * @param id
	 * 		engine id
	 * @param name
	 * 		display name
	 * @param colorRgb
	 * 		ARGB color (as from {@link java.awt.Color#getRGB()})
	 * @param spawnX
	 * 		spawn X
	 * @param spawnY
	 * 		spawn Y
	 * @param spawnZ
	 * 		spawn Z
	 */
	public record SubDef(int id, String name, int colorRgb, double spawnX, double spawnY, double spawnZ) {
	}

	/**
	 * Reconstructs the recorded match configuration: parameters captured in the header override the defaults for the
	 * recorded seed, so a match recorded with a non-default arena, duration, or depth limits regenerates the same world.
	 * Parameters the format does not capture (torpedo counts, terrain generation knobs, ...) keep their defaults, and
	 * hand-built terrain (worlds not produced by {@code WorldGenerator}) is not reproducible from a replay at all; see
	 * {@code docs/replay-format.md}. Zero-valued header fields (older or partial files) fall back to the defaults.
	 */
	public MatchConfig toMatchConfig() {
		MatchConfig d = MatchConfig.withDefaults(seed);
		return new MatchConfig(seed,
				tickRateHz > 0 ? tickRateHz : d.tickRateHz(),
				durationTicks > 0 ? (int) Math.min(durationTicks, Integer.MAX_VALUE) : d.matchDurationTicks(),
				d.submarineCount(), d.torpedoCount(),
				startingHp > 0 ? startingHp : d.startingHp(),
				d.blastRadius(), d.minFuseRadius(), d.maxFuseRadius(),
				ratedDepth != 0 ? ratedDepth : d.ratedDepth(),
				crushDepth != 0 ? crushDepth : d.crushDepth(),
				battleArea != null ? battleArea : d.battleArea(),
				d.terrainMarginMeters(), d.gridCellMeters(), d.minSeaFloorZ(), d.maxSeaFloorZ(), d.maxSubSpeed(),
				d.startTime());
	}
}
