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
package se.hirt.searobots.api;

/**
 * Configuration for a match. All fields are set before match start and
 * communicated to participants via {@code MatchContext}.
 *
 * @param worldSeed           seed for procedural world generation
 * @param tickRateHz          simulation ticks per second
 * @param matchDurationTicks  maximum ticks before the match is a draw
 * @param submarineCount      number of submarines in the match
 * @param torpedoCount        torpedoes available per submarine
 * @param startingHp          hit points per submarine at match start
 * @param blastRadius         torpedo explosion radius (metres)
 * @param minFuseRadius       lower bound on configurable fuse radius (metres)
 * @param maxFuseRadius       upper bound on configurable fuse radius (metres)
 * @param ratedDepth          submarine's rated max operating depth (negative Z);
 *                            below this, hull stress damage and implosion risk begin
 * @param crushDepth          absolute crush depth (negative Z): instant destruction
 * @param battleArea           shape and size of the battle zone
 * @param terrainMarginMeters extra terrain generated beyond the battle area
 * @param gridCellMeters      heightmap resolution (metres per cell)
 * @param minSeaFloorZ        shallowest sea floor elevation (e.g. -30)
 * @param maxSeaFloorZ        deepest sea floor elevation (e.g. -900)
 * @param maxSubSpeed         maximum submarine speed in m/s (derived from
 *                            thrust and drag parameters in the physics model)
 */
public record MatchConfig(
        long worldSeed,
        int tickRateHz,
        int matchDurationTicks,
        int submarineCount,
        int torpedoCount,
        int startingHp,
        double blastRadius,
        double minFuseRadius,
        double maxFuseRadius,
        double ratedDepth,
        double crushDepth,
        BattleArea battleArea,
        double terrainMarginMeters,
        double gridCellMeters,
        double minSeaFloorZ,
        double maxSeaFloorZ,
        double maxSubSpeed) {

    public static MatchConfig withDefaults(long worldSeed) {
        return new MatchConfig(
                worldSeed,
                50,
                300_000,
                2,
                4,
                1000,
                50.0,
                5.0,
                30.0,
                -400.0,   // rated depth: hull stress begins below this
                -700.0,   // crush depth: guaranteed destruction
                new BattleArea.Circular(7000.0),
                1500.0,
                10.0,
                -30.0,
                -1000.0,  // terrain can go well below crush depth
                15.0);    // max sub speed: sqrt(MAX_THRUST / DRAG_COEFF)
    }
}
