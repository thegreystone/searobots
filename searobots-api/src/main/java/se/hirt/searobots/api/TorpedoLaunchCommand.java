/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.api;

/**
 * Command issued by a submarine controller to launch a torpedo.
 * This is the only data the torpedo receives from the submarine --
 * after launch, there is no further communication.
 *
 * @param bearing     launch bearing in radians (0 = north, clockwise)
 * @param pitch       launch pitch in radians (negative = downward)
 * @param fuseRadius  proximity fuse trigger distance in meters
 *                    (clamped to MatchConfig min/max)
 * @param missionData free-form string with tactical context for the
 *                    torpedo controller: suspected target position,
 *                    heading, speed, engagement strategy, etc.
 */
public record TorpedoLaunchCommand(
        double bearing,
        double pitch,
        double fuseRadius,
        String missionData
) {}
