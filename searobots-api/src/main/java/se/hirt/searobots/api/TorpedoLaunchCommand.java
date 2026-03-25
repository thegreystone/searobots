/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.api;

/**
 * Command issued by a submarine controller to launch a torpedo.
 * This is the only data the torpedo receives from the submarine --
 * after launch, there is no further communication.
 *
 * <p>Torpedoes are ejected from fixed forward tubes. In the current
 * implementation the torpedo leaves the submarine in the submarine's
 * current heading and pitch; the command does not rotate the tube or
 * change the initial ejection vector.
 *
 * @param bearing     requested shot bearing in radians (0 = north, clockwise)
 * @param pitch       requested shot pitch in radians (negative = downward)
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
