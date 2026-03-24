/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.api;

/**
 * Context provided to a torpedo controller at launch time.
 * Contains everything the torpedo needs to begin its mission.
 *
 * @param config          match configuration (battle area, tick rate, etc.)
 * @param terrain         terrain map for path planning and depth awareness
 * @param launchPosition  world position where the torpedo was launched
 * @param launchHeading   initial heading in radians (0 = north, clockwise)
 * @param launchPitch     initial pitch in radians (negative = downward)
 * @param missionData     free-form tactical context from the launching sub
 */
public record TorpedoLaunchContext(
        MatchConfig config,
        TerrainMap terrain,
        Vec3 launchPosition,
        double launchHeading,
        double launchPitch,
        String missionData
) {}
