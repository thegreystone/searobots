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
 * @param launchHeading   actual tube-exit heading in radians
 *                        (0 = north, clockwise), taken from the
 *                        launching submarine at the moment of launch
 * @param launchPitch     actual tube-exit pitch in radians
 *                        (negative = downward), taken from the
 *                        launching submarine at the moment of launch
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
