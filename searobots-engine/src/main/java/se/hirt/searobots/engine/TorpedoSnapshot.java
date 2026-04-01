/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine;

import se.hirt.searobots.api.Pose;
import se.hirt.searobots.api.Velocity;

import java.awt.*;

/**
 * Immutable snapshot of a torpedo's state for viewer rendering and replay.
 */
public record TorpedoSnapshot(
        int id,
        int ownerId,
        Pose pose,
        Velocity velocity,
        double speed,
        Color color,
        double fuelRemaining,
        boolean detonated,
        boolean alive,
        double sourceLevelDb,
        boolean pingRequested,
        double targetX,
        double targetY,
        double targetZ
) {}
