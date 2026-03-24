/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships;

import se.hirt.searobots.api.*;

/**
 * Simple reference torpedo controller. Runs toward the suspected target
 * position from mission data, then goes active sonar to acquire and chase.
 */
public class SimpleTorpedoController implements TorpedoController {

    private double targetX, targetY, targetZ;
    private boolean hasTarget;
    private boolean hasActiveContact; // true when we have a live sonar fix
    private double activeContactRange = Double.MAX_VALUE;
    private TerrainMap terrain;
    private long launchTick;
    private static final double MIN_TERRAIN_CLEARANCE = 30; // meters above seabed
    private static final double PING_START_RANGE = 1500; // start pinging at this range
    private static final double TERMINAL_RANGE = 300; // close enough to dive directly at target

    @Override
    public void onLaunch(TorpedoLaunchContext context) {
        this.terrain = context.terrain();
        // Parse mission data for target position
        // Expected format: "targetX,targetY,targetZ,heading,speed" or free text
        String data = context.missionData();
        if (data != null && !data.isBlank()) {
            try {
                var parts = data.split(",");
                if (parts.length >= 2) {
                    targetX = Double.parseDouble(parts[0].trim());
                    targetY = Double.parseDouble(parts[1].trim());
                    targetZ = parts.length >= 3 ? Double.parseDouble(parts[2].trim()) : -200;
                    hasTarget = true;
                }
            } catch (NumberFormatException e) {
                // Could not parse, just run straight
            }
        }
    }

    @Override
    public void onTick(TorpedoInput input, TorpedoOutput output) {
        output.setThrottle(1.0);

        var pos = input.self().position();
        double heading = input.self().heading();

        if (!hasTarget) {
            output.setRudder(0);
            output.setSternPlanes(0);
            return;
        }

        double dx = targetX - pos.x();
        double dy = targetY - pos.y();
        double dist = Math.sqrt(dx * dx + dy * dy);

        // ── Update target from sonar ──

        hasActiveContact = false;
        // Active returns: best fix, update target position
        if (!input.activeSonarReturns().isEmpty()) {
            var best = input.activeSonarReturns().stream()
                    .max(java.util.Comparator.comparingDouble(SonarContact::signalExcess))
                    .orElse(null);
            if (best != null && best.range() > 0) {
                double range = best.range();
                double bearing = best.bearing();
                targetX = pos.x() + Math.sin(bearing) * range;
                targetY = pos.y() + Math.cos(bearing) * range;
                activeContactRange = range;
                hasActiveContact = true;
                dx = targetX - pos.x();
                dy = targetY - pos.y();
                dist = Math.sqrt(dx * dx + dy * dy);
            }
        }
        // Passive: update bearing estimate (less reliable)
        if (!hasActiveContact && !input.sonarContacts().isEmpty()) {
            var loudest = input.sonarContacts().stream()
                    .max(java.util.Comparator.comparingDouble(SonarContact::signalExcess))
                    .orElse(null);
            if (loudest != null) {
                double bearing = loudest.bearing();
                // Don't override range, just update direction
                targetX = pos.x() + Math.sin(bearing) * Math.max(dist, 500);
                targetY = pos.y() + Math.cos(bearing) * Math.max(dist, 500);
                dx = targetX - pos.x();
                dy = targetY - pos.y();
                dist = Math.sqrt(dx * dx + dy * dy);
            }
        }

        // ── Pinging: start when approaching suspected target area ──
        if (dist < PING_START_RANGE && input.activeSonarCooldownTicks() == 0) {
            output.activeSonarPing();
        }

        // ── Steering: always head toward target ──
        double bearingToTarget = Math.atan2(dx, dy);
        if (bearingToTarget < 0) bearingToTarget += 2 * Math.PI;
        double headingError = bearingToTarget - heading;
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        output.setRudder(Math.clamp(headingError * 2.0, -1, 1));

        // ── Depth control ──
        double desiredZ = pos.z(); // default: hold current depth

        if (hasActiveContact && activeContactRange < TERMINAL_RANGE) {
            // Terminal phase: dive directly at the target depth
            // Target is close, go for it aggressively
            desiredZ = targetZ;
        } else if (hasActiveContact) {
            // Active contact but not terminal: blend toward target depth
            desiredZ = pos.z() + (targetZ - pos.z()) * 0.05;
        } else if (!Double.isNaN(targetZ) && targetZ < 0) {
            // No active contact: approach mission data depth at moderate rate
            desiredZ = pos.z() + (targetZ - pos.z()) * 0.03;
        }

        // Terrain avoidance (go UP only, unless in terminal phase)
        if (terrain != null && !(hasActiveContact && activeContactRange < TERMINAL_RANGE)) {
            double floor = terrain.elevationAt(pos.x(), pos.y());
            double safeZ = floor + MIN_TERRAIN_CLEARANCE;
            double lookAhead = input.speed() * 3;
            double aheadFloor = terrain.elevationAt(
                    pos.x() + Math.sin(heading) * lookAhead,
                    pos.y() + Math.cos(heading) * lookAhead);
            safeZ = Math.max(safeZ, aheadFloor + MIN_TERRAIN_CLEARANCE + 10);
            if (desiredZ < safeZ) desiredZ = safeZ;
        }

        desiredZ = Math.min(desiredZ, -5); // don't breach surface

        double dz = desiredZ - pos.z();
        // Terminal phase gets more pitch authority
        double pitchGain = (hasActiveContact && activeContactRange < TERMINAL_RANGE) ? 0.06 : 0.03;
        double pitchMax = (hasActiveContact && activeContactRange < TERMINAL_RANGE) ? 0.6 : 0.4;
        output.setSternPlanes(Math.clamp(dz * pitchGain, -pitchMax, pitchMax));
    }
}
