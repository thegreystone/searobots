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
package se.hirt.searobots.engine;

import se.hirt.searobots.api.PathPlanner;
import se.hirt.searobots.api.PathPlanner.CorridorPoint;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;

import java.util.ArrayList;
import java.util.List;

/**
 * Projects a submarine's trajectory forward from its kinematic state,
 * placing waypoints that are each physically reachable from the previous
 * one given the sub's turn radius and depth change rate.
 *
 * <p>Works with a 2D corridor from the A* planner: the corridor defines
 * which zones of the map are safe and deep, and the projector generates
 * a 3D route through that corridor that respects the sub's momentum.
 */
final class TrajectoryProjector {

    private static final double FLOOR_CLEARANCE = 75.0;
    private static final double MIN_DEPTH = -20.0;
    private static final double SHALLOW_WARNING = -60.0; // flag legs that force the sub this shallow
    private static final double ARC_STEP = 50.0; // meters per arc sample

    private final PathPlanner pathPlanner;
    private final TerrainMap terrain;

    TrajectoryProjector(PathPlanner pathPlanner, TerrainMap terrain) {
        this.pathPlanner = pathPlanner;
        this.terrain = terrain;
    }

    /**
     * Projects a route from the sub's current kinematic state through the
     * corridor to the goal. Returns 3D waypoints that are each physically
     * reachable from the previous one.
     *
     * @param x               current position X
     * @param y               current position Y
     * @param z               current depth
     * @param heading         current heading (radians)
     * @param speed           current speed (m/s, used to determine turn radius)
     * @param corridor        2D corridor waypoints from A*
     * @param preferredDepth  strategic depth preference (NaN = as deep as possible)
     * @param expectedSpeed   expected cruising speed for envelope computation
     * @return list of 3D nav waypoints
     */
    List<Vec3> projectRoute(double x, double y, double z, double heading,
                             double speed, List<CorridorPoint> corridor,
                             double preferredDepth, double expectedSpeed) {
        if (corridor.isEmpty()) return List.of();

        var result = new ArrayList<Vec3>();
        double curX = x, curY = y, curZ = z, curHeading = heading;

        // Use at least 2 m/s for envelope computation (handles speed=0 at spawn)
        double planSpeed = Math.max(speed, 2.0);
        double turnRadius = SubmarineAutopilot.turnRadiusAtSpeed(planSpeed);
        double depthRatio = SubmarineAutopilot.depthChangeRatio(expectedSpeed);

        // Skip corridor points that are behind the sub or too close
        int corridorIdx = findFirstRelevantCorridorPoint(curX, curY, curHeading, corridor);

        for (int ci = corridorIdx; ci < corridor.size(); ci++) {
            var target = corridor.get(ci);
            double dx = target.x() - curX;
            double dy = target.y() - curY;
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < 50) continue; // too close, skip

            double bearingToTarget = Math.atan2(dx, dy);
            double turnAngle = angleDiff(bearingToTarget, curHeading);
            double absTurn = Math.abs(turnAngle);

            if (absTurn < Math.toRadians(20)) {
                // Small turn: go directly to corridor point
                double wpZ = computeDepth(curX, curY, curZ, target.x(), target.y(),
                        dist, depthRatio, preferredDepth);
                result.add(new Vec3(target.x(), target.y(), wpZ));
                curX = target.x();
                curY = target.y();
                curZ = wpZ;
                curHeading = bearingToTarget;
            } else {
                // Significant turn: project an arc, place waypoint where
                // the sub "unwinds" and can head toward the corridor point.
                var arcWp = projectArcToward(curX, curY, curZ, curHeading,
                        target.x(), target.y(), turnRadius, depthRatio, preferredDepth);
                if (arcWp != null) {
                    result.add(arcWp);
                    curHeading = Math.atan2(target.x() - arcWp.x(), target.y() - arcWp.y());
                    curX = arcWp.x();
                    curY = arcWp.y();
                    curZ = arcWp.z();
                }

                // The arc may have moved us off the original corridor. Re-plan
                // a sub-corridor from our new position to the remaining goal
                // to avoid crossing terrain the original corridor skirted.
                if (arcWp != null && ci == corridor.size() - 1) {
                    var subCorridor = pathPlanner.findCorridor(curX, curY, target.x(), target.y());
                    if (subCorridor.size() > 1) {
                        // Skip the first point (near our position) and process the rest
                        for (int si = 1; si < subCorridor.size(); si++) {
                            var sc = subCorridor.get(si);
                            double sd = Math.sqrt(Math.pow(sc.x() - curX, 2)
                                    + Math.pow(sc.y() - curY, 2));
                            if (sd > 50) {
                                double wpZ = computeDepth(curX, curY, curZ, sc.x(), sc.y(),
                                        sd, depthRatio, preferredDepth);
                                result.add(new Vec3(sc.x(), sc.y(), wpZ));
                                curHeading = Math.atan2(sc.x() - curX, sc.y() - curY);
                                curX = sc.x();
                                curY = sc.y();
                                curZ = wpZ;
                            }
                        }
                        continue; // corridor goal already added via sub-corridor
                    }
                }

                // Add the corridor point itself
                double d2 = Math.sqrt(Math.pow(target.x() - curX, 2)
                        + Math.pow(target.y() - curY, 2));
                if (d2 > 50) {
                    double wpZ = computeDepth(curX, curY, curZ, target.x(), target.y(),
                            d2, depthRatio, preferredDepth);
                    result.add(new Vec3(target.x(), target.y(), wpZ));
                    curX = target.x();
                    curY = target.y();
                    curZ = wpZ;
                    curHeading = bearingToTarget;
                }
            }
        }

        return result;
    }

    /**
     * Projects an arc from the current state toward a target point.
     * Returns a waypoint at the point where the arc completes and the
     * sub can head straight toward the target, or null if no arc is needed.
     */
    private Vec3 projectArcToward(double fromX, double fromY, double fromZ,
                                   double heading, double targetX, double targetY,
                                   double turnRadius, double depthRatio,
                                   double preferredDepth) {
        double dx = targetX - fromX;
        double dy = targetY - fromY;
        double bearingToTarget = Math.atan2(dx, dy);
        double turnAngle = angleDiff(bearingToTarget, heading);
        double turnDir = Math.signum(turnAngle);

        // Center of the turn circle: offset perpendicular to heading
        double perpAngle = heading + turnDir * Math.PI / 2;
        double cx = fromX + Math.sin(perpAngle) * turnRadius;
        double cy = fromY + Math.cos(perpAngle) * turnRadius;

        // Step along the arc until the bearing to target is small
        double arcLength = 0;
        double arcAngle = 0;
        double maxArc = Math.min(Math.abs(turnAngle) * 1.2, Math.PI) * turnRadius;
        double bestX = fromX, bestY = fromY;
        double bestAngleToTarget = Math.abs(turnAngle);

        while (arcLength < maxArc) {
            arcLength += ARC_STEP;
            arcAngle = arcLength / turnRadius;

            // Position on the arc: rotate from the start point around the center
            double startAngle = Math.atan2(fromX - cx, fromY - cy);
            double curAngle = startAngle + turnDir * arcAngle;
            double arcX = cx + Math.sin(curAngle) * turnRadius;
            double arcY = cy + Math.cos(curAngle) * turnRadius;

            // Check safety
            if (pathPlanner != null && !pathPlanner.isSafe(arcX, arcY)) {
                break; // arc enters unsafe terrain, stop here
            }

            double arcHeading = heading + turnDir * arcAngle;
            double bearingFromArc = Math.atan2(targetX - arcX, targetY - arcY);
            double remainingTurn = Math.abs(angleDiff(bearingFromArc, arcHeading));

            if (remainingTurn < bestAngleToTarget) {
                bestAngleToTarget = remainingTurn;
                bestX = arcX;
                bestY = arcY;
            }

            if (remainingTurn < Math.toRadians(20)) {
                // Arc has unwound enough to head toward target
                bestX = arcX;
                bestY = arcY;
                break;
            }
        }

        // Only return an arc waypoint if it's meaningfully different from start
        double movedDist = Math.sqrt(Math.pow(bestX - fromX, 2) + Math.pow(bestY - fromY, 2));
        if (movedDist < 50) return null;

        double wpZ = computeDepth(fromX, fromY, fromZ, bestX, bestY,
                movedDist, depthRatio, preferredDepth);
        return new Vec3(bestX, bestY, wpZ);
    }

    /**
     * Computes the achievable depth for a waypoint, given the sub's current
     * depth, horizontal distance, depth change ratio, and preferred depth.
     * Scans terrain along the leg to ensure clearance.
     */
    private double computeDepth(double fromX, double fromY, double fromZ,
                                 double toX, double toY, double dist,
                                 double depthRatio, double preferredDepth) {
        // Scan terrain along the leg
        double worstFloor = terrain.elevationAt(toX, toY);
        int samples = Math.max(3, (int) (dist / 75));
        for (int s = 0; s <= samples; s++) {
            double t = (double) s / samples;
            double sx = fromX + (toX - fromX) * t;
            double sy = fromY + (toY - fromY) * t;
            double floor = terrain.elevationAt(sx, sy);
            if (floor > worstFloor) worstFloor = floor;
        }

        // Target depth: as deep as possible, but clear the terrain
        double safeFloorDepth = worstFloor + FLOOR_CLEARANCE;
        double targetZ;
        if (Double.isNaN(preferredDepth)) {
            // Go as deep as the terrain allows
            targetZ = safeFloorDepth;
        } else {
            targetZ = Math.max(preferredDepth, safeFloorDepth);
        }
        targetZ = Math.min(targetZ, MIN_DEPTH); // don't go above -20m

        // Clamp by achievable depth change
        double maxChange = dist * depthRatio;
        double clampedZ = Math.max(targetZ, fromZ - maxChange); // can't dive faster
        clampedZ = Math.min(clampedZ, fromZ + maxChange);       // can't rise faster

        return clampedZ;
    }

    /**
     * Finds the first corridor point that is ahead of the sub (not behind it).
     */
    private int findFirstRelevantCorridorPoint(double x, double y, double heading,
                                                List<CorridorPoint> corridor) {
        for (int i = 0; i < corridor.size(); i++) {
            var cp = corridor.get(i);
            double dx = cp.x() - x;
            double dy = cp.y() - y;
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < 75) continue; // too close, skip

            // Check if it's roughly ahead (within 120 degrees of heading)
            double bearing = Math.atan2(dx, dy);
            double angle = Math.abs(angleDiff(bearing, heading));
            if (angle < Math.toRadians(120)) return i;
        }
        // All points behind or too close: start from the last one
        return Math.max(0, corridor.size() - 1);
    }

    private static double angleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
