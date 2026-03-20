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

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Autopilot layer that translates strategic waypoints into physical
 * control inputs (rudder, stern planes, throttle, ballast). Handles
 * A* route planning, steering, depth control, terrain avoidance,
 * movement pattern execution, and emergency recovery.
 *
 * <p>The strategic layer sets waypoints via {@link #setWaypoints};
 * the autopilot plans A* routes immediately and steers the sub
 * through them on each {@link #tick}.
 */
public final class SubmarineAutopilot {

    // Terrain / depth constants (mirrored from DefaultAttackSub)
    private static final double MIN_DEPTH = -20;
    private static final double FLOOR_CLEARANCE = 50;
    private static final double CRUSH_SAFETY_MARGIN = 50;
    private static final double BOUNDARY_TURN_DIST = 700;
    private static final double EMERGENCY_GAP = 40;
    private static final double[] SCAN_DISTANCES = {50, 100, 200, 400, 600, 1000, 1500};
    private static final double SCAN_SIDE_ANGLE = 0.6;
    private static final double HULL_HALF_LENGTH = 32.5;

    // Path planner constants
    private static final double WAYPOINT_ARRIVAL_DIST = 200.0;
    private static final double SHALLOW_WATER_LIMIT = -90.0;
    private static final double IMPASSABLE_LIMIT = -25.0;

    // Optimal rudder clamp: just below stall angle for tightest turns.
    // stallAngle=25° / 45° max deflection = 0.556. Use 0.55 for margin.
    private static final double OPTIMAL_RUDDER = 0.55;

    // Turn radius table measured from physics simulation at optimal rudder (0.35).
    // Index = speed in m/s (0-15). Value = steady-state turn radius in meters.
    // Minimum radius ~403m at 7 m/s. Full rudder is slightly worse (stall).
    static final double[] TURN_RADIUS = {
            0, 9999, 648, 505, 444, 416, 404, 403, 409, 419, 433, 449, 467, 487, 508, 530
    };

    /** Steady-state turn radius at a given speed (interpolated from physics table). */
    static double turnRadiusAtSpeed(double speed) {
        if (speed < 1) return 9999;
        int idx = Math.min((int) speed, TURN_RADIUS.length - 1);
        if (idx >= TURN_RADIUS.length - 1) return TURN_RADIUS[TURN_RADIUS.length - 1];
        double frac = speed - idx;
        return TURN_RADIUS[idx] * (1 - frac) + TURN_RADIUS[idx + 1] * frac;
    }

    /** Maximum speed that can achieve a given turn radius (inverse table lookup). */
    static double maxSpeedForRadius(double radius) {
        // The minimum radius is ~403m at 7 m/s. For radii below that, return 7.
        // For radii above, find the highest speed whose radius <= target.
        if (radius >= TURN_RADIUS[TURN_RADIUS.length - 1]) return TURN_RADIUS.length - 1;
        // Search from high speed down
        for (int i = TURN_RADIUS.length - 1; i >= 2; i--) {
            if (TURN_RADIUS[i] <= radius) return i;
        }
        return 2; // minimum useful speed
    }

    /**
     * Maximum speed to make a turn between three consecutive points.
     * The turn radius required is: r = distance / (2 * sin(turnAngle/2)).
     */
    static double maxSpeedForTurn(double turnAngle, double legDistance) {
        if (Math.abs(turnAngle) < Math.toRadians(5)) return Double.MAX_VALUE;
        double requiredRadius = legDistance / (2.0 * Math.sin(Math.abs(turnAngle) / 2.0));
        return Math.max(2.0, maxSpeedForRadius(requiredRadius));
    }

    // Noise policy throttle ranges
    private static final double SILENT_THROTTLE_MIN = 0.10;
    private static final double SILENT_THROTTLE_MAX = 0.15;
    private static final double QUIET_THROTTLE_MIN = 0.25;
    private static final double QUIET_THROTTLE_MAX = 0.35;
    private static final double NORMAL_THROTTLE = 0.40;
    private static final double SPRINT_THROTTLE_MIN = 0.80;
    private static final double SPRINT_THROTTLE_MAX = 1.00;

    // Movement pattern constants
    private static final double ZIGZAG_ANGLE = Math.toRadians(30);
    private static final int ZIGZAG_LEG_DURATION = 750; // 15 seconds at 50Hz
    private static final int SPRINT_PHASE_DURATION = 750;  // 15s
    private static final int DRIFT_PHASE_DURATION = 1000;  // 20s

    // Cavitation constants
    private static final double BASE_CAVITATION_SPEED = 5.0;
    private static final double CAVITATION_DEPTH_FACTOR = 0.02;

    // Tailing
    private static final double TAILING_FLOOR_CLEARANCE = 150.0;

    // Match context
    private final TerrainMap terrain;
    private final PathPlanner pathPlanner;
    private final BattleArea battleArea;
    private final double depthLimit;
    private final double maxSubSpeed;
    private final List<ThermalLayer> thermalLayers;

    // Strategic waypoints (set by strategic layer)
    private List<StrategicWaypoint> strategicWaypoints = List.of();
    private int strategicWaypointIndex = 0;

    // A* route (planned from current strategic waypoint)
    private final List<Vec3> navWaypoints = new ArrayList<>();
    private int currentNavIndex = 0;

    // State
    private boolean arrived = false;
    private boolean blocked = false;
    private long patternStartTick = 0;
    private boolean emergencyActive = false;

    // Three-point turn state
    private boolean threePointTurnActive = false;
    private int threePointReverseIndex = -1; // nav index to reverse toward
    private double threePointSafeHeading = Double.NaN; // direction we want to exit
    private long threePointCooldownUntil = -1; // suppress re-triggering until this tick

    // Terrain avoidance caches
    private double cachedProximityBearing = Double.NaN;
    private long proximityCheckTick = -100;
    private double cachedEscapeBearing = Double.NaN;
    private long escapeCheckTick = -100;

    // Output from last tick (for inspection by strategic layer)
    private double lastRudder;
    private double lastSternPlanes;
    private double lastThrottle;
    private double lastBallast;
    private String lastStatus = "";

    public SubmarineAutopilot(MatchContext context) {
        this.terrain = context.terrain();
        this.battleArea = context.config().battleArea();
        this.depthLimit = context.config().crushDepth() + CRUSH_SAFETY_MARGIN;
        this.maxSubSpeed = context.config().maxSubSpeed();
        this.thermalLayers = context.thermalLayers();
        this.pathPlanner = new PathPlanner(context.terrain(), SHALLOW_WATER_LIMIT, 200, 75);
    }

    // ── Public API ──────────────────────────────────────────────────

    /**
     * Sets new strategic waypoints. Plans A* routes immediately
     * (not deferred to first tick).
     */
    public void setWaypoints(List<StrategicWaypoint> waypoints, double posX, double posY,
                              double posZ, double heading, double speed) {
        this.strategicWaypoints = List.copyOf(waypoints);
        this.strategicWaypointIndex = 0;
        this.arrived = false;
        this.blocked = false;
        this.emergencyActive = false;
        this.threePointTurnActive = false;
        this.threePointReverseIndex = -1;
        this.patternStartTick = 0;

        // Plan route to first waypoint immediately
        navWaypoints.clear();
        currentNavIndex = 0;
        if (!strategicWaypoints.isEmpty()) {
            var wp = strategicWaypoints.getFirst();
            planRouteToWaypoint(posX, posY, posZ, heading, speed, wp);
        }
    }

    /**
     * Main autopilot tick. Reads sub state, writes control outputs.
     */
    public void tick(SubmarineInput input, SubmarineOutput output) {
        var self = input.self();
        var pos = self.pose().position();
        double heading = self.pose().heading();
        double speed = self.velocity().speed();
        double depth = pos.z();
        long tick = input.tick();

        StrategicWaypoint currentStrategic = strategicWaypoints.isEmpty()
                ? null : strategicWaypoints.get(strategicWaypointIndex);

        // Check strategic waypoint arrival
        if (currentStrategic != null) {
            double dx = currentStrategic.x() - pos.x();
            double dy = currentStrategic.y() - pos.y();
            double distToStrategic = Math.sqrt(dx * dx + dy * dy);
            if (distToStrategic < currentStrategic.arrivalRadius()) {
                arrived = true;
            }
        }

        // ── 1. Horizontal steering ─────────────────────────────────
        double tacticalRudder = 0;

        // Follow A* nav waypoints
        if (!navWaypoints.isEmpty() && currentNavIndex < navWaypoints.size()) {
            var wp = navWaypoints.get(currentNavIndex);
            double wpDx = wp.x() - pos.x();
            double wpDy = wp.y() - pos.y();
            double wpDist = Math.sqrt(wpDx * wpDx + wpDy * wpDy);

            // Don't auto-advance past the reverse waypoint during a three-point turn
            // (the turn execution block handles its own arrival check)
            boolean isReverseWp = threePointTurnActive && currentNavIndex == threePointReverseIndex;
            if (!isReverseWp && wpDist < WAYPOINT_ARRIVAL_DIST
                    && currentNavIndex < navWaypoints.size() - 1) {
                currentNavIndex++;
                wp = navWaypoints.get(currentNavIndex);
            }

            tacticalRudder = steerToward(pos.x(), pos.y(), heading, wp.x(), wp.y());
        }

        // Apply movement pattern
        if (currentStrategic != null) {
            tacticalRudder = applyMovementPattern(tacticalRudder, heading, pos.x(), pos.y(),
                    currentStrategic, tick);
        }

        // ── 2. Throttle from noise policy ──────────────────────────
        double tacticalThrottle = currentStrategic != null
                ? noiseToThrottle(currentStrategic.noise(), currentStrategic.targetSpeed(), depth)
                : NORMAL_THROTTLE;

        // Modulate throttle for SPRINT_DRIFT pattern
        if (currentStrategic != null && currentStrategic.pattern() == MovementPattern.SPRINT_DRIFT) {
            long elapsed = tick - patternStartTick;
            int cycle = SPRINT_PHASE_DURATION + DRIFT_PHASE_DURATION;
            long phase = elapsed % cycle;
            if (phase >= SPRINT_PHASE_DURATION) {
                // Drift phase: go quiet
                tacticalThrottle = Math.max(QUIET_THROTTLE_MIN,
                        Math.min(tacticalThrottle * 0.4, QUIET_THROTTLE_MAX));
            }
        }

        // ── 2b. Turn-aware speed limiting ─────────────────────────
        // Look ahead at the next turn in the nav waypoints. If the turn
        // requires a tighter radius than the current speed allows, reduce
        // throttle so the sub slows down in time.
        if (!navWaypoints.isEmpty() && currentNavIndex < navWaypoints.size() - 1
                && !threePointTurnActive) {
            var wpCurr = navWaypoints.get(currentNavIndex);
            var wpNext = navWaypoints.get(currentNavIndex + 1);

            // Bearing from current waypoint to next
            double bearingToNext = Math.atan2(wpNext.x() - wpCurr.x(), wpNext.y() - wpCurr.y());
            // Bearing from sub to current waypoint
            double bearingToCurr = Math.atan2(wpCurr.x() - pos.x(), wpCurr.y() - pos.y());
            double turnAngle = angleDiff(bearingToNext, bearingToCurr);

            double legDist = Math.sqrt(Math.pow(wpNext.x() - wpCurr.x(), 2)
                    + Math.pow(wpNext.y() - wpCurr.y(), 2));
            double maxTurnSpeed = maxSpeedForTurn(turnAngle, legDist);

            if (maxTurnSpeed < maxSubSpeed) {
                // Cap throttle to achieve the required speed
                double speedThrottle = maxTurnSpeed / maxSubSpeed;
                tacticalThrottle = Math.min(tacticalThrottle, speedThrottle);
            }
        }

        // ── 3. Depth preference ────────────────────────────────────
        double tacticalDepthPreference = currentStrategic != null
                ? currentStrategic.preferredDepth() : Double.NaN;

        // ── 4-12. Safety pipeline ──────────────────────────────────
        double rudder = tacticalRudder;
        double sternPlanes = 0;
        double throttle = tacticalThrottle;
        double ballast = 0.5;
        String status = "";

        // Step 3b: Proactive terrain safety override
        // When following an A* route to a safe waypoint, the A* planner has
        // already routed around obstacles. Proactive heading-based scans
        // would fight the route (the heading temporarily points at terrain
        // the route curves around). Only activate when NOT on a safe route,
        // or when the sub has drifted into actually unsafe territory.
        // On a safe route when: we have nav waypoints, the next waypoint is
        // in safe water, and we're not in an active emergency. We don't require
        // the sub's current cell to be safe because the A* route may pass through
        // cells adjacent to high-cost terrain (the route is valid but the grid
        // cell's worstFloorNear includes nearby shallow features).
        boolean onSafeRoute = !navWaypoints.isEmpty()
                && currentNavIndex < navWaypoints.size()
                && pathPlanner != null
                && !emergencyActive
                && pathPlanner.isSafe(navWaypoints.get(currentNavIndex).x(),
                                       navWaypoints.get(currentNavIndex).y());

        // Step 3a: Imminent wall check (always active, even on safe routes).
        // If terrain is shallow within 400m along current heading and no
        // three-point turn is already active, plan one with visible waypoints.
        if (!threePointTurnActive && tick > threePointCooldownUntil) {
            // Trigger three-point turn when wall is close along heading.
            // Very close (< 150m): always trigger (no room to steer around).
            // Medium range (150-250m): only if waypoint requires sharp turn (> 60°).
            double firstWallDist = Double.MAX_VALUE;
            for (double checkDist : new double[]{50, 100, 150, 200, 300, 400}) {
                double fx = pos.x() + Math.sin(heading) * checkDist;
                double fy = pos.y() + Math.cos(heading) * checkDist;
                if (terrain.elevationAt(fx, fy) > SHALLOW_WATER_LIMIT) {
                    firstWallDist = checkDist;
                    break;
                }
            }
            if (firstWallDist <= 200 && speed > 2) {
                // Moving toward a close wall: always trigger
                planThreePointTurn(pos.x(), pos.y(), pos.z(), heading, speed);
            } else if (firstWallDist < Double.MAX_VALUE
                    && !navWaypoints.isEmpty() && currentNavIndex < navWaypoints.size()) {
                // Wall detected: trigger if waypoint requires a turn
                // or if the waypoint itself is in unsafe terrain (failed A* fallback)
                var wp = navWaypoints.get(currentNavIndex);
                boolean wpUnsafe = pathPlanner != null
                        && !pathPlanner.isSafe(wp.x(), wp.y());
                double wpBearing = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
                double turnAngle = Math.abs(angleDiff(wpBearing, heading));
                if (turnAngle > Math.toRadians(30) || wpUnsafe) {
                    planThreePointTurn(pos.x(), pos.y(), pos.z(), heading, speed);
                }
            }
        }

        // Execute three-point turn via waypoints
        if (threePointTurnActive && currentNavIndex < navWaypoints.size()) {
            boolean reversing = currentNavIndex == threePointReverseIndex;

            if (reversing) {
                // Three-point turn reverse: engine in reverse, but use remaining
                // forward momentum to steer bow toward exit direction.
                // Once speed goes negative, negate rudder to keep the same swing.
                double bowDiff = angleDiff(threePointSafeHeading, heading);
                throttle = -0.5;
                // Neutralize depth control: stern planes work opposite in reverse
                // and would push the sub to surface. Keep level and hold depth with ballast.
                sternPlanes = 0;
                ballast = 0.5;

                double surgeSpeed = self.surgeSpeed();
                if (surgeSpeed > 0.3) {
                    // Still coasting forward: steer bow toward safe heading
                    rudder = Math.clamp(bowDiff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                    status = "THREE-PT COAST";
                } else {
                    // Actually reversing: negate rudder to maintain bow swing
                    rudder = Math.clamp(-bowDiff * 2.0, -1, 1);
                    status = "THREE-PT REVERSE";
                }

                // Reverse until the A* planner can find a physically feasible route.
                // Check every 50 ticks to avoid expensive replanning every tick.
                if (tick % 50 == 0 && navWaypoints.size() > currentNavIndex + 1) {
                    var exitWp = navWaypoints.get(currentNavIndex + 1);
                    var route = planRoute(pos.x(), pos.y(), exitWp.x(), exitWp.y());
                    if (!route.isEmpty() && pathPlanner.isSafe(route.getFirst().x(), route.getFirst().y())) {
                        // Verify the first turn is physically achievable:
                        // the sub will be slow after reversing, so check at low speed
                        var firstWp = route.getFirst();
                        double bearingToFirst = Math.atan2(firstWp.x() - pos.x(), firstWp.y() - pos.y());
                        double firstTurnAngle = Math.abs(angleDiff(bearingToFirst, heading));
                        double distToFirst = Math.sqrt(Math.pow(firstWp.x() - pos.x(), 2)
                                + Math.pow(firstWp.y() - pos.y(), 2));
                        double requiredSpeed = maxSpeedForTurn(firstTurnAngle, distToFirst);

                        // Accept if the turn is feasible at a reasonable speed (sub will
                        // be slow after reversing, so even tight turns work at low speed)
                        if (requiredSpeed >= 2.0) {
                            currentNavIndex++;
                            while (navWaypoints.size() > currentNavIndex) {
                                navWaypoints.removeLast();
                            }
                            navWaypoints.addAll(route);
                        }
                    }
                }
            } else {
                // Forward leg: follow the A* route (it already validated safety).
                // Complete when nav waypoints are exhausted or we've moved past
                // the three-point turn area.
                status = "THREE-PT FORWARD";

                if (currentNavIndex >= navWaypoints.size() - 1) {
                    // Reached the end of the A* route: turn is done
                    threePointTurnActive = false;
                    threePointReverseIndex = -1;
                    threePointCooldownUntil = tick + 500; // 10s cooldown
                    if (!strategicWaypoints.isEmpty()
                            && strategicWaypointIndex < strategicWaypoints.size()) {
                        var wp = strategicWaypoints.get(strategicWaypointIndex);
                        planRouteToWaypoint(pos.x(), pos.y(), pos.z(),
                                heading, speed, wp);
                    }
                }
            }
        } else if (threePointTurnActive) {
            // Ran out of waypoints during turn: replan
            threePointTurnActive = false;
            threePointReverseIndex = -1;
        }

        boolean dangerAhead = false;
        if (!onSafeRoute) {
            // No safe route: full proactive scan
            for (double checkDist : new double[]{200, 500, 1000, 1500}) {
                double fx = pos.x() + Math.sin(heading) * checkDist;
                double fy = pos.y() + Math.cos(heading) * checkDist;
                if (worstFloorNear(fx, fy, 100) > SHALLOW_WATER_LIMIT) {
                    dangerAhead = true;
                    break;
                }
            }

            if (!dangerAhead && (tick - proximityCheckTick >= 10)) {
                proximityCheckTick = tick;
                double worstBearing = Double.NaN;
                double worstFloor = Double.NEGATIVE_INFINITY;
                for (int deg = 0; deg < 360; deg += 30) {
                    double brg = Math.toRadians(deg);
                    double f = terrain.elevationAt(
                            pos.x() + Math.sin(brg) * 400, pos.y() + Math.cos(brg) * 400);
                    if (f > worstFloor) {
                        worstFloor = f;
                        worstBearing = brg;
                    }
                }
                if (worstFloor > SHALLOW_WATER_LIMIT + 30 && !Double.isNaN(worstBearing)) {
                    cachedProximityBearing = normalizeBearing(worstBearing + Math.PI);
                } else {
                    cachedProximityBearing = Double.NaN;
                }
            }
            if (!dangerAhead && !Double.isNaN(cachedProximityBearing)) {
                dangerAhead = true;
                double diff = angleDiff(cachedProximityBearing, heading);
                if (Math.abs(diff) > Math.toRadians(10)) {
                    rudder = Math.clamp(diff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                    status = "TERRAIN PROXIMITY";
                }
                throttle = Math.max(throttle, 0.3);
            }
            if (dangerAhead) {
                if (tick - escapeCheckTick >= 10) {
                    escapeCheckTick = tick;
                    cachedEscapeBearing = findDeepWaterBearing(pos.x(), pos.y());
                }
                double escapeBearing = cachedEscapeBearing;
                if (!Double.isNaN(escapeBearing)) {
                    double diff = angleDiff(escapeBearing, heading);
                    if (Math.abs(diff) > Math.toRadians(20)) {
                        rudder = Math.clamp(diff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                        status = "DANGER AHEAD";
                    }
                    throttle = Math.max(throttle, 0.3);

                    if (!navWaypoints.isEmpty() && currentNavIndex < navWaypoints.size()) {
                        var wp = navWaypoints.get(currentNavIndex);
                        if (worstFloorNear(wp.x(), wp.y(), 150) > SHALLOW_WATER_LIMIT) {
                            double escDist = 800;
                            double escX = pos.x() + Math.sin(escapeBearing) * escDist;
                            double escY = pos.y() + Math.cos(escapeBearing) * escDist;
                            if (terrain.elevationAt(escX, escY) < SHALLOW_WATER_LIMIT) {
                                navWaypoints.clear();
                                navWaypoints.add(new Vec3(escX, escY, safeDepthAt(escX, escY)));
                                currentNavIndex = 0;
                            }
                        }
                    }
                }
            }
        }

        // Floor clearance
        double floorClearance = FLOOR_CLEARANCE;

        // Step 4: Compute target depth
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        double floorCenter = terrain.elevationAt(pos.x(), pos.y());
        double floorBow = terrain.elevationAt(
                pos.x() + sinH * HULL_HALF_LENGTH, pos.y() + cosH * HULL_HALF_LENGTH);
        double floorStern = terrain.elevationAt(
                pos.x() - sinH * HULL_HALF_LENGTH, pos.y() - cosH * HULL_HALF_LENGTH);
        double floorBelow = Math.max(floorCenter, Math.max(floorBow, floorStern));
        double safetyFloor = clampTarget(floorBelow + floorClearance);

        double rawTarget;
        if (!Double.isNaN(tacticalDepthPreference)) {
            rawTarget = clampTarget(tacticalDepthPreference);
        } else {
            rawTarget = safetyFloor;
        }
        if (rawTarget < safetyFloor) {
            rawTarget = safetyFloor;
        }

        // Step 5: Multi-point terrain scan
        // Always scan for depth targeting (the sub needs to know about rising
        // terrain ahead to adjust depth). But when on a safe A* route, don't
        // let the scan results override horizontal steering (Step 6 rudder).
        double scanHeading = heading;
        double waypointHeading = heading;
        if (!navWaypoints.isEmpty() && currentNavIndex < navWaypoints.size()) {
            var wp = navWaypoints.get(currentNavIndex);
            waypointHeading = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
        }

        double worstFloor = floorBelow;
        double worstDist = 0;
        boolean dropOffAhead = false;
        boolean shallowWaterAhead = false;

        for (double dist : SCAN_DISTANCES) {
            double sx = pos.x() + Math.sin(scanHeading) * dist;
            double sy = pos.y() + Math.cos(scanHeading) * dist;
            double floor = terrain.elevationAt(sx, sy);
            if (floor > worstFloor) {
                worstFloor = floor;
                worstDist = dist;
            }
            if (floor < floorBelow - 100) {
                dropOffAhead = true;
            }
            if (floor > SHALLOW_WATER_LIMIT) {
                shallowWaterAhead = true;
                if (floor > worstFloor) {
                    worstFloor = floor;
                    worstDist = dist;
                }
            }
        }

        double waypointWorstFloor = floorBelow;
        if (Math.abs(angleDiff(waypointHeading, scanHeading)) > Math.toRadians(15)) {
            for (double dist : SCAN_DISTANCES) {
                double sx = pos.x() + Math.sin(waypointHeading) * dist;
                double sy = pos.y() + Math.cos(waypointHeading) * dist;
                double floor = terrain.elevationAt(sx, sy);
                if (floor > waypointWorstFloor) {
                    waypointWorstFloor = floor;
                }
            }
        }

        // Step 6: Terrain avoidance
        double effectiveWorstFloor = Math.max(worstFloor, waypointWorstFloor);
        double worstTarget = clampTarget(effectiveWorstFloor + floorClearance);
        double avoidanceThreshold = 50;
        double margin = depth - (worstFloor + floorClearance);
        boolean inShallowWater = floorBelow > SHALLOW_WATER_LIMIT;

        if (margin < avoidanceThreshold || shallowWaterAhead || inShallowWater) {
            double urgency = Math.clamp(1.0 - margin / avoidanceThreshold, 0.0, 1.0);
            if (shallowWaterAhead) {
                urgency = Math.max(urgency, 0.6);
            }
            if (inShallowWater) {
                // Actually in shallow water: always escape regardless of route
                urgency = 1.0;
                double escapeBearing = findDeepWaterBearing(pos.x(), pos.y());
                if (!Double.isNaN(escapeBearing)) {
                    double diff = angleDiff(escapeBearing, heading);
                    rudder = Math.clamp(diff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                    throttle = 0.15;
                    status = "SHALLOW ESCAPE";
                }
            }
            if (!inShallowWater) {
                status = "AVOIDING TERRAIN";
            }

            // Adjust depth target for vertical safety
            if (worstTarget > rawTarget) {
                rawTarget = worstTarget;
            }

            // Throttle and rudder overrides: only when NOT on a safe route.
            // On a safe route, the A* waypoints guide the sub through the
            // corridor. Rudder overrides would fight the route and push the
            // sub off the safe path. Depth adjustments above still protect
            // against floor proximity.
            if (!onSafeRoute && !inShallowWater && !threePointTurnActive) {
                throttle = Math.max(0.15, throttle * (1.0 - urgency * 0.7));
            }

            double scanDist = Math.max(worstDist, SCAN_DISTANCES[0]);
            double leftH = heading - SCAN_SIDE_ANGLE;
            double rightH = heading + SCAN_SIDE_ANGLE;
            double floorLeft = terrain.elevationAt(
                    pos.x() + Math.sin(leftH) * scanDist,
                    pos.y() + Math.cos(leftH) * scanDist);
            double floorRight = terrain.elevationAt(
                    pos.x() + Math.sin(rightH) * scanDist,
                    pos.y() + Math.cos(rightH) * scanDist);

            if (!onSafeRoute && !inShallowWater && !threePointTurnActive) {
                double floorDiff = Math.abs(floorLeft - floorRight);
                if (floorDiff > 1.0 || worstFloor > floorBelow + 5) {
                    double avoidRudder;
                    if (floorDiff > 1.0) {
                        avoidRudder = floorLeft < floorRight ? -urgency : urgency;
                    } else {
                        avoidRudder = rudder >= 0 ? urgency : -urgency;
                    }
                    rudder = rudder * (1 - urgency) + avoidRudder * urgency;
                }
            }
        }

        double targetDepth = rawTarget;

        // Step 7: Drop-off slowdown (only when not on a safe route)
        if (dropOffAhead && !onSafeRoute && !threePointTurnActive) {
            throttle = Math.min(throttle, 0.3);
        }

        // Step 8: Cascade depth controller
        // The D-term damps vertical speed to prevent overshoot. Use a target
        // vertical speed proportional to depth error so the sub rises/dives
        // briskly for large errors and settles smoothly near the target.
        double immediateGap = depth - floorBelow;
        double depthError = depth - targetDepth;
        double verticalSpeed = self.velocity().linear().z();

        double gapFactor = Math.clamp(
                (immediateGap - EMERGENCY_GAP) / (200 - EMERGENCY_GAP), 0, 1);

        // Desired vertical speed: proportional to error, capped at ±2 m/s
        double desiredVSpeed = Math.clamp(-0.02 * depthError, -2.0, 2.0);
        double vSpeedError = verticalSpeed - desiredVSpeed;

        double depthPGain = 0.015;
        double depthDGain = 0.15;
        double desiredPitch = Math.clamp(
                -depthPGain * depthError - depthDGain * vSpeedError,
                -Math.toRadians(20), Math.toRadians(20));

        if (desiredPitch < 0) {
            desiredPitch *= gapFactor;
        }

        double currentPitch = self.pose().pitch();
        double pitchError = desiredPitch - currentPitch;
        // Skip depth control planes during three-point turn reverse (they work opposite)
        boolean tptReversing = threePointTurnActive && currentNavIndex == threePointReverseIndex;
        if (!tptReversing) {
            sternPlanes = Math.clamp(3.0 * pitchError, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
        }

        // Ballast: offset from neutral proportional to depth error,
        // damped by vertical speed error (not raw speed)
        // During three-point turn reverse, hold neutral to maintain depth.
        if (!tptReversing) {
            double trimRate = 0.003;
            double trimDamp = 0.03;
            double rawBallast = Math.clamp(0.5 - trimRate * depthError - trimDamp * vSpeedError, 0.0, 1.0);
            if (rawBallast < 0.5) {
                ballast = Math.clamp(0.5 - (0.5 - rawBallast) * gapFactor, 0.0, 1.0);
            } else {
                ballast = Math.clamp(rawBallast, 0.0, 1.0);
            }
        }

        // Step 9: Pull-up
        boolean emergencySurface = false;
        if (immediateGap < floorClearance * 1.5 && verticalSpeed < -0.5) {
            double pullUpUrgency = Math.clamp(
                    (floorClearance * 1.5 - immediateGap) / floorClearance, 0.2, 0.8);
            sternPlanes = Math.max(sternPlanes, pullUpUrgency);
            ballast = Math.max(ballast, 0.5 + pullUpUrgency * 0.3);
            if (!threePointTurnActive) throttle = Math.min(throttle, 0.3);
            status = "PULL UP";
        }

        // Step 9b: Three-point turn (legacy, for unsafe terrain only)
        // Skip when the new waypoint-based three-point turn is active.
        if (!threePointTurnActive
                && pathPlanner != null && !pathPlanner.isSafe(pos.x(), pos.y())
                && speed < 3 && !navWaypoints.isEmpty()
                && currentNavIndex < navWaypoints.size()) {
            var wp = navWaypoints.get(currentNavIndex);
            double wpBearing = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
            double diff = angleDiff(wpBearing, heading);
            if (Math.abs(diff) > Math.toRadians(60)) {
                throttle = -0.5;
                rudder = Math.clamp(diff * 2.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                status = "THREE-POINT TURN";
            }
        }

        // Step 10: Emergency surface
        // Use center floor for the emergency check (imminent danger at the sub's
        // actual position), not the hull-aware floor which looks 32.5m ahead and
        // triggers false emergencies when approaching terrain features.
        double emergencyGap = depth - floorCenter;
        if (floorCenter > -30 || (emergencyGap < EMERGENCY_GAP && emergencyGap >= 0)) {
            emergencySurface = true;
            emergencyActive = true;
            sternPlanes = 0.8;
            ballast = 1.0;

            double escapeBearing = findDeepWaterBearing(pos.x(), pos.y());

            if (depth > -5) {
                double escDir = Double.NaN;
                if (!navWaypoints.isEmpty() && currentNavIndex < navWaypoints.size()) {
                    var wp = navWaypoints.get(currentNavIndex);
                    if (pathPlanner == null || pathPlanner.isSafe(wp.x(), wp.y())) {
                        escDir = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
                    }
                }
                if (Double.isNaN(escDir) && !Double.isNaN(escapeBearing)) {
                    escDir = escapeBearing;
                }

                if (!Double.isNaN(escDir)) {
                    double diff = angleDiff(escDir, heading);

                    if (Math.abs(diff) > Math.toRadians(90) && speed < 3) {
                        throttle = -0.5;
                        rudder = Math.clamp(diff * 2.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                        status = "THREE-POINT TURN";
                    } else if (Math.abs(diff) > Math.toRadians(90) && speed > 3) {
                        throttle = -1.0;
                        rudder = Math.clamp(diff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                        status = "EMERGENCY BRAKE";
                    } else {
                        rudder = Math.clamp(diff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                        throttle = 0.4;
                    }
                }
            } else if (depth > -5) {
                throttle = 0.3;
            } else {
                throttle = -1.0;
                if (!Double.isNaN(escapeBearing)) {
                    double diff = angleDiff(escapeBearing, heading);
                    rudder = Math.clamp(diff * 3.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
                }
            }
            status = "EMERGENCY SURFACE";
            blocked = true;
        } else {
            emergencyActive = false;
        }

        // Step 11: Surface avoidance
        if (!emergencySurface && depth > MIN_DEPTH) {
            sternPlanes = -0.5;
            ballast = 0.2;
            status = "SURFACE AVOIDANCE";
        }

        // Step 12: Border avoidance
        double distToBoundary = battleArea.distanceToBoundary(pos.x(), pos.y());
        if (distToBoundary < BOUNDARY_TURN_DIST) {
            status = "BORDER AVOIDANCE";
            double towardCenterAngle = Math.atan2(-pos.x(), -pos.y());
            if (towardCenterAngle < 0) towardCenterAngle += 2 * Math.PI;

            double angleDiffVal = towardCenterAngle - heading;
            while (angleDiffVal > Math.PI) angleDiffVal -= 2 * Math.PI;
            while (angleDiffVal < -Math.PI) angleDiffVal += 2 * Math.PI;

            double urgency = 1.0 - distToBoundary / BOUNDARY_TURN_DIST;
            rudder = Math.clamp(angleDiffVal * 2.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER) * Math.max(urgency, 0.5);
        }

        // Output
        output.setRudder(rudder);
        output.setSternPlanes(sternPlanes);
        output.setThrottle(throttle);
        output.setBallast(ballast);

        lastRudder = rudder;
        lastSternPlanes = sternPlanes;
        lastThrottle = throttle;
        lastBallast = ballast;
        lastStatus = status;

        // Publish waypoints for visualization
        for (int i = 0; i < navWaypoints.size(); i++) {
            var wp = navWaypoints.get(i);
            boolean isReverse = threePointTurnActive && i == threePointReverseIndex;
            output.publishWaypoint(new Waypoint(wp.x(), wp.y(), wp.z(),
                    i == currentNavIndex, isReverse));
        }
    }

    /**
     * Returns true if the current strategic waypoint has been reached.
     */
    public boolean hasArrived() {
        return arrived;
    }

    /**
     * Returns true if the autopilot is blocked (emergency state, no path).
     */
    public boolean isBlocked() {
        return blocked;
    }

    /**
     * Returns the current strategic waypoint index.
     */
    public int currentWaypointIndex() {
        return strategicWaypointIndex;
    }

    /**
     * Advances to the next strategic waypoint. Plans a new A* route.
     */
    public void advanceWaypoint(double posX, double posY, double posZ,
                                double heading, double speed) {
        if (strategicWaypointIndex < strategicWaypoints.size() - 1) {
            strategicWaypointIndex++;
            arrived = false;
            blocked = false;
            var wp = strategicWaypoints.get(strategicWaypointIndex);
            navWaypoints.clear();
            currentNavIndex = 0;
            planRouteToWaypoint(posX, posY, posZ, heading, speed, wp);
        }
    }

    /**
     * Returns the list of strategic waypoints.
     */
    public List<StrategicWaypoint> strategicWaypoints() {
        return strategicWaypoints;
    }

    /**
     * Returns the A* nav waypoints for the current leg.
     */
    public List<Vec3> navWaypoints() {
        return List.copyOf(navWaypoints);
    }

    /**
     * Returns the current A* nav waypoint index.
     */
    public int currentNavIndex() {
        return currentNavIndex;
    }

    // Accessors for last tick outputs
    public double lastRudder() { return lastRudder; }
    public double lastSternPlanes() { return lastSternPlanes; }
    public double lastThrottle() { return lastThrottle; }
    public double lastBallast() { return lastBallast; }
    public String lastStatus() { return lastStatus; }

    // Package-private for testing
    PathPlanner pathPlanner() { return pathPlanner; }

    // ── Route planning ──────────────────────────────────────────────

    private void planRouteToWaypoint(double posX, double posY, double posZ,
                                      double heading, double speed,
                                      StrategicWaypoint target) {
        navWaypoints.clear();
        currentNavIndex = 0;

        // Marker for current position (use actual depth, not terrain-derived)
        navWaypoints.add(new Vec3(posX, posY, posZ));
        currentNavIndex = 1;

        var route = planRouteFromSub(posX, posY, heading, speed,
                target.x(), target.y());
        if (!route.isEmpty()) {
            navWaypoints.addAll(route);
        } else {
            // Direct fallback
            navWaypoints.add(new Vec3(target.x(), target.y(), safeDepthAt(target.x(), target.y())));
        }
    }

    /**
     * Plans a three-point turn when the sub faces a wall.
     * Creates two nav waypoints:
     * WP0 (reverse): 150m directly behind the sub (the sub reverses to here).
     * WP1 (exit): 500m along the safest escape heading from WP0.
     * The safe heading prefers perpendicular escape (least turning > 60°).
     */
    private void planThreePointTurn(double posX, double posY, double posZ,
                                     double heading, double speed) {
        // Find the best escape heading: safe for 500m, prefer least turning > 60°
        double safeHeading = Double.NaN;
        double bestScore = Double.MAX_VALUE;
        for (int deg = 0; deg < 360; deg += 10) {
            double brg = Math.toRadians(deg);
            double turnNeeded = Math.abs(angleDiff(brg, heading));
            if (turnNeeded < Math.toRadians(60)) continue;

            boolean safe = true;
            for (double d = 50; d <= 1000; d += 50) {
                double fx = posX + Math.sin(brg) * d;
                double fy = posY + Math.cos(brg) * d;
                if (terrain.elevationAt(fx, fy) > SHALLOW_WATER_LIMIT) {
                    safe = false;
                    break;
                }
            }
            if (safe && (Double.isNaN(safeHeading) || turnNeeded < bestScore)) {
                safeHeading = brg;
                bestScore = turnNeeded;
            }
        }
        if (Double.isNaN(safeHeading)) {
            safeHeading = normalizeBearing(heading + Math.PI);
        }

        // WP0: reverse waypoint, 250m behind and offset OPPOSITE the safe heading.
        // When the stern tracks toward this waypoint, it naturally swings the
        // bow toward the safe heading (like a car three-point turn).
        double revBack = 200;  // distance behind
        double revSide = 200;  // lateral offset opposite safe heading (aggressive with coast turning)
        double oppSafe = normalizeBearing(safeHeading + Math.PI);
        double revX = posX - Math.sin(heading) * revBack + Math.sin(oppSafe) * revSide;
        double revY = posY - Math.cos(heading) * revBack + Math.cos(oppSafe) * revSide;
        // Shorten if reverse point is in shallow water
        while (terrain.elevationAt(revX, revY) > SHALLOW_WATER_LIMIT && revBack > 80) {
            revBack -= 25;
            revX = posX - Math.sin(heading) * revBack + Math.sin(oppSafe) * revSide;
            revY = posY - Math.cos(heading) * revBack + Math.cos(oppSafe) * revSide;
        }

        // WP1: exit waypoint along safe heading from the sub's current position.
        // Walk outward and find the farthest point where the ENTIRE path is safe.
        double exitDist = 200;
        for (double d = 250; d <= 800; d += 50) {
            double tx = posX + Math.sin(safeHeading) * d;
            double ty = posY + Math.cos(safeHeading) * d;
            if (terrain.elevationAt(tx, ty) > SHALLOW_WATER_LIMIT) break;
            exitDist = d;
        }
        double exitX = posX + Math.sin(safeHeading) * exitDist;
        double exitY = posY + Math.cos(safeHeading) * exitDist;

        navWaypoints.clear();
        navWaypoints.add(new Vec3(revX, revY, posZ));                           // WP0: reverse
        navWaypoints.add(new Vec3(exitX, exitY, safeDepthAt(exitX, exitY)));    // WP1: exit
        currentNavIndex = 0;

        threePointTurnActive = true;
        threePointReverseIndex = 0;
        threePointSafeHeading = safeHeading;
    }

    List<Vec3> planRoute(double fromX, double fromY, double toX, double toY) {
        if (pathPlanner != null) {
            double operatingDepth = stealthDepthAt(fromX, fromY);
            var path = pathPlanner.findPath(fromX, fromY, toX, toY, operatingDepth);
            if (!path.isEmpty()) return path;
        }
        return List.of(new Vec3(toX, toY, safeDepthAt(toX, toY)));
    }

    private List<Vec3> planRouteFromSub(double posX, double posY, double heading,
                                         double speed, double goalX, double goalY) {
        var directRoute = new ArrayList<>(planRoute(posX, posY, goalX, goalY));
        if (directRoute.isEmpty()) return directRoute;

        // Strip grid-snapped start if too close
        if (directRoute.size() > 1) {
            var first = directRoute.getFirst();
            double d = Math.sqrt(Math.pow(first.x() - posX, 2) + Math.pow(first.y() - posY, 2));
            if (d < 150) directRoute.removeFirst();
        }

        // Check angle to first waypoint
        var firstWp = directRoute.getFirst();
        double wpBearing = Math.atan2(firstWp.x() - posX, firstWp.y() - posY);
        double turnAngle = wpBearing - heading;
        while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

        if (Math.abs(turnAngle) < Math.PI / 2) {
            return directRoute;
        }

        // Sharp turn: insert lead point
        double leadDist = Math.max(200, speed * 20);
        double goalDist = Math.sqrt(Math.pow(goalX - posX, 2) + Math.pow(goalY - posY, 2));
        leadDist = Math.min(leadDist, goalDist * 0.4);
        leadDist = Math.max(leadDist, 100);

        double leadX = posX + Math.sin(heading) * leadDist;
        double leadY = posY + Math.cos(heading) * leadDist;

        if (pathPlanner != null && !pathPlanner.isSafe(leadX, leadY)) {
            boolean found = false;
            for (double tryDist = leadDist * 0.5; tryDist >= 100; tryDist *= 0.5) {
                double tx = posX + Math.sin(heading) * tryDist;
                double ty = posY + Math.cos(heading) * tryDist;
                if (pathPlanner.isSafe(tx, ty)) {
                    leadX = tx;
                    leadY = ty;
                    found = true;
                    break;
                }
            }
            if (!found) {
                return directRoute;
            }
        }

        var result = new ArrayList<Vec3>();
        result.add(new Vec3(leadX, leadY, safeDepthAt(leadX, leadY)));

        var astarPath = planRoute(leadX, leadY, goalX, goalY);
        for (int i = 0; i < astarPath.size(); i++) {
            var wp = astarPath.get(i);
            if (i == 0) {
                double d = Math.sqrt(Math.pow(wp.x() - leadX, 2) + Math.pow(wp.y() - leadY, 2));
                if (d < 100) continue;
            }
            result.add(wp);
        }
        return result;
    }

    // ── Steering ────────────────────────────────────────────────────

    private double steerToward(double posX, double posY, double heading,
                                double targetX, double targetY) {
        double targetBearing = Math.atan2(targetX - posX, targetY - posY);
        if (targetBearing < 0) targetBearing += 2 * Math.PI;
        double diff = angleDiff(targetBearing, heading);
        return Math.clamp(diff * 2.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
    }

    // ── Noise policy to throttle ────────────────────────────────────

    private double noiseToThrottle(NoisePolicy policy, double targetSpeed, double depth) {
        double baseThrottle = switch (policy) {
            case SILENT -> (SILENT_THROTTLE_MIN + SILENT_THROTTLE_MAX) / 2;
            case QUIET -> (QUIET_THROTTLE_MIN + QUIET_THROTTLE_MAX) / 2;
            case NORMAL -> NORMAL_THROTTLE;
            case SPRINT -> (SPRINT_THROTTLE_MIN + SPRINT_THROTTLE_MAX) / 2;
        };

        // Target speed override
        if (targetSpeed > 0) {
            double speedThrottle = targetSpeed / maxSubSpeed;
            baseThrottle = switch (policy) {
                case SILENT -> Math.clamp(speedThrottle, SILENT_THROTTLE_MIN, SILENT_THROTTLE_MAX);
                case QUIET -> Math.clamp(speedThrottle, QUIET_THROTTLE_MIN, QUIET_THROTTLE_MAX);
                case NORMAL -> Math.clamp(speedThrottle, QUIET_THROTTLE_MIN, SPRINT_THROTTLE_MIN);
                case SPRINT -> Math.clamp(speedThrottle, SPRINT_THROTTLE_MIN, SPRINT_THROTTLE_MAX);
            };
        }

        // Depth modulation: reduce throttle at shallow depths for cavitation
        if (policy == NoisePolicy.SPRINT && depth > -100) {
            double cavitationFactor = Math.clamp((-depth - 20) / 80, 0.5, 1.0);
            baseThrottle *= cavitationFactor;
        }

        return baseThrottle;
    }

    // ── Movement patterns ───────────────────────────────────────────

    private double applyMovementPattern(double baseRudder, double heading,
                                         double posX, double posY,
                                         StrategicWaypoint wp, long tick) {
        return switch (wp.pattern()) {
            case DIRECT -> baseRudder;
            case ZIGZAG_TMA -> {
                long elapsed = tick - patternStartTick;
                long cycle = ZIGZAG_LEG_DURATION * 2L;
                boolean zigLeft = (elapsed % cycle) < ZIGZAG_LEG_DURATION;

                // Direct bearing to waypoint
                double directBearing = Math.atan2(wp.x() - posX, wp.y() - posY);
                if (directBearing < 0) directBearing += 2 * Math.PI;

                // Distance-adaptive angle reduction
                double dist = Math.sqrt(Math.pow(wp.x() - posX, 2) + Math.pow(wp.y() - posY, 2));
                double angleFactor = Math.clamp(dist / 3000, 0.3, 1.0);
                double offset = ZIGZAG_ANGLE * angleFactor * (zigLeft ? -1 : 1);
                double targetBearing = normalizeBearing(directBearing + offset);

                double diff = angleDiff(targetBearing, heading);
                yield Math.clamp(diff * 2.0, -OPTIMAL_RUDDER, OPTIMAL_RUDDER);
            }
            case SPRINT_DRIFT -> baseRudder; // throttle handled separately
        };
    }

    // ── Terrain utilities ───────────────────────────────────────────

    private double findDeepWaterBearing(double x, double y) {
        double bestBearing = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        double deepestBearing = Double.NaN;
        double deepestFloor = Double.MAX_VALUE;
        for (int deg = 0; deg < 360; deg += 10) {
            double brg = Math.toRadians(deg);
            for (double dist = 200; dist <= 3000; dist += 200) {
                double sx = x + Math.sin(brg) * dist;
                double sy = y + Math.cos(brg) * dist;
                double floor = terrain.elevationAt(sx, sy);
                if (floor > IMPASSABLE_LIMIT) {
                    break;
                }
                if (floor < SHALLOW_WATER_LIMIT) {
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestBearing = brg;
                    }
                    break;
                }
                if (floor < deepestFloor) {
                    deepestFloor = floor;
                    deepestBearing = brg;
                }
            }
        }
        return Double.isNaN(bestBearing) ? deepestBearing : bestBearing;
    }

    private double worstFloorNear(double x, double y, double radius) {
        double worst = terrain.elevationAt(x, y);
        for (int deg = 0; deg < 360; deg += 45) {
            double brg = Math.toRadians(deg);
            double floor = terrain.elevationAt(x + Math.sin(brg) * radius, y + Math.cos(brg) * radius);
            if (floor > worst) worst = floor;
        }
        return worst;
    }

    private double safeDepthAt(double x, double y) {
        double floor = terrain.elevationAt(x, y);
        double target = floor + FLOOR_CLEARANCE;
        return clampTarget(target);
    }

    private double stealthDepthAt(double x, double y) {
        double floor = terrain.elevationAt(x, y);
        double target = floor + FLOOR_CLEARANCE * 0.6;
        return clampTarget(target);
    }

    private double clampTarget(double target) {
        if (target < depthLimit) target = depthLimit;
        if (target > MIN_DEPTH) target = MIN_DEPTH;
        return target;
    }

    double preferredDepthBelowThermocline(double currentDepth) {
        if (thermalLayers.isEmpty()) return Double.NaN;
        double shallowest = thermalLayers.getFirst().depth();
        for (var layer : thermalLayers) {
            if (layer.depth() > shallowest) shallowest = layer.depth();
        }
        return shallowest - 30;
    }

    // ── Geometry ────────────────────────────────────────────────────

    static double angleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    static double normalizeBearing(double bearing) {
        bearing = bearing % (2 * Math.PI);
        if (bearing < 0) bearing += 2 * Math.PI;
        return bearing;
    }
}
