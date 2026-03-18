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

public final class DefaultAttackSub implements SubmarineController {

    @Override
    public String name() { return "Default Sub"; }

    // State machine
    enum State { PATROL, TRACKING, CHASE, RAM, EVADE }

    // Throttle constants
    static final double PATROL_THROTTLE = 0.4;
    static final double TRACKING_THROTTLE = 0.25;
    static final double CHASE_THROTTLE = 0.8;
    static final double RAM_THROTTLE = 1.0;
    static final double EVADE_THROTTLE = 0.15;

    // Terrain / depth constants
    private static final double MIN_DEPTH = -20;
    private static final double FLOOR_CLEARANCE = 50;
    private static final double CRUSH_SAFETY_MARGIN = 50;
    private static final double BOUNDARY_TURN_DIST = 700;
    private static final double EMERGENCY_GAP = 60;
    private static final double[] SCAN_DISTANCES = {50, 100, 200, 400, 600, 1000, 1500};
    private static final double SCAN_SIDE_ANGLE = 0.6;
    private static final double HULL_HALF_LENGTH = 32.5; // must match SubmarinePhysics

    // Contact tracking constants
    static final int CONTACT_CONFIRM_TICKS = 3;
    static final double CHASE_RANGE = 3000.0;
    static final double RAM_RANGE = 500.0;
    static final double RAM_OVERSHOT_RANGE = 800.0;
    static final double TRACKING_DISPLACEMENT = 200.0;

    // Confidence-based tracking constants
    static final double CONFIDENCE_DECAY = 0.999;
    static final double CONFIDENCE_HUNT_MIN = 0.1;
    static final double CONFIDENCE_LOST = 0.02;
    static final double CONFIDENCE_RAM_MIN = 0.05;

    // Contact tracking physics constants
    static final double PASSIVE_IGNORE_RANGE = 5000.0; // ignore passive range beyond this
    static final double PASSIVE_WEAK_RANGE = 2000.0;   // reduced trust for passive range beyond this

    // Baffle clearing constants
    static final int BAFFLE_CLEAR_INTERVAL = 1500;   // 30s at 50Hz
    static final double BAFFLE_CLEAR_ANGLE = Math.toRadians(40);

    // Sprint-and-drift constants
    static final int SPRINT_DURATION = 750;           // 15s
    static final int DRIFT_DURATION = 1000;           // 20s

    // Patrol ping constants
    static final int PATROL_SILENCE_PING_TICKS = 3000; // 1 min at 50Hz
    private static final double BEHIND_OFFSET = 500.0;

    // Stern tailing constants
    private static final double BEHIND_ARC = Math.toRadians(45);  // within 45 deg of target stern
    private static final double TAILING_THROTTLE = 0.30;          // quiet following speed
    private static final double TAILING_FLOOR_CLEARANCE = 150.0;  // extra room when tailing

    // (Passive range estimation constants removed: TMA now handled by engine-side ContactTracker)

    // Cavitation constants (mirrored from SubmarinePhysics)
    private static final double BASE_CAVITATION_SPEED = 5.0;
    private static final double CAVITATION_DEPTH_FACTOR = 0.02;

    // Evade constants
    static final double EVADE_SE_THRESHOLD = 15.0;
    static final double EVADE_BEARING_RATE_THRESHOLD = Math.toRadians(2);

    // Ping range threshold for tracked contact
    private static final double TRACKED_PING_RANGE = 1500.0;

    // Path planner constants
    private static final double WAYPOINT_ARRIVAL_DIST = 200.0;
    private static final long REPLAN_INTERVAL = 2500; // replan every 50 seconds at 50Hz
    private static final double SHALLOW_WATER_LIMIT = -90.0; // minimum water depth for submerged passage
    private static final double IMPASSABLE_LIMIT = -25.0;   // floor above this is impassable even surfaced
    private static final int MAX_RECURSION_DEPTH = 3;
    private static final double ROUTE_SAMPLE_STEP = 100.0; // sample terrain every 100m along route

    // State
    private State state = State.PATROL;
    private MatchConfig config;
    private TerrainMap terrain;
    private double depthLimit;
    private double maxSubSpeed = 15.0;       // from MatchConfig, used for uncertainty growth and clamping
    private double refUncertainty = 50.0;    // reference uncertainty for confidence calculation, updated on ping fix
    private List<ThermalLayer> thermalLayers = List.of();

    // Contact tracking (sonar analysis, per-tick)
    private double lastHeading;  // cached for use in planPatrol / planRoute
    private double lastSpeed;    // cached for heading-aware route planning
    private double lastContactBearing;
    private double lastContactSE;
    private long lastContactTick = -1000;
    private int consecutiveContactTicks;
    private double estimatedRange = Double.MAX_VALUE;
    private boolean rangeConfirmedByActive;
    private int previousHp;

    // Tracked contact (persistent, dead-reckoned between updates)
    private boolean hasTrackedContact;
    private double trackedX, trackedY;
    private double trackedHeading = Double.NaN;
    private double trackedSpeed = 2.0;  // assume patrol speed
    private double contactAlive = 0.0;       // belief contact exists (0-1), decays at 0.999/tick
    private double uncertaintyRadius = 0.0;  // meters, grows by maxSubSpeed * dt each tick
    private long trackedLastFixTick = -1;

    // Ping fix history for viewer trace lines
    private record PingFix(double x, double y, long tick) {}
    private final List<PingFix> pingFixHistory = new ArrayList<>();

    // Baffle clearing
    private long baffleClearTimer;
    private double preBaffleClearHeading;
    private boolean clearingBaffles;
    private int baffleClearDirection = 1;

    // Chase timing
    private long chaseStartTick;

    // Last known contact area (preserved across state transitions for patrol biasing)
    private double lastKnownContactX = Double.NaN;
    private double lastKnownContactY = Double.NaN;

    // Patrol silence tracking
    private long patrolSilenceTicks;

    // Path planner state
    private final List<Vec3> navWaypoints = new ArrayList<>();
    private int currentWaypointIndex = 0;
    private long lastPlanTick = -1;
    private double lastPlanTargetX = Double.NaN, lastPlanTargetY = Double.NaN;
    private BattleArea battleArea;
    private PathPlanner pathPlanner;
    private double cachedProximityBearing = Double.NaN;
    private long proximityCheckTick = -100;
    private double cachedEscapeBearing = Double.NaN;
    private long escapeCheckTick = -100;

    @Override
    public void onMatchStart(MatchContext context) {
        this.config = context.config();
        this.terrain = context.terrain();
        this.depthLimit = config.crushDepth() + CRUSH_SAFETY_MARGIN;
        this.maxSubSpeed = config.maxSubSpeed();
        this.thermalLayers = context.thermalLayers();
        this.previousHp = config.startingHp();
        this.battleArea = config.battleArea();
        this.pathPlanner = new PathPlanner(context.terrain(), SHALLOW_WATER_LIMIT, 200, 75);
    }

    State state() { return state; }
    double estimatedRange() { return estimatedRange; }
    boolean hasTrackedContact() { return hasTrackedContact; }
    double trackedX() { return trackedX; }
    double trackedY() { return trackedY; }
    double contactAlive() { return contactAlive; }
    double uncertaintyRadius() { return uncertaintyRadius; }
    double trackedHeading() { return trackedHeading; }

    @Override
    public void onTick(SubmarineInput input, SubmarineOutput output) {
        var self = input.self();
        var pos = self.pose().position();
        double heading = self.pose().heading();
        this.lastHeading = heading;
        this.lastSpeed = self.velocity().speed();
        double depth = pos.z();
        long tick = input.tick();

        // =================================================================
        // Step 1: Process sonar contacts (engine provides TMA data)
        // =================================================================
        SonarContact bestContact = pickBestContact(input.sonarContacts(), input.activeSonarReturns());

        if (bestContact != null) {
            lastContactBearing = bestContact.bearing();
            lastContactSE = bestContact.signalExcess();
            lastContactTick = tick;
            consecutiveContactTicks++;

            // Range from engine TMA
            if (bestContact.range() > 0 && bestContact.range() < 50000) {
                estimatedRange = bestContact.range();
            }
            rangeConfirmedByActive = bestContact.isActive();
        } else {
            consecutiveContactTicks = 0;
        }

        // =================================================================
        // Step 1b: Update tracked contact from engine TMA data
        // =================================================================
        double dt = 1.0 / 50.0; // one tick at 50Hz
        if (hasTrackedContact) {
            // Dead-reckon only when we have a good heading and no fresh contact
            if (!Double.isNaN(trackedHeading) && bestContact == null) {
                trackedX += trackedSpeed * Math.sin(trackedHeading) * dt;
                trackedY += trackedSpeed * Math.cos(trackedHeading) * dt;
            }
            uncertaintyRadius += maxSubSpeed * dt;
            contactAlive *= CONFIDENCE_DECAY;
        }

        if (bestContact != null) {
            contactAlive = 1.0;

            // Speed from blade-rate
            if (bestContact.estimatedSpeed() >= 0) {
                double quality = Math.clamp(bestContact.signalExcess() / 30.0, 0.1, 0.8);
                trackedSpeed = hasTrackedContact ?
                        trackedSpeed * (1 - quality) + bestContact.estimatedSpeed() * quality :
                        bestContact.estimatedSpeed();
            }

            // Heading from engine TMA
            if (!Double.isNaN(bestContact.estimatedHeading())) {
                trackedHeading = bestContact.estimatedHeading();
            }

            // Position from bearing + range
            double range = bestContact.range() > 50 ? bestContact.range() : 1000;
            double tx = pos.x() + range * Math.sin(bestContact.bearing());
            double ty = pos.y() + range * Math.cos(bestContact.bearing());

            if (hasTrackedContact) {
                double blend = bestContact.isActive() ? 0.8 : 0.2;
                trackedX = trackedX * (1 - blend) + tx * blend;
                trackedY = trackedY * (1 - blend) + ty * blend;
            } else {
                trackedX = tx;
                trackedY = ty;
                hasTrackedContact = true;
            }

            // Uncertainty from engine TMA
            if (bestContact.rangeUncertainty() > 0) {
                uncertaintyRadius = bestContact.rangeUncertainty() * 2;
            }

            // Update ref uncertainty for confidence calculation
            if (bestContact.isActive() && bestContact.rangeUncertainty() > 0) {
                refUncertainty = bestContact.rangeUncertainty() * 2;
            }

            trackedLastFixTick = tick;

            // Record ping fix for viewer trace lines
            if (bestContact.isActive() && bestContact.range() > 0) {
                pingFixHistory.add(new PingFix(tx, ty, tick));
                while (pingFixHistory.size() > 20) pingFixHistory.removeFirst();
            }
        }

        // Clear tracked contact only when the search area is too large
        // to be useful. Until then, keep dead-reckoning the position so
        // the sub can navigate to the predicted location.
        if (hasTrackedContact && uncertaintyRadius > 5000) {
            hasTrackedContact = false;
            trackedHeading = Double.NaN;
            pingFixHistory.clear();
        }

        // Detect damage taken
        boolean tookDamage = self.hp() < previousHp;
        previousHp = self.hp();

        // =================================================================
        // Step 2: State transitions (confidence-based)
        // =================================================================
        long ticksSinceContact = tick - lastContactTick;

        // Compute distance to tracked contact for transition decisions
        double trackedDist = Double.MAX_VALUE;
        if (hasTrackedContact) {
            double dx = trackedX - pos.x();
            double dy = trackedY - pos.y();
            trackedDist = Math.sqrt(dx * dx + dy * dy);
        }

        switch (state) {
            case PATROL -> {
                // Active ping return: immediate CHASE (bypass TRACKING confirmation)
                if (bestContact != null && bestContact.isActive() && bestContact.range() > 0) {
                    enterState(State.CHASE, tick);
                } else if (consecutiveContactTicks >= CONTACT_CONFIRM_TICKS) {
                    enterState(State.TRACKING, tick);
                }
            }
            case TRACKING -> {
                if ((tookDamage || shouldEvade()) && terrain.elevationAt(pos.x(), pos.y()) < -60) {
                    enterState(State.EVADE, tick);
                } else if (bestContact != null && bestContact.isActive() && bestContact.range() > 0) {
                    enterState(State.CHASE, tick);
                } else if (estimatedRange < CHASE_RANGE && contactAlive >= CONFIDENCE_HUNT_MIN) {
                    enterState(State.CHASE, tick);
                } else if (hasTrackedContact && trackedDist < CHASE_RANGE && contactAlive >= CONFIDENCE_HUNT_MIN) {
                    enterState(State.CHASE, tick);
                } else if (consecutiveContactTicks > 500 && contactAlive >= CONFIDENCE_HUNT_MIN) {
                    // Sustained contact for 10+ seconds: we know enough to chase
                    // even if range is still large. TRACKING throttle is too slow
                    // to close on a fast target.
                    enterState(State.CHASE, tick);
                } else if (!hasTrackedContact && ticksSinceContact > 500) {
                    enterState(State.PATROL, tick);
                }
            }
            case CHASE -> {
                if (tookDamage && terrain.elevationAt(pos.x(), pos.y()) < -60) {
                    // Only evade if we're in water deep enough to maneuver
                    enterState(State.EVADE, tick);
                } else if (estimatedRange < RAM_RANGE && rangeConfirmedByActive) {
                    enterState(State.RAM, tick);
                } else if (!hasTrackedContact && uncertaintyRadius > 3000) {
                    // Only give up when the search area is too large to be useful.
                    // If the tracked contact was recently lost, the dead-reckoned
                    // position is still worth pursuing.
                    enterState(State.PATROL, tick);
                }
            }
            case RAM -> {
                if (estimatedRange > RAM_OVERSHOT_RANGE && ticksSinceContact > 50) {
                    enterState(State.CHASE, tick);
                } else if (!hasTrackedContact || contactAlive < CONFIDENCE_RAM_MIN) {
                    enterState(State.PATROL, tick);
                }
            }
            case EVADE -> {
                if (!hasTrackedContact) {
                    enterState(State.PATROL, tick);
                } else if (bestContact != null && lastContactSE < 8.0 && ticksSinceContact == 0) {
                    // Contact weakening: hunter passing by, flip the script
                    enterState(State.TRACKING, tick);
                }
            }
        }

        // =================================================================
        // Step 3: Tactical layer (throttle, rudder, depth preference)
        // =================================================================
        double tacticalThrottle;
        double tacticalRudder = 0;
        double tacticalDepthPreference = Double.NaN; // NaN = no preference

        switch (state) {
            case PATROL -> {
                tacticalThrottle = PATROL_THROTTLE;

                // Track silence duration for active ping gambit
                if (bestContact != null) {
                    patrolSilenceTicks = 0;
                } else {
                    patrolSilenceTicks++;
                }

                // Plan patrol route (only when we have no waypoints or have
                // completed the route). Don't replan mid-route: let the sub
                // follow the plan to completion so it actually covers ground.
                // Replan if route completed, or if we've drifted into unsafe territory
                boolean routeDone = navWaypoints.isEmpty()
                        || currentWaypointIndex >= navWaypoints.size() - 1;
                boolean driftedUnsafe = !navWaypoints.isEmpty()
                        && pathPlanner != null && !pathPlanner.isSafe(pos.x(), pos.y())
                        && tick - lastPlanTick > 50; // replan at most every 1 second
                if (routeDone || driftedUnsafe) {
                    planPatrol(pos.x(), pos.y(), terrain, battleArea);
                    lastPlanTick = tick;
                }

                // Follow waypoints
                if (!navWaypoints.isEmpty()) {
                    var wp = navWaypoints.get(currentWaypointIndex);
                    double wpDx = wp.x() - pos.x();
                    double wpDy = wp.y() - pos.y();
                    double wpDist = Math.sqrt(wpDx * wpDx + wpDy * wpDy);

                    if (wpDist < WAYPOINT_ARRIVAL_DIST && currentWaypointIndex < navWaypoints.size() - 1) {
                        currentWaypointIndex++;
                        wp = navWaypoints.get(currentWaypointIndex);
                    }

                    tacticalRudder = steerTowardWaypoint(pos.x(), pos.y(), heading);
                    tacticalDepthPreference = wp.z();
                } else {
                    // Fallback: baffle clearing when no waypoints
                    baffleClearTimer++;
                    if (baffleClearTimer > BAFFLE_CLEAR_INTERVAL && !clearingBaffles) {
                        clearingBaffles = true;
                        preBaffleClearHeading = heading;
                        baffleClearDirection *= -1;
                    }
                    if (clearingBaffles) {
                        tacticalRudder = baffleClearDirection * 0.4;
                        double turned = angleDiff(heading, preBaffleClearHeading);
                        if (Math.abs(turned) > BAFFLE_CLEAR_ANGLE) {
                            clearingBaffles = false;
                            baffleClearTimer = 0;
                        }
                    }
                    tacticalDepthPreference = preferredDepthBelowThermocline(depth);
                }

                // Active ping gambit after long silence
                if (patrolSilenceTicks > PATROL_SILENCE_PING_TICKS
                        && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                    patrolSilenceTicks = 0;
                }
            }
            case TRACKING -> {
                // Adaptive throttle: must be fast enough to close on target
                tacticalThrottle = hasTrackedContact
                        ? adaptiveThrottle(trackedSpeed, trackedDist, isBehindTarget())
                        : TRACKING_THROTTLE;

                // No first-contact ping: engine TMA provides range from
                // passive sonar. Stay quiet to avoid revealing our position.

                if (bestContact != null) {
                    // Fresh contact: turn perpendicular for cross-bearing fix
                    double perpBearing = normalizeBearing(lastContactBearing + Math.PI / 2);
                    double diff = angleDiff(perpBearing, heading);
                    double perpBearing2 = normalizeBearing(lastContactBearing - Math.PI / 2);
                    double diff2 = angleDiff(perpBearing2, heading);
                    double targetDiff = Math.abs(diff) < Math.abs(diff2) ? diff : diff2;
                    tacticalRudder = Math.clamp(targetDiff * 2.0, -1, 1);
                } else if (hasTrackedContact) {
                    // Route toward tracked position using path planner
                    boolean drifted = pathPlanner != null && !pathPlanner.isSafe(pos.x(), pos.y());
                    boolean needReplan = navWaypoints.isEmpty()
                            || Double.isNaN(lastPlanTargetX)
                            || (Math.sqrt(Math.pow(trackedX - lastPlanTargetX, 2)
                                + Math.pow(trackedY - lastPlanTargetY, 2)) > 500)
                            || drifted;

                    if (needReplan) {
                        replanToTarget(pos.x(), pos.y(), trackedX, trackedY, terrain, tick);
                    }

                    if (!navWaypoints.isEmpty()) {
                        var wp = navWaypoints.get(currentWaypointIndex);
                        double wpDx = wp.x() - pos.x();
                        double wpDy = wp.y() - pos.y();
                        double wpDist = Math.sqrt(wpDx * wpDx + wpDy * wpDy);

                        if (wpDist < WAYPOINT_ARRIVAL_DIST && currentWaypointIndex < navWaypoints.size() - 1) {
                            currentWaypointIndex++;
                        }

                        tacticalRudder = steerTowardWaypoint(pos.x(), pos.y(), heading);
                    }
                    // No direct bearing fallback without path planning

                    // No ping in TRACKING: stay quiet, engine TMA provides range.
                }
            }
            case CHASE -> {
                boolean behindTarget = isBehindTarget();

                // Zig-zag approach: alternate between angled legs to maintain
                // cross-track speed for passive TMA while still closing distance.
                // Each leg heads ~30 degrees off the direct bearing, alternating
                // left and right. The net movement converges on the target.
                double zigzagAngle = Math.toRadians(30);
                long legDuration = SPRINT_DURATION + DRIFT_DURATION; // one full sprint-drift cycle per leg
                long legPhase = (tick - chaseStartTick) % (legDuration * 2);
                boolean zigLeft = legPhase < legDuration;
                long phaseInLeg = legPhase % legDuration;
                boolean sprinting = phaseInLeg < SPRINT_DURATION;

                // Adaptive throttle based on target speed and distance.
                // Sprint-and-drift modulates: sprint phase uses adaptive
                // throttle, drift phase goes quiet for sonar.
                double baseThrottle = adaptiveThrottle(trackedSpeed, trackedDist, behindTarget);
                if (behindTarget && trackedDist < 1000) {
                    tacticalThrottle = baseThrottle; // adaptive handles tailing
                } else if (sprinting) {
                    tacticalThrottle = baseThrottle;
                } else {
                    // Drift phase: go quiet, listen. But never slower than
                    // target speed (don't fall behind during drift).
                    double minDrift = Math.clamp(trackedSpeed / maxSubSpeed, 0.15, 0.5);
                    tacticalThrottle = Math.max(TRACKING_THROTTLE, minDrift);
                }

                // Steering with zig-zag for TMA
                if (hasTrackedContact) {
                    double targetX = trackedX;
                    double targetY = trackedY;

                    // Aim for stern only at close range, but only if it's safe terrain
                    if (!Double.isNaN(trackedHeading)
                            && trackedDist < 1500 && trackedDist > RAM_RANGE * 2) {
                        double offsetFraction = 1.0 - (trackedDist - RAM_RANGE * 2) / (1500 - RAM_RANGE * 2);
                        double offset = BEHIND_OFFSET * offsetFraction;
                        double sternX = targetX - offset * Math.sin(trackedHeading);
                        double sternY = targetY - offset * Math.cos(trackedHeading);
                        if (pathPlanner == null || pathPlanner.isSafe(sternX, sternY)) {
                            targetX = sternX;
                            targetY = sternY;
                        }
                    }

                    double directBearing = Math.atan2(targetX - pos.x(), targetY - pos.y());
                    if (directBearing < 0) directBearing += 2 * Math.PI;

                    // Apply zig-zag offset (reduces as we get closer, no offset when tailing)
                    double approachBearing;
                    if (behindTarget && trackedDist < 2000) {
                        // Close and behind: head straight in, no zig-zag
                        approachBearing = directBearing;
                    } else {
                        // Offset the approach bearing for TMA.
                        // Reduce the angle as distance shrinks (less offset at close range)
                        double angleFactor = Math.clamp(trackedDist / CHASE_RANGE, 0.3, 1.0);
                        double offset = zigzagAngle * angleFactor * (zigLeft ? -1 : 1);
                        approachBearing = normalizeBearing(directBearing + offset);
                    }

                    // Route toward the approach bearing target point
                    double approachDist = Math.min(trackedDist * 0.7, 2000);
                    double approachX = pos.x() + approachDist * Math.sin(approachBearing);
                    double approachY = pos.y() + approachDist * Math.cos(approachBearing);

                    // If the approach point is in unsafe terrain, fall back to
                    // the tracked contact position (A* will route around obstacles)
                    if (pathPlanner != null && !pathPlanner.isSafe(approachX, approachY)) {
                        approachX = trackedX;
                        approachY = trackedY;
                    }

                    // Replan if target moved significantly or we drifted into unsafe territory
                    boolean driftedUnsafe = pathPlanner != null && !pathPlanner.isSafe(pos.x(), pos.y());
                    boolean needReplan = navWaypoints.isEmpty()
                            || Double.isNaN(lastPlanTargetX)
                            || (Math.sqrt(Math.pow(approachX - lastPlanTargetX, 2)
                                + Math.pow(approachY - lastPlanTargetY, 2)) > 500)
                            || driftedUnsafe;

                    if (needReplan) {
                        replanToTarget(pos.x(), pos.y(), approachX, approachY, terrain, tick);
                    }

                    // Follow waypoints if available
                    if (!navWaypoints.isEmpty()) {
                        var wp = navWaypoints.get(currentWaypointIndex);
                        double wpDx = wp.x() - pos.x();
                        double wpDy = wp.y() - pos.y();
                        double wpDist = Math.sqrt(wpDx * wpDx + wpDy * wpDy);

                        if (wpDist < WAYPOINT_ARRIVAL_DIST && currentWaypointIndex < navWaypoints.size() - 1) {
                            currentWaypointIndex++;
                            wp = navWaypoints.get(currentWaypointIndex);
                        }

                        tacticalRudder = steerTowardWaypoint(pos.x(), pos.y(), heading);
                        if (!Double.isNaN(wp.z())) {
                            tacticalDepthPreference = wp.z();
                        }
                    }
                    // No direct bearing fallback: if A* can't find a path,
                    // the sub keeps its current heading and the proactive
                    // danger check (step 3b) handles terrain avoidance.

                    // Ping to relocate when: close to the dead-reckoned position
                    // and haven't had contact for a while, OR the search area
                    // is getting large (target might have changed course).
                    boolean nearPredictedPos = trackedDist < TRACKED_PING_RANGE;
                    boolean contactStale = ticksSinceContact > 500; // 10 seconds
                    boolean searchAreaGrowing = uncertaintyRadius > 1000;
                    if (input.activeSonarCooldownTicks() == 0
                            && ((nearPredictedPos && contactStale)
                                || (searchAreaGrowing && contactStale))) {
                        output.activeSonarPing();
                    }
                } else if (lastContactTick >= 0) {
                    double diff = angleDiff(lastContactBearing, heading);
                    tacticalRudder = Math.clamp(diff * 2.0, -1, 1);
                }

                // No ping to confirm range: engine TMA provides passive range.

                // Depth: stay deep and below thermocline for stealth
                tacticalDepthPreference = preferredDepthBelowThermocline(depth);
                // When sprinting (not quiet approach or tailing), go extra deep for cavitation
                if (!behindTarget && !(hasTrackedContact && trackedDist > CHASE_RANGE)) {
                    double targetSpeed = 11.0;
                    double minDepthForQuiet = -(targetSpeed - BASE_CAVITATION_SPEED) / CAVITATION_DEPTH_FACTOR;
                    double deepTarget = Math.min(minDepthForQuiet - 20, -320);
                    if (!Double.isNaN(tacticalDepthPreference)) {
                        tacticalDepthPreference = Math.min(tacticalDepthPreference, deepTarget);
                    } else {
                        tacticalDepthPreference = deepTarget;
                    }
                }
            }
            case RAM -> {
                tacticalThrottle = RAM_THROTTLE;

                // Steer toward tracked position, or fall back to bearing
                if (hasTrackedContact && trackedDist > 50) {
                    double ramBearing = Math.atan2(trackedX - pos.x(), trackedY - pos.y());
                    if (ramBearing < 0) ramBearing += 2 * Math.PI;
                    double diff = angleDiff(ramBearing, heading);
                    tacticalRudder = Math.clamp(diff * 3.0, -1, 1);
                } else if (lastContactTick >= 0) {
                    double diff = angleDiff(lastContactBearing, heading);
                    tacticalRudder = Math.clamp(diff * 3.0, -1, 1);
                }

                // Ping if contact lost and cooldown ready
                if (ticksSinceContact > 100 && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }
            }
            case EVADE -> {
                tacticalThrottle = EVADE_THROTTLE;

                // Route perpendicular to threat using A* planned path.
                // Only replan if we don't already have a safe route.
                if (lastContactTick >= 0) {
                    boolean currentRouteSafe = !navWaypoints.isEmpty()
                            && currentWaypointIndex < navWaypoints.size()
                            && pathPlanner != null
                            && pathPlanner.isSafe(navWaypoints.get(currentWaypointIndex).x(),
                                                   navWaypoints.get(currentWaypointIndex).y());
                    boolean needPlan = !currentRouteSafe;
                    if (needPlan) {
                        double perpBearing1 = normalizeBearing(lastContactBearing + Math.PI / 2);
                        double perpBearing2 = normalizeBearing(lastContactBearing - Math.PI / 2);
                        double escapeDist = 2000;
                        double tx1 = pos.x() + Math.sin(perpBearing1) * escapeDist;
                        double ty1 = pos.y() + Math.cos(perpBearing1) * escapeDist;
                        double tx2 = pos.x() + Math.sin(perpBearing2) * escapeDist;
                        double ty2 = pos.y() + Math.cos(perpBearing2) * escapeDist;

                        boolean safe1 = pathPlanner != null && pathPlanner.isSafe(tx1, ty1);
                        boolean safe2 = pathPlanner != null && pathPlanner.isSafe(tx2, ty2);
                        double tx, ty;
                        if (safe1 && !safe2) { tx = tx1; ty = ty1; }
                        else if (safe2 && !safe1) { tx = tx2; ty = ty2; }
                        else {
                            double f1 = terrain.elevationAt(tx1, ty1);
                            double f2 = terrain.elevationAt(tx2, ty2);
                            if (f1 < f2) { tx = tx1; ty = ty1; }
                            else { tx = tx2; ty = ty2; }
                        }
                        replanToTarget(pos.x(), pos.y(), tx, ty, terrain, tick);
                    }

                    if (!navWaypoints.isEmpty()) {
                        var wp = navWaypoints.get(currentWaypointIndex);
                        double wpDist = Math.sqrt(Math.pow(wp.x() - pos.x(), 2) + Math.pow(wp.y() - pos.y(), 2));
                        if (wpDist < WAYPOINT_ARRIVAL_DIST && currentWaypointIndex < navWaypoints.size() - 1) {
                            currentWaypointIndex++;
                        }
                        tacticalRudder = steerTowardWaypoint(pos.x(), pos.y(), heading);
                    }
                }

                // Dive as deep as possible, below thermocline
                tacticalDepthPreference = depthLimit + 20;
            }
            default -> tacticalThrottle = PATROL_THROTTLE;
        }

        // =================================================================
        // Steps 4-12: Safety pipeline
        // =================================================================
        double rudder = tacticalRudder;
        double sternPlanes = 0;
        double throttle = tacticalThrottle;
        double ballast = 0.5;
        String status = state.name();


        // Step 3b: Proactive terrain safety override.
        // Check if the sub is heading toward dangerous shallow water at multiple ranges.
        // This overrides all tactical steering -- survival trumps tactics.
        // Check both ahead (at multiple distances) and nearby (immediate proximity)
        boolean dangerAhead = false;
        for (double checkDist : new double[]{200, 500, 1000, 1500}) {
            double fx = pos.x() + Math.sin(heading) * checkDist;
            double fy = pos.y() + Math.cos(heading) * checkDist;
            if (worstFloorNear(fx, fy, 100, terrain) > SHALLOW_WATER_LIMIT) {
                dangerAhead = true;
                break;
            }
        }
        // Also check proximity: steer AWAY from the nearest shallow terrain.
        // Scan every 10 ticks (0.2s) to avoid per-tick overhead.
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
                rudder = Math.clamp(diff * 3.0, -1, 1);
                status = "TERRAIN PROXIMITY";
            }
            throttle = Math.max(throttle, 0.3);
        }
        if (dangerAhead) {
            if (tick - escapeCheckTick >= 10) {
                escapeCheckTick = tick;
                cachedEscapeBearing = findDeepWaterBearing(pos.x(), pos.y(), terrain);
            }
            double escapeBearing = cachedEscapeBearing;
            if (!Double.isNaN(escapeBearing)) {
                double diff = angleDiff(escapeBearing, heading);
                if (Math.abs(diff) > Math.toRadians(20)) {
                    rudder = Math.clamp(diff * 3.0, -1, 1);
                    status = "DANGER AHEAD";
                }
                throttle = Math.max(throttle, 0.3);

                // If next waypoint is in danger too, reroute toward escape
                if (!navWaypoints.isEmpty() && currentWaypointIndex < navWaypoints.size()) {
                    var wp = navWaypoints.get(currentWaypointIndex);
                    if (worstFloorNear(wp.x(), wp.y(), 150, terrain) > SHALLOW_WATER_LIMIT) {
                        double escDist = 800;
                        double escX = pos.x() + Math.sin(escapeBearing) * escDist;
                        double escY = pos.y() + Math.cos(escapeBearing) * escDist;
                        if (terrain.elevationAt(escX, escY) < SHALLOW_WATER_LIMIT) {
                            navWaypoints.clear();
                            navWaypoints.add(new Vec3(escX, escY, safeDepthAt(escX, escY, terrain)));
                            currentWaypointIndex = 0;
                            lastPlanTick = -1;
                        }
                    }
                }
            }
        }

        // Use larger floor clearance when tailing
        boolean tailing = (state == State.CHASE && isBehindTarget());
        double floorClearance = tailing ? TAILING_FLOOR_CLEARANCE : FLOOR_CLEARANCE;

        // Step 4: Compute target depth
        // Primary: waypoint depth or tactical preference (navigation-driven).
        // Safety floor: never go below floor + minimum clearance.
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        double floorCenter = terrain.elevationAt(pos.x(), pos.y());
        double floorBow = terrain.elevationAt(
                pos.x() + sinH * HULL_HALF_LENGTH, pos.y() + cosH * HULL_HALF_LENGTH);
        double floorStern = terrain.elevationAt(
                pos.x() - sinH * HULL_HALF_LENGTH, pos.y() - cosH * HULL_HALF_LENGTH);
        double floorBelow = Math.max(floorCenter, Math.max(floorBow, floorStern));
        double safetyFloor = clampTarget(floorBelow + floorClearance);

        // Use waypoint depth as the primary target (the path planner already
        // chose safe depths). Fall back to tactical preference or safety floor.
        double rawTarget;
        if (!Double.isNaN(tacticalDepthPreference)) {
            rawTarget = clampTarget(tacticalDepthPreference);
        } else {
            rawTarget = safetyFloor;
        }
        // Never go below the safety floor (absolute minimum clearance)
        if (rawTarget < safetyFloor) {
            rawTarget = safetyFloor;
        }

        // Step 5: Multi-point terrain scan
        // Scan along TWO directions: current heading (for immediate safety)
        // and toward the active waypoint (for navigation-aware planning).
        // Use the worse of the two for depth targeting, but only the
        // heading scan for rudder avoidance (since that's the direction
        // we're actually moving).
        double scanHeading = heading;
        double waypointHeading = heading;
        if (!navWaypoints.isEmpty() && currentWaypointIndex < navWaypoints.size()) {
            var wp = navWaypoints.get(currentWaypointIndex);
            waypointHeading = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
        }

        double worstFloor = floorBelow;
        double worstDist = 0;
        boolean dropOffAhead = false;
        boolean shallowWaterAhead = false;

        // Scan along current heading (immediate safety)
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
            // Shallow water: treat like terrain obstacle
            if (floor > SHALLOW_WATER_LIMIT) {
                shallowWaterAhead = true;
                if (floor > worstFloor) {
                    worstFloor = floor;
                    worstDist = dist;
                }
            }
        }
        // Also scan along waypoint bearing to ensure we can reach it
        // (but only affects depth targeting, not rudder avoidance)
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
        // Uses the worse of heading scan and waypoint scan for depth targeting.
        // Only overrides rudder when terrain along the current heading is
        // genuinely dangerous. If the waypoint direction is clear, the sub
        // should be free to turn toward it.
        double effectiveWorstFloor = Math.max(worstFloor, waypointWorstFloor);
        double worstTarget = clampTarget(effectiveWorstFloor + floorClearance);
        double avoidanceThreshold = 50;
        double margin = depth - (worstFloor + floorClearance);

        // Also check if we're currently IN shallow water (floor above SHALLOW_WATER_LIMIT)
        boolean inShallowWater = floorBelow > SHALLOW_WATER_LIMIT;

        if (margin < avoidanceThreshold || shallowWaterAhead || inShallowWater) {
            double urgency = Math.clamp(1.0 - margin / avoidanceThreshold, 0.0, 1.0);
            if (shallowWaterAhead) {
                urgency = Math.max(urgency, 0.6);
            }
            if (inShallowWater) {
                // We are IN dangerously shallow water. Maximum urgency.
                // Find the nearest deep water and steer toward it.
                urgency = 1.0;
                double escapeBearing = findDeepWaterBearing(pos.x(), pos.y(), terrain);
                if (!Double.isNaN(escapeBearing)) {
                    double diff = angleDiff(escapeBearing, heading);
                    rudder = Math.clamp(diff * 3.0, -1, 1);
                    throttle = 0.15;
                    status = "SHALLOW ESCAPE";
                }
            }
            if (!inShallowWater) {
                status = "AVOIDING TERRAIN";
            }

            if (worstTarget > rawTarget) {
                rawTarget = worstTarget;
            }

            // Slow down when terrain is rising.
            if (!inShallowWater) {
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

            if (!inShallowWater) {
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

        // Step 7: Drop-off slowdown
        if (dropOffAhead) {
            throttle = Math.min(throttle, 0.3);
            if (state == State.PATROL) status = "DROP-OFF AHEAD";
        }

        // Step 8: Cascade depth controller
        double immediateGap = depth - floorBelow;
        double depthError = depth - targetDepth;
        double verticalSpeed = self.velocity().linear().z();

        double gapFactor = Math.clamp(
                (immediateGap - EMERGENCY_GAP) / (200 - EMERGENCY_GAP), 0, 1);

        // Outer loop: depth error -> desired pitch angle
        // Positive depthError = above target = need to dive (negative pitch)
        double depthPGain = 0.015;
        double depthDGain = 0.4;
        double desiredPitch = Math.clamp(
                -depthPGain * depthError - depthDGain * verticalSpeed,
                -Math.toRadians(20), Math.toRadians(20));

        // Limit dive authority near floor
        if (desiredPitch < 0) {
            desiredPitch *= gapFactor;
        }

        // Inner loop: pitch error -> stern planes
        double currentPitch = self.pose().pitch();
        double pitchError = desiredPitch - currentPitch;
        sternPlanes = Math.clamp(3.0 * pitchError, -1.0, 1.0);

        // Ballast trim: slowly adjust to maintain neutral buoyancy.
        // If we constantly need pitch to maintain depth, adjust ballast to offload.
        // Include a small damping term so ballast responds to vertical speed too.
        double trimRate = 0.002;
        double trimDamp = 0.05;
        double rawBallast = Math.clamp(0.5 - trimRate * depthError - trimDamp * verticalSpeed, 0.0, 1.0);
        if (rawBallast < 0.5) {
            ballast = Math.clamp(0.5 - (0.5 - rawBallast) * gapFactor, 0.0, 1.0);
        } else {
            ballast = Math.clamp(rawBallast, 0.0, 1.0);
        }

        // Step 9: Pull-up (gap closing while sinking)
        boolean emergencySurface = false;
        if (immediateGap < floorClearance * 1.5 && verticalSpeed < -0.5) {
            double pullUpUrgency = Math.clamp(
                    (floorClearance * 1.5 - immediateGap) / floorClearance, 0.2, 0.8);
            sternPlanes = Math.max(sternPlanes, pullUpUrgency);
            ballast = Math.max(ballast, 0.5 + pullUpUrgency * 0.3);
            throttle = Math.min(throttle, 0.3);
            status = "PULL UP";
        }

        // Step 9b: Three-point turn when stuck in unsafe territory.
        // If we're in an A*-blocked cell, slow, and heading away from safety,
        // reverse to turn around regardless of floor depth.
        if (pathPlanner != null && !pathPlanner.isSafe(pos.x(), pos.y())
                && self.velocity().speed() < 3 && !navWaypoints.isEmpty()
                && currentWaypointIndex < navWaypoints.size()) {
            var wp = navWaypoints.get(currentWaypointIndex);
            double wpBearing = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
            double diff = angleDiff(wpBearing, heading);
            if (Math.abs(diff) > Math.toRadians(60)) {
                throttle = -0.5;
                rudder = Math.clamp(diff * 2.0, -1, 1);
                status = "THREE-POINT TURN";
            }
        }

        // Step 10: Emergency surface -- floor too shallow to operate submerged.
        // Blow all ballast and surface. Very loud but better than dying.
        // Only trigger when floor is genuinely shallow (not deep ocean) and gap is tight.
        if (floorBelow > -30 || (immediateGap < EMERGENCY_GAP && immediateGap >= 0)) {
            emergencySurface = true;
            sternPlanes = 0.8;
            ballast = 1.0;

            double escapeBearing = findDeepWaterBearing(pos.x(), pos.y(), terrain);

            if (depth > -5) {
                // Surfaced emergency: determine best escape direction.
                // Prefer existing safe waypoint over escape bearing scan.
                double escDir = Double.NaN;
                if (!navWaypoints.isEmpty() && currentWaypointIndex < navWaypoints.size()) {
                    var wp = navWaypoints.get(currentWaypointIndex);
                    if (pathPlanner == null || pathPlanner.isSafe(wp.x(), wp.y())) {
                        escDir = Math.atan2(wp.x() - pos.x(), wp.y() - pos.y());
                    }
                }
                if (Double.isNaN(escDir) && !Double.isNaN(escapeBearing)) {
                    escDir = escapeBearing;
                }

                if (!Double.isNaN(escDir)) {
                    double diff = angleDiff(escDir, heading);
                    double spd = self.velocity().speed();

                    if (Math.abs(diff) > Math.toRadians(90) && spd < 3) {
                        throttle = -0.5;
                        rudder = Math.clamp(diff * 2.0, -1, 1);
                        status = "THREE-POINT TURN";
                    } else if (Math.abs(diff) > Math.toRadians(90) && spd > 3) {
                        throttle = -1.0;
                        rudder = Math.clamp(diff * 3.0, -1, 1);
                        status = "EMERGENCY BRAKE";
                    } else {
                        rudder = Math.clamp(diff * 3.0, -1, 1);
                        throttle = 0.4;
                    }
                }
            } else if (depth > -5) {
                throttle = 0.3;
            } else {
                // Still submerged: blow tanks and surface, steer toward safety
                throttle = -1.0;
                if (!Double.isNaN(escapeBearing)) {
                    double diff = angleDiff(escapeBearing, heading);
                    rudder = Math.clamp(diff * 3.0, -1, 1);
                }
            }
            status = "EMERGENCY SURFACE";
        }

        // Step 11: Surface avoidance (only when NOT in emergency surface)
        if (!emergencySurface && depth > MIN_DEPTH) {
            sternPlanes = -0.5;
            ballast = 0.2;
            status = "SURFACE AVOIDANCE";
        }

        // Step 12: Border avoidance
        double distToBoundary = config.battleArea().distanceToBoundary(pos.x(), pos.y());
        if (distToBoundary < BOUNDARY_TURN_DIST) {
            status = "BORDER AVOIDANCE";
            double towardCenterAngle = Math.atan2(-pos.x(), -pos.y());
            if (towardCenterAngle < 0) towardCenterAngle += 2 * Math.PI;

            double angleDiff = towardCenterAngle - heading;
            while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
            while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

            double urgency = 1.0 - distToBoundary / BOUNDARY_TURN_DIST;
            rudder = Math.clamp(angleDiff * 2.0, -1, 1) * Math.max(urgency, 0.5);
        }

        // =================================================================
        // Step 13: Output actuators
        // =================================================================
        output.setRudder(rudder);
        output.setSternPlanes(sternPlanes);
        output.setThrottle(throttle);
        output.setBallast(ballast);
        String stateTag = state.name().substring(0, 1);
        if (tailing) stateTag = "C/TAIL";
        else if (!status.equals(state.name())) stateTag += "/" + status.charAt(0);
        output.setStatus(String.format("%s f:%.0f g:%.0f",
                stateTag, -floorBelow, immediateGap));

        // Torpedo firing solution: behind the target, within range, good track
        if (hasTrackedContact && state == State.CHASE && isBehindTarget()
                && trackedDist < 1500 && trackedDist > 200
                && !Double.isNaN(trackedHeading) && trackedSpeed > 0
                && contactAlive > 0.5 && uncertaintyRadius < 300) {
            double solutionAge = (tick - trackedLastFixTick) / 50.0;
            if (solutionAge < 30) {
                double quality = Math.clamp(1.0 - uncertaintyRadius / 300.0, 0.1, 1.0);
                output.publishFiringSolution(new FiringSolution(
                        trackedX, trackedY, trackedHeading, trackedSpeed, quality));
            }
        }

        // Publish tracked contact estimate every tick if available
        if (hasTrackedContact) {
            double posConf = refUncertainty / (refUncertainty + uncertaintyRadius);
            double conf = contactAlive * posConf;
            String label = uncertaintyRadius < 100 ? "ping" : "passive";
            output.publishContactEstimate(new ContactEstimate(trackedX, trackedY, conf, contactAlive, uncertaintyRadius, trackedHeading, trackedSpeed, label));
        }

        // Publish navigation waypoints for viewer visualization
        for (int i = 0; i < navWaypoints.size(); i++) {
            var wp = navWaypoints.get(i);
            output.publishWaypoint(new Waypoint(wp.x(), wp.y(), wp.z(), i == currentWaypointIndex));
        }
    }

    // State machine helpers

    private void enterState(State newState, long tick) {
        state = newState;
        if (newState == State.PATROL) {
            // PATROL means fully lost: clear all tracking state
            estimatedRange = Double.MAX_VALUE;
            rangeConfirmedByActive = false;
            clearingBaffles = false;
            baffleClearTimer = 0;
            patrolSilenceTicks = 0;
            // Remember last known contact area before clearing
            if (hasTrackedContact) {
                lastKnownContactX = trackedX;
                lastKnownContactY = trackedY;
            }
            // Clear tracked contact
            hasTrackedContact = false;
            trackedHeading = Double.NaN;
            contactAlive = 0;
            uncertaintyRadius = 0;
            pingFixHistory.clear();
            // Clear navigation waypoints (will replan on next tick)
            navWaypoints.clear();
            currentWaypointIndex = 0;
            lastPlanTick = -1;
            lastPlanTargetX = Double.NaN;
            lastPlanTargetY = Double.NaN;
        } else if (newState == State.CHASE) {
            chaseStartTick = tick;
            // Clear patrol waypoints so chase can plan its own route
            navWaypoints.clear();
            currentWaypointIndex = 0;
            lastPlanTargetX = Double.NaN;
            lastPlanTargetY = Double.NaN;
        }
    }

    private boolean isBehindTarget() {
        if (Double.isNaN(trackedHeading)) return false;
        // If target heading is close to contact bearing, the target is heading away from us
        double diff = Math.abs(angleDiff(trackedHeading, lastContactBearing));
        return diff < BEHIND_ARC;
    }

    private boolean shouldEvade() {
        // Evade only when we are fairly certain a contact is closing on us:
        // 1. Very loud (SE > 25 dB, meaning close)
        // 2. Close range (< 2000m)
        // 3. We have a confident heading estimate (solution quality > 0.5)
        // 4. That heading points straight at us (within 20 degrees)
        // 5. Took damage (always evade if hit)
        if (lastContactSE > 25.0 && hasTrackedContact && !Double.isNaN(trackedHeading)
                && estimatedRange < 2000) {
            // Only trust the heading if the TMA solution is good
            // (otherwise a noisy heading estimate could trigger false evasion)
            double bearingToUs = normalizeBearing(lastContactBearing + Math.PI);
            double diff = Math.abs(angleDiff(trackedHeading, bearingToUs));
            return diff < Math.toRadians(20);
        }
        return false;
    }

    private SonarContact pickBestContact(List<SonarContact> passive, List<SonarContact> active) {
        SonarContact best = null;
        double bestSE = 0;
        for (var c : active) {
            if (c.signalExcess() > bestSE) {
                bestSE = c.signalExcess();
                best = c;
            }
        }
        for (var c : passive) {
            if (c.signalExcess() > bestSE) {
                bestSE = c.signalExcess();
                best = c;
            }
        }
        return best;
    }

    /**
     * Computes adaptive throttle based on target speed, distance, and
     * tactical situation. Ensures the sub can always close on the target
     * while balancing speed vs stealth.
     */
    private double adaptiveThrottle(double targetSpeed, double dist, boolean behind) {
        double maxSpd = maxSubSpeed;

        if (behind && dist < 1000) {
            // Behind and close: match target speed for quiet tailing
            return Math.clamp(targetSpeed / maxSpd, TRACKING_THROTTLE, 0.5);
        }

        // Minimum speed to close the gap (target speed + 1 m/s)
        double minClosingSpeed = targetSpeed + 1.0;

        if (dist > 4000) {
            // Long range: sprint hard (our noise won't reach the target).
            // At least fast enough to close, preferably full chase speed.
            double desiredSpeed = Math.max(minClosingSpeed, maxSpd * CHASE_THROTTLE);
            return Math.clamp(desiredSpeed / maxSpd, PATROL_THROTTLE, 1.0);
        } else if (dist > 2000) {
            // Medium range: moderate chase, somewhat stealthy
            double desiredSpeed = minClosingSpeed + 2.0;
            return Math.clamp(desiredSpeed / maxSpd, TRACKING_THROTTLE, CHASE_THROTTLE);
        } else if (behind && dist > 500) {
            // Close and behind: quiet approach
            double desiredSpeed = minClosingSpeed;
            return Math.clamp(desiredSpeed / maxSpd, TRACKING_THROTTLE, 0.6);
        } else {
            // Close but not behind: need to maneuver, moderate speed
            double desiredSpeed = minClosingSpeed + 1.0;
            return Math.clamp(desiredSpeed / maxSpd, TRACKING_THROTTLE, CHASE_THROTTLE);
        }
    }

    private double preferredDepthBelowThermocline(double currentDepth) {
        if (thermalLayers.isEmpty()) return Double.NaN;
        // Find the shallowest thermocline and go below it
        double shallowest = thermalLayers.getFirst().depth();
        for (var layer : thermalLayers) {
            if (layer.depth() > shallowest) shallowest = layer.depth();
        }
        // Target 30m below the thermocline
        return shallowest - 30;
    }

    // ── Path planner ────────────────────────────────────────────────

    /**
     * Returns a rudder value to steer toward the current waypoint.
     */
    /**
     * Replans the patrol route, preserving the current waypoint and replacing
     * everything after it. Plans a new patrol from the current waypoint's position.
     */
    private void replanFromCurrentWaypoint(double posX, double posY,
                                            TerrainMap terrain, BattleArea area) {
        // Simply replan from the sub's current position.
        // planPatrol handles prepending the marker and heading-aware first leg.
        planPatrol(posX, posY, terrain, area);
    }

    /**
     * Replans a route to a target, preserving the current waypoint and replacing
     * everything after it with a new route from the current waypoint to the target.
     */
    private void replanToTarget(double posX, double posY, double targetX, double targetY,
                                 TerrainMap terrain, long tick) {
        if (navWaypoints.isEmpty()) {
            // No existing plan: prepend current position marker, then
            // use heading-aware planning for a physically reachable path
            navWaypoints.add(new Vec3(posX, posY, safeDepthAt(posX, posY, terrain)));
            var route = planRouteFromSub(posX, posY, lastHeading, lastSpeed,
                    targetX, targetY, terrain);
            if (!route.isEmpty()) {
                navWaypoints.addAll(route);
                currentWaypointIndex = 1;
            } else {
                // No route found; add direct fallback so index 1 is valid
                navWaypoints.add(new Vec3(targetX, targetY, safeDepthAt(targetX, targetY, terrain)));
                currentWaypointIndex = 1;
            }
        } else {
            var currentWp = navWaypoints.get(currentWaypointIndex);
            // Plan new route from current waypoint to target
            var route = planRoute(currentWp.x(), currentWp.y(), targetX, targetY, terrain);
            if (!route.isEmpty()) {
                // Keep current waypoint, replace everything after
                while (navWaypoints.size() > currentWaypointIndex + 1) {
                    navWaypoints.removeLast();
                }
                navWaypoints.addAll(route);
            }
        }
        lastPlanTargetX = targetX;
        lastPlanTargetY = targetY;
        lastPlanTick = tick;
    }

    private double steerTowardWaypoint(double posX, double posY, double heading) {
        if (navWaypoints.isEmpty()) return 0;
        var wp = navWaypoints.get(currentWaypointIndex);
        double targetBearing = Math.atan2(wp.x() - posX, wp.y() - posY);
        if (targetBearing < 0) targetBearing += 2 * Math.PI;
        double diff = angleDiff(targetBearing, heading);
        return Math.clamp(diff * 2.0, -1, 1);
    }

    /**
     * Plans a route from (fromX, fromY) to (toX, toY) using A* path planning
     * to avoid shallow terrain. Returns a list of Vec3 waypoints with safe depths.
     */
    List<Vec3> planRoute(double fromX, double fromY, double toX, double toY,
                         TerrainMap terrain) {
        if (pathPlanner != null) {
            double operatingDepth = stealthDepthAt(fromX, fromY, terrain);
            var path = pathPlanner.findPath(fromX, fromY, toX, toY, operatingDepth);
            if (!path.isEmpty()) return path;
        }
        // Fallback: direct route with safe depth
        return List.of(new Vec3(toX, toY, safeDepthAt(toX, toY, terrain)));
    }

    /**
     * Plans a heading-aware route from the sub's current position. First
     * tries a normal A* route. If the first waypoint requires a sharp turn
     * (> 90° from current heading), inserts a "lead point" ahead of the sub
     * so the path starts in a physically reachable direction.
     *
     * Returns the route (without the current-position marker; caller prepends that).
     */
    private List<Vec3> planRouteFromSub(double posX, double posY, double heading,
                                         double speed, double goalX, double goalY,
                                         TerrainMap terrain) {
        // Plan normally first
        var directRoute = new ArrayList<>(planRoute(posX, posY, goalX, goalY, terrain));
        if (directRoute.isEmpty()) return directRoute;

        // Strip the grid-snapped start waypoint if it's just the origin
        // snapped to the A* grid (not a useful navigation target)
        if (directRoute.size() > 1) {
            var first = directRoute.get(0);
            double d = Math.sqrt(Math.pow(first.x() - posX, 2) + Math.pow(first.y() - posY, 2));
            if (d < 150) directRoute.remove(0);
        }

        // Check angle to first waypoint
        var firstWp = directRoute.get(0);
        double wpBearing = Math.atan2(firstWp.x() - posX, firstWp.y() - posY);
        double turnAngle = wpBearing - heading;
        while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
        while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

        // If first waypoint is reachable with a moderate turn, use direct route
        if (Math.abs(turnAngle) < Math.PI / 2) {
            return directRoute;
        }

        // Sharp turn needed (> 90°): insert a lead point ahead of the sub
        // to create a gentle curve instead of an immediate reversal
        double leadDist = Math.max(200, speed * 20);
        double goalDist = Math.sqrt(Math.pow(goalX - posX, 2) + Math.pow(goalY - posY, 2));
        leadDist = Math.min(leadDist, goalDist * 0.4);
        leadDist = Math.max(leadDist, 100);

        double leadX = posX + Math.sin(heading) * leadDist;
        double leadY = posY + Math.cos(heading) * leadDist;

        // Check if lead point is safe; if not, try shorter distances
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
                return directRoute; // can't insert safe lead point
            }
        }

        // Build route: lead point, then A* from lead to goal
        var result = new ArrayList<Vec3>();
        result.add(new Vec3(leadX, leadY, safeDepthAt(leadX, leadY, terrain)));

        var astarPath = planRoute(leadX, leadY, goalX, goalY, terrain);
        for (int i = 0; i < astarPath.size(); i++) {
            var wp = astarPath.get(i);
            if (i == 0) {
                double d = Math.sqrt(Math.pow(wp.x() - leadX, 2) + Math.pow(wp.y() - leadY, 2));
                if (d < 100) continue; // skip near-duplicate
            }
            result.add(wp);
        }
        return result;
    }


    /**
     * Plans a patrol route that explores the arena.
     * The first waypoint heads toward the arena center (where contacts
     * are most likely), then subsequent waypoints fan out to cover ground.
     * If we have a last known contact area, the route biases toward it.
     */
    void planPatrol(double x, double y, TerrainMap terrain, BattleArea area) {
        navWaypoints.clear();
        // Waypoint 0: current position marker (not navigated toward)
        navWaypoints.add(new Vec3(x, y, safeDepthAt(x, y, terrain)));
        currentWaypointIndex = 1;

        int numPoints = 5;
        double arenaExtent = area.extent();

        // Determine the primary search direction.
        // Priority: last known contact > arena center > current heading.
        double targetX, targetY;
        if (!Double.isNaN(lastKnownContactX)) {
            targetX = lastKnownContactX;
            targetY = lastKnownContactY;
            lastKnownContactX = Double.NaN;
            lastKnownContactY = Double.NaN;
        } else {
            // Head toward the arena center from our current position.
            // If already near center, head in current direction.
            double distFromCenter = Math.sqrt(x * x + y * y);
            if (distFromCenter > arenaExtent * 0.3) {
                targetX = 0;
                targetY = 0;
            } else {
                targetX = x + 3000 * Math.sin(lastHeading);
                targetY = y + 3000 * Math.cos(lastHeading);
            }
        }

        // Build waypoints: first toward the target, then fanning out
        double toTargetAngle = Math.atan2(targetX - x, targetY - y);
        if (toTargetAngle < 0) toTargetAngle += 2 * Math.PI;
        double distToTarget = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow(targetY - y, 2));

        var rawPoints = new ArrayList<Vec3>();
        for (int i = 0; i < numPoints; i++) {
            double angle;
            double radius;
            if (i == 0) {
                // First waypoint: head toward target, at a reasonable distance
                angle = toTargetAngle;
                radius = Math.min(distToTarget * 0.6, 3000);
                radius = Math.max(radius, 1500);
            } else {
                // Subsequent waypoints: fan out in an expanding search pattern
                // centered on the target direction
                double spread = Math.PI * 0.4; // +/- 72 degrees from target direction
                angle = toTargetAngle + spread * (i % 2 == 0 ? 1 : -1) * ((i + 1) / 2.0) / (numPoints / 2.0);
                radius = 2000 + i * 500;
            }

            double px = x + radius * Math.sin(angle);
            double py = y + radius * Math.cos(angle);

            // Clamp to stay within battle area (with margin for border avoidance)
            double margin = BOUNDARY_TURN_DIST + 200;
            double distToBound = area.distanceToBoundary(px, py);
            if (distToBound < margin) {
                double scale = 0.7;
                px = px * scale;
                py = py * scale;
            }

            // Reject waypoints in unsafe terrain (A* planner knows what's safe)
            boolean waypointUnsafe = pathPlanner != null
                    ? !pathPlanner.isSafe(px, py)
                    : worstFloorNear(px, py, 200, terrain) > SHALLOW_WATER_LIMIT;
            if (waypointUnsafe) {
                // Waypoint is in shallow water: try to find a safe nearby point
                // by scanning outward from the center of the arena
                double toCenterAngle = Math.atan2(-px, -py);
                boolean relocated = false;
                for (double pullDist = 500; pullDist <= 2000; pullDist += 500) {
                    double npx = px + Math.sin(toCenterAngle) * pullDist;
                    double npy = py + Math.cos(toCenterAngle) * pullDist;
                    if (terrain.elevationAt(npx, npy) < SHALLOW_WATER_LIMIT - 50) {
                        px = npx;
                        py = npy;
                        relocated = true;
                        break;
                    }
                }
                if (!relocated) continue; // skip this waypoint entirely
            }

            double safeZ = stealthDepthAt(px, py, terrain);
            rawPoints.add(new Vec3(px, py, safeZ));
        }

        // Build the final waypoint list, checking each leg for terrain obstacles.
        // First leg uses heading-aware planning so the path starts in a
        // physically reachable direction for the sub.
        for (int i = 0; i < rawPoints.size(); i++) {
            var to = rawPoints.get(i);
            List<Vec3> leg;
            if (i == 0) {
                // First leg: from sub's current position, heading-aware
                leg = planRouteFromSub(x, y, lastHeading, lastSpeed,
                        to.x(), to.y(), terrain);
            } else {
                var from = rawPoints.get(i - 1);
                leg = planRoute(from.x(), from.y(), to.x(), to.y(), terrain);
            }
            navWaypoints.addAll(leg);
        }

        // Remove duplicate waypoints that are very close together
        // (but never remove the marker at index 0)
        for (int i = navWaypoints.size() - 1; i > 1; i--) {
            var a = navWaypoints.get(i);
            var b = navWaypoints.get(i - 1);
            double dist = Math.sqrt(Math.pow(a.x() - b.x(), 2) + Math.pow(a.y() - b.y(), 2));
            if (dist < 50) {
                navWaypoints.remove(i);
            }
        }

        // Safety: ensure at least one navigable waypoint after marker
        if (navWaypoints.size() < 2) {
            currentWaypointIndex = 0; // fall back to navigating toward marker
        }
    }

    /**
     * Returns the safe operating depth at a world position:
     * terrain elevation + floor clearance, clamped to operating limits.
     */
    /**
     * Finds the bearing toward the nearest deep water from the given position.
     * Scans 36 directions (every 10 degrees) at increasing distances until
     * deep water (below SHALLOW_WATER_LIMIT) is found.
     * Returns the bearing in radians, or NaN if no deep water found.
     */
    private double findDeepWaterBearing(double x, double y, TerrainMap terrain) {
        double bestBearing = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        double deepestBearing = Double.NaN;
        double deepestFloor = Double.MAX_VALUE;
        for (int deg = 0; deg < 360; deg += 10) {
            double brg = Math.toRadians(deg);
            boolean pathBlocked = false;
            for (double dist = 200; dist <= 3000; dist += 200) {
                double sx = x + Math.sin(brg) * dist;
                double sy = y + Math.cos(brg) * dist;
                double floor = terrain.elevationAt(sx, sy);
                // If the path crosses very shallow or impassable terrain, skip
                if (floor > IMPASSABLE_LIMIT) {
                    pathBlocked = true;
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

    /**
     * Returns the shallowest (worst) floor elevation within a radius around (x,y).
     * Checks 8 compass directions plus the center.
     */
    private static double worstFloorNear(double x, double y, double radius, TerrainMap terrain) {
        double worst = terrain.elevationAt(x, y);
        for (int deg = 0; deg < 360; deg += 45) {
            double brg = Math.toRadians(deg);
            double floor = terrain.elevationAt(x + Math.sin(brg) * radius, y + Math.cos(brg) * radius);
            if (floor > worst) worst = floor;
        }
        return worst;
    }

    private double safeDepthAt(double x, double y, TerrainMap terrain) {
        double floor = terrain.elevationAt(x, y);
        double target = floor + FLOOR_CLEARANCE;
        return clampTarget(target);
    }

    /**
     * Returns a stealthy operating depth: close to the floor for reduced
     * detection, with just enough clearance for safe navigation.
     * Uses half the normal floor clearance.
     */
    private double stealthDepthAt(double x, double y, TerrainMap terrain) {
        double floor = terrain.elevationAt(x, y);
        double target = floor + FLOOR_CLEARANCE * 0.6; // tighter to the floor
        return clampTarget(target);
    }

    // Geometry helpers

    private double clampTarget(double target) {
        if (target < depthLimit) target = depthLimit;
        if (target > MIN_DEPTH) target = MIN_DEPTH;
        return target;
    }

    /**
     * Clamp rudder command based on current speed to avoid control surface stall.
     * At low speed, caps at ~0.35 (stall peak); at high speed, allows more
     * since v^2 provides ample authority even at moderate deflections.
     */
    static double speedAdaptiveRudder(double rawRudder, double speed) {
        return Math.clamp(rawRudder, -1, 1);
    }

    static double angleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    private static double normalizeBearing(double bearing) {
        bearing = bearing % (2 * Math.PI);
        if (bearing < 0) bearing += 2 * Math.PI;
        return bearing;
    }
}
