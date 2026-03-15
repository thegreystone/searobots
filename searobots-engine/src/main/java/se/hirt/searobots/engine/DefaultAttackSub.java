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
    private static final double FLOOR_CLEARANCE = 80;
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

    // Passive range estimation constants
    // SE-based: assume target SL ~90 dB (patrol speed), cylindrical spreading
    private static final double ASSUMED_TARGET_SL_DB = 90.0;
    private static final double ASSUMED_AMBIENT_NL_DB = 60.0;
    private static final double SPREADING_COEFFICIENT = 10.0; // cylindrical (must match SonarModel)
    // Bearing rate TMA: minimum bearing rate to produce useful estimate
    private static final double MIN_BEARING_RATE = Math.toRadians(0.05); // per second
    private static final int TMA_HISTORY_TICKS = 100; // 2 seconds of bearing history

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
    private static final double SHALLOW_WATER_LIMIT = -50.0; // minimum water depth for passage
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
    record BearingFix(long tick, double bearing, double se, double x, double y,
                      double ownSpeed, double ownHeading) {}
    private final List<BearingFix> contactTrack = new ArrayList<>();
    private double lastHeading;  // cached for use in planPatrol
    private double lastContactBearing;
    private double lastContactSE;
    private long lastContactTick = -1000;
    private int consecutiveContactTicks;
    private double estimatedRange = Double.MAX_VALUE;
    private boolean rangeConfirmedByActive;
    private double calibratedSL = Double.NaN; // SL computed from ping range + passive SE
    private int previousHp;

    // Tracked contact (persistent, dead-reckoned between updates)
    private boolean hasTrackedContact;
    private double trackedX, trackedY;
    private double trackedHeading = Double.NaN;
    private double trackedSpeed = 2.0;  // assume patrol speed
    private double contactAlive = 0.0;       // belief contact exists (0-1), decays at 0.999/tick
    private double uncertaintyRadius = 0.0;  // meters, grows by maxSubSpeed * dt each tick
    private long trackedLastFixTick = -1;
    // Previous position fix for heading estimation
    private double prevFixX = Double.NaN, prevFixY = Double.NaN;
    private long prevFixTick = -1;

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

    @Override
    public void onMatchStart(MatchContext context) {
        this.config = context.config();
        this.terrain = context.terrain();
        this.depthLimit = config.crushDepth() + CRUSH_SAFETY_MARGIN;
        this.maxSubSpeed = config.maxSubSpeed();
        this.thermalLayers = context.thermalLayers();
        this.previousHp = config.startingHp();
        this.battleArea = config.battleArea();
    }

    State state() { return state; }
    double estimatedRange() { return estimatedRange; }
    List<BearingFix> contactTrack() { return contactTrack; }
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
        double depth = pos.z();
        long tick = input.tick();

        // =================================================================
        // Step 1: Process sonar contacts and update contact track
        // =================================================================
        SonarContact bestContact = pickBestContact(input.sonarContacts(), input.activeSonarReturns());

        if (bestContact != null) {
            lastContactBearing = bestContact.bearing();
            lastContactSE = bestContact.signalExcess();
            lastContactTick = tick;
            consecutiveContactTicks++;
            double speed = self.velocity().linear().length();

            // Active returns give us range directly (best quality)
            if (bestContact.isActive() && bestContact.range() > 0) {
                estimatedRange = bestContact.range();
                rangeConfirmedByActive = true;

                // Calibrate source level: we know the exact range from the
                // ping. Combined with the passive SE (which we also have),
                // we can compute the target's true SL:
                //   SE = SL - TL - NL, TL = spreading * log10(range)
                //   SL = SE + TL + NL
                // This calibrated SL gives much better passive range
                // estimates on subsequent contacts.
                if (lastContactSE > 5.0) { // above detection threshold
                    double tl = SPREADING_COEFFICIENT * Math.log10(
                            Math.max(bestContact.range(), 1.0));
                    calibratedSL = lastContactSE + tl + ASSUMED_AMBIENT_NL_DB;
                }
            }

            // Record bearing fix (for triangulation and TMA)
            contactTrack.add(new BearingFix(tick, bestContact.bearing(), bestContact.signalExcess(),
                    pos.x(), pos.y(), speed, heading));
            // Cap track length
            while (contactTrack.size() > 200) contactTrack.removeFirst();

            // Passive range estimation. After a ping fix, protect the estimate
            // briefly (500 ticks = 10 seconds), then allow passive updates
            // to correct dead-reckoning drift.
            long lastPingTick = pingFixHistory.isEmpty() ? -1 : pingFixHistory.getLast().tick();
            boolean pingRecent = rangeConfirmedByActive && lastPingTick >= 0
                    && (tick - lastPingTick) < 500;
            if (!pingRecent) {
                // Method 1: Triangulation (requires displacement between fixes)
                if (contactTrack.size() >= 2) {
                    var latest = contactTrack.getLast();
                    for (int i = contactTrack.size() - 2; i >= 0; i--) {
                        var older = contactTrack.get(i);
                        if (displacement(pos, older) > TRACKING_DISPLACEMENT) {
                            double rangeEst = triangulate(latest, older);
                            if (rangeEst > 0 && rangeEst < 50000) {
                                estimatedRange = rangeEst;
                            }
                            break;
                        }
                    }
                }

                // Method 2: SE-based range estimate. Use calibrated SL from
                // a previous ping if available (most accurate). Otherwise
                // fall back to the sonar's estimated source level.
                double slForRanging = !Double.isNaN(calibratedSL) ? calibratedSL
                        : bestContact.estimatedSourceLevel();
                double seRange = estimateRangeFromSEAndSL(
                        bestContact.signalExcess(), slForRanging);
                if (seRange > 0 && seRange < 50000) {
                    if (estimatedRange == Double.MAX_VALUE) {
                        estimatedRange = seRange;
                    } else {
                        // Blend SE range with current estimate. SE is noisy but
                        // provides a continuous range signal even when heading
                        // straight at the target (where TMA fails).
                        double blend = 0.1;
                        estimatedRange = estimatedRange * (1 - blend) + seRange * blend;
                    }
                }

                // Method 3: Bearing rate TMA (needs bearing history and own-ship speed)
                double tmaRange = estimateRangeFromBearingRate(tick, speed, heading);
                if (tmaRange > 0 && tmaRange < 50000) {
                    estimatedRange = tmaRange;
                }
            }
        } else {
            consecutiveContactTicks = 0;
        }

        // =================================================================
        // Step 1b: Update tracked contact
        // =================================================================

        // Dead-reckon existing tracked contact and grow uncertainty
        double dt = 1.0 / 50.0; // one tick at 50Hz
        if (hasTrackedContact) {
            if (!Double.isNaN(trackedHeading)) {
                trackedX += trackedSpeed * Math.sin(trackedHeading) * dt;
                trackedY += trackedSpeed * Math.cos(trackedHeading) * dt;
            }
            uncertaintyRadius += maxSubSpeed * dt;
            contactAlive *= CONFIDENCE_DECAY;
        }

        // Any sonar contact resets contactAlive
        if (bestContact != null) {
            contactAlive = 1.0;

            // Update target speed from blade-rate tonals (when available).
            // Blend with current estimate for smoothing.
            if (bestContact.estimatedSpeed() >= 0) {
                double bladeSpeed = bestContact.estimatedSpeed();
                if (hasTrackedContact) {
                    // Blend: weight by SE quality (high SE = trust blade-rate more)
                    double quality = Math.clamp(bestContact.signalExcess() / 30.0, 0.1, 0.8);
                    trackedSpeed = trackedSpeed * (1 - quality) + bladeSpeed * quality;
                } else {
                    trackedSpeed = bladeSpeed;
                }
            }

            // Bearing correction: snap the tracked position onto the bearing
            // line at the current estimated range. Also use the bearing
            // history to estimate target heading via TMA.
            if (hasTrackedContact && !bestContact.isActive()) {
                // Correction range: the distance from us to the tracked position.
                // This is smoother than estimatedRange (which jumps wildly from
                // noisy triangulation/TMA), because the tracked position is
                // stabilized by the bearing correction and dead-reckoning.
                // Only use estimatedRange when it comes from a recent ping
                // (which gives precise range data).
                double geometricRange = Math.sqrt(
                        Math.pow(trackedX - pos.x(), 2) + Math.pow(trackedY - pos.y(), 2));
                // Correction range: how far along the bearing to project.
                // Use geometric distance (from sub to tracked position).
                // Key constraint: the correction must never pull the tracked
                // position toward us faster than a real target could close.
                // Bearing correction using independent range measurement.
                // Only correct when we have a ping-calibrated SL (from a
                // previous "first contact ping"). Without calibration, the
                // sonar's estimated SL is too inaccurate for ranging and
                // the correction would cause the tracked position to collapse.
                double correctionRange = -1;
                long lastPingAge = pingFixHistory.isEmpty() ? Long.MAX_VALUE
                        : tick - pingFixHistory.getLast().tick();
                if (lastPingAge < 250 && estimatedRange > 0 && estimatedRange < 50000) {
                    correctionRange = estimatedRange;
                } else if (!Double.isNaN(calibratedSL)) {
                    double seRange = estimateRangeFromSEAndSL(
                            bestContact.signalExcess(), calibratedSL);
                    if (seRange > 50 && seRange < 50000) {
                        correctionRange = seRange;
                    }
                }
                if (correctionRange > 50) {
                    double correctedX = pos.x() + correctionRange * Math.sin(bestContact.bearing());
                    double correctedY = pos.y() + correctionRange * Math.cos(bestContact.bearing());
                    double bearingBlend = 0.15;
                    trackedX = trackedX * (1 - bearingBlend) + correctedX * bearingBlend;
                    trackedY = trackedY * (1 - bearingBlend) + correctedY * bearingBlend;
                }

                // Heading estimation from tracked position history.
                // The tracked position is smoothed by bearing corrections,
                // so successive positions trace the target's path. Compare
                // current tracked position to where it was 5 seconds ago.
                if (tick - prevFixTick >= 250 && correctionRange > 100) {
                    if (!Double.isNaN(prevFixX)) {
                        double dx = trackedX - prevFixX;
                        double dy = trackedY - prevFixY;
                        double moved = Math.sqrt(dx * dx + dy * dy);

                        if (moved > 30) {
                            double tgtHeading = Math.atan2(dx, dy);
                            if (tgtHeading < 0) tgtHeading += 2 * Math.PI;

                            if (Double.isNaN(trackedHeading)) {
                                trackedHeading = tgtHeading;
                            } else {
                                double hDiff = angleDiff(tgtHeading, trackedHeading);
                                if (Math.abs(hDiff) < Math.toRadians(60)) {
                                    trackedHeading = normalizeBearing(
                                            trackedHeading + hDiff * 0.2);
                                }
                            }
                        }
                    }
                    prevFixX = trackedX;
                    prevFixY = trackedY;
                    prevFixTick = tick;
                }
            }
        }

        // Update from sonar fixes
        if (bestContact != null && bestContact.isActive() && bestContact.range() > 0) {
            // Active ping fix: snap to precise position
            double tx = pos.x() + bestContact.range() * Math.sin(bestContact.bearing());
            double ty = pos.y() + bestContact.range() * Math.cos(bestContact.bearing());

            // Plausibility check: if too far from last ping fix, treat as new contact
            if (hasTrackedContact && !pingFixHistory.isEmpty()) {
                var lastPingFix = pingFixHistory.getLast();
                long timeSinceLastPing = tick - lastPingFix.tick();
                double maxDist = maxSubSpeed * (timeSinceLastPing / 50.0);
                double ddx = tx - lastPingFix.x();
                double ddy = ty - lastPingFix.y();
                double dist = Math.sqrt(ddx * ddx + ddy * ddy);
                if (dist > maxDist) {
                    // Too far from last ping fix, treat as new contact
                    trackedHeading = Double.NaN;
                    prevFixX = Double.NaN;
                    prevFixY = Double.NaN;
                    prevFixTick = -1;
                    pingFixHistory.clear();
                }
            }

            // Compute heading from previous ping fix (in pingFixHistory)
            if (!pingFixHistory.isEmpty()) {
                var lastPing = pingFixHistory.getLast();
                double ddx = tx - lastPing.x();
                double ddy = ty - lastPing.y();
                if (ddx * ddx + ddy * ddy > 25) { // moved at least 5m
                    trackedHeading = Math.atan2(ddx, ddy);
                    if (trackedHeading < 0) trackedHeading += 2 * Math.PI;
                }
            }

            prevFixX = tx;
            prevFixY = ty;
            prevFixTick = tick;

            trackedX = tx;
            trackedY = ty;
            // Compute uncertainty from sonar contact data
            // 2-sigma for ~95% confidence
            double pingUncertainty = bestContact.rangeUncertainty() * 2;
            // Also account for bearing uncertainty at this range
            double lateralUncertainty = bestContact.range() * bestContact.bearingUncertainty();
            uncertaintyRadius = Math.sqrt(pingUncertainty * pingUncertainty + lateralUncertainty * lateralUncertainty);
            refUncertainty = uncertaintyRadius;
            trackedLastFixTick = tick;
            hasTrackedContact = true;

            // Record in ping fix history (cap at 20)
            pingFixHistory.add(new PingFix(tx, ty, tick));
            while (pingFixHistory.size() > 20) pingFixHistory.removeFirst();

        } else if (bestContact != null && estimatedRange > 0 && estimatedRange < 50000) {
            // Passive contact with range estimate, with speed clamping

            if (estimatedRange > PASSIVE_IGNORE_RANGE) {
                // Too far: only use bearing as heading hint, don't update position
                if (hasTrackedContact && Double.isNaN(trackedHeading)) {
                    trackedHeading = lastContactBearing;
                }
            } else {
                // Determine blend weight based on range
                double blendWeight;
                if (estimatedRange > PASSIVE_WEAK_RANGE) {
                    blendWeight = 0.1;
                } else {
                    blendWeight = 0.3;
                }

                double tx = pos.x() + estimatedRange * Math.sin(lastContactBearing);
                double ty = pos.y() + estimatedRange * Math.cos(lastContactBearing);

                if (hasTrackedContact) {
                    // Compute proposed position via blend
                    double proposedX = trackedX * (1 - blendWeight) + tx * blendWeight;
                    double proposedY = trackedY * (1 - blendWeight) + ty * blendWeight;

                    // Clamp movement to maxSubSpeed * dt_since_last_update
                    long dtSinceLast = (trackedLastFixTick >= 0) ? tick - trackedLastFixTick : 1;
                    double maxMove = maxSubSpeed * (dtSinceLast / 50.0);
                    double moveDx = proposedX - trackedX;
                    double moveDy = proposedY - trackedY;
                    double moveDist = Math.sqrt(moveDx * moveDx + moveDy * moveDy);
                    if (moveDist > maxMove && moveDist > 0) {
                        double scale = maxMove / moveDist;
                        proposedX = trackedX + moveDx * scale;
                        proposedY = trackedY + moveDy * scale;
                    }

                    trackedX = proposedX;
                    trackedY = proposedY;
                } else {
                    trackedX = tx;
                    trackedY = ty;
                    double lateralUncertainty = estimatedRange * bestContact.bearingUncertainty();
                    uncertaintyRadius = Math.max(lateralUncertainty * 2, Math.min(estimatedRange * 0.15, 500));
                    hasTrackedContact = true;
                }

                // Heading estimation from successive passive fixes
                if (!Double.isNaN(prevFixX) && prevFixTick >= 0 && tick - prevFixTick > 100) {
                    double ddx = tx - prevFixX;
                    double ddy = ty - prevFixY;
                    if (ddx * ddx + ddy * ddy > 25) {
                        trackedHeading = Math.atan2(ddx, ddy);
                        if (trackedHeading < 0) trackedHeading += 2 * Math.PI;
                    }
                    prevFixX = tx;
                    prevFixY = ty;
                    prevFixTick = tick;
                } else if (Double.isNaN(prevFixX)) {
                    prevFixX = tx;
                    prevFixY = ty;
                    prevFixTick = tick;
                }

                trackedLastFixTick = tick;
            }

            // Passive contact slows uncertainty growth (compensate the growth added above)
            // Multiply effective growth by 0.5 by subtracting half of what was added
            if (hasTrackedContact) {
                uncertaintyRadius -= maxSubSpeed * dt * 0.5;
                if (uncertaintyRadius < 0) uncertaintyRadius = 0;
            }
        }

        // Fallback heading estimation from bearing stability in CHASE
        // Only use this when we have no ping fix history (pure passive tracking)
        if (hasTrackedContact && Double.isNaN(trackedHeading) && state == State.CHASE
                && pingFixHistory.isEmpty() && contactTrack.size() >= 3) {
            var latest = contactTrack.getLast();
            var older = contactTrack.get(contactTrack.size() - 3);
            double bearingDrift = Math.abs(angleDiff(latest.bearing(), older.bearing()));
            if (bearingDrift < Math.toRadians(10)) {
                trackedHeading = lastContactBearing;
            }
        }

        // Clear tracked contact if contactAlive too low
        if (hasTrackedContact && contactAlive < CONFIDENCE_LOST) {
            hasTrackedContact = false;
            trackedHeading = Double.NaN;
            prevFixX = Double.NaN;
            prevFixY = Double.NaN;
            prevFixTick = -1;
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
                if (tookDamage || shouldEvade()) {
                    enterState(State.EVADE, tick);
                } else if (bestContact != null && bestContact.isActive() && bestContact.range() > 0) {
                    enterState(State.CHASE, tick);
                } else if (estimatedRange < CHASE_RANGE && contactAlive >= CONFIDENCE_HUNT_MIN) {
                    enterState(State.CHASE, tick);
                } else if (hasTrackedContact && trackedDist < CHASE_RANGE && contactAlive >= CONFIDENCE_HUNT_MIN) {
                    enterState(State.CHASE, tick);
                } else if (!hasTrackedContact && ticksSinceContact > 500) {
                    enterState(State.PATROL, tick);
                }
            }
            case CHASE -> {
                if (tookDamage) {
                    enterState(State.EVADE, tick);
                } else if (estimatedRange < RAM_RANGE && rangeConfirmedByActive) {
                    enterState(State.RAM, tick);
                } else if (!hasTrackedContact) {
                    enterState(State.PATROL, tick);
                } else if (contactAlive < CONFIDENCE_HUNT_MIN) {
                    enterState(State.TRACKING, tick);
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
                if (navWaypoints.isEmpty()
                        || currentWaypointIndex >= navWaypoints.size() - 1) {
                    planPatrol(pos.x(), pos.y(), terrain, battleArea);
                    lastPlanTick = tick;
                }

                // Follow waypoints
                if (!navWaypoints.isEmpty()) {
                    var wp = navWaypoints.get(currentWaypointIndex);
                    double wpDx = wp.x() - pos.x();
                    double wpDy = wp.y() - pos.y();
                    double wpDist = Math.sqrt(wpDx * wpDx + wpDy * wpDy);

                    if (wpDist < WAYPOINT_ARRIVAL_DIST) {
                        currentWaypointIndex = (currentWaypointIndex + 1) % navWaypoints.size();
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
                tacticalThrottle = TRACKING_THROTTLE;

                // First-contact ping: if we don't have a calibrated SL yet,
                // ping once to establish range and calibrate. This is the
                // classic "snap shot" to establish the tactical picture,
                // then go passive for the approach.
                if (Double.isNaN(calibratedSL) && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }

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
                    boolean needReplan = navWaypoints.isEmpty()
                            || Double.isNaN(lastPlanTargetX)
                            || (Math.sqrt(Math.pow(trackedX - lastPlanTargetX, 2)
                                + Math.pow(trackedY - lastPlanTargetY, 2)) > 500);

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
                    } else {
                        double targetBearing = Math.atan2(trackedX - pos.x(), trackedY - pos.y());
                        if (targetBearing < 0) targetBearing += 2 * Math.PI;
                        double diff = angleDiff(targetBearing, heading);
                        if (Math.abs(diff) > Math.toRadians(5)) {
                            tacticalRudder = Math.clamp(diff * 2.0, -1, 1);
                        }
                    }

                    // Ping when within range of tracked position and cooldown ready
                    if (trackedDist < TRACKED_PING_RANGE && input.activeSonarCooldownTicks() == 0) {
                        output.activeSonarPing();
                    }
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

                if (behindTarget && trackedDist < 1000) {
                    // Behind and close: quiet tailing, no zig-zag needed
                    tacticalThrottle = TAILING_THROTTLE;
                } else if (hasTrackedContact && trackedDist > CHASE_RANGE) {
                    // Long-range approach: quiet patrol speed
                    tacticalThrottle = PATROL_THROTTLE;
                } else if (behindTarget && trackedDist < 2000) {
                    // Behind and moderately close: quiet approach
                    tacticalThrottle = CHASE_THROTTLE * 0.7;
                } else if (sprinting) {
                    tacticalThrottle = CHASE_THROTTLE;
                } else {
                    // Drift phase: go quiet, listen
                    tacticalThrottle = TRACKING_THROTTLE;
                }

                // Steering with zig-zag for TMA
                if (hasTrackedContact) {
                    double targetX = trackedX;
                    double targetY = trackedY;

                    // Aim for stern only at close range
                    if (!Double.isNaN(trackedHeading)
                            && trackedDist < 1500 && trackedDist > RAM_RANGE * 2) {
                        double offsetFraction = 1.0 - (trackedDist - RAM_RANGE * 2) / (1500 - RAM_RANGE * 2);
                        double offset = BEHIND_OFFSET * offsetFraction;
                        targetX -= offset * Math.sin(trackedHeading);
                        targetY -= offset * Math.cos(trackedHeading);
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

                    // Replan if target moved significantly
                    boolean needReplan = navWaypoints.isEmpty()
                            || Double.isNaN(lastPlanTargetX)
                            || (Math.sqrt(Math.pow(approachX - lastPlanTargetX, 2)
                                + Math.pow(approachY - lastPlanTargetY, 2)) > 500);

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
                    } else {
                        double diff = angleDiff(approachBearing, heading);
                        tacticalRudder = Math.clamp(diff * 2.0, -1, 1);
                    }

                    // Ping only as a last resort when passive tracking fails
                    if (trackedDist < TRACKED_PING_RANGE && input.activeSonarCooldownTicks() == 0
                            && ticksSinceContact > 500) {
                        output.activeSonarPing();
                    }
                } else if (lastContactTick >= 0) {
                    double diff = angleDiff(lastContactBearing, heading);
                    tacticalRudder = Math.clamp(diff * 2.0, -1, 1);
                }

                // Ping to confirm range when passive estimates suggest close
                if (!rangeConfirmedByActive && estimatedRange < CHASE_RANGE
                        && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }

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

                // Turn perpendicular to threat
                if (lastContactTick >= 0) {
                    double perpBearing = normalizeBearing(lastContactBearing + Math.PI / 2);
                    double diff = angleDiff(perpBearing, heading);
                    double perpBearing2 = normalizeBearing(lastContactBearing - Math.PI / 2);
                    double diff2 = angleDiff(perpBearing2, heading);
                    double targetDiff = Math.abs(diff) < Math.abs(diff2) ? diff : diff2;
                    tacticalRudder = Math.clamp(targetDiff * 2.0, -1, 1);
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
        double avoidanceThreshold = 30;
        double margin = depth - (worstFloor + floorClearance);

        if (margin < avoidanceThreshold) {
            double urgency = Math.clamp(1.0 - margin / avoidanceThreshold, 0.0, 1.0);
            status = "AVOIDING TERRAIN";

            if (worstTarget > rawTarget) {
                rawTarget = worstTarget;
            }

            boolean threatBelow = ((depth - floorBelow) < avoidanceThreshold);
            if (threatBelow) {
                throttle = Math.max(0.2, throttle * (1.0 - urgency * 0.6));
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

            // Only override rudder if the terrain threat is along our current
            // heading AND the waypoint direction isn't clearly safer.
            // If the waypoint direction has deep water, let the sub turn toward
            // it instead of fighting the navigation plan.
            // Only consider waypoint path clear when we actually have waypoints
            // in a different direction than our current heading.
            boolean hasDistinctWaypointPath = !navWaypoints.isEmpty()
                    && Math.abs(angleDiff(waypointHeading, scanHeading)) > Math.toRadians(15);
            boolean waypointPathClear = hasDistinctWaypointPath
                    && waypointWorstFloor < worstFloor - 10;

            if (!waypointPathClear) {
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
            // When waypoint path is clear, don't override rudder.
            // The depth controller (Step 8) still handles vertical safety.
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
        if (immediateGap < floorClearance * 1.5 && verticalSpeed < -0.5) {
            double pullUpUrgency = Math.clamp(
                    (floorClearance * 1.5 - immediateGap) / floorClearance, 0.2, 0.8);
            sternPlanes = Math.max(sternPlanes, pullUpUrgency);
            ballast = Math.max(ballast, 0.5 + pullUpUrgency * 0.3);
            throttle = Math.min(throttle, 0.3);
            status = "PULL UP";
        }

        // Step 10: Emergency pull-up
        if (immediateGap < EMERGENCY_GAP && depth < targetDepth - 5) {
            sternPlanes = 0.8;
            ballast = 0.9;
            throttle = -1.0;
            status = "EMERGENCY PULL UP";
        }

        // Step 11: Surface avoidance
        if (depth > MIN_DEPTH) {
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

        // Torpedo firing check: behind the target, within range, good solution
        if (hasTrackedContact && state == State.CHASE && isBehindTarget()
                && trackedDist < 1500 && trackedDist > 200
                && !Double.isNaN(trackedHeading) && trackedSpeed > 0
                && contactAlive > 0.5 && uncertaintyRadius < 300) {
            // We have a firing solution: behind the target, within range,
            // known heading and speed, confident position.
            double solutionAge = (tick - trackedLastFixTick) / 50.0;
            if (solutionAge < 30) { // solution less than 30 seconds old
                System.out.printf("TORPEDO SOLUTION t=%d: pos=[%.0f,%.0f] target=[%.0f,%.0f] " +
                        "range=%.0f hdg=%.0f spd=%.1f ur=%.0f behind=%s%n",
                        tick, pos.x(), pos.y(), trackedX, trackedY,
                        trackedDist, Math.toDegrees(trackedHeading), trackedSpeed,
                        uncertaintyRadius, isBehindTarget());
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
            contactTrack.clear();
            estimatedRange = Double.MAX_VALUE;
            rangeConfirmedByActive = false;
            calibratedSL = Double.NaN;
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
            prevFixX = Double.NaN;
            prevFixY = Double.NaN;
            prevFixTick = -1;
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
        // Evade if contact is loud and bearing rate is low (closing head-on)
        if (lastContactSE > EVADE_SE_THRESHOLD && contactTrack.size() >= 2) {
            var recent = contactTrack.getLast();
            var prev = contactTrack.get(contactTrack.size() - 2);
            long dt = recent.tick() - prev.tick();
            if (dt > 0) {
                double bearingRate = Math.abs(angleDiff(recent.bearing(), prev.bearing())) / dt;
                return bearingRate < EVADE_BEARING_RATE_THRESHOLD / 50.0; // per tick
            }
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

    // Triangulation

    static double triangulate(BearingFix a, BearingFix b) {
        // Two bearing lines from different positions: find intersection distance
        double dx = a.x() - b.x();
        double dy = a.y() - b.y();
        double displacement = Math.sqrt(dx * dx + dy * dy);
        if (displacement < 1.0) return Double.MAX_VALUE;

        double dBearing = angleDiff(a.bearing(), b.bearing());
        if (Math.abs(dBearing) < Math.toRadians(0.5)) return Double.MAX_VALUE;

        // Simplified: range = displacement / sin(delta_bearing)
        return displacement / Math.abs(Math.sin(dBearing));
    }

    // Passive range estimation

    // Noise model constants (mirrored from SubmarinePhysics for SL estimation)
    private static final double BASE_SL_DB = 80.0;
    private static final double SPEED_NOISE_DB_PER_MS = 2.0;

    /**
     * Estimate range from signal excess using the propagation model in reverse.
     * If blade-rate speed estimate is available, computes a better SL.
     * Otherwise falls back to assuming patrol-speed SL.
     * SE = SL - TL - NL, where TL = spreading * log10(r)
     * so r = 10^((SL - SE - NL) / spreading)
     */
    static double estimateRangeFromSE(double signalExcess, double estimatedTargetSpeed) {
        double sl;
        if (estimatedTargetSpeed >= 0) {
            sl = BASE_SL_DB + SPEED_NOISE_DB_PER_MS * estimatedTargetSpeed;
        } else {
            sl = ASSUMED_TARGET_SL_DB;
        }
        double tl = sl - signalExcess - ASSUMED_AMBIENT_NL_DB;
        if (tl <= 0) return 1.0;
        return Math.pow(10, tl / SPREADING_COEFFICIENT);
    }

    /**
     * Estimate range using the sonar-estimated source level directly.
     * Much more accurate than assuming a standard SL, especially for
     * surface ships and other non-submarine contacts.
     */
    static double estimateRangeFromSEAndSL(double signalExcess, double estimatedSourceLevel) {
        double tl = estimatedSourceLevel - signalExcess - ASSUMED_AMBIENT_NL_DB;
        if (tl <= 0) return 1.0;
        return Math.pow(10, tl / SPREADING_COEFFICIENT);
    }

    /**
     * Estimate range from bearing rate (Target Motion Analysis).
     * When own-ship has cross-track speed relative to the contact bearing,
     * range is approximately cross_speed / bearing_rate.
     * Needs at least TMA_HISTORY_TICKS of bearing history.
     */
    private double estimateRangeFromBearingRate(long tick, double ownSpeed, double ownHeading) {
        if (contactTrack.size() < 2 || ownSpeed < 0.5) return -1;

        // Find a fix at least TMA_HISTORY_TICKS old
        var latest = contactTrack.getLast();
        BearingFix older = null;
        for (int i = contactTrack.size() - 2; i >= 0; i--) {
            if (latest.tick() - contactTrack.get(i).tick() >= TMA_HISTORY_TICKS) {
                older = contactTrack.get(i);
                break;
            }
        }
        if (older == null) return -1;

        long dtTicks = latest.tick() - older.tick();
        double dtSeconds = dtTicks / 50.0;
        double bearingRate = Math.abs(angleDiff(latest.bearing(), older.bearing())) / dtSeconds;

        if (bearingRate < MIN_BEARING_RATE) return -1; // too slow to estimate

        // Cross-track speed: component of own velocity perpendicular to contact bearing
        double crossSpeed = ownSpeed * Math.abs(Math.sin(ownHeading - latest.bearing()));
        if (crossSpeed < 0.3) return -1; // heading straight at/away from contact

        return crossSpeed / bearingRate;
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
        if (navWaypoints.isEmpty()) {
            planPatrol(posX, posY, terrain, area);
            return;
        }
        var currentWp = navWaypoints.get(currentWaypointIndex);
        // Remove everything after the current waypoint
        while (navWaypoints.size() > currentWaypointIndex + 1) {
            navWaypoints.removeLast();
        }
        // Plan new waypoints from the current waypoint's position
        var oldPlan = new ArrayList<>(navWaypoints);
        planPatrol(currentWp.x(), currentWp.y(), terrain, area);
        // Prepend the preserved current waypoint
        navWaypoints.addFirst(oldPlan.get(currentWaypointIndex));
        // currentWaypointIndex stays the same (still heading toward the preserved waypoint)
    }

    /**
     * Replans a route to a target, preserving the current waypoint and replacing
     * everything after it with a new route from the current waypoint to the target.
     */
    private void replanToTarget(double posX, double posY, double targetX, double targetY,
                                 TerrainMap terrain, long tick) {
        if (navWaypoints.isEmpty()) {
            // No existing plan: plan from current position
            var route = planRoute(posX, posY, targetX, targetY, terrain);
            if (!route.isEmpty()) {
                navWaypoints.addAll(route);
                currentWaypointIndex = 0;
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
     * Plans a route from (fromX, fromY) to (toX, toY), inserting detour
     * waypoints where the sea floor is too shallow for safe passage.
     * Returns a list of Vec3 waypoints with safe depths.
     */
    List<Vec3> planRoute(double fromX, double fromY, double toX, double toY,
                         TerrainMap terrain) {
        var result = new ArrayList<Vec3>();
        planRouteRecursive(fromX, fromY, toX, toY, terrain, result, 0);
        // Add the final destination
        double endDepth = safeDepthAt(toX, toY, terrain);
        result.add(new Vec3(toX, toY, endDepth));
        return result;
    }

    /**
     * Recursive route planner. Checks if the segment from (ax,ay) to (bx,by)
     * is blocked by shallow terrain. If blocked, finds a detour midpoint
     * and recursively checks each sub-segment.
     */
    private void planRouteRecursive(double ax, double ay, double bx, double by,
                                     TerrainMap terrain, List<Vec3> result, int depth) {
        if (depth > MAX_RECURSION_DEPTH) {
            // Max depth reached: just add the start point with safe depth
            double safeZ = safeDepthAt(ax, ay, terrain);
            result.add(new Vec3(ax, ay, safeZ));
            return;
        }

        // Check if the segment is blocked
        double dx = bx - ax;
        double dy = by - ay;
        double segLen = Math.sqrt(dx * dx + dy * dy);
        if (segLen < 10) {
            result.add(new Vec3(ax, ay, safeDepthAt(ax, ay, terrain)));
            return;
        }

        int samples = Math.max(2, (int) Math.ceil(segLen / ROUTE_SAMPLE_STEP));
        boolean blocked = false;
        double blockedT = 0;

        for (int i = 1; i < samples; i++) {
            double t = (double) i / samples;
            double sx = ax + dx * t;
            double sy = ay + dy * t;
            double floor = terrain.elevationAt(sx, sy);
            // Blocked if the floor is above SHALLOW_WATER_LIMIT
            // (meaning less than 50m of water above the floor)
            if (floor > SHALLOW_WATER_LIMIT) {
                blocked = true;
                blockedT = t;
                break;
            }
        }

        if (!blocked) {
            // Segment is clear: add start point with safe depth
            result.add(new Vec3(ax, ay, safeDepthAt(ax, ay, terrain)));
            return;
        }

        // Segment is blocked: find a detour by offsetting the midpoint perpendicular
        double midX = ax + dx * 0.5;
        double midY = ay + dy * 0.5;

        // Perpendicular direction (normalized)
        double perpX = -dy / segLen;
        double perpY = dx / segLen;

        double[] offsets = {200, 400, 800, 1600};
        double bestOffsetX = midX;
        double bestOffsetY = midY;
        double bestFloor = Double.MAX_VALUE;
        boolean foundDetour = false;

        for (double offset : offsets) {
            // Try left
            double lx = midX + perpX * offset;
            double ly = midY + perpY * offset;
            double lFloor = terrain.elevationAt(lx, ly);

            // Try right
            double rx = midX - perpX * offset;
            double ry = midY - perpY * offset;
            double rFloor = terrain.elevationAt(rx, ry);

            // Pick the deeper side
            if (lFloor < SHALLOW_WATER_LIMIT && lFloor < bestFloor) {
                bestFloor = lFloor;
                bestOffsetX = lx;
                bestOffsetY = ly;
                foundDetour = true;
            }
            if (rFloor < SHALLOW_WATER_LIMIT && rFloor < bestFloor) {
                bestFloor = rFloor;
                bestOffsetX = rx;
                bestOffsetY = ry;
                foundDetour = true;
            }

            if (foundDetour) break; // use smallest offset that works
        }

        if (!foundDetour) {
            // Could not find a detour: just go straight with safe depth
            result.add(new Vec3(ax, ay, safeDepthAt(ax, ay, terrain)));
            return;
        }

        // Recursively check each sub-segment
        planRouteRecursive(ax, ay, bestOffsetX, bestOffsetY, terrain, result, depth + 1);
        planRouteRecursive(bestOffsetX, bestOffsetY, bx, by, terrain, result, depth + 1);
    }

    /**
     * Plans a patrol route that explores the arena.
     * The first waypoint heads toward the arena center (where contacts
     * are most likely), then subsequent waypoints fan out to cover ground.
     * If we have a last known contact area, the route biases toward it.
     */
    void planPatrol(double x, double y, TerrainMap terrain, BattleArea area) {
        navWaypoints.clear();
        currentWaypointIndex = 0;

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
                // Pull point inward toward center
                double scale = 0.7;
                px = px * scale;
                py = py * scale;
            }

            double safeZ = stealthDepthAt(px, py, terrain);
            rawPoints.add(new Vec3(px, py, safeZ));
        }

        // Build the final waypoint list, checking each leg for terrain obstacles
        for (int i = 0; i < rawPoints.size(); i++) {
            var from = (i == 0) ? new Vec3(x, y, safeDepthAt(x, y, terrain)) : rawPoints.get(i - 1);
            var to = rawPoints.get(i);

            var leg = planRoute(from.x(), from.y(), to.x(), to.y(), terrain);
            // planRoute includes the destination, add all points
            navWaypoints.addAll(leg);
        }

        // Remove duplicate waypoints that are very close together
        for (int i = navWaypoints.size() - 1; i > 0; i--) {
            var a = navWaypoints.get(i);
            var b = navWaypoints.get(i - 1);
            double dist = Math.sqrt(Math.pow(a.x() - b.x(), 2) + Math.pow(a.y() - b.y(), 2));
            if (dist < 50) {
                navWaypoints.remove(i);
            }
        }
    }

    /**
     * Returns the safe operating depth at a world position:
     * terrain elevation + floor clearance, clamped to operating limits.
     */
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

    private static double displacement(Vec3 pos, BearingFix fix) {
        double dx = pos.x() - fix.x();
        double dy = pos.y() - fix.y();
        return Math.sqrt(dx * dx + dy * dy);
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
