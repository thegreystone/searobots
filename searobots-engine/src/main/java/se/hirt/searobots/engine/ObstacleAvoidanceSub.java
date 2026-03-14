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

public final class ObstacleAvoidanceSub implements SubmarineController {

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
    private static final double BOUNDARY_TURN_DIST = 500;
    private static final double EMERGENCY_GAP = 40;
    private static final double[] SCAN_DISTANCES = {25, 50, 100, 200, 400, 600, 800};
    private static final double SCAN_SIDE_ANGLE = 0.6;
    private static final double HULL_HALF_LENGTH = 15.0; // must match SubmarinePhysics

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
    private static final double TAILING_FLOOR_CLEARANCE = 100.0;  // extra room when tailing

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

    // Patrol silence tracking
    private long patrolSilenceTicks;

    @Override
    public void onMatchStart(MatchContext context) {
        this.config = context.config();
        this.terrain = context.terrain();
        this.depthLimit = config.crushDepth() + CRUSH_SAFETY_MARGIN;
        this.maxSubSpeed = config.maxSubSpeed();
        this.thermalLayers = context.thermalLayers();
        this.previousHp = config.startingHp();
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
            }

            // Record bearing fix (for triangulation and TMA)
            contactTrack.add(new BearingFix(tick, bestContact.bearing(), bestContact.signalExcess(),
                    pos.x(), pos.y(), speed, heading));
            // Cap track length
            while (contactTrack.size() > 200) contactTrack.removeFirst();

            // Passive range estimation (3 methods, best wins)

            // Method 1: Triangulation (requires displacement between fixes)
            if (contactTrack.size() >= 2) {
                var latest = contactTrack.getLast();
                // Find an older fix with enough displacement
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

            // Method 2: SE-based range estimate (rough, order of magnitude only)
            double seRange = estimateRangeFromSE(bestContact.signalExcess(),
                    bestContact.estimatedSpeed());
            if (seRange > 0 && seRange < 50000) {
                if (estimatedRange == Double.MAX_VALUE) {
                    estimatedRange = Math.max(seRange, CHASE_RANGE);
                }
            }

            // Method 3: Bearing rate TMA (needs bearing history and own-ship speed)
            double tmaRange = estimateRangeFromBearingRate(tick, speed, heading);
            if (tmaRange > 0 && tmaRange < 50000) {
                estimatedRange = tmaRange;
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

                // Normal baffle clearing (PATROL means fully lost, no tracked contact)
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

                // Active ping gambit after long silence
                if (patrolSilenceTicks > PATROL_SILENCE_PING_TICKS
                        && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                    patrolSilenceTicks = 0;
                }

                // Prefer depth below thermocline
                tacticalDepthPreference = preferredDepthBelowThermocline(depth);
            }
            case TRACKING -> {
                tacticalThrottle = TRACKING_THROTTLE;

                if (bestContact != null) {
                    // Fresh contact: turn perpendicular for cross-bearing fix
                    double perpBearing = normalizeBearing(lastContactBearing + Math.PI / 2);
                    double diff = angleDiff(perpBearing, heading);
                    double perpBearing2 = normalizeBearing(lastContactBearing - Math.PI / 2);
                    double diff2 = angleDiff(perpBearing2, heading);
                    double targetDiff = Math.abs(diff) < Math.abs(diff2) ? diff : diff2;
                    tacticalRudder = Math.clamp(targetDiff * 2.0, -1, 1);
                } else if (hasTrackedContact) {
                    // No fresh contact but have tracked position: head toward it
                    double targetBearing = Math.atan2(trackedX - pos.x(), trackedY - pos.y());
                    if (targetBearing < 0) targetBearing += 2 * Math.PI;
                    double diff = angleDiff(targetBearing, heading);
                    if (Math.abs(diff) > Math.toRadians(5)) {
                        tacticalRudder = Math.clamp(diff * 2.0, -1, 1);
                    }

                    // Ping when within range of tracked position and cooldown ready
                    if (trackedDist < TRACKED_PING_RANGE && input.activeSonarCooldownTicks() == 0) {
                        output.activeSonarPing();
                    }
                }
            }
            case CHASE -> {
                boolean behindTarget = isBehindTarget();

                if (behindTarget && estimatedRange < 1000) {
                    // Behind and close: quiet tailing
                    tacticalThrottle = TAILING_THROTTLE;
                } else if (hasTrackedContact && trackedDist > CHASE_RANGE) {
                    // Long-range approach: quiet patrol speed
                    tacticalThrottle = PATROL_THROTTLE;
                } else if (behindTarget) {
                    // Behind but far: close the gap, moderate speed
                    tacticalThrottle = CHASE_THROTTLE * 0.7;
                } else {
                    // Sprint-and-drift to close / get behind
                    long phaseTime = (tick - chaseStartTick) % (SPRINT_DURATION + DRIFT_DURATION);
                    if (phaseTime < SPRINT_DURATION) {
                        tacticalThrottle = CHASE_THROTTLE;
                    } else {
                        tacticalThrottle = TRACKING_THROTTLE;
                    }
                }

                // Steer toward tracked position
                if (hasTrackedContact) {
                    double targetX = trackedX;
                    double targetY = trackedY;

                    // If we know target heading and have range, aim for stern
                    if (!Double.isNaN(trackedHeading)
                            && trackedDist < 10000 && trackedDist > RAM_RANGE * 2) {
                        double offset = Math.min(trackedDist * 0.5, BEHIND_OFFSET);
                        targetX -= offset * Math.sin(trackedHeading);
                        targetY -= offset * Math.cos(trackedHeading);
                    }

                    double chaseBearing = Math.atan2(targetX - pos.x(), targetY - pos.y());
                    if (chaseBearing < 0) chaseBearing += 2 * Math.PI;
                    double diff = angleDiff(chaseBearing, heading);
                    tacticalRudder = Math.clamp(diff * 2.0, -1, 1);

                    // Ping when within range and no recent fix
                    if (trackedDist < TRACKED_PING_RANGE && input.activeSonarCooldownTicks() == 0
                            && ticksSinceContact > 100) {
                        output.activeSonarPing();
                    }
                } else if (lastContactTick >= 0) {
                    // Fallback: steer by last known bearing
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

        // Step 4: Compute target depth (bottom tracking + tactical)
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        double floorCenter = terrain.elevationAt(pos.x(), pos.y());
        double floorBow = terrain.elevationAt(
                pos.x() + sinH * HULL_HALF_LENGTH, pos.y() + cosH * HULL_HALF_LENGTH);
        double floorStern = terrain.elevationAt(
                pos.x() - sinH * HULL_HALF_LENGTH, pos.y() - cosH * HULL_HALF_LENGTH);
        double floorBelow = Math.max(floorCenter, Math.max(floorBow, floorStern));
        double rawTarget = clampTarget(floorBelow + floorClearance);

        if (!Double.isNaN(tacticalDepthPreference)) {
            double tacticalTarget = clampTarget(tacticalDepthPreference);
            if (tacticalTarget > rawTarget) {
                rawTarget = tacticalTarget;
            }
        }

        // Step 5: Multi-point terrain scan ahead
        double worstFloor = floorBelow;
        double worstDist = 0;
        boolean dropOffAhead = false;
        for (double dist : SCAN_DISTANCES) {
            double sx = pos.x() + Math.sin(heading) * dist;
            double sy = pos.y() + Math.cos(heading) * dist;
            double floor = terrain.elevationAt(sx, sy);
            if (floor > worstFloor) {
                worstFloor = floor;
                worstDist = dist;
            }
            if (floor < floorBelow - 100) {
                dropOffAhead = true;
            }
        }

        // Step 6: Terrain avoidance
        double worstTarget = clampTarget(worstFloor + floorClearance);
        double margin = depth - (worstFloor + floorClearance);

        if (margin < floorClearance) {
            double urgency = Math.clamp(1.0 - margin / floorClearance, 0.2, 1.0);
            status = "AVOIDING TERRAIN";

            if (worstTarget > rawTarget) {
                rawTarget = worstTarget;
            }

            boolean threatBelow = ((depth - floorBelow) < floorClearance);
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

            if (floorLeft < floorRight) {
                rudder = -urgency;
            } else {
                rudder = urgency;
            }
        }

        double targetDepth = rawTarget;

        // Step 7: Drop-off slowdown
        if (dropOffAhead) {
            throttle = Math.min(throttle, 0.3);
            if (state == State.PATROL) status = "DROP-OFF AHEAD";
        }

        // Step 8: PD depth controller
        double immediateGap = depth - floorBelow;
        double depthError = depth - targetDepth;
        double verticalSpeed = self.velocity().linear().z();

        double gapFactor = Math.clamp(
                (immediateGap - EMERGENCY_GAP) / (200 - EMERGENCY_GAP), 0, 1);

        double pGain = 0.08;
        double dGain = 0.3;
        double rawBallast = 0.5 - pGain * depthError - dGain * verticalSpeed;
        if (rawBallast < 0.5) {
            ballast = Math.clamp(0.5 - (0.5 - rawBallast) * gapFactor, 0.0, 1.0);
        } else {
            ballast = Math.clamp(rawBallast, 0.0, 1.0);
        }

        if (depthError > 5) {
            double maxDive = -0.5 * gapFactor;
            sternPlanes = Math.clamp(-depthError * 0.03 * gapFactor, maxDive, 0);
        } else if (depthError < -5) {
            sternPlanes = Math.clamp(-depthError * 0.04, 0, 0.6);
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

        // Publish tracked contact estimate every tick if available
        if (hasTrackedContact) {
            double posConf = refUncertainty / (refUncertainty + uncertaintyRadius);
            double conf = contactAlive * posConf;
            String label = uncertaintyRadius < 100 ? "ping" : "passive";
            output.publishContactEstimate(new ContactEstimate(trackedX, trackedY, conf, contactAlive, uncertaintyRadius, label));
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
            clearingBaffles = false;
            baffleClearTimer = 0;
            patrolSilenceTicks = 0;
            // Clear tracked contact
            hasTrackedContact = false;
            trackedHeading = Double.NaN;
            contactAlive = 0;
            uncertaintyRadius = 0;
            prevFixX = Double.NaN;
            prevFixY = Double.NaN;
            prevFixTick = -1;
            pingFixHistory.clear();
        } else if (newState == State.CHASE) {
            chaseStartTick = tick;
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
            // Compute SL from blade-rate speed estimate (more accurate when close)
            sl = BASE_SL_DB + SPEED_NOISE_DB_PER_MS * estimatedTargetSpeed;
        } else {
            sl = ASSUMED_TARGET_SL_DB; // fallback: assume patrol speed
        }
        double tl = sl - signalExcess - ASSUMED_AMBIENT_NL_DB;
        if (tl <= 0) return 1.0; // very close
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
