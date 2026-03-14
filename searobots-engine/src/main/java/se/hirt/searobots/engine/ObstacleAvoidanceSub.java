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

    // ── State machine ────────────────────────────────────────────────
    enum State { PATROL, TRACKING, CHASE, RAM, EVADE }

    // ── Throttle constants ───────────────────────────────────────────
    static final double PATROL_THROTTLE = 0.4;
    static final double TRACKING_THROTTLE = 0.25;
    static final double CHASE_THROTTLE = 0.8;
    static final double RAM_THROTTLE = 1.0;
    static final double EVADE_THROTTLE = 0.15;

    // ── Terrain / depth constants (unchanged from Phase 2) ───────────
    private static final double MIN_DEPTH = -20;
    private static final double FLOOR_CLEARANCE = 50;
    private static final double CRUSH_SAFETY_MARGIN = 50;
    private static final double BOUNDARY_TURN_DIST = 500;
    private static final double EMERGENCY_GAP = 40;
    private static final double[] SCAN_DISTANCES = {25, 50, 100, 200, 400, 600, 800};
    private static final double SCAN_SIDE_ANGLE = 0.6;
    private static final double HULL_HALF_LENGTH = 15.0; // must match SubmarinePhysics

    // ── Contact tracking constants ───────────────────────────────────
    static final int CONTACT_CONFIRM_TICKS = 3;
    static final int CONTACT_LOST_PATROL = 500;     // 10s
    static final int CONTACT_LOST_RAM = 250;         // 5s
    static final int CONTACT_LOST_EVADE = 1500;      // 30s
    static final double CHASE_RANGE = 3000.0;
    static final double RAM_RANGE = 500.0;
    static final double RAM_OVERSHOT_RANGE = 800.0;
    static final double TRACKING_DISPLACEMENT = 200.0;

    // ── Baffle clearing constants ────────────────────────────────────
    static final int BAFFLE_CLEAR_INTERVAL = 1500;   // 30s at 50Hz
    static final double BAFFLE_CLEAR_ANGLE = Math.toRadians(40);

    // ── Sprint-and-drift constants ───────────────────────────────────
    static final int SPRINT_DURATION = 750;           // 15s
    static final int DRIFT_DURATION = 1000;           // 20s

    // ── Pursuit / patrol ping constants ───────────────────────────────
    static final int CONTACT_LOST_CHASE = 1500;       // 30s — more persistent than TRACKING
    static final int PATROL_SILENCE_PING_TICKS = 3000; // 1 min at 50Hz
    private static final double BEHIND_OFFSET = 500.0;

    // ── Stern tailing constants ───────────────────────────────────────
    private static final double BEHIND_ARC = Math.toRadians(45);  // within 45° of target stern
    private static final double TAILING_THROTTLE = 0.30;          // quiet following speed
    private static final double TAILING_FLOOR_CLEARANCE = 100.0;  // extra room when tailing

    // ── Passive range estimation constants ──────────────────────────
    // SE-based: assume target SL ~90 dB (patrol speed), cylindrical spreading
    private static final double ASSUMED_TARGET_SL_DB = 90.0;
    private static final double ASSUMED_AMBIENT_NL_DB = 60.0;
    private static final double SPREADING_COEFFICIENT = 10.0; // cylindrical (must match SonarModel)
    // Bearing rate TMA: minimum bearing rate to produce useful estimate
    private static final double MIN_BEARING_RATE = Math.toRadians(0.05); // per second
    private static final int TMA_HISTORY_TICKS = 100; // 2 seconds of bearing history

    // ── Cavitation constants (mirrored from SubmarinePhysics) ────────
    private static final double BASE_CAVITATION_SPEED = 5.0;
    private static final double CAVITATION_DEPTH_FACTOR = 0.02;

    // ── Approach constants ─────────────────────────────────────────────
    static final long APPROACH_TIMEOUT = 15000;   // 5 min — max time to close on a ping fix

    // ── Evade constants ──────────────────────────────────────────────
    static final double EVADE_SE_THRESHOLD = 15.0;
    static final double EVADE_BEARING_RATE_THRESHOLD = Math.toRadians(2);

    // ── State ────────────────────────────────────────────────────────
    private State state = State.PATROL;
    private MatchConfig config;
    private TerrainMap terrain;
    private double depthLimit;
    private List<ThermalLayer> thermalLayers = List.of();

    // Contact tracking
    record BearingFix(long tick, double bearing, double se, double x, double y,
                      double ownSpeed, double ownHeading) {}
    enum ContactType { ACTIVE_PING, PASSIVE_SONAR }
    record ContactRecord(long tick, double estX, double estY, double confidence, ContactType type) {}
    private final List<BearingFix> contactTrack = new ArrayList<>();
    private final List<ContactRecord> contactLog = new ArrayList<>();
    private double lastContactBearing;
    private double lastContactSE;
    private long lastContactTick = -1000;
    private int consecutiveContactTicks;
    private double estimatedRange = Double.MAX_VALUE;
    private boolean rangeConfirmedByActive;
    private int previousHp;

    // Baffle clearing
    private long baffleClearTimer;
    private double preBaffleClearHeading;
    private boolean clearingBaffles;
    private int baffleClearDirection = 1;

    // Chase timing
    private long chaseStartTick;

    // Pursuit memory
    private double pursuitBearing = Double.NaN;
    private boolean hasPursuit;
    private long patrolSilenceTicks;

    // Target heading estimation
    private double estimatedTargetHeading = Double.NaN;
    private double prevEstTargetX = Double.NaN, prevEstTargetY = Double.NaN;
    private long prevEstTargetTick = -1;

    // Approach target (from precise ping fix)
    private double approachTargetX = Double.NaN, approachTargetY = Double.NaN;

    @Override
    public void onMatchStart(MatchContext context) {
        this.config = context.config();
        this.terrain = context.terrain();
        this.depthLimit = config.crushDepth() + CRUSH_SAFETY_MARGIN;
        this.thermalLayers = context.thermalLayers();
        this.previousHp = config.startingHp();
    }

    State state() { return state; }
    double estimatedRange() { return estimatedRange; }
    List<BearingFix> contactTrack() { return contactTrack; }
    double estimatedTargetHeading() { return estimatedTargetHeading; }
    boolean hasPursuit() { return hasPursuit; }
    double pursuitBearing() { return pursuitBearing; }
    List<ContactRecord> contactLog() { return contactLog; }

    @Override
    public void onTick(SubmarineInput input, SubmarineOutput output) {
        var self = input.self();
        var pos = self.pose().position();
        double heading = self.pose().heading();
        double depth = pos.z();
        long tick = input.tick();

        // ═══════════════════════════════════════════════════════════════
        // Step 1: Process sonar contacts and update contact track
        // ═══════════════════════════════════════════════════════════════
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
                updateTargetHeadingEstimate(pos.x(), pos.y(), tick);
            }

            // Record bearing fix (for triangulation and TMA)
            contactTrack.add(new BearingFix(tick, bestContact.bearing(), bestContact.signalExcess(),
                    pos.x(), pos.y(), speed, heading));
            // Cap track length
            while (contactTrack.size() > 200) contactTrack.removeFirst();

            // --- Passive range estimation (3 methods, best wins) ---

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
                            updateTargetHeadingEstimate(pos.x(), pos.y(), tick);
                        }
                        break;
                    }
                }
            }

            // Method 2: SE-based range estimate (rough — order of magnitude only)
            // Use blade-rate speed estimate for better SL assumption when available
            double seRange = estimateRangeFromSE(bestContact.signalExcess(),
                    bestContact.estimatedSpeed());
            if (seRange > 0 && seRange < 50000) {
                // SE range is imprecise — only use it when we have no other estimate,
                // and never let it alone drive us below CHASE_RANGE (could be wildly off)
                if (estimatedRange == Double.MAX_VALUE) {
                    estimatedRange = Math.max(seRange, CHASE_RANGE);
                    // Don't call updateTargetHeadingEstimate — SE range is too imprecise
                    // for heading estimation (would pollute reference position)
                }
            }

            // Method 3: Bearing rate TMA (needs bearing history and own-ship speed)
            double tmaRange = estimateRangeFromBearingRate(tick, speed, heading);
            if (tmaRange > 0 && tmaRange < 50000) {
                // TMA is more reliable than SE when we have good bearing rate data
                estimatedRange = tmaRange;
                updateTargetHeadingEstimate(pos.x(), pos.y(), tick);
            }

            // ── Record to contact log ──
            if (bestContact.isActive() && bestContact.range() > 0) {
                double tx = pos.x() + bestContact.range() * Math.sin(bestContact.bearing());
                double ty = pos.y() + bestContact.range() * Math.cos(bestContact.bearing());
                contactLog.add(new ContactRecord(tick, tx, ty, 0.95, ContactType.ACTIVE_PING));
                approachTargetX = tx;
                approachTargetY = ty;
            } else if (estimatedRange > 0 && estimatedRange < 50000) {
                double tx = pos.x() + estimatedRange * Math.sin(lastContactBearing);
                double ty = pos.y() + estimatedRange * Math.cos(lastContactBearing);
                double conf = Math.clamp((bestContact.signalExcess() - 5.0) / 30.0, 0.05, 0.5);
                contactLog.add(new ContactRecord(tick, tx, ty, conf, ContactType.PASSIVE_SONAR));
                approachTargetX = tx;
                approachTargetY = ty;
            }
            while (contactLog.size() > 500) contactLog.removeFirst();
        } else {
            consecutiveContactTicks = 0;
        }

        // Detect damage taken
        boolean tookDamage = self.hp() < previousHp;
        previousHp = self.hp();

        // ═══════════════════════════════════════════════════════════════
        // Step 2: State transitions
        // ═══════════════════════════════════════════════════════════════
        long ticksSinceContact = tick - lastContactTick;

        switch (state) {
            case PATROL -> {
                // Active ping return → immediate CHASE (bypass TRACKING confirmation)
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
                } else if (estimatedRange < CHASE_RANGE) {
                    enterState(State.CHASE, tick);
                } else if (ticksSinceContact > CONTACT_LOST_PATROL) {
                    enterState(State.PATROL, tick);
                }
            }
            case CHASE -> {
                boolean isApproaching = !Double.isNaN(approachTargetX);
                long lostTimeout = isApproaching ? APPROACH_TIMEOUT : CONTACT_LOST_CHASE;
                if (tookDamage) {
                    enterState(State.EVADE, tick);
                } else if (estimatedRange < RAM_RANGE && rangeConfirmedByActive) {
                    enterState(State.RAM, tick);
                } else if (ticksSinceContact > lostTimeout) {
                    enterState(State.PATROL, tick);
                }
            }
            case RAM -> {
                if (estimatedRange > RAM_OVERSHOT_RANGE && ticksSinceContact > 50) {
                    enterState(State.PATROL, tick);
                } else if (ticksSinceContact > CONTACT_LOST_RAM) {
                    enterState(State.PATROL, tick);
                }
            }
            case EVADE -> {
                if (ticksSinceContact > CONTACT_LOST_EVADE) {
                    enterState(State.PATROL, tick);
                } else if (bestContact != null && lastContactSE < 8.0 && ticksSinceContact == 0) {
                    // Contact weakening — hunter passing by, flip the script
                    enterState(State.TRACKING, tick);
                }
            }
        }

        // ═══════════════════════════════════════════════════════════════
        // Step 3: Tactical layer (throttle, rudder, depth preference)
        // ═══════════════════════════════════════════════════════════════
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

                if (hasPursuit) {
                    // Head toward last known contact area — use contact log for precise position
                    ContactRecord bestFix = getBestRecentContact(tick);
                    double targetBearing;
                    if (bestFix != null) {
                        targetBearing = Math.atan2(bestFix.estX() - pos.x(), bestFix.estY() - pos.y());
                        if (targetBearing < 0) targetBearing += 2 * Math.PI;
                    } else {
                        targetBearing = pursuitBearing;
                    }
                    double diff = angleDiff(targetBearing, heading);
                    if (Math.abs(diff) > Math.toRadians(5)) {
                        tacticalRudder = Math.clamp(diff * 2.0, -1, 1);
                    }
                } else {
                    // Normal baffle clearing
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
                }

                // Active ping gambit after long silence
                if (patrolSilenceTicks > PATROL_SILENCE_PING_TICKS
                        && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                    patrolSilenceTicks = 0;
                    hasPursuit = false;
                }

                // Prefer depth below thermocline
                tacticalDepthPreference = preferredDepthBelowThermocline(depth);
            }
            case TRACKING -> {
                tacticalThrottle = TRACKING_THROTTLE;

                // Turn perpendicular to contact bearing for cross-bearing fix
                if (lastContactTick >= 0) {
                    double perpBearing = normalizeBearing(lastContactBearing + Math.PI / 2);
                    double diff = angleDiff(perpBearing, heading);
                    // Also consider the other perpendicular direction
                    double perpBearing2 = normalizeBearing(lastContactBearing - Math.PI / 2);
                    double diff2 = angleDiff(perpBearing2, heading);
                    double targetDiff = Math.abs(diff) < Math.abs(diff2) ? diff : diff2;
                    tacticalRudder = Math.clamp(targetDiff * 2.0, -1, 1);
                }
            }
            case CHASE -> {
                boolean behindTarget = isBehindTarget();
                boolean isApproaching = !Double.isNaN(approachTargetX);

                if (behindTarget && estimatedRange < 1000) {
                    // Behind and close — quiet tailing
                    tacticalThrottle = TAILING_THROTTLE;
                } else if (isApproaching && estimatedRange > CHASE_RANGE) {
                    // Long-range approach from ping — quiet patrol speed
                    // At this distance our noise won't reach the target
                    tacticalThrottle = PATROL_THROTTLE;
                } else if (behindTarget) {
                    // Behind but far — close the gap, moderate speed
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

                // Steer toward target position (approach-aware)
                if (lastContactTick >= 0 || isApproaching) {
                    double targetX, targetY;
                    boolean havePosition = false;

                    if (estimatedRange < 50000 && ticksSinceContact < 100) {
                        // Fresh contact with range — compute position from current data
                        targetX = pos.x() + estimatedRange * Math.sin(lastContactBearing);
                        targetY = pos.y() + estimatedRange * Math.cos(lastContactBearing);
                        havePosition = true;
                    } else if (isApproaching) {
                        // No fresh contact — head toward stored approach target
                        targetX = approachTargetX;
                        targetY = approachTargetY;
                        havePosition = true;
                    } else {
                        targetX = targetY = 0;
                    }

                    double chaseBearing;
                    if (havePosition) {
                        // If we know target heading and have range, aim for stern
                        if (!Double.isNaN(estimatedTargetHeading)
                                && estimatedRange < 10000 && estimatedRange > RAM_RANGE * 2) {
                            double offset = Math.min(estimatedRange * 0.5, BEHIND_OFFSET);
                            targetX -= offset * Math.sin(estimatedTargetHeading);
                            targetY -= offset * Math.cos(estimatedTargetHeading);
                        }
                        chaseBearing = Math.atan2(targetX - pos.x(), targetY - pos.y());
                        if (chaseBearing < 0) chaseBearing += 2 * Math.PI;
                    } else {
                        chaseBearing = lastContactBearing;
                    }

                    double diff = angleDiff(chaseBearing, heading);
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
                if (!behindTarget && !(isApproaching && estimatedRange > CHASE_RANGE)) {
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

                // Steer directly at contact
                if (lastContactTick >= 0) {
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

        // ═══════════════════════════════════════════════════════════════
        // Steps 4-12: Safety pipeline
        // ═══════════════════════════════════════════════════════════════
        double rudder = tacticalRudder;
        double sternPlanes = 0;
        double throttle = tacticalThrottle;
        double ballast = 0.5;
        String status = state.name();

        // Use larger floor clearance when tailing — more room to maneuver
        boolean tailing = (state == State.CHASE && isBehindTarget());
        double floorClearance = tailing ? TAILING_FLOOR_CLEARANCE : FLOOR_CLEARANCE;

        // ── Step 4: Compute target depth (bottom tracking + tactical) ──
        // Check terrain at hull extent (bow, stern, beam) — same points the
        // physics checks for collision. Without this, a ridge just ahead can
        // kill the sub while the controller only sees the deep floor below center.
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        double floorCenter = terrain.elevationAt(pos.x(), pos.y());
        double floorBow = terrain.elevationAt(
                pos.x() + sinH * HULL_HALF_LENGTH, pos.y() + cosH * HULL_HALF_LENGTH);
        double floorStern = terrain.elevationAt(
                pos.x() - sinH * HULL_HALF_LENGTH, pos.y() - cosH * HULL_HALF_LENGTH);
        double floorBelow = Math.max(floorCenter, Math.max(floorBow, floorStern));
        double rawTarget = clampTarget(floorBelow + floorClearance);

        // Blend with tactical depth preference: use the SHALLOWER of the two
        // so tactical depth never violates floor safety margin.
        // In deep water (floor -500m, target -450m), tactical -150m wins → sub
        // cruises at -150m instead of hugging the floor.
        // In shallow water (floor -120m, target -70m), floor target -70m wins →
        // sub stays safe even if tactical wants -150m.
        if (!Double.isNaN(tacticalDepthPreference)) {
            double tacticalTarget = clampTarget(tacticalDepthPreference);
            if (tacticalTarget > rawTarget) {
                rawTarget = tacticalTarget;
            }
        }

        // ── Step 5: Multi-point terrain scan ahead ──
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

        // ── Step 6: Terrain avoidance ──
        double worstTarget = clampTarget(worstFloor + floorClearance);
        double margin = depth - (worstFloor + floorClearance);

        if (margin < floorClearance) {
            double urgency = Math.clamp(1.0 - margin / floorClearance, 0.2, 1.0);
            status = "AVOIDING TERRAIN";

            if (worstTarget > rawTarget) {
                rawTarget = worstTarget;
            }

            // Only reduce throttle when the floor is close BELOW us (vertical threat).
            // When the threat is ahead (worstDist > 0), we need speed for turning —
            // reducing throttle kills rudder authority and increases turn radius.
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

        // ── Step 7: Drop-off slowdown ──
        if (dropOffAhead) {
            throttle = Math.min(throttle, 0.3);
            if (state == State.PATROL) status = "DROP-OFF AHEAD";
        }

        // ── Step 8: PD depth controller ──
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

        // ── Step 9: Pull-up (gap closing while sinking) ──
        if (immediateGap < floorClearance * 1.5 && verticalSpeed < -0.5) {
            double pullUpUrgency = Math.clamp(
                    (floorClearance * 1.5 - immediateGap) / floorClearance, 0.2, 0.8);
            sternPlanes = Math.max(sternPlanes, pullUpUrgency);
            ballast = Math.max(ballast, 0.5 + pullUpUrgency * 0.3);
            throttle = Math.min(throttle, 0.3);
            status = "PULL UP";
        }

        // ── Step 10: Emergency pull-up ──
        if (immediateGap < EMERGENCY_GAP && depth < targetDepth - 5) {
            sternPlanes = 0.8;
            ballast = 0.9;
            throttle = -1.0;
            status = "EMERGENCY PULL UP";
        }

        // ── Step 11: Surface avoidance ──
        if (depth > MIN_DEPTH) {
            sternPlanes = -0.5;
            ballast = 0.2;
            status = "SURFACE AVOIDANCE";
        }

        // ── Step 12: Border avoidance ──
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

        // ═══════════════════════════════════════════════════════════════
        // Step 13: Output actuators
        // ═══════════════════════════════════════════════════════════════
        output.setRudder(rudder);
        output.setSternPlanes(sternPlanes);
        output.setThrottle(throttle);
        output.setBallast(ballast);
        String stateTag = state.name().substring(0, 1);
        if (tailing) stateTag = "C/TAIL";
        else if (!status.equals(state.name())) stateTag += "/" + status.charAt(0);
        output.setStatus(String.format("%s f:%.0f g:%.0f",
                stateTag, -floorBelow, immediateGap));
    }

    // ── State machine helpers ────────────────────────────────────────

    private void enterState(State newState, long tick) {
        state = newState;
        if (newState == State.PATROL) {
            // Preserve pursuit heading from last contact or approach target
            if (lastContactTick >= 0 && tick - lastContactTick < 10000) {
                pursuitBearing = lastContactBearing;
                hasPursuit = true;
            } else if (!Double.isNaN(approachTargetX)) {
                // Was approaching from a ping fix — head toward that area
                pursuitBearing = lastContactBearing;
                hasPursuit = true;
            } else {
                hasPursuit = false;
            }
            contactTrack.clear();
            estimatedRange = Double.MAX_VALUE;
            rangeConfirmedByActive = false;
            estimatedTargetHeading = Double.NaN;
            prevEstTargetX = Double.NaN;
            prevEstTargetY = Double.NaN;
            prevEstTargetTick = -1;
            approachTargetX = Double.NaN;
            approachTargetY = Double.NaN;
            clearingBaffles = false;
            baffleClearTimer = 0;
            patrolSilenceTicks = 0;
        } else if (newState == State.CHASE) {
            chaseStartTick = tick;
        }
    }

    private boolean isBehindTarget() {
        if (Double.isNaN(estimatedTargetHeading)) return false;
        // If target heading ≈ contact bearing, the target is heading away from us → we're behind
        double diff = Math.abs(angleDiff(estimatedTargetHeading, lastContactBearing));
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

    // ── Contact log queries ─────────────────────────────────────────

    private ContactRecord getBestRecentContact(long tick) {
        for (int i = contactLog.size() - 1; i >= 0; i--) {
            var r = contactLog.get(i);
            if (tick - r.tick() < APPROACH_TIMEOUT) {
                return r;
            }
        }
        return null;
    }

    // ── Target heading estimation ────────────────────────────────────

    private void updateTargetHeadingEstimate(double ourX, double ourY, long tick) {
        if (estimatedRange >= 50000) return;
        double tx = ourX + estimatedRange * Math.sin(lastContactBearing);
        double ty = ourY + estimatedRange * Math.cos(lastContactBearing);
        if (tick - prevEstTargetTick > 100 || Double.isNaN(prevEstTargetX)) {
            if (!Double.isNaN(prevEstTargetX)) {
                double dx = tx - prevEstTargetX;
                double dy = ty - prevEstTargetY;
                if (dx * dx + dy * dy > 25) { // moved at least 5m
                    estimatedTargetHeading = Math.atan2(dx, dy);
                    if (estimatedTargetHeading < 0) estimatedTargetHeading += 2 * Math.PI;
                }
            }
            prevEstTargetX = tx;
            prevEstTargetY = ty;
            prevEstTargetTick = tick;
        }

        // Fallback: if we're chasing and heading roughly toward the contact,
        // and bearing is stable, assume target is heading in the contact bearing direction
        if (Double.isNaN(estimatedTargetHeading) && state == State.CHASE
                && contactTrack.size() >= 3) {
            var latest = contactTrack.getLast();
            var older = contactTrack.get(contactTrack.size() - 3);
            double bearingDrift = Math.abs(angleDiff(latest.bearing(), older.bearing()));
            if (bearingDrift < Math.toRadians(10)) {
                // Bearing is stable — target is likely heading away from us
                estimatedTargetHeading = lastContactBearing;
            }
        }
    }

    // ── Triangulation ────────────────────────────────────────────────

    static double triangulate(BearingFix a, BearingFix b) {
        // Two bearing lines from different positions — find intersection distance
        double dx = a.x() - b.x();
        double dy = a.y() - b.y();
        double displacement = Math.sqrt(dx * dx + dy * dy);
        if (displacement < 1.0) return Double.MAX_VALUE;

        double dBearing = angleDiff(a.bearing(), b.bearing());
        if (Math.abs(dBearing) < Math.toRadians(0.5)) return Double.MAX_VALUE;

        // Simplified: range ≈ displacement / sin(Δbearing)
        return displacement / Math.abs(Math.sin(dBearing));
    }

    // ── Passive range estimation ────────────────────────────────────

    // Noise model constants (mirrored from SubmarinePhysics for SL estimation)
    private static final double BASE_SL_DB = 80.0;
    private static final double SPEED_NOISE_DB_PER_MS = 2.0;

    /**
     * Estimate range from signal excess using the propagation model in reverse.
     * If blade-rate speed estimate is available, computes a better SL.
     * Otherwise falls back to assuming patrol-speed SL.
     * SE = SL - TL - NL, where TL = spreading * log10(r)
     * → r = 10^((SL - SE - NL) / spreading)
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
     * range ≈ cross_speed / bearing_rate.
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

    // ── Geometry helpers ─────────────────────────────────────────────

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
