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
package se.hirt.searobots.engine.ships;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.SubmarineAutopilot;

import java.util.List;

/**
 * A smart submarine captain AI that behaves realistically:
 *
 * <ul>
 *   <li>Sprint-drift patrol below the thermocline (sprint deep, drift up to listen)</li>
 *   <li>Passive-only TMA: builds firing solutions through maneuvering, never pings</li>
 *   <li>Stalks targets from their stern quarter (baffles)</li>
 *   <li>Fires torpedoes from concealment, then goes silent</li>
 *   <li>Evades incoming torpedoes by turning toward them, crossing the thermocline,
 *       and ducking behind terrain</li>
 * </ul>
 */
public final class DefaultAttackSub implements SubmarineController {

    @Override
    public String name() { return "Default Sub"; }


    // ── State machine ──

    public enum State { PATROL, TRACKING, STALKING, ATTACKING, EVADING, REPOSITIONING }

    // ── Constants ──

    // Sprint-drift cycle
    private static final int SPRINT_TICKS = 750;         // 15 seconds
    private static final int DRIFT_TICKS = 1250;         // 25 seconds
    private static final double SPRINT_THROTTLE = 0.35;  // ~6 m/s, quiet repositioning
    private static final double QUIET_THROTTLE = 0.25;   // ~5 m/s, hard to detect
    private static final double THERMOCLINE_MARGIN = 30;  // meters below thermocline during sprint

    // TMA thresholds
    private static final double TMA_TRACKING_QUALITY = 0.3;  // enough to start stalking
    private static final double TMA_FIRING_QUALITY = 0.5;     // enough to fire
    private static final int CONTACT_CONFIRM_TICKS = 3;

    // Engagement
    private static final double STALKING_RANGE = 2000;   // desired firing distance
    private static final double MIN_FIRING_RANGE = 800;
    private static final double MAX_FIRING_RANGE = 2500;
    private static final double STERN_OFFSET = 500;       // how far behind target to position
    private static final long TORPEDO_COOLDOWN = 750;     // 15 seconds between launches

    // Torpedo classification
    private static final double TORPEDO_SL_THRESHOLD = 105; // dB, above = torpedo

    // Evasion and repositioning
    private static final int REPOSITION_TICKS = 2000;     // 40 seconds silent after attack
    private static final int EVADE_CLEAR_TICKS = 200;     // 4 seconds without torpedo = safe

    // Terrain / depth
    private static final double MIN_DEPTH = -25;
    private static final double FLOOR_CLEARANCE = 45;
    private static final double CRUSH_SAFETY = 50;
    private static final double PATROL_MARGIN = 900;

    // ── State ──

    private State state = State.PATROL;
    private MatchConfig config;
    private TerrainMap terrain;
    private BattleArea battleArea;
    private PathPlanner pathPlanner;
    private SubmarineAutopilot autopilot;
    private List<ThermalLayer> thermalLayers = List.of();
    private double thermoclineDepth = -200; // default if no layers
    private double depthLimit;

    // Sprint-drift: start with a short sprint to get moving, then alternate
    private long sprintDriftStart;
    private boolean inSprintPhase = true;

    // Ping evasion: cross thermocline when pinged
    private boolean wasPinged;
    private double pingerDepth = Double.NaN;
    private long lastPingedTick = Long.MIN_VALUE / 4;
    private boolean preferAboveThermocline = false; // which side to hide on

    // Contact tracking
    private boolean hasTrackedContact;
    private double trackedX = Double.NaN, trackedY = Double.NaN;
    private double trackedHeading = Double.NaN, trackedSpeed = 5;
    private double contactAlive;
    private double uncertaintyRadius;
    private double estimatedRange = Double.POSITIVE_INFINITY;
    private long lastContactTick = Long.MIN_VALUE / 4;
    private int consecutiveContactTicks;
    private double lastContactBearing = Double.NaN;
    private double bestSolutionQuality;

    // Torpedo threat
    private SonarContact torpedoThreat;
    private long lastTorpedoTick = Long.MIN_VALUE / 4;

    // Attack state
    private long lastTorpedoLaunchTick = Long.MIN_VALUE / 4;
    private long attackCompleteTick = Long.MIN_VALUE / 4;
    private int previousHp;

    // Strategic planning
    private List<StrategicWaypoint> strategicWaypoints = List.of();
    private long lastPlanTick = Long.MIN_VALUE / 4;
    private double lastPlanTargetX = Double.NaN, lastPlanTargetY = Double.NaN;
    private long planCount;

    // Objectives (from competition)
    private List<StrategicWaypoint> objectives = List.of();
    private int objectiveIndex;

    // ── Public accessors (for tests) ──

    public State state() { return state; }
    public boolean hasTrackedContact() { return hasTrackedContact; }
    public double trackedX() { return trackedX; }
    public double trackedY() { return trackedY; }
    public double contactAlive() { return contactAlive; }
    public double estimatedRange() { return estimatedRange; }
    public double trackedHeading() { return trackedHeading; }
    public SubmarineAutopilot autopilot() { return autopilot; }
    public static final double TRACKING_THROTTLE = QUIET_THROTTLE;
    public static final double RAM_THROTTLE = 1.0;
    public static final int BAFFLE_CLEAR_INTERVAL = 1500;
    public static final int PATROL_SILENCE_PING_TICKS = 3000;
    public static double angleDiff(double a, double b) {
        double d = a - b;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }
    public List<StrategicWaypoint> generatePatrolWaypoints(double x, double y, double h, BattleArea a) {
        return List.of(planPatrolWaypoint(x, y, h));
    }
    public List<StrategicWaypoint> generateEvadeWaypoints(double px, double py, double cb, TerrainMap t) {
        return List.of();
    }

    @Override
    public void setObjectives(java.util.List<StrategicWaypoint> objectives) {
        this.objectives = List.copyOf(objectives);
        this.objectiveIndex = 0;
    }

    @Override
    public void onMatchStart(MatchContext context) {
        config = context.config();
        terrain = context.terrain();
        battleArea = config.battleArea();
        thermalLayers = context.thermalLayers();
        depthLimit = config.crushDepth() + CRUSH_SAFETY;
        pathPlanner = new PathPlanner(terrain, -90.0, 225, 75, 400.0, 5.0);
        autopilot = new SubmarineAutopilot(context);
        previousHp = config.startingHp();

        // Find shallowest thermocline depth for sprint-drift cycle
        if (!thermalLayers.isEmpty()) {
            thermoclineDepth = thermalLayers.getFirst().depth();
            for (var l : thermalLayers) {
                if (l.depth() > thermoclineDepth) thermoclineDepth = l.depth();
            }
        }
    }

    // ── Main tick ──

    @Override
    public void onTick(SubmarineInput input, SubmarineOutput output) {
        var self = input.self();
        var pos = self.pose().position();
        double heading = self.pose().heading();
        double speed = self.velocity().speed();
        long tick = input.tick();

        // Step 1: Process sonar contacts
        SonarContact bestContact = pickBestSubContact(input.sonarContacts(), input.activeSonarReturns());
        torpedoThreat = detectTorpedoThreat(input.sonarContacts());
        updateTrackedContact(bestContact, pos, tick);

        // Step 2: Detect if we've been pinged (active return means someone pinged us)
        wasPinged = false;
        for (var c : input.activeSonarReturns()) {
            wasPinged = true;
            lastPingedTick = tick;
            if (!Double.isNaN(c.estimatedDepth())) {
                pingerDepth = c.estimatedDepth();
                // Pinger is below thermocline? Hide above. Pinger above? Hide below.
                preferAboveThermocline = pingerDepth < thermoclineDepth;
            }
            break; // one ping is enough
        }

        // Step 3: Check for damage (someone hit us)
        boolean tookDamage = self.hp() < previousHp;
        previousHp = self.hp();

        // Step 3: State transitions
        updateState(pos, tick, tookDamage);

        // Step 4: Handle objectives first (competition waypoints)
        boolean hasObj = !objectives.isEmpty() && objectiveIndex < objectives.size();
        if (hasObj) {
            if (autopilot.hasArrived()) {
                objectiveIndex++;
                hasObj = objectiveIndex < objectives.size();
            }
            if (hasObj) {
                var orig = objectives.get(objectiveIndex);
                double depth = safeDepth(orig.x(), orig.y(), belowThermocline());
                var wp = new StrategicWaypoint(orig.x(), orig.y(), depth,
                        orig.purpose(), NoisePolicy.NORMAL, orig.pattern(), orig.arrivalRadius(), 8.0);
                if (strategicWaypoints.isEmpty() || autopilot.hasArrived() || autopilot.isBlocked()) {
                    strategicWaypoints = List.of(wp);
                    autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), pos.z(), heading, speed);
                }
            }
        } else {
            // Step 5: State-specific waypoint generation
            boolean needPlan = stateChanged() || strategicWaypoints.isEmpty()
                    || autopilot.hasArrived() || autopilot.isBlocked()
                    || wasPinged; // immediately replan when pinged (change depth)
            if (!needPlan && hasTrackedContact) {
                double targetMoved = hdist(trackedX, trackedY, lastPlanTargetX, lastPlanTargetY);
                if (targetMoved > 300) needPlan = true;
            }
            if (needPlan && tick - lastPlanTick > 100) {
                strategicWaypoints = generateWaypoints(pos, heading, speed, tick);
                if (!strategicWaypoints.isEmpty()) {
                    autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), pos.z(), heading, speed);
                    lastPlanTick = tick;
                    if (hasTrackedContact) {
                        lastPlanTargetX = trackedX;
                        lastPlanTargetY = trackedY;
                    }
                }
            }
        }

        // Step 6: Run autopilot
        autopilot.tick(input, output);

        // Step 7: Sprint-drift and stalking throttle override
        applySprintDrift(output, tick, pos);

        // Step 8: Active sonar (almost never)
        // Ping defensively during evasion if we lost the torpedo track
        if (state == State.EVADING && torpedoThreat == null
                && tick - lastTorpedoTick < 500
                && input.activeSonarCooldownTicks() == 0) {
            output.activeSonarPing();
        }
        // One "snapshot" ping right before firing: confirm the solution.
        // We've been silent the whole approach; this one ping gives a precise
        // fix and we fire immediately after. The ping reveals us, but the
        // torpedoes are already in the water before the enemy can react.
        if (state == State.ATTACKING && input.activeSonarCooldownTicks() == 0
                && tick - lastTorpedoLaunchTick > TORPEDO_COOLDOWN) {
            output.activeSonarPing();
        }

        // Step 9: Torpedo launch
        launchTorpedoIfReady(input, output, pos, tick);

        // Step 10: Status and viewer data
        output.setStatus(String.format("%s %s",
                state.name().substring(0, Math.min(4, state.name().length())),
                inSprintPhase && state == State.PATROL ? "SPR" : ""));

        if (hasTrackedContact) {
            double confidence = Math.max(0, contactAlive * (1 - uncertaintyRadius / 3000));
            output.publishContactEstimate(new ContactEstimate(
                    trackedX, trackedY, confidence, contactAlive, uncertaintyRadius,
                    trackedHeading, trackedSpeed, "passive"));
        }

        // Publish strategic waypoints for viewer
        for (int i = 0; i < strategicWaypoints.size(); i++) {
            var wp = strategicWaypoints.get(i);
            output.publishStrategicWaypoint(
                    new Waypoint(wp.x(), wp.y(), wp.preferredDepth(),
                            i == autopilot.currentWaypointIndex()), wp.purpose());
        }
    }

    @Override
    public void onMatchEnd(MatchResult result) {}

    // ── Contact tracking ──

    private SonarContact pickBestSubContact(List<SonarContact> passive, List<SonarContact> active) {
        SonarContact best = null;
        double bestSE = 0;
        for (var c : active) {
            if (isConfirmedTorpedo(c)) continue; // skip confirmed torpedoes
            if (c.signalExcess() > bestSE) { best = c; bestSE = c.signalExcess(); }
        }
        for (var c : passive) {
            if (isConfirmedTorpedo(c)) continue;
            if (c.signalExcess() > bestSE) { best = c; bestSE = c.signalExcess(); }
        }
        return best;
    }

    /** A confirmed torpedo: loud AND fast. Loud but slow = surface ship or sprinting sub. */
    private boolean isConfirmedTorpedo(SonarContact c) {
        return c.estimatedSourceLevel() > TORPEDO_SL_THRESHOLD && c.estimatedSpeed() > 20;
    }

    private SonarContact detectTorpedoThreat(List<SonarContact> contacts) {
        for (var c : contacts) {
            // Torpedo signature: very loud (>105 dB) AND fast (>20 m/s).
            // Surface ships and sprinting subs are loud but not 20+ m/s.
            // Torpedoes cruise at 25 m/s, clearly distinct.
            if (c.estimatedSourceLevel() > TORPEDO_SL_THRESHOLD
                    && c.estimatedSpeed() > 20) {
                return c;
            }
        }
        return null;
    }

    private void updateTrackedContact(SonarContact contact, Vec3 pos, long tick) {
        double dt = 1.0 / 50;
        if (hasTrackedContact) {
            if (!Double.isNaN(trackedHeading)) {
                trackedX += trackedSpeed * Math.sin(trackedHeading) * dt;
                trackedY += trackedSpeed * Math.cos(trackedHeading) * dt;
            }
            uncertaintyRadius += config.maxSubSpeed() * dt;
            contactAlive *= 0.9985;
        }

        if (contact == null) {
            consecutiveContactTicks = 0;
            if (hasTrackedContact && uncertaintyRadius > 5000) clearTrack();
            return;
        }

        lastContactBearing = contact.bearing();
        lastContactTick = tick;
        consecutiveContactTicks++;
        contactAlive = 1.0;
        bestSolutionQuality = Math.max(bestSolutionQuality, contact.solutionQuality());

        if (contact.estimatedSpeed() >= 0) {
            double q = contact.isActive() ? 0.8 : Math.clamp(contact.signalExcess() / 25, 0.1, 0.4);
            trackedSpeed = hasTrackedContact ? trackedSpeed * (1 - q) + contact.estimatedSpeed() * q
                    : contact.estimatedSpeed();
        }
        if (!Double.isNaN(contact.estimatedHeading())) trackedHeading = contact.estimatedHeading();

        double range = contact.range() > 50 ? contact.range() : 2000;
        double tx = pos.x() + range * Math.sin(contact.bearing());
        double ty = pos.y() + range * Math.cos(contact.bearing());

        if (hasTrackedContact) {
            double blend = contact.isActive() ? 0.8 : 0.2;
            trackedX = trackedX * (1 - blend) + tx * blend;
            trackedY = trackedY * (1 - blend) + ty * blend;
        } else {
            trackedX = tx; trackedY = ty; hasTrackedContact = true;
        }

        estimatedRange = hdist(pos.x(), pos.y(), trackedX, trackedY);

        if (contact.rangeUncertainty() > 0) {
            uncertaintyRadius = contact.isActive() ?
                    contact.rangeUncertainty() * 2 : Math.min(uncertaintyRadius, range * 0.4);
        }
    }

    private void clearTrack() {
        hasTrackedContact = false;
        trackedX = trackedY = trackedHeading = Double.NaN;
        trackedSpeed = 5; contactAlive = 0; uncertaintyRadius = 0;
        estimatedRange = Double.POSITIVE_INFINITY;
        bestSolutionQuality = 0;
        consecutiveContactTicks = 0;
    }

    // ── State transitions ──

    private State prevState = State.PATROL;

    private boolean stateChanged() { return state != prevState; }

    private void updateState(Vec3 pos, long tick, boolean tookDamage) {
        prevState = state;

        // Torpedo threat always triggers evasion
        if (torpedoThreat != null) {
            lastTorpedoTick = tick;
            if (state != State.EVADING) {
                state = State.EVADING;
                return;
            }
        }
        if (tookDamage && state != State.EVADING) {
            state = State.EVADING;
            return;
        }

        switch (state) {
            case PATROL -> {
                if (consecutiveContactTicks >= CONTACT_CONFIRM_TICKS) {
                    // Contact detected: immediately stop sprinting and start tracking
                    if (inSprintPhase) {
                        inSprintPhase = false;
                        sprintDriftStart = tick;
                    }
                    state = State.TRACKING;
                }
            }
            case TRACKING -> {
                if (!hasTrackedContact) { state = State.PATROL; return; }
                if (bestSolutionQuality >= TMA_TRACKING_QUALITY
                        && !Double.isNaN(trackedHeading)) {
                    state = State.STALKING;
                }
            }
            case STALKING -> {
                if (!hasTrackedContact || contactAlive < 0.1) { state = State.PATROL; return; }
                if (bestSolutionQuality < TMA_TRACKING_QUALITY * 0.5) {
                    state = State.TRACKING; return;
                }
                double dist = hdist(pos.x(), pos.y(), trackedX, trackedY);
                // Only attack when: in range, good TMA, AND behind the target
                if (dist < MAX_FIRING_RANGE && dist > MIN_FIRING_RANGE
                        && bestSolutionQuality >= TMA_FIRING_QUALITY
                        && isBehindTarget(pos.x(), pos.y())) {
                    state = State.ATTACKING;
                }
            }
            case ATTACKING -> {
                // Transition handled in launchTorpedoIfReady
            }
            case EVADING -> {
                if (tick - lastTorpedoTick > EVADE_CLEAR_TICKS && torpedoThreat == null) {
                    state = State.REPOSITIONING;
                    attackCompleteTick = tick;
                }
            }
            case REPOSITIONING -> {
                if (tick - attackCompleteTick > REPOSITION_TICKS) {
                    state = State.PATROL;
                }
                if (hasTrackedContact && consecutiveContactTicks >= CONTACT_CONFIRM_TICKS) {
                    state = State.TRACKING;
                }
            }
        }
    }

    // ── Waypoint generation ──

    private List<StrategicWaypoint> generateWaypoints(Vec3 pos, double heading, double speed, long tick) {
        return switch (state) {
            case PATROL -> List.of(planPatrolWaypoint(pos.x(), pos.y(), heading));
            case TRACKING -> planTrackingWaypoints(pos, heading);
            case STALKING -> planStalkingWaypoint(pos, heading);
            case ATTACKING -> List.of(); // don't change waypoints during attack
            case EVADING -> planEvadeWaypoint(pos, heading);
            case REPOSITIONING -> planRepositionWaypoint(pos, heading);
        };
    }

    private StrategicWaypoint planPatrolWaypoint(double x, double y, double heading) {
        // Patrol at tactical depth (opposite thermocline side from enemy)
        double toCenter = norm(Math.atan2(-x, -y));
        double sweepBase = norm(toCenter + (planCount++) * 2.399963);
        double cruiseDepth = tacticalDepth(lastPingedTick > 0 ? lastPingedTick : 0);

        // Try several directions, prefer deep water and forward motion
        double bestX = x, bestY = y, bestDepth = cruiseDepth;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (double angle : new double[]{heading, sweepBase,
                norm(heading + 0.5), norm(heading - 0.5),
                norm(heading + 1.0), norm(heading - 1.0), toCenter}) {
            for (double dist : new double[]{1500, 2000, 2500}) {
                double tx = x + Math.sin(angle) * dist;
                double ty = y + Math.cos(angle) * dist;
                if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN) continue;
                if (!pathPlanner.isSafe(tx, ty)) continue;
                double floor = Math.abs(terrain.elevationAt(tx, ty));
                double boundary = battleArea.distanceToBoundary(tx, ty);
                double alignment = 1 + Math.cos(angleDiff(angle, heading));
                double score = floor * 1.5 + boundary * 0.1 + alignment * 200
                        - Math.abs(dist - 2000) * 0.1;
                if (score > bestScore) {
                    bestScore = score;
                    bestX = tx; bestY = ty;
                    bestDepth = safeDepth(tx, ty, cruiseDepth);
                }
            }
        }
        return new StrategicWaypoint(bestX, bestY, bestDepth, Purpose.PATROL,
                NoisePolicy.QUIET, MovementPattern.DIRECT, 250, 6.0);
    }

    private List<StrategicWaypoint> planTrackingWaypoints(Vec3 pos, double heading) {
        // Zig-zag perpendicular to contact bearing to build TMA
        double bearing = !Double.isNaN(lastContactBearing) ? lastContactBearing
                : norm(Math.atan2(trackedX - pos.x(), trackedY - pos.y()));

        // Pick the perpendicular direction closer to current heading
        double perpA = norm(bearing + Math.PI / 2);
        double perpB = norm(bearing - Math.PI / 2);
        double chosen = Math.abs(angleDiff(perpA, heading))
                < Math.abs(angleDiff(perpB, heading)) ? perpA : perpB;

        double dist = Math.clamp(estimatedRange * 0.2, 300, 800);
        double tx = pos.x() + Math.sin(chosen) * dist;
        double ty = pos.y() + Math.cos(chosen) * dist;
        double depth = safeDepth(tx, ty, tacticalDepth(lastPingedTick));

        return List.of(new StrategicWaypoint(tx, ty, depth, Purpose.INVESTIGATE,
                NoisePolicy.QUIET, MovementPattern.DIRECT, 200, 5.0));
    }

    private List<StrategicWaypoint> planStalkingWaypoint(Vec3 pos, double heading) {
        // Silently approach the target's stern quarter (their baffles).
        // This is the money maneuver: if we get behind them undetected,
        // they can't hear us and we have a clean shot.
        if (Double.isNaN(trackedHeading)) {
            return planTrackingWaypoints(pos, heading);
        }

        double dist = hdist(pos.x(), pos.y(), trackedX, trackedY);

        // If already behind the target, close in directly
        if (isBehindTarget(pos.x(), pos.y())) {
            double closeDist = Math.max(STALKING_RANGE, dist * 0.7);
            double bearing = norm(Math.atan2(trackedX - pos.x(), trackedY - pos.y()));
            double tx = pos.x() + Math.sin(bearing) * Math.min(closeDist, 800);
            double ty = pos.y() + Math.cos(bearing) * Math.min(closeDist, 800);
            double depth = safeDepth(tx, ty, tacticalDepth(lastPingedTick));
            // Creep in silently
            return List.of(new StrategicWaypoint(tx, ty, depth, Purpose.INTERCEPT,
                    NoisePolicy.SILENT, MovementPattern.DIRECT, 150, 3.5));
        }

        // Not behind yet: arc around to the stern. Pick the side that's
        // shorter to reach (left or right of the target's stern).
        double sternBearing = norm(trackedHeading + Math.PI);
        double toTargetBearing = norm(Math.atan2(trackedX - pos.x(), trackedY - pos.y()));

        // Offset to one side of the stern (whichever is closer to our current bearing)
        double sternLeft = norm(sternBearing - Math.toRadians(40));
        double sternRight = norm(sternBearing + Math.toRadians(40));
        double chosen = Math.abs(angleDiff(sternLeft, toTargetBearing))
                < Math.abs(angleDiff(sternRight, toTargetBearing)) ? sternLeft : sternRight;

        // Position behind the target at a comfortable standoff
        double approachDist = Math.max(STERN_OFFSET, dist * 0.5);
        double tx = trackedX + Math.sin(chosen) * approachDist;
        double ty = trackedY + Math.cos(chosen) * approachDist;

        if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN || !pathPlanner.isSafe(tx, ty)) {
            // Can't get behind: approach directly but quietly
            tx = trackedX + Math.sin(sternBearing) * STERN_OFFSET;
            ty = trackedY + Math.cos(sternBearing) * STERN_OFFSET;
        }

        double depth = safeDepth(tx, ty, tacticalDepth(lastPingedTick));

        // Quiet approach: slow, clutch engaged but minimal throttle
        return List.of(new StrategicWaypoint(tx, ty, depth, Purpose.INTERCEPT,
                NoisePolicy.QUIET, MovementPattern.DIRECT, 250, 4.5));
    }

    private List<StrategicWaypoint> planEvadeWaypoint(Vec3 pos, double heading) {
        // Turn TOWARD the torpedo (minimize cross-section, force overshoot)
        // Then cross the thermocline and go deep
        double evadeBearing;
        if (torpedoThreat != null) {
            evadeBearing = torpedoThreat.bearing(); // head toward the threat
        } else {
            evadeBearing = heading; // keep current heading if lost track
        }

        double dist = 500;
        double tx = pos.x() + Math.sin(evadeBearing) * dist;
        double ty = pos.y() + Math.cos(evadeBearing) * dist;

        // Try to find terrain cover
        for (double off : new double[]{0, Math.PI/4, -Math.PI/4, Math.PI/2, -Math.PI/2}) {
            double cx = pos.x() + Math.sin(evadeBearing + off) * 800;
            double cy = pos.y() + Math.cos(evadeBearing + off) * 800;
            double floor = terrain.elevationAt(cx, cy);
            if (floor > -80) { // shallow terrain = potential cover
                tx = cx; ty = cy; break;
            }
        }

        // Go as deep as possible
        double depth = safeDepth(tx, ty, depthLimit + 20);

        return List.of(new StrategicWaypoint(tx, ty, depth, Purpose.EVADE,
                NoisePolicy.SPRINT, MovementPattern.DIRECT, 150, 12.0));
    }

    private List<StrategicWaypoint> planRepositionWaypoint(Vec3 pos, double heading) {
        // Silent drift away from the engagement at 90 degrees
        double escapeDir = norm(heading + Math.PI / 2);
        double tx = pos.x() + Math.sin(escapeDir) * 1500;
        double ty = pos.y() + Math.cos(escapeDir) * 1500;

        if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN) {
            escapeDir = norm(heading - Math.PI / 2);
            tx = pos.x() + Math.sin(escapeDir) * 1500;
            ty = pos.y() + Math.cos(escapeDir) * 1500;
        }

        double depth = safeDepth(tx, ty, belowThermocline());

        return List.of(new StrategicWaypoint(tx, ty, depth, Purpose.PATROL,
                NoisePolicy.SILENT, MovementPattern.DIRECT, 300, 3.0));
    }

    // ── Sprint-drift cycle ──

    private void applySprintDrift(SubmarineOutput output, long tick, Vec3 pos) {
        if (state != State.PATROL) {
            output.setEngineClutch(true);

            // Silent running during repositioning
            if (state == State.REPOSITIONING) {
                output.setEngineClutch(false);
                output.setThrottle(0);
            }

            // Stalking: if we're in the target's baffles, they can't hear us.
            // Sprint in for the kill at moderate speed. If not yet behind them,
            // creep quietly to avoid detection.
            if (state == State.STALKING && hasTrackedContact) {
                double dist = hdist(pos.x(), pos.y(), trackedX, trackedY);
                if (isBehindTarget(pos.x(), pos.y())) {
                    // In the baffles: they're deaf to us. Move in confidently.
                    output.setEngineClutch(true);
                    output.setThrottle(dist > STALKING_RANGE ? 0.5 : 0.35);
                } else if (dist < STALKING_RANGE + 500) {
                    // Close but not behind yet: go silent, drift into position
                    output.setEngineClutch(false);
                    output.setThrottle(0);
                }
            }
            return;
        }

        long elapsed = tick - sprintDriftStart;
        if (inSprintPhase && elapsed >= SPRINT_TICKS) {
            inSprintPhase = false;
            sprintDriftStart = tick;
        } else if (!inSprintPhase && elapsed >= DRIFT_TICKS) {
            inSprintPhase = true;
            sprintDriftStart = tick;
        }

        if (inSprintPhase) {
            output.setEngineClutch(true);
            // Autopilot handles throttle, but ensure we're below thermocline
        } else {
            // Drift: cut engines, go silent, rise toward thermocline to listen
            output.setEngineClutch(false);
            output.setThrottle(0);
        }
    }

    // ── Torpedo launch ──

    private void launchTorpedoIfReady(SubmarineInput input, SubmarineOutput output,
                                       Vec3 pos, long tick) {
        if (state != State.ATTACKING && state != State.STALKING) return;
        if (input.self().torpedoesRemaining() <= 0) return;
        if (tick - lastTorpedoLaunchTick < TORPEDO_COOLDOWN) return;
        if (!hasTrackedContact || contactAlive < 0.4) return;
        if (bestSolutionQuality < TMA_FIRING_QUALITY) return;

        double dist = hdist(pos.x(), pos.y(), trackedX, trackedY);
        if (dist < MIN_FIRING_RANGE || dist > MAX_FIRING_RANGE) return;
        if (uncertaintyRadius > 250) return;

        // Compute lead position
        double torpSpeed = 25.0;
        double timeToTarget = dist / torpSpeed;
        double leadX = trackedX, leadY = trackedY;
        if (!Double.isNaN(trackedHeading) && trackedSpeed > 0.5 && trackedSpeed < 20) {
            double maxLead = dist * 0.4;
            double leadDist = Math.min(trackedSpeed * timeToTarget, maxLead);
            leadX = trackedX + Math.sin(trackedHeading) * leadDist;
            leadY = trackedY + Math.cos(trackedHeading) * leadDist;
        }

        double bearing = Math.atan2(leadX - pos.x(), leadY - pos.y());
        if (bearing < 0) bearing += 2 * Math.PI;

        // Check heading alignment (tubes are forward-facing)
        double headingError = angleDiff(bearing, input.self().pose().heading());
        if (Math.abs(headingError) > Math.toRadians(25)) return;

        // Target depth: estimate from our own depth and the thermocline.
        // If we're below the thermocline and can hear them, they're likely
        // near our depth or above the layer. Use a reasonable estimate.
        double targetDepth = Math.max(thermoclineDepth + 20, -150);
        String missionData = String.format("%.0f,%.0f,%.0f,%.4f,%.1f",
                leadX, leadY, targetDepth,
                Double.isNaN(trackedHeading) ? 0 : trackedHeading,
                trackedSpeed > 0.5 ? trackedSpeed : 5.0);

        output.launchTorpedo(new TorpedoLaunchCommand(bearing, 0, 20.0, missionData));
        lastTorpedoLaunchTick = tick;

        // After firing 2, go to repositioning
        int fired = config.torpedoCount() - input.self().torpedoesRemaining() + 1;
        if (fired % 2 == 0 || input.self().torpedoesRemaining() <= 1) {
            state = State.REPOSITIONING;
            attackCompleteTick = tick;
        }
    }

    // ── Depth helpers ──

    /** The preferred tactical depth: on the opposite side of the thermocline
     *  from the enemy when pinged, below it by default. */
    private double tacticalDepth(long tick) {
        boolean recentlyPinged = tick - lastPingedTick < 3000; // remember for 60s
        if (recentlyPinged && preferAboveThermocline) {
            // Hide above the thermocline (enemy is below)
            return Math.min(thermoclineDepth + THERMOCLINE_MARGIN, MIN_DEPTH);
        }
        // Default: below the thermocline (shielded from above)
        return Math.max(thermoclineDepth - THERMOCLINE_MARGIN, depthLimit);
    }

    /** Convenience: below the thermocline depth. */
    private double belowThermocline() {
        return Math.max(thermoclineDepth - THERMOCLINE_MARGIN, depthLimit);
    }

    private double safeDepth(double x, double y, double preferred) {
        double floor = terrain.elevationAt(x, y);
        double target = Math.max(preferred, floor + FLOOR_CLEARANCE);
        return Math.clamp(target, depthLimit, MIN_DEPTH);
    }

    // ── Geometry helpers ──

    private static double hdist(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1, dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** True if we are within 60 degrees of the target's stern arc (their baffles). */
    private boolean isBehindTarget(double x, double y) {
        if (Double.isNaN(trackedHeading)) return false;
        double sternBearing = norm(trackedHeading + Math.PI);
        double ourBearing = norm(Math.atan2(x - trackedX, y - trackedY));
        return Math.abs(angleDiff(ourBearing, sternBearing)) < Math.toRadians(60);
    }

    private static double norm(double a) {
        a %= 2 * Math.PI;
        if (a < 0) a += 2 * Math.PI;
        return a;
    }
}
