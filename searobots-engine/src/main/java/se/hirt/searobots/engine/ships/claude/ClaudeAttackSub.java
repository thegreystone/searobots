/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Claude-authored attack submarine. Stealth-optimized with full combat
 * capabilities: sonar tracking, pursuit, active pinging, firing solutions.
 */
public final class ClaudeAttackSub implements SubmarineController {

    // Patrol
    private static final double PATROL_MARGIN = 950.0;
    private static final double PROGRESS_THRESHOLD = 30.0;
    private static final long STUCK_TICKS = 1500;
    private static final double GOLDEN_ANGLE = 2.399963229728653;

    // Combat: aggressive detection and pursuit
    private static final int CONTACT_CONFIRM_TICKS = 2;   // faster confirmation
    private static final double CONTACT_DECAY = 0.9985;    // hold contacts longer
    private static final double LOST_TRACK_RADIUS = 5000.0;
    private static final double REPLAN_TARGET_MOVE = 250.0; // replan more often
    private static final long COMBAT_REPLAN_TICKS = 120;    // faster replanning
    private static final long PATROL_PING_INTERVAL = 600;   // ping more often
    private static final long STALE_CONTACT_TICKS = 150;    // refresh sooner
    private static final double CHASE_RANGE = 3500.0;       // enter chase earlier
    private static final double FIRING_RANGE = 2000.0;      // fire from further
    private static final double STERN_OFFSET = 450.0;       // wider stern approach

    private enum Mode { PATROL, TRACK, CHASE }

    private MatchConfig config;
    private TerrainMap terrain;
    private BattleArea battleArea;
    private PathPlanner pathPlanner;
    private ClaudeAutopilot autopilot;

    // Strategic state
    private List<StrategicWaypoint> strategicWaypoints = List.of();
    private long planCount;
    private long lastProgressTick;
    private double bestDistToGoal = Double.POSITIVE_INFINITY;
    private double lastTargetX = Double.NaN, lastTargetY = Double.NaN;

    // Objectives (injected by competition)
    private List<StrategicWaypoint> objectives = List.of();
    private int objectiveIndex = 0;

    // Combat state
    private Mode mode = Mode.PATROL, prevMode = Mode.PATROL;
    private boolean hasTrackedContact;
    private double trackedX = Double.NaN, trackedY = Double.NaN;
    private double trackedHeading = Double.NaN, trackedSpeed = 5.0;
    private double contactAlive = 0, uncertaintyRadius = 0, refUncertainty = 400;
    private double estimatedRange = Double.POSITIVE_INFINITY;
    private boolean rangeConfirmedByActive;
    private double lastContactBearing = Double.NaN, lastContactSE = 0;
    private long lastContactTick = Long.MIN_VALUE / 4;
    private long trackedLastFixTick = Long.MIN_VALUE / 4;
    private int consecutiveContactTicks;
    private long lastCombatPlanTick = Long.MIN_VALUE / 4;
    private long lastPingTick = Long.MIN_VALUE / 4;
    private double lastCombatTargetX = Double.NaN, lastCombatTargetY = Double.NaN;
    private long lastTorpedoLaunchTick = Long.MIN_VALUE / 4;
    private static final long TORPEDO_REFIRE_COOLDOWN = 750; // 15 seconds between launches (2 tubes)

    // Public accessors for tests
    public enum State { PATROL, TRACKING, CHASE, RAM, EVADE }
    public State state() {
        return switch (mode) {
            case PATROL -> State.PATROL;
            case TRACK -> State.TRACKING;
            case CHASE -> State.CHASE;
        };
    }
    public boolean hasTrackedContact() { return hasTrackedContact; }
    public double trackedX() { return trackedX; }
    public double trackedY() { return trackedY; }
    public double contactAlive() { return contactAlive; }
    public double estimatedRange() { return estimatedRange; }
    public double trackedHeading() { return trackedHeading; }
    public ClaudeAutopilot autopilot() { return autopilot; }
    public static final double TRACKING_THROTTLE = 0.25;
    public static final double RAM_THROTTLE = 1.0;
    public static final int BAFFLE_CLEAR_INTERVAL = 1500;
    public static final int PATROL_SILENCE_PING_TICKS = 3000;
    public static double angleDiff(double a, double b) { return ClaudeAutopilot.adiff(a, b); }
    public List<StrategicWaypoint> generatePatrolWaypoints(double x, double y, double h, BattleArea a) {
        return List.of(planPatrolWaypoint(x, y, h, 0));
    }
    public List<StrategicWaypoint> generateTrackingWaypoints(double px, double py, double h, double cb, TrackedContact c) { return List.of(); }
    public List<StrategicWaypoint> generateChaseWaypoints(double px, double py, double h, TrackedContact c, boolean b) { return List.of(); }
    public List<StrategicWaypoint> generateEvadeWaypoints(double px, double py, double cb, TerrainMap t) { return List.of(); }

    @Override public String name() { return "Claude Sub"; }

    @Override
    public TorpedoController createTorpedoController() {
        return new ClaudeTorpedoController();
    }

    @Override
    public void onMatchStart(MatchContext context) {
        config = context.config();
        terrain = context.terrain();
        battleArea = context.config().battleArea();
        pathPlanner = new PathPlanner(context.terrain(), -90.0, 225, 75, 400.0, 5.0);
        autopilot = new ClaudeAutopilot(context);
        strategicWaypoints = List.of();
        planCount = 0;
        lastProgressTick = 0;
        bestDistToGoal = Double.POSITIVE_INFINITY;
        mode = Mode.PATROL;
        prevMode = Mode.PATROL;
        clearTrack();
    }

    @Override
    public void setObjectives(java.util.List<StrategicWaypoint> objectives) {
        this.objectives = List.copyOf(objectives);
        this.objectiveIndex = 0;
    }

    @Override
    public void onTick(SubmarineInput input, SubmarineOutput output) {
        var self = input.self();
        var pos = self.pose().position();
        double heading = self.pose().heading();
        double speed = self.velocity().speed();
        long tick = input.tick();

        // ── Combat: track contacts ──
        SonarContact best = pickBestContact(input.sonarContacts(), input.activeSonarReturns());
        updateTrackedContact(best, pos, tick);
        updateMode(pos, tick);
        boolean modeChanged = mode != prevMode;

        // ── Objectives take priority ──
        boolean hasObj = !objectives.isEmpty() && objectiveIndex < objectives.size();
        if (hasObj) {
            if (autopilot.hasArrived()) {
                objectiveIndex++;
                hasObj = objectiveIndex < objectives.size();
            }
            // Stuck detection for objectives (same as patrol)
            boolean needPlan = strategicWaypoints.isEmpty() || autopilot.hasArrived()
                    || autopilot.isBlocked();
            if (!needPlan && !strategicWaypoints.isEmpty()) {
                double dist = autopilot.distanceToStrategic(pos.x(), pos.y());
                if (dist + PROGRESS_THRESHOLD < bestDistToGoal) {
                    bestDistToGoal = dist;
                    lastProgressTick = tick;
                } else if (tick - lastProgressTick > STUCK_TICKS) {
                    needPlan = true;
                }
            }
            if (hasObj && needPlan) {
                var orig = objectives.get(objectiveIndex);
                var wp = new StrategicWaypoint(orig.x(), orig.y(), orig.preferredDepth(),
                        orig.purpose(), NoisePolicy.SPRINT, orig.pattern(), orig.arrivalRadius(), 12.0);
                strategicWaypoints = List.of(wp);
                autopilot.setWaypoints(strategicWaypoints,
                        pos.x(), pos.y(), pos.z(), heading, speed, true);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
            }
        } else if (mode != Mode.PATROL && hasTrackedContact) {
            // ── Combat waypoint planning ──
            boolean needPlan = modeChanged || strategicWaypoints.isEmpty()
                    || autopilot.isBlocked() || autopilot.hasArrived();

            if (!needPlan && !Double.isNaN(lastCombatTargetX)) {
                double moved = ClaudeAutopilot.hdist(trackedX, trackedY,
                        lastCombatTargetX, lastCombatTargetY);
                if (moved > REPLAN_TARGET_MOVE) needPlan = true;
                else if (tick - lastCombatPlanTick > COMBAT_REPLAN_TICKS
                        && uncertaintyRadius > 700) needPlan = true;
            }

            if (needPlan) {
                var wp = planCombatWaypoint(pos.x(), pos.y(), pos.z(), heading, speed);
                strategicWaypoints = List.of(wp);
                autopilot.setWaypoints(strategicWaypoints,
                        pos.x(), pos.y(), pos.z(), heading, speed);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
                lastCombatPlanTick = tick;
                lastCombatTargetX = wp.x();
                lastCombatTargetY = wp.y();
            }
        } else {
            // ── Normal patrol ──
            boolean needPlan = modeChanged || strategicWaypoints.isEmpty()
                    || autopilot.isBlocked() || autopilot.hasArrived();

            if (!needPlan && !strategicWaypoints.isEmpty()) {
                double dist = autopilot.distanceToStrategic(pos.x(), pos.y());
                if (dist + PROGRESS_THRESHOLD < bestDistToGoal) {
                    bestDistToGoal = dist;
                    lastProgressTick = tick;
                } else if (tick - lastProgressTick > STUCK_TICKS) {
                    needPlan = true;
                }
            }

            if (needPlan) {
                var wp = planPatrolWaypoint(pos.x(), pos.y(), heading, speed);
                strategicWaypoints = List.of(wp);
                autopilot.setWaypoints(strategicWaypoints,
                        pos.x(), pos.y(), pos.z(), heading, speed);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
            }
        }

        autopilot.tick(input, output);

        // ── Active sonar ──
        handlePing(input, output, pos, tick);

        // ── Firing solution + torpedo launch ──
        publishFiringSolution(output, pos, tick);
        launchTorpedoIfReady(input, output, pos, tick);

        // ── Contact estimate ──
        if (hasTrackedContact) {
            double confidence = refUncertainty / Math.max(refUncertainty + uncertaintyRadius, 1);
            output.publishContactEstimate(new ContactEstimate(
                    trackedX, trackedY, contactAlive * confidence, contactAlive,
                    uncertaintyRadius, trackedHeading, trackedSpeed,
                    rangeConfirmedByActive ? "ping" : "passive"));
        }

        // ── Status ──
        double floor = terrain.elevationAt(pos.x(), pos.y());
        double gap = pos.z() - floor;
        output.setStatus(String.format("%s/%s f:%.0f g:%.0f",
                hasObj ? "OBJ" : mode.name().substring(0, 1),
                autopilot.lastStatus(), -floor, gap));

        // Publish all objectives (so the viewer shows all of them)
        for (int i = 0; i < objectives.size(); i++) {
            var obj = objectives.get(i);
            boolean active = i == objectiveIndex;
            boolean reached = i < objectiveIndex;
            output.publishStrategicWaypoint(
                    new Waypoint(obj.x(), obj.y(), obj.preferredDepth(), active),
                    reached ? Purpose.RALLY : obj.purpose());
        }
        // Also publish current patrol/combat waypoint if not in objective mode
        if (objectives.isEmpty() || objectiveIndex >= objectives.size()) {
            for (int i = 0; i < strategicWaypoints.size(); i++) {
                var wp = strategicWaypoints.get(i);
                output.publishStrategicWaypoint(
                        new Waypoint(wp.x(), wp.y(), wp.preferredDepth(),
                                i == autopilot.currentWaypointIndex()), wp.purpose());
            }
        }
        prevMode = mode;
    }

    // ── Contact tracking ────────────────────────────────────────────

    private SonarContact pickBestContact(List<SonarContact> passive, List<SonarContact> active) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (var c : active) {
            double s = 1000 + c.signalExcess();
            if (s > bestScore) { best = c; bestScore = s; }
        }
        for (var c : passive) {
            if (c.signalExcess() > bestScore) { best = c; bestScore = c.signalExcess(); }
        }
        return best;
    }

    private void updateTrackedContact(SonarContact contact, Vec3 pos, long tick) {
        double dt = 1.0 / 50;
        if (hasTrackedContact) {
            if (!Double.isNaN(trackedHeading)) {
                trackedX += trackedSpeed * Math.sin(trackedHeading) * dt;
                trackedY += trackedSpeed * Math.cos(trackedHeading) * dt;
            }
            uncertaintyRadius += config.maxSubSpeed() * dt;
            contactAlive *= CONTACT_DECAY;
            if (tick - trackedLastFixTick > 500) rangeConfirmedByActive = false;
        }

        if (contact == null) {
            consecutiveContactTicks = 0;
            if (hasTrackedContact && uncertaintyRadius > LOST_TRACK_RADIUS) clearTrack();
            return;
        }

        lastContactBearing = contact.bearing();
        lastContactSE = contact.signalExcess();
        lastContactTick = tick;
        consecutiveContactTicks++;
        contactAlive = 1.0;

        if (contact.estimatedSpeed() >= 0) {
            double q = contact.isActive() ? 0.85 : Math.clamp(contact.signalExcess() / 25, 0.15, 0.45);
            trackedSpeed = hasTrackedContact ? trackedSpeed * (1 - q) + contact.estimatedSpeed() * q
                    : contact.estimatedSpeed();
        }
        if (!Double.isNaN(contact.estimatedHeading())) trackedHeading = contact.estimatedHeading();

        double range = contact.range() > 50 ? contact.range()
                : hasTrackedContact ? ClaudeAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY)
                : estimatePassiveRange(contact);
        double tx = pos.x() + range * Math.sin(contact.bearing());
        double ty = pos.y() + range * Math.cos(contact.bearing());

        if (hasTrackedContact) {
            double blend = contact.isActive() ? 0.85 : 0.22;
            trackedX = trackedX * (1 - blend) + tx * blend;
            trackedY = trackedY * (1 - blend) + ty * blend;
        } else {
            trackedX = tx; trackedY = ty; hasTrackedContact = true;
        }

        if (contact.isActive() && contact.range() > 50) {
            estimatedRange = contact.range();
            rangeConfirmedByActive = true;
            trackedLastFixTick = tick;
            uncertaintyRadius = Math.max(80, contact.rangeUncertainty() > 0
                    ? contact.rangeUncertainty() * 2 : 90);
            refUncertainty = uncertaintyRadius;
        } else {
            double spread = Math.max(450,
                    range * Math.max(contact.bearingUncertainty(), Math.toRadians(6)) * 2);
            uncertaintyRadius = hasTrackedContact
                    ? Math.min(Math.max(uncertaintyRadius, 200), spread) : spread;
        }

        if (uncertaintyRadius > LOST_TRACK_RADIUS) clearTrack();
    }

    private void updateMode(Vec3 pos, long tick) {
        if (!hasTrackedContact) { mode = Mode.PATROL; return; }
        long since = tick - lastContactTick;
        double dist = ClaudeAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        if (rangeConfirmedByActive || (dist < CHASE_RANGE && contactAlive > 0.18)
                || (uncertaintyRadius < 300 && since < 250)) {
            mode = Mode.CHASE;
        } else if (consecutiveContactTicks >= CONTACT_CONFIRM_TICKS
                || (contactAlive > 0.05 && since < 1000)) {
            mode = Mode.TRACK;
        } else {
            mode = Mode.PATROL;
        }
    }

    private void clearTrack() {
        hasTrackedContact = false;
        trackedX = trackedY = trackedHeading = Double.NaN;
        trackedSpeed = 5; contactAlive = 0; uncertaintyRadius = 0;
        estimatedRange = Double.POSITIVE_INFINITY;
        rangeConfirmedByActive = false; consecutiveContactTicks = 0;
    }

    private double estimatePassiveRange(SonarContact c) {
        return Math.clamp(2600 - Math.min(c.signalExcess(), 20) * 85, 700, 2600);
    }

    // ── Active sonar ────────────────────────────────────────────────

    private void handlePing(SubmarineInput input, SubmarineOutput output, Vec3 pos, long tick) {
        if (input.activeSonarCooldownTicks() > 0) return;
        long since = tick - lastPingTick;
        double dist = hasTrackedContact
                ? ClaudeAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY) : Double.MAX_VALUE;
        long sinceFix = tick - trackedLastFixTick;

        boolean shouldPing = false;
        if (!hasTrackedContact) {
            shouldPing = since >= PATROL_PING_INTERVAL;
        } else if (mode == Mode.CHASE) {
            shouldPing = dist < FIRING_RANGE + 200 || sinceFix > STALE_CONTACT_TICKS || uncertaintyRadius > 450;
        } else {
            shouldPing = since >= PATROL_PING_INTERVAL / 2;
        }
        if (shouldPing) { output.activeSonarPing(); lastPingTick = tick; }
    }

    // ── Firing solution ─────────────────────────────────────────────

    private void publishFiringSolution(SubmarineOutput output, Vec3 pos, long tick) {
        if (!hasTrackedContact || contactAlive < 0.30) return;
        double dist = ClaudeAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        long sinceFix = tick - trackedLastFixTick;
        boolean fresh = rangeConfirmedByActive && sinceFix < 350;
        boolean behind = isBehindTarget(pos.x(), pos.y());
        // Relax precision when behind target (better geometry = more confident)
        double precisionLimit = fresh ? (behind ? 320 : 280) : (behind ? 180 : 140);
        if (uncertaintyRadius > precisionLimit || dist < 150 || dist > FIRING_RANGE) return;

        double bonus = behind ? 0.20 : 0;
        double quality = Math.clamp(contactAlive * (1 - uncertaintyRadius / 350 + bonus), 0.1, 1.0);
        output.publishFiringSolution(new FiringSolution(
                trackedX, trackedY, trackedHeading,
                trackedSpeed > 0 ? trackedSpeed : -1, quality));
    }

    private void launchTorpedoIfReady(SubmarineInput input, SubmarineOutput output,
                                      Vec3 pos, long tick) {
        if (input.self().torpedoesRemaining() <= 0) return;
        if (tick < 500) return; // don't fire in the first 10 seconds
        if (tick - lastTorpedoLaunchTick < TORPEDO_REFIRE_COOLDOWN) return;
        if (!hasTrackedContact) return;
        if (contactAlive < 0.50) return; // need solid contact
        if (mode != Mode.CHASE) return; // only fire when actively chasing
        if (!rangeConfirmedByActive) return; // need an active fix before firing

        double dist = ClaudeAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        // Torpedo range: ~3000m at 25 m/s * 120s fuel.
        // Min 500m (arming distance), max 2500m (need fuel margin for chase)
        if (dist < 500 || dist > 2500) return;
        if (uncertaintyRadius > 300) return;

        // Compute bearing to target
        double bearing = Math.atan2(trackedX - pos.x(), trackedY - pos.y());
        if (bearing < 0) bearing += 2 * Math.PI;

        // Lead the target: estimate where it will be when torpedo arrives
        double torpSpeed = 25.0; // approximate torpedo speed
        double timeToTarget = dist / torpSpeed;
        double leadX, leadY;
        if (!Double.isNaN(trackedHeading) && trackedSpeed > 0) {
            leadX = trackedX + Math.sin(trackedHeading) * trackedSpeed * timeToTarget;
            leadY = trackedY + Math.cos(trackedHeading) * trackedSpeed * timeToTarget;
        } else {
            leadX = trackedX;
            leadY = trackedY;
        }
        double leadBearing = Math.atan2(leadX - pos.x(), leadY - pos.y());
        if (leadBearing < 0) leadBearing += 2 * Math.PI;

        // Build mission data with target info
        // Target depth: send a moderate depth (~80m), most targets are at patrol depth
        // or shallower. The torpedo will adjust when it gets active sonar fixes.
        double targetDepth = -80;
        String missionData = String.format("%.0f,%.0f,%.0f,%.4f,%.1f",
                leadX, leadY, targetDepth,
                Double.isNaN(trackedHeading) ? 0 : trackedHeading,
                trackedSpeed > 0 ? trackedSpeed : 5.0);

        // Only fire when our heading is roughly aligned with the target
        // (torpedo tubes are forward-facing)
        double headingError = leadBearing - input.self().pose().heading();
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;
        if (Math.abs(headingError) > Math.toRadians(30)) return; // not aligned enough

        output.launchTorpedo(new TorpedoLaunchCommand(
                leadBearing, 0, 20.0, missionData));
        lastTorpedoLaunchTick = tick;
        System.out.printf("[Claude] Torpedo launched at tick %d, target=(%.0f,%.0f) dist=%.0fm hdgErr=%.1f°%n",
                tick, leadX, leadY, dist, Math.toDegrees(headingError));
    }

    private boolean isBehindTarget(double x, double y) {
        if (Double.isNaN(trackedHeading)) return false;
        double stern = ClaudeAutopilot.norm(trackedHeading + Math.PI);
        double own = ClaudeAutopilot.norm(Math.atan2(x - trackedX, y - trackedY));
        return Math.abs(ClaudeAutopilot.adiff(own, stern)) < Math.toRadians(60);
    }

    // ── Combat waypoint planning ────────────────────────────────────

    private StrategicWaypoint planCombatWaypoint(double x, double y, double z,
                                                  double heading, double speed) {
        double dist = ClaudeAutopilot.hdist(x, y, trackedX, trackedY);
        double cruiseDepth = autopilot.cruiseDepth();
        double tx = trackedX, ty = trackedY;
        double depth = safeDepth(tx, ty, Math.min(cruiseDepth, -160));
        Purpose purpose = Purpose.INTERCEPT;
        NoisePolicy noise = dist > 3500 ? NoisePolicy.SPRINT : NoisePolicy.NORMAL;
        double targetSpeed = dist > 3500 ? 11.0 : dist > 1800 ? 9.5 : 6.5;
        double arrival = dist > 2000 ? 260 : 180;

        if (mode == Mode.TRACK && !rangeConfirmedByActive) {
            // Cross-bearing approach for TMA
            double bearing = !Double.isNaN(lastContactBearing) ? lastContactBearing
                    : ClaudeAutopilot.norm(Math.atan2(trackedX - x, trackedY - y));
            double sA = ClaudeAutopilot.norm(bearing + Math.PI / 2);
            double sB = ClaudeAutopilot.norm(bearing - Math.PI / 2);
            double chosen = Math.abs(ClaudeAutopilot.adiff(sA, heading))
                    < Math.abs(ClaudeAutopilot.adiff(sB, heading)) ? sA : sB;
            double crossDist = Math.clamp(dist * 0.25, 350, 850);
            tx = x + Math.sin(chosen) * crossDist;
            ty = y + Math.cos(chosen) * crossDist;
            depth = safeDepth(tx, ty, Math.min(cruiseDepth, -180));
            purpose = Purpose.INVESTIGATE;
            noise = NoisePolicy.QUIET;
            targetSpeed = 6.0;
            arrival = 240;
        } else if (!Double.isNaN(trackedHeading) && trackedSpeed > 0.5) {
            // Shadow at torpedo engagement range (1500-2500m), don't close in
            // Stay behind the target for best torpedo geometry
            double predict = Math.clamp(dist / Math.max(speed, 4.5), 6, 35);
            tx += Math.sin(trackedHeading) * trackedSpeed * predict * 0.6;
            ty += Math.cos(trackedHeading) * trackedSpeed * predict * 0.6;

            // Maintain standoff: if too close, back off to stern at safe distance
            double desiredDist = 2000; // ideal shadow distance
            double minDist = 1200; // never closer than this
            if (dist < minDist) {
                // Back away: go to a point behind the target at desired distance
                double awayBearing = ClaudeAutopilot.norm(
                        Math.atan2(x - trackedX, y - trackedY)); // bearing FROM target TO us
                tx = trackedX + Math.sin(awayBearing) * desiredDist;
                ty = trackedY + Math.cos(awayBearing) * desiredDist;
                noise = NoisePolicy.QUIET;
                targetSpeed = 7.0;
                purpose = Purpose.EVADE;
            } else if (dist < 2500) {
                // Good range: get behind the target for torpedo shot
                double sternX = tx - Math.sin(trackedHeading) * STERN_OFFSET;
                double sternY = ty - Math.cos(trackedHeading) * STERN_OFFSET;
                if (battleArea.distanceToBoundary(sternX, sternY) > PATROL_MARGIN / 2
                        && pathPlanner.isSafe(sternX, sternY)) {
                    tx = sternX; ty = sternY;
                }
                noise = NoisePolicy.QUIET;
                targetSpeed = 6.5;
            }
            depth = safeDepth(tx, ty, Math.min(cruiseDepth, -170));
        }

        if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN
                || !pathPlanner.isSafe(tx, ty)) {
            // Don't chase into the boundary; pull back toward center
            tx = trackedX; ty = trackedY;
            if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN) {
                // Target is too close to edge; move toward center instead
                double pullDist = 500;
                tx = tx * (1.0 - pullDist / Math.max(Math.hypot(tx, ty), 1));
                ty = ty * (1.0 - pullDist / Math.max(Math.hypot(tx, ty), 1));
            }
            depth = safeDepth(tx, ty, Math.min(cruiseDepth, -150));
        }

        return new StrategicWaypoint(tx, ty, depth, purpose, noise,
                MovementPattern.DIRECT, arrival, targetSpeed);
    }

    // ── Patrol waypoint planning ────────────────────────────────────

    private StrategicWaypoint planPatrolWaypoint(double x, double y, double heading, double speed) {
        double toCenter = ClaudeAutopilot.norm(Math.atan2(-x, -y));
        double sweepBase = ClaudeAutopilot.norm(toCenter + planCount * GOLDEN_ANGLE);
        double cruiseDepth = autopilot != null ? autopilot.cruiseDepth() : -350;
        double desiredDist = planCount == 0 ? 1800 : 2200;

        var bearings = new ArrayList<Double>();
        bearings.add(heading);
        for (double off : new double[]{20, 40, 65, 90}) {
            bearings.add(ClaudeAutopilot.norm(heading + Math.toRadians(off)));
            bearings.add(ClaudeAutopilot.norm(heading - Math.toRadians(off)));
        }
        bearings.add(sweepBase);
        bearings.add(ClaudeAutopilot.norm(sweepBase + Math.toRadians(40)));
        bearings.add(ClaudeAutopilot.norm(sweepBase - Math.toRadians(40)));
        bearings.add(toCenter);

        double[] distances = planCount == 0
                ? new double[]{1400, 1800, 2200}
                : new double[]{1800, 2200, 2800};

        double bestScore = Double.NEGATIVE_INFINITY;
        double bestX = x, bestY = y, bestDepth = cruiseDepth, bestSpeed = 8.0;

        for (double bearing : bearings) {
            for (double dist : distances) {
                double tx = x + Math.sin(bearing) * dist;
                double ty = y + Math.cos(bearing) * dist;
                if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN) continue;
                if (!pathPlanner.isSafe(tx, ty)) continue;

                double depth = safeDepth(tx, ty, cruiseDepth);
                var path = pathPlanner.findPath(x, y, tx, ty, depth,
                        ClaudeAutopilot.depthChangeRatio(9), ClaudeAutopilot.turnRadiusAtSpeed(9));
                if (path.isEmpty()) continue;

                double direct = ClaudeAutopilot.hdist(x, y, tx, ty);
                double routeLen = polylineLength(path);
                double pathRatio = routeLen / Math.max(direct, 1);
                if (pathRatio > 1.8) continue;

                double alignment = 1 + Math.cos(ClaudeAutopilot.adiff(bearing, heading));
                double sweep = 1 + Math.cos(ClaudeAutopilot.adiff(bearing, sweepBase));
                double floorDepth = Math.abs(terrain.elevationAt(tx, ty));
                double boundary = battleArea.distanceToBoundary(tx, ty);

                double score = alignment * 300 + sweep * 180
                        + Math.min(boundary, 2000) * 0.12
                        + Math.min(floorDepth, 600) * 0.7
                        - Math.abs(dist - desiredDist) * 0.15
                        - (pathRatio - 1) * 1400;

                if (!Double.isNaN(lastTargetX)) {
                    double sep = ClaudeAutopilot.hdist(tx, ty, lastTargetX, lastTargetY);
                    score -= Math.max(0, 1500 - sep) * 0.35;
                }

                if (score > bestScore) {
                    bestScore = score;
                    bestX = tx; bestY = ty; bestDepth = depth;
                    bestSpeed = pathRatio > 1.3 ? 7.5 : 8.0;
                }
            }
        }

        if (bestScore == Double.NEGATIVE_INFINITY) {
            for (double d : new double[]{1500, 1000, 500}) {
                double tx = x + Math.sin(toCenter) * d;
                double ty = y + Math.cos(toCenter) * d;
                if (battleArea.distanceToBoundary(tx, ty) > PATROL_MARGIN / 2
                        && pathPlanner.isSafe(tx, ty)) {
                    bestX = tx; bestY = ty;
                    bestDepth = safeDepth(tx, ty, cruiseDepth);
                    bestSpeed = 7.5;
                    break;
                }
            }
        }

        planCount++;
        lastTargetX = bestX; lastTargetY = bestY;
        return new StrategicWaypoint(bestX, bestY, bestDepth, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, 250, bestSpeed);
    }

    // ── Utilities ───────────────────────────────────────────────────

    private double safeDepth(double x, double y, double cruiseDepth) {
        double floor = terrain.elevationAt(x, y);
        double target = Math.max(cruiseDepth, floor + 80);
        return Math.clamp(target, config.crushDepth() + 50, -50);
    }

    private double polylineLength(List<Vec3> path) {
        double len = 0;
        for (int i = 1; i < path.size(); i++) len += path.get(i - 1).horizontalDistanceTo(path.get(i));
        return len;
    }
}
