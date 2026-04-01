package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class CodexAttackSub implements SubmarineController {
    private static final double PATROL_MARGIN = 1_050.0;
    private static final double PROGRESS_THRESHOLD = 30.0;
    private static final long STUCK_TICKS = 1_500;
    private static final double GOLDEN_ANGLE = 2.399963229728653;
    private static final int CONTACT_CONFIRM_TICKS = 2;
    private static final double CONTACT_DECAY = 0.9985;
    private static final double LOST_TRACK_RADIUS = 5_000.0;
    private static final double REPLAN_TARGET_MOVE_DIST = 250.0;
    private static final long COMBAT_REPLAN_TICKS = 120L;
    private static final long PATROL_PING_INTERVAL = 600L;
    private static final long STALE_CONTACT_TICKS = 150L;
    private static final long TRACK_PING_INTERVAL = 250L;
    private static final long ACTIVE_TRACK_MEMORY_TICKS = 900L;
    private static final long TORPEDO_THREAT_MEMORY_TICKS = 450L;
    private static final long TORPEDO_THREAT_PING_INTERVAL = 150L;
    private static final double CHASE_RANGE = 3_500.0;
    private static final double FIRING_RANGE = 2_100.0;
    private static final double TMA_SETUP_QUALITY = 0.55;
    private static final double TORPEDO_THREAT_MAX_ACTIVE_RANGE = 2_200.0;
    private static final long OWN_TORPEDO_IGNORE_TICKS = 550L;
    private static final double OWN_TORPEDO_IGNORE_BEARING = Math.toRadians(30.0);
    private static final double STERN_OFFSET = 450.0;
    private static final double LONG_INTERCEPT_RANGE = 4_500.0;
    private static final double CENTER_PULL_DISTANCE = 1_500.0;
    private static final double INTERCEPT_SIDE_OFFSET = 700.0;
    private static final long TORPEDO_REFIRE_COOLDOWN = 350L;
    private static final long TORPEDO_LONG_RANGE_REFIRE_COOLDOWN = 430L;
    private static final long TORPEDO_FINISHER_REFIRE_COOLDOWN = 240L;
    private static final long TORPEDO_ARMING_DELAY_TICKS = 500L;
    private static final long TORPEDO_ACTIVE_FIX_MAX_AGE = 220L;
    private static final long TORPEDO_SALVO_RESET_TICKS = 2_200L;
    private static final long TORPEDO_LATE_MATCH_TICKS = 18_000L;
    private static final long TORPEDO_POST_LAUNCH_EGRESS_TICKS = 900L;
    private static final long DAMAGE_EGRESS_TICKS = 1_600L;
    private static final double TORPEDO_CLOSE_RANGE = 2_800.0;
    private static final double TORPEDO_FINISH_RANGE = 1_800.0;
    private static final double TORPEDO_COMMIT_RANGE = 1_200.0;
    private static final double TORPEDO_MIN_RANGE = 550.0;
    private static final double TORPEDO_MAX_RANGE = 3_600.0;
    private static final double TORPEDO_MAX_UNCERTAINTY = 280.0;
    private static final double TORPEDO_SALVO_RESET_DISTANCE = 850.0;
    private static final double TORPEDO_ALIGNMENT_LIMIT = Math.toRadians(75.0);
    private static final int TORPEDO_OPENING_SALVO_LIMIT = 4;
    private static final int TORPEDO_FINISHER_SALVO_LIMIT = 6;
    private static final double EGRESS_STANDOFF_RANGE = 2_850.0;
    private static final double EGRESS_MIN_RANGE = 2_300.0;
    private static final double EGRESS_MAX_RANGE = 4_100.0;
    private static final double EGRESS_LATERAL_ANGLE = Math.toRadians(65.0);
    private static final double EGRESS_STERN_ANGLE = Math.toRadians(35.0);
    private static final double SHADOW_RANGE = 2_000.0;
    private static final double SHADOW_MIN_RANGE = 1_150.0;
    private static final double SEARCH_TRANSIT_RADIUS = 2_700.0;
    private static final double SEARCH_RING_RADIUS = 1_850.0;
    private static final double SEARCH_RING_AHEAD_ANGLE = Math.toRadians(55.0);

    private enum Mode {
        PATROL,
        TRACK,
        CHASE
    }

    private MatchConfig config;
    private TerrainMap terrain;
    private BattleArea battleArea;
    private PathPlanner pathPlanner;
    private CodexAutopilot autopilot;

    private List<StrategicWaypoint> strategicWaypoints = List.of();
    private long planCount;
    private long lastProgressTick;
    private double bestDistToGoal = Double.POSITIVE_INFINITY;
    private double lastTargetX = Double.NaN;
    private double lastTargetY = Double.NaN;
    private double lastPlanBearing = Double.NaN;

    private List<StrategicWaypoint> objectives = List.of();
    private int objectiveIndex = 0;
    private Mode mode = Mode.PATROL;
    private Mode previousMode = Mode.PATROL;
    private boolean hasTrackedContact;
    private double trackedX = Double.NaN;
    private double trackedY = Double.NaN;
    private double trackedHeading = Double.NaN;
    private double trackedSpeed = 5.0;
    private double trackedDepth = Double.NaN;
    private double trackedSolutionQuality = 0.0;
    private double contactAlive = 0.0;
    private double uncertaintyRadius = 0.0;
    private double refUncertainty = 400.0;
    private double estimatedRange = Double.POSITIVE_INFINITY;
    private boolean rangeConfirmedByActive;
    private double lastContactBearing = Double.NaN;
    private double lastContactSE = 0.0;
    private long lastContactTick = Long.MIN_VALUE / 4;
    private long trackedLastFixTick = Long.MIN_VALUE / 4;
    private int consecutiveContactTicks;
    private long lastCombatPlanTick = Long.MIN_VALUE / 4;
    private long lastPingTick = Long.MIN_VALUE / 4;
    private double lastCombatTargetX = Double.NaN;
    private double lastCombatTargetY = Double.NaN;
    private long lastTorpedoLaunchTick = Long.MIN_VALUE / 4;
    private int torpedoSalvoShots;
    private long torpedoSalvoTick = Long.MIN_VALUE / 4;
    private double torpedoSalvoTargetX = Double.NaN;
    private double torpedoSalvoTargetY = Double.NaN;
    private double lastActiveFixX = Double.NaN;
    private double lastActiveFixY = Double.NaN;
    private double lastActiveFixDepth = Double.NaN;
    private double lastActiveFixHeading = Double.NaN;
    private double lastActiveFixSpeed = 0.0;
    private SonarContact torpedoThreat;
    private long lastTorpedoThreatTick = Long.MIN_VALUE / 4;
    private double lastTorpedoThreatBearing = Double.NaN;
    private double lastTorpedoThreatRange = Double.NaN;
    private long lastDamageTakenTick = Long.MIN_VALUE / 4;
    private int lastOwnHp;
    private boolean searchClockwise = true;

    @Override
    public String name() {
        return "Codex Sub";
    }

    @Override
    public TorpedoController createTorpedoController() {
        return new CodexTorpedoController();
    }

    @Override
    public void onMatchStart(MatchContext context) {
        this.config = context.config();
        this.terrain = context.terrain();
        this.battleArea = context.config().battleArea();
        this.pathPlanner = new PathPlanner(context.terrain(), -90.0, 225.0, 75.0, 400.0, 5.0);
        this.autopilot = new CodexAutopilot(context);
        this.strategicWaypoints = List.of();
        this.planCount = 0L;
        this.lastProgressTick = 0L;
        this.bestDistToGoal = Double.POSITIVE_INFINITY;
        this.lastTargetX = Double.NaN;
        this.lastTargetY = Double.NaN;
        this.lastPlanBearing = Double.NaN;
        this.mode = Mode.PATROL;
        this.previousMode = Mode.PATROL;
        this.hasTrackedContact = false;
        this.trackedX = Double.NaN;
        this.trackedY = Double.NaN;
        this.trackedHeading = Double.NaN;
        this.trackedSpeed = 5.0;
        this.trackedDepth = Double.NaN;
        this.trackedSolutionQuality = 0.0;
        this.contactAlive = 0.0;
        this.uncertaintyRadius = 0.0;
        this.refUncertainty = 400.0;
        this.estimatedRange = Double.POSITIVE_INFINITY;
        this.rangeConfirmedByActive = false;
        this.lastContactBearing = Double.NaN;
        this.lastContactSE = 0.0;
        this.lastContactTick = Long.MIN_VALUE / 4;
        this.trackedLastFixTick = Long.MIN_VALUE / 4;
        this.consecutiveContactTicks = 0;
        this.lastCombatPlanTick = Long.MIN_VALUE / 4;
        this.lastPingTick = Long.MIN_VALUE / 4;
        this.lastCombatTargetX = Double.NaN;
        this.lastCombatTargetY = Double.NaN;
        this.lastTorpedoLaunchTick = Long.MIN_VALUE / 4;
        resetTorpedoFireControl();
        this.lastActiveFixX = Double.NaN;
        this.lastActiveFixY = Double.NaN;
        this.lastActiveFixDepth = Double.NaN;
        this.lastActiveFixHeading = Double.NaN;
        this.lastActiveFixSpeed = 0.0;
        this.torpedoThreat = null;
        this.lastTorpedoThreatTick = Long.MIN_VALUE / 4;
        this.lastTorpedoThreatBearing = Double.NaN;
        this.lastTorpedoThreatRange = Double.NaN;
        this.lastDamageTakenTick = Long.MIN_VALUE / 4;
        this.lastOwnHp = context.config().startingHp();
        this.searchClockwise = (context.config().worldSeed() & 1L) == 0L;
    }

    public CodexAutopilot autopilot() {
        return autopilot;
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
        trackOwnDamage(self.hp(), tick);
        updateTorpedoThreat(input.sonarContacts(), input.activeSonarReturns(), heading, tick);
        boolean torpedoThreatActive = hasTorpedoThreat(tick);
        SonarContact bestContact = pickBestContact(input.sonarContacts(), input.activeSonarReturns());

        updateTrackedContact(bestContact, pos, tick);
        updateMode(pos, tick);

        boolean hasObjectives = !objectives.isEmpty() && objectiveIndex < objectives.size();
        boolean modeChanged = mode != previousMode;

        if (torpedoThreatActive) {
            boolean needPlan = modeChanged
                    || strategicWaypoints.isEmpty()
                    || autopilot.isBlocked()
                    || autopilot.hasArrived()
                    || lastTorpedoThreatTick > lastCombatPlanTick
                    || tick - lastCombatPlanTick > 90L;
            if (needPlan) {
                StrategicWaypoint wp = planTorpedoDefenseWaypoint(pos.x(), pos.y(), pos.z(), heading, speed);
                strategicWaypoints = List.of(wp);
                autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), pos.z(), heading, speed);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
                lastCombatPlanTick = tick;
                lastCombatTargetX = wp.x();
                lastCombatTargetY = wp.y();
            }
        } else if (hasObjectives) {
            var obj = objectives.get(objectiveIndex);
            boolean needSet = strategicWaypoints.isEmpty() || autopilot.hasArrived();

            if (autopilot.hasArrived() && objectiveIndex < objectives.size()) {
                objectiveIndex++;
                hasObjectives = objectiveIndex < objectives.size();
                if (hasObjectives) {
                    obj = objectives.get(objectiveIndex);
                    needSet = true;
                }
            }

            if (hasObjectives && needSet) {
                strategicWaypoints = List.of(obj);
                autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), pos.z(), heading, speed);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
            }
        } else if (mode != Mode.PATROL && hasTrackedContact) {
            boolean needPlan = modeChanged
                    || strategicWaypoints.isEmpty()
                    || autopilot.isBlocked()
                    || autopilot.hasArrived();

            if (!needPlan && !Double.isNaN(lastCombatTargetX)) {
                double moved = CodexAutopilot.hdist(trackedX, trackedY, lastCombatTargetX, lastCombatTargetY);
                if (moved > REPLAN_TARGET_MOVE_DIST) {
                    needPlan = true;
                } else if (Math.max(lastTorpedoLaunchTick, lastDamageTakenTick) > lastCombatPlanTick) {
                    needPlan = true;
                } else if (tick - lastCombatPlanTick > COMBAT_REPLAN_TICKS && uncertaintyRadius > 750.0) {
                    needPlan = true;
                }
            }

            if (needPlan) {
                StrategicWaypoint wp = planCombatWaypoint(pos.x(), pos.y(), pos.z(), heading, speed, tick);
                strategicWaypoints = List.of(wp);
                autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), pos.z(), heading, speed);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
                lastCombatPlanTick = tick;
                lastCombatTargetX = wp.x();
                lastCombatTargetY = wp.y();
            }
        } else {
            boolean needPlan = modeChanged
                    || strategicWaypoints.isEmpty()
                    || autopilot.isBlocked()
                    || autopilot.hasArrived();

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
                StrategicWaypoint wp = planWaypoint(pos.x(), pos.y(), heading, speed);
                strategicWaypoints = List.of(wp);
                autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), pos.z(), heading, speed);
                bestDistToGoal = autopilot.distanceToStrategic(pos.x(), pos.y());
                lastProgressTick = tick;
            }
        }

        autopilot.tick(input, output);
        if (torpedoThreatActive) {
            applyTorpedoDefenseHelm(output, pos, heading);
        } else if (!hasObjectives) {
            if (!hasTrackedContact) {
                applySearchHelm(output, pos, heading);
            } else {
                applyLongRangeInterceptHelm(input, output, pos, heading);
            }
        }
        handleActiveSonar(input, output, pos, tick);
        publishFiringSolution(output, pos, tick);
        launchTorpedoIfReady(input, output, pos, heading, tick);

        double floor = terrain.elevationAt(pos.x(), pos.y());
        double gap = pos.z() - floor;
        if (hasTrackedContact) {
            double confidence = refUncertainty / Math.max(refUncertainty + uncertaintyRadius, 1.0);
            output.publishContactEstimate(new ContactEstimate(
                    trackedX,
                    trackedY,
                    contactAlive * confidence,
                    contactAlive,
                    uncertaintyRadius,
                    trackedHeading,
                    trackedSpeed,
                    hasFreshActiveFix(tick, 350L) ? "ping" : "passive"));
        }
        output.setStatus(String.format("%s%s/%s f:%.0f g:%.0f",
                torpedoThreatActive ? "!" : "",
                hasObjectives ? "OBJ" : mode.name().substring(0, 1),
                autopilot.lastStatus(), -floor, gap));

        for (int i = 0; i < strategicWaypoints.size(); i++) {
            StrategicWaypoint wp = strategicWaypoints.get(i);
            output.publishStrategicWaypoint(
                    new Waypoint(wp.x(), wp.y(), wp.preferredDepth(), i == autopilot.currentWaypointIndex()),
                    wp.purpose());
        }
        previousMode = mode;
    }

    private StrategicWaypoint planWaypoint(double x, double y, double heading, double speed) {
        boolean firstLeg = planCount == 0;
        double toCenter = CodexAutopilot.norm(Math.atan2(-x, -y));
        double sweepBase = CodexAutopilot.norm(toCenter + planCount * GOLDEN_ANGLE);
        double cruiseDepth = autopilot != null ? autopilot.cruiseDepth() : -260.0;
        double desiredDist = firstLeg ? 1_800.0 : 3_400.0;
        double currentBoundary = battleArea.distanceToBoundary(x, y);

        if (planCount < 2) {
            double distToCenter = Math.hypot(x, y);
            double huntDist = Math.clamp(distToCenter * 0.6, 2_000.0, 4_200.0);
            double tx = x + Math.sin(toCenter) * huntDist;
            double ty = y + Math.cos(toCenter) * huntDist;
            if (battleArea.distanceToBoundary(tx, ty) > PATROL_MARGIN / 2.0 && pathPlanner.isSafe(tx, ty)) {
                planCount++;
                lastTargetX = tx;
                lastTargetY = ty;
                lastPlanBearing = toCenter;
                return new StrategicWaypoint(
                        tx, ty, safeDepth(tx, ty, Math.min(cruiseDepth, -180.0)),
                        Purpose.RALLY, NoisePolicy.SPRINT, MovementPattern.DIRECT, 260.0, 12.2);
            }
        }

        var bearings = new ArrayList<Double>();
        bearings.add(heading);
        if (!Double.isNaN(lastPlanBearing)) {
            bearings.add(lastPlanBearing);
        }
        for (double offset : new double[]{12.0, 25.0, 40.0, 60.0, 85.0}) {
            bearings.add(CodexAutopilot.norm(heading + Math.toRadians(offset)));
            bearings.add(CodexAutopilot.norm(heading - Math.toRadians(offset)));
        }
        bearings.add(sweepBase);
        bearings.add(CodexAutopilot.norm(sweepBase + Math.toRadians(35.0)));
        bearings.add(CodexAutopilot.norm(sweepBase - Math.toRadians(35.0)));
        bearings.add(toCenter);
        bearings.add(CodexAutopilot.norm(toCenter + Math.toRadians(28.0)));
        bearings.add(CodexAutopilot.norm(toCenter - Math.toRadians(28.0)));

        double[] distances = firstLeg
                ? new double[]{1_400.0, 1_800.0, 2_200.0}
                : new double[]{2_600.0, 3_400.0, 4_200.0};

        double bestScore = Double.NEGATIVE_INFINITY;
        double bestX = x;
        double bestY = y;
        double bestDepth = cruiseDepth;
        double bestSpeed = 9.2;

        for (double bearing : bearings) {
            for (double dist : distances) {
                double tx = x + Math.sin(bearing) * dist;
                double ty = y + Math.cos(bearing) * dist;

                if (battleArea.distanceToBoundary(tx, ty) < PATROL_MARGIN) continue;
                if (!pathPlanner.isSafe(tx, ty)) continue;

                double depth = safeDepth(tx, ty, cruiseDepth);
                var path = pathPlanner.findPath(
                        x, y, tx, ty, depth,
                        CodexAutopilot.depthChangeRatio(9.2),
                        CodexAutopilot.turnRadiusAtSpeed(9.2));
                if (path.isEmpty()) continue;

                double direct = CodexAutopilot.hdist(x, y, tx, ty);
                double routeLen = polylineLength(path);
                double pathRatio = routeLen / Math.max(direct, 1.0);
                if (pathRatio > 1.65) continue;

                double alignment = 1.0 + Math.cos(CodexAutopilot.adiff(bearing, heading));
                double continuity = Double.isNaN(lastPlanBearing)
                        ? alignment
                        : 1.0 + Math.cos(CodexAutopilot.adiff(bearing, lastPlanBearing));
                double sweep = 1.0 + Math.cos(CodexAutopilot.adiff(bearing, sweepBase));
                double centering = 1.0 + Math.cos(CodexAutopilot.adiff(bearing, toCenter));
                double floorDepth = Math.abs(terrain.elevationAt(tx, ty));
                double boundary = battleArea.distanceToBoundary(tx, ty);
                double centerWeight = currentBoundary < PATROL_MARGIN + 600.0 ? 180.0 : 35.0;

                double score = alignment * (firstLeg ? 520.0 : 460.0)
                        + continuity * (firstLeg ? 140.0 : 220.0)
                        + sweep * (firstLeg ? 20.0 : 70.0)
                        + centering * centerWeight
                        + Math.min(boundary, 2200.0) * 0.18
                        + Math.min(floorDepth, 550.0) * 0.65
                        - Math.abs(dist - desiredDist) * (firstLeg ? 0.06 : 0.08)
                        - (pathRatio - 1.0) * 1800.0;

                if (!Double.isNaN(lastTargetX)) {
                    double sep = CodexAutopilot.hdist(tx, ty, lastTargetX, lastTargetY);
                    score -= Math.max(0.0, 1800.0 - sep) * 0.12;
                }

                double targetSpeed = firstLeg
                        ? (pathRatio > 1.10 ? 8.8 : 9.8)
                        : (pathRatio > 1.12 ? 8.4 : 9.4);
                if (score > bestScore) {
                    bestScore = score;
                    bestX = tx;
                    bestY = ty;
                    bestDepth = depth;
                    bestSpeed = targetSpeed;
                }
            }
        }

        if (bestScore == Double.NEGATIVE_INFINITY) {
            for (double d : new double[]{1_800.0, 1_200.0, 700.0}) {
                double tx = x + Math.sin(toCenter) * d;
                double ty = y + Math.cos(toCenter) * d;
                if (battleArea.distanceToBoundary(tx, ty) > PATROL_MARGIN / 2.0 && pathPlanner.isSafe(tx, ty)) {
                    bestX = tx;
                    bestY = ty;
                    bestDepth = safeDepth(tx, ty, cruiseDepth);
                    bestSpeed = 8.0;
                    break;
                }
            }
        }

        planCount++;
        lastTargetX = bestX;
        lastTargetY = bestY;
        lastPlanBearing = CodexAutopilot.norm(Math.atan2(bestX - x, bestY - y));
        return new StrategicWaypoint(bestX, bestY, bestDepth, Purpose.PATROL,
                NoisePolicy.NORMAL, MovementPattern.DIRECT, firstLeg ? 250.0 : 220.0, bestSpeed);
    }

    private double safeDepth(double x, double y, double cruiseDepth) {
        double floor = terrain.elevationAt(x, y);
        double target = Math.max(cruiseDepth, floor + 90.0);
        return Math.clamp(target, config.crushDepth() + 50.0, -40.0);
    }

    private double polylineLength(List<Vec3> path) {
        double len = 0.0;
        for (int i = 1; i < path.size(); i++) {
            len += path.get(i - 1).horizontalDistanceTo(path.get(i));
        }
        return len;
    }

    private SonarContact pickBestContact(List<SonarContact> passive, List<SonarContact> active) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (SonarContact contact : active) {
            double score = 1_000.0 + contact.signalExcess();
            if (score > bestScore) {
                best = contact;
                bestScore = score;
            }
        }
        for (SonarContact contact : passive) {
            double score = contact.signalExcess();
            if (score > bestScore) {
                best = contact;
                bestScore = score;
            }
        }
        return best;
    }

    private void updateTrackedContact(SonarContact bestContact, Vec3 pos, long tick) {
        double dt = 1.0 / 50.0;
        if (hasTrackedContact) {
            if (!Double.isNaN(trackedHeading)) {
                trackedX += trackedSpeed * Math.sin(trackedHeading) * dt;
                trackedY += trackedSpeed * Math.cos(trackedHeading) * dt;
            }
            uncertaintyRadius += config.maxSubSpeed() * dt;
            contactAlive *= CONTACT_DECAY;
            if (tick - trackedLastFixTick > 500) {
                rangeConfirmedByActive = false;
            }
        }

        if (bestContact == null) {
            consecutiveContactTicks = 0;
            if (hasTrackedContact && uncertaintyRadius > LOST_TRACK_RADIUS) {
                clearTrack();
            }
            return;
        }

        lastContactBearing = bestContact.bearing();
        lastContactSE = bestContact.signalExcess();
        lastContactTick = tick;
        consecutiveContactTicks++;
        contactAlive = 1.0;
        trackedSolutionQuality = bestContact.solutionQuality();

        if (bestContact.estimatedSpeed() >= 0.0) {
            double quality = bestContact.isActive() ? 0.85 : Math.clamp(bestContact.signalExcess() / 25.0, 0.15, 0.45);
            trackedSpeed = hasTrackedContact
                    ? trackedSpeed * (1.0 - quality) + bestContact.estimatedSpeed() * quality
                    : bestContact.estimatedSpeed();
        }
        if (!Double.isNaN(bestContact.estimatedHeading())) {
            trackedHeading = bestContact.estimatedHeading();
        }
        if (!Double.isNaN(bestContact.estimatedDepth())) {
            trackedDepth = Double.isNaN(trackedDepth)
                    ? bestContact.estimatedDepth()
                    : trackedDepth * 0.3 + bestContact.estimatedDepth() * 0.7;
        }

        double range = bestContact.isActive()
                ? bestContact.range()
                : passiveTrackRange(bestContact, pos, tick);
        estimatedRange = range;
        double tx = pos.x() + range * Math.sin(bestContact.bearing());
        double ty = pos.y() + range * Math.cos(bestContact.bearing());

        if (hasTrackedContact) {
            double blend = bestContact.isActive()
                    ? 1.0
                    : Math.clamp(0.08 + bestContact.solutionQuality() * 0.28, 0.08, 0.36);
            trackedX = trackedX * (1.0 - blend) + tx * blend;
            trackedY = trackedY * (1.0 - blend) + ty * blend;
        } else {
            trackedX = tx;
            trackedY = ty;
            hasTrackedContact = true;
        }

        if (bestContact.isActive() && bestContact.range() > 50.0) {
            estimatedRange = bestContact.range();
            rangeConfirmedByActive = true;
            trackedLastFixTick = tick;
            double fixRadius = bestContact.rangeUncertainty() > 0.0
                    ? bestContact.rangeUncertainty() * 2.0
                    : 90.0;
            uncertaintyRadius = Math.max(80.0, fixRadius);
            refUncertainty = uncertaintyRadius;
            lastActiveFixX = trackedX;
            lastActiveFixY = trackedY;
            lastActiveFixDepth = trackedDepth;
            lastActiveFixHeading = trackedHeading;
            lastActiveFixSpeed = Math.max(trackedSpeed, 0.0);
        } else {
            double passiveSpread = Math.max(450.0,
                    range * Math.max(bestContact.bearingUncertainty(), Math.toRadians(6.0)) * 2.5);
            passiveSpread = Math.max(passiveSpread,
                    range * (1.0 - Math.clamp(bestContact.solutionQuality(), 0.05, 0.95)) * 0.55);
            uncertaintyRadius = hasTrackedContact
                    ? Math.max(250.0, uncertaintyRadius * 0.7 + passiveSpread * 0.3)
                    : passiveSpread;
        }

        if (uncertaintyRadius > LOST_TRACK_RADIUS) {
            clearTrack();
        }
    }

    private void updateMode(Vec3 pos, long tick) {
        if (!hasTrackedContact) {
            mode = Mode.PATROL;
            return;
        }

        long ticksSinceContact = tick - lastContactTick;
        double trackedDist = CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        boolean freshActiveFix = hasFreshActiveFix(tick, 300L);
        if (freshActiveFix
                || trackedSolutionQuality > TMA_SETUP_QUALITY
                || (trackedSolutionQuality > 0.42 && trackedDist < CHASE_RANGE && ticksSinceContact < 400)
                || (uncertaintyRadius < 425.0 && ticksSinceContact < 300)) {
            mode = Mode.CHASE;
        } else if (hasTrackedContact
                && (consecutiveContactTicks >= CONTACT_CONFIRM_TICKS
                || (contactAlive > 0.02 && ticksSinceContact < 3_000))) {
            mode = Mode.TRACK;
        } else {
            mode = Mode.PATROL;
        }
    }

    private StrategicWaypoint planCombatWaypoint(double x, double y, double z,
                                                 double heading, double speed, long tick) {
        double dist = CodexAutopilot.hdist(x, y, trackedX, trackedY);
        double cruiseDepth = autopilot != null ? autopilot.cruiseDepth() : -220.0;
        double targetX = trackedX;
        double targetY = trackedY;
        double targetDepth = safeDepth(targetX, targetY, Math.min(cruiseDepth, -160.0));
        Purpose purpose = Purpose.INTERCEPT;
        NoisePolicy noise = dist > 3_500.0 ? NoisePolicy.SPRINT : NoisePolicy.NORMAL;
        double targetSpeed = dist > 3_500.0 ? 11.0 : dist > 1_800.0 ? 9.5 : 6.5;
        double arrivalRadius = dist > 2_000.0 ? 260.0 : 180.0;
        StrategicWaypoint egress = planEgressWaypoint(x, y, z, heading, dist, cruiseDepth, tick);
        if (egress != null) {
            return egress;
        }

        if (mode == Mode.TRACK && !rangeConfirmedByActive) {
            double bearing = !Double.isNaN(lastContactBearing)
                    ? lastContactBearing
                    : CodexAutopilot.norm(Math.atan2(trackedX - x, trackedY - y));
            double passiveRange = passivePlanningRange(x, y);
            double sideA = CodexAutopilot.norm(bearing + Math.PI / 2.0);
            double sideB = CodexAutopilot.norm(bearing - Math.PI / 2.0);
            double chosen = Math.abs(CodexAutopilot.adiff(sideA, heading))
                    > Math.abs(CodexAutopilot.adiff(sideB, heading))
                    ? sideA : sideB;
            double forwardDist = trackedSolutionQuality < 0.25
                    ? Math.clamp(passiveRange * 0.18, 350.0, 800.0)
                    : Math.clamp(passiveRange * 0.28, 500.0, 1_100.0);
            double crossDist = trackedSolutionQuality < 0.25
                    ? Math.clamp(passiveRange * 0.40, 950.0, 1_800.0)
                    : Math.clamp(passiveRange * 0.30, 700.0, 1_400.0);
            targetX = x + Math.sin(bearing) * forwardDist + Math.sin(chosen) * crossDist;
            targetY = y + Math.cos(bearing) * forwardDist + Math.cos(chosen) * crossDist;
            targetDepth = safeDepth(targetX, targetY, Math.min(cruiseDepth, -180.0));
            purpose = Purpose.INVESTIGATE;
            noise = trackedSolutionQuality < 0.30 ? NoisePolicy.NORMAL : NoisePolicy.QUIET;
            targetSpeed = trackedSolutionQuality < 0.30 ? 8.8 : 7.2;
            arrivalRadius = trackedSolutionQuality < 0.30 ? 320.0 : 250.0;
        } else if (!Double.isNaN(trackedHeading) && trackedSpeed > 0.5) {
            double predictionSeconds = Math.clamp(dist / Math.max(speed, 4.5), 8.0, 40.0);
            targetX += Math.sin(trackedHeading) * trackedSpeed * predictionSeconds * 0.6;
            targetY += Math.cos(trackedHeading) * trackedSpeed * predictionSeconds * 0.6;
            if (dist > LONG_INTERCEPT_RANGE) {
                double centerBearing = CodexAutopilot.norm(Math.atan2(-trackedX, -trackedY));
                double sideA = CodexAutopilot.norm(trackedHeading + Math.PI / 2.0);
                double sideB = CodexAutopilot.norm(trackedHeading - Math.PI / 2.0);
                double chosen = Math.abs(CodexAutopilot.adiff(sideA, heading))
                        < Math.abs(CodexAutopilot.adiff(sideB, heading))
                        ? sideA : sideB;
                targetX += Math.sin(centerBearing) * CENTER_PULL_DISTANCE + Math.sin(chosen) * INTERCEPT_SIDE_OFFSET;
                targetY += Math.cos(centerBearing) * CENTER_PULL_DISTANCE + Math.cos(chosen) * INTERCEPT_SIDE_OFFSET;
                noise = NoisePolicy.SPRINT;
                targetSpeed = 12.5;
                arrivalRadius = 340.0;
            } else if (dist < SHADOW_MIN_RANGE) {
                double awayBearing = CodexAutopilot.norm(Math.atan2(x - trackedX, y - trackedY));
                double backX = trackedX + Math.sin(awayBearing) * SHADOW_RANGE;
                double backY = trackedY + Math.cos(awayBearing) * SHADOW_RANGE;
                if (battleArea.distanceToBoundary(backX, backY) > PATROL_MARGIN / 2.0
                        && pathPlanner.isSafe(backX, backY)) {
                    targetX = backX;
                    targetY = backY;
                    purpose = Purpose.EVADE;
                    noise = NoisePolicy.QUIET;
                    targetSpeed = 7.0;
                    arrivalRadius = 220.0;
                }
            } else if (dist < TORPEDO_MAX_RANGE) {
                double sternOffset = dist < SHADOW_RANGE ? STERN_OFFSET + 250.0 : STERN_OFFSET;
                double sternX = targetX - Math.sin(trackedHeading) * sternOffset;
                double sternY = targetY - Math.cos(trackedHeading) * sternOffset;
                if (battleArea.distanceToBoundary(sternX, sternY) > PATROL_MARGIN / 2.0
                        && pathPlanner.isSafe(sternX, sternY)) {
                    targetX = sternX;
                    targetY = sternY;
                }
                noise = NoisePolicy.QUIET;
                targetSpeed = dist > 1_800.0 ? 6.8 : 6.2;
            }
            targetDepth = safeDepth(targetX, targetY, Math.min(cruiseDepth, -170.0));
        }

        if (battleArea.distanceToBoundary(targetX, targetY) < PATROL_MARGIN
                || !pathPlanner.isSafe(targetX, targetY)) {
            targetX = trackedX;
            targetY = trackedY;
            if (battleArea.distanceToBoundary(targetX, targetY) < PATROL_MARGIN) {
                double range = Math.max(Math.hypot(targetX, targetY), 1.0);
                double pull = 500.0 / range;
                targetX = targetX * (1.0 - pull);
                targetY = targetY * (1.0 - pull);
            }
            targetDepth = safeDepth(targetX, targetY, Math.min(cruiseDepth, -150.0));
        }

        return new StrategicWaypoint(
                targetX, targetY, targetDepth, purpose, noise,
                MovementPattern.DIRECT, arrivalRadius, targetSpeed);
    }

    private StrategicWaypoint planEgressWaypoint(double x, double y, double z,
                                                 double heading, double dist,
                                                 double cruiseDepth, long tick) {
        boolean recentLaunch = tick - lastTorpedoLaunchTick < TORPEDO_POST_LAUNCH_EGRESS_TICKS
                && dist < TORPEDO_MAX_RANGE + 900.0;
        boolean recentDamage = tick - lastDamageTakenTick < DAMAGE_EGRESS_TICKS;
        if (!recentLaunch && !recentDamage) {
            return null;
        }

        double anchorX = trackedX;
        double anchorY = trackedY;
        if (!Double.isNaN(trackedHeading) && trackedSpeed > 0.5) {
            double predictSeconds = Math.clamp(dist / Math.max(trackedSpeed + 2.0, 6.5), 10.0, 26.0);
            anchorX += Math.sin(trackedHeading) * trackedSpeed * predictSeconds * 0.55;
            anchorY += Math.cos(trackedHeading) * trackedSpeed * predictSeconds * 0.55;
        }
        double awayBearing = CodexAutopilot.norm(Math.atan2(x - anchorX, y - anchorY));
        double desiredRange = Math.clamp(
                Math.max(EGRESS_STANDOFF_RANGE, dist + (recentDamage ? 850.0 : 500.0)),
                EGRESS_MIN_RANGE,
                EGRESS_MAX_RANGE);

        var candidateBearings = new ArrayList<Double>();
        if (!Double.isNaN(trackedHeading)) {
            candidateBearings.add(CodexAutopilot.norm(trackedHeading + Math.PI + EGRESS_STERN_ANGLE));
            candidateBearings.add(CodexAutopilot.norm(trackedHeading + Math.PI - EGRESS_STERN_ANGLE));
        }
        candidateBearings.add(CodexAutopilot.norm(awayBearing + EGRESS_LATERAL_ANGLE));
        candidateBearings.add(CodexAutopilot.norm(awayBearing - EGRESS_LATERAL_ANGLE));
        candidateBearings.add(awayBearing);

        double bestScore = Double.NEGATIVE_INFINITY;
        double bestX = Double.NaN;
        double bestY = Double.NaN;
        for (double candidateBearing : candidateBearings) {
            double tx = anchorX + Math.sin(candidateBearing) * desiredRange;
            double ty = anchorY + Math.cos(candidateBearing) * desiredRange;
            double boundary = battleArea.distanceToBoundary(tx, ty);
            if (boundary < PATROL_MARGIN / 2.0 || !pathPlanner.isSafe(tx, ty)) {
                continue;
            }
            double routeBearing = CodexAutopilot.norm(Math.atan2(tx - x, ty - y));
            double routeDistance = CodexAutopilot.hdist(x, y, tx, ty);
            double floorDepth = Math.abs(terrain.elevationAt(tx, ty));
            double score = boundary * 0.22
                    + Math.min(floorDepth, 500.0) * 0.45
                    + Math.min(routeDistance, 2_800.0) * (recentDamage ? 0.08 : 0.05)
                    - Math.abs(CodexAutopilot.adiff(routeBearing, heading)) * 130.0;
            double lateralAspect = 1.0 - Math.abs(Math.cos(CodexAutopilot.adiff(candidateBearing, awayBearing)));
            score += lateralAspect * 150.0;
            if (!Double.isNaN(trackedHeading)) {
                double sternAspect = 1.0 + Math.cos(CodexAutopilot.adiff(candidateBearing, trackedHeading + Math.PI));
                score += sternAspect * 170.0;
            }
            if (score > bestScore) {
                bestScore = score;
                bestX = tx;
                bestY = ty;
            }
        }

        if (Double.isNaN(bestX) || Double.isNaN(bestY)) {
            return null;
        }

        double depthBias = recentDamage || z > -120.0 ? -200.0 : -180.0;
        double targetDepth = safeDepth(bestX, bestY, Math.min(cruiseDepth, depthBias));
        double targetSpeed = recentDamage || dist < 1_700.0 ? 8.8 : 7.4;
        NoisePolicy noise = recentDamage || dist < 1_500.0 ? NoisePolicy.NORMAL : NoisePolicy.QUIET;
        double arrivalRadius = recentDamage ? 320.0 : 260.0;
        return new StrategicWaypoint(bestX, bestY, targetDepth, Purpose.EVADE, noise,
                MovementPattern.DIRECT, arrivalRadius, targetSpeed);
    }

    private StrategicWaypoint planTorpedoDefenseWaypoint(double x, double y, double z,
                                                         double heading, double speed) {
        double threatBearing = Double.isNaN(lastTorpedoThreatBearing) ? heading : lastTorpedoThreatBearing;
        double threatRange = Double.isFinite(lastTorpedoThreatRange) && lastTorpedoThreatRange > 0.0
                ? lastTorpedoThreatRange
                : 950.0;
        double runDistance = Math.clamp(threatRange * 0.75, 700.0, 1_450.0);
        double cruiseDepth = autopilot != null ? autopilot.cruiseDepth() : -220.0;

        var candidateBearings = new ArrayList<Double>();
        candidateBearings.add(threatBearing);
        for (double offsetDegrees : new double[]{18.0, 35.0, 55.0}) {
            double offset = Math.toRadians(offsetDegrees);
            candidateBearings.add(CodexAutopilot.norm(threatBearing + offset));
            candidateBearings.add(CodexAutopilot.norm(threatBearing - offset));
        }
        if (hasTrackedContact) {
            candidateBearings.add(CodexAutopilot.norm(Math.atan2(x - trackedX, y - trackedY)));
        }

        double bestScore = Double.NEGATIVE_INFINITY;
        double bestX = Double.NaN;
        double bestY = Double.NaN;
        for (double candidateBearing : candidateBearings) {
            double tx = x + Math.sin(candidateBearing) * runDistance;
            double ty = y + Math.cos(candidateBearing) * runDistance;
            double boundary = battleArea.distanceToBoundary(tx, ty);
            if (boundary < PATROL_MARGIN / 2.0 || !pathPlanner.isSafe(tx, ty)) {
                continue;
            }
            double routeBearing = CodexAutopilot.norm(Math.atan2(tx - x, ty - y));
            double headingPenalty = Math.abs(CodexAutopilot.adiff(routeBearing, heading));
            double threatAlignment = 1.0 + Math.cos(CodexAutopilot.adiff(candidateBearing, threatBearing));
            double floorDepth = Math.abs(terrain.elevationAt(tx, ty));
            double score = threatAlignment * 260.0
                    + Math.min(boundary, 2_000.0) * 0.18
                    + Math.min(floorDepth, 500.0) * 0.42
                    - headingPenalty * 120.0;
            if (hasTrackedContact) {
                double enemyBearing = CodexAutopilot.norm(Math.atan2(trackedX - x, trackedY - y));
                double lateralAspect = 1.0 - Math.abs(Math.cos(CodexAutopilot.adiff(candidateBearing, enemyBearing)));
                score += lateralAspect * 110.0;
            }
            if (score > bestScore) {
                bestScore = score;
                bestX = tx;
                bestY = ty;
            }
        }

        if (Double.isNaN(bestX) || Double.isNaN(bestY)) {
            bestX = x + Math.sin(threatBearing) * Math.min(runDistance, 800.0);
            bestY = y + Math.cos(threatBearing) * Math.min(runDistance, 800.0);
            if (battleArea.distanceToBoundary(bestX, bestY) < PATROL_MARGIN) {
                double retreat = Math.max(Math.hypot(bestX, bestY), 1.0);
                double pull = 450.0 / retreat;
                bestX = bestX * (1.0 - pull);
                bestY = bestY * (1.0 - pull);
            }
        }

        double targetDepth = safeDepth(bestX, bestY, Math.min(cruiseDepth, z > -140.0 ? -220.0 : -200.0));
        double targetSpeed = threatRange < 1_000.0 ? 12.0 : 10.5;
        return new StrategicWaypoint(bestX, bestY, targetDepth, Purpose.EVADE,
                NoisePolicy.SPRINT, MovementPattern.DIRECT, 180.0, targetSpeed);
    }

    private void handleActiveSonar(SubmarineInput input, SubmarineOutput output, Vec3 pos, long tick) {
        if (input.activeSonarCooldownTicks() > 0) {
            return;
        }

        long sincePing = tick - lastPingTick;
        long ticksSinceFix = tick - trackedLastFixTick;
        double trackedDist = hasTrackedContact
                ? CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY)
                : Double.POSITIVE_INFINITY;
        boolean postLaunchEgress = tick - lastTorpedoLaunchTick < TORPEDO_POST_LAUNCH_EGRESS_TICKS;

        boolean shouldPing;
        if (hasTorpedoThreat(tick)) {
            shouldPing = sincePing >= TORPEDO_THREAT_PING_INTERVAL;
        } else if (!hasTrackedContact) {
            shouldPing = sincePing >= TRACK_PING_INTERVAL;
        } else if (!rangeConfirmedByActive) {
            boolean recentActiveTrack = hasRecentActiveTrack(tick, ACTIVE_TRACK_MEMORY_TICKS);
            boolean firingSetup = trackedSolutionQuality > TMA_SETUP_QUALITY || uncertaintyRadius < 500.0;
            boolean closePassiveTrack = trackedSolutionQuality > 0.42 && trackedDist < TORPEDO_MAX_RANGE + 600.0;
            boolean chaseReacquire = recentActiveTrack
                    && trackedDist < TORPEDO_MAX_RANGE + 900.0
                    && uncertaintyRadius > 550.0;
            boolean contactCooling = tick - lastContactTick > 300L;
            shouldPing = (sincePing >= TRACK_PING_INTERVAL && (firingSetup || closePassiveTrack || chaseReacquire))
                    || (contactCooling && sincePing >= PATROL_PING_INTERVAL);
        } else {
            shouldPing = (!postLaunchEgress && sincePing >= TRACK_PING_INTERVAL)
                    || trackedDist < FIRING_RANGE + 200.0
                    || ticksSinceFix > STALE_CONTACT_TICKS
                    || uncertaintyRadius > 450.0;
        }

        if (shouldPing) {
            output.activeSonarPing();
            lastPingTick = tick;
        }
    }

    private void trackOwnDamage(int hp, long tick) {
        if (hp < lastOwnHp) {
            lastDamageTakenTick = tick;
        }
        lastOwnHp = hp;
    }

    private void updateTorpedoThreat(List<SonarContact> passive,
                                     List<SonarContact> active,
                                     double ownHeading,
                                     long tick) {
        torpedoThreat = detectTorpedoThreat(passive, active, ownHeading, tick);
        if (torpedoThreat != null) {
            lastTorpedoThreatTick = tick;
            lastTorpedoThreatBearing = torpedoThreat.bearing();
            if (torpedoThreat.range() > 0.0) {
                lastTorpedoThreatRange = torpedoThreat.range();
            }
        } else if (!hasTorpedoThreat(tick)) {
            lastTorpedoThreatBearing = Double.NaN;
            lastTorpedoThreatRange = Double.NaN;
        }
    }

    private SonarContact detectTorpedoThreat(List<SonarContact> passive,
                                             List<SonarContact> active,
                                             double ownHeading,
                                             long tick) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (SonarContact contact : active) {
            double score = torpedoThreatScore(contact, ownHeading, tick);
            if (score > bestScore) {
                bestScore = score;
                best = contact;
            }
        }
        for (SonarContact contact : passive) {
            double score = torpedoThreatScore(contact, ownHeading, tick);
            if (score > bestScore) {
                bestScore = score;
                best = contact;
            }
        }
        return best;
    }

    private double torpedoThreatScore(SonarContact contact, double ownHeading, long tick) {
        if (contact.classification() != SonarContact.Classification.TORPEDO) {
            return Double.NEGATIVE_INFINITY;
        }
        if (looksLikeOwnOutboundTorpedo(contact, ownHeading, tick)) {
            return Double.NEGATIVE_INFINITY;
        }
        double score = (contact.isActive() ? 1_000.0 : 0.0)
                + contact.signalExcess()
                + Math.min(Math.max(contact.estimatedSpeed(), 0.0), 30.0);
        if (contact.range() > 0.0) {
            score += Math.max(0.0, TORPEDO_THREAT_MAX_ACTIVE_RANGE - contact.range()) * 0.06;
        }
        return score;
    }

    private boolean looksLikeOwnOutboundTorpedo(SonarContact contact, double ownHeading, long tick) {
        if (tick - lastTorpedoLaunchTick > OWN_TORPEDO_IGNORE_TICKS) {
            return false;
        }
        if (Math.abs(CodexAutopilot.adiff(contact.bearing(), ownHeading)) > OWN_TORPEDO_IGNORE_BEARING) {
            return false;
        }
        return !contact.isActive() || contact.range() <= TORPEDO_THREAT_MAX_ACTIVE_RANGE;
    }

    private boolean hasTorpedoThreat(long tick) {
        return tick - lastTorpedoThreatTick <= TORPEDO_THREAT_MEMORY_TICKS
                && !Double.isNaN(lastTorpedoThreatBearing);
    }

    private void applySearchHelm(SubmarineOutput output, Vec3 pos, double heading) {
        double gap = pos.z() - terrain.elevationAt(pos.x(), pos.y());
        double aheadFloor = terrain.elevationAt(
                pos.x() + Math.sin(heading) * 450.0,
                pos.y() + Math.cos(heading) * 450.0);
        double aheadGap = pos.z() - aheadFloor;
        if (gap < 160.0 || aheadGap < 180.0 || battleArea.distanceToBoundary(pos.x(), pos.y()) < PATROL_MARGIN) {
            return;
        }

        double distToCenter = Math.hypot(pos.x(), pos.y());
        double centerBearing = CodexAutopilot.norm(Math.atan2(-pos.x(), -pos.y()));
        double desiredBearing;
        double throttle;

        if (distToCenter > SEARCH_TRANSIT_RADIUS) {
            desiredBearing = centerBearing;
            throttle = 0.92;
        } else {
            double ringAngle = Math.atan2(pos.x(), pos.y())
                    + (searchClockwise ? SEARCH_RING_AHEAD_ANGLE : -SEARCH_RING_AHEAD_ANGLE);
            double tx = Math.sin(ringAngle) * SEARCH_RING_RADIUS;
            double ty = Math.cos(ringAngle) * SEARCH_RING_RADIUS;
            desiredBearing = CodexAutopilot.norm(Math.atan2(tx - pos.x(), ty - pos.y()));
            throttle = distToCenter < SEARCH_RING_RADIUS * 0.75 ? 0.70 : 0.82;
        }

        double headingError = CodexAutopilot.adiff(desiredBearing, heading);
        double rudder = Math.clamp(headingError * 2.2, -0.85, 0.85);
        if (Math.abs(headingError) > Math.toRadians(40.0)) {
            throttle = Math.min(throttle, 0.62);
        }
        output.setRudder(rudder);
        output.setThrottle(throttle);
    }

    private void applyTorpedoDefenseHelm(SubmarineOutput output, Vec3 pos, double heading) {
        if (Double.isNaN(lastTorpedoThreatBearing)) {
            return;
        }
        double headingError = CodexAutopilot.adiff(lastTorpedoThreatBearing, heading);
        double rudder = Math.clamp(headingError * 2.6, -0.95, 0.95);
        double throttle = Math.abs(headingError) < Math.toRadians(25.0)
                ? 0.95
                : Math.abs(headingError) < Math.toRadians(50.0)
                ? 0.72
                : 0.56;

        double aheadFloor = terrain.elevationAt(
                pos.x() + Math.sin(heading) * 420.0,
                pos.y() + Math.cos(heading) * 420.0);
        double aheadGap = pos.z() - aheadFloor;
        if (aheadGap < 150.0) {
            throttle = Math.min(throttle, 0.35);
            rudder = Math.clamp(rudder, -0.45, 0.45);
        }

        output.setEngineClutch(true);
        output.setRudder(rudder);
        output.setThrottle(throttle);
    }

    private void applyLongRangeInterceptHelm(SubmarineInput input, SubmarineOutput output, Vec3 pos, double heading) {
        if (mode != Mode.CHASE || !hasTrackedContact) {
            return;
        }

        double trackedDist = CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        if (trackedDist <= TORPEDO_MAX_RANGE + 800.0) {
            return;
        }

        double gap = pos.z() - terrain.elevationAt(pos.x(), pos.y());
        if (gap < 140.0 || battleArea.distanceToBoundary(pos.x(), pos.y()) < PATROL_MARGIN) {
            return;
        }

        double aimX = trackedX;
        double aimY = trackedY;
        if (!Double.isNaN(trackedHeading) && trackedSpeed > 0.5) {
            double predictSeconds = Math.clamp(trackedDist / Math.max(input.self().velocity().speed(), 6.0), 18.0, 90.0);
            aimX += Math.sin(trackedHeading) * trackedSpeed * predictSeconds * 0.85;
            aimY += Math.cos(trackedHeading) * trackedSpeed * predictSeconds * 0.85;
        }
        double centerBearing = CodexAutopilot.norm(Math.atan2(-trackedX, -trackedY));
        aimX += Math.sin(centerBearing) * CENTER_PULL_DISTANCE;
        aimY += Math.cos(centerBearing) * CENTER_PULL_DISTANCE;

        double desiredBearing = CodexAutopilot.norm(Math.atan2(aimX - pos.x(), aimY - pos.y()));
        double headingError = CodexAutopilot.adiff(desiredBearing, heading);
        double rudder = Math.clamp(headingError * 2.3, -0.85, 0.85);
        double throttle = Math.abs(headingError) < Math.toRadians(20.0)
                ? 0.9
                : Math.abs(headingError) < Math.toRadians(45.0)
                ? 0.72
                : 0.5;

        double aheadFloor = terrain.elevationAt(
                pos.x() + Math.sin(heading) * 450.0,
                pos.y() + Math.cos(heading) * 450.0);
        double aheadGap = pos.z() - aheadFloor;
        if (aheadGap < 160.0) {
            throttle = Math.min(throttle, 0.25);
            rudder = Math.clamp(rudder, -0.35, 0.35);
        }

        output.setRudder(rudder);
        output.setThrottle(throttle);
    }

    private void publishFiringSolution(SubmarineOutput output, Vec3 pos, long tick) {
        if (hasTorpedoThreat(tick) || !hasTrackedContact || contactAlive < 0.35) {
            return;
        }

        double trackedDist = CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        long ticksSinceFix = tick - trackedLastFixTick;
        boolean freshRange = rangeConfirmedByActive && ticksSinceFix < 350;
        boolean behind = isBehindTargetEstimate(pos.x(), pos.y());
        double precisionLimit = freshRange
                ? (behind ? 320.0 : 280.0)
                : (behind ? 180.0 : 140.0);
        if (uncertaintyRadius > precisionLimit || trackedDist < 150.0 || trackedDist > FIRING_RANGE) {
            return;
        }

        double geometryBonus = behind ? 0.20 : 0.0;
        double solutionSpeed = trackedSpeed > 0.0 ? trackedSpeed : -1.0;
        double quality = Math.clamp(contactAlive * (1.0 - uncertaintyRadius / 350.0 + geometryBonus), 0.1, 1.0);
        output.publishFiringSolution(new FiringSolution(
                trackedX, trackedY, trackedHeading, solutionSpeed, quality));
    }

    private void launchTorpedoIfReady(SubmarineInput input, SubmarineOutput output,
                                      Vec3 pos, double ownHeading, long tick) {
        if (input.self().torpedoesRemaining() <= 0 || tick < TORPEDO_ARMING_DELAY_TICKS) {
            return;
        }
        if (hasTorpedoThreat(tick)) {
            return;
        }
        if (!hasTrackedContact || mode != Mode.CHASE || contactAlive < 0.3) {
            return;
        }

        long ticksSinceFix = tick - trackedLastFixTick;
        if (!rangeConfirmedByActive || ticksSinceFix > TORPEDO_ACTIVE_FIX_MAX_AGE) {
            return;
        }

        double targetX = predictedActiveTargetX(tick);
        double targetY = predictedActiveTargetY(tick);
        if (Double.isNaN(targetX) || Double.isNaN(targetY)) {
            return;
        }

        double trackedDist = CodexAutopilot.hdist(pos.x(), pos.y(), targetX, targetY);
        if (trackedDist < TORPEDO_MIN_RANGE || trackedDist > TORPEDO_MAX_RANGE) {
            return;
        }

        boolean behind = isBehindTargetEstimate(pos.x(), pos.y());
        double maxUncertainty = behind ? TORPEDO_MAX_UNCERTAINTY + 40.0 : TORPEDO_MAX_UNCERTAINTY;
        if (trackedDist > 3_000.0) {
            maxUncertainty += 70.0;
        }
        if (trackedDist < TORPEDO_CLOSE_RANGE) {
            maxUncertainty = Math.min(maxUncertainty, behind ? 240.0 : 180.0);
        }
        double shotUncertainty = effectiveShotUncertainty(tick);
        if (shotUncertainty > maxUncertainty) {
            return;
        }

        double bearingToTarget = CodexAutopilot.norm(Math.atan2(targetX - pos.x(), targetY - pos.y()));
        double headingError = CodexAutopilot.adiff(bearingToTarget, ownHeading);
        if (Math.abs(headingError) > TORPEDO_ALIGNMENT_LIMIT) {
            return;
        }
        if (trackedDist < TORPEDO_CLOSE_RANGE) {
            double closeAlignmentLimit = behind ? Math.toRadians(28.0) : Math.toRadians(18.0);
            if (Math.abs(headingError) > closeAlignmentLimit) {
                return;
            }
        }

        VehicleConfig torpedoConfig = VehicleConfig.torpedo();
        double torpedoSpeed = Math.sqrt(torpedoConfig.maxThrust() / torpedoConfig.dragCoeff());
        double interceptSeconds = Math.clamp(trackedDist / Math.max(torpedoSpeed, 12.0), 8.0, 45.0);
        double leadFactor = behind ? 0.95 : 0.75;
        double shotHeading = !Double.isNaN(lastActiveFixHeading) ? lastActiveFixHeading : trackedHeading;
        double shotSpeed = lastActiveFixSpeed > 0.0 ? lastActiveFixSpeed : trackedSpeed;
        double leadX = targetX;
        double leadY = targetY;
        if (!Double.isNaN(shotHeading) && shotSpeed > 0.5) {
            leadX += Math.sin(shotHeading) * shotSpeed * interceptSeconds * leadFactor;
            leadY += Math.cos(shotHeading) * shotSpeed * interceptSeconds * leadFactor;
        }

        double leadBearing = CodexAutopilot.norm(Math.atan2(leadX - pos.x(), leadY - pos.y()));
        double leadHeadingError = CodexAutopilot.adiff(leadBearing, ownHeading);
        double absLeadHeadingError = Math.abs(leadHeadingError);
        double maxLeadLimit = trackedDist > 2_600.0
                ? (behind ? Math.toRadians(38.0) : Math.toRadians(26.0))
                : (behind ? Math.toRadians(52.0) : Math.toRadians(36.0));
        if (absLeadHeadingError > maxLeadLimit) {
            return;
        }
        if (trackedDist < TORPEDO_CLOSE_RANGE) {
            double closeLeadLimit = behind ? Math.toRadians(34.0) : Math.toRadians(22.0);
            if (absLeadHeadingError > closeLeadLimit) {
                return;
            }
        }

        boolean finisherWindow = trackedDist <= TORPEDO_FINISH_RANGE
                && absLeadHeadingError <= (behind ? Math.toRadians(30.0) : Math.toRadians(18.0))
                && shotUncertainty <= (behind ? 230.0 : 170.0);
        boolean commitWindow = trackedDist <= TORPEDO_COMMIT_RANGE
                && absLeadHeadingError <= (behind ? Math.toRadians(22.0) : Math.toRadians(14.0))
                && shotUncertainty <= (behind ? 200.0 : 140.0);
        long ticksRemaining = Math.max(0L, (long) config.matchDurationTicks() - tick);
        boolean lateMatch = ticksRemaining <= TORPEDO_LATE_MATCH_TICKS;
        long refireCooldown = trackedDist > 2_600.0
                ? TORPEDO_LONG_RANGE_REFIRE_COOLDOWN
                : finisherWindow
                ? TORPEDO_FINISHER_REFIRE_COOLDOWN
                : TORPEDO_REFIRE_COOLDOWN;
        if (tick - lastTorpedoLaunchTick < refireCooldown) {
            return;
        }

        refreshTorpedoSalvo(tick, targetX, targetY);
        if (!canSpendTorpedo(input.self().torpedoesRemaining(), trackedDist, lateMatch, finisherWindow, commitWindow)) {
            return;
        }
        int salvoLimit = allowedSalvoLimit(trackedDist, lateMatch, finisherWindow, commitWindow);
        if (!commitWindow && torpedoSalvoShots >= salvoLimit) {
            return;
        }

        double shotDepth = !Double.isNaN(lastActiveFixDepth) ? lastActiveFixDepth : trackedDepth;
        double targetDepth = Double.isNaN(shotDepth) || shotDepth > -15.0
                ? -90.0
                : Math.clamp(shotDepth, config.crushDepth() + 70.0, -40.0);
        String missionData = String.format(Locale.US, "%.1f;%.1f;%.1f;%.6f;%.2f",
                leadX, leadY, targetDepth,
                Double.isNaN(shotHeading) ? ownHeading : shotHeading,
                Math.max(shotSpeed, 0.0));

        output.launchTorpedo(new TorpedoLaunchCommand(
                bearingToTarget, 0.0, config.maxFuseRadius(), missionData));
        lastTorpedoLaunchTick = tick;
        torpedoSalvoShots++;
        torpedoSalvoTick = tick;
        torpedoSalvoTargetX = targetX;
        torpedoSalvoTargetY = targetY;
    }

    private double effectiveShotUncertainty(long tick) {
        if (!rangeConfirmedByActive) {
            return uncertaintyRadius;
        }
        double dt = Math.max(0.0, tick - trackedLastFixTick) / 50.0;
        double motionAllowance = Math.max(35.0, config.maxSubSpeed() * dt * 1.6);
        return Math.min(uncertaintyRadius, refUncertainty + motionAllowance);
    }

    private void refreshTorpedoSalvo(long tick, double targetX, double targetY) {
        if (torpedoSalvoShots == 0) {
            torpedoSalvoTargetX = targetX;
            torpedoSalvoTargetY = targetY;
            return;
        }
        boolean stale = tick - torpedoSalvoTick > TORPEDO_SALVO_RESET_TICKS;
        boolean displaced = Double.isNaN(torpedoSalvoTargetX)
                || CodexAutopilot.hdist(targetX, targetY, torpedoSalvoTargetX, torpedoSalvoTargetY)
                > TORPEDO_SALVO_RESET_DISTANCE;
        if (stale || displaced) {
            torpedoSalvoShots = 0;
            torpedoSalvoTick = Long.MIN_VALUE / 4;
            torpedoSalvoTargetX = targetX;
            torpedoSalvoTargetY = targetY;
        }
    }

    private boolean canSpendTorpedo(int torpedoesRemaining, double trackedDist,
                                    boolean lateMatch, boolean finisherWindow, boolean commitWindow) {
        int reserve = trackedDist > 2_600.0 ? 3 : 2;
        if (lateMatch) {
            reserve = Math.min(reserve, 1);
        }
        if (finisherWindow) {
            reserve = lateMatch ? 0 : 1;
        }
        if (commitWindow) {
            reserve = 0;
        }
        return torpedoesRemaining > reserve;
    }

    private int allowedSalvoLimit(double trackedDist, boolean lateMatch,
                                  boolean finisherWindow, boolean commitWindow) {
        if (commitWindow) {
            return config.torpedoCount();
        }
        int limit = trackedDist > 2_900.0 ? 3 : TORPEDO_OPENING_SALVO_LIMIT;
        if (trackedDist < 2_200.0) {
            limit = 5;
        }
        if (finisherWindow) {
            limit = Math.max(limit, TORPEDO_FINISHER_SALVO_LIMIT);
        }
        if (lateMatch) {
            limit++;
        }
        return limit;
    }

    private void resetTorpedoFireControl() {
        torpedoSalvoShots = 0;
        torpedoSalvoTick = Long.MIN_VALUE / 4;
        torpedoSalvoTargetX = Double.NaN;
        torpedoSalvoTargetY = Double.NaN;
    }

    private boolean isBehindTargetEstimate(double ownX, double ownY) {
        if (Double.isNaN(trackedHeading)) {
            return false;
        }
        double sternBearing = CodexAutopilot.norm(trackedHeading + Math.PI);
        double ownBearing = CodexAutopilot.norm(Math.atan2(ownX - trackedX, ownY - trackedY));
        return Math.abs(CodexAutopilot.adiff(ownBearing, sternBearing)) < Math.toRadians(60.0);
    }

    private double estimatePassiveRange(SonarContact contact) {
        double estimate = 2_600.0 - Math.min(contact.signalExcess(), 20.0) * 85.0;
        return Math.clamp(estimate, 700.0, 2_600.0);
    }

    private double passiveTrackRange(SonarContact contact, Vec3 pos, long tick) {
        double rawRange = contact.range() > 50.0
                ? contact.range()
                : hasTrackedContact
                ? CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY)
                : estimatePassiveRange(contact);
        double quality = Math.clamp(contact.solutionQuality(), 0.05, 0.95);
        double clampedRange = quality < 0.20
                ? Math.clamp(rawRange, 1_500.0, 3_000.0)
                : quality < TMA_SETUP_QUALITY
                ? Math.clamp(rawRange, 1_200.0, 4_200.0)
                : Math.clamp(rawRange, 900.0, 5_500.0);
        double anchorRange = clampedRange;
        double activeX = tick - trackedLastFixTick <= 500L ? predictedActiveTargetX(tick) : Double.NaN;
        double activeY = tick - trackedLastFixTick <= 500L ? predictedActiveTargetY(tick) : Double.NaN;
        if (!Double.isNaN(activeX) && !Double.isNaN(activeY)) {
            anchorRange = CodexAutopilot.hdist(pos.x(), pos.y(), activeX, activeY);
        } else if (hasTrackedContact) {
            anchorRange = CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY);
        }
        double maxShift = quality < 0.20 ? 140.0 : quality < TMA_SETUP_QUALITY ? 280.0 : 650.0;
        return anchorRange + Math.clamp(clampedRange - anchorRange, -maxShift, maxShift);
    }

    private double passivePlanningRange(double x, double y) {
        double range = Double.isFinite(estimatedRange) && estimatedRange > 50.0
                ? estimatedRange
                : CodexAutopilot.hdist(x, y, trackedX, trackedY);
        if (!Double.isFinite(range) || range < 100.0) {
            range = 2_400.0;
        }
        if (trackedSolutionQuality < 0.20) {
            return Math.clamp(range, 1_800.0, 3_000.0);
        }
        if (trackedSolutionQuality < TMA_SETUP_QUALITY) {
            return Math.clamp(range, 1_400.0, 4_200.0);
        }
        return Math.clamp(range, 1_000.0, 5_500.0);
    }

    private boolean hasFreshActiveFix(long tick, long maxAgeTicks) {
        return rangeConfirmedByActive && tick - trackedLastFixTick <= maxAgeTicks;
    }

    private boolean hasRecentActiveTrack(long tick, long maxAgeTicks) {
        return !Double.isNaN(lastActiveFixX)
                && tick - trackedLastFixTick <= maxAgeTicks
                && contactAlive > 0.05;
    }

    private double predictedActiveTargetX(long tick) {
        if (Double.isNaN(lastActiveFixX)) {
            return Double.NaN;
        }
        if (Double.isNaN(lastActiveFixHeading) || lastActiveFixSpeed <= 0.0) {
            return lastActiveFixX;
        }
        double dt = Math.max(0.0, tick - trackedLastFixTick) / 50.0;
        return lastActiveFixX + Math.sin(lastActiveFixHeading) * lastActiveFixSpeed * dt;
    }

    private double predictedActiveTargetY(long tick) {
        if (Double.isNaN(lastActiveFixY)) {
            return Double.NaN;
        }
        if (Double.isNaN(lastActiveFixHeading) || lastActiveFixSpeed <= 0.0) {
            return lastActiveFixY;
        }
        double dt = Math.max(0.0, tick - trackedLastFixTick) / 50.0;
        return lastActiveFixY + Math.cos(lastActiveFixHeading) * lastActiveFixSpeed * dt;
    }

    private void clearTrack() {
        hasTrackedContact = false;
        trackedX = Double.NaN;
        trackedY = Double.NaN;
        trackedHeading = Double.NaN;
        trackedSpeed = 5.0;
        trackedDepth = Double.NaN;
        trackedSolutionQuality = 0.0;
        contactAlive = 0.0;
        uncertaintyRadius = 0.0;
        estimatedRange = Double.POSITIVE_INFINITY;
        rangeConfirmedByActive = false;
        consecutiveContactTicks = 0;
        lastActiveFixX = Double.NaN;
        lastActiveFixY = Double.NaN;
        lastActiveFixDepth = Double.NaN;
        lastActiveFixHeading = Double.NaN;
        lastActiveFixSpeed = 0.0;
        resetTorpedoFireControl();
    }
}
