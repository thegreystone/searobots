package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.ContactEstimate;
import se.hirt.searobots.api.FiringSolution;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.MovementPattern;
import se.hirt.searobots.api.NoisePolicy;
import se.hirt.searobots.api.PathPlanner;
import se.hirt.searobots.api.Purpose;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.StrategicWaypoint;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Waypoint;

import java.util.ArrayList;
import java.util.List;

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
    private static final double CHASE_RANGE = 3_500.0;
    private static final double FIRING_RANGE = 2_100.0;
    private static final double STERN_OFFSET = 450.0;

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

    @Override
    public String name() {
        return "Codex Sub";
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
        SonarContact bestContact = pickBestContact(input.sonarContacts(), input.activeSonarReturns());

        updateTrackedContact(bestContact, pos, tick);
        updateMode(pos, tick);

        boolean hasObjectives = !objectives.isEmpty() && objectiveIndex < objectives.size();
        boolean modeChanged = mode != previousMode;

        if (hasObjectives) {
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
                } else if (tick - lastCombatPlanTick > COMBAT_REPLAN_TICKS && uncertaintyRadius > 750.0) {
                    needPlan = true;
                }
            }

            if (needPlan) {
                StrategicWaypoint wp = planCombatWaypoint(pos.x(), pos.y(), pos.z(), heading, speed);
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
        handleActiveSonar(input, output, pos, tick);
        publishFiringSolution(output, pos, tick);

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
                    rangeConfirmedByActive ? "ping" : "passive"));
        }
        output.setStatus(String.format("%s/%s f:%.0f g:%.0f",
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

        if (bestContact.estimatedSpeed() >= 0.0) {
            double quality = bestContact.isActive() ? 0.85 : Math.clamp(bestContact.signalExcess() / 25.0, 0.15, 0.45);
            trackedSpeed = hasTrackedContact
                    ? trackedSpeed * (1.0 - quality) + bestContact.estimatedSpeed() * quality
                    : bestContact.estimatedSpeed();
        }
        if (!Double.isNaN(bestContact.estimatedHeading())) {
            trackedHeading = bestContact.estimatedHeading();
        }

        double range = bestContact.range() > 50.0
                ? bestContact.range()
                : hasTrackedContact
                ? CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY)
                : estimatePassiveRange(bestContact);
        double tx = pos.x() + range * Math.sin(bestContact.bearing());
        double ty = pos.y() + range * Math.cos(bestContact.bearing());

        if (hasTrackedContact) {
            double blend = bestContact.isActive() ? 0.85 : 0.22;
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
        } else {
            double passiveSpread = Math.max(450.0,
                    range * Math.max(bestContact.bearingUncertainty(), Math.toRadians(6.0)) * 2.0);
            uncertaintyRadius = hasTrackedContact
                    ? Math.min(Math.max(uncertaintyRadius, 200.0), passiveSpread)
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
        if (rangeConfirmedByActive
                || (trackedDist < CHASE_RANGE && contactAlive > 0.18)
                || (uncertaintyRadius < 300.0 && ticksSinceContact < 250)) {
            mode = Mode.CHASE;
        } else if (consecutiveContactTicks >= CONTACT_CONFIRM_TICKS
                || (contactAlive > 0.05 && ticksSinceContact < 1_000)) {
            mode = Mode.TRACK;
        } else {
            mode = Mode.PATROL;
        }
    }

    private StrategicWaypoint planCombatWaypoint(double x, double y, double z,
                                                 double heading, double speed) {
        double dist = CodexAutopilot.hdist(x, y, trackedX, trackedY);
        double cruiseDepth = autopilot != null ? autopilot.cruiseDepth() : -220.0;
        double targetX = trackedX;
        double targetY = trackedY;
        double targetDepth = safeDepth(targetX, targetY, Math.min(cruiseDepth, -160.0));
        Purpose purpose = Purpose.INTERCEPT;
        NoisePolicy noise = dist > 3_500.0 ? NoisePolicy.SPRINT : NoisePolicy.NORMAL;
        double targetSpeed = dist > 3_500.0 ? 11.0 : dist > 1_800.0 ? 9.5 : 6.5;
        double arrivalRadius = dist > 2_000.0 ? 260.0 : 180.0;

        if (mode == Mode.TRACK && !rangeConfirmedByActive) {
            double bearing = !Double.isNaN(lastContactBearing)
                    ? lastContactBearing
                    : CodexAutopilot.norm(Math.atan2(trackedX - x, trackedY - y));
            double sideA = CodexAutopilot.norm(bearing + Math.PI / 2.0);
            double sideB = CodexAutopilot.norm(bearing - Math.PI / 2.0);
            double chosen = Math.abs(CodexAutopilot.adiff(sideA, heading))
                    < Math.abs(CodexAutopilot.adiff(sideB, heading))
                    ? sideA : sideB;
            double crossDist = Math.clamp(dist * 0.25, 350.0, 850.0);
            targetX = x + Math.sin(chosen) * crossDist;
            targetY = y + Math.cos(chosen) * crossDist;
            targetDepth = safeDepth(targetX, targetY, Math.min(cruiseDepth, -180.0));
            purpose = Purpose.INVESTIGATE;
            noise = NoisePolicy.QUIET;
            targetSpeed = 6.0;
            arrivalRadius = 240.0;
        } else if (!Double.isNaN(trackedHeading) && trackedSpeed > 0.5) {
            double predictionSeconds = Math.clamp(dist / Math.max(speed, 4.5), 6.0, 35.0);
            targetX += Math.sin(trackedHeading) * trackedSpeed * predictionSeconds * 0.6;
            targetY += Math.cos(trackedHeading) * trackedSpeed * predictionSeconds * 0.6;
            if (dist < 1_800.0) {
                double sternX = targetX - Math.sin(trackedHeading) * STERN_OFFSET;
                double sternY = targetY - Math.cos(trackedHeading) * STERN_OFFSET;
                if (battleArea.distanceToBoundary(sternX, sternY) > PATROL_MARGIN / 2.0
                        && pathPlanner.isSafe(sternX, sternY)) {
                    targetX = sternX;
                    targetY = sternY;
                    if (dist < 1_100.0) {
                        noise = NoisePolicy.QUIET;
                        targetSpeed = 5.8;
                    }
                }
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

    private void handleActiveSonar(SubmarineInput input, SubmarineOutput output, Vec3 pos, long tick) {
        if (input.activeSonarCooldownTicks() > 0) {
            return;
        }

        long sincePing = tick - lastPingTick;
        long ticksSinceFix = tick - trackedLastFixTick;
        double trackedDist = hasTrackedContact
                ? CodexAutopilot.hdist(pos.x(), pos.y(), trackedX, trackedY)
                : Double.POSITIVE_INFINITY;

        boolean shouldPing;
        if (!hasTrackedContact) {
            shouldPing = sincePing >= PATROL_PING_INTERVAL;
        } else if (mode == Mode.CHASE) {
            shouldPing = trackedDist < FIRING_RANGE + 200.0
                    || ticksSinceFix > STALE_CONTACT_TICKS
                    || uncertaintyRadius > 450.0;
        } else {
            shouldPing = sincePing >= PATROL_PING_INTERVAL / 2;
        }

        if (shouldPing) {
            output.activeSonarPing();
            lastPingTick = tick;
        }
    }

    private void publishFiringSolution(SubmarineOutput output, Vec3 pos, long tick) {
        if (!hasTrackedContact || contactAlive < 0.35) {
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

    private void clearTrack() {
        hasTrackedContact = false;
        trackedX = Double.NaN;
        trackedY = Double.NaN;
        trackedHeading = Double.NaN;
        trackedSpeed = 5.0;
        contactAlive = 0.0;
        uncertaintyRadius = 0.0;
        estimatedRange = Double.POSITIVE_INFINITY;
        rangeConfirmedByActive = false;
        consecutiveContactTicks = 0;
    }
}
