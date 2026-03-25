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

import se.hirt.searobots.engine.*;

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

public final class DefaultAttackSub implements SubmarineController {

    @Override
    public String name() { return "Default Sub"; }

    // State machine
    public enum State { PATROL, TRACKING, CHASE, RAM, EVADE }

    // Throttle constants
    static final double PATROL_THROTTLE = 0.4;
    public static final double TRACKING_THROTTLE = 0.25;
    static final double CHASE_THROTTLE = 0.8;
    public static final double RAM_THROTTLE = 1.0;
    static final double EVADE_THROTTLE = 0.15;

    // Terrain / depth constants
    private static final double MIN_DEPTH = -20;
    private static final double FLOOR_CLEARANCE = 50;
    private static final double CRUSH_SAFETY_MARGIN = 50;
    private static final double BOUNDARY_TURN_DIST = 700;
    private static final double SHALLOW_WATER_LIMIT = -90.0;

    // Contact tracking constants
    public static final int CONTACT_CONFIRM_TICKS = 3;
    static final double CHASE_RANGE = 3000.0;
    static final double RAM_RANGE = 500.0;
    static final double RAM_OVERSHOT_RANGE = 800.0;
    // Confidence-based tracking constants
    static final double CONFIDENCE_DECAY = 0.999;
    static final double CONFIDENCE_HUNT_MIN = 0.1;
    static final double CONFIDENCE_RAM_MIN = 0.05;

    // Baffle clearing constants (referenced by tests)
    public static final int BAFFLE_CLEAR_INTERVAL = 1500;

    // Patrol ping constants
    public static final int PATROL_SILENCE_PING_TICKS = 3000;
    private static final double BEHIND_OFFSET = 500.0;

    // Stern tailing constants
    private static final double BEHIND_ARC = Math.toRadians(45);

    // Cavitation constants
    private static final double BASE_CAVITATION_SPEED = 5.0;
    private static final double CAVITATION_DEPTH_FACTOR = 0.02;

    // Ping range threshold for tracked contact
    private static final double TRACKED_PING_RANGE = 1500.0;

    // Replan thresholds
    private static final double REPLAN_TARGET_MOVE_DIST = 500.0;

    // State
    private State state = State.PATROL;
    private MatchConfig config;
    private TerrainMap terrain;
    private double depthLimit;
    private double maxSubSpeed = 15.0;
    private double refUncertainty = 50.0;
    private List<ThermalLayer> thermalLayers = List.of();

    // Contact tracking (sonar analysis, per-tick)
    private double lastHeading;
    private double lastSpeed;
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
    private double trackedSpeed = 2.0;
    private double contactAlive = 0.0;
    private double uncertaintyRadius = 0.0;
    private long trackedLastFixTick = -1;

    // Ping fix history for viewer trace lines
    private record PingFix(double x, double y, long tick) {}
    private final List<PingFix> pingFixHistory = new ArrayList<>();

    // Chase timing
    private long chaseStartTick;

    // Torpedo launch state
    private long lastTorpedoLaunchTick = -10000;

    // Last known contact area (preserved across state transitions for patrol biasing)
    private double lastKnownContactX = Double.NaN;
    private double lastKnownContactY = Double.NaN;

    // Patrol silence tracking
    private long patrolSilenceTicks;

    // Autopilot
    private SubmarineAutopilot autopilot;
    private List<StrategicWaypoint> strategicWaypoints = List.of();
    private double lastStrategicTargetX = Double.NaN;
    private double lastStrategicTargetY = Double.NaN;

    // Mandatory objectives
    private List<StrategicWaypoint> objectives = List.of();
    private int objectiveIndex = 0;
    private long lastReplanTick = -1000;
    private static final int REPLAN_COOLDOWN_TICKS = 250; // 5 seconds
    private BattleArea battleArea;
    private PathPlanner pathPlanner;

    @Override
    public void setObjectives(java.util.List<StrategicWaypoint> objectives) {
        this.objectives = List.copyOf(objectives);
        this.objectiveIndex = 0;
    }

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
        this.autopilot = new SubmarineAutopilot(context);
    }

    public State state() { return state; }
    public double estimatedRange() { return estimatedRange; }
    public boolean hasTrackedContact() { return hasTrackedContact; }
    public double trackedX() { return trackedX; }
    public double trackedY() { return trackedY; }
    public double contactAlive() { return contactAlive; }
    double uncertaintyRadius() { return uncertaintyRadius; }
    public double trackedHeading() { return trackedHeading; }
    public SubmarineAutopilot autopilot() { return autopilot; }

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
        double dt = 1.0 / 50.0;
        if (hasTrackedContact) {
            if (!Double.isNaN(trackedHeading) && bestContact == null) {
                trackedX += trackedSpeed * Math.sin(trackedHeading) * dt;
                trackedY += trackedSpeed * Math.cos(trackedHeading) * dt;
            }
            uncertaintyRadius += maxSubSpeed * dt;
            contactAlive *= CONFIDENCE_DECAY;
        }

        if (bestContact != null) {
            contactAlive = 1.0;

            if (bestContact.estimatedSpeed() >= 0) {
                double quality = Math.clamp(bestContact.signalExcess() / 30.0, 0.1, 0.8);
                trackedSpeed = hasTrackedContact ?
                        trackedSpeed * (1 - quality) + bestContact.estimatedSpeed() * quality :
                        bestContact.estimatedSpeed();
            }

            if (!Double.isNaN(bestContact.estimatedHeading())) {
                trackedHeading = bestContact.estimatedHeading();
            }

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

            if (bestContact.rangeUncertainty() > 0) {
                uncertaintyRadius = bestContact.rangeUncertainty() * 2;
            }

            if (bestContact.isActive() && bestContact.rangeUncertainty() > 0) {
                refUncertainty = bestContact.rangeUncertainty() * 2;
            }

            trackedLastFixTick = tick;

            if (bestContact.isActive() && bestContact.range() > 0) {
                pingFixHistory.add(new PingFix(tx, ty, tick));
                while (pingFixHistory.size() > 20) pingFixHistory.removeFirst();
            }
        }

        if (hasTrackedContact && uncertaintyRadius > 5000) {
            hasTrackedContact = false;
            trackedHeading = Double.NaN;
            pingFixHistory.clear();
        }

        boolean tookDamage = self.hp() < previousHp;
        previousHp = self.hp();

        // =================================================================
        // Step 2: State transitions (confidence-based)
        // =================================================================
        long ticksSinceContact = tick - lastContactTick;

        double trackedDist = Double.MAX_VALUE;
        if (hasTrackedContact) {
            double dx = trackedX - pos.x();
            double dy = trackedY - pos.y();
            trackedDist = Math.sqrt(dx * dx + dy * dy);
        }

        State prevState = state;

        switch (state) {
            case PATROL -> {
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
                    enterState(State.CHASE, tick);
                } else if (!hasTrackedContact && ticksSinceContact > 500) {
                    enterState(State.PATROL, tick);
                }
            }
            case CHASE -> {
                if (tookDamage && terrain.elevationAt(pos.x(), pos.y()) < -60) {
                    enterState(State.EVADE, tick);
                } else if (estimatedRange < RAM_RANGE && rangeConfirmedByActive) {
                    enterState(State.RAM, tick);
                } else if (!hasTrackedContact && uncertaintyRadius > 3000) {
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
                    enterState(State.TRACKING, tick);
                }
            }
        }

        boolean stateChanged = state != prevState;

        // =================================================================
        // Step 3: Strategic waypoint generation (BEFORE autopilot tick)
        // =================================================================
        // If objectives are set, navigate to them first
        boolean hasObjective = !objectives.isEmpty() && objectiveIndex < objectives.size();
        if (hasObjective) {
            if (autopilot != null && autopilot.hasArrived()) {
                objectiveIndex++;
                hasObjective = objectiveIndex < objectives.size();
            }
            if (hasObjective) {
                var obj = objectives.get(objectiveIndex);
                if (strategicWaypoints.isEmpty() || autopilot.hasArrived()) {
                    strategicWaypoints = List.of(obj);
                    if (autopilot != null) {
                        autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), depth, heading, lastSpeed);
                    }
                }
            }
        }

        boolean needReplan = !hasObjective && (stateChanged || strategicWaypoints.isEmpty());

        // State-specific replan triggers
        if (!needReplan) {
            switch (state) {
                case PATROL -> {
                    if (autopilot != null && autopilot.hasArrived()) {
                        autopilot.advanceWaypoint(pos.x(), pos.y(), depth, heading, lastSpeed);
                        if (autopilot.currentWaypointIndex() >= strategicWaypoints.size()) {
                            needReplan = true;
                        }
                    }
                    if (autopilot != null && autopilot.isBlocked()) {
                        needReplan = true;
                    }
                }
                case TRACKING -> {
                    if (hasTrackedContact && !Double.isNaN(lastStrategicTargetX)) {
                        double targetMoved = Math.sqrt(Math.pow(trackedX - lastStrategicTargetX, 2)
                                + Math.pow(trackedY - lastStrategicTargetY, 2));
                        if (targetMoved > REPLAN_TARGET_MOVE_DIST) needReplan = true;
                    }
                    if (autopilot != null && (autopilot.hasArrived() || autopilot.isBlocked())) {
                        needReplan = true;
                    }
                }
                case CHASE -> {
                    if (hasTrackedContact && !Double.isNaN(lastStrategicTargetX)) {
                        double targetMoved = Math.sqrt(Math.pow(trackedX - lastStrategicTargetX, 2)
                                + Math.pow(trackedY - lastStrategicTargetY, 2));
                        if (targetMoved > REPLAN_TARGET_MOVE_DIST) needReplan = true;
                    }
                    if (autopilot != null && (autopilot.hasArrived() || autopilot.isBlocked())) {
                        needReplan = true;
                    }
                }
                case EVADE -> {
                    if (autopilot != null && autopilot.isBlocked()) {
                        needReplan = true;
                    }
                }
                case RAM -> {} // RAM bypasses autopilot
            }
        }

        // Enforce replan cooldown to prevent thrashing when the sub is in
        // a difficult situation (e.g., emergency surface near terrain). Each
        // replan produces a new route that may conflict with the previous one,
        // and replanning every tick makes it worse. State changes bypass the
        // cooldown since they represent genuine strategic shifts.
        if (needReplan && !stateChanged && (tick - lastReplanTick) < REPLAN_COOLDOWN_TICKS) {
            needReplan = false;
        }

        if (needReplan && state != State.RAM) {
            strategicWaypoints = switch (state) {
                case PATROL -> generatePatrolWaypoints(pos.x(), pos.y(), heading, battleArea);
                case TRACKING -> {
                    if (hasTrackedContact) {
                        var contact = buildTrackedContact(pos, ticksSinceContact);
                        yield generateTrackingWaypoints(pos.x(), pos.y(), heading,
                                lastContactBearing, contact);
                    }
                    yield generatePatrolWaypoints(pos.x(), pos.y(), heading, battleArea);
                }
                case CHASE -> {
                    if (hasTrackedContact) {
                        var contact = buildTrackedContact(pos, ticksSinceContact);
                        yield generateChaseWaypoints(pos.x(), pos.y(), heading, contact,
                                input.activeSonarCooldownTicks() == 0);
                    }
                    yield generatePatrolWaypoints(pos.x(), pos.y(), heading, battleArea);
                }
                case EVADE -> generateEvadeWaypoints(pos.x(), pos.y(),
                        lastContactBearing, terrain);
                case RAM -> List.of(); // not reached
            };

            if (autopilot != null && !strategicWaypoints.isEmpty()) {
                autopilot.setWaypoints(strategicWaypoints, pos.x(), pos.y(), depth, heading, lastSpeed);
                lastStrategicTargetX = strategicWaypoints.getFirst().x();
                lastStrategicTargetY = strategicWaypoints.getFirst().y();
                lastReplanTick = tick;
            }
        }

        // =================================================================
        // Step 4: Execute controls
        // =================================================================
        if (state == State.RAM) {
            // RAM bypasses autopilot: direct steering and full throttle
            double rudder = 0;
            if (hasTrackedContact && trackedDist > 50) {
                double ramBearing = Math.atan2(trackedX - pos.x(), trackedY - pos.y());
                if (ramBearing < 0) ramBearing += 2 * Math.PI;
                double diff = angleDiff(ramBearing, heading);
                rudder = Math.clamp(diff * 3.0, -1, 1);
            } else if (lastContactTick >= 0) {
                double diff = angleDiff(lastContactBearing, heading);
                rudder = Math.clamp(diff * 3.0, -1, 1);
            }

            output.setRudder(rudder);
            output.setSternPlanes(0);
            output.setThrottle(RAM_THROTTLE);
            output.setBallast(0.5);
        } else if (autopilot != null) {
            // Autopilot handles all navigation
            autopilot.tick(input, output);
        }

        // =================================================================
        // Step 5: Active sonar decisions
        // =================================================================
        switch (state) {
            case PATROL -> {
                if (bestContact != null) {
                    patrolSilenceTicks = 0;
                } else {
                    patrolSilenceTicks++;
                }
                if (patrolSilenceTicks > PATROL_SILENCE_PING_TICKS
                        && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                    patrolSilenceTicks = 0;
                }
            }
            case CHASE -> {
                if (hasTrackedContact && input.activeSonarCooldownTicks() == 0) {
                    boolean nearPredictedPos = trackedDist < TRACKED_PING_RANGE;
                    boolean contactStale = ticksSinceContact > 500;
                    boolean searchAreaGrowing = uncertaintyRadius > 1000;
                    boolean wantFiringFix = trackedDist < 2500 && !rangeConfirmedByActive;
                    if ((nearPredictedPos && contactStale)
                            || (searchAreaGrowing && contactStale)
                            || wantFiringFix) {
                        output.activeSonarPing();
                    }
                }
            }
            case RAM -> {
                if (ticksSinceContact > 100 && input.activeSonarCooldownTicks() == 0) {
                    output.activeSonarPing();
                }
            }
            default -> {}
        }

        // =================================================================
        // Step 6: Status, firing solution, contact estimate
        // =================================================================
        String stateTag = state.name().substring(0, 1);
        boolean tailing = (state == State.CHASE && isBehindTarget());
        if (tailing) stateTag = "C/TAIL";
        double floorBelow = terrain.elevationAt(pos.x(), pos.y());
        double immediateGap = depth - floorBelow;
        String apStatus = autopilot != null ? autopilot.lastStatus() : "";
        if (!apStatus.isEmpty()) stateTag += "/" + apStatus.charAt(0);
        output.setStatus(String.format("%s f:%.0f g:%.0f",
                stateTag, -floorBelow, immediateGap));

        // Torpedo firing solution + launch
        if (hasTrackedContact && (state == State.CHASE || state == State.RAM)
                && trackedDist < 2500 && trackedDist > 200
                && contactAlive > 0.3) {
            double solutionAge = (tick - trackedLastFixTick) / 50.0;
            if (solutionAge < 30 && !Double.isNaN(trackedHeading) && trackedSpeed > 0
                    && uncertaintyRadius < 300) {
                double quality = Math.clamp(1.0 - uncertaintyRadius / 300.0, 0.1, 1.0);
                output.publishFiringSolution(new FiringSolution(
                        trackedX, trackedY, trackedHeading, trackedSpeed, quality));
            }

            // Launch torpedo if ready (needs active fix for targeting data)
            if (input.self().torpedoesRemaining() > 0
                    && tick - lastTorpedoLaunchTick > 750 // 15s cooldown
                    && rangeConfirmedByActive
                    && solutionAge < 15) {
                double bearingToTarget = Math.atan2(trackedX - pos.x(), trackedY - pos.y());
                if (bearingToTarget < 0) bearingToTarget += 2 * Math.PI;
                double headingError = angleDiff(bearingToTarget, heading);
                if (Math.abs(headingError) < Math.toRadians(30)) {
                    String missionData = String.format("%.0f,%.0f,%.0f",
                            trackedX, trackedY, -80.0);
                    output.launchTorpedo(new TorpedoLaunchCommand(
                            bearingToTarget, 0, 20.0, missionData));
                    lastTorpedoLaunchTick = tick;
                }
            }
        }

        // Publish strategic waypoints for viewer visualization
        for (int i = 0; i < strategicWaypoints.size(); i++) {
            var sw = strategicWaypoints.get(i);
            boolean active = autopilot != null && i == autopilot.currentWaypointIndex();
            output.publishStrategicWaypoint(
                    new Waypoint(sw.x(), sw.y(), sw.preferredDepth(), active), sw.purpose());
        }

        // Publish tracked contact estimate
        if (hasTrackedContact) {
            double posConf = refUncertainty / (refUncertainty + uncertaintyRadius);
            double conf = contactAlive * posConf;
            String label = uncertaintyRadius < 100 ? "ping" : "passive";
            output.publishContactEstimate(new ContactEstimate(
                    trackedX, trackedY, conf, contactAlive, uncertaintyRadius,
                    trackedHeading, trackedSpeed, label));
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────

    private TrackedContact buildTrackedContact(Vec3 pos, long ticksSinceContact) {
        double dx = trackedX - pos.x();
        double dy = trackedY - pos.y();
        double dist = Math.sqrt(dx * dx + dy * dy);
        return new TrackedContact(
                trackedX, trackedY, trackedHeading, trackedSpeed,
                dist, isBehindTarget(), ticksSinceContact,
                ticksSinceContact > 500);
    }

    // State machine helpers

    private void enterState(State newState, long tick) {
        state = newState;
        if (newState == State.PATROL) {
            estimatedRange = Double.MAX_VALUE;
            rangeConfirmedByActive = false;
            patrolSilenceTicks = 0;
            if (hasTrackedContact) {
                lastKnownContactX = trackedX;
                lastKnownContactY = trackedY;
            }
            hasTrackedContact = false;
            trackedHeading = Double.NaN;
            contactAlive = 0;
            uncertaintyRadius = 0;
            pingFixHistory.clear();
            lastStrategicTargetX = Double.NaN;
            lastStrategicTargetY = Double.NaN;
        } else if (newState == State.CHASE) {
            chaseStartTick = tick;
            lastStrategicTargetX = Double.NaN;
            lastStrategicTargetY = Double.NaN;
        }
    }

    private boolean isBehindTarget() {
        if (Double.isNaN(trackedHeading)) return false;
        double diff = Math.abs(angleDiff(trackedHeading, lastContactBearing));
        return diff < BEHIND_ARC;
    }

    private boolean shouldEvade() {
        if (lastContactSE > 25.0 && hasTrackedContact && !Double.isNaN(trackedHeading)
                && estimatedRange < 2000) {
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

    private double preferredDepthBelowThermocline(double currentDepth) {
        if (thermalLayers.isEmpty()) return Double.NaN;
        double shallowest = thermalLayers.getFirst().depth();
        for (var layer : thermalLayers) {
            if (layer.depth() > shallowest) shallowest = layer.depth();
        }
        return shallowest - 30;
    }

    // ── Terrain utilities (kept for strategic waypoint generation) ──

    private double safeDepthAt(double x, double y, TerrainMap terrain) {
        double floor = terrain.elevationAt(x, y);
        double target = floor + FLOOR_CLEARANCE;
        return clampTarget(target);
    }

    private double stealthDepthAt(double x, double y, TerrainMap terrain) {
        double floor = terrain.elevationAt(x, y);
        double target = floor + FLOOR_CLEARANCE * 0.6;
        return clampTarget(target);
    }

    private double clampTarget(double target) {
        if (target < depthLimit) target = depthLimit;
        if (target > MIN_DEPTH) target = MIN_DEPTH;
        return target;
    }

    private static double worstFloorNear(double x, double y, double radius, TerrainMap terrain) {
        double worst = terrain.elevationAt(x, y);
        for (int deg = 0; deg < 360; deg += 45) {
            double brg = Math.toRadians(deg);
            double floor = terrain.elevationAt(x + Math.sin(brg) * radius, y + Math.cos(brg) * radius);
            if (floor > worst) worst = floor;
        }
        return worst;
    }

    // ── Geometry ────────────────────────────────────────────────────

    public static double angleDiff(double a, double b) {
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

    // ── Strategic waypoint generation ───────────────────────────────

    public List<StrategicWaypoint> generatePatrolWaypoints(double x, double y,
                                                            double heading, BattleArea area) {
        var waypoints = new ArrayList<StrategicWaypoint>();
        int numPoints = 5;
        double arenaExtent = area.extent();

        double targetX, targetY;
        if (!Double.isNaN(lastKnownContactX)) {
            targetX = lastKnownContactX;
            targetY = lastKnownContactY;
        } else {
            double distFromCenter = Math.sqrt(x * x + y * y);
            if (distFromCenter > arenaExtent * 0.3) {
                targetX = 0;
                targetY = 0;
            } else {
                targetX = x + 3000 * Math.sin(heading);
                targetY = y + 3000 * Math.cos(heading);
            }
        }

        double toTargetAngle = Math.atan2(targetX - x, targetY - y);
        if (toTargetAngle < 0) toTargetAngle += 2 * Math.PI;
        double distToTarget = Math.sqrt(Math.pow(targetX - x, 2) + Math.pow(targetY - y, 2));

        for (int i = 0; i < numPoints; i++) {
            double angle;
            double radius;
            if (i == 0) {
                angle = toTargetAngle;
                radius = Math.min(distToTarget * 0.6, 3000);
                radius = Math.max(radius, 1500);
            } else {
                double spread = Math.PI * 0.4;
                angle = toTargetAngle + spread * (i % 2 == 0 ? 1 : -1) * ((i + 1) / 2.0) / (numPoints / 2.0);
                radius = 2000 + i * 500;
            }

            double px = x + radius * Math.sin(angle);
            double py = y + radius * Math.cos(angle);

            double margin = BOUNDARY_TURN_DIST + 200;
            if (area.distanceToBoundary(px, py) < margin) {
                px *= 0.7;
                py *= 0.7;
            }

            boolean waypointUnsafe = pathPlanner != null
                    ? !pathPlanner.isSafe(px, py)
                    : worstFloorNear(px, py, 200, terrain) > SHALLOW_WATER_LIMIT;
            if (waypointUnsafe) {
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
                if (!relocated) continue;
            }

            double safeZ = stealthDepthAt(px, py, terrain);

            Purpose purpose = Purpose.PATROL;
            NoisePolicy noise = NoisePolicy.NORMAL;
            if (i == 2 && !thermalLayers.isEmpty()) {
                double thermoclineDepth = thermalLayers.getFirst().depth();
                for (var layer : thermalLayers) {
                    if (layer.depth() > thermoclineDepth) thermoclineDepth = layer.depth();
                }
                safeZ = Math.max(thermoclineDepth + 10, MIN_DEPTH);
                purpose = Purpose.PING_POSITION;
                noise = NoisePolicy.NORMAL;
            }

            waypoints.add(new StrategicWaypoint(
                    px, py, safeZ, purpose, noise,
                    MovementPattern.DIRECT, 200, -1));
        }

        return waypoints;
    }

    public List<StrategicWaypoint> generateTrackingWaypoints(double posX, double posY,
                                                       double heading, double contactBearing,
                                                       TrackedContact contact) {
        var waypoints = new ArrayList<StrategicWaypoint>();

        double perpBearing = normalizeBearing(contactBearing + Math.PI / 2);
        double perpBearing2 = normalizeBearing(contactBearing - Math.PI / 2);
        double diff1 = Math.abs(angleDiff(perpBearing, heading));
        double diff2 = Math.abs(angleDiff(perpBearing2, heading));
        double chosenPerp = diff1 < diff2 ? perpBearing : perpBearing2;

        double crossDist = Math.min(contact.distance() * 0.3, 1000);
        crossDist = Math.max(crossDist, 400);
        double crossX = posX + Math.sin(chosenPerp) * crossDist;
        double crossY = posY + Math.cos(chosenPerp) * crossDist;
        double crossDepth = preferredDepthBelowThermocline(-100);
        if (Double.isNaN(crossDepth)) crossDepth = stealthDepthAt(crossX, crossY, terrain);

        waypoints.add(new StrategicWaypoint(
                crossX, crossY, crossDepth, Purpose.INVESTIGATE, NoisePolicy.QUIET,
                MovementPattern.DIRECT, 300, contact.speed() + 1));

        double targetDepth = preferredDepthBelowThermocline(-100);
        if (Double.isNaN(targetDepth)) targetDepth = stealthDepthAt(contact.x(), contact.y(), terrain);

        waypoints.add(new StrategicWaypoint(
                contact.x(), contact.y(), targetDepth, Purpose.INTERCEPT, NoisePolicy.QUIET,
                MovementPattern.DIRECT, 400, contact.speed() + 1));

        return waypoints;
    }

    public List<StrategicWaypoint> generateChaseWaypoints(double posX, double posY,
                                                    double heading, TrackedContact contact,
                                                    boolean sonarCooldownReady) {
        var waypoints = new ArrayList<StrategicWaypoint>();

        double targetX = contact.x();
        double targetY = contact.y();
        double dist = contact.distance();

        if (!Double.isNaN(contact.heading()) && dist < 1500 && dist > RAM_RANGE * 2) {
            double offsetFraction = 1.0 - (dist - RAM_RANGE * 2) / (1500 - RAM_RANGE * 2);
            double offset = BEHIND_OFFSET * offsetFraction;
            double sternX = targetX - offset * Math.sin(contact.heading());
            double sternY = targetY - offset * Math.cos(contact.heading());
            if (pathPlanner == null || pathPlanner.isSafe(sternX, sternY)) {
                targetX = sternX;
                targetY = sternY;
            }
        }

        MovementPattern pattern;
        NoisePolicy noise;
        if (dist > 4000) {
            pattern = MovementPattern.ZIGZAG_TMA;
            noise = NoisePolicy.SPRINT;
        } else if (dist > 2000) {
            pattern = MovementPattern.SPRINT_DRIFT;
            noise = NoisePolicy.NORMAL;
        } else if (contact.behindTarget() && dist < 1000) {
            pattern = MovementPattern.DIRECT;
            noise = NoisePolicy.QUIET;
        } else {
            pattern = MovementPattern.DIRECT;
            noise = NoisePolicy.NORMAL;
        }

        double chaseDepth = preferredDepthBelowThermocline(-100);
        if (Double.isNaN(chaseDepth)) chaseDepth = -200;
        if (noise == NoisePolicy.SPRINT) {
            double minDepthForQuiet = -(11.0 - BASE_CAVITATION_SPEED) / CAVITATION_DEPTH_FACTOR;
            chaseDepth = Math.min(chaseDepth, minDepthForQuiet - 20);
        }

        waypoints.add(new StrategicWaypoint(
                targetX, targetY, chaseDepth, Purpose.INTERCEPT, noise,
                pattern, 200, contact.speed() + 1));

        if (contact.contactStale() && sonarCooldownReady) {
            waypoints.add(new StrategicWaypoint(
                    contact.x(), contact.y(), chaseDepth, Purpose.PING_POSITION,
                    NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1));
        }

        return waypoints;
    }

    public List<StrategicWaypoint> generateEvadeWaypoints(double posX, double posY,
                                                    double threatBearing, TerrainMap terrain) {
        double perpBearing1 = normalizeBearing(threatBearing + Math.PI / 2);
        double perpBearing2 = normalizeBearing(threatBearing - Math.PI / 2);
        double escapeDist = 2000;

        double tx1 = posX + Math.sin(perpBearing1) * escapeDist;
        double ty1 = posY + Math.cos(perpBearing1) * escapeDist;
        double tx2 = posX + Math.sin(perpBearing2) * escapeDist;
        double ty2 = posY + Math.cos(perpBearing2) * escapeDist;

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

        double evadeDepth = depthLimit + 20;

        return List.of(new StrategicWaypoint(
                tx, ty, evadeDepth, Purpose.EVADE, NoisePolicy.SILENT,
                MovementPattern.DIRECT, 500, -1));
    }
}
