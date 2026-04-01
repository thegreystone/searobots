/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine;

import se.hirt.searobots.api.Waypoint;

import java.util.List;

/**
 * Tracks navigation metrics from simulation ticks. Implements SimulationListener
 * so it can be attached to any SimulationLoop. After the sim ends, call
 * {@link #getMetrics()} to retrieve the computed metrics.
 */
public final class NavMetricsTracker implements SimulationListener {

    private final SubmarineCompetition.Objectives objectives;
    private final int subIndex; // which sub in the snapshot list to track (0 for nav, 0 or 1 for combat)
    private static final double OBJ_HIT_RADIUS = 400.0;

    // Objective tracking
    private double closestToObj1 = Double.MAX_VALUE;
    private double closestToObj2AfterObj1 = Double.MAX_VALUE;
    private long obj1HitTick = -1;

    // Aggregates
    private double totalDepth;
    private double totalNoiseDb;
    private double totalSpeed;
    private double peakShallowest = Double.NEGATIVE_INFINITY;
    private double peakNoiseDb = Double.NEGATIVE_INFINITY;
    private int aliveTicks;
    private double timeOfFirstDamage = -1;
    private double timeToDeath = -1;
    private int normalTicks;
    private int maxHp = 1000;

    // Waypoint tracking
    private int waypointsReached;
    private int waypointsAttempted;
    private double timeToFirstWaypoint = -1;
    private double totalHitDistance;
    private int currentChain;
    private int longestChain;
    private double[] currentTargetWp;

    // Path tracking
    private double startX = Double.NaN, startY = Double.NaN;
    private double lastX = Double.NaN, lastY = Double.NaN;
    private double totalDistTraveled;
    private double endX, endY;

    private boolean finished;

    public NavMetricsTracker(SubmarineCompetition.Objectives objectives, int subIndex) {
        this.objectives = objectives;
        this.subIndex = subIndex;
    }

    @Override
    public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
        if (finished || submarines.size() <= subIndex) return;
        var snap = submarines.get(subIndex);
        var pos = snap.pose().position();
        double x = pos.x(), y = pos.y(), z = pos.z();

        if (Double.isNaN(startX)) {
            startX = x; startY = y;
            lastX = x; lastY = y;
            maxHp = snap.hp();
        }

        aliveTicks = (int) tick + 1;
        endX = x; endY = y;

        // Distance traveled
        double dx = x - lastX, dy = y - lastY;
        totalDistTraveled += Math.sqrt(dx * dx + dy * dy);
        lastX = x; lastY = y;

        // Depth
        totalDepth += z;
        if (z > peakShallowest) peakShallowest = z;

        // Noise
        double noiseDb = snap.noiseLevel(); // This is linear noise, not dB
        // Use sourceLevelDb from snapshot if available
        double slDb = snap.sourceLevelDb();
        totalNoiseDb += slDb;
        if (slDb > peakNoiseDb) peakNoiseDb = slDb;

        // Speed
        totalSpeed += snap.speed();

        // Objective proximity
        if (objectives != null) {
            double d1 = Math.hypot(x - objectives.x1(), y - objectives.y1());
            if (d1 < closestToObj1) closestToObj1 = d1;
            if (obj1HitTick < 0 && d1 < OBJ_HIT_RADIUS) obj1HitTick = tick;
            if (obj1HitTick >= 0) {
                double d2 = Math.hypot(x - objectives.x2(), y - objectives.y2());
                if (d2 < closestToObj2AfterObj1) closestToObj2AfterObj1 = d2;
            }
        }

        // Damage tracking
        if (snap.hp() < maxHp && timeOfFirstDamage < 0) {
            timeOfFirstDamage = tick / 50.0;
        }
        if (snap.hp() <= 0) {
            if (timeToDeath < 0) timeToDeath = tick / 50.0;
            finished = true;
        }
        if (snap.forfeited()) {
            finished = true;
        }

        // Status tracking
        String status = snap.status();
        if (status == null || status.isEmpty()) {
            normalTicks++;
        } else {
            String key = status.contains(" f:") ? status.substring(0, status.indexOf(" f:")) : status;
            if (!key.contains("EMERGENCY") && !key.contains("AVOIDING")
                    && !key.contains("PULL UP") && !key.contains("SHALLOW")
                    && !key.contains("THREE-PT") && !key.contains("DANGER")) {
                normalTicks++;
            }
        }

        // Strategic waypoint tracking from snapshot
        var stratWps = snap.strategicWaypoints();
        if (stratWps != null && !stratWps.isEmpty()) {
            Waypoint activeWp = null;
            for (var swv : stratWps) {
                if (swv.waypoint().active()) {
                    activeWp = swv.waypoint();
                    break;
                }
            }
            if (activeWp != null) {
                trackWaypoint(activeWp, x, y, tick);
            }
        } else {
            // Fall back to nav waypoints
            var navWps = snap.waypoints();
            if (navWps != null) {
                for (var wp : navWps) {
                    if (wp.active()) {
                        trackWaypoint(wp, x, y, tick);
                        break;
                    }
                }
            }
        }
    }

    private void trackWaypoint(Waypoint activeWp, double x, double y, long tick) {
        if (currentTargetWp == null) {
            currentTargetWp = new double[]{activeWp.x(), activeWp.y()};
            waypointsAttempted++;
        } else {
            double wpDist = Math.hypot(activeWp.x() - currentTargetWp[0],
                    activeWp.y() - currentTargetWp[1]);
            if (wpDist > 300) {
                double distToOld = Math.hypot(x - currentTargetWp[0], y - currentTargetWp[1]);
                if (distToOld < 400) {
                    waypointsReached++;
                    totalHitDistance += distToOld;
                    currentChain++;
                    if (currentChain > longestChain) longestChain = currentChain;
                    if (timeToFirstWaypoint < 0) timeToFirstWaypoint = tick / 50.0;
                } else {
                    currentChain = 0;
                }
                currentTargetWp = new double[]{activeWp.x(), activeWp.y()};
                waypointsAttempted++;
            }
        }
    }

    @Override
    public void onMatchEnd() {
        // Final waypoint check
        if (currentTargetWp != null && !Double.isNaN(endX)) {
            double distToFinal = Math.hypot(endX - currentTargetWp[0], endY - currentTargetWp[1]);
            if (distToFinal < 400) {
                waypointsReached++;
                totalHitDistance += distToFinal;
                currentChain++;
                if (currentChain > longestChain) longestChain = currentChain;
                if (timeToFirstWaypoint < 0) timeToFirstWaypoint = aliveTicks / 50.0;
            }
        }
        finished = true;
    }

    /** Returns the computed metrics. Call after the simulation ends. */
    public SubmarineCompetition.Metrics getMetrics() {
        double progressDist = Double.isNaN(startX) ? 0 :
                Math.hypot(endX - startX, endY - startY);
        double pathEff = totalDistTraveled > 0 ? progressDist / totalDistTraveled : 0;
        double avgHitDist = waypointsReached > 0 ? totalHitDistance / waypointsReached : -1;

        int objectivesHit = obj1HitTick >= 0 ? 1 : 0;
        if (obj1HitTick >= 0 && closestToObj2AfterObj1 < OBJ_HIT_RADIUS) objectivesHit = 2;

        return new SubmarineCompetition.Metrics(
                longestChain, avgHitDist, waypointsReached, waypointsAttempted,
                timeToFirstWaypoint, objectivesHit, closestToObj1, closestToObj2AfterObj1,
                aliveTicks > 0 ? totalDepth / aliveTicks : 0,
                peakShallowest,
                aliveTicks > 0 ? totalNoiseDb / aliveTicks : 0,
                peakNoiseDb,
                aliveTicks > 0 ? totalSpeed / aliveTicks : 0,
                pathEff,
                aliveTicks > 0 ? 100.0 * normalTicks / aliveTicks : 0,
                timeOfFirstDamage, timeToDeath
        );
    }

    /** True if objectives were both reached or the sub died/forfeited. */
    public boolean isFinished() { return finished; }

    /** Number of objectives reached (0, 1, or 2). */
    public int objectivesHit() {
        int hits = obj1HitTick >= 0 ? 1 : 0;
        if (obj1HitTick >= 0 && closestToObj2AfterObj1 < OBJ_HIT_RADIUS) hits = 2;
        return hits;
    }
}
