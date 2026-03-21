package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Clean autopilot for ClaudeAttackSub. Uses lookahead steering on densified
 * A* routes for smooth tracking. Minimal terrain avoidance (emergency only).
 */
final class ClaudeAutopilot {
    private static final double MIN_DEPTH = -20.0;
    private static final double CRUSH_SAFETY_MARGIN = 50.0;
    private static final double FLOOR_CLEARANCE = 78.0;
    private static final double EMERGENCY_GAP = 40.0;
    private static final double WARNING_GAP = 70.0;
    private static final double BOUNDARY_MARGIN = 850.0;
    private static final double NAV_SPACING = 300.0;
    private static final double NAV_ACCEPTANCE = 150.0;
    private static final double LOOKAHEAD_BASE = 240.0;
    private static final double LOOKAHEAD_PER_MPS = 17.0;
    private static final double MAX_LOOKAHEAD = 580.0;
    private static final double MAX_RUDDER = 0.55;
    private static final double MAX_PLANES = 0.55;

    static final double[] TURN_RADIUS = {
            0, 405, 254, 218, 213, 219, 233, 250, 268, 289, 310, 332, 354, 377, 400, 424
    };
    static final double[] DEPTH_RATE = {
            0, 0.02, 0.09, 0.21, 0.37, 0.59, 0.87, 1.21, 1.58, 1.99, 2.42, 2.86, 3.30, 3.75, 4.20, 4.64
    };

    private final TerrainMap terrain;
    private final BattleArea battleArea;
    private final PathPlanner pathPlanner;
    private final double depthLimit;
    private final double maxSubSpeed;
    private final List<ThermalLayer> thermalLayers;

    private List<StrategicWaypoint> strategicWaypoints = List.of();
    private int strategicWaypointIndex;
    private final List<Vec3> route = new ArrayList<>();
    private int routeIndex;
    private boolean arrived;
    private boolean blocked;
    private String lastStatus = "";

    ClaudeAutopilot(MatchContext context) {
        this.terrain = context.terrain();
        this.battleArea = context.config().battleArea();
        // Stronger depth preference: comfortable at -400m, 5x penalty for shallow cells.
        // This makes the A* more aggressively prefer deep routes for stealth.
        this.pathPlanner = new PathPlanner(context.terrain(), -90.0, 225, 75, 400.0, 5.0);
        this.depthLimit = context.config().crushDepth() + CRUSH_SAFETY_MARGIN;
        this.maxSubSpeed = context.config().maxSubSpeed();
        this.thermalLayers = context.thermalLayers();
    }

    void setWaypoints(List<StrategicWaypoint> waypoints,
                      double posX, double posY, double posZ,
                      double heading, double speed) {
        this.strategicWaypoints = List.copyOf(waypoints);
        this.strategicWaypointIndex = 0;
        this.routeIndex = 0;
        this.arrived = false;
        this.blocked = false;
        this.route.clear();
        if (!strategicWaypoints.isEmpty()) {
            planRoute(posX, posY, posZ, heading, speed, strategicWaypoints.getFirst());
        }
    }

    /**
     * Plans a continuous route through a chain of strategic waypoints.
     * The A* corridors are stitched together and depth is planned with a
     * backward pass so the sub transitions smoothly between waypoints.
     */
    void setWaypointsChain(List<StrategicWaypoint> waypoints,
                            double posX, double posY, double posZ,
                            double heading, double speed) {
        this.strategicWaypoints = List.copyOf(waypoints);
        this.strategicWaypointIndex = 0;
        this.routeIndex = 0;
        this.arrived = false;
        this.blocked = false;
        this.route.clear();

        if (waypoints.isEmpty()) return;

        double expectedSpeed = speedFor(waypoints.getFirst());
        double ratio = depthChangeRatio(expectedSpeed);
        double radius = turnRadiusAtSpeed(expectedSpeed);

        // Build one continuous A* route through all waypoints
        var allRaw = new ArrayList<Vec3>();
        double cx = posX, cy = posY;
        double deepestPreferred = clampDepth(waypoints.getFirst().preferredDepth());

        for (var wp : waypoints) {
            double pref = clampDepth(wp.preferredDepth());
            if (pref < deepestPreferred) deepestPreferred = pref;
            var segment = pathPlanner.findPath(cx, cy, wp.x(), wp.y(), pref, ratio, radius);
            if (!segment.isEmpty()) {
                // Skip the first point if it overlaps with the end of the previous segment
                int start = (!allRaw.isEmpty() && !segment.isEmpty()
                        && allRaw.getLast().horizontalDistanceTo(segment.getFirst()) < 100) ? 1 : 0;
                for (int i = start; i < segment.size(); i++) {
                    allRaw.add(segment.get(i));
                }
            } else {
                allRaw.add(new Vec3(wp.x(), wp.y(), safeDepth(wp.x(), wp.y(), pref)));
            }
            cx = wp.x();
            cy = wp.y();
        }

        if (allRaw.isEmpty()) {
            var wp = waypoints.getFirst();
            route.add(new Vec3(wp.x(), wp.y(), safeDepth(wp.x(), wp.y(), deepestPreferred)));
            blocked = true;
            return;
        }

        // Densify the continuous route
        var dense = densify(allRaw, deepestPreferred);

        // Backward depth pass: if a later waypoint needs to be shallow,
        // propagate the constraint backward so the sub starts rising earlier.
        for (int i = dense.size() - 2; i >= 0; i--) {
            var cur = dense.get(i);
            var next = dense.get(i + 1);
            double legDist = cur.horizontalDistanceTo(next);
            double maxChange = legDist * ratio;
            // If next waypoint is shallower, this one must already be rising
            if (cur.z() < next.z() - maxChange) {
                dense.set(i, new Vec3(cur.x(), cur.y(), next.z() - maxChange));
            }
        }
        // Forward pass: can't dive faster than the rate allows
        double prevZ = posZ;
        for (int i = 0; i < dense.size(); i++) {
            var wp = dense.get(i);
            double legDist = i == 0
                    ? hdist(posX, posY, wp.x(), wp.y())
                    : dense.get(i - 1).horizontalDistanceTo(wp);
            double maxChange = legDist * ratio;
            double clampedZ = Math.max(wp.z(), prevZ - maxChange);
            clampedZ = Math.min(clampedZ, prevZ + maxChange);
            if (clampedZ != wp.z()) {
                dense.set(i, new Vec3(wp.x(), wp.y(), clampedZ));
            }
            prevZ = dense.get(i).z();
        }

        route.addAll(dense);
        if (route.isEmpty()) {
            var wp = waypoints.getFirst();
            route.add(new Vec3(wp.x(), wp.y(), safeDepth(wp.x(), wp.y(), deepestPreferred)));
        }
    }

    void tick(SubmarineInput input, SubmarineOutput output) {
        var self = input.self();
        var pos = self.pose().position();
        double heading = self.pose().heading();
        double pitch = self.pose().pitch();
        double speed = self.velocity().speed();
        double verticalSpeed = self.velocity().linear().z();
        double depth = pos.z();

        StrategicWaypoint current = strategicWaypoints.isEmpty()
                ? null : strategicWaypoints.get(strategicWaypointIndex);
        if (current == null) {
            blocked = true;
            lastStatus = "NO ROUTE";
            output.setThrottle(0);
            output.setRudder(0);
            output.setSternPlanes(0);
            output.setBallast(0.5);
            return;
        }

        double distToStrategic = hdist(pos.x(), pos.y(), current.x(), current.y());
        // For chained waypoints: advance through intermediate strategic waypoints
        // as the sub passes through them (the route is continuous).
        if (distToStrategic <= current.arrivalRadius()) {
            if (strategicWaypointIndex < strategicWaypoints.size() - 1) {
                strategicWaypointIndex++;
                current = strategicWaypoints.get(strategicWaypointIndex);
                distToStrategic = hdist(pos.x(), pos.y(), current.x(), current.y());
            }
        }
        arrived = strategicWaypointIndex >= strategicWaypoints.size() - 1
                && distToStrategic <= current.arrivalRadius();

        advanceRouteIndex(pos.x(), pos.y());

        // Lookahead steering: compute a point along the route ahead of the sub
        double lookahead = Math.min(MAX_LOOKAHEAD, LOOKAHEAD_BASE + speed * LOOKAHEAD_PER_MPS);
        Vec3 target = route.isEmpty()
                ? new Vec3(current.x(), current.y(), current.preferredDepth())
                : computeLookahead(pos.x(), pos.y(), lookahead);

        double targetBearing = norm(Math.atan2(target.x() - pos.x(), target.y() - pos.y()));
        double headingErr = adiff(targetBearing, heading);
        double rudder = Math.clamp(headingErr * 2.0, -MAX_RUDDER, MAX_RUDDER);

        // Speed: higher for better performance, limited by upcoming turns
        double desiredSpeed = speedFor(current);
        desiredSpeed = Math.min(desiredSpeed, turnSpeedLimit(pos.x(), pos.y(), heading));

        // Border avoidance
        if (battleArea.distanceToBoundary(pos.x(), pos.y()) < BOUNDARY_MARGIN) {
            double centerBearing = norm(Math.atan2(-pos.x(), -pos.y()));
            double centerErr = adiff(centerBearing, heading);
            rudder = Math.clamp(centerErr * 2.5, -MAX_RUDDER, MAX_RUDDER);
            desiredSpeed = Math.min(desiredSpeed, 7.0);
            lastStatus = "BORDER";
        } else {
            lastStatus = "TRACK";
        }

        // Depth control: prefer as deep as possible, scan route ahead for terrain
        double routeWorstFloor = worstFloorAhead(pos.x(), pos.y(), lookahead + 800);
        double targetDepth = depthTarget(current, target, routeWorstFloor);
        double depthErr = targetDepth - depth;
        double desiredPitch = Math.clamp(depthErr * 0.012 - verticalSpeed * 0.25, -0.30, 0.30);
        double sternPlanes = Math.clamp((desiredPitch - pitch) * 3.0, -MAX_PLANES, MAX_PLANES);
        double ballast = Math.clamp(0.5 + depthErr * 0.004 - verticalSpeed * 0.07, 0.08, 0.92);
        double throttle = throttle(desiredSpeed);

        // Emergency floor avoidance: last resort only
        double floorBelow = terrain.elevationAt(pos.x(), pos.y());
        double gap = depth - floorBelow;
        if (gap < EMERGENCY_GAP || (gap < WARNING_GAP && verticalSpeed < -0.4)) {
            sternPlanes = MAX_PLANES;
            ballast = 0.95;
            throttle = gap < EMERGENCY_GAP ? -0.3 : Math.min(throttle, 0.15);
            rudder = Math.clamp(rudder, -0.3, 0.3);
            lastStatus = "PULL UP";
            blocked = floorBelow > -30;
        } else {
            blocked = route.isEmpty();
        }

        // Surface avoidance
        if (depth > MIN_DEPTH) {
            sternPlanes = -0.45;
            ballast = 0.18;
            throttle = Math.min(throttle, 0.2);
            lastStatus = "SURFACE";
        }

        output.setRudder(rudder);
        output.setSternPlanes(sternPlanes);
        output.setThrottle(throttle);
        output.setBallast(ballast);

        for (int i = 0; i < route.size(); i++) {
            var wp = route.get(i);
            output.publishWaypoint(new Waypoint(wp.x(), wp.y(), wp.z(), i == routeIndex));
        }
    }

    boolean hasArrived() { return arrived; }
    boolean isBlocked() { return blocked; }
    int currentWaypointIndex() { return strategicWaypointIndex; }
    int currentNavIndex() { return routeIndex; }
    List<Vec3> navWaypoints() { return List.copyOf(route); }
    List<StrategicWaypoint> strategicWaypoints() { return strategicWaypoints; }
    String lastStatus() { return lastStatus; }
    PathPlanner pathPlanner() { return pathPlanner; }

    double distanceToStrategic(double x, double y) {
        if (strategicWaypoints.isEmpty()) return Double.POSITIVE_INFINITY;
        var c = strategicWaypoints.get(strategicWaypointIndex);
        return hdist(x, y, c.x(), c.y());
    }

    // ── Route planning ──────────────────────────────────────────────

    private void planRoute(double posX, double posY, double posZ,
                           double heading, double speed, StrategicWaypoint target) {
        route.clear();
        routeIndex = 0;

        double expectedSpeed = speedFor(target);
        double preferredDepth = clampDepth(target.preferredDepth());
        var raw = pathPlanner.findPath(posX, posY, target.x(), target.y(),
                preferredDepth, depthChangeRatio(expectedSpeed), turnRadiusAtSpeed(expectedSpeed));

        if (raw.isEmpty()) {
            route.add(new Vec3(target.x(), target.y(), safeDepth(target.x(), target.y(), preferredDepth)));
            blocked = true;
            return;
        }

        route.addAll(densify(raw, preferredDepth));
        if (route.isEmpty()) {
            route.add(new Vec3(target.x(), target.y(), safeDepth(target.x(), target.y(), preferredDepth)));
        }
    }

    private List<Vec3> densify(List<Vec3> raw, double preferredDepth) {
        var dense = new ArrayList<Vec3>();
        double carried = 0;
        for (int i = 1; i < raw.size(); i++) {
            Vec3 from = raw.get(i - 1);
            Vec3 to = raw.get(i);
            double seg = from.horizontalDistanceTo(to);
            if (seg < 1) continue;

            double next = NAV_SPACING - carried;
            while (next <= seg) {
                double t = next / seg;
                double x = lerp(from.x(), to.x(), t);
                double y = lerp(from.y(), to.y(), t);
                double z = lerp(from.z(), to.z(), t);
                // Scan ahead along this leg to find worst floor within NAV_SPACING
                double worstFloor = terrain.elevationAt(x, y);
                double scanEnd = Math.min(next + NAV_SPACING, seg);
                for (double s = next; s <= scanEnd; s += 50) {
                    double st = s / seg;
                    worstFloor = Math.max(worstFloor,
                            terrain.elevationAt(lerp(from.x(), to.x(), st), lerp(from.y(), to.y(), st)));
                }
                double safeZ = clampDepth(Math.max(Math.max(z, preferredDepth), worstFloor + FLOOR_CLEARANCE));
                dense.add(new Vec3(x, y, safeZ));
                next += NAV_SPACING;
            }
            carried = seg - (next - NAV_SPACING);
            if (carried >= NAV_SPACING) carried = 0;
        }
        Vec3 last = raw.getLast();
        double safeFinalZ = safeDepth(last.x(), last.y(), Math.max(last.z(), preferredDepth));
        if (dense.isEmpty() || dense.getLast().horizontalDistanceTo(last) > 100) {
            dense.add(new Vec3(last.x(), last.y(), safeFinalZ));
        } else {
            dense.set(dense.size() - 1, new Vec3(last.x(), last.y(), safeFinalZ));
        }
        return dense;
    }

    // ── Steering ────────────────────────────────────────────────────

    private void advanceRouteIndex(double x, double y) {
        if (route.isEmpty()) return;
        // Jump ahead if closer to a future waypoint
        int best = routeIndex;
        double bestDist = hdist(x, y, route.get(routeIndex).x(), route.get(routeIndex).y());
        for (int i = routeIndex + 1; i < Math.min(route.size(), routeIndex + 4); i++) {
            double d = hdist(x, y, route.get(i).x(), route.get(i).y());
            if (d + 20 < bestDist) { bestDist = d; best = i; }
        }
        routeIndex = best;
        // Advance past close waypoints
        while (routeIndex < route.size() - 1 &&
               hdist(x, y, route.get(routeIndex).x(), route.get(routeIndex).y()) < NAV_ACCEPTANCE) {
            routeIndex++;
        }
    }

    private Vec3 computeLookahead(double x, double y, double distance) {
        if (route.isEmpty()) return null;
        double remaining = distance;
        double ax = x, ay = y;
        for (int i = routeIndex; i < route.size(); i++) {
            Vec3 wp = route.get(i);
            double seg = hdist(ax, ay, wp.x(), wp.y());
            if (seg >= remaining) {
                double t = seg < 1 ? 1 : remaining / seg;
                return new Vec3(lerp(ax, wp.x(), t), lerp(ay, wp.y(), t), wp.z());
            }
            remaining -= seg;
            ax = wp.x(); ay = wp.y();
        }
        return route.getLast();
    }

    private double turnSpeedLimit(double x, double y, double heading) {
        if (route.isEmpty()) return maxSubSpeed;
        double limit = maxSubSpeed;
        Vec3 cur = route.get(Math.min(routeIndex, route.size() - 1));
        double curBearing = norm(Math.atan2(cur.x() - x, cur.y() - y));
        double err = Math.abs(adiff(curBearing, heading));
        if (err > Math.toRadians(70)) limit = Math.min(limit, 5.5);
        else if (err > Math.toRadians(40)) limit = Math.min(limit, 7.0);

        for (int i = routeIndex; i < route.size() - 1 && i < routeIndex + 2; i++) {
            Vec3 a = route.get(i), b = route.get(i + 1);
            double bA = norm(Math.atan2(a.x() - x, a.y() - y));
            double bB = norm(Math.atan2(b.x() - a.x(), b.y() - a.y()));
            double turn = Math.abs(adiff(bB, bA));
            if (turn > Math.toRadians(55)) {
                double d = Math.max(1, a.horizontalDistanceTo(b));
                limit = Math.min(limit, maxSpeedForTurn(turn, d));
            }
        }
        return Math.max(4.5, limit);
    }

    // ── Depth ───────────────────────────────────────────────────────

    private double depthTarget(StrategicWaypoint current, Vec3 steeringTarget, double worstFloor) {
        double desired = clampDepth(current.preferredDepth());
        if (Double.isNaN(desired)) desired = cruiseDepth();
        if (steeringTarget != null && !Double.isNaN(steeringTarget.z())) {
            desired = Math.min(desired, steeringTarget.z());
        }
        desired = Math.max(desired, worstFloor + FLOOR_CLEARANCE + 5);
        return clampDepth(desired);
    }

    private double worstFloorAhead(double x, double y, double limit) {
        double worst = terrain.elevationAt(x, y);
        if (route.isEmpty()) return worst;
        double walked = 0;
        double ax = x, ay = y;
        for (int i = routeIndex; i < route.size() && walked <= limit; i++) {
            Vec3 wp = route.get(i);
            double seg = hdist(ax, ay, wp.x(), wp.y());
            int samples = Math.max(1, (int) Math.ceil(seg / 50));
            for (int s = 1; s <= samples; s++) {
                double t = (double) s / samples;
                if (walked + seg * t > limit) break;
                worst = Math.max(worst, terrain.elevationAt(lerp(ax, wp.x(), t), lerp(ay, wp.y(), t)));
            }
            walked += seg;
            ax = wp.x(); ay = wp.y();
        }
        return worst;
    }

    double cruiseDepth() {
        if (thermalLayers.isEmpty()) return -400.0;
        double shallowest = thermalLayers.getFirst().depth();
        for (var l : thermalLayers) if (l.depth() > shallowest) shallowest = l.depth();
        return shallowest - 40.0;
    }

    // ── Utilities ───────────────────────────────────────────────────

    private double speedFor(StrategicWaypoint wp) {
        if (wp.targetSpeed() > 0) return Math.min(maxSubSpeed, wp.targetSpeed());
        return switch (wp.noise()) {
            case SILENT -> 3.5;
            case QUIET -> 5.5;
            case NORMAL -> 7.8;  // balance: deep + quiet + enough speed to reach waypoints
            case SPRINT -> 12.0;
        };
    }

    private double throttle(double speed) {
        double s = Math.clamp(speed, 0, maxSubSpeed);
        if (s <= 4.5) return lerp(0.08, 0.16, s / 4.5);
        if (s <= 7.5) return lerp(0.16, 0.24, (s - 4.5) / 3.0);
        if (s <= 9.5) return lerp(0.24, 0.40, (s - 7.5) / 2.0);
        return lerp(0.40, 1.0, (s - 9.5) / (maxSubSpeed - 9.5));
    }

    private double safeDepth(double x, double y, double preferred) {
        return clampDepth(Math.max(preferred, terrain.elevationAt(x, y) + FLOOR_CLEARANCE));
    }

    private double clampDepth(double d) {
        if (Double.isNaN(d)) return cruiseDepth();
        return Math.clamp(d, depthLimit, MIN_DEPTH);
    }

    static double turnRadiusAtSpeed(double speed) {
        return speed < 1 ? TURN_RADIUS[1] : interp(TURN_RADIUS, speed);
    }

    static double depthChangeRatio(double speed) {
        return speed < 1 ? 0 : interp(DEPTH_RATE, speed) / speed * 0.65;
    }

    static double maxSpeedForTurn(double angle, double dist) {
        if (Math.abs(angle) < Math.toRadians(5)) return 15;
        double r = dist / (2 * Math.sin(Math.abs(angle) / 2));
        for (int i = TURN_RADIUS.length - 1; i >= 2; i--) if (TURN_RADIUS[i] <= r) return i;
        return 4;
    }

    private static double interp(double[] t, double s) {
        int i = Math.max(0, Math.min(t.length - 2, (int) s));
        return t[i] * (1 - (s - i)) + t[i + 1] * (s - i);
    }

    static double hdist(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    static double adiff(double a, double b) {
        double d = a - b;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    static double norm(double b) {
        double n = b % (2 * Math.PI);
        return n < 0 ? n + 2 * Math.PI : n;
    }

    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }
}
