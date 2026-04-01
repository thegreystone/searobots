package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

final class CodexAutopilot {
    private static final double MIN_DEPTH = -20.0;
    private static final double CRUSH_SAFETY_MARGIN = 50.0;
    private static final double FLOOR_CLEARANCE = 80.0;
    private static final double EMERGENCY_GAP = 40.0;
    private static final double WARNING_GAP = 70.0;
    private static final double BOUNDARY_MARGIN = 900.0;
    private static final double FIRST_NAV_SPACING = 200.0;
    private static final double NAV_SPACING = 300.0;
    private static final double NAV_ACCEPTANCE = 115.0;
    private static final double LOOKAHEAD_BASE = 210.0;
    private static final double LOOKAHEAD_PER_MPS = 16.0;
    private static final double MAX_LOOKAHEAD = 560.0;
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

    CodexAutopilot(MatchContext context) {
        this.terrain = context.terrain();
        this.battleArea = context.config().battleArea();
        this.pathPlanner = new PathPlanner(context.terrain(), -90.0, 225.0, 75.0, 400.0, 5.0);
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
            output.setThrottle(0.0);
            output.setRudder(0.0);
            output.setSternPlanes(0.0);
            output.setBallast(0.5);
            return;
        }

        double distToStrategic = hdist(pos.x(), pos.y(), current.x(), current.y());
        arrived = distToStrategic <= current.arrivalRadius();

        advanceRouteIndex(pos.x(), pos.y());

        double lookahead = Math.min(MAX_LOOKAHEAD, LOOKAHEAD_BASE + speed * LOOKAHEAD_PER_MPS);
        Vec3 target = route.isEmpty()
                ? new Vec3(current.x(), current.y(), current.preferredDepth())
                : computeLookahead(pos.x(), pos.y(), lookahead);

        double targetBearing = norm(Math.atan2(target.x() - pos.x(), target.y() - pos.y()));
        double headingErr = adiff(targetBearing, heading);
        double rudder = Math.clamp(headingErr * 1.95, -MAX_RUDDER, MAX_RUDDER);

        double desiredSpeed = speedFor(current);
        desiredSpeed = Math.min(desiredSpeed, turnSpeedLimit(pos.x(), pos.y(), heading));
        double floorAhead = worstFloorAhead(pos.x(), pos.y(), 400.0);
        double gapAhead = depth - floorAhead;
        if (gapAhead < 100.0) {
            desiredSpeed = Math.min(desiredSpeed, 5.0 + (gapAhead / 100.0) * 3.0);
        }
        double navDistance = route.isEmpty()
                ? distToStrategic
                : hdist(pos.x(), pos.y(), route.get(Math.min(routeIndex, route.size() - 1)).x(),
                route.get(Math.min(routeIndex, route.size() - 1)).y());
        desiredSpeed = approachSpeedLimit(current, navDistance, distToStrategic, desiredSpeed);

        double distToBoundary = battleArea.distanceToBoundary(pos.x(), pos.y());
        if (distToBoundary < BOUNDARY_MARGIN) {
            double centerBearing = norm(Math.atan2(-pos.x(), -pos.y()));
            double centerErr = adiff(centerBearing, heading);
            double urgency = 1.0 - distToBoundary / BOUNDARY_MARGIN;
            rudder = Math.clamp(centerErr * (2.0 + urgency * 3.0), -MAX_RUDDER, MAX_RUDDER);
            desiredSpeed = Math.min(desiredSpeed, 4.0 + (1.0 - urgency) * 4.0);
            if (distToBoundary < 200.0) {
                desiredSpeed = -0.5;
                rudder = Math.clamp(centerErr * 5.0, -1.0, 1.0);
            }
            lastStatus = "BORDER";
        } else {
            lastStatus = "TRACK";
        }

        double routeWorstFloor = worstFloorAhead(pos.x(), pos.y(), lookahead + 700.0);
        double targetDepth = depthTarget(current, target, routeWorstFloor);
        double depthErr = targetDepth - depth;
        double desiredPitch = Math.clamp(depthErr * 0.012 - verticalSpeed * 0.25, -0.30, 0.30);
        double sternPlanes = Math.clamp((desiredPitch - pitch) * 3.0, -MAX_PLANES, MAX_PLANES);
        double ballast = Math.clamp(0.5 + depthErr * 0.004 - verticalSpeed * 0.07, 0.08, 0.92);
        double throttle = throttle(desiredSpeed);

        double floorBelow = terrain.elevationAt(pos.x(), pos.y());
        double gap = depth - floorBelow;
        if (gap < EMERGENCY_GAP || gap < WARNING_GAP) {
            sternPlanes = MAX_PLANES;
            ballast = 0.95;
            throttle = gap < EMERGENCY_GAP ? -0.3 : Math.min(throttle, 0.15);
            rudder = Math.clamp(rudder, -0.3, 0.3);
            lastStatus = "PULL UP";
            blocked = floorBelow > -30.0;
        } else {
            blocked = route.isEmpty();
        }

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
            Vec3 wp = route.get(i);
            output.publishWaypoint(new Waypoint(wp.x(), wp.y(), wp.z(), i == routeIndex));
        }
    }

    boolean hasArrived() { return arrived; }
    boolean isBlocked() { return blocked; }
    int currentWaypointIndex() { return strategicWaypointIndex; }
    List<Vec3> navWaypoints() { return List.copyOf(route); }
    String lastStatus() { return lastStatus; }
    double cruiseDepth() { return preferredCruiseDepth(thermalLayers); }

    double distanceToStrategic(double x, double y) {
        if (strategicWaypoints.isEmpty()) return Double.POSITIVE_INFINITY;
        var c = strategicWaypoints.get(strategicWaypointIndex);
        return hdist(x, y, c.x(), c.y());
    }

    private void planRoute(double posX, double posY, double posZ,
                           double heading, double speed, StrategicWaypoint target) {
        route.clear();
        routeIndex = 0;

        double expectedSpeed = speedFor(target);
        double preferredDepth = clampDepth(target.preferredDepth());
        var raw = pathPlanner.findPath(
                posX, posY, target.x(), target.y(),
                preferredDepth,
                depthChangeRatio(expectedSpeed),
                turnRadiusAtSpeed(expectedSpeed));

        if (raw.isEmpty()) {
            route.add(new Vec3(target.x(), target.y(), safeDepth(target.x(), target.y(), preferredDepth)));
            blocked = true;
            return;
        }

        route.addAll(buildRoute(raw, preferredDepth, posZ, posX, posY, expectedSpeed));
        if (route.isEmpty()) {
            route.add(new Vec3(target.x(), target.y(), safeDepth(target.x(), target.y(), preferredDepth)));
        }
    }

    private List<Vec3> buildRoute(List<Vec3> raw, double preferredDepth,
                                  double startZ, double startX, double startY,
                                  double expectedSpeed) {
        var dense = new ArrayList<Vec3>();
        double carried = 0.0;
        for (int i = 1; i < raw.size(); i++) {
            Vec3 from = raw.get(i - 1);
            Vec3 to = raw.get(i);
            double segment = from.horizontalDistanceTo(to);
            if (segment < 1.0) {
                continue;
            }
            double spacing = dense.isEmpty() ? FIRST_NAV_SPACING : NAV_SPACING;
            double next = spacing - carried;
            while (next <= segment) {
                double t = next / segment;
                double x = lerp(from.x(), to.x(), t);
                double y = lerp(from.y(), to.y(), t);
                dense.add(new Vec3(x, y, 0.0));
                next += spacing;
            }
            carried = segment - (next - spacing);
            if (carried >= spacing) {
                carried = 0.0;
            }
        }

        Vec3 last = raw.getLast();
        if (dense.isEmpty() || dense.getLast().horizontalDistanceTo(last) > 100.0) {
            dense.add(new Vec3(last.x(), last.y(), 0.0));
        }

        if (dense.isEmpty()) {
            return dense;
        }

        double[] worstFloor = new double[dense.size()];
        for (int i = 0; i < dense.size(); i++) {
            var wp = dense.get(i);
            worstFloor[i] = terrain.elevationAt(wp.x(), wp.y());
            if (i < dense.size() - 1) {
                var nextWp = dense.get(i + 1);
                double seg = wp.horizontalDistanceTo(nextWp);
                for (double s = 50.0; s < seg; s += 50.0) {
                    double t = s / seg;
                    double floor = terrain.elevationAt(
                            lerp(wp.x(), nextWp.x(), t),
                            lerp(wp.y(), nextWp.y(), t));
                    worstFloor[i] = Math.max(worstFloor[i], floor);
                }
            }
        }

        double[] depths = new double[dense.size()];
        for (int i = 0; i < dense.size(); i++) {
            depths[i] = clampDepth(Math.max(preferredDepth, worstFloor[i] + FLOOR_CLEARANCE));
        }

        double ratio = depthChangeRatio(expectedSpeed);
        for (int i = dense.size() - 2; i >= 0; i--) {
            double legDist = dense.get(i).horizontalDistanceTo(dense.get(i + 1));
            double maxChange = legDist * ratio;
            if (depths[i] < depths[i + 1] - maxChange) {
                depths[i] = depths[i + 1] - maxChange;
            }
        }

        double prevZ = startZ;
        double prevX = startX;
        double prevY = startY;
        for (int i = 0; i < dense.size(); i++) {
            var wp = dense.get(i);
            double legDist = Math.max(1.0, hdist(prevX, prevY, wp.x(), wp.y()));
            double maxChange = legDist * ratio;
            depths[i] = Math.max(depths[i], prevZ - maxChange);
            depths[i] = Math.min(depths[i], prevZ + maxChange);
            prevZ = depths[i];
            prevX = wp.x();
            prevY = wp.y();
        }

        var result = new ArrayList<Vec3>(dense.size());
        for (int i = 0; i < dense.size(); i++) {
            var wp = dense.get(i);
            result.add(new Vec3(wp.x(), wp.y(), depths[i]));
        }
        return result;
    }

    private void advanceRouteIndex(double x, double y) {
        if (route.isEmpty()) {
            return;
        }
        int best = routeIndex;
        double bestDist = hdist(x, y, route.get(routeIndex).x(), route.get(routeIndex).y());
        for (int i = routeIndex + 1; i < Math.min(route.size(), routeIndex + 4); i++) {
            double dist = hdist(x, y, route.get(i).x(), route.get(i).y());
            if (dist + 20.0 < bestDist) {
                bestDist = dist;
                best = i;
            }
        }
        routeIndex = best;
        while (routeIndex < route.size() - 1
                && hdist(x, y, route.get(routeIndex).x(), route.get(routeIndex).y()) < NAV_ACCEPTANCE) {
            routeIndex++;
        }
    }

    private Vec3 computeLookahead(double x, double y, double distance) {
        if (route.isEmpty()) return null;

        double remaining = distance;
        double ax = x;
        double ay = y;
        for (int i = routeIndex; i < route.size(); i++) {
            Vec3 wp = route.get(i);
            double seg = hdist(ax, ay, wp.x(), wp.y());
            if (seg >= remaining) {
                double t = seg < 1.0 ? 1.0 : remaining / seg;
                return new Vec3(lerp(ax, wp.x(), t), lerp(ay, wp.y(), t), wp.z());
            }
            remaining -= seg;
            ax = wp.x();
            ay = wp.y();
        }
        return route.getLast();
    }

    private double turnSpeedLimit(double x, double y, double heading) {
        if (route.isEmpty()) return maxSubSpeed;

        double limit = maxSubSpeed;
        Vec3 cur = route.get(Math.min(routeIndex, route.size() - 1));
        double curBearing = norm(Math.atan2(cur.x() - x, cur.y() - y));
        double err = Math.abs(adiff(curBearing, heading));
        if (err > Math.toRadians(70.0)) limit = Math.min(limit, 5.5);
        else if (err > Math.toRadians(40.0)) limit = Math.min(limit, 7.0);

        for (int i = routeIndex; i < route.size() - 1 && i < routeIndex + 2; i++) {
            Vec3 a = route.get(i);
            Vec3 b = route.get(i + 1);
            double bearingA = norm(Math.atan2(a.x() - x, a.y() - y));
            double bearingB = norm(Math.atan2(b.x() - a.x(), b.y() - a.y()));
            double turn = Math.abs(adiff(bearingB, bearingA));
            double dist = Math.max(1.0, a.horizontalDistanceTo(b));
            if (turn > Math.toRadians(55.0)) {
                limit = Math.min(limit, maxSpeedForTurn(turn, dist));
            }
        }
        return Math.max(4.5, limit);
    }

    private double depthTarget(StrategicWaypoint current, Vec3 steeringTarget, double worstFloor) {
        double desired = clampDepth(current.preferredDepth());
        if (Double.isNaN(desired)) desired = cruiseDepth();
        if (steeringTarget != null && !Double.isNaN(steeringTarget.z())) {
            desired = Math.min(desired, steeringTarget.z());
        }
        desired = Math.max(desired, worstFloor + FLOOR_CLEARANCE + 5.0);
        return clampDepth(desired);
    }

    private double worstFloorAhead(double x, double y, double limit) {
        double worst = terrain.elevationAt(x, y);
        if (route.isEmpty()) return worst;

        double walked = 0.0;
        double ax = x;
        double ay = y;
        for (int i = routeIndex; i < route.size() && walked <= limit; i++) {
            Vec3 wp = route.get(i);
            double seg = hdist(ax, ay, wp.x(), wp.y());
            int samples = Math.max(1, (int) Math.ceil(seg / 50.0));
            for (int s = 1; s <= samples; s++) {
                double t = (double) s / samples;
                if (walked + seg * t > limit) break;
                worst = Math.max(worst, terrain.elevationAt(
                        lerp(ax, wp.x(), t),
                        lerp(ay, wp.y(), t)));
            }
            walked += seg;
            ax = wp.x();
            ay = wp.y();
        }
        return worst;
    }

    private double approachSpeedLimit(StrategicWaypoint current, double navDistance,
                                      double strategicDistance, double limitedSpeed) {
        double limited = limitedSpeed;
        if (navDistance < 260.0) limited = Math.min(limited, 7.8);
        if (navDistance < 170.0) limited = Math.min(limited, 6.4);
        if (navDistance < 95.0) limited = Math.min(limited, 5.2);
        if (strategicDistance < Math.max(current.arrivalRadius() * 1.5, 280.0)) {
            limited = Math.min(limited, 6.0);
        }
        return limited;
    }

    private double speedFor(StrategicWaypoint wp) {
        if (wp.targetSpeed() > 0.0) return Math.min(maxSubSpeed, wp.targetSpeed());
        return switch (wp.noise()) {
            case SILENT -> 3.5;
            case QUIET -> 5.7;
            case NORMAL -> 9.4;
            case SPRINT -> 12.2;
        };
    }

    private double throttle(double speed) {
        double s = Math.clamp(speed, 0.0, maxSubSpeed);
        if (s <= 4.5) return lerp(0.08, 0.16, s / 4.5);
        if (s <= 7.5) return lerp(0.16, 0.24, (s - 4.5) / 3.0);
        if (s <= 9.5) return lerp(0.24, 0.42, (s - 7.5) / 2.0);
        return lerp(0.42, 1.0, (s - 9.5) / (maxSubSpeed - 9.5));
    }

    private double safeDepth(double x, double y, double preferred) {
        return clampDepth(Math.max(preferred, terrain.elevationAt(x, y) + FLOOR_CLEARANCE));
    }

    private double clampDepth(double d) {
        if (Double.isNaN(d)) return cruiseDepth();
        return Math.clamp(d, depthLimit, MIN_DEPTH);
    }

    static double preferredCruiseDepth(List<ThermalLayer> thermalLayers) {
        if (thermalLayers.isEmpty()) return -260.0;
        double shallowest = thermalLayers.getFirst().depth();
        for (ThermalLayer layer : thermalLayers) {
            if (layer.depth() > shallowest) shallowest = layer.depth();
        }
        return shallowest - 55.0;
    }

    static double turnRadiusAtSpeed(double speed) {
        return speed < 1.0 ? TURN_RADIUS[1] : interp(TURN_RADIUS, speed);
    }

    static double depthChangeRatio(double speed) {
        return speed < 1.0 ? 0.0 : interp(DEPTH_RATE, speed) / speed * 0.65;
    }

    static double maxSpeedForTurn(double angle, double dist) {
        if (Math.abs(angle) < Math.toRadians(5.0)) return 15.0;
        double radius = dist / (2.0 * Math.sin(Math.abs(angle) / 2.0));
        for (int i = TURN_RADIUS.length - 1; i >= 2; i--) {
            if (TURN_RADIUS[i] <= radius) return i;
        }
        return 4.2;
    }

    private static double interp(double[] table, double speed) {
        int lower = Math.max(0, Math.min(table.length - 2, (int) Math.floor(speed)));
        double fraction = speed - lower;
        return table[lower] * (1.0 - fraction) + table[lower + 1] * fraction;
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

    static double horizontalDistance(double x1, double y1, double x2, double y2) {
        return hdist(x1, y1, x2, y2);
    }

    static double angleDiff(double a, double b) {
        return adiff(a, b);
    }

    static double normalizeBearing(double bearing) {
        return norm(bearing);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
