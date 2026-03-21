package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Shared test helpers for submarine controller testing.
 * Public so that controller-specific test packages can use them.
 */
public final class TestHelpers {

    private TestHelpers() {}

    public record TestInput(long tick, double deltaTimeSeconds,
                            SubmarineState self, EnvironmentSnapshot environment,
                            List<SonarContact> sonarContacts,
                            List<SonarContact> activeSonarReturns,
                            int activeSonarCooldownTicks)
            implements SubmarineInput {}

    public static final class CapturedOutput implements SubmarineOutput {
        public double rudder, sternPlanes, throttle, ballast;
        public boolean pinged;
        public String status;
        public final ArrayList<Waypoint> waypoints = new ArrayList<>();
        public final ArrayList<ContactEstimate> contactEstimates = new ArrayList<>();

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void setStatus(String s) { status = s; }
        @Override public void publishWaypoint(Waypoint wp) { waypoints.add(wp); }
        @Override public void publishContactEstimate(ContactEstimate e) { contactEstimates.add(e); }
    }

    public static TestInput makeInput(long tick, SubmarineEntity entity,
                                       TerrainMap terrain, CurrentField currentField) {
        var pose = new Pose(new Vec3(entity.x(), entity.y(), entity.z()),
                entity.heading(), entity.pitch(), 0);
        var vel = new Velocity(
                new Vec3(entity.speed() * Math.sin(entity.heading()),
                         entity.speed() * Math.cos(entity.heading()),
                         entity.verticalSpeed()),
                Vec3.ZERO);
        var state = new SubmarineState(pose, vel, entity.hp(), 0);
        var env = new EnvironmentSnapshot(terrain, List.of(), currentField);
        return new TestInput(tick, 1.0 / 50, state, env, List.of(), List.of(), 0);
    }
}
