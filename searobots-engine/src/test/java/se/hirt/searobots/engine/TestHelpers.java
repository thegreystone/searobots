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
        public final ArrayList<Waypoint> strategicWaypoints = new ArrayList<>();
        public final ArrayList<Purpose> strategicPurposes = new ArrayList<>();
        public final ArrayList<ContactEstimate> contactEstimates = new ArrayList<>();
        public TorpedoLaunchCommand launchedTorpedo;
        public int launchedTorpedoCount;
        public boolean engineClutchEngaged = true;

        @Override public void setRudder(double value) { rudder = value; }
        @Override public void setSternPlanes(double value) { sternPlanes = value; }
        @Override public void setThrottle(double value) { throttle = value; }
        @Override public void setBallast(double value) { ballast = value; }
        @Override public void activeSonarPing() { pinged = true; }
        @Override public void setStatus(String s) { status = s; }
        @Override public void publishWaypoint(Waypoint wp) { waypoints.add(wp); }
        @Override public void publishStrategicWaypoint(Waypoint wp, Purpose purpose) {
            strategicWaypoints.add(wp);
            strategicPurposes.add(purpose);
        }
        @Override public void publishContactEstimate(ContactEstimate e) { contactEstimates.add(e); }
        @Override public void launchTorpedo(TorpedoLaunchCommand command) {
            launchedTorpedo = command;
            launchedTorpedoCount++;
        }
        @Override public void setEngineClutch(boolean engaged) { engineClutchEngaged = engaged; }
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
