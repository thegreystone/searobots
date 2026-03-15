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

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

public final class SubmarineEntity implements SubmarineOutput {

    private final VehicleConfig vehicleConfig;
    private final int id;
    private final SubmarineController controller;
    private final Color color;
    private final int maxHp;

    // state
    private double x, y, z;
    private double heading;
    private double pitch;
    private double speed;
    private double verticalSpeed;
    private int hp;
    private boolean forfeited;
    private double noiseLevel;
    private double sourceLevelDb;           // radiated noise in dB, computed by physics
    private double actualBallast = 0.5;     // physical ballast state (lags behind commanded)
    private double previousActualBallast = 0.5; // for tracking ballast change rate

    // sonar state
    private boolean pingRequested;
    private int activeSonarCooldown;

    // actuators (written by controller each tick)
    private double rudder;
    private double sternPlanes;
    private double throttle;
    private double ballast = 0.5;
    private boolean engineClutch = true;  // true = engaged
    private String status = "";

    // contact estimates (published by controller, cleared each tick)
    private final List<ContactEstimate> contactEstimates = new ArrayList<>();

    // navigation waypoints (published by controller, cleared each tick)
    private final List<Waypoint> waypoints = new ArrayList<>();

    public SubmarineEntity(VehicleConfig vehicleConfig, int id, SubmarineController controller,
                           Vec3 spawn, double heading, Color color, int maxHp) {
        this.vehicleConfig = vehicleConfig;
        this.id = id;
        this.controller = controller;
        this.color = color;
        this.maxHp = maxHp;
        this.hp = maxHp;
        this.x = spawn.x();
        this.y = spawn.y();
        this.z = vehicleConfig.surfaceLocked() ? 0 : spawn.z();
        this.heading = heading;
    }

    public VehicleConfig vehicleConfig() { return vehicleConfig; }

    // ── SubmarineOutput ──

    @Override
    public void setRudder(double value) {
        this.rudder = Math.clamp(value, -1, 1);
    }

    @Override
    public void setSternPlanes(double value) {
        this.sternPlanes = Math.clamp(value, -1, 1);
    }

    @Override
    public void setThrottle(double value) {
        this.throttle = Math.clamp(value, -1, 1);
    }

    @Override
    public void setBallast(double value) {
        this.ballast = Math.clamp(value, 0, 1);
    }

    @Override
    public void activeSonarPing() {
        if (activeSonarCooldown <= 0) {
            this.pingRequested = true;
        }
    }

    @Override
    public void setStatus(String status) {
        this.status = status != null && status.length() > 40 ? status.substring(0, 40) : (status != null ? status : "");
    }

    @Override
    public void publishContactEstimate(ContactEstimate estimate) {
        if (estimate != null) {
            contactEstimates.add(estimate);
        }
    }

    @Override
    public void publishWaypoint(Waypoint waypoint) {
        if (waypoint != null) {
            waypoints.add(waypoint);
        }
    }

    @Override
    public void setEngineClutch(boolean engaged) {
        this.engineClutch = engaged;
    }

    public boolean engineClutch() { return engineClutch; }

    // ── accessors ──

    public int id() { return id; }
    public SubmarineController controller() { return controller; }
    public Color color() { return color; }

    public double x() { return x; }
    public double y() { return y; }
    public double z() { return z; }
    public double heading() { return heading; }
    public double pitch() { return pitch; }
    public double speed() { return speed; }
    public double verticalSpeed() { return verticalSpeed; }
    public int hp() { return hp; }
    public boolean forfeited() { return forfeited; }
    public double noiseLevel() { return noiseLevel; }
    public double sourceLevelDb() { return sourceLevelDb; }
    public double actualBallast() { return actualBallast; }
    public double previousActualBallast() { return previousActualBallast; }

    public boolean pingRequested() { return pingRequested; }
    public int activeSonarCooldown() { return activeSonarCooldown; }

    public double rudder() { return rudder; }
    public double sternPlanes() { return sternPlanes; }
    public double throttle() { return throttle; }
    public double ballast() { return ballast; }
    public String status() { return status; }
    public List<ContactEstimate> contactEstimates() { return List.copyOf(contactEstimates); }
    public void clearContactEstimates() { contactEstimates.clear(); }
    public List<Waypoint> waypoints() { return List.copyOf(waypoints); }
    public void clearWaypoints() { waypoints.clear(); }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }
    public void setZ(double z) { this.z = z; }
    public void setHeading(double heading) { this.heading = heading; }
    public void setPitch(double pitch) { this.pitch = pitch; }
    public void setSpeed(double speed) { this.speed = speed; }
    public void setVerticalSpeed(double vs) { this.verticalSpeed = vs; }
    public void setHp(int hp) { this.hp = hp; }
    public void setForfeited(boolean f) { this.forfeited = f; }
    public void setNoiseLevel(double n) { this.noiseLevel = n; }
    public void setSourceLevelDb(double db) { this.sourceLevelDb = db; }
    public void setActualBallast(double b) { this.actualBallast = b; }
    public void setPreviousActualBallast(double b) { this.previousActualBallast = b; }
    public void setPingRequested(boolean p) { this.pingRequested = p; }
    public void setActiveSonarCooldown(int ticks) { this.activeSonarCooldown = ticks; }

    public Pose pose() {
        return new Pose(new Vec3(x, y, z), heading, pitch, 0);
    }

    public Velocity velocity() {
        double vx = speed * Math.sin(heading) * Math.cos(pitch);
        double vy = speed * Math.cos(heading) * Math.cos(pitch);
        double vz = speed * Math.sin(pitch) + verticalSpeed;
        return new Velocity(new Vec3(vx, vy, vz), new Vec3(0, pitch, 0));
    }

    public SubmarineSnapshot snapshot() {
        return new SubmarineSnapshot(id, controller.name(), pose(), velocity(), speed, color,
                forfeited, hp, noiseLevel, throttle, status, pingRequested,
                contactEstimates(), waypoints());
    }

    public SubmarineState state() {
        return new SubmarineState(pose(), velocity(), hp, 0);
    }
}
