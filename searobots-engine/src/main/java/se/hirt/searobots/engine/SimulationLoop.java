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
import java.util.Objects;

public final class SimulationLoop {

    private static final Color[] SUB_COLORS = {
            new Color(60, 220, 120), new Color(220, 80, 80),
            new Color(80, 140, 255), new Color(255, 180, 40),
            new Color(200, 80, 220), new Color(80, 220, 220)
    };

    public enum State { CREATED, INITIALIZING, RUNNING, PAUSED, STOPPED }

    private volatile double speedMultiplier = 1.0;
    private volatile boolean paused;
    private volatile boolean stepOnce;
    private volatile boolean stopped;
    private volatile State state = State.CREATED;

    private final List<SubmarineEntity> entities = new ArrayList<>();

    public void run(GeneratedWorld world, List<SubmarineController> controllers,
                    List<VehicleConfig> vehicleConfigs, SimulationListener listener) {
        run(world, controllers, vehicleConfigs, null, listener);
    }

    /**
     * Run the simulation with optional per-entity headings.
     * @param headings optional list of initial headings in radians, or null
     *                 to use the default (face toward center). Individual
     *                 entries may be Double.NaN to use the default for that entity.
     */
    public void run(GeneratedWorld world, List<SubmarineController> controllers,
                    List<VehicleConfig> vehicleConfigs, List<Double> headings,
                    SimulationListener listener) {
        Objects.requireNonNull(vehicleConfigs, "vehicleConfigs must not be null");
        state = State.INITIALIZING;
        var config = world.config();
        var physics = new SubmarinePhysics();
        var sonar = new SonarModel(config.worldSeed(), config.maxSubSpeed());
        double dt = 1.0 / config.tickRateHz();

        var envSnapshot = new EnvironmentSnapshot(
                world.terrain(), world.thermalLayers(), world.currentField());
        var matchContext = new MatchContext(
                config, world.terrain(), world.thermalLayers(), world.currentField());

        // Create entities at spawn points
        entities.clear();
        var spawns = world.spawnPoints();
        for (int i = 0; i < controllers.size(); i++) {
            var spawn = i < spawns.size() ? spawns.get(i) : spawns.getFirst();
            double heading;
            if (headings != null && i < headings.size() && !Double.isNaN(headings.get(i))) {
                heading = headings.get(i);
            } else {
                // Pick a heading that doesn't point into shallow water
                double safeHeading = WorldGenerator.findSafeHeading(
                        world.terrain(), spawn.x(), spawn.y());
                if (!Double.isNaN(safeHeading)) {
                    heading = safeHeading;
                } else {
                    heading = Math.atan2(-spawn.x(), -spawn.y()); // fallback: face center
                    if (heading < 0) heading += 2 * Math.PI;
                }
            }
            var vCfg = i < vehicleConfigs.size() ? vehicleConfigs.get(i) : VehicleConfig.submarine();
            var entity = new SubmarineEntity(vCfg, i, controllers.get(i), spawn, heading,
                    SUB_COLORS[i % SUB_COLORS.length], config.startingHp());
            entities.add(entity);
        }

        // Main loop (try-finally ensures onMatchEnd always fires,
        // even if the thread is interrupted during sleep)
        boolean matchStarted = false;
        try {
            for (long tick = 0; tick < config.matchDurationTicks() && !stopped; tick++) {
                // Handle pause (sim may start paused to let viewers register)
                if (paused) state = State.PAUSED;
                while (paused && !stepOnce && !stopped) {
                    try { Thread.sleep(50); } catch (InterruptedException ex) { break; }
                }
                if (stopped) break;
                stepOnce = false;
                state = State.RUNNING;

                // Notify controllers on first active tick, not during setup pause.
                // This prevents controllers from gaining extra processing time
                // while viewers are initializing.
                if (!matchStarted) {
                    for (var e : entities) {
                        e.controller().onMatchStart(matchContext);
                    }
                    matchStarted = true;
                }

                final long currentTick = tick;

                // Compute sonar contacts (based on current positions, before controller runs)
                var sonarResults = sonar.computeContacts(
                        currentTick, entities, world.terrain(), world.thermalLayers());

                // Build input, call controller, advance physics
                for (var entity : entities) {
                    if (entity.forfeited() || entity.hp() <= 0) continue;

                    var sr = sonarResults.getOrDefault(entity.id(),
                            new SonarModel.SonarResult(List.of(), List.of(), 0));
                    var input = new SubmarineInput() {
                        @Override public long tick() { return currentTick; }
                        @Override public double deltaTimeSeconds() { return dt; }
                        @Override public SubmarineState self() { return entity.state(); }
                        @Override public EnvironmentSnapshot environment() { return envSnapshot; }
                        @Override public List<SonarContact> sonarContacts() { return sr.passiveContacts(); }
                        @Override public List<SonarContact> activeSonarReturns() { return sr.activeReturns(); }
                        @Override public int activeSonarCooldownTicks() { return sr.cooldownTicks(); }
                    };

                    entity.controller().onTick(input, entity);
                    physics.step(entity, dt, world.terrain(), world.currentField(),
                            config.battleArea());
                }

                // Sub-to-sub collision (ramming)
                checkSubCollisions(entities);

                // Snapshot before postTick clears pingRequested
                var snapshots = entities.stream().map(SubmarineEntity::snapshot).toList();

                // Clear per-tick published data after snapshot
                entities.forEach(SubmarineEntity::clearContactEstimates);
                entities.forEach(SubmarineEntity::clearWaypoints);
                entities.forEach(SubmarineEntity::clearStrategicWaypoints);
                entities.forEach(SubmarineEntity::clearFiringSolution);

                // Post-tick: consume pings, tick cooldowns
                sonar.postTick(entities);

                // Notify listener
                listener.onTick(tick, snapshots);

                // Timing
                long sleepMs = (long) (1000.0 / config.tickRateHz() / speedMultiplier);
                if (sleepMs > 0 && !paused) {
                    try { Thread.sleep(sleepMs); } catch (InterruptedException ex) { break; }
                }
            }
        } finally {
            state = State.STOPPED;
            // Match end: always called regardless of how the loop exits
            for (var e : entities) {
                e.controller().onMatchEnd(new MatchResult(e.forfeited()));
            }
            listener.onMatchEnd();
        }
    }

    private static final double COLLISION_DAMAGE_FACTOR = 5.0;

    static void checkSubCollisions(List<SubmarineEntity> entities) {
        for (int i = 0; i < entities.size(); i++) {
            var a = entities.get(i);
            if (a.forfeited() || a.hp() <= 0) continue;
            for (int j = i + 1; j < entities.size(); j++) {
                var b = entities.get(j);
                if (b.forfeited() || b.hp() <= 0) continue;

                if (!ellipsoidsOverlap(a, b)) continue;

                var posA = new Vec3(a.x(), a.y(), a.z());
                var posB = new Vec3(b.x(), b.y(), b.z());
                double dist = posA.distanceTo(posB);

                // Collision line: from A to B
                Vec3 line = (dist > 0.01)
                        ? posB.subtract(posA).normalize()
                        : new Vec3(1, 0, 0); // degenerate: pick arbitrary

                Vec3 velA = a.velocity().linear();
                Vec3 velB = b.velocity().linear();
                double vAlong = velA.dot(line);
                double vBlong = velB.dot(line);
                double closingSpeed = vAlong - vBlong;

                if (closingSpeed > 0) {
                    int damage = Math.max(1,
                            (int) (COLLISION_DAMAGE_FACTOR * closingSpeed * closingSpeed));
                    a.setHp(Math.max(0, a.hp() - damage));
                    b.setHp(Math.max(0, b.hp() - damage));

                    // Bounce apart: push each sub along collision line
                    double halfSep = dist > 0.01 ? 1.0 : 0.5;
                    a.setX(a.x() - line.x() * halfSep);
                    a.setY(a.y() - line.y() * halfSep);
                    a.setZ(a.z() - line.z() * halfSep);
                    b.setX(b.x() + line.x() * halfSep);
                    b.setY(b.y() + line.y() * halfSep);
                    b.setZ(b.z() + line.z() * halfSep);

                    // Reduce speeds
                    a.setSpeed(a.speed() * 0.3);
                    b.setSpeed(b.speed() * 0.3);
                }
            }
        }
    }

    /**
     * Check whether two oriented ellipsoids overlap by testing sample points
     * (center, bow, stern) of each sub against the other's ellipsoid.
     * No Minkowski expansion needed: direct point-in-ellipsoid checks.
     */
    // Collision ellipsoid tuned to match visual (B key debug overlay):
    // semiLength=38m, semiBeam=5.5m, semiHeight=4.5m, center 2m aft
    private static final double COLLISION_SEMI_LENGTH = 38.0;
    private static final double COLLISION_SEMI_BEAM = 5.5;
    private static final double COLLISION_SEMI_HEIGHT = 4.5;
    private static final double COLLISION_AFT_OFFSET = -2.0;

    static boolean ellipsoidsOverlap(SubmarineEntity a, SubmarineEntity b) {
        // Sample 3 points on each sub: bow tip, center, stern tip
        double[][] pointsA = hullSamplePoints(a);
        double[][] pointsB = hullSamplePoints(b);

        // Check if any point on A is inside B's ellipsoid
        for (var pt : pointsA) {
            if (pointInEllipsoid(pt, b)) return true;
        }
        // Check if any point on B is inside A's ellipsoid
        for (var pt : pointsB) {
            if (pointInEllipsoid(pt, a)) return true;
        }
        return false;
    }

    /** Returns 3 sample points on the sub's hull: center, bow tip, stern tip. */
    private static double[][] hullSamplePoints(SubmarineEntity sub) {
        double cosH = Math.cos(sub.heading()), sinH = Math.sin(sub.heading());
        double cosP = Math.cos(sub.pitch()), sinP = Math.sin(sub.pitch());
        double fwdX = sinH * cosP, fwdY = cosH * cosP, fwdZ = sinP;

        // Center (with aft offset)
        double cx = sub.x() + fwdX * COLLISION_AFT_OFFSET;
        double cy = sub.y() + fwdY * COLLISION_AFT_OFFSET;
        double cz = sub.z() + fwdZ * COLLISION_AFT_OFFSET;

        return new double[][] {
            {cx, cy, cz},                                                                     // center
            {cx + fwdX * COLLISION_SEMI_LENGTH, cy + fwdY * COLLISION_SEMI_LENGTH, cz + fwdZ * COLLISION_SEMI_LENGTH},  // bow
            {cx - fwdX * COLLISION_SEMI_LENGTH, cy - fwdY * COLLISION_SEMI_LENGTH, cz - fwdZ * COLLISION_SEMI_LENGTH},  // stern
        };
    }

    /** Check if a world-space point lies inside a sub's collision ellipsoid. */
    private static boolean pointInEllipsoid(double[] pt, SubmarineEntity sub) {
        double cosH = Math.cos(sub.heading()), sinH = Math.sin(sub.heading());
        double cosP = Math.cos(sub.pitch()), sinP = Math.sin(sub.pitch());

        double fwdX = sinH * cosP, fwdY = cosP * cosH, fwdZ = sinP;
        double rightX = cosH, rightY = -sinH, rightZ = 0;
        double upX = -sinH * sinP, upY = -cosH * sinP, upZ = cosP;

        // Ellipsoid center (with aft offset)
        double cx = sub.x() + fwdX * COLLISION_AFT_OFFSET;
        double cy = sub.y() + fwdY * COLLISION_AFT_OFFSET;
        double cz = sub.z() + fwdZ * COLLISION_AFT_OFFSET;

        double dx = pt[0] - cx, dy = pt[1] - cy, dz = pt[2] - cz;

        double localFwd = dx * fwdX + dy * fwdY + dz * fwdZ;
        double localRight = dx * rightX + dy * rightY + dz * rightZ;
        double localUp = dx * upX + dy * upY + dz * upZ;

        double norm = (localFwd * localFwd) / (COLLISION_SEMI_LENGTH * COLLISION_SEMI_LENGTH)
                + (localRight * localRight) / (COLLISION_SEMI_BEAM * COLLISION_SEMI_BEAM)
                + (localUp * localUp) / (COLLISION_SEMI_HEIGHT * COLLISION_SEMI_HEIGHT);
        return norm <= 1.0;
    }

    public double getSpeedMultiplier() { return speedMultiplier; }
    public void setSpeedMultiplier(double m) { this.speedMultiplier = m; }
    public State getState() { return state; }
    public void setPaused(boolean p) { this.paused = p; }
    public boolean isPaused() { return paused; }
    public void stepOnce() { this.stepOnce = true; }
    public void stop() { this.stopped = true; }
    public boolean isStopped() { return stopped; }
}
