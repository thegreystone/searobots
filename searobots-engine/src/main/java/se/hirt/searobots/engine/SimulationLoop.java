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

public final class SimulationLoop {

    private static final Color[] SUB_COLORS = {
            new Color(60, 220, 120), new Color(220, 80, 80),
            new Color(80, 140, 255), new Color(255, 180, 40),
            new Color(200, 80, 220), new Color(80, 220, 220)
    };

    private volatile double speedMultiplier = 1.0;
    private volatile boolean paused;
    private volatile boolean stepOnce;
    private volatile boolean stopped;

    private final List<SubmarineEntity> entities = new ArrayList<>();

    public void run(GeneratedWorld world, List<SubmarineController> controllers,
                    SimulationListener listener) {
        var config = world.config();
        var physics = new SubmarinePhysics();
        var sonar = new SonarModel(config.worldSeed());
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
            double heading = Math.atan2(-spawn.x(), -spawn.y()); // face toward center
            if (heading < 0) heading += 2 * Math.PI;
            var entity = new SubmarineEntity(i, controllers.get(i), spawn, heading,
                    SUB_COLORS[i % SUB_COLORS.length], config.startingHp());
            entities.add(entity);
        }

        // Notify match start
        for (var e : entities) {
            e.controller().onMatchStart(matchContext);
        }

        // Main loop
        for (long tick = 0; tick < config.matchDurationTicks() && !stopped; tick++) {
            // Handle pause
            while (paused && !stepOnce && !stopped) {
                try { Thread.sleep(50); } catch (InterruptedException ex) { return; }
            }
            stepOnce = false;

            final long currentTick = tick;

            // Compute sonar contacts (based on current positions, before controller runs)
            var sonarResults = sonar.computeContacts(
                    entities, world.terrain(), world.thermalLayers());

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

            // Post-tick: consume pings, tick cooldowns
            sonar.postTick(entities);

            // Notify listener
            listener.onTick(tick, snapshots);

            // Timing
            long sleepMs = (long) (1000.0 / config.tickRateHz() / speedMultiplier);
            if (sleepMs > 0 && !paused) {
                try { Thread.sleep(sleepMs); } catch (InterruptedException ex) { return; }
            }
        }

        // Match end
        for (var e : entities) {
            e.controller().onMatchEnd(new MatchResult(e.forfeited()));
        }
        listener.onMatchEnd();
    }

    // Collision radius: simplified as sphere with hull length as diameter
    private static final double COLLISION_RADIUS = 15.0;  // HULL_HALF_LENGTH
    private static final double COLLISION_DAMAGE_FACTOR = 5.0;
    private static final double COLLISION_BOUNCE_SPEED = 2.0;

    static void checkSubCollisions(List<SubmarineEntity> entities) {
        for (int i = 0; i < entities.size(); i++) {
            var a = entities.get(i);
            if (a.forfeited() || a.hp() <= 0) continue;
            for (int j = i + 1; j < entities.size(); j++) {
                var b = entities.get(j);
                if (b.forfeited() || b.hp() <= 0) continue;

                var posA = new Vec3(a.x(), a.y(), a.z());
                var posB = new Vec3(b.x(), b.y(), b.z());
                double dist = posA.distanceTo(posB);

                if (dist < COLLISION_RADIUS * 2) {
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
                        double separation = COLLISION_RADIUS * 2 - dist;
                        double halfSep = separation / 2.0 + 0.5;
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
    }

    public void setSpeedMultiplier(double m) { this.speedMultiplier = m; }
    public void setPaused(boolean p) { this.paused = p; }
    public boolean isPaused() { return paused; }
    public void stepOnce() { this.stepOnce = true; }
    public void stop() { this.stopped = true; }
    public boolean isStopped() { return stopped; }
}
