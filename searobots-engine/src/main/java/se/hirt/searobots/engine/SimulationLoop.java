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

import java.awt.*;
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
    private final List<TorpedoEntity> torpedoes = new ArrayList<>();
    private final List<TorpedoEntity> pendingLaunches = new ArrayList<>();
    private int nextTorpedoId = 1000; // torpedo IDs start at 1000 to avoid conflict with sub IDs
    private final TorpedoPhysics torpedoPhysics = new TorpedoPhysics();

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
        torpedoes.clear();
        pendingLaunches.clear();
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
            entity.setTorpedoesRemaining(config.torpedoCount());
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
                        currentTick, entities, torpedoes, world.terrain(), world.thermalLayers());

                // Build input, call controller, advance physics
                for (var entity : entities) {
                    if (entity.forfeited()) continue;

                    if (entity.hp() <= 0) {
                        // Dead sub: no controller, but physics still runs for sinking.
                        // Disengage clutch (prop freewheels), flood ballast (sink).
                        entity.setThrottle(0);
                        entity.setEngineClutch(false);
                        entity.setBallast(0); // full flood = maximum negative buoyancy
                        entity.setRudder(0);
                        entity.setSternPlanes(0);
                        physics.step(entity, dt, world.terrain(), world.currentField(),
                                config.battleArea());
                        continue;
                    }

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

                // Process pending torpedo launches
                for (var entity : entities) {
                    var cmd = entity.pendingTorpedoLaunch();
                    if (cmd != null) {
                        entity.clearPendingTorpedoLaunch();
                        // Launch in the submarine's current heading direction,
                        // at the sub's speed + ejection boost.
                        // Torpedo exits along the sub's heading (tubes are fixed forward)
                        // Alternate port/starboard tubes (offset ~2m from centerline)
                        double launchHdg = entity.heading();
                        double launchPitch = entity.pitch();
                        double ejectionSpeed = 3.0; // m/s additional from tube ejection
                        // Use the same local frame as the collision system
                        double sinH = Math.sin(launchHdg);
                        double cosH = Math.cos(launchHdg);
                        double sinP = Math.sin(launchPitch);
                        double cosP = Math.cos(launchPitch);
                        // Forward: bow direction (same as terrain collision points)
                        double fwdX = sinH * cosP, fwdY = cosH * cosP, fwdZ = sinP;
                        // Right: perpendicular in horizontal plane
                        double rightX = cosH, rightY = -sinH;

                        // Launch from the bow collision point, pulled back one torpedo length
                        double bowDist = 33.5; // same as SubmarinePhysics
                        double torpBack = 6.0; // pull back from bow tip
                        double tubeLateral = 2.0; // port/starboard offset
                        boolean portTube = (nextTorpedoId % 2 == 0);
                        double latSign = portTube ? -1.0 : 1.0;

                        double spawnFwd = bowDist - torpBack;
                        var launchPos = new Vec3(
                                entity.x() + fwdX * spawnFwd + rightX * latSign * tubeLateral,
                                entity.y() + fwdY * spawnFwd + rightY * latSign * tubeLateral,
                                entity.z() + fwdZ * spawnFwd - 1.5); // below hull centerline
                        double fuseR = Math.clamp(cmd.fuseRadius(),
                                config.minFuseRadius(), config.maxFuseRadius());

                        var customCtrl = entity.controller().createTorpedoController();
                        var torpCtrl = customCtrl != null ? customCtrl
                                : new se.hirt.searobots.engine.ships.SimpleTorpedoController();
                        var torp = new TorpedoEntity(nextTorpedoId++, entity.id(),
                                VehicleConfig.torpedo(), torpCtrl, launchPos,
                                launchHdg, launchPitch, fuseR, entity.color());
                        // Initial speed = sub speed + ejection
                        torp.setSpeed(Math.max(entity.speed(), 0) + ejectionSpeed);
                        torp.setLaunchTick(tick);

                        var launchCtx = new TorpedoLaunchContext(
                                config, world.terrain(), launchPos,
                                launchHdg, launchPitch,
                                cmd.missionData() != null ? cmd.missionData() : "");
                        torpCtrl.onLaunch(launchCtx);
                        pendingLaunches.add(torp);
                    }
                    entity.decrementLaunchTransient();
                }
                torpedoes.addAll(pendingLaunches);
                pendingLaunches.clear();

                // Torpedo controller + physics
                for (var torp : torpedoes) {
                    if (!torp.alive()) continue;

                    // Manual detonation request
                    if (torp.detonateRequested()) {
                        handleDetonation(torp, entities, config);
                        continue;
                    }

                    // Build torpedo input with sonar contacts
                    var torpOutput = torp.createOutput();
                    final long torpTick = tick;
                    var torpSonar = sonarResults.getOrDefault(torp.id(),
                            new SonarModel.SonarResult(List.of(), List.of(), 0));
                    var torpInput = new TorpedoInput() {
                        @Override public long tick() { return torpTick; }
                        @Override public double deltaTimeSeconds() { return dt; }
                        @Override public Pose self() { return torp.pose(); }
                        @Override public Velocity velocity() { return torp.velocity(); }
                        @Override public double speed() { return torp.speed(); }
                        @Override public double fuelRemaining() { return torp.fuelRemaining(); }
                        @Override public List<SonarContact> sonarContacts() { return torpSonar.passiveContacts(); }
                        @Override public List<SonarContact> activeSonarReturns() { return torpSonar.activeReturns(); }
                        @Override public int activeSonarCooldownTicks() { return torpSonar.cooldownTicks(); }
                    };
                    torp.controller().onTick(torpInput, torpOutput);

                    // Check detonation request from controller
                    if (torp.detonateRequested()) {
                        handleDetonation(torp, entities, config);
                        continue;
                    }

                    // Torpedo physics
                    torpedoPhysics.step(torp, dt, world.terrain(), world.currentField(),
                            config.battleArea());
                }

                // Proximity fuse check against sub ellipsoid (only after arming delay)
                for (var torp : torpedoes) {
                    if (!torp.alive()) continue;
                    torp.decrementArmingDelay();
                    if (!torp.fuseArmed()) continue;
                    for (var sub : entities) {
                        if (sub.hp() <= 0 || sub.forfeited()) continue;
                        // Skip owner during first 5 seconds (250 ticks) to let torpedo clear
                        if (sub.id() == torp.ownerId() && tick - torp.launchTick() < 250) continue;
                        // Use torpedo bow (nose) point: warhead is in the front
                        double hullDist = HullGeometry.bowDistanceToHull(
                                torp.x(), torp.y(), torp.z(),
                                torp.heading(), torp.pitch(),
                                torp.vehicleConfig().hullHalfLength(),
                                sub.x(), sub.y(), sub.z(),
                                sub.heading(), sub.pitch());
                        if (hullDist < torp.fuseRadius()) {
                            System.out.printf("[Torpedo %d] PROXIMITY FUSE at tick %d, hull dist=%.1fm to sub %d (%s)%n",
                                    torp.id(), tick, hullDist, sub.id(), sub.controller().name());
                            handleDetonation(torp, entities, config);
                            break;
                        }
                    }
                }

                // Handle detonations that weren't from proximity fuse
                // (terrain impact, manual detonation, stall)
                for (var t : torpedoes) {
                    if (t.detonated() && !t.explosionProcessed()) {
                        handleDetonation(t, entities, config);
                    }
                }

                // Sub-to-sub collision (ramming)
                checkSubCollisions(entities);

                // Snapshots BEFORE removing dead torpedoes, so the viewer sees
                // detonated torpedoes for one tick and can create explosion effects.
                var snapshots = entities.stream().map(SubmarineEntity::snapshot).toList();
                var torpSnapshots = torpedoes.stream().map(TorpedoEntity::snapshot).toList();

                // Clear per-tick published data after snapshot
                entities.forEach(SubmarineEntity::clearContactEstimates);
                entities.forEach(SubmarineEntity::clearWaypoints);
                entities.forEach(SubmarineEntity::clearStrategicWaypoints);
                entities.forEach(SubmarineEntity::clearFiringSolution);
                // Note: torpedo pings are NOT cleared here. They're cleared by the sonar
                // model on the next tick's computeContacts(), same as submarine pings.

                // Post-tick: consume pings, tick cooldowns
                sonar.postTick(entities);
                torpedoes.forEach(TorpedoEntity::decrementSonarCooldown);

                // Notify listener
                listener.onTick(tick, snapshots, torpSnapshots);

                // Remove dead torpedoes (after snapshot and listener notification)
                final long logTick = tick;
                torpedoes.removeIf(t -> {
                    if (t.alive()) return false;
                    String fate = t.detonated() ? "DETONATED" : "LOST";
                    if (!t.detonated()) {
                        if (t.speed() < TorpedoEntity.minimumSpeed()) fate = "STALLED (sank)";
                        else if (t.fuelRemaining() <= 0 && t.speed() < 1) fate = "FUEL DEPLETED (sank)";
                        else fate = "TERRAIN/BOUNDARY";
                    }
                    System.out.printf("[Torpedo %d] %s at tick %d  pos=(%.0f,%.0f,%.0f) speed=%.1f fuel=%.1f%n",
                            t.id(), fate, logTick, t.x(), t.y(), t.z(), t.speed(), t.fuelRemaining());
                    return true;
                });

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

    /** Distance from a torpedo to the nearest point on a sub's hull ellipsoid. */
    private static double distanceToHullSurface(TorpedoEntity torp, SubmarineEntity sub) {
        return HullGeometry.distanceToHull(torp, sub);
    }

    // ── Torpedo explosion ────────────────────────────────────────────

    private static final int EXPLOSION_BASE_DAMAGE = 1200; // instant kill at point blank, heavy damage at close range
    private static final double EXPLOSION_IMPULSE = 15.0; // m/s velocity change at zero range (before mass division)

    private static void handleDetonation(TorpedoEntity torp, List<SubmarineEntity> subs,
                                          MatchConfig config) {
        torp.detonate();
        torp.setExplosionProcessed();
        double blastRadius = config.blastRadius();

        // Blast center is at the torpedo's bow (warhead in the nose)
        double cosP = Math.cos(torp.pitch()), sinP = Math.sin(torp.pitch());
        double halfLen = torp.vehicleConfig().hullHalfLength();
        double blastX = torp.x() + Math.sin(torp.heading()) * cosP * halfLen;
        double blastY = torp.y() + Math.cos(torp.heading()) * cosP * halfLen;
        double blastZ = torp.z() + sinP * halfLen;

        for (var sub : subs) {
            if (sub.hp() <= 0 || sub.forfeited()) continue;
            double dx = sub.x() - blastX;
            double dy = sub.y() - blastY;
            double dz = sub.z() - blastZ;
            double centerDist = Math.sqrt(dx * dx + dy * dy + dz * dz);

            // Use hull surface distance from blast point for damage
            double hullDist = HullGeometry.distanceToHull(
                    blastX, blastY, blastZ,
                    sub.x(), sub.y(), sub.z(), sub.heading(), sub.pitch());
            if (hullDist < blastRadius) {
                // Quadratic falloff based on distance to hull surface
                double falloff = (1.0 - hullDist / blastRadius);
                falloff *= falloff;

                // Damage
                int damage = Math.max(1, (int) (EXPLOSION_BASE_DAMAGE * falloff));
                sub.setHp(Math.max(0, sub.hp() - damage));

                // Impulse: push sub away from blast. Divide by mass so
                // heavier subs resist more. A submarine is thousands of tons,
                // so the velocity change is modest (shove, not launch).
                if (centerDist > 0.1) {
                    double force = EXPLOSION_IMPULSE * falloff;
                    double mass = sub.vehicleConfig().massSurge();
                    double dv = force / (mass * 0.001); // scale: mass is in kg, want moderate push
                    double nx = dx / centerDist, ny = dy / centerDist, nz = dz / centerDist;

                    // Linear impulse: velocity change along sub heading
                    double along = nx * Math.sin(sub.heading()) + ny * Math.cos(sub.heading());
                    sub.setSpeed(sub.speed() + dv * along);
                    sub.setVerticalSpeed(sub.verticalSpeed() + dv * nz * 0.5);

                    // Angular impulse: very small. A torpedo blast nudges a
                    // submarine's heading, it doesn't spin it around.
                    double fwdX = Math.sin(sub.heading());
                    double fwdY = Math.cos(sub.heading());
                    double crossZ = nx * fwdY - ny * fwdX;
                    double rotInertia = mass * sub.vehicleConfig().rotationalInertia();
                    sub.setYawRate(sub.yawRate() + crossZ * force / rotInertia * 0.5);
                    sub.setPitchRate(sub.pitchRate() + nz * force / rotInertia * 0.2);
                }
            }
        }
    }

    // ── Sub-to-sub collision ───────────────────────────────────────

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
        double cx = sub.x() + fwdX * HullGeometry.AFT_OFFSET;
        double cy = sub.y() + fwdY * HullGeometry.AFT_OFFSET;
        double cz = sub.z() + fwdZ * HullGeometry.AFT_OFFSET;

        return new double[][] {
            {cx, cy, cz},                                                                     // center
            {cx + fwdX * HullGeometry.SEMI_LENGTH, cy + fwdY * HullGeometry.SEMI_LENGTH, cz + fwdZ * HullGeometry.SEMI_LENGTH},  // bow
            {cx - fwdX * HullGeometry.SEMI_LENGTH, cy - fwdY * HullGeometry.SEMI_LENGTH, cz - fwdZ * HullGeometry.SEMI_LENGTH},  // stern
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
        double cx = sub.x() + fwdX * HullGeometry.AFT_OFFSET;
        double cy = sub.y() + fwdY * HullGeometry.AFT_OFFSET;
        double cz = sub.z() + fwdZ * HullGeometry.AFT_OFFSET;

        double dx = pt[0] - cx, dy = pt[1] - cy, dz = pt[2] - cz;

        double localFwd = dx * fwdX + dy * fwdY + dz * fwdZ;
        double localRight = dx * rightX + dy * rightY + dz * rightZ;
        double localUp = dx * upX + dy * upY + dz * upZ;

        double norm = (localFwd * localFwd) / (HullGeometry.SEMI_LENGTH * HullGeometry.SEMI_LENGTH)
                + (localRight * localRight) / (HullGeometry.SEMI_BEAM * HullGeometry.SEMI_BEAM)
                + (localUp * localUp) / (HullGeometry.SEMI_HEIGHT * HullGeometry.SEMI_HEIGHT);
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
