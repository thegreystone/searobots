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

import java.util.*;

public final class SonarModel {

    // Detection
    static final double DETECTION_THRESHOLD_DB = 5.0;
    static final double AMBIENT_NOISE_DB = 60.0;
    // Self-noise is lower than radiated SL (hydrophones are isolated from machinery)
    static final double SELF_NOISE_OFFSET_DB = 30.0;
    // Cylindrical spreading for shallow water at our scale (surface + floor waveguide)
    static final double SPREADING_COEFFICIENT = 10.0;

    // Baffles: ~100° blind cone behind sub (>130° from bow on each side)
    static final double BAFFLE_HALF_ARC = Math.toRadians(130);
    static final double BAFFLE_PENALTY_DB = 20.0;

    // Active sonar
    static final double ACTIVE_PING_SL_DB = 220.0;
    static final int ACTIVE_PING_COOLDOWN_TICKS = 250;
    static final double TARGET_STRENGTH_DB = 20.0;
    static final double RANGE_NOISE_FRACTION = 0.02; // 2% RMS range noise on active returns

    // Terrain occlusion: per-cell penalties (samples step at half-cell, so halved per sample)
    // Underwater ridge: up to 15 dB per cell of terrain above the sound path
    static final double RIDGE_OCCLUSION_PER_CELL_DB = 15.0;
    static final double RIDGE_FULL_EXCESS = 50.0;  // meters above path for max per-cell penalty
    // Island (terrain above sea level): rock is essentially opaque to sound
    static final double ISLAND_OCCLUSION_PER_CELL_DB = 30.0;

    private final Random rng;

    public SonarModel(long seed) {
        this.rng = new Random(seed);
    }

    record SonarResult(List<SonarContact> passiveContacts,
                       List<SonarContact> activeReturns,
                       int cooldownTicks) {}

    /**
     * Compute all sonar contacts for each entity this tick.
     * Returns a map from entity ID to its sonar result.
     */
    public Map<Integer, SonarResult> computeContacts(
            List<SubmarineEntity> entities,
            TerrainMap terrain,
            List<ThermalLayer> thermalLayers) {

        var results = new HashMap<Integer, SonarResult>();

        for (var listener : entities) {
            if (listener.forfeited() || listener.hp() <= 0) {
                results.put(listener.id(), new SonarResult(List.of(), List.of(), listener.activeSonarCooldown()));
                continue;
            }

            var passive = new ArrayList<SonarContact>();
            var active = new ArrayList<SonarContact>();
            var listenerPos = new Vec3(listener.x(), listener.y(), listener.z());

            // Self-noise: radiated SL minus isolation offset (hydrophones don't hear all own noise)
            double selfNoiseDb = listener.sourceLevelDb() - SELF_NOISE_OFFSET_DB;
            double nlBase = Math.max(AMBIENT_NOISE_DB, selfNoiseDb);

            for (var source : entities) {
                if (source.id() == listener.id()) continue;
                if (source.forfeited() || source.hp() <= 0) continue;

                var sourcePos = new Vec3(source.x(), source.y(), source.z());
                double distance = listenerPos.distanceTo(sourcePos);
                if (distance < 1.0) distance = 1.0;

                // Bearing from listener to source (absolute, [0, 2pi))
                double trueBearing = Math.atan2(
                        sourcePos.x() - listenerPos.x(),
                        sourcePos.y() - listenerPos.y());
                if (trueBearing < 0) trueBearing += 2 * Math.PI;

                // Transmission loss
                double tl = transmissionLossDb(distance, listenerPos, sourcePos,
                        terrain, thermalLayers);

                // --- Passive detection ---
                double sl = source.sourceLevelDb();

                // If the source pinged this tick, their SL is the ping level
                // (everyone hears the ping as a passive contact)
                if (source.pingRequested()) {
                    sl = Math.max(sl, ACTIVE_PING_SL_DB);
                }

                // Noise level: base + baffle penalty
                double nl = nlBase;
                if (isInBaffles(listener.heading(), trueBearing)) {
                    nl += BAFFLE_PENALTY_DB;
                }

                double se = sl - tl - nl;
                if (se > DETECTION_THRESHOLD_DB) {
                    double brgStdDev = bearingStdDev(se);
                    double bearingError = rng.nextGaussian() * brgStdDev;
                    double reportedBearing = normalizeBearing(trueBearing + bearingError);
                    double estSpeed = estimateTargetSpeed(source.speed(), se, rng);
                    // Estimate source level from signal characteristics.
                    // Accuracy improves with SE (closer = better signal analysis).
                    double slError = Math.clamp(10.0 / Math.max(se, 1.0), 1.0, 8.0);
                    double estSL = sl + rng.nextGaussian() * slError;
                    passive.add(new SonarContact(reportedBearing, se, 0, false, estSpeed,
                            brgStdDev, 0, estSL));
                }

                // --- Active sonar returns (for the listener's own ping) ---
                if (listener.pingRequested() && listener.activeSonarCooldown() <= 0) {
                    // Round-trip: 2 * TL
                    double activeSe = ACTIVE_PING_SL_DB - 2 * tl + TARGET_STRENGTH_DB - nl;
                    if (activeSe > DETECTION_THRESHOLD_DB) {
                        double activeBrgStdDev = bearingStdDev(activeSe);
                        double bearingError = rng.nextGaussian() * activeBrgStdDev;
                        double reportedBearing = normalizeBearing(trueBearing + bearingError);
                        double rangeRmsNoise = distance * RANGE_NOISE_FRACTION;
                        double rangeNoise = rangeRmsNoise * rng.nextGaussian();
                        double reportedRange = Math.max(1.0, distance + rangeNoise);
                        double estSpeed = estimateTargetSpeed(source.speed(), se, rng);
                        double slError = Math.clamp(10.0 / Math.max(activeSe, 1.0), 1.0, 5.0);
                        double estSL = sl + rng.nextGaussian() * slError;
                        active.add(new SonarContact(reportedBearing, activeSe, reportedRange, true,
                                estSpeed, activeBrgStdDev, rangeRmsNoise, estSL));
                    }
                }
            }

            results.put(listener.id(), new SonarResult(
                    List.copyOf(passive), List.copyOf(active), listener.activeSonarCooldown()));
        }

        // Consume ping requests AFTER processing all entities, so that a
        // pinger's SL boost is visible to all listeners during this tick.
        for (var entity : entities) {
            if (entity.pingRequested()) {
                if (entity.activeSonarCooldown() <= 0) {
                    entity.setActiveSonarCooldown(ACTIVE_PING_COOLDOWN_TICKS);
                }
                entity.setPingRequested(false);
            }
        }

        return results;
    }

    /**
     * Post-tick: tick cooldowns only. Ping requests are NOT cleared here.
     * They persist until sonar's computeContacts processes them on the next
     * tick (sonar runs before the controller, so a ping requested on tick N
     * must be visible to sonar on tick N+1).
     */
    public void postTick(List<SubmarineEntity> entities) {
        for (var entity : entities) {
            if (entity.activeSonarCooldown() > 0) {
                entity.setActiveSonarCooldown(entity.activeSonarCooldown() - 1);
            }
        }
    }

    // ── Transmission loss ────────────────────────────────────────────

    static double transmissionLossDb(double distance, Vec3 src, Vec3 dst,
                                     TerrainMap terrain, List<ThermalLayer> layers) {
        double tl = SPREADING_COEFFICIENT * Math.log10(Math.max(distance, 1.0));
        tl += terrainOcclusionDb(src, dst, terrain);
        tl += thermoclineDb(src.z(), dst.z(), layers);
        return tl;
    }

    static double terrainOcclusionDb(Vec3 src, Vec3 dst, TerrainMap terrain) {
        double dx = dst.x() - src.x();
        double dy = dst.y() - src.y();
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 1.0) return 0;

        // Step at half-cell resolution so no terrain cell is skipped
        double step = terrain.getCellSize() * 0.5;
        int samples = Math.max(2, (int) Math.ceil(dist / step));
        // Each sample represents half a cell, so per-sample penalty is halved
        double ridgePerSample = RIDGE_OCCLUSION_PER_CELL_DB * 0.5;
        double islandPerSample = ISLAND_OCCLUSION_PER_CELL_DB * 0.5;

        double totalDb = 0;
        for (int i = 1; i < samples; i++) {
            double t = (double) i / samples;
            double floor = terrain.elevationAt(src.x() + dx * t, src.y() + dy * t);
            double lineDepth = src.z() + (dst.z() - src.z()) * t;
            if (floor > lineDepth) {
                double excess = floor - lineDepth;
                if (floor >= 0) {
                    totalDb += islandPerSample;
                } else {
                    totalDb += ridgePerSample * Math.min(excess / RIDGE_FULL_EXCESS, 1.0);
                }
            }
        }
        return totalDb;
    }

    static double thermoclineDb(double srcZ, double dstZ, List<ThermalLayer> layers) {
        double totalDb = 0;
        for (var layer : layers) {
            double boundary = layer.depth();
            boolean srcAbove = srcZ > boundary;
            boolean dstAbove = dstZ > boundary;
            if (srcAbove == dstAbove) {
                totalDb -= 2.0; // same-layer waveguide bonus
            } else {
                double tempDiff = Math.abs(layer.temperatureAbove() - layer.temperatureBelow());
                double penalty = Math.clamp(3.0 + 0.5 * tempDiff, 5.0, 10.0);
                totalDb += penalty;
            }
        }
        return totalDb;
    }

    // ── Baffles ──────────────────────────────────────────────────────

    static boolean isInBaffles(double listenerHeading, double bearingToSource) {
        double relative = bearingToSource - listenerHeading;
        // Normalize to [-pi, pi]
        relative = relative % (2 * Math.PI);
        if (relative > Math.PI) relative -= 2 * Math.PI;
        if (relative < -Math.PI) relative += 2 * Math.PI;
        // Baffle zone: bearing from bow > BAFFLE_HALF_ARC
        return Math.abs(relative) > BAFFLE_HALF_ARC;
    }

    // ── Bearing accuracy ─────────────────────────────────────────────

    static double bearingStdDev(double signalExcessDb) {
        return Math.toRadians(Math.clamp(30.0 / signalExcessDb, 0.3, 10.0));
    }

    // ── Blade-rate speed estimation ──────────────────────────────────

    // Minimum SE to attempt blade-rate analysis (below this, tonals are buried in noise)
    private static final double SPEED_EST_MIN_SE = 5.0;
    // At this SE, speed estimate error is ~±50%; at 30 dB SE, ~±5%
    private static final double SPEED_EST_BASE_ERROR = 0.50;
    private static final double SPEED_EST_GOOD_SE = 30.0;

    /**
     * Estimate target speed from blade-rate tonals.
     * Accuracy depends on signal excess: close range (high SE) → tight estimate,
     * long range (low SE) → noisy or unavailable.
     *
     * @return estimated speed in m/s, or -1 if SE too low for analysis
     */
    static double estimateTargetSpeed(double actualSpeed, double signalExcess, Random rng) {
        if (signalExcess < SPEED_EST_MIN_SE) return -1;

        // Error fraction: 50% at threshold, dropping to ~5% at 30 dB SE
        // Linear interpolation clamped to [0.05, 0.50]
        double t = Math.clamp((signalExcess - SPEED_EST_MIN_SE) / (SPEED_EST_GOOD_SE - SPEED_EST_MIN_SE), 0, 1);
        double errorFraction = SPEED_EST_BASE_ERROR * (1.0 - t * 0.9); // 0.50 → 0.05

        double noise = rng.nextGaussian() * errorFraction * Math.max(actualSpeed, 1.0);
        return Math.max(0, actualSpeed + noise);
    }

    private static double normalizeBearing(double bearing) {
        bearing = bearing % (2 * Math.PI);
        if (bearing < 0) bearing += 2 * Math.PI;
        return bearing;
    }
}
