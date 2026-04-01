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

import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.ThermalLayer;
import se.hirt.searobots.api.Vec3;

import java.util.*;

public final class SonarModel {

    // Detection
    static final double DETECTION_THRESHOLD_DB = 5.0;
    static final double AMBIENT_NOISE_DB = 55.0;
    // Self-noise is lower than radiated SL (hydrophones are isolated from machinery)
    static final double SELF_NOISE_OFFSET_DB = 35.0;
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
    private final double maxSubSpeed;
    private final Map<Long, ContactTracker> trackers = new HashMap<>();

    public SonarModel(long seed) {
        this(seed, 15.0);
    }

    public SonarModel(long seed, double maxSubSpeed) {
        this.rng = new Random(seed);
        this.maxSubSpeed = maxSubSpeed;
    }

    record SonarResult(List<SonarContact> passiveContacts,
                       List<SonarContact> activeReturns,
                       int cooldownTicks) {}

    /**
     * Classify a sonar contact from its acoustic signature.
     * Requires sufficient signal excess to classify; faint contacts are UNKNOWN.
     *
     * Classification rules (modelling blade-rate tonal analysis):
     * - TORPEDO: high speed (>18 m/s) = small fast high-RPM prop
     * - SURFACE_SHIP: loud (SL>108) + moderate speed (<12) = big slow prop
     * - SUBMARINE: submerged, moderate noise, moderate speed
     * - UNKNOWN: insufficient signal excess to classify (<8 dB)
     */
    private static SonarContact.Classification classify(double signalExcess,
                                                         double estimatedSpeed,
                                                         double estimatedSL) {
        // Need enough signal to analyse the tonal structure
        if (signalExcess < 8) return SonarContact.Classification.UNKNOWN;

        // High blade rate + fast = torpedo (very distinctive signature)
        if (estimatedSpeed > 18) return SonarContact.Classification.TORPEDO;

        // Very loud + moderate speed = surface ship (big slow prop, machinery noise)
        if (estimatedSL > 108 && estimatedSpeed >= 0 && estimatedSpeed < 12) {
            return SonarContact.Classification.SURFACE_SHIP;
        }

        // Default: submarine
        return SonarContact.Classification.SUBMARINE;
    }

    /**
     * Compute all sonar contacts for each entity this tick.
     * Returns a map from entity ID to its sonar result.
     */
    /**
     * Compute sonar contacts including torpedoes as sources and listeners.
     */
    public Map<Integer, SonarResult> computeContacts(
            long tick,
            List<SubmarineEntity> entities,
            List<TorpedoEntity> torpedoes,
            TerrainMap terrain,
            List<ThermalLayer> thermalLayers) {

        // First: compute sub-to-sub contacts using the existing logic
        var results = computeSubToSubContacts(tick, entities, terrain, thermalLayers);

        // Second: torpedoes as additional noise sources for submarine listeners
        // (subs hear torpedoes via passive sonar)
        for (var listener : entities) {
            if (listener.forfeited() || listener.hp() <= 0) continue;
            var existing = results.get(listener.id());
            var extraPassive = new ArrayList<>(existing.passiveContacts());

            for (var torp : torpedoes) {
                if (!torp.alive()) continue;
                var contact = passiveDetect(listener.x(), listener.y(), listener.z(),
                        listener.heading(), listener.sourceLevelDb(),
                        listener.vehicleConfig().sonarSelfNoiseOffsetDb(),
                        torp.x(), torp.y(), torp.z(), torp.sourceLevelDb(),
                        torp.speed(), torp.pingRequested(), terrain, thermalLayers);
                if (contact != null) extraPassive.add(contact);
            }

            if (extraPassive.size() > existing.passiveContacts().size()) {
                results.put(listener.id(), new SonarResult(extraPassive, existing.activeReturns(), existing.cooldownTicks()));
            }
        }

        // Third: compute contacts FOR torpedo listeners (torpedoes hear subs)
        for (var torp : torpedoes) {
            if (!torp.alive()) continue;
            var passive = new ArrayList<SonarContact>();
            var active = new ArrayList<SonarContact>();

            // Torpedo passive: bow hydrophone has good isolation (62 dB offset)
            double torpSonarOffset = torp.vehicleConfig().sonarSelfNoiseOffsetDb();
            for (var source : entities) {
                if (source.forfeited() || source.hp() <= 0) continue;
                var contact = passiveDetect(torp.x(), torp.y(), torp.z(),
                        torp.heading(), torp.sourceLevelDb(), torpSonarOffset,
                        source.x(), source.y(), source.z(), source.sourceLevelDb(),
                        source.speed(), source.pingRequested(), terrain, thermalLayers);
                if (contact != null) passive.add(contact);
            }

            // Torpedo active sonar returns
            if (torp.pingRequested()) {
                for (var source : entities) {
                    if (source.forfeited() || source.hp() <= 0) continue;
                    var ret = activeDetect(torp.x(), torp.y(), torp.z(),
                            torp.sourceLevelDb(), torpSonarOffset,
                            source.x(), source.y(), source.z(), source.speed(),
                            terrain, thermalLayers);
                    if (ret != null) active.add(ret);
                }
                // Clear ping and set cooldown (same as submarine ping handling)
                torp.clearPingRequested();
                torp.setActiveSonarCooldown(ACTIVE_PING_COOLDOWN_TICKS);
            }

            results.put(torp.id(), new SonarResult(passive, active, torp.activeSonarCooldown()));
        }

        return results;
    }

    public Map<Integer, SonarResult> computeContacts(
            long tick,
            List<SubmarineEntity> entities,
            TerrainMap terrain,
            List<ThermalLayer> thermalLayers) {
        return computeContacts(tick, entities, List.of(), terrain, thermalLayers);
    }

    /** Passive detection: can listener hear source? Returns contact or null. */
    private SonarContact passiveDetect(double lx, double ly, double lz, double lHeading, double lSLdB,
                                        double lSonarOffset,
                                        double sx, double sy, double sz, double sSLdB,
                                        double sSpeed, boolean sPinged,
                                        TerrainMap terrain, List<ThermalLayer> thermalLayers) {
        double distance = Math.sqrt((sx-lx)*(sx-lx) + (sy-ly)*(sy-ly) + (sz-lz)*(sz-lz));
        if (distance < 1.0) distance = 1.0;

        double trueBearing = Math.atan2(sx - lx, sy - ly);
        if (trueBearing < 0) trueBearing += 2 * Math.PI;

        double tl = transmissionLossDb(distance, new Vec3(lx, ly, lz), new Vec3(sx, sy, sz),
                terrain, thermalLayers);
        double sl = sSLdB;
        if (sPinged) sl = Math.max(sl, ACTIVE_PING_SL_DB);

        double selfNoiseDb = lSLdB - lSonarOffset;
        double nl = Math.max(AMBIENT_NOISE_DB, selfNoiseDb);
        if (isInBaffles(lHeading, trueBearing)) nl += BAFFLE_PENALTY_DB;

        double se = sl - tl - nl;
        if (se > DETECTION_THRESHOLD_DB) {
            double bearingError = bearingStdDev(se);
            double noisyBearing = trueBearing + bearingError * rng.nextGaussian();
            noisyBearing = ((noisyBearing % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
            // Speed estimate from blade-rate analysis (noisy)
            double estSpd = sSpeed > 0 ? sSpeed + rng.nextGaussian() * 2 : -1;
            return new SonarContact(noisyBearing, se, 0, false, estSpd,
                    bearingError, 0, sl, 0, Double.NaN,
                    Double.NaN, classify(se, estSpd, sl));
        }
        return null;
    }

    /** Active sonar return: does the ping illuminate the target? Returns contact or null. */
    private SonarContact activeDetect(double lx, double ly, double lz, double lSLdB,
                                       double lSonarOffset,
                                       double sx, double sy, double sz, double sSpeed,
                                       TerrainMap terrain, List<ThermalLayer> thermalLayers) {
        double distance = Math.sqrt((sx-lx)*(sx-lx) + (sy-ly)*(sy-ly) + (sz-lz)*(sz-lz));
        if (distance < 1.0) distance = 1.0;

        double trueBearing = Math.atan2(sx - lx, sy - ly);
        if (trueBearing < 0) trueBearing += 2 * Math.PI;

        double tl = transmissionLossDb(distance, new Vec3(lx, ly, lz), new Vec3(sx, sy, sz),
                terrain, thermalLayers);

        double selfNoiseDb = lSLdB - lSonarOffset;
        double nl = Math.max(AMBIENT_NOISE_DB, selfNoiseDb);
        double activeSe = ACTIVE_PING_SL_DB - 2 * tl + TARGET_STRENGTH_DB - nl;

        if (activeSe > DETECTION_THRESHOLD_DB) {
            double bearingError = bearingStdDev(activeSe);
            double noisyBearing = trueBearing + bearingError * rng.nextGaussian();
            noisyBearing = ((noisyBearing % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
            double rangeRmsNoise = distance * RANGE_NOISE_FRACTION;
            double noisyRange = distance + rangeRmsNoise * rng.nextGaussian();
            if (noisyRange < 0) noisyRange = 10;
            // Estimate target depth from vertical angle + range
            // True depth = listener depth + vertical component of range
            double trueDepthDiff = sz - lz; // positive = target above listener
            double depthNoiseRms = Math.max(5.0, distance * 0.05); // 5% depth noise, min 5m
            double estimatedDepth = lz + trueDepthDiff + depthNoiseRms * rng.nextGaussian();

            double estSpd = sSpeed > 0 ? sSpeed + rng.nextGaussian() * 2 : -1;
            return new SonarContact(noisyBearing, activeSe, noisyRange, true, estSpd,
                    bearingError, rangeRmsNoise, ACTIVE_PING_SL_DB, 0, Double.NaN,
                    estimatedDepth, classify(activeSe, estSpd, ACTIVE_PING_SL_DB));
        }
        return null;
    }

    /**
     * Original submarine-to-submarine sonar computation (unchanged).
     */
    private Map<Integer, SonarResult> computeSubToSubContacts(
            long tick,
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
            double selfNoiseDb = listener.sourceLevelDb() - listener.vehicleConfig().sonarSelfNoiseOffsetDb();
            double nlBase = Math.max(AMBIENT_NOISE_DB, selfNoiseDb);

            // Track which source IDs were observed this tick (for decay)
            var observedSourceIds = new HashSet<Integer>();

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
                boolean inBaffles = isInBaffles(listener.heading(), trueBearing);
                double nl = nlBase;
                if (inBaffles) {
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

                    // Update contact tracker
                    var tracker = getOrCreateTracker(listener.id(), source.id());
                    tracker.update(tick, reportedBearing, se, estSpeed, estSL,
                            listener.x(), listener.y(), listener.heading(), inBaffles,
                            distance, source.x(), source.y(), rng);
                    observedSourceIds.add(source.id());

                    passive.add(new SonarContact(reportedBearing, se, tracker.estimatedRange(), false, estSpeed,
                            brgStdDev, tracker.rangeUncertainty(), estSL,
                            tracker.solutionQuality(), tracker.estimatedHeading(),
                            Double.NaN, classify(se, estSpeed, estSL)));
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
                        double estSpeed = estimateTargetSpeed(source.speed(), activeSe, rng);
                        double slError = Math.clamp(10.0 / Math.max(activeSe, 1.0), 1.0, 5.0);
                        double estSL = sl + rng.nextGaussian() * slError;
                        double depthNoiseRms = Math.max(5.0, distance * 0.05);
                        double estimatedDepth = source.z() + depthNoiseRms * rng.nextGaussian();

                        // Update contact tracker from ping
                        var tracker = getOrCreateTracker(listener.id(), source.id());
                        tracker.updateFromPing(tick, reportedRange, activeSe);
                        observedSourceIds.add(source.id());

                        active.add(new SonarContact(reportedBearing, activeSe, reportedRange, true,
                                estSpeed, activeBrgStdDev, rangeRmsNoise, estSL,
                                tracker.solutionQuality(), tracker.estimatedHeading(),
                                estimatedDepth, classify(activeSe, estSpeed, estSL)));
                    }
                }
            }

            // Decay and expire trackers for this listener
            decayAndExpireTrackers(tick, listener.id(), observedSourceIds);

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
            double top = layer.top();
            double bottom = layer.bottom();
            // Classify each sub's position relative to the gradient band.
            // "Above" = clearly above the layer, "below" = clearly below it,
            // "inside" = within the gradient band.
            boolean srcAbove = srcZ > top;
            boolean srcBelow = srcZ < bottom;
            boolean dstAbove = dstZ > top;
            boolean dstBelow = dstZ < bottom;

            // Same clear side: waveguide bonus
            if ((srcAbove && dstAbove) || (srcBelow && dstBelow)) {
                totalDb -= 2.0;
            } else if (srcAbove != dstAbove && srcBelow != dstBelow) {
                // One is clearly above and the other clearly below: full crossing.
                double tempDiff = layer.gradient();
                double maxPenalty = Math.clamp(3.0 + 0.5 * tempDiff, 5.0, 12.0);
                totalDb += maxPenalty;
            } else {
                // One is inside the band, the other is outside on one side.
                // Partial crossing penalty proportional to fraction traversed.
                double tempDiff = layer.gradient();
                double maxPenalty = Math.clamp(3.0 + 0.5 * tempDiff, 5.0, 12.0);

                if (layer.thickness() < 1.0) {
                    totalDb += maxPenalty;
                } else {
                    double srcClamped = Math.clamp(srcZ, bottom, top);
                    double dstClamped = Math.clamp(dstZ, bottom, top);
                    double crossedFraction = Math.abs(srcClamped - dstClamped) / layer.thickness();
                    totalDb += maxPenalty * crossedFraction;
                }
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

    // -- Contact tracker management --

    private static long trackerKey(int listenerId, int sourceId) {
        return ((long) listenerId << 32) | (sourceId & 0xFFFFFFFFL);
    }

    private ContactTracker getOrCreateTracker(int listenerId, int sourceId) {
        return trackers.computeIfAbsent(trackerKey(listenerId, sourceId), k -> new ContactTracker());
    }

    private void decayAndExpireTrackers(long tick, int listenerId, Set<Integer> observedSourceIds) {
        var iter = trackers.entrySet().iterator();
        while (iter.hasNext()) {
            var entry = iter.next();
            long key = entry.getKey();
            int keyListenerId = (int) (key >> 32);
            if (keyListenerId != listenerId) continue;

            int keySourceId = (int) key;
            var tracker = entry.getValue();

            if (!observedSourceIds.contains(keySourceId)) {
                tracker.decay(tick, maxSubSpeed);
            }
            if (tracker.isExpired(tick)) {
                iter.remove();
            }
        }
    }
}
