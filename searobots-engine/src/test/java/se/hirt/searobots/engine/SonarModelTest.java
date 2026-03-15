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

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import static se.hirt.searobots.api.VehicleConfig.submarine;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class SonarModelTest {

    private static final List<ThermalLayer> NO_LAYERS = List.of();

    private static TerrainMap deepFlat() {
        int size = 201;
        double cellSize = 100;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -500);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    private static final TerrainMap TERRAIN = deepFlat();

    private SubmarineEntity makeSub(int id, Vec3 pos, double heading, double sourceLevelDb) {
        var controller = new DefaultAttackSub();
        var sub = new SubmarineEntity(submarine(), id, controller, pos, heading, Color.GREEN, 1000);
        sub.setSourceLevelDb(sourceLevelDb);
        return sub;
    }

    // ── Transmission loss ────────────────────────────────────────────

    @Test
    void cylindricalSpreadingAt1km() {
        // TL at 1000m = 10 * log10(1000) = 30 dB
        var src = new Vec3(0, 0, -200);
        var dst = new Vec3(0, 1000, -200);
        double tl = SonarModel.transmissionLossDb(1000, src, dst, TERRAIN, NO_LAYERS);
        assertEquals(30.0, tl, 0.5, "TL at 1km should be ~30 dB, got " + tl);
    }

    @Test
    void closerSourceHasHigherSE() {
        // Same SL, half distance -> ~3 dB more SE (10 * log10(2) ~ 3 dB less TL)
        // Listener SL 80: self-noise = 80-30 = 50, NL = max(60, 50) = 60
        // SL 100 at 1000m: TL = 30, SE = 100-30-60 = 10 -> detected
        // SL 100 at 2000m: TL = 33, SE = 100-33-60 = 7 -> detected
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, 0, -200), 0, 80);

        var farSource = makeSub(1, new Vec3(0, 2000, -200), Math.PI, 100);
        var nearSource = makeSub(2, new Vec3(0, 1000, -200), Math.PI, 100);

        var resultsFar = sonar.computeContacts(0L,
                List.of(listener, farSource), TERRAIN, NO_LAYERS);
        var resultsNear = sonar.computeContacts(0L,
                List.of(listener, nearSource), TERRAIN, NO_LAYERS);

        var farContacts = resultsFar.get(0).passiveContacts();
        var nearContacts = resultsNear.get(0).passiveContacts();

        assertFalse(nearContacts.isEmpty(), "Near source should be detected");
        assertFalse(farContacts.isEmpty(), "Far source should also be detected");
        assertTrue(nearContacts.getFirst().signalExcess() > farContacts.getFirst().signalExcess(),
                "Closer source should have higher SE");
    }

    @Test
    void quietSubDetectedAtShorterRange() {
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        var quietSource = makeSub(1, new Vec3(0, 2000, -200), Math.PI, 88);
        var loudSource = makeSub(2, new Vec3(0, 2000, -200), Math.PI, 100);

        var qr = sonar.computeContacts(0L, List.of(listener, quietSource), TERRAIN, NO_LAYERS);
        var lr = sonar.computeContacts(0L, List.of(listener, loudSource), TERRAIN, NO_LAYERS);

        assertTrue(qr.get(0).passiveContacts().isEmpty(),
                "Quiet sub (SL 88) at 2km should NOT be detected");
        assertFalse(lr.get(0).passiveContacts().isEmpty(),
                "Loud sub (SL 100) at 2km SHOULD be detected");
    }

    // ── Terrain occlusion ────────────────────────────────────────────

    @Test
    void terrainOccludesSignal() {
        int size = 201;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -500);
        for (int x = 0; x < size; x++) {
            int midY = size / 2;
            data[midY * size + x] = -100; // ridge at -100m
        }
        var ridgeTerrain = new TerrainMap(data, size, size, origin, origin, cellSize);

        var src = new Vec3(0, -200, -200);
        var dst = new Vec3(0, 200, -200);
        double occlusion = SonarModel.terrainOcclusionDb(src, dst, ridgeTerrain);

        // Single-cell ridge: 1 half-cell sample hits -> 7.5 dB (half of per-cell 15 dB)
        assertEquals(7.5, occlusion, 1.0,
                "Thin ridge fully above path should give ~7.5 dB, got " + occlusion);
    }

    @Test
    void partialTerrainOcclusion() {
        int size = 201;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -500);
        for (int x = 0; x < size; x++) {
            int midY = size / 2;
            data[midY * size + x] = -190; // ridge at -190m
        }
        var ridgeTerrain = new TerrainMap(data, size, size, origin, origin, cellSize);

        var src = new Vec3(0, -200, -200);
        var dst = new Vec3(0, 200, -200);
        double occlusion = SonarModel.terrainOcclusionDb(src, dst, ridgeTerrain);

        double expected = 7.5 * 10.0 / 50.0; // 1.5 dB
        assertEquals(expected, occlusion, 0.5,
                "10m ridge should give ~1.5 dB occlusion, got " + occlusion);
    }

    @Test
    void islandBlocksSonarCompletely() {
        var src = new Vec3(0, -200, -200);
        var dst = new Vec3(0, 200, -200);

        int size = 201;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, 0); // entire floor at surface (island)
        var wallTerrain = new TerrainMap(data, size, size, origin, origin, cellSize);

        double occlusion = SonarModel.terrainOcclusionDb(src, dst, wallTerrain);
        assertTrue(occlusion > 500,
                "Island should produce massive occlusion (hundreds of dB), got " + occlusion);
    }

    // ── Thermocline ──────────────────────────────────────────────────

    @Test
    void sameThermalLayerHasDuctBonus() {
        var layers = List.of(new ThermalLayer(-120, 15.0, 5.0));
        double tl = SonarModel.thermoclineDb(-100, -100, layers);
        assertEquals(-2.0, tl, 0.01,
                "Same layer should give -2 dB bonus (duct), got " + tl);
    }

    @Test
    void crossThermoclineAttenuates() {
        var layers = List.of(new ThermalLayer(-120, 15.0, 5.0));
        double tl = SonarModel.thermoclineDb(-100, -200, layers);
        assertEquals(8.0, tl, 0.01,
                "Cross dT=10C layer should give +8 dB penalty, got " + tl);
    }

    @Test
    void strongerGradientMoreAttenuation() {
        var weakLayer = List.of(new ThermalLayer(-120, 12.5, 7.5));
        var strongLayer = List.of(new ThermalLayer(-120, 17.5, 2.5));

        double weakPenalty = SonarModel.thermoclineDb(-100, -200, weakLayer);
        double strongPenalty = SonarModel.thermoclineDb(-100, -200, strongLayer);

        assertTrue(strongPenalty > weakPenalty,
                "Stronger gradient should attenuate more: weak=" + weakPenalty
                        + " strong=" + strongPenalty);
    }

    // ── Self-noise masking ───────────────────────────────────────────

    @Test
    void selfNoiseRaisesThreshold() {
        var sonar = new SonarModel(42);
        var source = makeSub(1, new Vec3(0, 1000, -200), Math.PI, 100);
        var quietListener = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        var sprintListener = makeSub(2, new Vec3(0, 0, -200), 0, 120);

        var qr = sonar.computeContacts(0L,
                List.of(quietListener, source), TERRAIN, NO_LAYERS);
        var sr = sonar.computeContacts(0L,
                List.of(sprintListener, source), TERRAIN, NO_LAYERS);

        assertFalse(qr.get(0).passiveContacts().isEmpty(),
                "Quiet listener should detect SL 100 at 1km");
        assertTrue(sr.get(2).passiveContacts().isEmpty(),
                "Sprint listener (self-noise 90 dB) should NOT detect SL 100 at 1km");
    }

    // ── Baffles ──────────────────────────────────────────────────────

    @Test
    void baffleZoneDegradesDetection() {
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        var sourceBehind = makeSub(1, new Vec3(0, -500, -200), 0, 100);

        var results = sonar.computeContacts(0L,
                List.of(listener, sourceBehind), TERRAIN, NO_LAYERS);

        assertTrue(results.get(0).passiveContacts().isEmpty(),
                "Source in baffles should not be detected at 500m");
    }

    @Test
    void baffleZoneDoesNotAffectForwardArc() {
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        var sourceAhead = makeSub(1, new Vec3(0, 500, -200), Math.PI, 100);

        var results = sonar.computeContacts(0L,
                List.of(listener, sourceAhead), TERRAIN, NO_LAYERS);

        assertFalse(results.get(0).passiveContacts().isEmpty(),
                "Source in forward arc should be detected (no baffle penalty)");
    }

    // ── Bearing accuracy ─────────────────────────────────────────────

    @Test
    void bearingAccuracyImprovesWithSE() {
        double weakStdDev = SonarModel.bearingStdDev(6);
        double strongStdDev = SonarModel.bearingStdDev(30);

        assertTrue(strongStdDev < weakStdDev,
                "Strong SE should have smaller bearing error: weak=" + Math.toDegrees(weakStdDev)
                        + " strong=" + Math.toDegrees(strongStdDev));

        assertTrue(weakStdDev > Math.toRadians(3),
                "Weak SE should have >3 deg error, got " + Math.toDegrees(weakStdDev));
        assertTrue(strongStdDev < Math.toRadians(2),
                "Strong SE should have <2 deg error, got " + Math.toDegrees(strongStdDev));
    }

    // ── Active sonar ─────────────────────────────────────────────────

    @Test
    void activePingGivesRange() {
        var sonar = new SonarModel(42);
        var pinger = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        pinger.activeSonarPing();
        var target = makeSub(1, new Vec3(0, 1000, -200), Math.PI, 80);

        var results = sonar.computeContacts(0L,
                List.of(pinger, target), TERRAIN, NO_LAYERS);

        var activeReturns = results.get(0).activeReturns();
        assertFalse(activeReturns.isEmpty(), "Active ping should produce returns");
        assertTrue(activeReturns.getFirst().isActive(), "Return should be marked active");
        assertTrue(activeReturns.getFirst().range() > 0,
                "Active return should include range");
        assertEquals(1000, activeReturns.getFirst().range(), 50,
                "Range should be ~1000m, got " + activeReturns.getFirst().range());
    }

    @Test
    void activePingIsLoudForEveryone() {
        var sonar = new SonarModel(42);
        var pinger = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        pinger.activeSonarPing();
        var farListener = makeSub(1, new Vec3(0, 5000, -200), Math.PI, 80);

        var results = sonar.computeContacts(0L,
                List.of(pinger, farListener), TERRAIN, NO_LAYERS);

        var passiveOnListener = results.get(1).passiveContacts();
        assertFalse(passiveOnListener.isEmpty(),
                "Active ping at 220 dB should be heard at 5km");
        assertTrue(passiveOnListener.getFirst().signalExcess() > 100,
                "Ping should have enormous SE at 5km, got "
                        + passiveOnListener.getFirst().signalExcess());
    }

    @Test
    void activePingCooldownEnforced() {
        var sonar = new SonarModel(42);
        var pinger = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        var target = makeSub(1, new Vec3(0, 500, -200), Math.PI, 80);

        pinger.activeSonarPing();
        var results1 = sonar.computeContacts(0L,
                List.of(pinger, target), TERRAIN, NO_LAYERS);
        sonar.postTick(List.of(pinger, target));

        assertFalse(results1.get(0).activeReturns().isEmpty(),
                "First ping should produce returns");
        assertEquals(SonarModel.ACTIVE_PING_COOLDOWN_TICKS - 1, pinger.activeSonarCooldown(),
                "Cooldown should be set by computeContacts and decremented once by postTick");

        pinger.activeSonarPing();
        var results2 = sonar.computeContacts(0L,
                List.of(pinger, target), TERRAIN, NO_LAYERS);

        assertTrue(results2.get(0).activeReturns().isEmpty(),
                "Second ping during cooldown should produce no returns");
    }

    @Test
    void activePingUsesRoundTripTL() {
        var sonar = new SonarModel(42);
        var pinger1 = makeSub(0, new Vec3(0, 0, -200), 0, 80);
        pinger1.activeSonarPing();
        var nearTarget = makeSub(1, new Vec3(0, 1000, -200), Math.PI, 80);

        var pinger2 = makeSub(2, new Vec3(0, 0, -200), 0, 80);
        pinger2.activeSonarPing();
        var farTarget = makeSub(3, new Vec3(0, 5000, -200), Math.PI, 80);

        var nearResults = sonar.computeContacts(0L,
                List.of(pinger1, nearTarget), TERRAIN, NO_LAYERS);
        var farResults = sonar.computeContacts(0L,
                List.of(pinger2, farTarget), TERRAIN, NO_LAYERS);

        var nearActive = nearResults.get(0).activeReturns();
        var farActive = farResults.get(2).activeReturns();

        assertFalse(nearActive.isEmpty(), "Near active should have returns");
        assertFalse(farActive.isEmpty(), "Far active should have returns");
        assertTrue(nearActive.getFirst().signalExcess() > farActive.getFirst().signalExcess(),
                "Near target should have higher active SE (round-trip TL is 2x)");
    }

    // ── Baffle helper ────────────────────────────────────────────────

    @Test
    void isInBafflesCorrectForSternBearing() {
        assertTrue(SonarModel.isInBaffles(0, Math.PI));
        assertTrue(SonarModel.isInBaffles(0, Math.toRadians(160)));
    }

    @Test
    void isInBafflesCorrectForBowBearing() {
        assertFalse(SonarModel.isInBaffles(0, 0));
        assertFalse(SonarModel.isInBaffles(0, Math.toRadians(30)));
        assertFalse(SonarModel.isInBaffles(0, Math.toRadians(120)));
    }

    // ══════════════════════════════════════════════════════════════════
    // Realistic terrain scenarios
    // ══════════════════════════════════════════════════════════════════
    //
    // All terrains use 10m cell size (same as default MatchConfig).
    // Z=0 is sea level, negative is deeper.
    //
    // IMPORTANT: listener heading must face toward source to avoid the
    // baffle zone (130+ deg from bow = +20 dB noise penalty).

    // ── Terrain builders ──────────────────────────────────────────────

    /** Deep ocean (-400m) with a circular island centered at (cx, cy), peak +50m. */
    private static TerrainMap islandTerrain(double cx, double cy, double radius) {
        int size = 401;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                double wy = origin + row * cellSize;
                double dist = Math.sqrt((wx - cx) * (wx - cx) + (wy - cy) * (wy - cy));
                if (dist < radius) {
                    double t = dist / radius;
                    data[row * size + col] = 50.0 * (1 - t * t) - 400.0 * t * t;
                } else {
                    data[row * size + col] = -400;
                }
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /** Two circular islands at y=+-gap/2, each with given radius. Deep ocean between/around. */
    private static TerrainMap channelTerrain(double gap, double islandRadius) {
        int size = 401;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                double wx = origin + col * cellSize;
                double wy = origin + row * cellSize;
                double d1 = Math.sqrt(wx * wx + (wy + gap / 2) * (wy + gap / 2));
                double d2 = Math.sqrt(wx * wx + (wy - gap / 2) * (wy - gap / 2));
                double elev = -400;
                if (d1 < islandRadius) {
                    double t = d1 / islandRadius;
                    elev = Math.max(elev, 50.0 * (1 - t * t) - 400.0 * t * t);
                }
                if (d2 < islandRadius) {
                    double t = d2 / islandRadius;
                    elev = Math.max(elev, 50.0 * (1 - t * t) - 400.0 * t * t);
                }
                data[row * size + col] = elev;
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /**
     * Continental shelf: shallow (-50m) for y > 0, drops steeply to -400m for y < -200.
     * Cliff face is a 200m linear transition between y=-200 and y=0.
     */
    private static TerrainMap shelfCliffTerrain() {
        int size = 401;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            double elev;
            if (wy > 0) {
                elev = -50;
            } else if (wy < -200) {
                elev = -400;
            } else {
                double t = -wy / 200.0; // 0 at y=0 (shelf), 1 at y=-200 (deep)
                elev = -50 + t * (-400 + 50);
            }
            for (int col = 0; col < size; col++) {
                data[row * size + col] = elev;
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /** Deep ocean (-400m) with Gaussian ridge at y=0, peaking at ridgePeakDepth. */
    private static TerrainMap underwaterRidgeTerrain(double ridgePeakDepth) {
        int size = 401;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        double sigma = 200;
        for (int row = 0; row < size; row++) {
            double wy = origin + row * cellSize;
            double rise = (ridgePeakDepth + 400) // how much above -400
                    * Math.exp(-(wy * wy) / (2 * sigma * sigma));
            for (int col = 0; col < size; col++) {
                data[row * size + col] = -400 + rise;
            }
        }
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    /** Heading from (x1,y1) toward (x2,y2). */
    private static double headingToward(double x1, double y1, double x2, double y2) {
        double h = Math.atan2(x2 - x1, y2 - y1);
        if (h < 0) h += 2 * Math.PI;
        return h;
    }

    // ── Island cover: passive detection ───────────────────────────────

    @Test
    void islandBlocksPassiveDetection() {
        // Island at origin (r=200). Loud source north, listener south.
        // Sound path goes straight through the island.
        var terrain = islandTerrain(0, 0, 200);
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, -800, -200), 0, 80); // facing north
        var source = makeSub(1, new Vec3(0, 800, -200), Math.PI, 120);

        var results = sonar.computeContacts(0L, List.of(listener, source), terrain, NO_LAYERS);
        assertTrue(results.get(0).passiveContacts().isEmpty(),
                "Island should block passive detection even for a loud source");
    }

    @Test
    void islandDoesNotBlockWhenPathIsClear() {
        // Island at origin, but source offset east. Path goes around island.
        var terrain = islandTerrain(0, 0, 200);
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, -800, -200),
                headingToward(0, -800, 600, 800), 80);
        var source = makeSub(1, new Vec3(600, 800, -200), Math.PI, 120);

        var results = sonar.computeContacts(0L, List.of(listener, source), terrain, NO_LAYERS);
        assertFalse(results.get(0).passiveContacts().isEmpty(),
                "Clear path around island should allow passive detection");
    }

    @Test
    void sameSideOfIslandNoCover() {
        // Both subs north of the island, no terrain between them.
        var terrain = islandTerrain(0, 0, 200);
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, 400, -200), 0, 80); // north, facing north
        var source = makeSub(1, new Vec3(0, 800, -200), Math.PI, 100);

        var results = sonar.computeContacts(0L, List.of(listener, source), terrain, NO_LAYERS);
        assertFalse(results.get(0).passiveContacts().isEmpty(),
                "Both subs on same side of island should detect normally");
    }

    // ── Island cover: active ping ─────────────────────────────────────

    @Test
    void islandBlocksActivePing() {
        // Pinger south of island, target north. 220 dB ping can't penetrate rock.
        var terrain = islandTerrain(0, 0, 200);
        var sonar = new SonarModel(42);
        var pinger = makeSub(0, new Vec3(0, -600, -200), 0, 80);
        pinger.activeSonarPing();
        var target = makeSub(1, new Vec3(0, 600, -200), Math.PI, 80);

        var results = sonar.computeContacts(0L, List.of(pinger, target), terrain, NO_LAYERS);
        assertTrue(results.get(0).activeReturns().isEmpty(),
                "Active ping should not return through an island");
        assertTrue(results.get(1).passiveContacts().isEmpty(),
                "Target should not hear the ping through an island");
    }

    @Test
    void activePingWorksAroundIsland() {
        // Pinger and target south of island, clear east-west path.
        var terrain = islandTerrain(0, 0, 200);
        var sonar = new SonarModel(42);
        var pinger = makeSub(0, new Vec3(-500, -400, -200),
                headingToward(-500, -400, 500, -400), 80);
        pinger.activeSonarPing();
        var target = makeSub(1, new Vec3(500, -400, -200), Math.PI, 80);

        var results = sonar.computeContacts(0L, List.of(pinger, target), terrain, NO_LAYERS);
        assertFalse(results.get(0).activeReturns().isEmpty(),
                "Active ping should work when path doesn't cross island");
    }

    // ── Channel between two islands ──────────────────────────────────

    @Test
    void soundTravelsThroughWideChannel() {
        // Two islands at y=+-500, r=200. Subs east-west so path goes through the gap.
        //
        //   [Island at (0,-500)]
        //        gap (y=-300 to y=+300)
        //   [Island at (0,+500)]
        //
        //   Source (-800, 0) ---path through gap---> Listener (800, 0)
        //
        var terrain = channelTerrain(1000, 200);
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(800, 0, -200),
                headingToward(800, 0, -800, 0), 80);
        var source = makeSub(1, new Vec3(-800, 0, -200), 0, 120);

        assertTrue(terrain.elevationAt(0, 0) < -300, "Mid-channel should be deep ocean");
        var results = sonar.computeContacts(0L, List.of(listener, source), terrain, NO_LAYERS);
        assertFalse(results.get(0).passiveContacts().isEmpty(),
                "Sound should travel through wide channel between islands");
    }

    @Test
    void soundBlockedThroughIslandNotChannel() {
        // Same terrain, but path goes north-south through one island.
        var terrain = channelTerrain(1000, 200);
        var sonar = new SonarModel(42);
        var listener = makeSub(0, new Vec3(0, 800, -200), Math.PI, 80);
        var source = makeSub(1, new Vec3(0, -800, -200), 0, 120);

        // Path from (0,-800) to (0,800) goes through island at (0,-500)
        assertTrue(terrain.elevationAt(0, -500) > 0, "Island center should be above sea level");
        var results = sonar.computeContacts(0L, List.of(listener, source), terrain, NO_LAYERS);
        assertTrue(results.get(0).passiveContacts().isEmpty(),
                "Path through island should block detection");
    }

    // ── Continental shelf / cliff ─────────────────────────────────────
    //
    // y > 0: shelf at -50m.  y < -200: deep at -400m.
    // Cliff face between y=-200 and y=0 (linear -50 to -400).

    @Test
    void cliffOcclusionBasic() {
        // Acoustic path crosses the cliff face.
        var terrain = shelfCliffTerrain();
        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -300, -300), new Vec3(0, 200, -80), terrain);
        // At y=0: cliff is -50m, path depth = -300 + 0.6*220 = -168 -> cliff 118m above path
        assertTrue(occlusion > 20,
                "Cliff should produce significant occlusion, got " + occlusion + " dB");
    }

    @Test
    void deepSubBelowCliffHiddenFromShallowSub() {
        // Deep sub just south of cliff, shallow sub on the shelf.
        var terrain = shelfCliffTerrain();
        var sonar = new SonarModel(42);
        var deepSub = makeSub(0, new Vec3(0, -300, -350),
                headingToward(0, -300, 0, 300), 80); // facing toward source
        var shallowSub = makeSub(1, new Vec3(0, 300, -80), Math.PI, 120);

        var results = sonar.computeContacts(0L, List.of(deepSub, shallowSub), terrain, NO_LAYERS);
        assertTrue(results.get(0).passiveContacts().isEmpty(),
                "Cliff should hide deep sub from shallow sub");
    }

    @Test
    void noCliffOcclusionWhenBothOnDeepSide() {
        // Both subs south of cliff in deep water, floor at -400m, no terrain above path.
        var terrain = shelfCliffTerrain();
        var sonar = new SonarModel(42);
        var sub1 = makeSub(0, new Vec3(0, -400, -300), Math.PI, 80); // facing south
        var sub2 = makeSub(1, new Vec3(0, -800, -300), 0, 100);

        var results = sonar.computeContacts(0L, List.of(sub1, sub2), terrain, NO_LAYERS);
        assertFalse(results.get(0).passiveContacts().isEmpty(),
                "Both on deep side -- cliff doesn't block");
    }

    @Test
    void shelfOcclusionScalesWithPathLength() {
        // Longer path through the shelf should produce more occlusion.
        var terrain = shelfCliffTerrain();

        double shortOcclusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -250, -300), new Vec3(0, 50, -80), terrain);
        double longOcclusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -300), new Vec3(0, 500, -80), terrain);

        assertTrue(longOcclusion > shortOcclusion,
                "Longer path through shelf should produce more occlusion: short="
                        + shortOcclusion + " long=" + longOcclusion);
        assertTrue(shortOcclusion > 5,
                "Even short cliff crossing should produce meaningful occlusion, got " + shortOcclusion);
    }

    // ── Underwater ridge ──────────────────────────────────────────────

    @Test
    void underwaterRidgePartiallyOccludes() {
        // Ridge peaks at -80m. Path at -200m crosses it.
        var terrain = underwaterRidgeTerrain(-80);

        double peakElev = terrain.elevationAt(0, 0);
        assertEquals(-80, peakElev, 5, "Ridge peak should be near -80m, got " + peakElev);

        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -200), new Vec3(0, 500, -200), terrain);
        assertTrue(occlusion > 30,
                "Tall underwater ridge should produce significant occlusion, got " + occlusion + " dB");
    }

    @Test
    void underwaterRidgeNoOcclusionWhenAboveIt() {
        // Path at -60m is ABOVE the ridge peak at -80m.
        var terrain = underwaterRidgeTerrain(-80);
        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -60), new Vec3(0, 500, -60), terrain);
        assertEquals(0, occlusion, 0.1,
                "Sound path above ridge peak should have zero occlusion");
    }

    @Test
    void deepSubHidesBehindRidgeFromPinger() {
        // Pinger at -100m, target at -300m on other side of ridge (peak -80m).
        var terrain = underwaterRidgeTerrain(-80);
        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -100), new Vec3(0, 500, -300), terrain);
        assertTrue(occlusion > 30,
                "Ridge should heavily occlude ping path, got " + occlusion + " dB");
    }

    @Test
    void shallowRidgeMinimalEffect() {
        // Ridge peaks at -350m (only 50m above -400 floor). Path at -380m.
        // Ridge barely pokes 30m above path near peak, small occlusion.
        var terrain = underwaterRidgeTerrain(-350);
        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -380), new Vec3(0, 500, -380), terrain);
        assertTrue(occlusion > 0,
                "Ridge above path should cause some occlusion, got " + occlusion + " dB");

        // Compare with tall ridge
        var tallTerrain = underwaterRidgeTerrain(-80);
        double tallOcclusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -200), new Vec3(0, 500, -200), tallTerrain);
        assertTrue(tallOcclusion > occlusion,
                "Tall ridge should block more: tall=" + tallOcclusion + " shallow=" + occlusion);
    }

    @Test
    void ridgeLessEffectiveThanIsland() {
        // Same geometry, ridge (-80m peak) vs island (+50m peak).
        var ridgeTerrain = underwaterRidgeTerrain(-80);
        var islandTerr = islandTerrain(0, 0, 300);

        double ridgeOcclusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -200), new Vec3(0, 500, -200), ridgeTerrain);
        double islandOcclusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -200), new Vec3(0, 500, -200), islandTerr);

        assertTrue(islandOcclusion > ridgeOcclusion,
                "Island should block more than a ridge: island=" + islandOcclusion
                        + " ridge=" + ridgeOcclusion);
    }

    // ── Narrow / thin feature detection ───────────────────────────────

    @Test
    void narrowIslandDetectedAtLongRange() {
        // Thin island (r=30, ~6 cells across). At 3km range, old fixed-sample
        // approach would have missed it.
        var terrain = islandTerrain(0, 0, 30);
        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -1500, -200), new Vec3(0, 1500, -200), terrain);
        assertTrue(occlusion > 0,
                "Narrow island should still be detected at long range, got " + occlusion + " dB");
    }

    @Test
    void thinPeninsulaBlocksPath() {
        // 20m-wide strip of island spanning the map at y=0.
        int size = 401;
        double cellSize = 10;
        double origin = -(size / 2) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -400);
        int midRow = size / 2;
        for (int col = 0; col < size; col++) {
            data[midRow * size + col] = 20;
            data[(midRow + 1) * size + col] = 20;
        }
        var terrain = new TerrainMap(data, size, size, origin, origin, cellSize);

        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -1500, -200), new Vec3(0, 1500, -200), terrain);
        assertTrue(occlusion > 10,
                "Thin peninsula (20m) should be detected and block sound, got " + occlusion + " dB");
    }

    // ── Tactical hiding strategies ────────────────────────────────────

    @Test
    void cliffBaseHidingStrategyWorks() {
        // Sub at -350m tucked against cliff base, hidden from sub on shelf.
        var terrain = shelfCliffTerrain();
        var sonar = new SonarModel(42);
        var hider = makeSub(0, new Vec3(0, -250, -350),
                headingToward(0, -250, 0, 400), 80);
        var hunter = makeSub(1, new Vec3(0, 400, -80), Math.PI, 100);

        var results = sonar.computeContacts(0L, List.of(hider, hunter), terrain, NO_LAYERS);
        assertTrue(results.get(0).passiveContacts().isEmpty(),
                "Sub hugging cliff base should be hidden from sub on the shelf");
    }

    @Test
    void cliffBaseHiderExposedFromDeepSide() {
        // Same hider, but hunter approaches from the deep side. No cliff between them.
        var terrain = shelfCliffTerrain();
        var sonar = new SonarModel(42);
        var hider = makeSub(0, new Vec3(0, -250, -350),
                headingToward(0, -250, 0, -800), 80); // facing toward hunter
        var hunter = makeSub(1, new Vec3(0, -800, -300), 0, 100);

        var results = sonar.computeContacts(0L, List.of(hider, hunter), terrain, NO_LAYERS);
        assertFalse(results.get(0).passiveContacts().isEmpty(),
                "Cliff base hider exposed to hunter from the deep side");
    }

    @Test
    void ridgeProvidesCoverButNotTotal() {
        // Sub behind underwater ridge (peak -80m) at -250m.
        // Loud hunter on other side at -150m.
        var terrain = underwaterRidgeTerrain(-80);
        var sonar = new SonarModel(42);
        var hider = makeSub(0, new Vec3(0, 500, -250),
                headingToward(0, 500, 0, -500), 80);
        var hunter = makeSub(1, new Vec3(0, -500, -150), 0, 120);

        double occlusion = SonarModel.terrainOcclusionDb(
                new Vec3(0, -500, -150), new Vec3(0, 500, -250), terrain);
        assertTrue(occlusion > 10,
                "Ridge should provide meaningful cover, got " + occlusion + " dB");
    }
}
