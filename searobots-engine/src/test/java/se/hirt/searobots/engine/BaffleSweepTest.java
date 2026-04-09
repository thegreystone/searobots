/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.ThermalLayer;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.*;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;
import static se.hirt.searobots.api.VehicleConfig.submarine;

/**
 * Diagnostic sweep: verifies that the baffle zone is correctly oriented
 * relative to the listener's heading, and maps detection across the full
 * 360° arc.
 *
 * Coordinate convention:
 *   bearing  0  = north (+Y)
 *   bearing  90 = east  (+X)
 *   heading  0  = sub pointing north (bow toward +Y)
 */
class BaffleSweepTest {

    private static final List<ThermalLayer> NO_LAYERS = List.of();
    private static final TerrainMap TERRAIN = deepFlat();

    private static TerrainMap deepFlat() {
        int size = 501;
        double cellSize = 100;
        double origin = -(size / 2.0) * cellSize;
        double[] data = new double[size * size];
        java.util.Arrays.fill(data, -500);
        return new TerrainMap(data, size, size, origin, origin, cellSize);
    }

    private SubmarineEntity makeSub(int id, Vec3 pos, double heading, double sourceLevelDb) {
        var sub = new SubmarineEntity(submarine(), id, new DefaultAttackSub(), pos, heading, Color.GREEN, 1000);
        sub.setSourceLevelDb(sourceLevelDb);
        sub.setSpeed(5);
        return sub;
    }

    /** Direct baffle check sweep — no randomness, tests the pure geometry. */
    @Test
    void baffleGeometrySweep() {
        System.out.println("\n=== Baffle geometry: isInBaffles(listenerHeading, bearingToSource) ===");
        System.out.println("Listener heading 0° (north). Source at various absolute bearings.");
        System.out.printf("Baffle half-arc = %.0f° → baffled when |relative| > %.0f°%n",
                Math.toDegrees(SonarModel.BAFFLE_HALF_ARC),
                Math.toDegrees(SonarModel.BAFFLE_HALF_ARC));
        System.out.printf("%-20s %-15s %-15s %-10s%n", "Abs bearing", "Rel bearing", "Baffled?", "Meaning");
        System.out.println("-".repeat(65));

        double listenerHdg = 0; // heading north
        int[] angles = {0, 30, 60, 90, 120, 130, 135, 150, 180, 210, 225, 230, 270, 300, 330};
        for (int deg : angles) {
            double bearing = Math.toRadians(deg);
            boolean baffled = SonarModel.isInBaffles(listenerHdg, bearing);
            double relDeg = Math.toDegrees(bearing - listenerHdg);
            if (relDeg > 180) relDeg -= 360;
            if (relDeg < -180) relDeg += 360;
            String meaning = deg == 0 ? "dead ahead" : deg == 90 ? "starboard beam" :
                             deg == 180 ? "dead astern" : deg == 270 ? "port beam" :
                             deg < 180 ? "starboard" : "port";
            System.out.printf("%-20s %-15s %-15s %-10s%n",
                    deg + "°", String.format("%.0f°", relDeg),
                    baffled ? "BAFFLED" : "clear", meaning);
        }

        // Verify key angles
        assertFalse(SonarModel.isInBaffles(0, 0),               "Dead ahead must NOT be baffled");
        assertFalse(SonarModel.isInBaffles(0, Math.PI / 2),     "Starboard beam must NOT be baffled");
        assertFalse(SonarModel.isInBaffles(0, -Math.PI / 2),    "Port beam must NOT be baffled");
        assertTrue (SonarModel.isInBaffles(0, Math.PI),         "Dead astern MUST be baffled");

        // Exact boundary: 130° is the cutoff
        assertFalse(SonarModel.isInBaffles(0, Math.toRadians(129)), "129° should NOT be baffled");
        assertTrue (SonarModel.isInBaffles(0, Math.toRadians(131)), "131° MUST be baffled");
        assertFalse(SonarModel.isInBaffles(0, Math.toRadians(-129)), "-129° should NOT be baffled");
        assertTrue (SonarModel.isInBaffles(0, Math.toRadians(-131)), "-131° MUST be baffled");
    }

    /** Heading-varying baffle check: rotate the listener, verify bow always leads. */
    @Test
    void baffleRotatesWithHeading() {
        System.out.println("\n=== Baffle follows heading: listener at various headings, source 500m due north ===");
        System.out.printf("%-20s %-20s %-15s%n", "Listener heading", "Rel bearing to N", "Baffled?");
        System.out.println("-".repeat(60));

        // Source is always due north (bearing = 0)
        double sourceBearing = 0;
        int[] headings = {0, 45, 90, 135, 180, 225, 270, 315};
        for (int hdgDeg : headings) {
            double hdg = Math.toRadians(hdgDeg);
            boolean baffled = SonarModel.isInBaffles(hdg, sourceBearing);
            double rel = sourceBearing - hdg;
            if (rel > Math.PI) rel -= 2 * Math.PI;
            if (rel < -Math.PI) rel += 2 * Math.PI;
            System.out.printf("%-20s %-20s %-15s%n",
                    hdgDeg + "° (" + headingName(hdgDeg) + ")",
                    String.format("%.0f°", Math.toDegrees(rel)),
                    baffled ? "BAFFLED" : "clear");
        }

        // Sub heading north: source north → clear (ahead)
        assertFalse(SonarModel.isInBaffles(0,          0), "N heading, N source: ahead = clear");
        // Sub heading south: source north → 180° relative = astern = baffled
        assertTrue (SonarModel.isInBaffles(Math.PI,    0), "S heading, N source: astern = baffled");
        // Sub heading east: source north → -90° relative = port bow = clear
        assertFalse(SonarModel.isInBaffles(Math.PI/2,  0), "E heading, N source: port bow = clear");
        // Sub heading west: source north → +90° relative = stbd bow = clear
        assertFalse(SonarModel.isInBaffles(3*Math.PI/2, 0), "W heading, N source: stbd bow = clear");
    }

    /**
     * Full passive-detection sweep: listener heading north, source at 1000m at every 15°.
     * Source SL set to ensure detection when not baffled but marginal when baffled.
     */
    @Test
    void passiveDetectionSweep() {
        System.out.println("\n=== Passive detection sweep (deterministic, no RNG noise) ===");
        System.out.println("Listener heading 0° (north), source SL=100 dB at 1000m.");
        System.out.println("TL at 1000m ≈ 30 dB, NL_base=55 dB → SE=15 dB when clear, SE=-5 dB when baffled");
        System.out.printf("%-12s %-12s %-12s %-12s %-10s%n",
                "Src bearing", "Rel bearing", "Expected", "isInBaffles", "SE (approx)");
        System.out.println("-".repeat(62));

        double listenerHdg = 0;
        double sl = 100;
        double tl = 10 * Math.log10(1000); // ~30 dB at 1000m
        double nlClear   = Math.max(55, 80 - 35); // = 55 (ambient dominates for quiet listener)
        double nlBaffled = nlClear + SonarModel.BAFFLE_PENALTY_DB;

        for (int deg = 0; deg < 360; deg += 15) {
            double bearing = Math.toRadians(deg);
            boolean baffled = SonarModel.isInBaffles(listenerHdg, bearing);
            double nl = baffled ? nlBaffled : nlClear;
            double se = sl - tl - nl;
            double rel = Math.toDegrees(bearing - listenerHdg);
            if (rel > 180) rel -= 360;
            if (rel < -180) rel += 360;
            boolean detected = se > SonarModel.DETECTION_THRESHOLD_DB;
            System.out.printf("%-12s %-12s %-12s %-12s %-10s%n",
                    deg + "°", String.format("%.0f°", rel),
                    detected ? "DETECT" : "miss",
                    baffled ? "BAFFLED" : "clear",
                    String.format("%.1f dB", se));
        }
    }

    /**
     * Models the "just passed" scenario: two subs heading in opposite directions,
     * post-crossing, each is dead astern of the other.
     */
    @Test
    void opposingHeadingsPostCrossing() {
        System.out.println("\n=== Post-crossing scenario: subs heading away from each other ===");
        System.out.println("Sub A: heading north (0°), at origin");
        System.out.println("Sub B: heading south (180°), 1000m north of A");
        System.out.println("B is AHEAD of A (bearing 0°, rel bearing 0° → CLEAR for A)");
        System.out.println("A is ASTERN of B (bearing 180°, heading 180°, rel bearing 0° → CLEAR for B)");
        System.out.println();

        // A heading north, B is north of A (B is AHEAD of A, NOT in A's baffles)
        // B heading south, A is south of B (A is AHEAD of B, NOT in B's baffles!)
        // When subs head TOWARD each other they are in each other's forward arcs.

        boolean aHearB = !SonarModel.isInBaffles(0,        0);       // A heading N, B at bearing 0 (north)
        boolean bHearA = !SonarModel.isInBaffles(Math.PI,  Math.PI); // B heading S, A at bearing 180° (south)

        System.out.printf("A can hear B (B ahead of A): %b%n", aHearB);
        System.out.printf("B can hear A (A ahead of B): %b%n", bHearA);
        assertTrue(aHearB, "A heading north, B is north: B is ahead, should be heard");
        assertTrue(bHearA, "B heading south, A is south: A is ahead, should be heard");

        System.out.println();
        System.out.println("--- After crossing (A continues north, B continues south, now 1000m apart) ---");
        System.out.println("Sub A: heading north (0°), B is now 1000m SOUTH of A");
        System.out.println("Sub B: heading south (180°), A is now 1000m NORTH of B");

        // A heading north, B is now SOUTH of A → bearing from A to B = 180° = dead astern
        boolean aHearBAfter = !SonarModel.isInBaffles(0,        Math.PI);       // A heading N, B at south = 180°
        // B heading south, A is now NORTH of B → bearing from B to A = 0° = dead astern of south-heading B
        boolean bHearAAfter = !SonarModel.isInBaffles(Math.PI,  0);             // B heading S, A at north = 0°

        System.out.printf("A can hear B (B astern of A): %b%n", aHearBAfter);
        System.out.printf("B can hear A (A astern of B): %b%n", bHearAAfter);
        assertFalse(aHearBAfter, "A heading north, B is south: B is astern, should be BAFFLED");
        assertFalse(bHearAAfter, "B heading south, A is north: A is astern, should be BAFFLED");

        System.out.println();
        System.out.println("CONCLUSION: After crossing, both subs are in each other's baffles.");
        System.out.println("Firing solutions after crossing are based on STALE track data, not fresh contacts.");
        System.out.println("This is expected behaviour (track held in memory), not a baffle bug.");
    }

    /**
     * Models the "one overtook the other" (same direction) post-crossing scenario.
     */
    @Test
    void sameDirectionOvertaking() {
        System.out.println("\n=== Overtaking scenario: A overtook B, both heading north ===");
        System.out.println("Sub A: heading north (0°), now 1000m north of B");
        System.out.println("Sub B: heading north (0°), 1000m south of A");

        // A heading north, B is SOUTH of A → bearing from A to B = 180° → BAFFLED for A
        boolean aHearB = !SonarModel.isInBaffles(0, Math.PI);
        // B heading north, A is NORTH of B → bearing from B to A = 0° → clear for B
        boolean bHearA = !SonarModel.isInBaffles(0, 0);

        System.out.printf("A can hear B (B astern of A): %b  ← B is in A's baffles%n", aHearB);
        System.out.printf("B can hear A (A ahead of B):  %b  ← A is in B's forward arc%n", bHearA);

        assertFalse(aHearB, "After overtaking: B is behind A, should be baffled");
        assertTrue (bHearA, "After overtaking: A is ahead of B, should be heard");

        System.out.println();
        System.out.println("CONCLUSION: B gets a live contact on A (A is ahead of B).");
        System.out.println("A gets no live contact on B (B is behind A, baffled).");
        System.out.println("B can generate a firing solution on A; A cannot on B from fresh data.");
    }

    private static String headingName(int deg) {
        return switch (deg) {
            case 0 -> "N"; case 45 -> "NE"; case 90 -> "E"; case 135 -> "SE";
            case 180 -> "S"; case 225 -> "SW"; case 270 -> "W"; case 315 -> "NW";
            default -> "?";
        };
    }
}
