/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Detects artificial plateaus in terrain generation by checking how many
 * above-water cells share nearly the same elevation. A plateau is a cluster
 * of adjacent cells within a narrow elevation band.
 */
public class TerrainPlateauTest {

    @Test
    void noPlateausIn20Seeds() {
        long[] seeds = new long[20];
        for (int i = 0; i < 20; i++) seeds[i] = 0xCAFE0000L + i * 0xBEEF;

        int worstPlateau = 0;
        long worstSeed = 0;
        double worstPct = 0;
        int seedsWithIslands = 0;

        System.out.println("=== Terrain Plateau Detection ===");
        System.out.printf("%-14s %8s %8s %8s %8s %8s  %s%n",
                "Seed", "AboveW", "MaxElev", "Plateau", "Pct", "Band", "Status");
        System.out.println("-".repeat(80));

        for (long seed : seeds) {
            var result = analyzePlateau(seed);
            if (result.aboveWaterCells < 100) continue; // skip seeds with tiny/no islands
            seedsWithIslands++;

            double pct = 100.0 * result.plateauCells / result.aboveWaterCells;
            String status = pct > 20 ? "PLATEAU!" : pct > 10 ? "marginal" : "ok";

            System.out.printf("0x%08x %8d %8.0f %8d %7.1f%% %7.1fm  %s%n",
                    seed, result.aboveWaterCells, result.maxElevation,
                    result.plateauCells, pct, result.plateauBandCenter, status);

            if (pct > worstPct) {
                worstPct = pct;
                worstSeed = seed;
            }
            if (result.plateauCells > worstPlateau) {
                worstPlateau = result.plateauCells;
            }
        }

        System.out.printf("%nSeeds with islands: %d / %d%n", seedsWithIslands, seeds.length);
        if (seedsWithIslands > 0) {
            assertTrue(worstPct < 25,
                    String.format("Worst plateau: %.1f%% of above-water cells in one band (seed 0x%x). " +
                            "Should be < 25%%.", worstPct, worstSeed));
        }
    }

    /** Detect LOCAL flat regions: contiguous above-water cells with slope < threshold. */
    @Test
    void noLargeFlatRegions() {
        long[] seeds = new long[10];
        for (int i = 0; i < 10; i++) seeds[i] = 0xF1A70000L + i * 0x1337;

        int worstBiggestFlat = 0;
        long worstFlatSeed = 0;

        System.out.println("\n=== Local Flatness Detection ===");
        System.out.printf("%-14s %8s %8s %8s %8s  %s%n",
                "Seed", "AboveW", "FlatCnt", "BigFlat", "FlatPct", "Status");
        System.out.println("-".repeat(70));

        for (long seed : seeds) {
            var config = MatchConfig.withDefaults(seed);
            var terrain = new WorldGenerator().generate(config).terrain();
            int cols = terrain.getCols(), rows = terrain.getRows();
            double cell = terrain.getCellSize();

            // Count above-water cells and flat above-water cells (slope < 3 degrees)
            int aboveWater = 0;
            int flatCells = 0;
            boolean[][] isFlat = new boolean[rows][cols];

            for (int r = 1; r < rows - 1; r++) {
                for (int c = 1; c < cols - 1; c++) {
                    double e = terrain.elevationAtGrid(c, r);
                    if (e <= 0) continue;
                    aboveWater++;
                    double dzdx = (terrain.elevationAtGrid(c+1, r) - terrain.elevationAtGrid(c-1, r)) / (2 * cell);
                    double dzdy = (terrain.elevationAtGrid(c, r+1) - terrain.elevationAtGrid(c, r-1)) / (2 * cell);
                    double slopeDeg = Math.toDegrees(Math.atan(Math.sqrt(dzdx*dzdx + dzdy*dzdy)));
                    if (slopeDeg < 3) {
                        flatCells++;
                        isFlat[r][c] = true;
                    }
                }
            }

            // Flood-fill to find largest contiguous flat region
            boolean[][] visited = new boolean[rows][cols];
            int biggestFlat = 0;
            for (int r = 1; r < rows - 1; r++) {
                for (int c = 1; c < cols - 1; c++) {
                    if (isFlat[r][c] && !visited[r][c]) {
                        int size = floodFill(isFlat, visited, r, c, rows, cols);
                        biggestFlat = Math.max(biggestFlat, size);
                    }
                }
            }

            double flatPct = aboveWater > 0 ? 100.0 * flatCells / aboveWater : 0;
            double bigFlatArea = biggestFlat * cell * cell; // m²
            String status = biggestFlat > 5000 ? "PLATEAU!" : biggestFlat > 2000 ? "marginal" : "ok";

            System.out.printf("0x%08x %8d %8d %8d %7.1f%%  %s (biggest=%.0fm²)%n",
                    seed, aboveWater, flatCells, biggestFlat, flatPct, status, bigFlatArea);

            if (biggestFlat > worstBiggestFlat) {
                worstBiggestFlat = biggestFlat;
                worstFlatSeed = seed;
            }
        }

        assertTrue(worstBiggestFlat < 5000,
                String.format("Largest contiguous flat region: %d cells (seed 0x%x). Should be < 5000.",
                        worstBiggestFlat, worstFlatSeed));
    }

    private int floodFill(boolean[][] isFlat, boolean[][] visited, int r, int c, int rows, int cols) {
        var stack = new java.util.ArrayDeque<int[]>();
        stack.push(new int[]{r, c});
        visited[r][c] = true;
        int count = 0;
        while (!stack.isEmpty()) {
            var p = stack.pop();
            count++;
            for (int[] d : new int[][]{{-1,0},{1,0},{0,-1},{0,1}}) {
                int nr = p[0]+d[0], nc = p[1]+d[1];
                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols
                        && isFlat[nr][nc] && !visited[nr][nc]) {
                    visited[nr][nc] = true;
                    stack.push(new int[]{nr, nc});
                }
            }
        }
        return count;
    }

    record PlateauResult(int aboveWaterCells, double maxElevation,
                          int plateauCells, double plateauBandCenter) {}

    private PlateauResult analyzePlateau(long seed) {
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var terrain = world.terrain();

        // Collect all above-water elevations
        int cols = terrain.getCols(), rows = terrain.getRows();
        int aboveWater = 0;
        double maxElev = 0;

        // Histogram: 1m bins from 0 to 2000m
        int[] bins = new int[2000];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                double e = terrain.elevationAtGrid(c, r);
                if (e > 0) {
                    aboveWater++;
                    maxElev = Math.max(maxElev, e);
                    int bin = Math.min((int) e, bins.length - 1);
                    bins[bin]++;
                }
            }
        }

        if (aboveWater == 0) {
            return new PlateauResult(0, 0, 0, 0);
        }

        // Find the densest 5m band (sliding window of 5 bins)
        // A plateau shows up as many cells in a narrow band
        int maxCount = 0;
        int maxBin = 0;
        for (int i = 0; i < bins.length - 5; i++) {
            int count = 0;
            for (int j = 0; j < 5; j++) count += bins[i + j];
            if (count > maxCount) {
                maxCount = count;
                maxBin = i;
            }
        }

        return new PlateauResult(aboveWater, maxElev, maxCount, maxBin + 2.5);
    }
}
