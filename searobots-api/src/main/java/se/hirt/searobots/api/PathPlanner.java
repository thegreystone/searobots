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
package se.hirt.searobots.api;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

/**
 * A* path planner for submarine navigation. Builds a coarse navigation
 * grid from the terrain map where cells shallower than a configurable
 * depth limit are blocked. Routes through safe deep water, with a cost
 * penalty for cells near shallow terrain to keep the path away from danger.
 *
 * <p>Intended as a shared utility that any {@link SubmarineController}
 * can use for route planning.
 *
 * <p>Usage:
 * <pre>
 *   var planner = new PathPlanner(terrain, -80, 200, 50);
 *   List&lt;Vec3&gt; path = planner.findPath(startX, startY, goalX, goalY, -100);
 * </pre>
 */
public final class PathPlanner {

    private final TerrainMap terrain;
    private final double minFloorDepth;
    private final double safetyMargin;
    private final double gridStep;

    // Navigation grid: cost multiplier per cell (0 = blocked, 1 = clear, >1 = near danger)
    private final float[] costGrid;
    private final int gridCols;
    private final int gridRows;
    private final double gridOriginX;
    private final double gridOriginY;

    /**
     * Creates a path planner for the given terrain.
     *
     * @param terrain        the sea floor heightmap
     * @param minFloorDepth  floor elevation above which cells are blocked (e.g. -80)
     * @param safetyMargin   distance in meters to keep from blocked cells;
     *                       cells within this distance have increased traversal cost
     * @param gridStep       navigation grid resolution in meters (e.g. 50-100)
     */
    public PathPlanner(TerrainMap terrain, double minFloorDepth, double safetyMargin, double gridStep) {
        this.terrain = terrain;
        this.minFloorDepth = minFloorDepth;
        this.safetyMargin = safetyMargin;
        this.gridStep = gridStep;

        // Build coarse grid covering the terrain extent
        double totalW = terrain.worldWidth();
        double totalH = terrain.worldHeight();
        this.gridOriginX = terrain.getOriginX();
        this.gridOriginY = terrain.getOriginY();
        this.gridCols = (int) Math.ceil(totalW / gridStep) + 1;
        this.gridRows = (int) Math.ceil(totalH / gridStep) + 1;
        this.costGrid = buildCostGrid();
    }

    /**
     * Finds a path from (startX, startY) to (goalX, goalY) through safe water.
     * Returns a list of world-coordinate waypoints with the specified operating
     * depth, or an empty list if no safe path exists.
     *
     * @param startX         start world X
     * @param startY         start world Y
     * @param goalX          goal world X
     * @param goalY          goal world Y
     * @param operatingDepth the Z depth for waypoints (e.g. -100); will be
     *                       adjusted upward if the floor is too shallow
     * @return list of waypoints (may be empty if no path found)
     */
    public List<Vec3> findPath(double startX, double startY,
                                double goalX, double goalY,
                                double operatingDepth) {
        int sc = worldToCol(startX);
        int sr = worldToRow(startY);
        int gc = worldToCol(goalX);
        int gr = worldToRow(goalY);

        // Clamp to grid bounds
        sc = Math.clamp(sc, 0, gridCols - 1);
        sr = Math.clamp(sr, 0, gridRows - 1);
        gc = Math.clamp(gc, 0, gridCols - 1);
        gr = Math.clamp(gr, 0, gridRows - 1);

        // If start or goal is blocked, snap to nearest safe cell
        if (costGrid[sr * gridCols + sc] == 0) {
            int[] safe = findNearestSafe(sc, sr);
            if (safe == null) return List.of();
            sc = safe[0]; sr = safe[1];
        }
        if (costGrid[gr * gridCols + gc] == 0) {
            int[] safe = findNearestSafe(gc, gr);
            if (safe == null) return List.of();
            gc = safe[0]; gr = safe[1];
        }

        // A* search
        int startIdx = sr * gridCols + sc;
        int goalIdx = gr * gridCols + gc;
        int totalCells = gridCols * gridRows;

        float[] gScore = new float[totalCells];
        int[] cameFrom = new int[totalCells];
        boolean[] closed = new boolean[totalCells];
        java.util.Arrays.fill(gScore, Float.MAX_VALUE);
        java.util.Arrays.fill(cameFrom, -1);
        gScore[startIdx] = 0;

        var open = new PriorityQueue<long[]>(Comparator.comparingLong(a -> a[0]));
        open.add(new long[]{(long) (heuristic(sc, sr, gc, gr) * 1000), startIdx});
        int maxExpansions = 50_000; // limit search to avoid pathological cases

        // 8-directional neighbors
        int[] dc = {-1, 0, 1, -1, 1, -1, 0, 1};
        int[] dr = {-1, -1, -1, 0, 0, 1, 1, 1};
        float[] dist = {1.414f, 1, 1.414f, 1, 1, 1.414f, 1, 1.414f};

        while (!open.isEmpty() && maxExpansions-- > 0) {
            long[] current = open.poll();
            int ci = (int) current[1];
            if (ci == goalIdx) break;
            if (closed[ci]) continue;
            closed[ci] = true;

            int cc = ci % gridCols;
            int cr = ci / gridCols;
            float currentG = gScore[ci];

            for (int d = 0; d < 8; d++) {
                int nc = cc + dc[d];
                int nr = cr + dr[d];
                if (nc < 0 || nc >= gridCols || nr < 0 || nr >= gridRows) continue;

                int ni = nr * gridCols + nc;
                float cellCost = costGrid[ni];
                if (cellCost == 0) continue; // blocked

                float tentativeG = currentG + dist[d] * cellCost;
                if (tentativeG < gScore[ni]) {
                    gScore[ni] = tentativeG;
                    cameFrom[ni] = ci;
                    float f = tentativeG + heuristic(nc, nr, gc, gr);
                    open.add(new long[]{(long) (f * 1000), ni});
                }
            }
        }

        // Reconstruct path
        if (cameFrom[goalIdx] == -1 && startIdx != goalIdx) {
            return List.of(); // no path found
        }

        var gridPath = new ArrayList<int[]>();
        for (int idx = goalIdx; idx != -1; idx = cameFrom[idx]) {
            gridPath.add(new int[]{idx % gridCols, idx / gridCols});
        }
        // Reverse to get start-to-goal order
        java.util.Collections.reverse(gridPath);

        // Convert to world coordinates and simplify
        return simplifyPath(gridPath, operatingDepth);
    }

    /**
     * Returns true if the given world position is in safe (navigable) water.
     */
    public boolean isSafe(double worldX, double worldY) {
        int c = worldToCol(worldX);
        int r = worldToRow(worldY);
        if (c < 0 || c >= gridCols || r < 0 || r >= gridRows) return false;
        return costGrid[r * gridCols + c] > 0;
    }

    /**
     * Returns the traversal cost at the given world position.
     * 0 = blocked, 1 = clear open water, >1 = near shallow terrain.
     */
    public float costAt(double worldX, double worldY) {
        int c = worldToCol(worldX);
        int r = worldToRow(worldY);
        if (c < 0 || c >= gridCols || r < 0 || r >= gridRows) return 0;
        return costGrid[r * gridCols + c];
    }

    // ── internals ──

    private float[] buildCostGrid() {
        int total = gridCols * gridRows;
        float[] grid = new float[total];
        int marginCells = Math.max(1, (int) Math.ceil(safetyMargin / gridStep));

        // Pass 1: mark cells as blocked (0) or clear (1)
        for (int r = 0; r < gridRows; r++) {
            for (int c = 0; c < gridCols; c++) {
                double wx = colToWorldX(c);
                double wy = rowToWorldY(r);
                double floor = worstFloorNear(wx, wy, gridStep * 0.5);
                grid[r * gridCols + c] = (floor > minFloorDepth) ? 0 : 1;
            }
        }

        // Pass 2: BFS distance transform from blocked cells
        int[] distToBlocked = new int[total];
        java.util.Arrays.fill(distToBlocked, Integer.MAX_VALUE);
        var queue = new java.util.ArrayDeque<Integer>();
        for (int i = 0; i < total; i++) {
            if (grid[i] == 0) {
                distToBlocked[i] = 0;
                queue.add(i);
            }
        }
        int[] dc = {-1, 0, 1, -1, 1, -1, 0, 1};
        int[] dr = {-1, -1, -1, 0, 0, 1, 1, 1};
        while (!queue.isEmpty()) {
            int idx = queue.poll();
            int cc = idx % gridCols, cr = idx / gridCols;
            int curDist = distToBlocked[idx];
            if (curDist >= marginCells) continue;
            for (int d = 0; d < 8; d++) {
                int nc = cc + dc[d], nr = cr + dr[d];
                if (nc < 0 || nc >= gridCols || nr < 0 || nr >= gridRows) continue;
                int ni = nr * gridCols + nc;
                if (distToBlocked[ni] > curDist + 1) {
                    distToBlocked[ni] = curDist + 1;
                    queue.add(ni);
                }
            }
        }

        // Apply cost: cells near blocked terrain are more expensive to traverse
        for (int i = 0; i < total; i++) {
            if (grid[i] == 0) continue;
            int d = distToBlocked[i];
            if (d <= marginCells) {
                float proximity = 1.0f - (float) d / (marginCells + 1);
                grid[i] = 1.0f + proximity * 4.0f;
            }
        }
        return grid;
    }

    private double worstFloorNear(double wx, double wy, double radius) {
        double worst = terrain.elevationAt(wx, wy);
        for (int deg = 0; deg < 360; deg += 90) {
            double brg = Math.toRadians(deg);
            double f = terrain.elevationAt(wx + Math.sin(brg) * radius,
                                           wy + Math.cos(brg) * radius);
            if (f > worst) worst = f;
        }
        return worst;
    }

    private int[] findNearestSafe(int col, int row) {
        for (int radius = 1; radius < Math.max(gridCols, gridRows); radius++) {
            // At each radius, find the safe cell with the lowest cost (deepest water)
            int[] best = null;
            float bestCost = Float.MAX_VALUE;
            for (int dc = -radius; dc <= radius; dc++) {
                for (int dr = -radius; dr <= radius; dr++) {
                    if (Math.abs(dc) != radius && Math.abs(dr) != radius) continue;
                    int nc = col + dc, nr = row + dr;
                    if (nc < 0 || nc >= gridCols || nr < 0 || nr >= gridRows) continue;
                    float c = costGrid[nr * gridCols + nc];
                    if (c > 0 && c < bestCost) {
                        bestCost = c;
                        best = new int[]{nc, nr};
                    }
                }
            }
            if (best != null) return best;
        }
        return null;
    }

    private List<Vec3> simplifyPath(List<int[]> gridPath, double operatingDepth) {
        if (gridPath.size() <= 2) {
            var result = new ArrayList<Vec3>();
            for (var cell : gridPath) {
                double wx = colToWorldX(cell[0]);
                double wy = rowToWorldY(cell[1]);
                double floor = terrain.elevationAt(wx, wy);
                double z = Math.max(operatingDepth, floor + 50);
                result.add(new Vec3(wx, wy, Math.min(z, -20)));
            }
            return result;
        }

        // Line-of-sight simplification: skip intermediate waypoints
        // if the straight line between two points stays in safe cells
        var simplified = new ArrayList<int[]>();
        simplified.add(gridPath.getFirst());
        int anchor = 0;

        for (int i = 2; i < gridPath.size(); i++) {
            if (!lineOfSight(gridPath.get(anchor), gridPath.get(i))) {
                simplified.add(gridPath.get(i - 1));
                anchor = i - 1;
            }
        }
        simplified.add(gridPath.getLast());

        var result = new ArrayList<Vec3>();
        for (var cell : simplified) {
            double wx = colToWorldX(cell[0]);
            double wy = rowToWorldY(cell[1]);
            double floor = terrain.elevationAt(wx, wy);
            double z = Math.max(operatingDepth, floor + 50);
            result.add(new Vec3(wx, wy, Math.min(z, -20)));
        }
        return result;
    }

    private boolean lineOfSight(int[] from, int[] to) {
        // Bresenham-style walk checking all cells along the line
        int c0 = from[0], r0 = from[1];
        int c1 = to[0], r1 = to[1];
        int dc = Math.abs(c1 - c0), dr = Math.abs(r1 - r0);
        int sc = c0 < c1 ? 1 : -1, sr = r0 < r1 ? 1 : -1;
        int err = dc - dr;

        while (true) {
            if (c0 < 0 || c0 >= gridCols || r0 < 0 || r0 >= gridRows) return false;
            if (costGrid[r0 * gridCols + c0] == 0) return false;
            if (c0 == c1 && r0 == r1) break;
            int e2 = 2 * err;
            if (e2 > -dr) { err -= dr; c0 += sc; }
            if (e2 < dc) { err += dc; r0 += sr; }
        }
        return true;
    }

    private float heuristic(int c1, int r1, int c2, int r2) {
        int dc = Math.abs(c1 - c2), dr = Math.abs(r1 - r2);
        return Math.max(dc, dr) + 0.414f * Math.min(dc, dr); // octile distance
    }

    private int worldToCol(double wx) { return (int) Math.round((wx - gridOriginX) / gridStep); }
    private int worldToRow(double wy) { return (int) Math.round((wy - gridOriginY) / gridStep); }
    private double colToWorldX(int c) { return gridOriginX + c * gridStep; }
    private double rowToWorldY(int r) { return gridOriginY + r * gridStep; }
}
