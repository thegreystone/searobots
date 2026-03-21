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
    private final double depthComfort;
    private final double depthPenalty;

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
        this(terrain, minFloorDepth, safetyMargin, gridStep, 300.0, 3.0);
    }

    /**
     * Creates a path planner with configurable depth preference.
     *
     * @param depthComfort  cells deeper than this (absolute meters) get minimum cost
     * @param depthPenalty  maximum cost multiplier for cells at the blocking threshold
     */
    public PathPlanner(TerrainMap terrain, double minFloorDepth, double safetyMargin,
                       double gridStep, double depthComfort, double depthPenalty) {
        this.terrain = terrain;
        this.minFloorDepth = minFloorDepth;
        this.safetyMargin = safetyMargin;
        this.gridStep = gridStep;
        this.depthComfort = depthComfort;
        this.depthPenalty = depthPenalty;

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
     * <p>The depth planning and corner smoothing use the physics-derived constraints
     * to ensure the route stays within the submarine's tractable envelope.
     *
     * @param startX           start world X
     * @param startY           start world Y
     * @param goalX            goal world X
     * @param goalY            goal world Y
     * @param operatingDepth   the Z depth for waypoints (e.g. -100); will be
     *                         adjusted upward if the floor is too shallow
     * @param depthChangeRatio max meters of depth change per meter of horizontal
     *                         travel (symmetric for rise and dive); derived from
     *                         physics characterization at the expected speed
     * @param turnRadius       minimum turn radius in meters at the expected speed
     * @return list of waypoints (may be empty if no path found)
     */
    public List<Vec3> findPath(double startX, double startY,
                                double goalX, double goalY,
                                double operatingDepth,
                                double depthChangeRatio,
                                double turnRadius) {
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
        return simplifyPath(gridPath, operatingDepth, depthChangeRatio, turnRadius);
    }

    /**
     * Finds a path using default depth/turn constraints (for backward compatibility).
     * Uses conservative values: depth ratio 1:10, turn radius 250m.
     */
    public List<Vec3> findPath(double startX, double startY,
                                double goalX, double goalY,
                                double operatingDepth) {
        return findPath(startX, startY, goalX, goalY, operatingDepth, 0.10, 250);
    }

    /**
     * A 2D corridor waypoint with terrain metadata. Used by the trajectory
     * projector to plan physically-feasible 3D routes through the corridor.
     */
    public record CorridorPoint(double x, double y, double floorElevation) {}

    /**
     * Finds a 2D corridor from start to goal through safe deep water.
     * Returns corridor points with terrain metadata but no depth assignment
     * or corner smoothing. The caller (trajectory projector) handles 3D
     * feasibility using the sub's kinematic state.
     */
    public List<CorridorPoint> findCorridor(double startX, double startY,
                                             double goalX, double goalY) {
        var gridPath = runAstar(startX, startY, goalX, goalY);
        if (gridPath == null) return List.of();

        // Line-of-sight simplification
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

        var result = new ArrayList<CorridorPoint>();
        for (var cell : simplified) {
            double wx = colToWorldX(cell[0]);
            double wy = rowToWorldY(cell[1]);
            double floor = terrain.elevationAt(wx, wy);
            result.add(new CorridorPoint(wx, wy, floor));
        }
        return result;
    }

    /** Runs A* and returns the raw grid path, or null if no path found. */
    private List<int[]> runAstar(double startX, double startY,
                                  double goalX, double goalY) {
        int sc = worldToCol(startX);
        int sr = worldToRow(startY);
        int gc = worldToCol(goalX);
        int gr = worldToRow(goalY);

        sc = Math.clamp(sc, 0, gridCols - 1);
        sr = Math.clamp(sr, 0, gridRows - 1);
        gc = Math.clamp(gc, 0, gridCols - 1);
        gr = Math.clamp(gr, 0, gridRows - 1);

        if (costGrid[sr * gridCols + sc] == 0) {
            int[] safe = findNearestSafe(sc, sr);
            if (safe == null) return null;
            sc = safe[0]; sr = safe[1];
        }
        if (costGrid[gr * gridCols + gc] == 0) {
            int[] safe = findNearestSafe(gc, gr);
            if (safe == null) return null;
            gc = safe[0]; gr = safe[1];
        }

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
        int maxExpansions = 50_000;

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
                if (cellCost == 0) continue;

                float tentativeG = currentG + dist[d] * cellCost;
                if (tentativeG < gScore[ni]) {
                    gScore[ni] = tentativeG;
                    cameFrom[ni] = ci;
                    float f = tentativeG + heuristic(nc, nr, gc, gr);
                    open.add(new long[]{(long) (f * 1000), ni});
                }
            }
        }

        if (cameFrom[goalIdx] == -1 && startIdx != goalIdx) {
            return null;
        }

        var gridPath = new ArrayList<int[]>();
        for (int idx = goalIdx; idx != -1; idx = cameFrom[idx]) {
            gridPath.add(new int[]{idx % gridCols, idx / gridCols});
        }
        java.util.Collections.reverse(gridPath);
        return gridPath;
    }

    /** Floor elevation at a world position. */
    public double floorAt(double worldX, double worldY) {
        return terrain.elevationAt(worldX, worldY);
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

        // Pass 1: base cost from floor depth (deeper = cheaper).
        // Cells deeper than depthComfort get minimum cost (1.0).
        // Shallower navigable cells ramp up to 1 + depthPenalty.
        for (int r = 0; r < gridRows; r++) {
            for (int c = 0; c < gridCols; c++) {
                double wx = colToWorldX(c);
                double wy = rowToWorldY(r);
                double floor = worstFloorNear(wx, wy, gridStep * 0.5);
                if (floor > minFloorDepth) {
                    grid[r * gridCols + c] = 0; // blocked
                } else {
                    double absFloor = Math.abs(floor);
                    double shallowFrac = Math.max(0, 1.0 - absFloor / depthComfort);
                    grid[r * gridCols + c] = (float) (1.0 + depthPenalty * shallowFrac * shallowFrac);
                }
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

        // Pass 3: add proximity penalty for cells near blocked terrain
        for (int i = 0; i < total; i++) {
            if (grid[i] == 0) continue;
            int d = distToBlocked[i];
            if (d <= marginCells) {
                float proximity = 1.0f - (float) d / (marginCells + 1);
                grid[i] += proximity * 4.0f;
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

    private List<Vec3> simplifyPath(List<int[]> gridPath, double operatingDepth,
                                     double depthChangeRatio, double turnRadius) {
        if (gridPath.size() <= 2) {
            // Short path: still apply depth lookahead by scanning terrain
            // between start and end for the worst floor
            var result = new ArrayList<Vec3>();
            double worstFloor = Double.NEGATIVE_INFINITY;
            for (var cell : gridPath) {
                double f = terrain.elevationAt(colToWorldX(cell[0]), rowToWorldY(cell[1]));
                if (f > worstFloor) worstFloor = f;
            }
            if (gridPath.size() == 2) {
                var c0 = gridPath.get(0);
                var c1 = gridPath.get(1);
                double x0 = colToWorldX(c0[0]), y0 = rowToWorldY(c0[1]);
                double x1 = colToWorldX(c1[0]), y1 = rowToWorldY(c1[1]);
                double len = Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));
                for (int s = 1; s < Math.max(2, (int) (len / 100)); s++) {
                    double t = (double) s / (int) (len / 100);
                    double f = terrain.elevationAt(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t);
                    if (f > worstFloor) worstFloor = f;
                }
            }
            double z = Math.min(Math.max(operatingDepth, worstFloor + 50), -20);
            for (var cell : gridPath) {
                result.add(new Vec3(colToWorldX(cell[0]), rowToWorldY(cell[1]), z));
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

        // Convert grid cells to world coordinates with proactive depth planning.
        // For each waypoint, scan ahead along the route to find the shallowest
        // floor within a lookahead distance. Set the depth to clear that floor
        // BEFORE arriving at it, so the sub is pre-positioned at the right depth.
        var worldPoints = new ArrayList<double[]>(); // [wx, wy, localFloor]
        for (var cell : simplified) {
            double wx = colToWorldX(cell[0]);
            double wy = rowToWorldY(cell[1]);
            double floor = terrain.elevationAt(wx, wy);
            worldPoints.add(new double[]{wx, wy, floor});
        }

        // Compute cumulative distances along the route
        double[] cumDist = new double[worldPoints.size()];
        cumDist[0] = 0;
        for (int i = 1; i < worldPoints.size(); i++) {
            double dx = worldPoints.get(i)[0] - worldPoints.get(i - 1)[0];
            double dy = worldPoints.get(i)[1] - worldPoints.get(i - 1)[1];
            cumDist[i] = cumDist[i - 1] + Math.sqrt(dx * dx + dy * dy);
        }

        // Depth lookahead: how far ahead to scan for terrain the sub must
        // clear. Derived from the maximum depth change the sub can achieve:
        // to rise 200m at the given ratio, it needs 200/ratio meters of
        // horizontal travel. Cap at 3000m to bound the scan.
        double depthLookahead = Math.min(3000, 200.0 / Math.max(depthChangeRatio, 0.01));
        double CLEARANCE = 75;      // must exceed EMERGENCY_GAP (60) + margin
        double[] targetDepths = new double[worldPoints.size()];
        for (int i = 0; i < worldPoints.size(); i++) {
            double worstFloor = worldPoints.get(i)[2];

            // Scan ahead along subsequent waypoints and legs
            for (int j = i; j < worldPoints.size() - 1; j++) {
                if (cumDist[j] - cumDist[i] > depthLookahead) break;

                // Sample terrain between waypoints j and j+1
                double[] from = worldPoints.get(j);
                double[] to = worldPoints.get(j + 1);
                double legLen = cumDist[j + 1] - cumDist[j];
                int samples = Math.max(2, (int) (legLen / 100));
                for (int s = 0; s <= samples; s++) {
                    double t = (double) s / samples;
                    double sx = from[0] + (to[0] - from[0]) * t;
                    double sy = from[1] + (to[1] - from[1]) * t;
                    double sampleDist = cumDist[j] + legLen * t - cumDist[i];
                    if (sampleDist > depthLookahead) break;
                    double f = terrain.elevationAt(sx, sy);
                    if (f > worstFloor) worstFloor = f;
                }
            }

            targetDepths[i] = Math.min(
                    Math.max(operatingDepth, worstFloor + CLEARANCE), -20);
        }

        // Depth rate limiting: symmetric (rise rate = dive rate) based on
        // physics-derived depth change ratio at the expected cruising speed.
        // Backward pass: ensure waypoints before a rise have started rising
        for (int i = worldPoints.size() - 2; i >= 0; i--) {
            double legDist = cumDist[i + 1] - cumDist[i];
            double maxChange = legDist * depthChangeRatio;
            if (targetDepths[i] < targetDepths[i + 1] - maxChange) {
                targetDepths[i] = targetDepths[i + 1] - maxChange;
            }
        }
        // Forward pass: limit dive rate
        for (int i = 1; i < worldPoints.size(); i++) {
            double legDist = cumDist[i] - cumDist[i - 1];
            double maxChange = legDist * depthChangeRatio;
            if (targetDepths[i] < targetDepths[i - 1] - maxChange) {
                targetDepths[i] = targetDepths[i - 1] - maxChange;
            }
        }

        var withDepths = new ArrayList<Vec3>();
        for (int i = 0; i < worldPoints.size(); i++) {
            withDepths.add(new Vec3(worldPoints.get(i)[0], worldPoints.get(i)[1],
                    targetDepths[i]));
        }

        // Corner smoothing: replace sharp corners with achievable arcs
        // using the physics-derived turn radius at the expected speed.
        return smoothCorners(withDepths, turnRadius);
    }

    private List<Vec3> smoothCorners(List<Vec3> path, double turnRadius) {
        if (path.size() < 3) return path;

        var result = new ArrayList<Vec3>();
        result.add(path.getFirst());

        for (int i = 1; i < path.size() - 1; i++) {
            var prev = path.get(i - 1);
            var curr = path.get(i);
            var next = path.get(i + 1);

            // Bearings of incoming and outgoing legs
            double inBearing = Math.atan2(curr.x() - prev.x(), curr.y() - prev.y());
            double outBearing = Math.atan2(next.x() - curr.x(), next.y() - curr.y());
            double turnAngle = outBearing - inBearing;
            while (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
            while (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

            double absTurn = Math.abs(turnAngle);
            if (absTurn < Math.toRadians(20)) {
                // Gentle turn: keep the waypoint as-is
                result.add(curr);
                continue;
            }

            // Sharp turn: compute tangent distance from corner
            // d = R * tan(angle/2), capped to half the shorter leg
            double tangentDist = turnRadius * Math.tan(Math.min(absTurn, Math.toRadians(150)) / 2);
            double legIn = curr.horizontalDistanceTo(prev);
            double legOut = curr.horizontalDistanceTo(next);
            tangentDist = Math.min(tangentDist, Math.min(legIn, legOut) * 0.4);
            tangentDist = Math.max(tangentDist, 50); // minimum offset

            // Entry tangent point: back along incoming leg
            double entryX = curr.x() - Math.sin(inBearing) * tangentDist;
            double entryY = curr.y() - Math.cos(inBearing) * tangentDist;
            // Exit tangent point: forward along outgoing leg
            double exitX = curr.x() + Math.sin(outBearing) * tangentDist;
            double exitY = curr.y() + Math.cos(outBearing) * tangentDist;

            // Depth: interpolate from incoming to outgoing
            double entryZ = (prev.z() + curr.z()) / 2;
            double exitZ = (curr.z() + next.z()) / 2;

            // Verify tangent points are in safe water
            boolean entrySafe = isSafe(entryX, entryY);
            boolean exitSafe = isSafe(exitX, exitY);

            if (entrySafe && exitSafe) {
                result.add(new Vec3(entryX, entryY, entryZ));
                result.add(new Vec3(exitX, exitY, exitZ));
            } else {
                // Can't smooth: keep original corner
                result.add(curr);
            }
        }

        result.add(path.getLast());
        return result;
    }

    private boolean lineOfSight(int[] from, int[] to) {
        // Bresenham-style walk checking all cells along the line.
        // Rejects lines through blocked cells, near-danger cells, AND cells
        // that are significantly shallower than the endpoints. This preserves
        // intermediate A* waypoints that route through deep water around ridges.
        float fromCost = costGrid[from[1] * gridCols + from[0]];
        float toCost = costGrid[to[1] * gridCols + to[0]];
        float maxEndCost = Math.max(fromCost, toCost);
        // Relative threshold: reject if a cell along the line is significantly
        // more expensive (shallower) than the endpoints. This preserves detours
        // that the A* found around underwater ridges.
        // Absolute threshold of 3.0 catches proximity to blocked terrain.
        float depthThreshold = maxEndCost + 0.3f;
        float proximityThreshold = 3.0f;

        int c0 = from[0], r0 = from[1];
        int c1 = to[0], r1 = to[1];
        int dc = Math.abs(c1 - c0), dr = Math.abs(r1 - r0);
        int sc = c0 < c1 ? 1 : -1, sr = r0 < r1 ? 1 : -1;
        int err = dc - dr;

        while (true) {
            if (c0 < 0 || c0 >= gridCols || r0 < 0 || r0 >= gridRows) return false;
            float cost = costGrid[r0 * gridCols + c0];
            if (cost == 0 || cost > depthThreshold || cost > proximityThreshold) return false;
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
