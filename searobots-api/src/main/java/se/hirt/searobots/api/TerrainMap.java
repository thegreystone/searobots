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

/**
 * Heightmap of the sea floor. Stores the Z elevation (negative values) of the
 * ocean bottom at each grid cell.
 * <p>
 * Grid cell (col, row) maps to world position
 * (originX + col * cellSize, originY + row * cellSize).
 * Row 0 is the southernmost row (lowest Y).
 */
public final class TerrainMap {

    private final double[] elevations;
    private final int cols;
    private final int rows;
    private final double originX;
    private final double originY;
    private final double cellSize;
    private final double minElevation;
    private final double maxElevation;

    public TerrainMap(double[] elevations, int cols, int rows,
                      double originX, double originY, double cellSize) {
        this.elevations = elevations.clone();
        this.cols = cols;
        this.rows = rows;
        this.originX = originX;
        this.originY = originY;
        this.cellSize = cellSize;

        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (double e : elevations) {
            if (e < min) min = e;
            if (e > max) max = e;
        }
        this.minElevation = min;
        this.maxElevation = max;
    }

    public double elevationAtGrid(int col, int row) {
        if (col < 0 || col >= cols || row < 0 || row >= rows) {
            return minElevation;
        }
        return elevations[row * cols + col];
    }

    /** Bilinear interpolation of sea floor elevation at world coordinates. */
    public double elevationAt(double worldX, double worldY) {
        double gx = (worldX - originX) / cellSize;
        double gy = (worldY - originY) / cellSize;

        int col = (int) Math.floor(gx);
        int row = (int) Math.floor(gy);

        double fx = gx - col;
        double fy = gy - row;

        double e00 = elevationAtGrid(col, row);
        double e10 = elevationAtGrid(col + 1, row);
        double e01 = elevationAtGrid(col, row + 1);
        double e11 = elevationAtGrid(col + 1, row + 1);

        double ex0 = e00 + (e10 - e00) * fx;
        double ex1 = e01 + (e11 - e01) * fx;
        return ex0 + (ex1 - ex0) * fy;
    }

    public int getCols() { return cols; }
    public int getRows() { return rows; }
    public double getOriginX() { return originX; }
    public double getOriginY() { return originY; }
    public double getCellSize() { return cellSize; }
    public double getMinElevation() { return minElevation; }
    public double getMaxElevation() { return maxElevation; }

    public double worldWidth() { return (cols - 1) * cellSize; }
    public double worldHeight() { return (rows - 1) * cellSize; }
}
