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
package se.hirt.searobots.viewer;

import com.jme3.math.Vector3f;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;

/**
 * Builds a jMonkeyEngine {@link Mesh} from a {@link TerrainMap} heightmap,
 * with downsampled vertices, smooth normals, and depth-based vertex colours.
 */
final class TerrainMeshBuilder {

    private TerrainMeshBuilder() {}

    /**
     * Build a terrain mesh from the given map.
     *
     * @param terrain    the heightmap
     * @param downsample skip factor (e.g. 4 means sample every 4th cell)
     * @return a jME Mesh ready to attach to a Geometry
     */
    static Mesh build(TerrainMap terrain, int downsample) {
        if (downsample < 1) downsample = 1;

        int cols = terrain.getCols();
        int rows = terrain.getRows();
        double cellSize = terrain.getCellSize();
        double originX = terrain.getOriginX();
        double originY = terrain.getOriginY();

        // Sampled grid dimensions
        int sampledCols = (cols - 1) / downsample + 1;
        int sampledRows = (rows - 1) / downsample + 1;
        int vertCount = sampledCols * sampledRows;

        // Buffers
        float[] positions = new float[vertCount * 3];
        float[] colors = new float[vertCount * 4];

        // Elevation range for colouring
        double minE = terrain.getMinElevation();
        double maxE = terrain.getMaxElevation();
        double rangeE = maxE - minE;
        if (rangeE == 0) rangeE = 1;

        // Fill vertex positions and colours
        for (int sr = 0; sr < sampledRows; sr++) {
            int row = Math.min(sr * downsample, rows - 1);
            for (int sc = 0; sc < sampledCols; sc++) {
                int col = Math.min(sc * downsample, cols - 1);
                int vi = sr * sampledCols + sc;

                double elev = terrain.elevationAtGrid(col, row);

                // Sim coords: X=east, Y=north, Z=up (elev is Z)
                // jME coords: X=right, Y=up, Z=towards camera
                float jmeX = (float) (originX + col * cellSize);
                float jmeY = (float) elev;
                float jmeZ = (float) -(originY + row * cellSize);

                positions[vi * 3] = jmeX;
                positions[vi * 3 + 1] = jmeY;
                positions[vi * 3 + 2] = jmeZ;

                // Vertex colour matching MapPanel.elevationColor()
                // with sand/beach blending near sea level
                float r, g, b;
                if (elev >= 0) {
                    double t = Math.min(elev / Math.max(1, maxE), 1.0);
                    r = (float) ((50 + t * 90) / 255.0);
                    g = (float) ((100 + t * 155) / 255.0);
                    b = (float) ((30 + t * 40) / 255.0);
                } else {
                    double t = (elev - minE) / rangeE;
                    r = (float) ((15 + t * 70) / 255.0);
                    g = (float) ((25 + t * 155) / 255.0);
                    b = (float) ((70 + t * 130) / 255.0);
                }

                // Shore blending: sand tone within 5m of sea level
                double shoreDist = Math.abs(elev);
                if (shoreDist < 5.0) {
                    float blend = (float) (1.0 - shoreDist / 5.0);
                    blend *= blend; // ease-in for smoother falloff
                    float sandR = 194f / 255f;
                    float sandG = 178f / 255f;
                    float sandB = 128f / 255f;
                    r = r + (sandR - r) * blend;
                    g = g + (sandG - g) * blend;
                    b = b + (sandB - b) * blend;
                }
                colors[vi * 4] = r;
                colors[vi * 4 + 1] = g;
                colors[vi * 4 + 2] = b;
                colors[vi * 4 + 3] = 1f;
            }
        }

        // Index buffer: two triangles per quad
        int quadCols = sampledCols - 1;
        int quadRows = sampledRows - 1;
        int triCount = quadCols * quadRows * 2;
        int[] indices = new int[triCount * 3];
        int idx = 0;
        for (int sr = 0; sr < quadRows; sr++) {
            for (int sc = 0; sc < quadCols; sc++) {
                int tl = sr * sampledCols + sc;
                int tr = tl + 1;
                int bl = (sr + 1) * sampledCols + sc;
                int br = bl + 1;

                // Triangle 1: tl - tr - bl (CCW from above, normal points up)
                indices[idx++] = tl;
                indices[idx++] = tr;
                indices[idx++] = bl;

                // Triangle 2: tr - br - bl (CCW from above, normal points up)
                indices[idx++] = tr;
                indices[idx++] = br;
                indices[idx++] = bl;
            }
        }

        // Smooth normals: accumulate face normals at each vertex, then normalize
        float[] normals = new float[vertCount * 3];
        Vector3f v0 = new Vector3f(), v1 = new Vector3f(), v2 = new Vector3f();
        Vector3f e1 = new Vector3f(), e2 = new Vector3f(), fn = new Vector3f();
        for (int t = 0; t < triCount; t++) {
            int i0 = indices[t * 3], i1 = indices[t * 3 + 1], i2 = indices[t * 3 + 2];
            v0.set(positions[i0 * 3], positions[i0 * 3 + 1], positions[i0 * 3 + 2]);
            v1.set(positions[i1 * 3], positions[i1 * 3 + 1], positions[i1 * 3 + 2]);
            v2.set(positions[i2 * 3], positions[i2 * 3 + 1], positions[i2 * 3 + 2]);
            v1.subtract(v0, e1);
            v2.subtract(v0, e2);
            e1.cross(e2, fn);
            for (int vi : new int[]{i0, i1, i2}) {
                normals[vi * 3] += fn.x;
                normals[vi * 3 + 1] += fn.y;
                normals[vi * 3 + 2] += fn.z;
            }
        }
        for (int i = 0; i < vertCount; i++) {
            float nx = normals[i * 3], ny = normals[i * 3 + 1], nz = normals[i * 3 + 2];
            float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0) {
                normals[i * 3] /= len;
                normals[i * 3 + 1] /= len;
                normals[i * 3 + 2] /= len;
            }
        }

        // Validate mesh integrity
        int nanCount = 0, infCount = 0, degenerateCount = 0, downNormalCount = 0;
        for (int i = 0; i < positions.length; i++) {
            if (Float.isNaN(positions[i])) nanCount++;
            if (Float.isInfinite(positions[i])) infCount++;
        }
        for (int t = 0; t < triCount; t++) {
            int i0 = indices[t * 3], i1 = indices[t * 3 + 1], i2 = indices[t * 3 + 2];
            v0.set(positions[i0 * 3], positions[i0 * 3 + 1], positions[i0 * 3 + 2]);
            v1.set(positions[i1 * 3], positions[i1 * 3 + 1], positions[i1 * 3 + 2]);
            v2.set(positions[i2 * 3], positions[i2 * 3 + 1], positions[i2 * 3 + 2]);
            v1.subtract(v0, e1);
            v2.subtract(v0, e2);
            e1.cross(e2, fn);
            float area = fn.length();
            if (area < 1e-6f) degenerateCount++;
            if (fn.y < 0) downNormalCount++;
        }
        System.out.printf("Terrain mesh: %d verts, %d tris, grid %dx%d (ds=%d from %dx%d)%n",
                vertCount, triCount, sampledCols, sampledRows, downsample, cols, rows);
        if (nanCount > 0 || infCount > 0 || degenerateCount > 0 || downNormalCount > 0) {
            System.out.printf("  WARNINGS: NaN=%d, Inf=%d, degenerate=%d, downward-normal=%d%n",
                    nanCount, infCount, degenerateCount, downNormalCount);
        } else {
            System.out.println("  Mesh integrity OK (no NaN/Inf/degenerate/downward normals)");
        }

        // Assemble mesh
        Mesh mesh = new Mesh();
        mesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(positions));
        mesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        mesh.setBuffer(VertexBuffer.Type.Color, 4, BufferUtils.createFloatBuffer(colors));
        mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(indices));
        mesh.updateBound();
        return mesh;
    }
}
