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

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;

import java.util.ArrayList;
import java.util.Random;

/**
 * Scatters billboard trees across vegetated terrain in natural-looking clusters.
 * Builds raw meshes directly (all quads for each tree type in a single mesh)
 * for maximum performance: 4 draw calls total regardless of tree count.
 */
final class TreeScatter {

    private static final int MAX_TREES = 500000;
    private static final float MIN_ELEVATION = 3f;
    private static final float MAX_SLOPE_DEG = 35f;
    private static final float BASE_SPACING = 4f;

    private TreeScatter() {}

    /** A tree to be placed: position + dimensions + rotation. */
    private record TreeInstance(float x, float y, float z, float w, float h, float rot) {}

    static Node create(TerrainMap terrain, TerrainQuad terrainQuad,
                        AssetManager assetManager, long seed) {
        Node treeNode = new Node("trees");
        var rng = new Random(seed);
        long startTime = System.currentTimeMillis();

        Material[] mats = {
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree1.png"),
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree2.png"),
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree3.png"),
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree4.png"),
        };
        float[][] heights = {{8, 15}, {10, 18}, {10, 20}, {3, 7}};
        float[] widthRatios = {0.7f, 0.35f, 0.5f, 0.9f};

        // Collect tree instances per type
        @SuppressWarnings("unchecked")
        var perType = new ArrayList[4];
        for (int i = 0; i < 4; i++) perType[i] = new ArrayList<TreeInstance>();

        double cellSize = terrain.getCellSize();
        double originX = terrain.getOriginX();
        double originY = terrain.getOriginY();

        int placed = 0;
        int gridCols = (int) (terrain.worldWidth() / BASE_SPACING);
        int gridRows = (int) (terrain.worldHeight() / BASE_SPACING);

        for (int gr = 0; gr < gridRows && placed < MAX_TREES; gr++) {
            for (int gc = 0; gc < gridCols && placed < MAX_TREES; gc++) {
                double wx = originX + (gc + rng.nextFloat()) * BASE_SPACING;
                double wy = originY + (gr + rng.nextFloat()) * BASE_SPACING;

                float elev = (float) terrain.elevationAt(wx, wy);
                if (elev < MIN_ELEVATION) continue;

                double e1 = terrain.elevationAt(wx + cellSize, wy);
                double e2 = terrain.elevationAt(wx, wy + cellSize);
                double dzdx = (e1 - elev) / cellSize;
                double dzdy = (e2 - elev) / cellSize;
                float slopeDeg = (float) Math.toDegrees(
                        Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy)));
                if (slopeDeg > MAX_SLOPE_DEG) continue;

                double density = noiseDensity(wx, wy, seed);
                if (rng.nextFloat() > density) continue;

                // Elevation-based type selection
                int type;
                if (elev < 15) {
                    type = rng.nextFloat() < 0.65f ? 2 : 3;
                } else if (elev < 50) {
                    float roll = rng.nextFloat();
                    if (roll < 0.40f) type = 0;
                    else if (roll < 0.75f) type = 1;
                    else if (roll < 0.85f) type = 2;
                    else type = 3;
                } else {
                    type = rng.nextFloat() < 0.7f ? (rng.nextBoolean() ? 0 : 1) : 3;
                }

                float height = heights[type][0] + rng.nextFloat() * (heights[type][1] - heights[type][0]);
                height *= 1f - Math.min(elev / 300f, 0.3f);
                float width = height * widthRatios[type] * (0.85f + rng.nextFloat() * 0.3f);
                float rotation = rng.nextFloat() * FastMath.PI;

                // Raycast for ground height
                float jmeX = (float) wx;
                float jmeZ = (float) -wy;
                var ray = new com.jme3.math.Ray(
                        new Vector3f(jmeX, 1000, jmeZ), new Vector3f(0, -1, 0));
                var hits = new com.jme3.collision.CollisionResults();
                terrainQuad.collideWith(ray, hits);
                float groundY = hits.size() > 0
                        ? hits.getClosestCollision().getContactPoint().y : elev;

                @SuppressWarnings("unchecked")
                var list = (ArrayList<TreeInstance>) perType[type];
                list.add(new TreeInstance(jmeX, groundY, jmeZ, width, height, rotation));
                placed++;
            }
        }

        // Build one mesh per tree type (all cross-billboards baked into vertices)
        for (int type = 0; type < 4; type++) {
            @SuppressWarnings("unchecked")
            var trees = (ArrayList<TreeInstance>) perType[type];
            if (trees.isEmpty()) continue;

            Mesh mesh = buildCrossBillboardMesh(trees);
            Geometry geom = new Geometry("trees_" + type, mesh);
            geom.setMaterial(mats[type]);
            geom.setQueueBucket(RenderQueue.Bucket.Transparent);
            treeNode.attachChild(geom);
        }

        long elapsed = System.currentTimeMillis() - startTime;
        System.out.printf("TreeScatter: placed %d trees in %dms (4 draw calls, direct mesh)%n",
                placed, elapsed);
        return treeNode;
    }

    /**
     * Build a single mesh containing all cross-billboard quads.
     * Each tree = 2 quads (4 triangles, 8 vertices).
     */
    private static Mesh buildCrossBillboardMesh(ArrayList<TreeInstance> trees) {
        int treeCount = trees.size();
        int vertCount = treeCount * 8;   // 4 verts per quad, 2 quads per tree
        int triCount = treeCount * 4;    // 2 triangles per quad, 2 quads per tree

        float[] positions = new float[vertCount * 3];
        float[] texCoords = new float[vertCount * 2];
        float[] normals = new float[vertCount * 3];
        int[] indices = new int[triCount * 3];

        int vi = 0, ii = 0;
        for (var tree : trees) {
            float cos = FastMath.cos(tree.rot);
            float sin = FastMath.sin(tree.rot);

            // Quad 1: aligned with rotation direction
            float hw = tree.w / 2;
            // Four corners: bottom-left, bottom-right, top-right, top-left
            float[] dx1 = {-hw, hw, hw, -hw};
            float[] dz1 = {0, 0, 0, 0};
            float[] dy1 = {0, 0, tree.h, tree.h};
            addQuad(positions, texCoords, normals, indices, vi, ii,
                    tree.x, tree.y, tree.z, dx1, dy1, dz1, cos, sin);
            vi += 4; ii += 6;

            // Quad 2: perpendicular (rotated 90 degrees)
            float cos2 = FastMath.cos(tree.rot + FastMath.HALF_PI);
            float sin2 = FastMath.sin(tree.rot + FastMath.HALF_PI);
            addQuad(positions, texCoords, normals, indices, vi, ii,
                    tree.x, tree.y, tree.z, dx1, dy1, dz1, cos2, sin2);
            vi += 4; ii += 6;
        }

        Mesh mesh = new Mesh();
        mesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(positions));
        mesh.setBuffer(VertexBuffer.Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoords));
        mesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
        mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(indices));
        mesh.updateBound();
        return mesh;
    }

    private static void addQuad(float[] pos, float[] tc, float[] norm, int[] idx,
                                 int vi, int ii,
                                 float ox, float oy, float oz,
                                 float[] dx, float[] dy, float[] dz,
                                 float cos, float sin) {
        // UV coordinates for quad corners
        float[] u = {0, 1, 1, 0};
        float[] v = {0, 0, 1, 1};

        for (int i = 0; i < 4; i++) {
            int pi = (vi + i) * 3;
            // Rotate dx,dz around Y axis
            float rx = dx[i] * cos - dz[i] * sin;
            float rz = dx[i] * sin + dz[i] * cos;
            pos[pi] = ox + rx;
            pos[pi + 1] = oy + dy[i];
            pos[pi + 2] = oz + rz;

            int ti = (vi + i) * 2;
            tc[ti] = u[i];
            tc[ti + 1] = v[i];

            // Normal facing outward from the quad
            norm[pi] = sin;
            norm[pi + 1] = 0;
            norm[pi + 2] = cos;
        }

        // Two triangles: 0-1-2, 0-2-3
        idx[ii] = vi;
        idx[ii + 1] = vi + 1;
        idx[ii + 2] = vi + 2;
        idx[ii + 3] = vi;
        idx[ii + 4] = vi + 2;
        idx[ii + 5] = vi + 3;
    }

    private static Material createTreeMaterial(AssetManager am, String texturePath) {
        Material mat = new Material(am, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setTexture("ColorMap", am.loadTexture(texturePath));
        mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
        mat.setFloat("AlphaDiscardThreshold", 0.5f);
        return mat;
    }

    private static double noiseDensity(double wx, double wy, long seed) {
        double n1 = Math.sin(wx * 0.007 + seed * 0.001) * Math.cos(wy * 0.008 + seed * 0.002);
        double n2 = Math.sin(wx * 0.025 + wy * 0.02 + seed * 0.003) * 0.4;
        double n3 = Math.sin(wx * 0.06 + wy * 0.05) * 0.15;
        double density = 0.8 + n1 * 0.18 + n2 * 0.10 + n3 * 0.08;
        return Math.max(0, Math.min(1, density));
    }
}
