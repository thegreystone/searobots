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
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Quad;
import com.jme3.terrain.geomipmap.TerrainQuad;
import se.hirt.searobots.api.TerrainMap;

import java.util.Random;

/**
 * Scatters billboard trees across vegetated terrain in natural-looking clusters.
 * Uses cross-billboards (two quads at 90 degrees, 4 triangles per tree) for
 * efficient rendering of tens of thousands of trees without LOD overhead.
 */
final class TreeScatter {

    private static final int MAX_TREES = 60000;
    private static final float MIN_ELEVATION = 3f;
    private static final float MAX_SLOPE_DEG = 35f;
    private static final float BASE_SPACING = 10f;

    private TreeScatter() {}

    /**
     * Create a node containing scattered billboard trees across the terrain.
     */
    static Node create(TerrainMap terrain, TerrainQuad terrainQuad,
                        AssetManager assetManager, long seed) {
        Node treeNode = new Node("trees");
        var rng = new Random(seed);

        // Four tree types: broadleaf, conifer, palm, bush
        Material[] mats = {
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree1.png"),
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree2.png"),
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree3.png"),
            createTreeMaterial(assetManager, "Textures/Terrain/custom/tree4.png"),
        };
        // Height ranges per type: {min, max} in metres (island-scale trees)
        float[][] heights = {
            {8, 15},   // broadleaf
            {10, 18},  // conifer
            {10, 20},  // palm
            {3, 7},    // bush
        };
        // Width-to-height ratios per type
        float[] widthRatios = {0.7f, 0.35f, 0.5f, 0.9f};
        // Relative frequency weights
        float[] weights = {0.35f, 0.15f, 0.20f, 0.30f};

        double cellSize = terrain.getCellSize();
        double originX = terrain.getOriginX();
        double originY = terrain.getOriginY();

        int placed = 0;
        int gridCols = (int) (terrain.worldWidth() / BASE_SPACING);
        int gridRows = (int) (terrain.worldHeight() / BASE_SPACING);

        // Pre-create shared quad meshes for two size classes
        Quad quadSmall = new Quad(1, 1); // unit quad, scaled per instance
        Quad quadLarge = new Quad(1, 1);

        for (int gr = 0; gr < gridRows && placed < MAX_TREES; gr++) {
            for (int gc = 0; gc < gridCols && placed < MAX_TREES; gc++) {
                double wx = originX + (gc + rng.nextFloat()) * BASE_SPACING;
                double wy = originY + (gr + rng.nextFloat()) * BASE_SPACING;

                float elev = (float) terrain.elevationAt(wx, wy);
                if (elev < MIN_ELEVATION) continue;

                // Slope check
                double e1 = terrain.elevationAt(wx + cellSize, wy);
                double e2 = terrain.elevationAt(wx, wy + cellSize);
                double dzdx = (e1 - elev) / cellSize;
                double dzdy = (e2 - elev) / cellSize;
                float slopeDeg = (float) Math.toDegrees(
                        Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy)));
                if (slopeDeg > MAX_SLOPE_DEG) continue;

                // Noise-based density for natural clumps
                double density = noiseDensity(wx, wy, seed);
                if (rng.nextFloat() > density) continue;

                // Pick tree type by elevation zone (ecological realism)
                // Coastal (<15m): palms + bushes
                // Mid (15-50m): mixed broadleaf forest
                // High (>50m): dense broadleaf, some understory
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
                Material mat = mats[type];

                // Random rotation for variety
                float rotation = rng.nextFloat() * FastMath.PI;

                // Build cross-billboard
                Node tree = new Node();

                Geometry g1 = new Geometry("t", quadSmall);
                g1.setMaterial(mat);
                g1.setQueueBucket(RenderQueue.Bucket.Transparent);
                g1.setLocalScale(width, height, 1);
                g1.setLocalTranslation(-width / 2, 0, 0);
                tree.attachChild(g1);

                Geometry g2 = new Geometry("t", quadLarge);
                g2.setMaterial(mat);
                g2.setQueueBucket(RenderQueue.Bucket.Transparent);
                g2.setLocalScale(width, height, 1);
                g2.setLocalTranslation(0, 0, width / 2);
                g2.setLocalRotation(new Quaternion().fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_Y));
                tree.attachChild(g2);

                tree.setLocalRotation(new Quaternion().fromAngleAxis(rotation, Vector3f.UNIT_Y));

                // Raycast down to find exact rendered terrain surface
                float jmeX = (float) wx;
                float jmeZ = (float) -wy;
                var ray = new com.jme3.math.Ray(
                        new Vector3f(jmeX, 1000, jmeZ), new Vector3f(0, -1, 0));
                var hits = new com.jme3.collision.CollisionResults();
                terrainQuad.collideWith(ray, hits);
                float groundY = hits.size() > 0
                        ? hits.getClosestCollision().getContactPoint().y : elev;
                tree.setLocalTranslation(jmeX, groundY, jmeZ);

                treeNode.attachChild(tree);
                placed++;
            }
        }

        System.out.printf("TreeScatter: placed %d billboard trees%n", placed);
        return treeNode;
    }

    private static Material createTreeMaterial(AssetManager am, String texturePath) {
        Material mat = new Material(am, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setTexture("ColorMap", am.loadTexture(texturePath));
        mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
        mat.setFloat("AlphaDiscardThreshold", 0.5f);
        return mat;
    }

    /** Pick a random index using weighted probabilities. */
    private static int pickWeighted(Random rng, float[] weights) {
        float r = rng.nextFloat();
        float sum = 0;
        for (int i = 0; i < weights.length; i++) {
            sum += weights[i];
            if (r < sum) return i;
        }
        return weights.length - 1;
    }

    /**
     * Noise-based tree density. Returns 0 (no trees) to 1 (dense forest).
     * Creates organic clumps at ~150m scale with finer variation.
     */
    private static double noiseDensity(double wx, double wy, long seed) {
        double n1 = Math.sin(wx * 0.007 + seed * 0.001) * Math.cos(wy * 0.008 + seed * 0.002);
        double n2 = Math.sin(wx * 0.025 + wy * 0.02 + seed * 0.003) * 0.4;
        double n3 = Math.sin(wx * 0.06 + wy * 0.05) * 0.15; // fine detail
        double density = 0.7 + n1 * 0.25 + n2 * 0.12 + n3;
        return Math.max(0, Math.min(1, density));
    }
}
