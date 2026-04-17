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
import com.jme3.texture.image.ColorSpace;
import com.jme3.renderer.Camera;
import com.jme3.terrain.geomipmap.TerrainLodControl;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.terrain.geomipmap.lodcalc.DistanceLodCalculator;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;

import java.nio.ByteBuffer;

/**
 * Builds a jMonkeyEngine {@link TerrainQuad} from a {@link TerrainMap},
 * with LOD, lit texture splatting via procedural alpha maps, normal maps,
 * and tri-planar texturing for steep slopes.
 *
 * <p>Five terrain layers are blended by elevation and slope:
 * <ol>
 *   <li>Deep ocean floor (dark sediment)</li>
 *   <li>Continental shelf (rocky)</li>
 *   <li>Beach / coastal sand</li>
 *   <li>Forest / vegetation</li>
 *   <li>Cliff rock (steep slopes, any elevation)</li>
 * </ol>
 *
 * <p>TerrainQuad requires a (2^n + 1) square heightmap. The simulation's
 * 1600x1600 grid is resampled to 1025x1025 with bilinear interpolation.
 */
final class TerrainQuadBuilder {

    /** Terrain quad size: 1025 = 2^10 + 1. */
    private static final int QUAD_SIZE = 1025;
    /** Patch size for LOD tiles: 65 = 2^6 + 1. */
    private static final int PATCH_SIZE = 65;

    private TerrainQuadBuilder() {}

    /**
     * Build a TerrainQuad from the simulation terrain map.
     *
     * @param terrain      the simulation heightmap
     * @param assetManager jME asset manager for loading textures
     * @param camera       camera for LOD control
     * @return a TerrainQuad positioned and scaled to match world coordinates
     */
    static TerrainQuad build(TerrainMap terrain, AssetManager assetManager, Camera camera) {
        float[] heightmap = resample(terrain);

        TerrainQuad quad = new TerrainQuad("terrain", PATCH_SIZE, QUAD_SIZE, heightmap);
        quad.setMaterial(createMaterial(terrain, assetManager, heightmap));

        // Scale: TerrainQuad spans [-size/2, +size/2] in local X and Z,
        // with (QUAD_SIZE - 1) cells. We need it to cover the world extent.
        double worldW = terrain.worldWidth();
        float scaleXZ = (float) (worldW / (QUAD_SIZE - 1));
        quad.setLocalScale(scaleXZ, 1f, scaleXZ);

        // Position: TerrainQuad is centred at origin. The sim terrain has
        // its origin at (originX, originY). We need to shift so the centre
        // of the quad matches the centre of the sim terrain.
        //
        // Sim coords: X=east, Y=north, Z=up
        // jME coords: X=east, Y=up, Z=-north (south)
        double worldH = terrain.worldHeight();
        float centreX = (float) (terrain.getOriginX() + worldW / 2);
        float centreZ = (float) -(terrain.getOriginY() + worldH / 2);
        quad.setLocalTranslation(centreX, 0, centreZ);

        // LOD control: reduce triangle count for distant patches.
        // High multiplier (5) keeps detail visible further from the camera,
        // preventing distant terrain from appearing flat.
        TerrainLodControl lodControl = new TerrainLodControl(quad, camera);
        lodControl.setLodCalculator(new DistanceLodCalculator(PATCH_SIZE, 5f));
        quad.addControl(lodControl);

        System.out.printf("TerrainQuad: %dx%d resampled from %dx%d, scale=%.2f, " +
                        "world=%.0fx%.0f, elev=[%.0f, %.0f]%n",
                QUAD_SIZE, QUAD_SIZE, terrain.getCols(), terrain.getRows(),
                scaleXZ, worldW, worldH,
                terrain.getMinElevation(), terrain.getMaxElevation());

        return quad;
    }

    /**
     * Resample the simulation terrain to a (QUAD_SIZE x QUAD_SIZE) float array
     * using bilinear interpolation via {@link TerrainMap#elevationAt}.
     */
    private static float[] resample(TerrainMap terrain) {
        float[] heightmap = new float[QUAD_SIZE * QUAD_SIZE];

        double worldW = terrain.worldWidth();
        double worldH = terrain.worldHeight();
        double originX = terrain.getOriginX();
        double originY = terrain.getOriginY();

        for (int row = 0; row < QUAD_SIZE; row++) {
            // TerrainQuad row 0 corresponds to jME -Z = sim +Y (north).
            // Map row 0..1024 to sim Y from north (originY + worldH) to south (originY).
            double simY = originY + worldH - row * (worldH / (QUAD_SIZE - 1));
            for (int col = 0; col < QUAD_SIZE; col++) {
                double simX = originX + col * (worldW / (QUAD_SIZE - 1));
                heightmap[row * QUAD_SIZE + col] = (float) terrain.elevationAt(simX, simY);
            }
        }

        return heightmap;
    }

    // ── Material and alpha maps ────────────────────────────────────────

    /**
     * Create a lit terrain material with 5 texture layers blended by
     * procedural alpha maps derived from elevation and slope.
     */
    private static Material createMaterial(TerrainMap terrain, AssetManager assetManager,
                                            float[] heightmap) {
        Material mat = new Material(assetManager,
                "Common/MatDefs/Terrain/TerrainLighting.j3md");

        // No specular: avoids washed-out highlights from strong sun
        mat.setFloat("Shininess", 0f);

        // Generate procedural alpha maps from elevation + slope
        Texture2D[] alphaMaps = generateAlphaMaps(heightmap,
                (float) terrain.worldWidth() / (QUAD_SIZE - 1));
        mat.setTexture("AlphaMap", alphaMaps[0]);
        mat.setTexture("AlphaMap_1", alphaMaps[1]);

        // Texture scale: TerrainQuad UVs go 0..1 across the terrain.
        // The scale multiplier controls how many times a texture repeats.
        // At 256x the terrain is ~17km, so each tile covers ~66m.

        // Layer 1 (AlphaMap.R): underwater seabed (all depths)
        // Single texture for all underwater terrain avoids blocky transitions.
        // The water filter and fog naturally darken deeper areas.
        mat.setTexture("DiffuseMap", loadWrap(assetManager,
                "Textures/Terrain/PBR/Gravel015_1K_Color.png"));
        mat.setTexture("NormalMap", loadWrap(assetManager,
                "Textures/Terrain/PBR/Gravel015_1K_Normal.png"));
        mat.setFloat("DiffuseMap_0_scale", 256f);

        // Layer 2 (AlphaMap.G): same as layer 1 (unified underwater)
        mat.setTexture("DiffuseMap_1", loadWrap(assetManager,
                "Textures/Terrain/PBR/Gravel015_1K_Color.png"));
        mat.setTexture("NormalMap_1", loadWrap(assetManager,
                "Textures/Terrain/PBR/Gravel015_1K_Normal.png"));
        mat.setFloat("DiffuseMap_1_scale", 256f);

        // Layer 3 (AlphaMap.B): beach sand
        mat.setTexture("DiffuseMap_2", loadWrap(assetManager,
                "Textures/Terrain/PBR/Ground037_1K_Color.png"));
        mat.setTexture("NormalMap_2", loadWrap(assetManager,
                "Textures/Terrain/PBR/Ground037_1K_Normal.png"));
        mat.setFloat("DiffuseMap_2_scale", 384f);

        // Layer 4 (AlphaMap.A): lush green vegetation
        mat.setTexture("DiffuseMap_3", loadWrap(assetManager,
                "Textures/Terrain/custom/vegetation.png"));
        mat.setTexture("NormalMap_3", loadWrap(assetManager,
                "Textures/Terrain/custom/vegetation_normal.png"));
        mat.setFloat("DiffuseMap_3_scale", 320f);

        // Layer 5 (AlphaMap_1.R): above-water cliff rock (gray gravel)
        mat.setTexture("DiffuseMap_4", loadWrap(assetManager,
                "Textures/Terrain/PBR/Gravel015_1K_Color.png"));
        mat.setTexture("NormalMap_4", loadWrap(assetManager,
                "Textures/Terrain/PBR/Gravel015_1K_Normal.png"));
        mat.setFloat("DiffuseMap_4_scale", 256f);

        return mat;
    }

    /**
     * Generate two RGBA alpha maps from the heightmap data. Each pixel's
     * channels encode the blend weight for one terrain layer:
     *
     * <pre>
     * AlphaMap[0]: R=deep sediment, G=shelf rock, B=sand, A=forest
     * AlphaMap[1]: R=cliff rock (steep slopes), G/B/A=unused
     * </pre>
     *
     * Weights are computed from elevation zones with smooth transitions,
     * overridden by slope angle for steep cliff faces.
     */
    private static Texture2D[] generateAlphaMaps(float[] heightmap, float cellSize) {
        ByteBuffer buf0 = BufferUtils.createByteBuffer(QUAD_SIZE * QUAD_SIZE * 4);
        ByteBuffer buf1 = BufferUtils.createByteBuffer(QUAD_SIZE * QUAD_SIZE * 4);

        for (int row = 0; row < QUAD_SIZE; row++) {
            // TerrainQuad UV (0,0) is at the bottom of the alpha map texture,
            // but our heightmap row 0 is the top (north). Flip vertically.
            int hRow = QUAD_SIZE - 1 - row;
            for (int col = 0; col < QUAD_SIZE; col++) {
                float elev = heightmap[hRow * QUAD_SIZE + col];

                // Slope from central finite differences
                int cl = Math.max(0, col - 1);
                int cr = Math.min(QUAD_SIZE - 1, col + 1);
                int ru = Math.max(0, hRow - 1);
                int rd = Math.min(QUAD_SIZE - 1, hRow + 1);
                float dzdx = (heightmap[hRow * QUAD_SIZE + cr] - heightmap[hRow * QUAD_SIZE + cl])
                        / ((cr - cl) * cellSize);
                float dzdy = (heightmap[rd * QUAD_SIZE + col] - heightmap[ru * QUAD_SIZE + col])
                        / ((rd - ru) * cellSize);
                float slopeDeg = (float) Math.toDegrees(
                        Math.atan(Math.sqrt(dzdx * dzdx + dzdy * dzdy)));

                // Elevation-based biome weights following real ocean depth zones:
                // < -200m: deep sediment (midnight zone, dark)
                // -200 to -18m: rocky shelf (twilight zone, no red light)
                // -18 to +3m: sandy shallows and beach (photic zone)
                // > +3m: vegetation/forest
                float wDeep = 0, wShelf = 0, wSand = 0, wForest = 0;

                if (elev < -300) {
                    wDeep = 1;
                } else if (elev < -150) {
                    float t = smoothstep(-300, -150, elev);
                    wDeep = 1 - t;
                    wShelf = t;
                } else if (elev < -18) {
                    wShelf = 1;
                } else if (elev < 3) {
                    // Sandy shallows (photic zone, -18m to +3m above)
                    float t = smoothstep(-18, -8, elev);
                    wShelf = 1 - t;
                    wSand = t;
                } else if (elev < 8) {
                    // Sand to forest transition
                    float t = smoothstep(3, 8, elev);
                    wSand = 1 - t;
                    wForest = t;
                } else {
                    wForest = 1;
                }

                // Sand only on flat ground (beaches and sandy shelves)
                if (slopeDeg > 12 && wSand > 0) {
                    float slopeReduce = smoothstep(12, 30, slopeDeg);
                    float removed = wSand * slopeReduce;
                    wSand -= removed;
                    if (elev < 0) wShelf += removed;
                    else wForest += removed;
                }

                // Underwater slopes: redistribute to shelf rock
                if (elev < 0 && slopeDeg > 15) {
                    float slopeToShelf = smoothstep(15, 35, slopeDeg);
                    float moved = wDeep * slopeToShelf;
                    wDeep -= moved;
                    wShelf += moved;
                }

                // Above water: cliff rock only on very steep faces (layer 5)
                // Vegetation clings to slopes up to ~50 degrees
                float wCliff = 0;
                if (elev > 0) {
                    wCliff = smoothstep(50, 70, slopeDeg);

                    // Mountain peaks: slight rock exposure above 100m
                    if (elev > 100) {
                        float peakExposure = smoothstep(100, 150, elev) * 0.3f;
                        wCliff = Math.max(wCliff, peakExposure);
                    }

                    float biomeScale = 1 - wCliff;
                    wSand *= biomeScale;
                    wForest *= biomeScale;
                }

                // jME RGBA8 ByteBuffer: R, G, B, A bytes per pixel.
                // AlphaMap channels: R->DiffuseMap, G->DiffuseMap_1, B->DiffuseMap_2, A->DiffuseMap_3
                // Ensure exactly one dominant weight to avoid washed-out blending.
                buf0.put((byte) Math.min(255, (int)(wDeep * 255)))
                    .put((byte) Math.min(255, (int)(wShelf * 255)))
                    .put((byte) Math.min(255, (int)(wSand * 255)))
                    .put((byte) Math.min(255, (int)(wForest * 255)));

                buf1.put((byte) Math.min(255, (int)(wCliff * 255)))
                    .put((byte) 0)
                    .put((byte) 0)
                    .put((byte) 0);
            }
        }
        buf0.flip();
        buf1.flip();

        Texture2D tex0 = bufferToTexture(buf0);
        Texture2D tex1 = bufferToTexture(buf1);
        return new Texture2D[]{tex0, tex1};
    }

    private static Texture2D bufferToTexture(ByteBuffer buf) {
        Image img = new Image(Image.Format.RGBA8, QUAD_SIZE, QUAD_SIZE, buf, ColorSpace.Linear);
        Texture2D tex = new Texture2D(img);
        tex.setMinFilter(Texture.MinFilter.BilinearNearestMipMap);
        tex.setMagFilter(Texture.MagFilter.Bilinear);
        return tex;
    }

    /** Hermite smoothstep: 0 at lo, 1 at hi, smooth S-curve between. */
    private static float smoothstep(float lo, float hi, float val) {
        float t = Math.max(0, Math.min(1, (val - lo) / (hi - lo)));
        return t * t * (3 - 2 * t);
    }

    private static Texture loadWrap(AssetManager am, String path) {
        Texture t = am.loadTexture(path);
        t.setWrap(Texture.WrapMode.Repeat);
        return t;
    }
}
