/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.viewer.tools;


import com.jme3.app.SimpleApplication;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.*;
import com.jme3.post.FilterPostProcessor;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Quad;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture;
import com.jme3.water.WaterFilter;

/**
 * Tiny test island for evaluating tree billboard appearance and placement.
 * Uses the same materials and lighting as the full viewer. A small dome-shaped
 * island surrounded by water, with clusters of each tree type.
 *
 * <p>Launch: {@code java -cp searobots-viewer/target/searobots-viewer-*-SNAPSHOT.jar
 * se.hirt.searobots.viewer.TreeTestScene}
 */
public class TreeTestScene extends SimpleApplication {

    private float orbitAzimuth = 0;
    private float orbitElevation = 0.35f;
    private float orbitDistance = 120f;
    private final Vector3f orbitCenter = new Vector3f(0, 15, 0);

    public static void main(String[] args) {
        var app = new TreeTestScene();
        var settings = new AppSettings(true);
        settings.setTitle("Tree Test Scene");
        settings.setWidth(1280);
        settings.setHeight(720);
        settings.setVSync(true);
        app.setSettings(settings);
        app.setShowSettings(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {
        setDisplayStatView(false);
        setDisplayFps(true);
        flyCam.setEnabled(false);
        viewPort.setBackgroundColor(new ColorRGBA(0.4f, 0.6f, 0.8f, 1f));

        // Lighting (same as main viewer)
        var sun = new DirectionalLight();
        sun.setDirection(new Vector3f(-1, -1, -1).normalizeLocal());
        sun.setColor(ColorRGBA.White.mult(1.2f));
        rootNode.addLight(sun);

        var fill = new DirectionalLight();
        fill.setDirection(new Vector3f(1, 0.5f, 1).normalizeLocal());
        fill.setColor(new ColorRGBA(0.3f, 0.4f, 0.5f, 1f));
        rootNode.addLight(fill);

        var ambient = new AmbientLight();
        ambient.setColor(new ColorRGBA(0.35f, 0.4f, 0.45f, 1f));
        rootNode.addLight(ambient);

        // Water
        var fpp = new FilterPostProcessor(assetManager);
        var water = new WaterFilter(rootNode, sun.getDirection().mult(-1));
        water.setWaterHeight(0f);
        water.setSpeed(0.8f);
        water.setWaveScale(0.003f);
        water.setMaxAmplitude(0.5f);
        water.setWaterColor(new ColorRGBA(0.0f, 0.18f, 0.60f, 1f));
        water.setDeepWaterColor(new ColorRGBA(0.0f, 0.09f, 0.42f, 1f));
        water.setWaterTransparency(0.11f);
        water.setSunScale(3f);
        water.setUseRipples(true);
        water.setUseSpecular(true);
        water.setUseRefraction(true);
        fpp.addFilter(water);
        viewPort.addProcessor(fpp);

        // Tiny island terrain (129x129 vertices, ~380m across at 3m/cell)
        int size = 129;
        float cellScale = 3f;
        float[] heightmap = createTestIsland(size);
        var tq = new TerrainQuad("island", 65, size, heightmap);
        tq.setMaterial(createIslandMaterial());
        tq.setLocalScale(cellScale, 1f, cellScale);
        rootNode.attachChild(tq);

        // Place tree clusters
        placeTreeGroups(tq);

        // Screenshot support (press F1)
        stateManager.attach(new ScreenshotAppState("", "TreeTest"));
        cam.setFrustumFar(2000f);
        System.out.println("Tree test scene ready. Auto-orbiting around island.");
    }

    private ScreenshotAppState ssState;
    private int frameCount;
    private int shotsTaken;

    @Override
    public void simpleUpdate(float tpf) {
        frameCount++;
        // Auto-capture 4 screenshots at different angles
        if (ssState == null) ssState = stateManager.getState(ScreenshotAppState.class);
        if (ssState != null && shotsTaken < 4) {
            if (frameCount == 60 || frameCount == 160 || frameCount == 260 || frameCount == 360) {
                ssState.takeScreenshot();
                shotsTaken++;
                System.out.println("TreeTest screenshot " + shotsTaken + "/4");
            }
        }
        orbitAzimuth += tpf * 0.12f;
        float x = orbitCenter.x + orbitDistance * FastMath.cos(orbitElevation) * FastMath.cos(orbitAzimuth);
        float y = orbitCenter.y + orbitDistance * FastMath.sin(orbitElevation);
        float z = orbitCenter.z + orbitDistance * FastMath.cos(orbitElevation) * FastMath.sin(orbitAzimuth);
        cam.setLocation(new Vector3f(x, y, z));
        cam.lookAt(orbitCenter, Vector3f.UNIT_Y);
    }

    /** Dome-shaped island with terrain noise, surrounded by shallow water. */
    private float[] createTestIsland(int size) {
        float[] hm = new float[size * size];
        float center = size / 2f;
        var rng = new java.util.Random(42);
        for (int r = 0; r < size; r++) {
            for (int c = 0; c < size; c++) {
                float dx = (c - center) / center;
                float dz = (r - center) / center;
                float dist = (float) Math.sqrt(dx * dx + dz * dz);

                // Base dome
                float elev = Math.max(0, 1f - dist * 1.2f) * 45f;
                // Ridge feature
                elev += (float)(Math.sin(c * 0.12 + r * 0.08) * 4 * Math.max(0, 1 - dist));
                // Noise
                elev += (rng.nextFloat() - 0.5f) * 2f;
                // Shore: smooth transition to underwater
                if (dist > 0.75f) {
                    float shore = (dist - 0.75f) / 0.25f;
                    elev = elev * (1 - shore) + (-8) * shore;
                }
                hm[r * size + c] = elev;
            }
        }
        return hm;
    }

    /** HeightBasedTerrain material matching the main viewer's look. */
    private Material createIslandMaterial() {
        Material mat = new Material(assetManager,
                "Common/MatDefs/Terrain/HeightBasedTerrain.j3md");
        // Underwater gravel
        Texture t1 = loadWrap("Textures/Terrain/PBR/Gravel015_1K_Color.png");
        mat.setTexture("region1ColorMap", t1);
        mat.setVector3("region1", new Vector3f(-20, 0, 16));
        // Sand
        Texture t2 = loadWrap("Textures/Terrain/PBR/Ground037_1K_Color.png");
        mat.setTexture("region2ColorMap", t2);
        mat.setVector3("region2", new Vector3f(0, 5, 24));
        // Vegetation
        Texture t3 = loadWrap("Textures/Terrain/custom/vegetation.png");
        mat.setTexture("region3ColorMap", t3);
        mat.setVector3("region3", new Vector3f(5, 40, 32));
        // Rock on peaks
        Texture t4 = loadWrap("Textures/Terrain/PBR/Gravel015_1K_Color.png");
        mat.setTexture("region4ColorMap", t4);
        mat.setVector3("region4", new Vector3f(40, 60, 16));
        // Slopes
        mat.setTexture("slopeColorMap", t4);
        mat.setFloat("slopeTileFactor", 16f);
        mat.setFloat("terrainSize", 129);
        return mat;
    }

    /** Place labelled groups of each tree type around the island. */
    private void placeTreeGroups(TerrainQuad tq) {
        String[] texPaths = {
            "Textures/Terrain/custom/tree1.png",
            "Textures/Terrain/custom/tree2.png",
            "Textures/Terrain/custom/tree3.png",
            "Textures/Terrain/custom/tree4.png"
        };
        String[] names = {"Broadleaf", "Conifer", "Palm", "Bush"};
        float[][] heights = {{8, 15}, {10, 18}, {10, 20}, {3, 7}};
        float[] wRatios = {0.7f, 0.35f, 0.5f, 0.9f};

        var rng = new java.util.Random(77);
        Vector3f scale = tq.getWorldScale();

        for (int type = 0; type < 4; type++) {
            Material mat = createTreeMat(texPaths[type]);
            float angle = type * FastMath.HALF_PI + 0.3f;
            float groupR = 60f;
            int placed = 0;

            for (int i = 0; i < 15; i++) {
                float wx = (float)(Math.cos(angle) * groupR + (rng.nextFloat() - 0.5) * 35);
                float wz = (float)(Math.sin(angle) * groupR + (rng.nextFloat() - 0.5) * 35);

                // Raycast straight down to find exact terrain surface
                var ray = new com.jme3.math.Ray(
                        new Vector3f(wx, 200, wz), new Vector3f(0, -1, 0));
                var results = new com.jme3.collision.CollisionResults();
                tq.collideWith(ray, results);
                if (results.size() == 0) continue;
                Vector3f hitPoint = results.getClosestCollision().getContactPoint();
                if (hitPoint.y < 2) continue;

                float h = heights[type][0] + rng.nextFloat() * (heights[type][1] - heights[type][0]);
                float w = h * wRatios[type] * (0.85f + rng.nextFloat() * 0.3f);

                Node tree = makeCrossBillboard(mat, w, h, rng.nextFloat() * FastMath.PI);
                tree.setLocalTranslation(hitPoint.x, hitPoint.y, hitPoint.z);
                rootNode.attachChild(tree);
                placed++;
            }
            System.out.printf("  %s: %d trees%n", names[type], placed);
        }
    }

    private Material createTreeMat(String path) {
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setTexture("ColorMap", assetManager.loadTexture(path));
        mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
        mat.setFloat("AlphaDiscardThreshold", 0.7f);
        return mat;
    }

    private Node makeCrossBillboard(Material mat, float width, float height, float rot) {
        Node cross = new Node();
        Quad q = new Quad(1, 1);

        Geometry g1 = new Geometry("t", q);
        g1.setMaterial(mat);
        g1.setQueueBucket(RenderQueue.Bucket.Transparent);
        g1.setLocalScale(width, height, 1);
        g1.setLocalTranslation(-width / 2, 0, 0);
        cross.attachChild(g1);

        Geometry g2 = new Geometry("t", q);
        g2.setMaterial(mat);
        g2.setQueueBucket(RenderQueue.Bucket.Transparent);
        g2.setLocalScale(width, height, 1);
        g2.setLocalTranslation(0, 0, width / 2);
        g2.setLocalRotation(new Quaternion().fromAngleAxis(FastMath.HALF_PI, Vector3f.UNIT_Y));
        cross.attachChild(g2);

        cross.setLocalRotation(new Quaternion().fromAngleAxis(rot, Vector3f.UNIT_Y));
        return cross;
    }

    private Texture loadWrap(String path) {
        Texture t = assetManager.loadTexture(path);
        t.setWrap(Texture.WrapMode.Repeat);
        return t;
    }
}
