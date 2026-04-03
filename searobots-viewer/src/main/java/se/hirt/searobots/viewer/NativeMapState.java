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

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.BaseAppState;
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.shape.Quad;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;

import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.engine.GeneratedWorld;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * jME3-native tactical map overlay. Renders the 2D map using GPU-accelerated
 * geometry in an offscreen viewport, eliminating the Java2D freeze that would
 * happen on Mac when trying to use the Map.
 *
 * {@link MapViewState}.
 */
final class NativeMapState extends BaseAppState {

    // ── Map display modes ──
    enum MapMode { HIDDEN, MINIMAP, FULLSCREEN }
    private static final float MINIMAP_SCALE = 0.28f;
    private static final float MINIMAP_MARGIN = 15f;
    private static final float MINIMAP_ALPHA = 0.9f;
    private static final float FADE_SPEED = 4f;      // alpha units per second
    private static final float RESIZE_SPEED = 10f;    // exponential lerp speed

    // ── Zoom ──
    private static final float ZOOM_LERP_SPEED = 12f;

    // ── Z-layers (higher = rendered on top) ──
    private static final float Z_TERRAIN     = 0f;
    private static final float Z_CONTOURS    = 0.1f;
    private static final float Z_BATTLE_AREA = 0.2f;
    private static final float Z_SPAWN       = 0.3f;
    private static final float Z_ROUTES      = 1.0f;
    private static final float Z_TRAILS      = 1.5f;
    private static final float Z_COMP_OBJ    = 2.0f;
    private static final float Z_WAYPOINTS   = 2.5f;
    private static final float Z_SUBS        = 3.0f;
    private static final float Z_TORPS       = 3.5f;
    private static final float Z_CONTACTS    = 4.0f;
    private static final float Z_FIRING_SOL  = 5.0f;
    private static final float Z_PINGS       = 6.0f;
    private static final float Z_EXPLOSIONS  = 7.0f;
    private static final float Z_DETECTION   = 8.0f;

    private static final int CIRCLE_SEGMENTS = 64;

    // ── Data source ──
    private final MapRenderer data;

    // ── View state ──
    private double viewX, viewY;
    private double pixelsPerMeter = 0.3;

    // ── Offscreen rendering ──
    private Camera mapCam;
    private ViewPort mapViewPort;
    private Texture2D renderTexture;
    private int fbWidth, fbHeight;

    // ── Display quad (in guiNode) ──
    private Geometry displayQuad;
    private Material displayMaterial;
    private Mesh fullscreenMesh;
    private Mesh minimapMesh;

    // ── Map scene graph ──
    private Node mapRoot;
    private Geometry terrainGeom;
    private Geometry contourGeom;
    private Geometry battleAreaGeom;
    private final List<Geometry> subGeoms  = new ArrayList<>();
    private final List<Geometry> torpGeoms = new ArrayList<>();
    private final List<Geometry> trailGeoms = new ArrayList<>();
    private final List<Geometry> routeGeoms = new ArrayList<>();
    private Node waypointNode;
    private Node contactNode;
    private Node animationNode;
    private Node firingSolNode;
    private Node compObjNode;

    // ── HUD (in guiNode, screen-space) ──
    private Node hudNode;
    private BitmapText infoText;
    private BitmapText subHudText;
    private BitmapText compText;
    private BitmapFont font;

    // ── Mode + animation state ──
    private MapMode mode = MapMode.HIDDEN;
    private float currentAlpha = 0f;
    private float alphaTarget = 0f;
    private float quadX, quadY, quadW, quadH;
    private float tgtX, tgtY, tgtW, tgtH;

    // ── Zoom animation ──
    private double targetPpm, targetViewX, targetViewY;
    private boolean zooming;

    // ── Throttling ──
    private int frameCount;
    private double lastContourInterval = -1;
    private GeneratedWorld lastWorld;

    private com.jme3.asset.AssetManager am;

    NativeMapState(MapRenderer data) {
        this.data = data;
    }

    // ═══════════════════════════════════════════════════════════════
    //  BaseAppState lifecycle
    // ═══════════════════════════════════════════════════════════════

    @Override
    protected void initialize(Application app) {
        am = app.getAssetManager();
        int w = app.getCamera().getWidth();
        int h = app.getCamera().getHeight();

        createOffscreenTarget(app, w, h);
        createDisplayQuad(w, h);
        createMapScene();
        createHud(app);

        if (data.getWorld() != null) worldChanged(data.getWorld());
    }

    @Override protected void onEnable() {
        var sa = (SimpleApplication) getApplication();
        sa.getGuiNode().attachChild(displayQuad);
        sa.getGuiNode().attachChild(hudNode);
    }

    @Override protected void onDisable() {
        displayQuad.removeFromParent();
        hudNode.removeFromParent();
        mode = MapMode.HIDDEN;
        currentAlpha = alphaTarget = 0f;
        displayMaterial.setColor("Color", new ColorRGBA(1, 1, 1, 0));
        mapViewPort.setEnabled(false);
    }

    @Override protected void cleanup(Application app) {
        app.getRenderManager().removePreView(mapViewPort);
    }

    // ═══════════════════════════════════════════════════════════════
    //  Public API (mirrors MapViewState)
    // ═══════════════════════════════════════════════════════════════

    void toggle() {
        switch (mode) {
            case HIDDEN -> {
                mode = MapMode.MINIMAP;
                alphaTarget = MINIMAP_ALPHA;
                computeMinimapTarget();
                quadX = tgtX; quadY = tgtY; quadW = tgtW; quadH = tgtH;
                displayQuad.setMesh(minimapMesh);
            }
            case MINIMAP -> {
                mode = MapMode.FULLSCREEN;
                alphaTarget = 1f;
                computeFullscreenTarget();
                displayQuad.setMesh(fullscreenMesh);
            }
            case FULLSCREEN -> {
                mode = MapMode.HIDDEN;
                alphaTarget = 0f;
            }
        }
    }

    /** True when the map is rendering (minimap or fullscreen). */
    boolean isMapVisible() { return mode != MapMode.HIDDEN || currentAlpha > 0; }

    /** True when the map covers the full screen. */
    boolean isFullscreen() { return mode == MapMode.FULLSCREEN; }

    /** True when mouse input should be redirected to the map (fullscreen, or cursor over minimap). */
    boolean wantsMouseInput(float cursorX, float cursorY) {
        if (mode == MapMode.FULLSCREEN) return true;
        if (mode == MapMode.MINIMAP) {
            return cursorX >= quadX && cursorX <= quadX + quadW
                && cursorY >= quadY && cursorY <= quadY + quadH;
        }
        return false;
    }

    void pan(float dx, float dy) {
        if (mode != MapMode.HIDDEN) {
            viewX += dx / pixelsPerMeter;
            viewY -= dy / pixelsPerMeter;
        }
    }

    void zoomAt(double factor, float screenX, float screenY) {
        if (mode == MapMode.HIDDEN) return;
        int w = getApplication().getCamera().getWidth();
        int h = getApplication().getCamera().getHeight();

        double basePpm = zooming ? targetPpm : pixelsPerMeter;
        double baseVX  = zooming ? targetViewX : viewX;
        double baseVY  = zooming ? targetViewY : viewY;

        double newPpm = Math.max(0.01, Math.min(10.0, basePpm * factor));
        targetPpm = newPpm;

        if (mode == MapMode.FULLSCREEN) {
            // Cursor-centered zoom
            double worldX = (screenX - w / 2.0) / basePpm + baseVX;
            double worldY = (screenY - h / 2.0) / basePpm + baseVY;
            targetViewX = worldX - (screenX - w / 2.0) / newPpm;
            targetViewY = worldY - (screenY - h / 2.0) / newPpm;
        } else {
            // Center-centered zoom (minimap)
            targetViewX = baseVX;
            targetViewY = baseVY;
        }
        zooming = true;
    }

    private void computeMinimapTarget() {
        int w = getApplication().getCamera().getWidth();
        int h = getApplication().getCamera().getHeight();
        float miniSize = Math.min(w, h) * MINIMAP_SCALE;
        tgtW = miniSize;
        tgtH = miniSize;
        tgtX = MINIMAP_MARGIN;
        tgtY = MINIMAP_MARGIN;
    }

    private void computeFullscreenTarget() {
        int w = getApplication().getCamera().getWidth();
        int h = getApplication().getCamera().getHeight();
        tgtW = w; tgtH = h; tgtX = 0; tgtY = 0;
    }

    void worldChanged(GeneratedWorld world) {
        if (world == null) return;
        lastWorld = world;

        int w = getApplication() != null ? getApplication().getCamera().getWidth() : 1920;
        int h = getApplication() != null ? getApplication().getCamera().getHeight() : 1080;
        double diameter = world.config().battleArea().extent() * 2;
        pixelsPerMeter = Math.min(w, h) * 0.8 / diameter;
        viewX = viewY = 0;

        rebuildTerrain(world);
        rebuildContours(world);
        rebuildBattleArea(world);
        lastContourInterval = effectiveContourInterval();

        for (var g : subGeoms)   g.setCullHint(Spatial.CullHint.Always);
        for (var g : torpGeoms)  g.setCullHint(Spatial.CullHint.Always);
        for (var g : trailGeoms) g.setCullHint(Spatial.CullHint.Always);
        for (var g : routeGeoms) g.setCullHint(Spatial.CullHint.Always);
        waypointNode.detachAllChildren();
        contactNode.detachAllChildren();
        animationNode.detachAllChildren();
        firingSolNode.detachAllChildren();
        compObjNode.detachAllChildren();
    }

    // ═══════════════════════════════════════════════════════════════
    //  Main update loop
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void update(float tpf) {
        // Lerp alpha toward target
        if (currentAlpha < alphaTarget)
            currentAlpha = Math.min(alphaTarget, currentAlpha + FADE_SPEED * tpf);
        else if (currentAlpha > alphaTarget)
            currentAlpha = Math.max(alphaTarget, currentAlpha - FADE_SPEED * tpf);

        // Fully hidden: disable viewport and bail
        if (mode == MapMode.HIDDEN && currentAlpha <= 0f) {
            mapViewPort.setEnabled(false); return;
        }
        mapViewPort.setEnabled(true);

        // Disable 3D scene when map covers the full screen
        getApplication().getViewPort().setEnabled(mode != MapMode.FULLSCREEN);

        // Lerp quad position/size toward target
        float lt = 1f - (float) Math.exp(-RESIZE_SPEED * tpf);
        quadX += (tgtX - quadX) * lt;
        quadY += (tgtY - quadY) * lt;
        quadW += (tgtW - quadW) * lt;
        quadH += (tgtH - quadH) * lt;
        displayQuad.setLocalScale(quadW, quadH, 1f);
        displayQuad.setLocalTranslation(quadX, quadY, 5);
        displayMaterial.setColor("Color", new ColorRGBA(1, 1, 1, currentAlpha));

        // No blending needed when fully opaque
        displayMaterial.getAdditionalRenderState().setBlendMode(
                currentAlpha >= 1f ? RenderState.BlendMode.Off : RenderState.BlendMode.Alpha);

        // HUD text only in fullscreen mode
        hudNode.setCullHint(mode == MapMode.FULLSCREEN && currentAlpha > 0.5f
                ? Spatial.CullHint.Inherit : Spatial.CullHint.Always);

        // Zoom animation
        if (zooming) {
            double t = 1.0 - Math.exp(-ZOOM_LERP_SPEED * tpf);
            pixelsPerMeter += (targetPpm - pixelsPerMeter) * t;
            viewX += (targetViewX - viewX) * t;
            viewY += (targetViewY - viewY) * t;
            if (Math.abs(pixelsPerMeter - targetPpm) < 0.0001) {
                pixelsPerMeter = targetPpm; viewX = targetViewX; viewY = targetViewY;
                zooming = false;
            }
        }

        checkResize();
        updateCamera();

        // Detect world change
        var world = data.getWorld();
        if (world != null && world != lastWorld) worldChanged(world);
        if (world == null) return;

        // Contour interval may change with zoom
        double ci = effectiveContourInterval();
        if (ci != lastContourInterval) { rebuildContours(world); lastContourInterval = ci; }

        // Contour visibility toggle
        if (contourGeom != null) {
            contourGeom.setCullHint(data.overlayConfig.contours
                    ? Spatial.CullHint.Inherit : Spatial.CullHint.Always);
        }

        frameCount++;
        updateSubmarines();
        updateTorpedoes();
        if (frameCount % 10 == 0) { updateTrails(); updateRoutes(); }
        updateWaypoints();
        updateContacts();
        updateFiringSolution();
        updateCompetitionObjectives();
        updateAnimations();
        updateHud();

        mapRoot.updateLogicalState(tpf);
        mapRoot.updateGeometricState();
    }

    // ═══════════════════════════════════════════════════════════════
    //  Offscreen viewport + camera
    // ═══════════════════════════════════════════════════════════════

    private void createOffscreenTarget(Application app, int w, int h) {
        fbWidth = w; fbHeight = h;
        mapCam = new Camera(w, h);
        mapCam.setParallelProjection(true);
        mapCam.setLocation(new Vector3f(0, 0, 10));
        mapCam.lookAtDirection(new Vector3f(0, 0, -1), Vector3f.UNIT_Y);
        updateCameraFrustum();

        renderTexture = new Texture2D(w, h, Image.Format.RGB8);
        renderTexture.setMinFilter(Texture.MinFilter.BilinearNoMipMaps);
        renderTexture.setMagFilter(Texture.MagFilter.Bilinear);

        var fb = new FrameBuffer(w, h, 1);
        fb.setDepthBuffer(Image.Format.Depth);
        fb.setColorTexture(renderTexture);

        mapViewPort = app.getRenderManager().createPreView("MapView", mapCam);
        mapViewPort.setClearFlags(true, true, true);
        mapViewPort.setBackgroundColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));
        mapViewPort.setOutputFrameBuffer(fb);
        mapViewPort.setEnabled(false);
    }

    private void checkResize() {
        int w = getApplication().getCamera().getWidth();
        int h = getApplication().getCamera().getHeight();
        if (w == fbWidth && h == fbHeight || w <= 0 || h <= 0) return;

        getApplication().getRenderManager().removePreView(mapViewPort);
        createOffscreenTarget(getApplication(), w, h);
        mapViewPort.attachScene(mapRoot);
        mapViewPort.setEnabled(currentAlpha > 0);

        displayMaterial.setTexture("ColorMap", renderTexture);
        minimapMesh = buildMinimapMesh();
        repositionHud(w, h);

        // Recompute targets for current mode
        if (mode == MapMode.MINIMAP) {
            computeMinimapTarget();
            displayQuad.setMesh(minimapMesh);
        } else if (mode == MapMode.FULLSCREEN) {
            computeFullscreenTarget();
        }
    }

    private void updateCamera() {
        mapCam.setLocation(new Vector3f((float) viewX, (float) viewY, 10));
        updateCameraFrustum();
    }

    private void updateCameraFrustum() {
        float halfW = (float) (fbWidth  / (2.0 * pixelsPerMeter));
        float halfH = (float) (fbHeight / (2.0 * pixelsPerMeter));
        mapCam.setFrustum(0.1f, 100f, -halfW, halfW, halfH, -halfH);
    }

    // ═══════════════════════════════════════════════════════════════
    //  Display quad + scene graph + HUD creation
    // ═══════════════════════════════════════════════════════════════

    private void createDisplayQuad(int w, int h) {
        fullscreenMesh = new Quad(1, 1);
        minimapMesh = buildMinimapMesh();
        displayQuad = new Geometry("MapDisplay", fullscreenMesh);
        displayMaterial = new Material(am, "Common/MatDefs/Misc/Unshaded.j3md");
        displayMaterial.setTexture("ColorMap", renderTexture);
        displayMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        displayMaterial.setColor("Color", new ColorRGBA(1, 1, 1, 0));
        displayQuad.setMaterial(displayMaterial);
        displayQuad.setQueueBucket(RenderQueue.Bucket.Gui);
        displayQuad.setLocalTranslation(0, 0, 5);
        displayQuad.setLocalScale(w, h, 1);
    }

    /** Quad mesh with UVs cropped to the center square of the framebuffer. */
    private Mesh buildMinimapMesh() {
        float aspect = (float) fbWidth / fbHeight;
        float uMin, uMax, vMin, vMax;
        if (aspect >= 1f) {
            float crop = (1f - 1f / aspect) / 2f;
            uMin = crop; uMax = 1f - crop; vMin = 0; vMax = 1;
        } else {
            float crop = (1f - aspect) / 2f;
            uMin = 0; uMax = 1; vMin = crop; vMax = 1f - crop;
        }
        var m = new Mesh();
        m.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                0, 0, 0,  1, 0, 0,  1, 1, 0,  0, 1, 0});
        m.setBuffer(VertexBuffer.Type.TexCoord, 2, new float[]{
                uMin, vMin,  uMax, vMin,  uMax, vMax,  uMin, vMax});
        m.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 1, 2, 0, 2, 3});
        m.updateBound();
        return m;
    }

    private void createMapScene() {
        mapRoot = new Node("MapRoot");
        mapViewPort.attachScene(mapRoot);

        waypointNode  = new Node("Waypoints");
        contactNode   = new Node("Contacts");
        animationNode = new Node("Animations");
        firingSolNode = new Node("FiringSolution");
        compObjNode   = new Node("CompObjectives");
        mapRoot.attachChild(waypointNode);
        mapRoot.attachChild(contactNode);
        mapRoot.attachChild(animationNode);
        mapRoot.attachChild(firingSolNode);
        mapRoot.attachChild(compObjNode);
    }

    private void createHud(Application app) {
        font = am.loadFont("Interface/Fonts/Default.fnt");
        hudNode = new Node("MapHud");
        hudNode.setLocalTranslation(0, 0, 6);
        hudNode.setCullHint(Spatial.CullHint.Always);

        infoText   = makeBitmapText(new ColorRGBA(0.78f, 0.86f, 1f, 1f));
        subHudText = makeBitmapText(ColorRGBA.White);
        compText   = makeBitmapText(new ColorRGBA(0.75f, 0.82f, 0.9f, 1f));
        hudNode.attachChild(infoText);
        hudNode.attachChild(subHudText);
        hudNode.attachChild(compText);

        repositionHud(app.getCamera().getWidth(), app.getCamera().getHeight());
    }

    private BitmapText makeBitmapText(ColorRGBA color) {
        var bt = new BitmapText(font);
        bt.setSize(font.getCharSet().getRenderedSize());
        bt.setColor(color);
        return bt;
    }

    private void repositionHud(int w, int h) {
        infoText.setLocalTranslation(10, h - 10, 0);
        subHudText.setLocalTranslation(w - 320, h - 10, 0);
        compText.setLocalTranslation(w - 350, h - 300, 0);
    }

    // ═══════════════════════════════════════════════════════════════
    //  Static world geometry
    // ═══════════════════════════════════════════════════════════════

    private void rebuildTerrain(GeneratedWorld world) {
        if (terrainGeom != null) terrainGeom.removeFromParent();
        var terrain = world.terrain();
        float cellSize = (float) terrain.getCellSize();
        float tw = terrain.getCols() * cellSize;
        float th = terrain.getRows() * cellSize;

        terrainGeom = new Geometry("Terrain", new Quad(tw, th));
        terrainGeom.setLocalTranslation(
                (float) terrain.getOriginX(), (float) terrain.getOriginY(), Z_TERRAIN);
        var mat = new Material(am, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setTexture("ColorMap", renderTerrainTexture(world));
        terrainGeom.setMaterial(mat);
        mapRoot.attachChild(terrainGeom);
    }

    private void rebuildContours(GeneratedWorld world) {
        if (contourGeom != null) contourGeom.removeFromParent();
        contourGeom = null;
        var mesh = buildContourMesh(world);
        if (mesh == null) return;
        contourGeom = new Geometry("Contours", mesh);
        contourGeom.setLocalTranslation(0, 0, Z_CONTOURS);
        contourGeom.setMaterial(makeSolidMat(new ColorRGBA(0, 0, 0, 0.24f)));
        contourGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
        mapRoot.attachChild(contourGeom);
    }

    private void rebuildBattleArea(GeneratedWorld world) {
        if (battleAreaGeom != null) battleAreaGeom.removeFromParent();
        Mesh mesh = switch (world.config().battleArea()) {
            case BattleArea.Circular(var r)        -> makeCircleMesh((float) r, 128);
            case BattleArea.Rectangular(var hw, var hh) -> makeRectMesh((float) hw, (float) hh);
        };
        battleAreaGeom = new Geometry("BattleArea", mesh);
        battleAreaGeom.setLocalTranslation(0, 0, Z_BATTLE_AREA);
        var mat = makeSolidMat(new ColorRGBA(1f, 0.78f, 0.2f, 0.63f));
        mat.getAdditionalRenderState().setLineWidth(2f);
        battleAreaGeom.setMaterial(mat);
        battleAreaGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
        mapRoot.attachChild(battleAreaGeom);
    }

    // ── contour mesh builder ──

    private double effectiveContourInterval() {
        double interval = 25.0;
        while (interval * pixelsPerMeter < 4.0) interval *= 2;
        return interval;
    }

    private Mesh buildContourMesh(GeneratedWorld world) {
        var terrain = world.terrain();
        int cols = terrain.getCols(), rows = terrain.getRows();
        double cell = terrain.getCellSize();
        double ox = terrain.getOriginX(), oy = terrain.getOriginY();
        double interval = effectiveContourInterval();
        var v = new FloatList();

        for (int row = 0; row < rows - 1; row++) {
            for (int col = 0; col < cols - 1; col++) {
                double e00 = terrain.elevationAtGrid(col, row);
                double e10 = terrain.elevationAtGrid(col + 1, row);
                double e01 = terrain.elevationAtGrid(col, row + 1);
                double e11 = terrain.elevationAtGrid(col + 1, row + 1);

                long c0 = (long) Math.floor(e00 / interval);
                long c1 = (long) Math.floor(e10 / interval);
                if (c0 != c1) {
                    double ce = Math.max(c0, c1) * interval;
                    double t = (ce - e00) / (e10 - e00);
                    contourSegH(v, col, row, ox + (col + t) * cell, oy + row * cell,
                                cell, ox, oy, interval, e00, e10, e01, e11);
                }
                long c2 = (long) Math.floor(e01 / interval);
                if (c0 != c2) {
                    double ce = Math.max(c0, c2) * interval;
                    double t = (ce - e00) / (e01 - e00);
                    contourSegV(v, col, row, ox + col * cell, oy + (row + t) * cell,
                                cell, ox, oy, interval, e00, e10, e01, e11);
                }
            }
        }
        if (v.size == 0) return null;
        var mesh = new Mesh();
        mesh.setMode(Mesh.Mode.Lines);
        mesh.setBuffer(VertexBuffer.Type.Position, 3, v.toArray());
        mesh.updateBound();
        return mesh;
    }

    private void contourSegH(FloatList v, int col, int row,
                              double cx, double cy, double cell,
                              double ox, double oy, double interval,
                              double e00, double e10, double e01, double e11) {
        double ce = Math.max(Math.floor(e00 / interval), Math.floor(e10 / interval)) * interval;
        if (crosses(e01, e11, interval)) {
            double t = (ce - e01) / (e11 - e01);
            if (t >= 0 && t <= 1) { v.addLine(cx, cy, ox + (col + t) * cell, oy + (row + 1) * cell, 0); return; }
        }
        if (crosses(e00, e01, interval)) {
            double t = (ce - e00) / (e01 - e00);
            if (t >= 0 && t <= 1) { v.addLine(cx, cy, ox + col * cell, oy + (row + t) * cell, 0); return; }
        }
        if (crosses(e10, e11, interval)) {
            double t = (ce - e10) / (e11 - e10);
            if (t >= 0 && t <= 1) v.addLine(cx, cy, ox + (col + 1) * cell, oy + (row + t) * cell, 0);
        }
    }

    private void contourSegV(FloatList v, int col, int row,
                              double cx, double cy, double cell,
                              double ox, double oy, double interval,
                              double e00, double e10, double e01, double e11) {
        double ce = Math.max(Math.floor(e00 / interval), Math.floor(e01 / interval)) * interval;
        if (crosses(e10, e11, interval)) {
            double t = (ce - e10) / (e11 - e10);
            if (t >= 0 && t <= 1) { v.addLine(cx, cy, ox + (col + 1) * cell, oy + (row + t) * cell, 0); return; }
        }
        if (crosses(e01, e11, interval)) {
            double t = (ce - e01) / (e11 - e01);
            if (t >= 0 && t <= 1) { v.addLine(cx, cy, ox + (col + t) * cell, oy + (row + 1) * cell, 0); return; }
        }
        if (crosses(e00, e10, interval)) {
            double t = (ce - e00) / (e10 - e00);
            if (t >= 0 && t <= 1) v.addLine(cx, cy, ox + (col + t) * cell, oy + row * cell, 0);
        }
    }

    private static boolean crosses(double a, double b, double interval) {
        return (long) Math.floor(a / interval) != (long) Math.floor(b / interval);
    }

    // ═══════════════════════════════════════════════════════════════
    //  Dynamic entity updates
    // ═══════════════════════════════════════════════════════════════

    private void updateSubmarines() {
        var subs = data.submarines;
        ensurePool(subGeoms, subs.size(), this::makeSubTriangle);
        float size = 30f / (float) pixelsPerMeter;
        for (int i = 0; i < subs.size(); i++) {
            var sub = subs.get(i);
            var g = subGeoms.get(i);
            var pos = sub.pose().position();
            g.setLocalTranslation((float) pos.x(), (float) pos.y(), Z_SUBS);
            g.setLocalScale(size, size, 1f);
            var q = new Quaternion();
            q.fromAngleAxis(-(float) sub.pose().heading(), Vector3f.UNIT_Z);
            g.setLocalRotation(q);
            var c = sub.forfeited() ? java.awt.Color.DARK_GRAY : sub.color();
            g.getMaterial().setColor("Color", toColor(c, 0.7f));
            g.setCullHint(Spatial.CullHint.Inherit);
        }
        for (int i = subs.size(); i < subGeoms.size(); i++)
            subGeoms.get(i).setCullHint(Spatial.CullHint.Always);
    }

    private void updateTorpedoes() {
        var torps = data.torpedoSnapshots;
        int alive = 0;
        for (var t : torps) if (t.alive()) alive++;
        ensurePool(torpGeoms, alive, this::makeTorpDiamond);

        float size = 15f / (float) pixelsPerMeter;
        int idx = 0;
        for (var torp : torps) {
            if (!torp.alive() || idx >= torpGeoms.size()) continue;
            var g = torpGeoms.get(idx);
            var pos = torp.pose().position();
            g.setLocalTranslation((float) pos.x(), (float) pos.y(), Z_TORPS);
            g.setLocalScale(size, size, 1f);
            var q = new Quaternion();
            q.fromAngleAxis(-(float) torp.pose().heading(), Vector3f.UNIT_Z);
            g.setLocalRotation(q);
            g.getMaterial().setColor("Color", toColor(torp.color(), 0.86f));
            g.setCullHint(Spatial.CullHint.Inherit);
            idx++;
        }
        for (int i = idx; i < torpGeoms.size(); i++)
            torpGeoms.get(i).setCullHint(Spatial.CullHint.Always);
    }

    private void updateTrails() {
        var subs = data.submarines;
        if (!data.overlayConfig.trails) {
            for (var g : trailGeoms) g.setCullHint(Spatial.CullHint.Always);
            return;
        }
        ensurePool(trailGeoms, subs.size(), this::makeVertexColorGeom);
        for (int i = 0; i < subs.size() && i < data.trails.length; i++) {
            var trail = data.trails[i];
            var g = trailGeoms.get(i);
            if (trail == null || trail.size() < 2) {
                g.setCullHint(Spatial.CullHint.Always); continue;
            }
            Vec3[] pts;
            try { pts = trail.toArray(new Vec3[0]); } catch (Exception e) { continue; }
            var c = subs.get(i).color();
            float cr = c.getRed() / 255f, cg = c.getGreen() / 255f, cb = c.getBlue() / 255f;
            var vb = new FloatList(); var cb2 = new FloatList();
            for (int j = 1; j < pts.length; j++) {
                if (pts[j - 1] == null || pts[j] == null) continue;
                float a = (float) j / pts.length * 0.6f;
                vb.add3((float) pts[j-1].x(), (float) pts[j-1].y(), Z_TRAILS);
                vb.add3((float) pts[j].x(),   (float) pts[j].y(),   Z_TRAILS);
                cb2.add4(cr, cg, cb, a); cb2.add4(cr, cg, cb, a);
            }
            if (vb.size == 0) { g.setCullHint(Spatial.CullHint.Always); continue; }
            var m = new Mesh(); m.setMode(Mesh.Mode.Lines);
            m.setBuffer(VertexBuffer.Type.Position, 3, vb.toArray());
            m.setBuffer(VertexBuffer.Type.Color, 4, cb2.toArray());
            m.updateBound();
            g.setMesh(m);
            g.setCullHint(Spatial.CullHint.Inherit);
        }
        for (int i = subs.size(); i < trailGeoms.size(); i++)
            trailGeoms.get(i).setCullHint(Spatial.CullHint.Always);
    }

    private void updateRoutes() {
        var subs = data.submarines;
        if (!data.overlayConfig.route) {
            for (var g : routeGeoms) g.setCullHint(Spatial.CullHint.Always);
            return;
        }
        ensurePool(routeGeoms, subs.size(), () -> {
            var g = new Geometry("Route", new Mesh());
            g.setMaterial(makeSolidMat(ColorRGBA.White));
            g.setQueueBucket(RenderQueue.Bucket.Transparent);
            return g;
        });
        for (int i = 0; i < subs.size() && i < data.routes.length; i++) {
            var route = data.routes[i];
            var g = routeGeoms.get(i);
            if (route == null || route.size() < 2) {
                g.setCullHint(Spatial.CullHint.Always); continue;
            }
            var c = subs.get(i).color();
            var vb = new FloatList();
            for (int j = 1; j < route.size(); j++) {
                var p0 = route.get(j - 1); var p1 = route.get(j);
                vb.add3((float) p0.x(), (float) p0.y(), Z_ROUTES);
                vb.add3((float) p1.x(), (float) p1.y(), Z_ROUTES);
            }
            var m = new Mesh(); m.setMode(Mesh.Mode.Lines);
            m.setBuffer(VertexBuffer.Type.Position, 3, vb.toArray());
            m.updateBound();
            g.setMesh(m);
            g.getMaterial().setColor("Color", toColor(c, 0.31f));
            g.setCullHint(Spatial.CullHint.Inherit);
        }
        for (int i = subs.size(); i < routeGeoms.size(); i++)
            routeGeoms.get(i).setCullHint(Spatial.CullHint.Always);
    }

    // ═══════════════════════════════════════════════════════════════
    //  Overlay updates
    // ═══════════════════════════════════════════════════════════════

    private void updateWaypoints() {
        waypointNode.detachAllChildren();
        if (!data.overlayConfig.waypoints) return;
        for (var sub : data.submarines) {
            var wps = sub.waypoints();
            if (wps == null || wps.isEmpty()) continue;
            var c = sub.color();

            // Line segments between waypoints
            if (wps.size() >= 2) {
                var vb = new FloatList();
                for (int i = 0; i < wps.size() - 1; i++) {
                    var a = wps.get(i); var b = wps.get(i + 1);
                    vb.add3((float) a.x(), (float) a.y(), 0);
                    vb.add3((float) b.x(), (float) b.y(), 0);
                }
                waypointNode.attachChild(makeLineGeom("WpLine", vb, Z_WAYPOINTS,
                        toColor(c, 0.5f, 0.5f)));
            }

            // Markers
            for (var wp : wps) {
                float r = 8f / (float) pixelsPerMeter;
                if (wp.active()) r *= 1.4f;
                ColorRGBA color = wp.reverse()
                        ? new ColorRGBA(0.71f, 0.24f, 0.86f, 0.78f)
                        : waypointDepthColor(wp.z());
                Mesh marker = wp.reverse() ? makeDiamondMesh(r) : makeCircleMesh(r, 24);
                var g = new Geometry("Wp", marker);
                g.setLocalTranslation((float) wp.x(), (float) wp.y(), Z_WAYPOINTS + 0.1f);
                g.setMaterial(makeSolidMat(color));
                g.setQueueBucket(RenderQueue.Bucket.Transparent);
                waypointNode.attachChild(g);
            }

            // Strategic waypoints
            if (data.overlayConfig.strategicWaypoints) {
                var swps = sub.strategicWaypoints();
                if (swps != null) for (int i = 0; i < swps.size(); i++) {
                    var wp = swps.get(i).waypoint();
                    float sr = 20f / (float) pixelsPerMeter;
                    var color = wp.active()
                            ? new ColorRGBA(1f, 0.86f, 0.2f, 0.86f)
                            : new ColorRGBA(1f, 0.78f, 0.2f, 0.55f);
                    var g = new Geometry("Swp", makeCrosshairMesh(sr));
                    g.setLocalTranslation((float) wp.x(), (float) wp.y(), Z_WAYPOINTS + 0.2f);
                    g.setMaterial(makeSolidMat(color));
                    g.setQueueBucket(RenderQueue.Bucket.Transparent);
                    waypointNode.attachChild(g);
                }
            }
        }
    }

    private void updateContacts() {
        contactNode.detachAllChildren();
        if (!data.overlayConfig.contactEstimates) return;
        for (var sub : data.submarines) {
            var ests = sub.contactEstimates();
            if (ests == null || ests.isEmpty()) continue;
            var base = sub.color();
            for (var est : ests) {
                float alive = (float) est.contactAlive();
                float alpha = 0.24f + 0.7f * alive;

                // Uncertainty circle
                float ur = (float) est.uncertaintyRadius();
                if (ur > 10) {
                    var g = new Geometry("UrC", makeCircleMesh(ur, 48));
                    g.setLocalTranslation((float) est.x(), (float) est.y(), Z_CONTACTS);
                    g.setMaterial(makeSolidMat(toColor(base, alive * 0.47f)));
                    g.setQueueBucket(RenderQueue.Bucket.Transparent);
                    contactNode.attachChild(g);
                }
                // Center diamond
                float ds = (float) (20f / pixelsPerMeter * (0.5 + 0.5 * alive));
                var g = new Geometry("Est", makeDiamondMesh(ds));
                g.setLocalTranslation((float) est.x(), (float) est.y(), Z_CONTACTS + 0.1f);
                g.setMaterial(makeSolidMat(toColor(base, alpha)));
                g.setQueueBucket(RenderQueue.Bucket.Transparent);
                contactNode.attachChild(g);

                // Heading arrow
                if (!Double.isNaN(est.estimatedHeading()) && est.estimatedSpeed() > 0) {
                    float aLen = (float) (est.estimatedSpeed() * 20 / pixelsPerMeter);
                    float ax = (float) (est.x() + aLen * Math.sin(est.estimatedHeading()));
                    float ay = (float) (est.y() + aLen * Math.cos(est.estimatedHeading()));
                    var vb = new FloatList();
                    vb.add3((float) est.x(), (float) est.y(), 0);
                    vb.add3(ax, ay, 0);
                    contactNode.attachChild(makeLineGeom("Arrow", vb, Z_CONTACTS + 0.2f,
                            toColor(base, alpha)));
                }
            }
        }
    }

    private void updateFiringSolution() {
        firingSolNode.detachAllChildren();
        for (var sub : data.submarines) {
            var sol = sub.firingSolution();
            if (sol == null) continue;
            float r1 = 32f / (float) pixelsPerMeter, r2 = 48f / (float) pixelsPerMeter;
            var g = new Geometry("FS", makeFiringSolMesh(r1, r2));
            g.setLocalTranslation((float) sol.targetX(), (float) sol.targetY(), Z_FIRING_SOL);
            var mat = makeSolidMat(toColor(sub.color(), 0.86f));
            mat.getAdditionalRenderState().setLineWidth(2.5f);
            g.setMaterial(mat);
            g.setQueueBucket(RenderQueue.Bucket.Transparent);
            firingSolNode.attachChild(g);
        }
    }

    private void updateCompetitionObjectives() {
        compObjNode.detachAllChildren();
        var obj = data.competitionObjectives;
        if (obj == null || obj.isEmpty()) return;
        for (var wp : obj) {
            float x = (float) wp.x(), y = (float) wp.y();
            // Outer ring (400m)
            var ring = new Geometry("ObjR", makeCircleMesh(400f, 64));
            ring.setLocalTranslation(x, y, Z_COMP_OBJ);
            ring.setMaterial(makeSolidMat(new ColorRGBA(1f, 0.78f, 0.2f, 0.47f)));
            ring.setQueueBucket(RenderQueue.Bucket.Transparent);
            compObjNode.attachChild(ring);
            // Center crosshair
            var cross = new Geometry("ObjC", makeCrosshairMesh(80f));
            cross.setLocalTranslation(x, y, Z_COMP_OBJ + 0.1f);
            cross.setMaterial(makeSolidMat(new ColorRGBA(1f, 0.86f, 0.2f, 0.78f)));
            cross.setQueueBucket(RenderQueue.Bucket.Transparent);
            compObjNode.attachChild(cross);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  Animations (pings, explosions, detection highlights)
    // ═══════════════════════════════════════════════════════════════

    private void updateAnimations() {
        animationNode.detachAllChildren();
        long tick = data.simTick;

        // Ping rings
        for (var ping : data.pingAnimations) {
            double elapsed = (tick - ping.startTick()) / 50.0;
            double leadR = elapsed * 1500.0;
            if (leadR <= 0 || leadR > 10000) continue;
            var c = ping.color();
            float progress = (float) Math.min(leadR / 10000, 1.0);
            float fade = 1f - progress * 0.6f;

            // Leading bright ring
            int br = Math.min(255, c.getRed() + 140);
            int bg = Math.min(255, c.getGreen() + 140);
            int bb = Math.min(255, c.getBlue() + 140);
            addRing(animationNode, (float) ping.x(), (float) ping.y(), (float) leadR,
                    new ColorRGBA(br / 255f, bg / 255f, bb / 255f, fade * 0.86f),
                    Z_PINGS, 3f);

            // Trailing rings
            for (int i = 1; i <= 4; i++) {
                double r = leadR - i * 200.0;
                if (r <= 0) break;
                float a = fade * (0.55f - i * 0.12f);
                if (a <= 0) continue;
                addRing(animationNode, (float) ping.x(), (float) ping.y(), (float) r,
                        toColor(c, a), Z_PINGS, 2f);
            }

            // Origin flash
            if (elapsed < 1.2) {
                float ft = (float) (elapsed / 1.2);
                float flashFade = (1f - ft) * (1f - ft);
                float flashR = (float) (150 * (0.3 + 0.7 * ft));
                addFilledCircle(animationNode, (float) ping.x(), (float) ping.y(),
                        flashR, new ColorRGBA(1, 1, 1, flashFade * 0.63f), Z_PINGS + 0.1f);
            }
        }

        // Explosions
        for (var exp : data.explosionAnimations) {
            double elapsed = (tick - exp.startTick()) / 50.0;
            double t = elapsed / 2.0;
            if (t < 0 || t > 1) continue;
            float radius = (float) (50 * Math.sqrt(t));
            float fade = (float) ((1 - t) * (1 - t));

            if (elapsed < 0.3) {
                float ft = (float) (elapsed / 0.3);
                addFilledCircle(animationNode, (float) exp.x(), (float) exp.y(),
                        (float) (15 * (1 + ft)), new ColorRGBA(1, 1, 0.78f, 1f - ft),
                        Z_EXPLOSIONS + 0.1f);
            }
            addRing(animationNode, (float) exp.x(), (float) exp.y(), radius,
                    new ColorRGBA(1f, 0.55f, 0.12f, fade * 0.78f), Z_EXPLOSIONS, 3f);
        }

        // Detection highlights
        for (var h : data.detectionHighlights) {
            double elapsed = (tick - h.startTick()) / 50.0;
            float t = (float) (elapsed / 1.5);
            if (t > 1f) continue;
            float fade = (1f - t) * (1f - t);
            float pulseR = (float) (80 * (0.5 + 0.5 * t));

            addFilledCircle(animationNode, (float) h.x(), (float) h.y(),
                    pulseR, toColor(h.color(), fade * 0.39f), Z_DETECTION);
            addRing(animationNode, (float) h.x(), (float) h.y(), pulseR,
                    new ColorRGBA(1, 1, 1, fade * 0.86f), Z_DETECTION + 0.1f, 2.5f);
        }
    }

    private void addRing(Node parent, float x, float y, float radius,
                          ColorRGBA color, float z, float lineWidth) {
        var g = new Geometry("Ring", makeCircleMesh(radius, CIRCLE_SEGMENTS));
        g.setLocalTranslation(x, y, z);
        var mat = makeSolidMat(color);
        mat.getAdditionalRenderState().setLineWidth(lineWidth);
        g.setMaterial(mat);
        g.setQueueBucket(RenderQueue.Bucket.Transparent);
        parent.attachChild(g);
    }

    private void addFilledCircle(Node parent, float x, float y, float radius,
                                  ColorRGBA color, float z) {
        var g = new Geometry("Disc", makeFilledCircleMesh(radius, 32));
        g.setLocalTranslation(x, y, z);
        g.setMaterial(makeSolidMat(color));
        g.setQueueBucket(RenderQueue.Bucket.Transparent);
        parent.attachChild(g);
    }

    // ═══════════════════════════════════════════════════════════════
    //  HUD text updates
    // ═══════════════════════════════════════════════════════════════

    private void updateHud() {
        float alpha = currentAlpha;
        var world = data.getWorld();

        // Info text (top-left)
        var sb = new StringBuilder();
        sb.append("Seed: ").append(Long.toHexString(world.config().worldSeed())).append('\n');
        long tick = data.simTick;
        long hz = world.config().tickRateHz();
        sb.append(String.format("Tick: %d", tick));
        boolean paused = data.simPausedSupplier != null && data.simPausedSupplier.getAsBoolean();
        double speed = data.simSpeedSupplier != null ? data.simSpeedSupplier.getAsDouble() : 1;
        sb.append(paused ? "  PAUSED" : String.format("  %.0fx", speed)).append('\n');
        long totalSec = tick / hz;
        sb.append(String.format("Time: %d:%02d", totalSec / 60, totalSec % 60)).append('\n');
        for (int i = 0; i < world.thermalLayers().size(); i++) {
            var tl = world.thermalLayers().get(i);
            sb.append(String.format("Thermocline %d: %.0fm  %.1f/%.1f C",
                    i + 1, -tl.depth(), tl.temperatureAbove(), tl.temperatureBelow()));
            sb.append('\n');
        }
        sb.append("Spawns: ").append(world.spawnPoints().size());
        infoText.setText(sb.toString());
        infoText.setColor(new ColorRGBA(0.78f, 0.86f, 1f, alpha));

        // Sub HUD text (top-right)
        var subs = data.submarines;
        if (!subs.isEmpty()) {
            var sh = new StringBuilder();
            for (var sub : subs) {
                var pos = sub.pose().position();
                var c = sub.forfeited() ? java.awt.Color.DARK_GRAY : sub.color();
                String label = sub.forfeited() ? "FORFEIT" : sub.hp() <= 0 ? "DEAD" : "";
                sh.append(String.format("%s (#%d)  HP %d  %s\n",
                        sub.name(), sub.id(), sub.hp(), label));
                sh.append(String.format("  %.1f m/s  depth %.0fm  hdg %.1f\n",
                        sub.speed(), -pos.z(), Math.toDegrees(sub.pose().heading())));
                double noiseDb = 80 + 20 * Math.log10(Math.max(sub.noiseLevel(), 0.001));
                sh.append(String.format("  noise:%.0f dB  throttle:%+.0f%%  torps:%d\n",
                        noiseDb, sub.throttle() * 100, sub.torpedoesRemaining()));
                if (sub.status() != null && !sub.status().isEmpty())
                    sh.append("  ").append(sub.status()).append('\n');
            }
            subHudText.setText(sh.toString());
        } else {
            subHudText.setText("");
        }
        subHudText.setColor(new ColorRGBA(1, 1, 1, alpha));

        // Competition text
        var compResults = data.competitionResults;
        var compPhase = data.competitionPhase;
        if ((compResults != null && !compResults.isEmpty())
                || (compPhase != null && !compPhase.isEmpty())) {
            var ct = new StringBuilder();
            if (compPhase != null && !compPhase.isEmpty())
                ct.append(compPhase).append('\n');
            if (compResults != null) {
                int start = Math.max(0, compResults.size() - 20);
                for (int i = start; i < compResults.size(); i++)
                    ct.append(compResults.get(i)).append('\n');
            }
            compText.setText(ct.toString());
        } else {
            compText.setText("");
        }
        compText.setColor(new ColorRGBA(0.75f, 0.82f, 0.9f, alpha));
    }

    // ═══════════════════════════════════════════════════════════════
    //  Mesh utilities
    // ═══════════════════════════════════════════════════════════════

    private static Mesh makeCircleMesh(float radius, int segments) {
        float[] verts = new float[segments * 3];
        short[] idx = new short[segments * 2];
        for (int i = 0; i < segments; i++) {
            double a = 2 * Math.PI * i / segments;
            verts[i * 3]     = (float) (Math.cos(a) * radius);
            verts[i * 3 + 1] = (float) (Math.sin(a) * radius);
            verts[i * 3 + 2] = 0;
            idx[i * 2]     = (short) i;
            idx[i * 2 + 1] = (short) ((i + 1) % segments);
        }
        var m = new Mesh();
        m.setMode(Mesh.Mode.Lines);
        m.setBuffer(VertexBuffer.Type.Position, 3, verts);
        m.setBuffer(VertexBuffer.Type.Index, 1, idx);
        m.updateBound();
        return m;
    }

    private static Mesh makeFilledCircleMesh(float radius, int segments) {
        float[] verts = new float[(segments + 1) * 3];
        // Center vertex
        verts[0] = 0; verts[1] = 0; verts[2] = 0;
        for (int i = 0; i < segments; i++) {
            double a = 2 * Math.PI * i / segments;
            int vi = (i + 1) * 3;
            verts[vi]     = (float) (Math.cos(a) * radius);
            verts[vi + 1] = (float) (Math.sin(a) * radius);
            verts[vi + 2] = 0;
        }
        short[] idx = new short[segments * 3];
        for (int i = 0; i < segments; i++) {
            idx[i * 3]     = 0;
            idx[i * 3 + 1] = (short) (i + 1);
            idx[i * 3 + 2] = (short) ((i < segments - 1) ? i + 2 : 1);
        }
        var m = new Mesh();
        m.setMode(Mesh.Mode.Triangles);
        m.setBuffer(VertexBuffer.Type.Position, 3, verts);
        m.setBuffer(VertexBuffer.Type.Index, 1, idx);
        m.updateBound();
        return m;
    }

    private static Mesh makeDiamondMesh(float size) {
        var m = new Mesh();
        m.setMode(Mesh.Mode.Triangles);
        m.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                0, size, 0,   size * 0.6f, 0, 0,   0, -size, 0,   -size * 0.6f, 0, 0
        });
        m.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 3, 1, 1, 3, 2});
        m.updateBound();
        return m;
    }

    private static Mesh makeCrosshairMesh(float size) {
        var m = new Mesh();
        m.setMode(Mesh.Mode.Lines);
        m.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                -size, 0, 0,  size, 0, 0,   0, -size, 0,  0, size, 0
        });
        m.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 1, 2, 3});
        m.updateBound();
        return m;
    }

    private static Mesh makeRectMesh(float halfW, float halfH) {
        var m = new Mesh();
        m.setMode(Mesh.Mode.Lines);
        m.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                -halfW, -halfH, 0,  halfW, -halfH, 0,
                 halfW,  halfH, 0, -halfW,  halfH, 0
        });
        m.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 1, 1, 2, 2, 3, 3, 0});
        m.updateBound();
        return m;
    }

    private Mesh makeFiringSolMesh(float r1, float r2) {
        // Crosshair gap + outer circle
        var v = new FloatList();
        // Horizontal gap crosshair
        v.add3(-r2, 0, 0); v.add3(-r1, 0, 0);
        v.add3( r1, 0, 0); v.add3( r2, 0, 0);
        // Vertical gap crosshair
        v.add3(0, -r2, 0); v.add3(0, -r1, 0);
        v.add3(0,  r1, 0); v.add3(0,  r2, 0);
        // Circle at r2
        int seg = 48;
        for (int i = 0; i < seg; i++) {
            double a1 = 2 * Math.PI * i / seg;
            double a2 = 2 * Math.PI * ((i + 1) % seg) / seg;
            v.add3((float)(Math.cos(a1)*r2), (float)(Math.sin(a1)*r2), 0);
            v.add3((float)(Math.cos(a2)*r2), (float)(Math.sin(a2)*r2), 0);
        }
        var m = new Mesh();
        m.setMode(Mesh.Mode.Lines);
        m.setBuffer(VertexBuffer.Type.Position, 3, v.toArray());
        m.updateBound();
        return m;
    }

    // ═══════════════════════════════════════════════════════════════
    //  Material + color utilities
    // ═══════════════════════════════════════════════════════════════

    private Material makeSolidMat(ColorRGBA color) {
        var mat = new Material(am, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", color);
        mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        mat.getAdditionalRenderState().setDepthWrite(false);
        return mat;
    }

    private static ColorRGBA toColor(java.awt.Color c, float alpha) {
        return new ColorRGBA(c.getRed() / 255f, c.getGreen() / 255f,
                             c.getBlue() / 255f, alpha);
    }

    /** Darker shade (multiply RGB by darkFactor). */
    private static ColorRGBA toColor(java.awt.Color c, float alpha, float darkFactor) {
        return new ColorRGBA(c.getRed() / 255f * darkFactor,
                             c.getGreen() / 255f * darkFactor,
                             c.getBlue() / 255f * darkFactor, alpha);
    }

    private static ColorRGBA waypointDepthColor(double z) {
        double depth = -z;
        int r, g, b;
        if (depth <= 20)       { r = 150; g = 220; b = 255; }
        else if (depth <= 200) {
            double t = (depth - 20) / 180.0;
            r = (int)(150 + t * -100); g = (int)(220 + t * -120); b = (int)(255 + t * -35);
        } else if (depth <= 500) {
            double t = (depth - 200) / 300.0;
            r = (int)(50 + t * 30); g = (int)(100 + t * -60); b = (int)(220 + t * -40);
        } else { r = 80; g = 40; b = 180; }
        return new ColorRGBA(r / 255f, g / 255f, b / 255f, 0.78f);
    }

    private Texture2D renderTerrainTexture(GeneratedWorld world) {
        var terrain = world.terrain();
        int w = terrain.getCols(), h = terrain.getRows();
        double minE = terrain.getMinElevation(), maxE = terrain.getMaxElevation();
        double rangeE = maxE - minE;
        if (rangeE == 0) rangeE = 1;

        ByteBuffer buf = ByteBuffer.allocateDirect(w * h * 4);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                double elev = terrain.elevationAtGrid(x, y);
                int r, g, b;
                if (elev >= 0) {
                    double t = Math.min(elev / Math.max(1, maxE), 1.0);
                    r = (int)(50 + t * 90); g = (int)(100 + t * 155); b = (int)(30 + t * 40);
                } else {
                    double t = (elev - minE) / rangeE;
                    r = (int)(15 + t * 70); g = (int)(25 + t * 155); b = (int)(70 + t * 130);
                }
                buf.put((byte) r).put((byte) g).put((byte) b).put((byte) 0xFF);
            }
        }
        buf.flip();
        var tex = new Texture2D(new Image(Image.Format.RGBA8, w, h, buf, ColorSpace.sRGB));
        tex.setMinFilter(Texture.MinFilter.BilinearNoMipMaps);
        tex.setMagFilter(Texture.MagFilter.Bilinear);
        return tex;
    }

    // ═══════════════════════════════════════════════════════════════
    //  Geometry factory helpers
    // ═══════════════════════════════════════════════════════════════

    private Geometry makeSubTriangle() {
        var m = new Mesh();
        m.setMode(Mesh.Mode.Triangles);
        m.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                0, 0.6f, 0, -0.35f, -0.4f, 0, 0.35f, -0.4f, 0
        });
        m.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 1, 2});
        m.updateBound();
        var g = new Geometry("Sub", m);
        g.setMaterial(makeSolidMat(ColorRGBA.White));
        g.setQueueBucket(RenderQueue.Bucket.Transparent);
        return g;
    }

    private Geometry makeTorpDiamond() {
        var m = new Mesh();
        m.setMode(Mesh.Mode.Triangles);
        m.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                0, 0.8f, 0, -0.15f, 0, 0, 0, -0.4f, 0, 0.15f, 0, 0
        });
        m.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 1, 3, 1, 2, 3});
        m.updateBound();
        var g = new Geometry("Torp", m);
        g.setMaterial(makeSolidMat(ColorRGBA.White));
        g.setQueueBucket(RenderQueue.Bucket.Transparent);
        return g;
    }

    private Geometry makeVertexColorGeom() {
        var g = new Geometry("Trail", new Mesh());
        var mat = new Material(am, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setBoolean("VertexColor", true);
        mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        mat.getAdditionalRenderState().setDepthWrite(false);
        g.setMaterial(mat);
        g.setQueueBucket(RenderQueue.Bucket.Transparent);
        return g;
    }

    private Geometry makeLineGeom(String name, FloatList verts, float z, ColorRGBA color) {
        var m = new Mesh();
        m.setMode(Mesh.Mode.Lines);
        m.setBuffer(VertexBuffer.Type.Position, 3, verts.toArray());
        m.updateBound();
        var g = new Geometry(name, m);
        g.setLocalTranslation(0, 0, z);
        g.setMaterial(makeSolidMat(color));
        g.setQueueBucket(RenderQueue.Bucket.Transparent);
        return g;
    }

    private void ensurePool(List<Geometry> pool, int needed,
                             java.util.function.Supplier<Geometry> factory) {
        while (pool.size() < needed) {
            var g = factory.get();
            pool.add(g);
            mapRoot.attachChild(g);
        }
    }

    private static final class FloatList {
        float[] data = new float[256];
        int size = 0;

        void add(float v) {
            if (size == data.length) data = Arrays.copyOf(data, data.length * 2);
            data[size++] = v;
        }

        void add3(float x, float y, float z) { add(x); add(y); add(z); }
        void add4(float r, float g, float b, float a) { add(r); add(g); add(b); add(a); }

        void addLine(double x1, double y1, double x2, double y2, float z) {
            add3((float) x1, (float) y1, z);
            add3((float) x2, (float) y2, z);
        }

        float[] toArray() { return Arrays.copyOf(data, size); }
    }
}
