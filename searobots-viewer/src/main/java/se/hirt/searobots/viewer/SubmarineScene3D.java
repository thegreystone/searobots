/*
 * Copyright (C) 2026 Marcus Hirt
 *                    (see TerrainViewer.java for full license text)
 */
package se.hirt.searobots.viewer;

import com.jme3.app.SimpleApplication;
import com.jme3.font.BitmapText;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.FogFilter;
import com.jme3.post.filters.LightScatteringFilter;
import com.jme3.water.WaterFilter;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.MouseAxisTrigger;
import com.jme3.input.controls.MouseButtonTrigger;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.control.BillboardControl;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Quad;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;
import com.jme3.texture.Image;
import com.jme3.texture.Texture2D;
import com.jme3.texture.TextureCubeMap;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.SkyFactory;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.engine.GeneratedWorld;

import com.jme3.input.KeyInput;
import com.jme3.input.controls.KeyTrigger;
import se.hirt.searobots.engine.SubmarineSnapshot;

import java.awt.*;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * 3D submarine scene using jMonkeyEngine, embeddable in a Swing panel
 * via {@link #getCanvas()}.
 */
public final class SubmarineScene3D extends SimpleApplication {

    private Node modelNode; // template, loaded at init
    private Geometry terrainGeometry;
    private Spatial sky;
    private Geometry sunBillboard;
    private volatile GeneratedWorld pendingWorld;

    // Vehicle tracking
    private final Map<Integer, Node> subNodes = new HashMap<>();
    private volatile List<SubmarineSnapshot> latestSnapshots = List.of();
    private volatile long latestTick;
    private int selectedSubId = 0;
    private static final float LERP_SPEED = 8f; // higher = snappier tracking
    private static final float SURFACE_SPEED = 2f; // control surface actuator lerp rate
    private BitmapText hudText;
    private BitmapText keysText;

    // 3D overlays: trails, waypoints, routes
    private boolean showTrails = true;
    private boolean showWaypoints = true;
    private boolean showRoute = true;
    private final Map<Integer, java.util.Deque<Vector3f>> trailBuffers = new HashMap<>();
    private final Map<Integer, Node> trailNodes = new HashMap<>();
    private final Map<Integer, Node> waypointNodes = new HashMap<>();
    private final Map<Integer, Node> routeNodes = new HashMap<>();
    private static final int MAX_TRAIL_POINTS = 500; // match 2D viewer
    private static final int TRAIL_SAMPLE_INTERVAL = 1; // every tick, like 2D
    private final Map<Integer, java.util.List<Vector3f>> routeBuffers = new HashMap<>();
    private static final int ROUTE_SAMPLE_INTERVAL = 100; // ~2 seconds between samples
    private long lastTrailTick = -1;

    // Firing solution crosshair
    private Geometry crosshairGeom;
    private Node crosshairNode;

    // Lighting and atmosphere
    private DirectionalLight sun;
    private DirectionalLight fill;
    private AmbientLight ambient;
    private FogFilter fogFilter;
    private WaterFilter waterFilter;
    private LightScatteringFilter godRaysFilter;
    private FilterPostProcessor fpp;
    private volatile boolean atmosphereEnabled = true;

    // Sun simulation: time of day = config.startTime + elapsed tick time
    private java.time.LocalTime startTime = java.time.LocalTime.NOON;
    private float sunHour = 12f;

    // Camera modes
    private enum CameraMode {
        ORBIT, CHASE, TARGET, PERISCOPE, FREE_LOOK, FLY_BY;
        private static final CameraMode[] VALUES = values();
        CameraMode next() { return VALUES[(ordinal() + 1) % VALUES.length]; }
        String label() {
            return switch (this) {
                case ORBIT -> "Orbit"; case CHASE -> "Chase"; case TARGET -> "Target";
                case PERISCOPE -> "Periscope"; case FREE_LOOK -> "Free Look"; case FLY_BY -> "Fly-by";
            };
        }
    }
    private CameraMode cameraMode = CameraMode.ORBIT;

    // Orbit camera state
    private final Vector3f orbitCenter = new Vector3f();
    private float orbitAzimuth = FastMath.QUARTER_PI;
    private float orbitElevation = 0.4f;
    private float orbitDistance = 200f;
    private boolean dragging;

    // Chase camera state
    private final Vector3f chasePos = new Vector3f();
    private boolean chaseInitialized = false;

    // Free Look state
    private float freeLookAzimuth;
    private float freeLookElevation;
    private float freeLookDistance;
    private final Vector3f freeLookCenter = new Vector3f();

    // Fly-by state
    private final Vector3f flyByStation = new Vector3f();
    private static final float FLY_BY_REPOSITION_DIST = 500f;

    // Tab transition: phase 1 = rotate to face target, phase 2 = move to target
    private float transitionTimer = 0f;
    private static final float LOOK_PHASE_DURATION = 0.6f;

    // Mode transition (smooth lerp between camera positions)
    private final Vector3f modeTransFromPos = new Vector3f();
    private final Vector3f modeTransFromLookAt = new Vector3f();
    private float modeTransTimer = 0f;
    private static final float MODE_TRANS_DURATION = 0.5f;

    /** Create the app and its canvas. Call from EDT. */
    public static SubmarineScene3D create() {
        var app = new SubmarineScene3D();
        var settings = new AppSettings(true);
        settings.setWidth(1280);
        settings.setHeight(800);
        settings.setTitle("SeaRobots 3D");
        settings.setFrameRate(60);
        app.setSettings(settings);
        app.setShowSettings(false);
        app.setPauseOnLostFocus(false);
        app.createCanvas();
        return app;
    }

    /** Return the AWT canvas for embedding in Swing. */
    public Canvas getCanvas() {
        return ((JmeCanvasContext) getContext()).getCanvas();
    }

    @Override
    public void simpleInitApp() {
        System.out.println("simpleInitApp: PHASE 1 - submarine model only");
        flyCam.setEnabled(false);
        viewPort.setBackgroundColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));
        setupInput();

        // Lighting
        sun = new DirectionalLight();
        sun.setDirection(new Vector3f(-1, -1, -1).normalizeLocal());
        sun.setColor(ColorRGBA.White.mult(1.2f));
        rootNode.addLight(sun);

        fill = new DirectionalLight();
        fill.setDirection(new Vector3f(1, 0.5f, 1).normalizeLocal());
        fill.setColor(new ColorRGBA(0.3f, 0.4f, 0.5f, 1f));
        rootNode.addLight(fill);

        ambient = new AmbientLight();
        ambient.setColor(new ColorRGBA(0.3f, 0.35f, 0.4f, 1f));
        rootNode.addLight(ambient);

        // Sky (procedural cube map, no projection artifacts) - must be before water for reflections
        sky = SkyFactory.createSky(assetManager, createSkyCubeMap(), SkyFactory.EnvMapType.CubeMap);
        rootNode.attachChild(sky);

        // Sun disc (billboard in Sky bucket, renders at sky depth)
        Quad sunQuad = new Quad(400, 400);
        sunBillboard = new Geometry("SunDisc", sunQuad);
        Material sunMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        sunMat.setTexture("ColorMap", createSunGlowTexture());
        sunMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
        sunMat.getAdditionalRenderState().setDepthWrite(false);
        sunBillboard.setMaterial(sunMat);
        sunBillboard.setQueueBucket(RenderQueue.Bucket.Sky);
        sunBillboard.addControl(new BillboardControl());
        sunBillboard.setCullHint(Spatial.CullHint.Never);
        // Position along sun direction (opposite of light direction), offset to center the quad
        Vector3f sunPos = sun.getDirection().mult(-900f);
        sunBillboard.setLocalTranslation(sunPos.subtract(200, 200, 0));
        rootNode.attachChild(sunBillboard);

        // Water surface (after sky so reflections pick it up)
        fpp = new FilterPostProcessor(assetManager);
        waterFilter = new WaterFilter(rootNode, sun.getDirection().mult(-1));
        waterFilter.setWaterHeight(0f);
        waterFilter.setSpeed(0.8f);
        waterFilter.setWaveScale(0.003f);
        waterFilter.setMaxAmplitude(1.5f);
        waterFilter.setWaterColor(new ColorRGBA(0.0f, 0.18f, 0.60f, 1f));
        waterFilter.setDeepWaterColor(new ColorRGBA(0.0f, 0.14f, 0.42f, 1f));
        waterFilter.setWaterTransparency(0.1f);
        waterFilter.setColorExtinction(new Vector3f(10f, 30f, 60f));
        waterFilter.setSunScale(3f);
        waterFilter.setLightDirection(sun.getDirection());
        waterFilter.setLightColor(ColorRGBA.White);
        waterFilter.setFoamExistence(new Vector3f(0.8f, 3f, 0.5f));
        waterFilter.setFoamHardness(50f);
        waterFilter.setCausticsIntensity(0.3f);
        waterFilter.setShininess(14f);
        waterFilter.setReflectionMapSize(512);
        waterFilter.setUseRipples(true);
        waterFilter.setUseSpecular(true);
        waterFilter.setUseRefraction(true);
        // Fog (depth-dependent, updated in updateAtmosphere)
        fogFilter = new FogFilter();
        fogFilter.setFogDistance(2000f);
        fogFilter.setFogDensity(1.0f);
        fogFilter.setFogColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));
        fpp.addFilter(fogFilter);

        fpp.addFilter(waterFilter);
        viewPort.addProcessor(fpp);

        // Load submarine model
        modelNode = new Node("submarineTemplate");
        try {
            Spatial hull = assetManager.loadModel("models/submarine-hybrid.obj");
            generateSmoothNormals(hull);
            disableBackFaceCulling(hull);
            modelNode.attachChild(hull);
            // Set up pivot nodes for control surfaces (hinge at hull attachment)
            // OBJ coords: Y=fore-aft, X=left-right, Z=up-down
            setupPivotAt(modelNode, "rudderl",   new Vector3f(0f, 51f,  0.33f));
            setupPivotAt(modelNode, "rudderu",   new Vector3f(0f, 51f,  0.23f));
            setupPivotAt(modelNode, "elevatorl",  new Vector3f(10.8f, -20.55f, 0f));
            setupPivotAt(modelNode, "elevatorr", new Vector3f(-11.1f, -20.55f, 0f));
            setupPivotAt(modelNode, "Propeller",  new Vector3f(0f, 65.25f, 0.28f));
            System.out.println("Loaded submarine-hybrid.obj");
        } catch (Exception e) {
            System.err.println("Failed to load submarine model: " + e.getMessage());
            // Fallback: red box placeholder
            Box box = new Box(5, 2, 10);
            Geometry ph = new Geometry("placeholder", box);
            Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", ColorRGBA.Red);
            ph.setMaterial(mat);
            modelNode.attachChild(ph);
        }
        // Rotate so it faces along Z (model is built along Y axis)
        modelNode.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0));
        // modelNode is a template for cloning - don't attach to rootNode

        // HUD overlay - top left: sub telemetry
        hudText = new BitmapText(guiFont);
        hudText.setSize(guiFont.getCharSet().getRenderedSize());
        hudText.setColor(ColorRGBA.White);
        hudText.setLocalTranslation(10, settings.getHeight() - 10, 0);
        guiNode.attachChild(hudText);

        // HUD overlay - bottom right: keybindings
        keysText = new BitmapText(guiFont);
        keysText.setSize(guiFont.getCharSet().getRenderedSize());
        keysText.setColor(new ColorRGBA(0.7f, 0.7f, 0.7f, 1f));
        keysText.setText(
                "[1] 1x  [2] 2x  [3] 5x  [4] 10x  [5] 22x\n" +
                "[P] Pause  [N] Step  [Tab] Cycle sub  [V] Camera\n" +
                "[T] Trails  [W] Waypoints  [R] Route  [Space] New map");
        float keysWidth = keysText.getLineWidth();
        float keysHeight = keysText.getHeight();
        keysText.setLocalTranslation(settings.getWidth() - keysWidth - 10, keysHeight + 10, 0);
        guiNode.attachChild(keysText);

        // Firing solution crosshair (wireframe diamond)
        crosshairNode = new Node("crosshair");
        crosshairNode.setCullHint(Spatial.CullHint.Always); // hidden until needed
        float r = 30f; // radius
        Mesh chMesh = new Mesh();
        chMesh.setMode(Mesh.Mode.Lines);
        // Diamond shape + inner cross
        chMesh.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                0, r, 0,   r, 0, 0,   // top to right
                r, 0, 0,   0, -r, 0,  // right to bottom
                0, -r, 0,  -r, 0, 0,  // bottom to left
                -r, 0, 0,  0, r, 0,   // left to top
                -r * 0.4f, 0, 0,  r * 0.4f, 0, 0,  // horizontal tick
                0, -r * 0.4f, 0,  0, r * 0.4f, 0,   // vertical tick
        });
        chMesh.setBuffer(VertexBuffer.Type.Index, 1, new short[]{
                0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
        chMesh.updateBound();
        crosshairGeom = new Geometry("crosshairLines", chMesh);
        Material chMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        chMat.setColor("Color", ColorRGBA.Red);
        chMat.getAdditionalRenderState().setLineWidth(2f);
        crosshairGeom.setMaterial(chMat);
        crosshairNode.attachChild(crosshairGeom);
        crosshairNode.addControl(new BillboardControl());
        rootNode.attachChild(crosshairNode);

        cam.setLocation(new Vector3f(0, 15, 60));
        cam.lookAt(Vector3f.ZERO, Vector3f.UNIT_Y);
        System.out.println("simpleInitApp: PHASE 1 DONE");
    }

    @Override
    public void simpleUpdate(float tpf) {
        // Phase 3: terrain + vehicles
        GeneratedWorld w = pendingWorld;
        if (w != null) {
            pendingWorld = null;
            attachTerrain(w);
        }
        updateVehicles();
        updateCamera(tpf);
        updateHud();
        updateCrosshair();
        updateOverlays();
        // Keep sun billboard centered on camera (infinitely far away)
        if (sunBillboard != null) {
            Vector3f sunPos = cam.getLocation().add(sun.getDirection().mult(-900f));
            sunBillboard.setLocalTranslation(sunPos.subtract(200, 200, 0));
        }
        // Keep HUD anchored to current window size
        hudText.setLocalTranslation(10, cam.getHeight() - 10, 0);
        keysText.setLocalTranslation(
                cam.getWidth() - keysText.getLineWidth() - 10,
                keysText.getHeight() + 10, 0);
        sunHour = (startTime.toSecondOfDay() / 3600f + latestTick / 50f / 3600f) % 24f;
        updateAtmosphere();
    }

    /**
     * Set the world to display terrain for. Safe to call from any thread.
     * The terrain mesh will be built and attached on the next jME update tick.
     */
    public void setWorld(GeneratedWorld world) {
        System.out.println("SubmarineScene3D.setWorld() called, world=" + (world != null));
        pendingWorld = world;
    }

    public void setStartTime(java.time.LocalTime time) {
        this.startTime = time;
    }

    public java.time.LocalTime getStartTime() {
        return startTime;
    }

    public void setAtmosphereEnabled(boolean enabled) {
        atmosphereEnabled = enabled;
    }

    public WaterFilter getWaterFilter() {
        return waterFilter;
    }

    public void setGodRaysEnabled(boolean enabled) {
        if (godRaysFilter != null) godRaysFilter.setEnabled(enabled);
    }

    /** Feed simulation snapshots. Safe to call from any thread. */
    public void updateSubmarines(long tick, List<SubmarineSnapshot> snapshots) {
        latestTick = tick;
        latestSnapshots = List.copyOf(snapshots);
    }

    private void attachTerrain(GeneratedWorld world) {
        System.out.println("attachTerrain: building mesh...");

        // Remove previous terrain
        if (terrainGeometry != null) {
            terrainGeometry.removeFromParent();
            terrainGeometry = null;
        }

        TerrainMap terrain = world.terrain();
        Mesh mesh = TerrainMeshBuilder.build(terrain, 2);

        terrainGeometry = new Geometry("terrain", mesh);
        Material mat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
        mat.setBoolean("UseVertexColor", true);
        terrainGeometry.setMaterial(mat);
        rootNode.attachChild(terrainGeometry);

        // Read start time from config
        startTime = world.config().startTime();

        // Clear previous vehicle models and overlays
        for (Node n : subNodes.values()) n.removeFromParent();
        subNodes.clear();
        for (Node n : trailNodes.values()) n.removeFromParent();
        trailNodes.clear();
        trailBuffers.clear();
        for (Node n : waypointNodes.values()) n.removeFromParent();
        waypointNodes.clear();
        for (Node n : routeNodes.values()) n.removeFromParent();
        routeNodes.clear();
        routeBuffers.clear();
        lastTrailTick = -1;
        latestSnapshots = List.of();

        // Position orbit camera at first spawn
        float worldW = (float) terrain.worldWidth();
        cam.setFrustumFar(worldW * 3f);
        var spawns = world.spawnPoints();
        var spawn = spawns.isEmpty() ? new se.hirt.searobots.api.Vec3(0, 0, -50) : spawns.getFirst();
        orbitCenter.set((float) spawn.x(), (float) spawn.z(), (float) -spawn.y());
        orbitDistance = 200f;
        orbitAzimuth = FastMath.QUARTER_PI;
        orbitElevation = 0.4f;

        System.out.println("attachTerrain: done, worldWidth=" + worldW);
    }

    // ---- atmosphere ----

    private void updateAtmosphere() {
        // Sun path: equator at equinox. Sun up from 6:00 to 18:00.
        // Arc angle: 0 at east horizon (6am), PI/2 at zenith (noon), PI at west horizon (6pm)
        float sunProgress = (sunHour - 6f) / 12f; // 0..1 during daylight
        boolean sunUp = sunProgress > 0f && sunProgress < 1f;
        float arcAngle = sunProgress * FastMath.PI; // 0..PI

        // Sun elevation (0 = horizon, 1 = zenith)
        float sunElev = sunUp ? FastMath.sin(arcAngle) : 0f;

        // Sun direction: east-to-west arc through zenith
        // jME: X=east, Y=up. Light direction = where light travels (opposite of sun position)
        if (sunUp) {
            float sunX = FastMath.cos(arcAngle);  // +1 east, 0 overhead, -1 west
            float sunY = sunElev;
            var sunDir = new Vector3f(-sunX, -sunY, 0.15f).normalizeLocal();
            sun.setDirection(sunDir);
            waterFilter.setLightDirection(sunDir);
            // Place sun billboard and god rays source in sun's direction
            var sunPos = cam.getLocation().add(sunDir.mult(-4000f));
            if (godRaysFilter != null) godRaysFilter.setLightPosition(sunPos);
            if (sunBillboard != null) sunBillboard.setLocalTranslation(sunPos.subtract(200, 200, 0));
        }

        // Sun colour: white at high noon, golden below 30 deg, orange/red below 15 deg
        ColorRGBA sunColor;
        if (!sunUp) {
            sunColor = ColorRGBA.Black;
        } else if (sunElev > 0.5f) {
            // Above 30 deg: full white sunlight
            sunColor = ColorRGBA.White.mult(1.2f);
        } else if (sunElev > 0.26f) {
            // 15-30 deg: blend white to golden
            float t = (sunElev - 0.26f) / 0.24f;
            sunColor = new ColorRGBA(1.2f, 0.9f + 0.3f * t, 0.5f + 0.7f * t, 1f);
        } else {
            // 0-15 deg: blend golden to orange-red
            float t = sunElev / 0.26f;
            sunColor = new ColorRGBA(0.9f + 0.3f * t, 0.3f + 0.6f * t, 0.1f + 0.4f * t, 1f);
        }
        waterFilter.setLightColor(sunColor.mult(1.3f));

        if (!atmosphereEnabled) {
            // Full flat lighting, no fog, but still track sun direction
            sun.setColor(sunUp ? ColorRGBA.White.mult(1.2f) : ColorRGBA.Black);
            fill.setColor(new ColorRGBA(0.5f, 0.6f, 0.7f, 1f));
            ambient.setColor(new ColorRGBA(0.45f, 0.5f, 0.55f, 1f));
            fogFilter.setEnabled(false);
            viewPort.setBackgroundColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));
            return;
        }

        fogFilter.setEnabled(true);
        float camY = cam.getLocation().y; // jME Y = sim depth (negative = underwater)

        // Scale sun intensity by elevation (dimmer near horizon)
        float sunIntensity = sunUp ? Math.min(1f, sunElev * 2f) : 0f;

        if (camY < 0) {
            // Underwater: sun dims with depth, fog thickens
            float depthFactor = Math.max(0f, 1f + camY / 300f);
            sun.setColor(sunColor.mult(depthFactor));
            fill.setColor(new ColorRGBA(
                    0.5f * depthFactor * sunIntensity,
                    0.6f * depthFactor * sunIntensity,
                    0.7f * depthFactor * sunIntensity, 1f));
            ambient.setColor(new ColorRGBA(
                    0.12f + 0.23f * depthFactor * sunIntensity,
                    0.14f + 0.26f * depthFactor * sunIntensity,
                    0.20f + 0.25f * depthFactor * sunIntensity, 1f));

            float fogDist = 500f + 1500f * depthFactor;
            fogFilter.setFogDistance(fogDist);
            fogFilter.setFogDensity(1.5f + 2f * (1f - depthFactor));
            fogFilter.setFogColor(new ColorRGBA(
                    0.01f + 0.04f * depthFactor,
                    0.04f + 0.08f * depthFactor,
                    0.10f + 0.10f * depthFactor, 1f));
            viewPort.setBackgroundColor(fogFilter.getFogColor());
        } else {
            // Above water
            sun.setColor(sunColor);
            fill.setColor(new ColorRGBA(
                    0.4f * sunIntensity + 0.1f,
                    0.5f * sunIntensity + 0.1f,
                    0.6f * sunIntensity + 0.1f, 1f));
            ambient.setColor(new ColorRGBA(
                    0.15f + 0.20f * sunIntensity,
                    0.18f + 0.22f * sunIntensity,
                    0.25f + 0.20f * sunIntensity, 1f));
            fogFilter.setFogDistance(5000f);
            fogFilter.setFogDensity(0.8f);
            // Sky colour shifts with sun
            float skyBlend = sunIntensity;
            fogFilter.setFogColor(new ColorRGBA(
                    0.5f * skyBlend, 0.6f * skyBlend, 0.75f * skyBlend, 1f));
            viewPort.setBackgroundColor(new ColorRGBA(
                    0.4f * skyBlend, 0.55f * skyBlend, 0.7f * skyBlend, 1f));
        }
    }

    // ---- HUD ----

    private void updateHud() {
        var snapshots = latestSnapshots;
        var snap = snapshots.stream()
                .filter(s -> s.id() == selectedSubId).findFirst().orElse(null);
        if (snap == null) {
            hudText.setText("");
            return;
        }
        var pos = snap.pose().position();
        double hdgDeg = Math.toDegrees(snap.pose().heading());
        if (hdgDeg < 0) hdgDeg += 360;
        double pitchDeg = Math.toDegrees(snap.pose().pitch());
        double rollDeg = Math.toDegrees(snap.pose().roll());
        long tick = latestTick;
        var elapsed = java.time.Duration.ofMillis((long) (tick * 1000.0 / 50));
        var tod = startTime.plusSeconds((long) (tick / 50.0));
        hudText.setText(String.format(
                "%s  |  Speed: %.1f kn  Depth: %.0f m  Throttle: %.0f%%  HP: %d\n" +
                "Heading: %03.0f\u00b0  Pitch: %+.1f\u00b0  Roll: %+.1f\u00b0  Rudder: %+.0f%%  Planes: %+.0f%%\n" +
                "Tick: %d  Elapsed: %02d:%02d:%02d  ToD: %s  Cam: %s",
                snap.name(), snap.speed(), -pos.z(), snap.throttle() * 100, snap.hp(),
                hdgDeg, pitchDeg, rollDeg,
                snap.rudder() * 100, snap.sternPlanes() * 100,
                tick, elapsed.toHoursPart(), elapsed.toMinutesPart(), elapsed.toSecondsPart(),
                tod.toString(), cameraMode.label()));
    }

    // ---- crosshair ----

    private void updateCrosshair() {
        var snapshots = latestSnapshots;
        // Find any sub with a firing solution
        SubmarineSnapshot attacker = null;
        for (var snap : snapshots) {
            if (snap.firingSolution() != null) {
                attacker = snap;
                break;
            }
        }
        if (attacker == null) {
            crosshairNode.setCullHint(Spatial.CullHint.Always);
            return;
        }

        var sol = attacker.firingSolution();
        // Find the target sub closest to the firing solution position
        Node targetNode = null;
        double bestDist = Double.MAX_VALUE;
        for (var snap : snapshots) {
            if (snap.id() == attacker.id()) continue;
            var pos = snap.pose().position();
            double dx = pos.x() - sol.targetX();
            double dy = pos.y() - sol.targetY();
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < bestDist) {
                bestDist = dist;
                targetNode = subNodes.get(snap.id());
            }
        }
        if (targetNode == null) {
            crosshairNode.setCullHint(Spatial.CullHint.Always);
            return;
        }

        // Set crosshair color to attacker's game color
        java.awt.Color awtColor = attacker.color();
        ColorRGBA chColor = new ColorRGBA(
                awtColor.getRed() / 255f,
                awtColor.getGreen() / 255f,
                awtColor.getBlue() / 255f, 1f);
        crosshairGeom.getMaterial().setColor("Color", chColor);

        // Position crosshair at target
        crosshairNode.setLocalTranslation(targetNode.getLocalTranslation());
        crosshairNode.setCullHint(Spatial.CullHint.Never);
    }

    // ---- 3D overlays: trails, waypoints, routes ----

    private void updateOverlays() {
        var snapshots = latestSnapshots;
        long tick = latestTick;

        for (var snap : snapshots) {
            int id = snap.id();
            var pos = snap.pose().position();
            Vector3f jmePos = new Vector3f((float) pos.x(), (float) pos.z(), (float) -pos.y());

            // Sub colour
            java.awt.Color awtColor = snap.color();
            ColorRGBA color = new ColorRGBA(
                    awtColor.getRed() / 255f, awtColor.getGreen() / 255f,
                    awtColor.getBlue() / 255f, 1f);

            // --- Trails ---
            var trail = trailBuffers.computeIfAbsent(id, k -> new java.util.ArrayDeque<>());
            if (tick > lastTrailTick && tick % TRAIL_SAMPLE_INTERVAL == 0) {
                trail.addLast(jmePos.clone());
                if (trail.size() > MAX_TRAIL_POINTS) trail.removeFirst();
            }

            Node trailNode = trailNodes.get(id);
            if (trailNode == null) {
                trailNode = new Node("trail-" + id);
                trailNodes.put(id, trailNode);
                rootNode.attachChild(trailNode);
            }
            trailNode.setCullHint(showTrails ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            if (showTrails && trail.size() >= 2 && tick % 10 == 0) {
                trailNode.detachAllChildren();
                float[] positions = new float[trail.size() * 3];
                float[] colors = new float[trail.size() * 4];
                int i = 0;
                int total = trail.size();
                for (Vector3f p : trail) {
                    float alpha = (float) i / total * 0.6f; // 0 = old (faded), 0.6 = newest (match 2D)
                    positions[i * 3] = p.x;
                    positions[i * 3 + 1] = p.y;
                    positions[i * 3 + 2] = p.z;
                    float glow = 2f; // overbright for additive glow
                    colors[i * 4] = color.r * glow;
                    colors[i * 4 + 1] = color.g * glow;
                    colors[i * 4 + 2] = color.b * glow;
                    colors[i * 4 + 3] = alpha;
                    i++;
                }
                Mesh trailMesh = new Mesh();
                trailMesh.setMode(Mesh.Mode.LineStrip);
                trailMesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
                trailMesh.setBuffer(VertexBuffer.Type.Color, 4, colors);
                trailMesh.updateBound();
                Geometry trailGeom = new Geometry("trailLine-" + id, trailMesh);
                Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                mat.setBoolean("VertexColor", true);
                mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
                mat.getAdditionalRenderState().setDepthWrite(false);
                trailGeom.setMaterial(mat);
                trailGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
                trailGeom.setCullHint(Spatial.CullHint.Never);
                trailNode.attachChild(trailGeom);
            }

            // --- Waypoints ---
            Node wpNode = waypointNodes.get(id);
            if (wpNode == null) {
                wpNode = new Node("waypoints-" + id);
                waypointNodes.put(id, wpNode);
                rootNode.attachChild(wpNode);
            }
            wpNode.setCullHint(showWaypoints ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            if (showWaypoints) {
                wpNode.detachAllChildren();
                var waypoints = snap.waypoints();
                if (!waypoints.isEmpty()) {
                    // Spline connecting waypoints (like 2D Catmull-Rom, simplified to line strip)
                    if (waypoints.size() >= 2) {
                        float[] spos = new float[waypoints.size() * 3];
                        for (int j = 0; j < waypoints.size(); j++) {
                            var wp = waypoints.get(j);
                            spos[j * 3] = (float) wp.x();
                            spos[j * 3 + 1] = (float) wp.z();
                            spos[j * 3 + 2] = (float) -wp.y();
                        }
                        Mesh splineMesh = new Mesh();
                        splineMesh.setMode(Mesh.Mode.LineStrip);
                        splineMesh.setBuffer(VertexBuffer.Type.Position, 3, spos);
                        splineMesh.updateBound();
                        Geometry splineGeom = new Geometry("wpSpline", splineMesh);
                        Material spMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                        spMat.setColor("Color", new ColorRGBA(color.r * 0.5f, color.g * 0.5f, color.b * 0.5f, 1f));
                        spMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
                        spMat.getAdditionalRenderState().setDepthWrite(false);
                        splineGeom.setMaterial(spMat);
                        splineGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
                        splineGeom.setCullHint(Spatial.CullHint.Never);
                        wpNode.attachChild(splineGeom);
                    }

                    // Waypoint markers: all shown, active one larger and white
                    for (var wp : waypoints) {
                        float wpX = (float) wp.x();
                        float wpY = (float) wp.z();
                        float wpZ = (float) -wp.y();
                        boolean active = wp.active();
                        float s = active ? 20f : 10f;

                        // Diamond marker
                        Mesh dm = new Mesh();
                        dm.setMode(Mesh.Mode.Lines);
                        dm.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                                0, s, 0,  s, 0, 0,  s, 0, 0,  0, -s, 0,
                                0, -s, 0, -s, 0, 0, -s, 0, 0,  0, s, 0});
                        dm.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0,1,2,3,4,5,6,7});
                        dm.updateBound();
                        Geometry wpGeom = new Geometry("wp", dm);
                        Material wpMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                        wpMat.setColor("Color", active ? ColorRGBA.White : color);
                        wpGeom.setMaterial(wpMat);
                        wpGeom.setLocalTranslation(wpX, wpY, wpZ);
                        wpGeom.addControl(new BillboardControl());
                        wpGeom.setCullHint(Spatial.CullHint.Never);
                        wpNode.attachChild(wpGeom);

                        // Active waypoint: extra outer ring
                        if (active) {
                            float outerS = s * 1.5f;
                            Mesh ring = new Mesh();
                            ring.setMode(Mesh.Mode.Lines);
                            ring.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                                    0, outerS, 0,  outerS, 0, 0,  outerS, 0, 0,  0, -outerS, 0,
                                    0, -outerS, 0, -outerS, 0, 0, -outerS, 0, 0,  0, outerS, 0});
                            ring.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0,1,2,3,4,5,6,7});
                            ring.updateBound();
                            Geometry ringGeom = new Geometry("wpRing", ring);
                            Material ringMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                            ringMat.setColor("Color", new ColorRGBA(1f, 1f, 1f, 0.4f));
                            ringMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
                            ringGeom.setMaterial(ringMat);
                            ringGeom.setLocalTranslation(wpX, wpY, wpZ);
                            ringGeom.addControl(new BillboardControl());
                            ringGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
                            ringGeom.setCullHint(Spatial.CullHint.Never);
                            wpNode.attachChild(ringGeom);
                        }
                    }
                }
            }

            // --- Route ---
            Node rtNode = routeNodes.get(id);
            if (rtNode == null) {
                rtNode = new Node("route-" + id);
                routeNodes.put(id, rtNode);
                rootNode.attachChild(rtNode);
            }
            rtNode.setCullHint(showRoute ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            // Sample route history (less frequent than trails, full match history)
            var route = routeBuffers.computeIfAbsent(id, k -> new java.util.ArrayList<>());
            if (tick > lastTrailTick && tick % ROUTE_SAMPLE_INTERVAL == 0) {
                route.add(jmePos.clone());
            }

            if (showRoute && route.size() >= 2 && tick % 10 == 0) {
                rtNode.detachAllChildren();
                int n = route.size();
                float w = 4f; // half-width of the duct

                // Build a rectangular duct: 4 walls (top, bottom, left, right)
                // Cross-section in YZ plane (perpendicular to travel in XZ)
                // Each offset pair: (dY1, dZ1, dY2, dZ2)
                float[][] offsets = {
                    {+w, -w, +w, +w},  // top wall:    top-left to top-right
                    {-w, -w, -w, +w},  // bottom wall: bottom-left to bottom-right
                    {-w, -w, +w, -w},  // left wall:   bottom-left to top-left
                    {-w, +w, +w, +w},  // right wall:  bottom-right to top-right
                };

                Material rtMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                rtMat.setColor("Color", color);
                rtMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);

                for (float[] off : offsets) {
                    float[] rpos = new float[n * 2 * 3];
                    for (int j = 0; j < n; j++) {
                        var rp = route.get(j);
                        rpos[j * 6]     = rp.x;
                        rpos[j * 6 + 1] = rp.y + off[0];
                        rpos[j * 6 + 2] = rp.z + off[1];
                        rpos[j * 6 + 3] = rp.x;
                        rpos[j * 6 + 4] = rp.y + off[2];
                        rpos[j * 6 + 5] = rp.z + off[3];
                    }
                    Mesh wall = new Mesh();
                    wall.setMode(Mesh.Mode.TriangleStrip);
                    wall.setBuffer(VertexBuffer.Type.Position, 3, rpos);
                    wall.updateBound();
                    Geometry wallGeom = new Geometry("routeWall-" + id, wall);
                    wallGeom.setMaterial(rtMat);
                    wallGeom.setQueueBucket(RenderQueue.Bucket.Opaque);
                    wallGeom.setCullHint(Spatial.CullHint.Never);
                    rtNode.attachChild(wallGeom);
                }
            }
        }
        lastTrailTick = tick;
    }

    // ---- vehicle tracking ----

    private void updateVehicles() {
        var snapshots = latestSnapshots;
        if (snapshots.isEmpty()) return;

        float tpf = timer.getTimePerFrame();
        float lerpFactor = Math.min(1f, tpf * LERP_SPEED);

        for (var snap : snapshots) {
            Node subNode = subNodes.get(snap.id());

            // Target position: sim (X,Y,Z) -> jME (X, Z, -Y)
            var pos = snap.pose().position();
            float jmeX = (float) pos.x();
            float jmeY = (float) pos.z();
            float jmeZ = (float) -pos.y();
            var targetPos = new Vector3f(jmeX, jmeY, jmeZ);

            // Target orientation
            // jME fromAngles(pitch, yaw, roll): positive X pitch = nose down
            // sim pitch: positive = nose up, so negate for jME
            float heading = (float) -snap.pose().heading() + FastMath.PI;
            float pitch = (float) snap.pose().pitch();
            float roll = (float) snap.pose().roll();
            var targetRot = new Quaternion().fromAngles(
                    -FastMath.HALF_PI - pitch, heading, roll);

            if (subNode == null) {
                // Create a new sub model by cloning the template
                subNode = (Node) modelNode.deepClone();
                subNode.setName("sub-" + snap.id());
                subNodes.put(snap.id(), subNode);
                rootNode.attachChild(subNode);
                // Snap to position on first appearance
                subNode.setLocalTranslation(targetPos);
                subNode.setLocalRotation(targetRot);
                if (subNodes.size() == 1) selectedSubId = snap.id();
            } else {
                // Smoothly interpolate position and rotation
                Vector3f currentPos = subNode.getLocalTranslation();
                subNode.setLocalTranslation(currentPos.interpolateLocal(targetPos, lerpFactor));

                Quaternion currentRot = subNode.getLocalRotation();
                currentRot.slerp(targetRot, lerpFactor);
                subNode.setLocalRotation(currentRot);
            }

            // Spin propeller based on throttle
            Spatial prop = findChild(subNode, "Propeller");
            if (prop != null) {
                float spinRate = (float) snap.throttle() * tpf * 8f;
                prop.rotate(0, spinRate, 0);
            }

            // Animate control surfaces with inertia (slerp toward target)
            // ~3 deg/s actuator rate: full throw in ~8 seconds
            float surfaceLerp = Math.min(1f, tpf * SURFACE_SPEED);

            float rudderAngle = (float) snap.rudder() * 0.4f;  // +/- 0.4 rad (~23 deg)
            float elevAngle = (float) -snap.sternPlanes() * 0.3f; // +/- 0.3 rad (~17 deg)
            var targetRudder = new Quaternion().fromAngles(0, 0, rudderAngle);
            var targetElev   = new Quaternion().fromAngles(elevAngle, 0, 0);

            Spatial ru = findChild(subNode, "rudderu");
            Spatial rl = findChild(subNode, "rudderl");
            if (ru != null) { ru.getLocalRotation().slerp(targetRudder, surfaceLerp); }
            if (rl != null) { rl.getLocalRotation().slerp(targetRudder, surfaceLerp); }

            Spatial el = findChild(subNode, "elevatorl");
            Spatial er = findChild(subNode, "elevatorr");
            if (el != null) { el.getLocalRotation().slerp(targetElev, surfaceLerp); }
            if (er != null) { er.getLocalRotation().slerp(targetElev, surfaceLerp); }
        }

        // Update orbit center tracking (used by Orbit mode and as fallback)
        Node selected = subNodes.get(selectedSubId);
        if (selected != null) {
            Vector3f targetPos = selected.getLocalTranslation();
            if (transitionTimer > 0) {
                transitionTimer -= tpf;
                Vector3f toTarget = targetPos.subtract(orbitCenter);
                float targetAzimuth = FastMath.atan2(toTarget.x, toTarget.z);
                float angleDiff = targetAzimuth - orbitAzimuth;
                while (angleDiff > FastMath.PI) angleDiff -= FastMath.TWO_PI;
                while (angleDiff < -FastMath.PI) angleDiff += FastMath.TWO_PI;
                orbitAzimuth += angleDiff * Math.min(1f, tpf * 5f);
            } else {
                float dist = orbitCenter.distance(targetPos);
                float speed = dist > 100f ? 3f : 5f;
                float factor = Math.min(1f, tpf * speed);
                if (dist > 10f) factor = Math.max(factor, 0.05f);
                orbitCenter.interpolateLocal(targetPos, factor);
            }
        }
    }

    // ---- camera system ----

    private SubmarineSnapshot findSnapshot(int id) {
        for (var snap : latestSnapshots) {
            if (snap.id() == id) return snap;
        }
        return null;
    }

    private void setupInput() {
        // Tab to cycle between submarines
        inputManager.addMapping("CycleSub", new KeyTrigger(KeyInput.KEY_TAB));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (isPressed && !latestSnapshots.isEmpty()) {
                var ids = latestSnapshots.stream().mapToInt(SubmarineSnapshot::id).toArray();
                int currentIdx = 0;
                for (int i = 0; i < ids.length; i++) {
                    if (ids[i] == selectedSubId) { currentIdx = i; break; }
                }
                selectedSubId = ids[(currentIdx + 1) % ids.length];
                chaseInitialized = false;
                if (cameraMode == CameraMode.ORBIT) {
                    transitionTimer = LOOK_PHASE_DURATION;
                } else {
                    modeTransFromPos.set(cam.getLocation());
                    modeTransFromLookAt.set(orbitCenter);
                    modeTransTimer = MODE_TRANS_DURATION;
                }
                System.out.println("Tracking: " + latestSnapshots.stream()
                        .filter(s -> s.id() == selectedSubId)
                        .map(SubmarineSnapshot::name).findFirst().orElse("?"));
            }
        }, "CycleSub");

        // V to cycle camera modes
        inputManager.addMapping("CycleCamera", new KeyTrigger(KeyInput.KEY_V));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (isPressed) {
                modeTransFromPos.set(cam.getLocation());
                modeTransFromLookAt.set(orbitCenter);
                cameraMode = cameraMode.next();
                modeTransTimer = MODE_TRANS_DURATION;
                chaseInitialized = false;
                if (cameraMode == CameraMode.FREE_LOOK) {
                    freeLookAzimuth = orbitAzimuth;
                    freeLookElevation = orbitElevation;
                    freeLookDistance = orbitDistance;
                    freeLookCenter.set(orbitCenter);
                }
                if (cameraMode == CameraMode.FLY_BY) {
                    pickFlyByStation();
                }
                System.out.println("Camera: " + cameraMode.label());
            }
        }, "CycleCamera");

        // T/W/R to toggle overlays
        inputManager.addMapping("ToggleTrails", new KeyTrigger(KeyInput.KEY_T));
        inputManager.addMapping("ToggleWaypoints", new KeyTrigger(KeyInput.KEY_W));
        inputManager.addMapping("ToggleRoute", new KeyTrigger(KeyInput.KEY_R));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            switch (name) {
                case "ToggleTrails" -> showTrails = !showTrails;
                case "ToggleWaypoints" -> showWaypoints = !showWaypoints;
                case "ToggleRoute" -> showRoute = !showRoute;
            }
        }, "ToggleTrails", "ToggleWaypoints", "ToggleRoute");

        // Mouse orbit/zoom (Orbit and Free Look modes)
        inputManager.addMapping("OrbitLeft", new MouseAxisTrigger(MouseInput.AXIS_X, true));
        inputManager.addMapping("OrbitRight", new MouseAxisTrigger(MouseInput.AXIS_X, false));
        inputManager.addMapping("OrbitUp", new MouseAxisTrigger(MouseInput.AXIS_Y, false));
        inputManager.addMapping("OrbitDown", new MouseAxisTrigger(MouseInput.AXIS_Y, true));
        inputManager.addMapping("ZoomIn", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, false));
        inputManager.addMapping("ZoomOut", new MouseAxisTrigger(MouseInput.AXIS_WHEEL, true));
        inputManager.addMapping("DragStart", new MouseButtonTrigger(MouseInput.BUTTON_LEFT));

        inputManager.addListener((ActionListener) (name, isPressed, tpf) ->
                dragging = isPressed, "DragStart");

        inputManager.addListener((AnalogListener) (name, value, tpf) -> {
            if (cameraMode != CameraMode.ORBIT && cameraMode != CameraMode.FREE_LOOK) return;
            boolean fl = cameraMode == CameraMode.FREE_LOOK;
            if (dragging) {
                float spd = 2.5f;
                switch (name) {
                    case "OrbitLeft" -> { if (fl) freeLookAzimuth -= value * spd; else orbitAzimuth -= value * spd; }
                    case "OrbitRight" -> { if (fl) freeLookAzimuth += value * spd; else orbitAzimuth += value * spd; }
                    case "OrbitUp" -> {
                        float v = (fl ? freeLookElevation : orbitElevation) + value * spd;
                        v = Math.min(FastMath.HALF_PI - 0.01f, v);
                        if (fl) freeLookElevation = v; else orbitElevation = v;
                    }
                    case "OrbitDown" -> {
                        float v = (fl ? freeLookElevation : orbitElevation) - value * spd;
                        v = Math.max(-0.2f, v);
                        if (fl) freeLookElevation = v; else orbitElevation = v;
                    }
                }
            }
            switch (name) {
                case "ZoomIn" -> { if (fl) freeLookDistance = Math.max(20f, freeLookDistance * 0.9f); else orbitDistance = Math.max(20f, orbitDistance * 0.9f); }
                case "ZoomOut" -> { if (fl) freeLookDistance *= 1.1f; else orbitDistance *= 1.1f; }
            }
        }, "OrbitLeft", "OrbitRight", "OrbitUp", "OrbitDown", "ZoomIn", "ZoomOut");
    }

    private void updateCamera(float tpf) {
        Vector3f desiredPos = new Vector3f();
        Vector3f desiredLookAt = new Vector3f();
        computeCameraForMode(desiredPos, desiredLookAt, tpf);

        if (modeTransTimer > 0) {
            modeTransTimer -= tpf;
            float t = 1f - Math.max(0f, modeTransTimer / MODE_TRANS_DURATION);
            t = t * t * (3f - 2f * t); // smoothstep
            Vector3f pos = new Vector3f(modeTransFromPos).interpolateLocal(desiredPos, t);
            Vector3f look = new Vector3f(modeTransFromLookAt).interpolateLocal(desiredLookAt, t);
            cam.setLocation(pos);
            cam.lookAt(look, Vector3f.UNIT_Y);
        } else {
            cam.setLocation(desiredPos);
            cam.lookAt(desiredLookAt, Vector3f.UNIT_Y);
        }
    }

    private void computeCameraForMode(Vector3f outPos, Vector3f outLookAt, float tpf) {
        switch (cameraMode) {
            case ORBIT     -> computeOrbitCamera(outPos, outLookAt);
            case CHASE     -> computeChaseCamera(outPos, outLookAt, tpf);
            case TARGET    -> computeTargetCamera(outPos, outLookAt, tpf);
            case PERISCOPE -> computePeriscopeCamera(outPos, outLookAt);
            case FREE_LOOK -> computeFreeLookCamera(outPos, outLookAt, tpf);
            case FLY_BY    -> computeFlyByCamera(outPos, outLookAt, tpf);
        }
    }

    private void computeOrbitCamera(Vector3f outPos, Vector3f outLookAt) {
        outPos.set(
            orbitCenter.x + orbitDistance * FastMath.cos(orbitElevation) * FastMath.sin(orbitAzimuth),
            orbitCenter.y + orbitDistance * FastMath.sin(orbitElevation),
            orbitCenter.z + orbitDistance * FastMath.cos(orbitElevation) * FastMath.cos(orbitAzimuth));
        outLookAt.set(orbitCenter);
    }

    private void computeChaseCamera(Vector3f outPos, Vector3f outLookAt, float tpf) {
        Node sel = subNodes.get(selectedSubId);
        SubmarineSnapshot snap = findSnapshot(selectedSubId);
        if (sel == null || snap == null) { computeOrbitCamera(outPos, outLookAt); return; }
        Vector3f subPos = sel.getLocalTranslation();

        // Chase camera: behind and above the sub along its heading.
        // Slow lerp creates cinematic trailing when the sub turns.
        float heading = (float) snap.pose().heading();
        float fwdX = (float) Math.sin(heading);
        float fwdZ = (float) -Math.cos(heading);
        float asternDist = 250f;
        float aboveHeight = 50f;

        Vector3f target = new Vector3f(
                subPos.x - fwdX * asternDist,
                subPos.y + aboveHeight,
                subPos.z - fwdZ * asternDist);

        if (!chaseInitialized) { chasePos.set(target); chaseInitialized = true; }

        // Slow lerp: camera trails behind, swinging wide on turns
        chasePos.interpolateLocal(target, Math.min(1f, tpf * 1.5f));

        outPos.set(chasePos);
        outLookAt.set(subPos);
    }

    private void computeTargetCamera(Vector3f outPos, Vector3f outLookAt, float tpf) {
        computeChaseCamera(outPos, outLookAt, tpf);
        SubmarineSnapshot snap = findSnapshot(selectedSubId);
        if (snap == null) return;

        // Look toward best contact instead of the sub
        var best = snap.contactEstimates().stream()
                .filter(c -> c.confidence() > 0.1)
                .max(java.util.Comparator.comparingDouble(se.hirt.searobots.api.ContactEstimate::confidence))
                .orElse(null);
        if (best != null) {
            outLookAt.set((float) best.x(), (float) snap.pose().position().z(), (float) -best.y());
        }
    }

    private void computePeriscopeCamera(Vector3f outPos, Vector3f outLookAt) {
        Node sel = subNodes.get(selectedSubId);
        SubmarineSnapshot snap = findSnapshot(selectedSubId);
        if (sel == null || snap == null) { computeOrbitCamera(outPos, outLookAt); return; }

        Vector3f subPos = sel.getLocalTranslation();
        float heading = (float) snap.pose().heading();
        float fwdX = (float) Math.sin(heading);
        float fwdZ = (float) -Math.cos(heading);
        float towerHeight = 12f;

        outPos.set(subPos.x, subPos.y + towerHeight, subPos.z);
        float lookDist = 200f;
        float downPitch = FastMath.tan(FastMath.DEG_TO_RAD * 5f);
        outLookAt.set(
                subPos.x + fwdX * lookDist,
                subPos.y + towerHeight - lookDist * downPitch,
                subPos.z + fwdZ * lookDist);
    }

    private void computeFreeLookCamera(Vector3f outPos, Vector3f outLookAt, float tpf) {
        Node sel = subNodes.get(selectedSubId);
        if (sel != null) {
            freeLookCenter.interpolateLocal(sel.getLocalTranslation(), Math.min(1f, tpf * 0.5f));
        }
        outPos.set(
            freeLookCenter.x + freeLookDistance * FastMath.cos(freeLookElevation) * FastMath.sin(freeLookAzimuth),
            freeLookCenter.y + freeLookDistance * FastMath.sin(freeLookElevation),
            freeLookCenter.z + freeLookDistance * FastMath.cos(freeLookElevation) * FastMath.cos(freeLookAzimuth));
        outLookAt.set(freeLookCenter);
    }

    private void pickFlyByStation() {
        Node sel = subNodes.get(selectedSubId);
        SubmarineSnapshot snap = findSnapshot(selectedSubId);
        if (sel == null || snap == null) return;
        Vector3f subPos = sel.getLocalTranslation();
        float heading = (float) snap.pose().heading();
        float fwdX = (float) Math.sin(heading);
        float fwdZ = (float) -Math.cos(heading);
        float perpX = fwdZ, perpZ = -fwdX;
        flyByStation.set(
                subPos.x + fwdX * 300f + perpX * 150f,
                subPos.y + 30f,
                subPos.z + fwdZ * 300f + perpZ * 150f);
    }

    private void computeFlyByCamera(Vector3f outPos, Vector3f outLookAt, float tpf) {
        Node sel = subNodes.get(selectedSubId);
        if (sel == null) { computeOrbitCamera(outPos, outLookAt); return; }
        Vector3f subPos = sel.getLocalTranslation();
        if (subPos.distance(flyByStation) > FLY_BY_REPOSITION_DIST) {
            pickFlyByStation();
        }
        outPos.set(flyByStation);
        outLookAt.set(subPos);
    }

    // ---- helpers ----

    private Spatial loadModel(String path, float x, float y, float z) {
        try {
            Spatial model = assetManager.loadModel(path);
            generateSmoothNormals(model);
            disableBackFaceCulling(model);
            model.setLocalTranslation(x, y, z);
            modelNode.attachChild(model);
            return model;
        } catch (Exception e) {
            System.err.println("Failed to load " + path + ": " + e.getMessage());
            Box box = new Box(5, 2, 10);
            Geometry ph = new Geometry(path, box);
            Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", ColorRGBA.Red);
            ph.setMaterial(mat);
            ph.setLocalTranslation(x, y, z);
            modelNode.attachChild(ph);
            return ph;
        }
    }

    private Spatial setupPivot(Node parent, Spatial part, Vector3f offset) {
        part.getParent().detachChild(part);
        Node pivot = new Node(part.getName() + "Pivot");
        Vector3f center = ((com.jme3.bounding.BoundingBox) part.getWorldBound())
                .getCenter().add(offset);
        pivot.setLocalTranslation(center);
        part.setLocalTranslation(part.getLocalTranslation().subtract(center));
        pivot.attachChild(part);
        parent.attachChild(pivot);
        return pivot;
    }

    /**
     * Reparent a named child under a pivot node at a fixed hinge position.
     * The pivot keeps the original child name so findChild() still finds it.
     */
    private void setupPivotAt(Node root, String childName, Vector3f hingePos) {
        Spatial part = findChild(root, childName);
        if (part == null) {
            System.err.println("setupPivotAt: child not found: " + childName);
            return;
        }
        Node oldParent = part.getParent();
        oldParent.detachChild(part);
        Node pivot = new Node(childName); // same name so findChild picks up the pivot
        part.setName(childName + "_mesh");
        pivot.setLocalTranslation(hingePos);
        part.setLocalTranslation(part.getLocalTranslation().subtract(hingePos));
        pivot.attachChild(part);
        oldParent.attachChild(pivot);
    }

    private Spatial findChild(Node node, String name) {
        for (Spatial child : node.getChildren()) {
            if (name.equals(child.getName())) return child;
            if (child instanceof Node n) {
                Spatial found = findChild(n, name);
                if (found != null) return found;
            }
        }
        return null;
    }

    private void generateSmoothNormals(Spatial spatial) {
        if (spatial instanceof Geometry geom) {
            Mesh mesh = geom.getMesh();
            if (mesh.getBuffer(VertexBuffer.Type.Normal) == null) {
                var posBuf = mesh.getFloatBuffer(VertexBuffer.Type.Position);
                int vertCount = posBuf.limit() / 3;
                float[] normals = new float[vertCount * 3];
                var idxBuf = mesh.getIndexBuffer();
                int triCount = idxBuf.size() / 3;
                Vector3f v0 = new Vector3f(), v1 = new Vector3f(), v2 = new Vector3f();
                Vector3f e1 = new Vector3f(), e2 = new Vector3f(), fn = new Vector3f();
                for (int t = 0; t < triCount; t++) {
                    int i0 = idxBuf.get(t * 3), i1 = idxBuf.get(t * 3 + 1), i2 = idxBuf.get(t * 3 + 2);
                    v0.set(posBuf.get(i0*3), posBuf.get(i0*3+1), posBuf.get(i0*3+2));
                    v1.set(posBuf.get(i1*3), posBuf.get(i1*3+1), posBuf.get(i1*3+2));
                    v2.set(posBuf.get(i2*3), posBuf.get(i2*3+1), posBuf.get(i2*3+2));
                    v1.subtract(v0, e1); v2.subtract(v0, e2); e1.cross(e2, fn);
                    for (int idx : new int[]{i0, i1, i2}) {
                        normals[idx*3] += fn.x; normals[idx*3+1] += fn.y; normals[idx*3+2] += fn.z;
                    }
                }
                for (int i = 0; i < vertCount; i++) {
                    float nx = normals[i*3], ny = normals[i*3+1], nz = normals[i*3+2];
                    float len = (float) Math.sqrt(nx*nx + ny*ny + nz*nz);
                    if (len > 0) { normals[i*3] /= len; normals[i*3+1] /= len; normals[i*3+2] /= len; }
                }
                mesh.setBuffer(VertexBuffer.Type.Normal, 3, BufferUtils.createFloatBuffer(normals));
                mesh.updateBound();
            }
        } else if (spatial instanceof Node node) {
            for (Spatial child : node.getChildren()) generateSmoothNormals(child);
        }
    }

    private void disableBackFaceCulling(Spatial spatial) {
        if (spatial instanceof Geometry geom) {
            Material mat = geom.getMaterial();
            if (mat != null) {
                mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
            }
        } else if (spatial instanceof Node node) {
            for (Spatial child : node.getChildren()) disableBackFaceCulling(child);
        }
    }

    private TextureCubeMap createSkyCubeMap() {
        int size = 256;
        var faces = new java.util.ArrayList<ByteBuffer>();
        for (int face = 0; face < 6; face++) {
            ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
            for (int py = 0; py < size; py++) {
                for (int px = 0; px < size; px++) {
                    float s = 2f * (px + 0.5f) / size - 1f;
                    float t = 2f * (py + 0.5f) / size - 1f;
                    Vector3f dir = cubeFaceDir(face, s, t).normalizeLocal();

                    // dir.y = elevation: +1 at zenith, -1 at nadir
                    float elevation = dir.y;

                    int r, g, b;
                    if (elevation > 0) {
                        // Sky: bright horizon to deep blue at zenith
                        r = (int) (180 - elevation * 130);
                        g = (int) (210 - elevation * 100);
                        b = (int) (245 - elevation * 30);
                    } else {
                        // Below horizon: dark
                        float d = -elevation;
                        r = (int) (180 - d * 170);
                        g = (int) (210 - d * 200);
                        b = (int) (245 - d * 220);
                    }
                    r = Math.max(0, Math.min(255, r));
                    g = Math.max(0, Math.min(255, g));
                    b = Math.max(0, Math.min(255, b));
                    buf.put((byte) r).put((byte) g).put((byte) b).put((byte) 255);
                }
            }
            buf.flip();
            faces.add(buf);
        }
        var img = new Image(Image.Format.RGBA8, size, size, 0, faces, ColorSpace.sRGB);
        return new TextureCubeMap(img);
    }

    private Vector3f cubeFaceDir(int face, float s, float t) {
        return switch (face) {
            case 0 -> new Vector3f(+1, -t, -s);  // +X
            case 1 -> new Vector3f(-1, -t, +s);  // -X
            case 2 -> new Vector3f(+s, +1, +t);  // +Y
            case 3 -> new Vector3f(+s, -1, -t);  // -Y
            case 4 -> new Vector3f(+s, -t, +1);  // +Z
            case 5 -> new Vector3f(-s, -t, -1);  // -Z
            default -> throw new IllegalArgumentException();
        };
    }

    private Texture2D createSunGlowTexture() {
        int size = 128;
        ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
        float center = size / 2f;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float dx = (x - center) / center;
                float dy = (y - center) / center;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);
                // Bright core with soft falloff
                float intensity = Math.max(0f, 1f - dist);
                intensity = intensity * intensity * intensity; // cubic falloff for soft glow
                int r = (int) (255 * intensity);
                int g = (int) (245 * intensity);
                int b = (int) (200 * intensity);
                int a = (int) (255 * intensity);
                buf.put((byte) r).put((byte) g).put((byte) b).put((byte) a);
            }
        }
        buf.flip();
        return new Texture2D(new Image(Image.Format.RGBA8, size, size, buf, ColorSpace.sRGB));
    }

    private Texture2D createBubbleTexture() {
        int size = 32;
        ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
        float center = size / 2f, radius = size / 2f;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float dx = x - center, dy = y - center;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);
                float alpha = Math.max(0, 1f - dist / radius);
                alpha *= alpha;
                buf.put((byte) 255).put((byte) 255).put((byte) 255).put((byte) (alpha * 255));
            }
        }
        buf.flip();
        return new Texture2D(new Image(Image.Format.RGBA8, size, size, buf, ColorSpace.sRGB));
    }
}
