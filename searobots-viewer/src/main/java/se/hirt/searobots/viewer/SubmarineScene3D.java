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

import com.jme3.app.SimpleApplication;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
import com.jme3.font.BitmapText;
import com.jme3.input.KeyInput;
import com.jme3.input.MouseInput;
import com.jme3.input.controls.*;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.FogFilter;
import com.jme3.post.filters.LightScatteringFilter;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.*;
import com.jme3.scene.control.BillboardControl;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Quad;
import com.jme3.system.AppSettings;
import com.jme3.texture.Image;
import com.jme3.texture.Texture2D;
import com.jme3.texture.TextureCubeMap;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.SkyFactory;
import com.jme3.water.WaterFilter;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Waypoint;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * 3D submarine scene using jMonkeyEngine. Standalone application with
 * Lemur GUI, 2D map overlay, and full keyboard/mouse controls.
 */
public final class SubmarineScene3D extends SimpleApplication implements se.hirt.searobots.engine.SimulationListener {

    private Node modelNode; // template, loaded at init
    private Geometry terrainGeometry;
    private Spatial sky;
    private Geometry sunBillboard;
    private volatile GeneratedWorld pendingWorld;

    // Vehicle tracking
    private final Map<Integer, Node> subNodes = new HashMap<>();
    // Torpedo 3D state
    private Node torpedoModelNode; // template, loaded at init
    private final Map<Integer, Node> torpedoNodes = new HashMap<>();
    private final Map<Integer, Geometry> torpedoCollisionGeoms = new HashMap<>();
    private volatile List<TorpedoSnapshot> latestTorpedoSnapshots = List.of();

    // 3D explosion effects
    private record Explosion3D(int id, Vector3f position, float startTime, ColorRGBA color) {}
    private final java.util.List<Explosion3D> activeExplosions = new java.util.ArrayList<>();
    private final Map<Integer, Geometry> explosionGeoms = new HashMap<>(); // fireball sphere
    private final Map<Integer, ParticleEmitter> debrisEmitters = new HashMap<>(); // debris particles
    private final Map<Integer, ParticleEmitter> deathBubbles = new HashMap<>(); // sub death bubbles
    private final Map<Integer, ParticleEmitter> torpedoBubbles = new HashMap<>(); // torpedo wake
    private final Map<Integer, Float> deathPropSpin = new HashMap<>(); // decaying prop spin rate
    private float appTime = 0; // running time for animations
    private static final float EXPLOSION_DURATION = 3.5f; // seconds
    private static final float EXPLOSION_MAX_RADIUS = 80f;

    // Torpedo collision cylinder dimensions (visual, shrunk from physical hull)
    // Torpedo visual cylinder matches physics: VehicleConfig.torpedo() hullHalfLength/hullHalfBeam
    private static final float TORP_HALF_LENGTH = 2.5f;   // 5m total (matches physics)
    private static final float TORP_RADIUS = 0.25f;       // 0.5m diameter (matches physics)
    private final java.util.Set<Integer> knownTorpedoIds3D = new java.util.HashSet<>();
    // Torpedo intercept marker (3D diamond at published target)
    private Geometry interceptMarker;

    // Cinematic director (automatic camera)
    private CinematicDirector cinematicDirector;
    private volatile List<SubmarineSnapshot> latestSnapshots = List.of();
    private volatile long latestTick;
    private int selectedSubId = 0;
    private static final float LERP_SPEED = 8f; // higher = snappier tracking
    private static final float SURFACE_SPEED = 2f; // control surface actuator lerp rate
    private BitmapText hudText;
    private BitmapText tacticalText;
    private BitmapText keysText;
    private BitmapText loadingText;
    private BitmapText speedText;
    private BitmapText toggleStatusText;
    private BitmapText competitionScoreText;  // compact score at top
    private BitmapText competitionPhaseText;  // current phase label below score
    private BitmapText competitionDetailText; // detailed breakdown, toggleable
    private boolean showCompetitionDetails = false;
    private final java.util.concurrent.CopyOnWriteArrayList<String> competitionDetailLines = new java.util.concurrent.CopyOnWriteArrayList<>();
    private CompetitionRunner activeCompetition;
    private MapRenderer standaloneMapRenderer;
    private volatile java.util.function.Supplier<se.hirt.searobots.engine.SimulationLoop.State> simStateSupplier = () -> se.hirt.searobots.engine.SimulationLoop.State.RUNNING;

    // 3D overlays: shared config so 2D and 3D views stay in sync
    private final OverlayConfig overlayConfig = new OverlayConfig();
    private final Map<Integer, java.util.Deque<Vector3f>> trailBuffers = new HashMap<>();
    private final Map<Integer, Node> trailNodes = new HashMap<>();
    private final Map<Integer, Node> waypointNodes = new HashMap<>();  // A* nav waypoints
    private final Map<Integer, Node> strategicNodes = new HashMap<>(); // strategic waypoints
    private final Map<Integer, Node> routeNodes = new HashMap<>();
    private static final int MAX_TRAIL_POINTS = 750; // ~30 seconds at 25 samples/sec
    private static final int TRAIL_SAMPLE_INTERVAL = 2; // every 2 ticks (25 Hz)
    private final Map<Integer, java.util.List<Vector3f>> routeBuffers = new HashMap<>();
    private static final int ROUTE_SAMPLE_INTERVAL = 100; // ~2 seconds between samples

    // Debug collision ellipsoid + terrain collision points
    private boolean showCollisionEllipsoids = false;
    private final Map<Integer, Geometry> ellipsoidGeoms = new HashMap<>();
    private final Map<Integer, Geometry[]> terrainPointGeoms = new HashMap<>();
    private long lastTrailTick = -1;

    // Bubble emitters (cavitation noise visualization)
    private final Map<Integer, ParticleEmitter> bubbleEmitters = new HashMap<>();

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
    private volatile boolean atmosphereEnabled = false;
    private volatile boolean debugMode = false;

    // Sun simulation: time of day = config.startTime + elapsed tick time
    private java.time.LocalTime startTime = java.time.LocalTime.NOON;
    private float sunHour = 12f;

    // Camera modes
    private enum CameraMode {
        ORBIT, CHASE, TARGET, PERISCOPE, FREE_LOOK, FLY_BY, DIRECTOR;
        private static final CameraMode[] VALUES = values();
        CameraMode next() { return VALUES[(ordinal() + 1) % VALUES.length]; }
        String label() {
            return switch (this) {
                case ORBIT -> "Orbit"; case CHASE -> "Chase"; case TARGET -> "Target";
                case PERISCOPE -> "Periscope"; case FREE_LOOK -> "Free Look"; case FLY_BY -> "Fly-by";
                case DIRECTOR -> "Director";
            };
        }
    }
    private CameraMode cameraMode = CameraMode.DIRECTOR;

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

    /**
     * Standalone entry point: launches a full JME window (no Swing).
     * Works on macOS with -XstartOnFirstThread.
     */
    public static void main(String[] args) {
        long seed = args.length > 0
                ? Long.parseLong(args[0])
                : java.util.concurrent.ThreadLocalRandom.current().nextLong();

        var app = new SubmarineScene3D();
        app.standalone = true;

        var settings = new AppSettings(true);
        settings.setWidth(1920);
        settings.setHeight(1080);
        settings.setTitle("SeaRobots [seed: " + Long.toHexString(seed) + "]");
        settings.setFrameRate(60);
        settings.setVSync(true);
        settings.setResizable(true);
        loadIcons(settings);
        app.setSettings(settings);
        app.setShowSettings(false);
        app.setPauseOnLostFocus(false);

        // Wire up simulation
        var generator = new se.hirt.searobots.engine.WorldGenerator();
        var world = generator.generate(se.hirt.searobots.api.MatchConfig.withDefaults(seed));
        app.standaloneSeed = seed;
        app.standaloneGenerator = generator;
        app.standaloneWorld = world;

        app.start();
    }

    // Standalone mode state (null/false when embedded in Swing)
    private boolean standalone;
    volatile boolean dialogOpen; // suppresses game key mappings when a text-input dialog is open
    private long standaloneSeed;
    private se.hirt.searobots.engine.WorldGenerator standaloneGenerator;
    private volatile GeneratedWorld standaloneWorld;
    private SimulationManager standaloneSimManager;

    @Override
    public void simpleInitApp() {
        System.out.println("simpleInitApp: PHASE 1 - submarine model only");
        setDisplayStatView(false);
        setDisplayFps(false);
        flyCam.setEnabled(false);
        viewPort.setBackgroundColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));
        setupInput();

        // Initialize Lemur GUI (must happen before any Lemur widget creation)
        // Note: Glass style requires Groovy which doesn't support Java 25 yet,
        // so we use Lemur's default style instead.
        if (standalone) {
            com.simsilica.lemur.GuiGlobals.initialize(this);
        }

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
        waterFilter.setWaterTransparency(0.09f);
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
            // Scaled model: 75m x 12m (sx=0.4, sy=0.567, sz=0.4 from original)
            setupPivotAt(modelNode, "rudderl",   new Vector3f(0f, 34f, 0.13f));
            setupPivotAt(modelNode, "rudderu",   new Vector3f(0f, 34f, 0.09f));
            setupPivotAt(modelNode, "elevatorl",  new Vector3f(4.3f, -10f, 0f));  // under tower center
            setupPivotAt(modelNode, "elevatorr", new Vector3f(-4.4f, -10f, 0f));  // under tower center
            setupPivotAt(modelNode, "Propeller",  new Vector3f(0f, 37f, 0.11f));
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

        // Load torpedo model template (scaled to ~5m, sub is ~75m so torpedo is ~1/15th)
        torpedoModelNode = new Node("torpedoTemplate");
        try {
            Spatial torpHull = assetManager.loadModel("models/torpedo.obj");
            torpHull.setLocalScale(0.19f); // scaled to match physics (5m length)
            // Center the model: Y origin is at 1.57 in model units, shift to geometric center
            torpHull.setLocalTranslation(0, -1.57f * 0.19f, 0);
            // Disable backface culling on all geometries (propeller visible from both sides)
            torpHull.depthFirstTraversal(spatial -> {
                if (spatial instanceof Geometry g) {
                    g.getMaterial().getAdditionalRenderState().setFaceCullMode(
                            com.jme3.material.RenderState.FaceCullMode.Off);
                }
            });
            torpedoModelNode.attachChild(torpHull);
            System.out.println("Loaded torpedo.obj (scale 0.07)");
        } catch (Exception e) {
            // Fallback: small yellow elongated sphere
            var cyl = new com.jme3.scene.shape.Sphere(8, 8, 1f);
            Geometry ph = new Geometry("torpPlaceholder", cyl);
            Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", ColorRGBA.Yellow);
            ph.setMaterial(mat);
            ph.setLocalScale(0.25f, 2.5f, 0.25f);
            torpedoModelNode.attachChild(ph);
            System.out.println("Using torpedo placeholder geometry");
        }
        torpedoModelNode.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0));

        // HUD overlay - top left: sub telemetry
        hudText = new BitmapText(guiFont);
        hudText.setSize(guiFont.getCharSet().getRenderedSize());
        hudText.setColor(ColorRGBA.White);
        hudText.setLocalTranslation(10, settings.getHeight() - 10, 0);
        guiNode.attachChild(hudText);

        // HUD overlay - top right: tactical info
        tacticalText = new BitmapText(guiFont);
        tacticalText.setSize(guiFont.getCharSet().getRenderedSize());
        tacticalText.setColor(new ColorRGBA(0.9f, 0.95f, 1.0f, 1f));
        // Position updated dynamically in updateHud() to track window resize
        guiNode.attachChild(tacticalText);

        // HUD overlay - center: loading indicator
        loadingText = new BitmapText(guiFont);
        loadingText.setSize(guiFont.getCharSet().getRenderedSize() * 2);
        loadingText.setColor(new ColorRGBA(0.5f, 0.8f, 1.0f, 0.9f));
        loadingText.setText("INITIALIZING...");
        loadingText.setCullHint(Spatial.CullHint.Always); // hidden by default
        guiNode.attachChild(loadingText);

        // HUD overlay - top center: speed indicator
        speedText = new BitmapText(guiFont);
        speedText.setSize(guiFont.getCharSet().getRenderedSize() * 1.5f);
        speedText.setColor(new ColorRGBA(1f, 1f, 1f, 0.9f));
        speedText.setText("");
        guiNode.attachChild(speedText);

        // HUD overlay - bottom right: keybindings
        keysText = new BitmapText(guiFont);
        keysText.setSize(guiFont.getCharSet().getRenderedSize());
        keysText.setColor(new ColorRGBA(0.7f, 0.7f, 0.7f, 1f));
        keysText.setText(
                "[1-6] Speed  [0] Max  [P] Pause  [N] Step  [F11] Fullscreen\n" +
                "[Tab] Cycle sub  [V] Camera  [Space] New map  [Esc] Menu\n" +
                "[T] Trails  [R] Route  [E] Contacts  [W] Waypoints  [G] Strategic\n" +
                "[B] Collision  [D] Pause death  [F] Pause solution  [L] Pause launch\n" +
                "[I] Score details  [F2] Config  [F3] Render  [Ctrl+C] Copy seed");
        float keysWidth = keysText.getLineWidth();
        float keysHeight = keysText.getHeight();
        keysText.setLocalTranslation(settings.getWidth() - keysWidth - 10, keysHeight + 10, 0);
        guiNode.attachChild(keysText);

        // Toggle status indicators (above keybindings, updated each frame)
        toggleStatusText = new BitmapText(guiFont);
        toggleStatusText.setSize(guiFont.getCharSet().getRenderedSize());
        toggleStatusText.setText("");
        guiNode.attachChild(toggleStatusText);

        // Competition score (compact, centered below speed indicator)
        competitionScoreText = new BitmapText(guiFont);
        competitionScoreText.setSize(guiFont.getCharSet().getRenderedSize() * 1.3f);
        competitionScoreText.setColor(new ColorRGBA(1f, 0.9f, 0.3f, 0.95f));
        competitionScoreText.setText("");
        guiNode.attachChild(competitionScoreText);

        // Competition phase label (centered below score)
        competitionPhaseText = new BitmapText(guiFont);
        competitionPhaseText.setSize(guiFont.getCharSet().getRenderedSize());
        competitionPhaseText.setColor(ColorRGBA.White);
        competitionPhaseText.setText("");
        guiNode.attachChild(competitionPhaseText);

        // Competition details (toggleable, center-left)
        competitionDetailText = new BitmapText(guiFont);
        competitionDetailText.setSize(guiFont.getCharSet().getRenderedSize());
        competitionDetailText.setColor(new ColorRGBA(0.85f, 0.9f, 1f, 0.85f));
        competitionDetailText.setText("");
        guiNode.attachChild(competitionDetailText);

        // Firing solution crosshair (solid circle + cross, built from triangle strips)
        crosshairNode = new Node("crosshair");
        crosshairNode.setCullHint(Spatial.CullHint.Always);
        float r = 37.5f;     // circle radius
        float cr = r * 1.5f; // cross extends past circle
        float w = 1.5f;      // cross arm half-width (thickness)

        Material chMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        chMat.setColor("Color", ColorRGBA.Red);
        chMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);

        // Circle: thick solid ring (inner radius r-w, outer radius r+w)
        int segs = 48;
        float[] circPos = new float[segs * 2 * 3];
        for (int i = 0; i < segs; i++) {
            float a = (float) (2 * Math.PI * i / (segs - 1));
            float cos = FastMath.cos(a), sin = FastMath.sin(a);
            circPos[i * 6]     = (r - w) * cos;
            circPos[i * 6 + 1] = (r - w) * sin;
            circPos[i * 6 + 2] = 0;
            circPos[i * 6 + 3] = (r + w) * cos;
            circPos[i * 6 + 4] = (r + w) * sin;
            circPos[i * 6 + 5] = 0;
        }
        Mesh circMesh = new Mesh();
        circMesh.setMode(Mesh.Mode.TriangleStrip);
        circMesh.setBuffer(VertexBuffer.Type.Position, 3, circPos);
        circMesh.updateBound();
        Geometry circGeom = new Geometry("chCircle", circMesh);
        circGeom.setMaterial(chMat);
        crosshairNode.attachChild(circGeom);

        // Cross arms: 4 filled quads (each as 2 triangles)
        float[][] arms = {
            {-w, r*0.7f, w, r*0.7f, w, cr, -w, cr},       // top
            {-w, -cr, w, -cr, w, -r*0.7f, -w, -r*0.7f},   // bottom
            {r*0.7f, -w, r*0.7f, w, cr, w, cr, -w},        // right
            {-cr, -w, -cr, w, -r*0.7f, w, -r*0.7f, -w},   // left
        };
        for (float[] arm : arms) {
            Mesh armMesh = new Mesh();
            armMesh.setBuffer(VertexBuffer.Type.Position, 3, new float[]{
                arm[0], arm[1], 0,  arm[2], arm[3], 0,
                arm[4], arm[5], 0,  arm[6], arm[7], 0
            });
            armMesh.setBuffer(VertexBuffer.Type.Index, 1, new short[]{0, 1, 2, 0, 2, 3});
            armMesh.updateBound();
            Geometry armGeom = new Geometry("chArm", armMesh);
            armGeom.setMaterial(chMat);
            crosshairNode.attachChild(armGeom);
        }

        crosshairGeom = circGeom; // reference for color changes
        crosshairNode.addControl(new BillboardControl());
        rootNode.attachChild(crosshairNode);

        // Torpedo intercept marker (small diamond wireframe)
        {
            var diamond = new com.jme3.scene.shape.Sphere(4, 4, 3f);
            interceptMarker = new Geometry("interceptMarker", diamond);
            Material iMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            iMat.setColor("Color", new ColorRGBA(1f, 0.4f, 0.1f, 0.8f));
            iMat.getAdditionalRenderState().setWireframe(true);
            iMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
            interceptMarker.setMaterial(iMat);
            interceptMarker.setQueueBucket(RenderQueue.Bucket.Transparent);
            interceptMarker.setCullHint(Spatial.CullHint.Always);
            rootNode.attachChild(interceptMarker);
        }

        cam.setLocation(new Vector3f(0, 15, 60));
        cam.lookAt(Vector3f.ZERO, Vector3f.UNIT_Y);
        System.out.println("simpleInitApp: PHASE 1 DONE");

        // Standalone mode: auto-start simulation and register Lemur AppStates
        if (standalone && standaloneWorld != null) {
            setWorld(standaloneWorld);
            standaloneSimManager = new SimulationManager();
            standaloneSimManager.addListener(this);
            var controllers = SimConfigState.currentControllers();
            var vehicleConfigs = SimConfigState.currentVehicleConfigs();
            if (!controllers.isEmpty()) {
                standaloneSimManager.start(standaloneWorld, controllers, vehicleConfigs);
                standaloneSimManager.play();
            }

            // Register 2D map view with speed/pause suppliers
            standaloneMapRenderer = new MapRenderer(standaloneWorld, overlayConfig);
            var mapRenderer = standaloneMapRenderer;
            mapRenderer.setSimSpeedSupplier(() -> {
                var sim = standaloneSimManager.currentLoop();
                return sim != null ? sim.getSpeedMultiplier() : 1.0;
            });
            mapRenderer.setSimPausedSupplier(() -> {
                var sim = standaloneSimManager.currentLoop();
                return sim != null && sim.isPaused();
            });
            mapRenderer.setSimStateSupplier(() -> {
                var sim = standaloneSimManager.currentLoop();
                return sim != null ? sim.getState() : se.hirt.searobots.engine.SimulationLoop.State.STOPPED;
            });
            standaloneSimManager.addListener(mapRenderer);
            var nativeMapState = new NativeMapState(mapRenderer);
            stateManager.attach(nativeMapState);

            // Register Lemur GUI AppStates (disabled by default, toggled by keys)
            var simConfigState = new SimConfigState(this::restartSim);
            simConfigState.seedSupplier = () -> standaloneSeed;
            simConfigState.onSeedChanged = seed -> standaloneSeed = seed;
            simConfigState.simManager = standaloneSimManager;
            simConfigState.scene = this;
            stateManager.attach(simConfigState);
            simConfigState.setEnabled(false);

            var waterState = new WaterSettingsState();
            stateManager.attach(waterState);
            waterState.setEnabled(false);

            var palette = new CommandPaletteState();
            palette.simManager = standaloneSimManager;
            palette.onNewMap = () -> {
                standaloneSeed = java.util.concurrent.ThreadLocalRandom.current().nextLong();
                restartSim();
            };
            palette.onRerun = this::restartSim;
            palette.onConfigure = () -> simConfigState.setEnabled(true);
            palette.onFlatOcean = () -> {
                standaloneWorld = se.hirt.searobots.engine.GeneratedWorld.deepFlat();
                enqueue(() -> { setWorld(standaloneWorld); return null; });
                if (standaloneMapRenderer != null) standaloneMapRenderer.setWorld(standaloneWorld);
                standaloneSimManager.stop();
                standaloneSimManager.setWorld(standaloneWorld);
                var c = SimConfigState.currentControllers();
                var v = SimConfigState.currentVehicleConfigs();
                if (!c.isEmpty()) {
                    standaloneSimManager.start(standaloneWorld, c, v);
                    standaloneSimManager.play();
                }
            };
            palette.onLIsland = () -> {
                standaloneWorld = se.hirt.searobots.engine.GeneratedWorld.lIslandRecovery();
                enqueue(() -> { setWorld(standaloneWorld); return null; });
                if (standaloneMapRenderer != null) standaloneMapRenderer.setWorld(standaloneWorld);
                standaloneSimManager.stop();
                standaloneSimManager.setWorld(standaloneWorld);
                var c = SimConfigState.currentControllers();
                var v = SimConfigState.currentVehicleConfigs();
                if (!c.isEmpty()) {
                    standaloneSimManager.start(standaloneWorld, c, v);
                    standaloneSimManager.play();
                }
            };
            palette.onSeedInput = () -> {
                String hex = Long.toHexString(standaloneSeed);
                long window = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
                org.lwjgl.glfw.GLFW.glfwSetClipboardString(window, hex);
                System.out.println("Seed copied to clipboard: " + hex);
            };
            stateManager.attach(palette);
            palette.setEnabled(false);

            setupStandaloneInput();
        }
    }

    /** Extra keybindings for standalone mode (sim control, fullscreen). */
    private void setupStandaloneInput() {
        // Ctrl+C: copy seed to clipboard
        inputManager.addMapping("CopySeed", new KeyTrigger(KeyInput.KEY_C));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            long win = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
            if (org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT_CONTROL) != org.lwjgl.glfw.GLFW.GLFW_PRESS
                    && org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT_CONTROL) != org.lwjgl.glfw.GLFW.GLFW_PRESS)
                return;
            String hex = Long.toHexString(standaloneSeed);
            org.lwjgl.glfw.GLFW.glfwSetClipboardString(win, hex);
            System.out.println("Seed copied: " + hex);
        }, "CopySeed");

        // Space: new random map (or skip to next match during competition)
        inputManager.addMapping("NewMap", new KeyTrigger(KeyInput.KEY_SPACE));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || dialogOpen) return;
            if (activeCompetition != null && activeCompetition.isRunning()) {
                activeCompetition.skipToNext();
            } else {
                standaloneSeed = java.util.concurrent.ThreadLocalRandom.current().nextLong();
                restartSim();
            }
        }, "NewMap");

        // P: pause/unpause
        inputManager.addMapping("Pause", new KeyTrigger(KeyInput.KEY_P));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || dialogOpen) return;
            var sim = getActiveSim();
            if (sim != null) sim.setPaused(!sim.isPaused());
        }, "Pause");

        // N: step once
        inputManager.addMapping("StepOnce", new KeyTrigger(KeyInput.KEY_N));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || dialogOpen) return;
            var sim = getActiveSim();
            if (sim != null) sim.stepOnce();
        }, "StepOnce");

        // D: pause on death, F: pause on firing solution, L: pause on torpedo launch
        inputManager.addMapping("TogglePauseOnDeath", new KeyTrigger(KeyInput.KEY_D));
        inputManager.addMapping("TogglePauseOnSolution", new KeyTrigger(KeyInput.KEY_F));
        inputManager.addMapping("TogglePauseOnLaunch", new KeyTrigger(KeyInput.KEY_L));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || dialogOpen) return;
            switch (name) {
                case "TogglePauseOnDeath" -> {
                    standaloneSimManager.pauseOnDeath = !standaloneSimManager.pauseOnDeath;
                    System.out.println("Pause on death: " + (standaloneSimManager.pauseOnDeath ? "ON" : "OFF"));
                }
                case "TogglePauseOnSolution" -> {
                    standaloneSimManager.pauseOnTorpedoSolution = !standaloneSimManager.pauseOnTorpedoSolution;
                    System.out.println("Pause on torpedo solution: " + (standaloneSimManager.pauseOnTorpedoSolution ? "ON" : "OFF"));
                }
                case "TogglePauseOnLaunch" -> {
                    standaloneSimManager.pauseOnTorpedoLaunch = !standaloneSimManager.pauseOnTorpedoLaunch;
                    System.out.println("Pause on torpedo launch: " + (standaloneSimManager.pauseOnTorpedoLaunch ? "ON" : "OFF"));
                }
            }
        }, "TogglePauseOnDeath", "TogglePauseOnSolution", "TogglePauseOnLaunch");

        // 1-6, 0: speed multipliers
        inputManager.addMapping("Speed1", new KeyTrigger(KeyInput.KEY_1));
        inputManager.addMapping("Speed2", new KeyTrigger(KeyInput.KEY_2));
        inputManager.addMapping("Speed4", new KeyTrigger(KeyInput.KEY_3));
        inputManager.addMapping("Speed8", new KeyTrigger(KeyInput.KEY_4));
        inputManager.addMapping("Speed16", new KeyTrigger(KeyInput.KEY_5));
        inputManager.addMapping("Speed24", new KeyTrigger(KeyInput.KEY_6));
        inputManager.addMapping("SpeedMax", new KeyTrigger(KeyInput.KEY_0));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || dialogOpen) return;
            // Use competition sim if running, otherwise standalone sim
            var sim = (activeCompetition != null && activeCompetition.isRunning())
                    ? activeCompetition.currentSim()
                    : standaloneSimManager.currentLoop();
            if (sim == null) return;
            int mult = switch (name) {
                case "Speed1" -> 1;
                case "Speed2" -> 2;
                case "Speed4" -> 4;
                case "Speed8" -> 8;
                case "Speed16" -> 16;
                case "Speed24" -> 24;
                case "SpeedMax" -> 1_000_000;
                default -> -1;
            };
            if (mult > 0) {
                sim.setSpeedMultiplier(mult);
                // Persist speed across competition phases
                if (activeCompetition != null) activeCompetition.setSpeed(mult);
            }
        }, "Speed1", "Speed2", "Speed4", "Speed8", "Speed16", "Speed24", "SpeedMax");

        // F11: toggle fullscreen
        inputManager.addMapping("Fullscreen", new KeyTrigger(KeyInput.KEY_F11));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            enqueue(() -> {
                var s = getContext().getSettings();
                boolean goFull = !s.isFullscreen();
                s.setFullscreen(goFull);
                if (goFull) {
                    // Use GLFW to get monitor resolution (works on Mac, no AWT)
                    long monitor = org.lwjgl.glfw.GLFW.glfwGetPrimaryMonitor();
                    var vidMode = org.lwjgl.glfw.GLFW.glfwGetVideoMode(monitor);
                    if (vidMode != null) {
                        s.setWidth(vidMode.width());
                        s.setHeight(vidMode.height());
                    }
                } else {
                    s.setWidth(1920);
                    s.setHeight(1080);
                }
                getContext().restart();
                return null;
            });
        }, "Fullscreen");

        // M: toggle 2D map view with cross-fade (registered here alongside other overlay keys)

        // +/- or =/- : zoom map (when map is visible)
        inputManager.addMapping("MapZoomIn", new KeyTrigger(KeyInput.KEY_EQUALS));
        inputManager.addMapping("MapZoomOut", new KeyTrigger(KeyInput.KEY_MINUS));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            var mapState = stateManager.getState(NativeMapState.class);
            if (mapState != null && mapState.isMapVisible()) {
                var c = inputManager.getCursorPosition();
                mapState.zoomAt(name.equals("MapZoomIn") ? 1.3 : 1 / 1.3, c.x, c.y);
            }
        }, "MapZoomIn", "MapZoomOut");

        // Escape: toggle command palette (remove JME's default exit-on-escape first)
        inputManager.deleteMapping("SIMPLEAPP_Exit");
        inputManager.addMapping("CommandPalette", new KeyTrigger(KeyInput.KEY_ESCAPE));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            var palette = stateManager.getState(CommandPaletteState.class);
            if (palette != null) palette.setEnabled(!palette.isEnabled());
        }, "CommandPalette");

        // F2: open sim config
        inputManager.addMapping("SimConfig", new KeyTrigger(KeyInput.KEY_F2));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            var cfg = stateManager.getState(SimConfigState.class);
            if (cfg != null) cfg.setEnabled(!cfg.isEnabled());
        }, "SimConfig");

        // F3: toggle water settings
        inputManager.addMapping("WaterSettings", new KeyTrigger(KeyInput.KEY_F3));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed) return;
            var ws = stateManager.getState(WaterSettingsState.class);
            if (ws != null) {
                if (!ws.isEnabled()) {
                    ws.setWaterFilter(waterFilter);
                    ws.setScene(SubmarineScene3D.this);
                }
                ws.setEnabled(!ws.isEnabled());
            }
        }, "WaterSettings");
    }

    private static void loadIcons(AppSettings settings) {
        // macOS: GLFW doesn't support window icons (Cocoa windows use the app bundle icon),
        // and loading via AWT ImageIO deadlocks with GLFW's main-thread requirement.
        if (System.getProperty("os.name", "").toLowerCase().contains("mac")) return;

        String[] sizes = {"16", "24", "32", "48", "64", "128", "256", "512", "1024"};
        var icons = new java.util.ArrayList<java.awt.image.BufferedImage>();
        for (String size : sizes) {
            var url = SubmarineScene3D.class.getResource("/icons/searobots-" + size + ".png");
            if (url != null) {
                try {
                    icons.add(javax.imageio.ImageIO.read(url));
                } catch (java.io.IOException e) {
                    // skip missing sizes
                }
            }
        }
        if (!icons.isEmpty()) {
            settings.setIcons(icons.toArray(new java.awt.image.BufferedImage[0]));
        }
    }

    private void restartSim() {
        // Stop any active competition
        if (activeCompetition != null) {
            activeCompetition.stopAll();
            activeCompetition = null;
        }

        if (SimConfigState.isCompetitionMode()) {
            runCompetition();
        } else {
            runFreePatrol();
        }
    }

    private void runFreePatrol() {
        var world = standaloneGenerator.generate(
                se.hirt.searobots.api.MatchConfig.withDefaults(standaloneSeed));
        standaloneWorld = world;
        enqueue(() -> {
            setWorld(world);
            competitionScoreText.setText("");
            competitionPhaseText.setText("");
            competitionDetailText.setText("");
            // Update title without restarting the OpenGL context
            try {
                long win = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
                org.lwjgl.glfw.GLFW.glfwSetWindowTitle(win,
                        "SeaRobots [seed: " + Long.toHexString(standaloneSeed) + "]");
            } catch (Exception e) { /* ignore */ }
            return null;
        });
        if (standaloneMapRenderer != null) standaloneMapRenderer.setWorld(world);
        standaloneSimManager.stop();
        standaloneSimManager.setWorld(world);
        var controllers = SimConfigState.currentControllers();
        var vehicleConfigs = SimConfigState.currentVehicleConfigs();
        if (!controllers.isEmpty()) {
            standaloneSimManager.start(world, controllers, vehicleConfigs);
            standaloneSimManager.play();
        }
    }

    private void runCompetition() {
        var names = SimConfigState.currentNames();
        var factories = SimConfigState.currentFactories();
        if (factories.size() < 2) {
            System.out.println("Need 2 ships for competition.");
            return;
        }

        standaloneSimManager.stop();

        var competitors = new java.util.ArrayList<se.hirt.searobots.engine.SubmarineCompetition.Competitor>();
        for (int i = 0; i < factories.size(); i++) {
            competitors.add(new se.hirt.searobots.engine.SubmarineCompetition.Competitor(
                    names.get(i), factories.get(i)));
        }

        var callbacks = new CompetitionRunner.ViewerCallbacks() {
            @Override public void setTitle(String title) {
                enqueue(() -> {
                    try {
                        long win = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
                        org.lwjgl.glfw.GLFW.glfwSetWindowTitle(win, title);
                    } catch (Exception e) { /* ignore */ }
                    return null;
                });
            }
            @Override public void setCompetitionScore(String score) {
                enqueue(() -> {
                    competitionScoreText.setText(score);
                    float sw = competitionScoreText.getLineWidth();
                    competitionScoreText.setLocalTranslation(
                            (cam.getWidth() - sw) / 2f,
                            cam.getHeight() - 35, 0);
                    return null;
                });
            }
            @Override public void setCompetitionPhase(String phase) {
                if (standaloneMapRenderer != null) standaloneMapRenderer.setCompetitionPhase(phase);
                enqueue(() -> {
                    competitionPhaseText.setText(phase);
                    float pw = competitionPhaseText.getLineWidth();
                    competitionPhaseText.setLocalTranslation(
                            (cam.getWidth() - pw) / 2f,
                            cam.getHeight() - 58, 0);
                    return null;
                });
            }
            @Override public void addDetailLine(String line) {
                competitionDetailLines.add(line);
                if (standaloneMapRenderer != null) standaloneMapRenderer.addCompetitionResult(line);
                updateDetailHud();
            }
            @Override public void clearCompetition() {
                competitionDetailLines.clear();
                if (standaloneMapRenderer != null) standaloneMapRenderer.clearCompetitionResults();
                enqueue(() -> {
                    competitionScoreText.setText("");
                    competitionPhaseText.setText("");
                    competitionDetailText.setText("");
                    return null;
                });
            }
            @Override public void setObjectives(java.util.List<se.hirt.searobots.api.StrategicWaypoint> objectives) {
                if (standaloneMapRenderer != null) standaloneMapRenderer.setCompetitionObjectives(objectives);
            }
            @Override public void showResultsDialog(String text) {
                System.out.println(text);
            }
            @Override public void scheduleDelayed(long delayMs, Runnable action) {
                // Use a daemon thread with sleep for scheduling
                Thread.ofPlatform().daemon().name("comp-delay").start(() -> {
                    try { Thread.sleep(delayMs); } catch (InterruptedException ignored) {}
                    action.run();
                });
            }
        };

        activeCompetition = new CompetitionRunner(callbacks, standaloneSimManager);
        // Use the current seed as the competition master seed
        var format = se.hirt.searobots.engine.SubmarineCompetition.CompetitionFormat.standard(standaloneSeed);
        System.out.printf("Competition master seed: %s%n", Long.toHexString(standaloneSeed));
        activeCompetition.start(competitors, format);
    }

    /** Returns the active sim loop (competition or free patrol). */
    private se.hirt.searobots.engine.SimulationLoop getActiveSim() {
        if (activeCompetition != null && activeCompetition.isRunning()) {
            return activeCompetition.currentSim();
        }
        return standaloneSimManager.currentLoop();
    }

    private void updateDetailHud() {
        if (!showCompetitionDetails) return;
        enqueue(() -> {
            var sb = new StringBuilder();
            for (var line : competitionDetailLines) {
                sb.append(line).append("\n");
            }
            competitionDetailText.setText(sb.toString());
            competitionDetailText.setLocalTranslation(10, cam.getHeight() * 0.85f, 0);
            return null;
        });
    }

    private long frameCount = 0;

    @Override
    public void handleError(String errMsg, Throwable t) {
        System.err.println("JME ERROR: " + errMsg);
        if (t != null) t.printStackTrace(System.err);
    }

    @Override
    public void simpleUpdate(float tpf) {
        frameCount++;
        if (frameCount <= 3 || frameCount % 500 == 0) {
            System.out.println("simpleUpdate frame #" + frameCount);
        }
        try {
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
            // Loading indicator
            if (loadingText != null) {
                var simSt = simStateSupplier.get();
                boolean showLoading = simSt == se.hirt.searobots.engine.SimulationLoop.State.INITIALIZING || simSt == se.hirt.searobots.engine.SimulationLoop.State.CREATED;
                loadingText.setCullHint(showLoading ? Spatial.CullHint.Never : Spatial.CullHint.Always);
                if (showLoading) {
                    float cx = settings.getWidth() / 2f - loadingText.getLineWidth() / 2f;
                    float cy = settings.getHeight() / 2f;
                    loadingText.setLocalTranslation(cx, cy, 0);
                }
            }
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

            // Update toggle status indicators (all active toggles shown in green)
            {
                var sb = new StringBuilder();
                if (overlayConfig.trails) sb.append("[T] Trails  ");
                if (overlayConfig.route) sb.append("[R] Route  ");
                if (overlayConfig.contactEstimates) sb.append("[E] Contacts  ");
                if (overlayConfig.waypoints) sb.append("[W] Waypoints  ");
                if (overlayConfig.strategicWaypoints) sb.append("[G] Strategic  ");
                if (showCollisionEllipsoids) sb.append("[B] Collision  ");
                if (showCompetitionDetails) sb.append("[I] Details  ");
                if (standaloneSimManager != null && standaloneSimManager.pauseOnDeath) sb.append("[D] Death  ");
                if (standaloneSimManager != null && standaloneSimManager.pauseOnTorpedoSolution) sb.append("[F] Solution  ");
                if (standaloneSimManager != null && standaloneSimManager.pauseOnTorpedoLaunch) sb.append("[L] Launch  ");
                toggleStatusText.setText(sb.toString());
                toggleStatusText.setColor(new ColorRGBA(0.3f, 1f, 0.4f, 0.9f));
                toggleStatusText.setLocalTranslation(
                        cam.getWidth() - toggleStatusText.getLineWidth() - 10,
                        keysText.getHeight() + keysText.getLineHeight() + 14, 0);
            }
            // Toggle competition detail visibility
            competitionDetailText.setCullHint(
                    showCompetitionDetails ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            sunHour = (startTime.toSecondOfDay() / 3600f + latestTick / 50f / 3600f) % 24f;
            updateAtmosphere();
        } catch (Throwable t) {
            System.err.println("EXCEPTION in simpleUpdate (frame " + frameCount + "):");
            t.printStackTrace(System.err);
        }
    }

    /**
     * Set the world to display terrain for. Safe to call from any thread.
     * The terrain mesh will be built and attached on the next jME update tick.
     */
    public void setWorld(GeneratedWorld world) {
        System.out.println("SubmarineScene3D.setWorld() called, world=" + (world != null));
        pendingWorld = world;
        // Reset the cinematic director so it replays the opening flyaround
        cinematicDirector = null;
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

    public void setDebugMode(boolean enabled) {
        debugMode = enabled;
        if (enabled) {
            atmosphereEnabled = false;
        }
    }

    public WaterFilter getWaterFilter() {
        return waterFilter;
    }

    public boolean isAtmosphereEnabled() { return atmosphereEnabled; }

    public boolean isDebugMode() { return debugMode; }

    public boolean isGodRaysEnabled() { return godRaysFilter != null && godRaysFilter.isEnabled(); }

    public void setGodRaysEnabled(boolean enabled) {
        if (godRaysFilter != null) godRaysFilter.setEnabled(enabled);
    }

    public void setSimStateSupplier(java.util.function.Supplier<se.hirt.searobots.engine.SimulationLoop.State> supplier) {
        this.simStateSupplier = supplier;
    }

    @Override
    public void onTick(long tick, List<SubmarineSnapshot> submarines, List<se.hirt.searobots.engine.TorpedoSnapshot> torpedoes) {
        updateSubmarines(tick, submarines);
        latestTorpedoSnapshots = torpedoes != null ? torpedoes : List.of();
    }

    @Override
    public void onMatchEnd() {
        // Nothing to do; the scene keeps displaying the last state
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
        for (Node n : strategicNodes.values()) n.removeFromParent();
        strategicNodes.clear();
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

        if (debugMode) {
            // Debug mode: no fog, no water effect, full bright flat lighting
            sun.setColor(ColorRGBA.White.mult(1.5f));
            fill.setColor(new ColorRGBA(0.7f, 0.7f, 0.7f, 1f));
            ambient.setColor(new ColorRGBA(0.6f, 0.6f, 0.6f, 1f));
            fogFilter.setEnabled(false);
            waterFilter.setEnabled(false);
            viewPort.setBackgroundColor(new ColorRGBA(0.15f, 0.15f, 0.25f, 1f));
            return;
        }
        // Director mode can fade water/fog for overhead shots.
        // Only touch transparency when actively fading; preserve user's F3 setting otherwise.
        // Director mode: simply enable/disable water filter for overhead shots.
        // No transparency manipulation (it causes visual glitches).
        boolean directorHidesWater = cameraMode == CameraMode.DIRECTOR
                && cinematicDirector != null && cinematicDirector.waterOpacity() < 0.5f;
        waterFilter.setEnabled(!directorHidesWater);

        if (!atmosphereEnabled) {
            // Full flat lighting, no fog, but still track sun direction
            sun.setColor(sunUp ? ColorRGBA.White.mult(1.2f) : ColorRGBA.Black);
            fill.setColor(new ColorRGBA(0.5f, 0.6f, 0.7f, 1f));
            ambient.setColor(new ColorRGBA(0.45f, 0.5f, 0.55f, 1f));
            fogFilter.setEnabled(false);
            viewPort.setBackgroundColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));
            return;
        }

        fogFilter.setEnabled(!directorHidesWater);
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
        var torpSnapshots = latestTorpedoSnapshots;

        // Check if selected entity is a torpedo
        var torpSnap = torpSnapshots.stream()
                .filter(t -> t.id() == selectedSubId && t.alive()).findFirst().orElse(null);
        if (torpSnap != null) {
            updateTorpedoHud(torpSnap);
            return;
        }

        var snap = snapshots.stream()
                .filter(s -> s.id() == selectedSubId).findFirst().orElse(null);
        if (snap == null) {
            hudText.setText("");
            tacticalText.setText("");
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
                "%s  |  Speed: %.1f kn  Depth: %.0f m  Throttle: %.0f%%  HP: %d  Torps: %d\n" +
                "Heading: %03.0f\u00b0  Pitch: %+.1f\u00b0  Roll: %+.1f\u00b0  Rudder: %+.0f%%  Planes: %+.0f%%\n" +
                "Tick: %d  Elapsed: %02d:%02d:%02d  ToD: %s  Cam: %s",
                snap.name(), snap.speed(), -pos.z(), snap.throttle() * 100, snap.hp(),
                snap.torpedoesRemaining(),
                hdgDeg, pitchDeg, rollDeg,
                snap.rudder() * 100, snap.sternPlanes() * 100,
                tick, elapsed.toHoursPart(), elapsed.toMinutesPart(), elapsed.toSecondsPart(),
                tod.toString(), camLabel()));

        // Speed/pause indicator (separate text, top center, colored)
        {
            var sim = getActiveSim();
            if (sim != null && sim.isPaused()) {
                speedText.setText("PAUSED");
                speedText.setColor(new ColorRGBA(1f, 0.3f, 0.3f, 0.95f));
            } else if (sim != null && sim.getSpeedMultiplier() > 1) {
                int mult = (int) sim.getSpeedMultiplier();
                speedText.setText(mult >= 1_000_000 ? "MAX SPEED" : mult + "x");
                speedText.setColor(new ColorRGBA(1f, 0.9f, 0.3f, 0.9f));
            } else {
                speedText.setText("");
            }
            // Position top center
            float sw = speedText.getLineWidth();
            speedText.setLocalTranslation(
                    (cam.getWidth() - sw) / 2f,
                    cam.getHeight() - 10, 0);
        }

        // Tactical info panel (top right)
        updateTacticalHud(snap, pos);
    }

    private void updateTorpedoHud(TorpedoSnapshot ts) {
        var pos = ts.pose().position();
        double hdgDeg = Math.toDegrees(ts.pose().heading());
        if (hdgDeg < 0) hdgDeg += 360;
        double pitchDeg = Math.toDegrees(ts.pose().pitch());
        long tick = latestTick;
        var elapsed = java.time.Duration.ofMillis((long) (tick * 1000.0 / 50));

        // Find owner name
        String ownerName = "?";
        for (var s : latestSnapshots) {
            if (s.id() == ts.ownerId()) { ownerName = s.name(); break; }
        }

        // Target/intercept info line
        String targetLine = "";
        double tx = ts.targetX(), ty = ts.targetY(), tz = ts.targetZ();
        if (!Double.isNaN(tx) && !Double.isNaN(ty)) {
            double dx = tx - pos.x();
            double dy = ty - pos.y();
            double dist = Math.sqrt(dx * dx + dy * dy);
            double bearing = Math.toDegrees(Math.atan2(dx, dy));
            if (bearing < 0) bearing += 360;
            double tgtDepth = Double.isNaN(tz) ? 0 : -tz;
            targetLine = String.format(
                    "Target: %03.0f\u00b0  %.0fm  Depth: %.0fm",
                    bearing, dist, tgtDepth);
        }

        hudText.setText(String.format(
                "TORPEDO #%d (from %s)  |  Speed: %.1f m/s  Depth: %.0f m\n" +
                "Heading: %03.0f\u00b0  Pitch: %+.1f\u00b0  %s\n" +
                "Fuel: %.0fs  Noise: %.0f dB  Tick: %d  Elapsed: %02d:%02d:%02d  Cam: %s",
                ts.id(), ownerName, ts.speed(), -pos.z(),
                hdgDeg, pitchDeg, targetLine,
                ts.fuelRemaining(), ts.sourceLevelDb(),
                tick, elapsed.toHoursPart(), elapsed.toMinutesPart(), elapsed.toSecondsPart(),
                camLabel()));
        tacticalText.setText("");
    }

    private void updateTacticalHud(SubmarineSnapshot snap, Vec3 pos) {
        var sb = new StringBuilder();

        // Status
        String status = snap.status() != null ? snap.status() : "---";
        sb.append(String.format("STATUS: %s%n", status));
        sb.append(String.format("Noise: %.0f dB%n", 80 + 20 * Math.log10(Math.max(0.01, snap.noiseLevel()))));

        // Active nav waypoint
        var navWps = snap.waypoints();
        if (navWps != null && !navWps.isEmpty()) {
            var active = navWps.stream().filter(Waypoint::active).findFirst().orElse(null);
            if (active != null) {
                double dx = active.x() - pos.x();
                double dy = active.y() - pos.y();
                double dist = Math.sqrt(dx * dx + dy * dy);
                double bearing = Math.toDegrees(Math.atan2(dx, dy));
                if (bearing < 0) bearing += 360;
                String type = active.reverse() ? "REVERSE" : "NAV";
                sb.append(String.format("%n[%s] WP  %03.0f\u00b0  %.0fm  (%.0fm depth)%n",
                        type, bearing, dist, -active.z()));
            }
        }

        // Strategic waypoints
        var strats = snap.strategicWaypoints();
        if (strats != null && !strats.isEmpty()) {
            sb.append(String.format("%nTACTICAL PLAN:%n"));
            for (int i = 0; i < strats.size(); i++) {
                var sw = strats.get(i);
                var wp = sw.waypoint();
                double dx = wp.x() - pos.x();
                double dy = wp.y() - pos.y();
                double dist = Math.sqrt(dx * dx + dy * dy);
                double bearing = Math.toDegrees(Math.atan2(dx, dy));
                if (bearing < 0) bearing += 360;
                String marker = wp.active() ? "> " : "  ";
                sb.append(String.format("%s%d. %-10s %03.0f\u00b0 %5.0fm%n",
                        marker, i + 1, sw.purpose().name(), bearing, dist));
            }
        }

        // Contact count
        var contacts = snap.contactEstimates();
        if (contacts != null && !contacts.isEmpty()) {
            sb.append(String.format("%nCONTACTS: %d%n", contacts.size()));
            for (var c : contacts) {
                double dx = c.x() - pos.x();
                double dy = c.y() - pos.y();
                double dist = Math.sqrt(dx * dx + dy * dy);
                double bearing = Math.toDegrees(Math.atan2(dx, dy));
                if (bearing < 0) bearing += 360;
                sb.append(String.format("  %03.0f\u00b0  %.0fm%n", bearing, dist));
            }
        }

        tacticalText.setText(sb.toString());
        // Position relative to top-right corner
        tacticalText.setLocalTranslation(
                cam.getWidth() - 420,
                cam.getHeight() - 10, 0);
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
            if (tick > lastTrailTick
                    && (lastTrailTick < 0 || tick / TRAIL_SAMPLE_INTERVAL > lastTrailTick / TRAIL_SAMPLE_INTERVAL)) {
                trail.addLast(jmePos.clone());
                if (trail.size() > MAX_TRAIL_POINTS) trail.removeFirst();
            }

            Node trailNode = trailNodes.get(id);
            if (trailNode == null) {
                trailNode = new Node("trail-" + id);
                trailNodes.put(id, trailNode);
                rootNode.attachChild(trailNode);
            }
            trailNode.setCullHint(overlayConfig.trails ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            if (overlayConfig.trails && trail.size() >= 2 && tick % 10 == 0) {
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
                mat.getAdditionalRenderState().setDepthTest(false);
                mat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
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
            wpNode.setCullHint(overlayConfig.waypoints ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            if (overlayConfig.waypoints) {
                wpNode.detachAllChildren();
                var waypoints = snap.waypoints();
                if (!waypoints.isEmpty()) {
                    // Solid ribbon connecting waypoints (triangle strip tube)
                    if (waypoints.size() >= 2) {
                        float hw = 2f; // ribbon half-width
                        // Build a flat ribbon strip: for each waypoint, emit two
                        // vertices offset perpendicular to the segment direction
                        float[] spos = new float[waypoints.size() * 2 * 3];
                        for (int j = 0; j < waypoints.size(); j++) {
                            var wp = waypoints.get(j);
                            float cx = (float) wp.x();
                            float cy = (float) wp.z();
                            float cz = (float) -wp.y();
                            // Perpendicular in XZ plane (horizontal ribbon)
                            float dx = 0, dz = 0;
                            if (j < waypoints.size() - 1) {
                                var next = waypoints.get(j + 1);
                                dx = (float) next.x() - cx;
                                dz = (float) -next.y() - cz;
                            } else {
                                var prev = waypoints.get(j - 1);
                                dx = cx - (float) prev.x();
                                dz = cz - (float) (-prev.y());
                            }
                            float len = (float) Math.sqrt(dx * dx + dz * dz);
                            if (len > 0.001f) { dx /= len; dz /= len; }
                            // Perpendicular: rotate 90 degrees in XZ
                            float px = -dz * hw, pz = dx * hw;
                            spos[j * 6]     = cx + px;
                            spos[j * 6 + 1] = cy;
                            spos[j * 6 + 2] = cz + pz;
                            spos[j * 6 + 3] = cx - px;
                            spos[j * 6 + 4] = cy;
                            spos[j * 6 + 5] = cz - pz;
                        }
                        Mesh splineMesh = new Mesh();
                        splineMesh.setMode(Mesh.Mode.TriangleStrip);
                        splineMesh.setBuffer(VertexBuffer.Type.Position, 3, spos);
                        splineMesh.updateBound();
                        Geometry splineGeom = new Geometry("wpSpline", splineMesh);
                        Material spMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                        spMat.setColor("Color", new ColorRGBA(color.r * 0.5f, color.g * 0.5f, color.b * 0.5f, 1f));
                        spMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
                        splineGeom.setMaterial(spMat);
                        splineGeom.setCullHint(Spatial.CullHint.Never);
                        wpNode.attachChild(splineGeom);
                    }

                    // Waypoint markers: solid circle rings, active one larger and white
                    int circSegs = 24;
                    for (var wp : waypoints) {
                        float wpX = (float) wp.x();
                        float wpY = (float) wp.z();
                        float wpZ = (float) -wp.y();
                        boolean active = wp.active();
                        float r = active ? 18f : 10f;
                        float w = active ? 2.5f : 1.5f;

                        // Solid circle ring (triangle strip)
                        float[] cpos = new float[circSegs * 2 * 3];
                        for (int ci = 0; ci < circSegs; ci++) {
                            float a = (float) (2 * Math.PI * ci / (circSegs - 1));
                            float cos = FastMath.cos(a), sin = FastMath.sin(a);
                            cpos[ci * 6]     = (r - w) * cos;
                            cpos[ci * 6 + 1] = (r - w) * sin;
                            cpos[ci * 6 + 2] = 0;
                            cpos[ci * 6 + 3] = (r + w) * cos;
                            cpos[ci * 6 + 4] = (r + w) * sin;
                            cpos[ci * 6 + 5] = 0;
                        }
                        Mesh cm = new Mesh();
                        cm.setMode(Mesh.Mode.TriangleStrip);
                        cm.setBuffer(VertexBuffer.Type.Position, 3, cpos);
                        cm.updateBound();
                        Geometry wpGeom = new Geometry("wp", cm);
                        Material wpMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                        wpMat.setColor("Color", active ? ColorRGBA.White : color);
                        wpMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
                        wpGeom.setMaterial(wpMat);
                        wpGeom.setLocalTranslation(wpX, wpY, wpZ);
                        wpGeom.addControl(new BillboardControl());
                        wpGeom.setCullHint(Spatial.CullHint.Never);
                        wpNode.attachChild(wpGeom);
                    }
                }
            }

            // --- Strategic Waypoints ---
            Node stNode = strategicNodes.get(id);
            if (stNode == null) {
                stNode = new Node("strategic-" + id);
                strategicNodes.put(id, stNode);
                rootNode.attachChild(stNode);
            }
            stNode.setCullHint(overlayConfig.strategicWaypoints ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            if (overlayConfig.strategicWaypoints) {
                stNode.detachAllChildren();
                var stratWps = snap.strategicWaypoints();
                if (stratWps != null && !stratWps.isEmpty()) {
                    int circSegs = 24;
                    for (var sw : stratWps) {
                        var wp = sw.waypoint();
                        float wpX = (float) wp.x();
                        float wpY = (float) wp.z();
                        float wpZ = (float) -wp.y();
                        boolean active = wp.active();
                        float r = active ? 25f : 16f;
                        float w = active ? 3.5f : 2.5f;

                        // Color-code by purpose
                        ColorRGBA purposeColor = switch (sw.purpose()) {
                            case PATROL -> new ColorRGBA(0.2f, 0.8f, 0.2f, 1f);
                            case INVESTIGATE -> new ColorRGBA(1f, 1f, 0.2f, 1f);
                            case PING_POSITION -> new ColorRGBA(1f, 0.5f, 0f, 1f);
                            case STEALTH_TRANSIT -> new ColorRGBA(0.3f, 0.3f, 0.8f, 1f);
                            case INTERCEPT -> new ColorRGBA(1f, 0.2f, 0.2f, 1f);
                            case EVADE -> new ColorRGBA(0.8f, 0.2f, 0.8f, 1f);
                            case RALLY -> ColorRGBA.Cyan;
                        };

                        float[] cpos = new float[circSegs * 2 * 3];
                        for (int ci = 0; ci < circSegs; ci++) {
                            float a = (float) (2 * Math.PI * ci / (circSegs - 1));
                            float cos = FastMath.cos(a), sin = FastMath.sin(a);
                            cpos[ci * 6]     = (r - w) * cos;
                            cpos[ci * 6 + 1] = (r - w) * sin;
                            cpos[ci * 6 + 2] = 0;
                            cpos[ci * 6 + 3] = (r + w) * cos;
                            cpos[ci * 6 + 4] = (r + w) * sin;
                            cpos[ci * 6 + 5] = 0;
                        }
                        Mesh cm = new Mesh();
                        cm.setMode(Mesh.Mode.TriangleStrip);
                        cm.setBuffer(VertexBuffer.Type.Position, 3, cpos);
                        cm.updateBound();
                        Geometry swGeom = new Geometry("strategic-wp", cm);
                        Material swMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                        swMat.setColor("Color", active ? ColorRGBA.White : purposeColor);
                        swMat.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
                        swGeom.setMaterial(swMat);
                        swGeom.setLocalTranslation(wpX, wpY, wpZ);
                        swGeom.addControl(new BillboardControl());
                        swGeom.setCullHint(Spatial.CullHint.Never);
                        stNode.attachChild(swGeom);
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
            rtNode.setCullHint(overlayConfig.route ? Spatial.CullHint.Never : Spatial.CullHint.Always);

            // Sample route history (less frequent than trails, full match history)
            // Use interval-crossing check so high speed multipliers don't skip samples
            var route = routeBuffers.computeIfAbsent(id, k -> new java.util.ArrayList<>());
            if (tick > lastTrailTick
                    && (lastTrailTick < 0 || tick / ROUTE_SAMPLE_INTERVAL > lastTrailTick / ROUTE_SAMPLE_INTERVAL)) {
                route.add(jmePos.clone());
            }

            if (overlayConfig.route && route.size() >= 2 && tick % 10 == 0) {
                rtNode.detachAllChildren();
                int n = route.size();
                float w = 2f; // half-width of the route duct

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

            // Spin propeller: based on throttle when alive, free-spinning when dead
            Spatial prop = findChild(subNode, "Propeller");
            if (prop != null) {
                if (snap.hp() > 0) {
                    float spinRate = (float) snap.throttle() * tpf * 8f;
                    prop.rotate(0, spinRate, 0);
                } else {
                    // Dead: prop freewheels, decaying from water drag
                    float spin = deathPropSpin.getOrDefault(snap.id(), 3f);
                    spin *= (1f - tpf * 0.3f); // exponential decay, ~3 second half-life
                    if (spin < 0.01f) spin = 0;
                    deathPropSpin.put(snap.id(), spin);
                    prop.rotate(0, tpf * spin, 0);
                }
            }

            // Death bubbles: stream of bubbles from a sinking sub
            if (snap.hp() <= 0) {
                ParticleEmitter db = deathBubbles.get(snap.id());
                if (db == null) {
                    db = new ParticleEmitter("deathBubbles-" + snap.id(),
                            ParticleMesh.Type.Triangle, 100);
                    Material dbMat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
                    dbMat.setTexture("Texture",
                            assetManager.loadTexture("Effects/Explosion/smoketrail.png"));
                    dbMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
                    db.setMaterial(dbMat);
                    db.setImagesX(1); db.setImagesY(3);
                    db.setStartColor(new ColorRGBA(0.8f, 0.9f, 1f, 0.6f));
                    db.setEndColor(new ColorRGBA(0.9f, 0.95f, 1f, 0f));
                    db.setStartSize(0.5f);
                    db.setEndSize(3f);
                    db.setGravity(0, 5f, 0); // bubbles float up (JME Y = up)
                    db.setLowLife(2f);
                    db.setHighLife(5f);
                    db.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 8f, 0));
                    db.getParticleInfluencer().setVelocityVariation(0.5f);
                    db.setParticlesPerSec(15);
                    rootNode.attachChild(db);
                    deathBubbles.put(snap.id(), db);
                }
                db.setLocalTranslation(targetPos);
                // Reduce bubbles as the sub settles (less motion = fewer air leaks)
                float sinkSpeed = Math.abs((float) snap.speed()) + 0.5f;
                var simLp = getActiveSim();
                db.setParticlesPerSec(simLp != null && simLp.isPaused() ? 0 : Math.max(2, (int)(sinkSpeed * 3)));
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

            // Bubble emitter: cavitation noise visualization
            ParticleEmitter bubbles = bubbleEmitters.get(snap.id());
            if (bubbles == null) {
                bubbles = new ParticleEmitter("bubbles-" + snap.id(), ParticleMesh.Type.Triangle, 200);
                Material bubbleMat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
                bubbleMat.setTexture("Texture",
                        assetManager.loadTexture("Effects/Explosion/smoketrail.png"));
                bubbleMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
                bubbleMat.getAdditionalRenderState().setFaceCullMode(
                        com.jme3.material.RenderState.FaceCullMode.Off);
                bubbles.setMaterial(bubbleMat);
                bubbles.setImagesX(1);
                bubbles.setImagesY(3);
                bubbles.setStartColor(new ColorRGBA(1f, 1f, 1f, 0.6f));
                bubbles.setEndColor(new ColorRGBA(0.8f, 0.9f, 1f, 0f));
                bubbles.setStartSize(0.15f);
                bubbles.setEndSize(0.5f);
                bubbles.setGravity(0, -5f, 0); // bubbles rise (negative = upward in local space)
                bubbles.setLowLife(0.5f);
                bubbles.setHighLife(2f);
                bubbles.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 2f, 0));
                bubbles.getParticleInfluencer().setVelocityVariation(0.5f);
                bubbles.setLocalTranslation(0, 37f, 0); // stern: model Y=37, try jME Y
                subNode.attachChild(bubbles);
                bubbleEmitters.put(snap.id(), bubbles);
            }

            // Scale emission rate by noise level (noiseLevel is linear, ~1.0 = quiet, ~10+ = loud)
            float noise = (float) snap.noiseLevel();
            float emitRate = Math.max(0, (noise - 1.5f) * 30f); // no bubbles below noise 1.5
            var simL = getActiveSim();
            bubbles.setParticlesPerSec(simL != null && simL.isPaused() ? 0 : emitRate);

            // Debug collision ellipsoid
            Geometry ellGeom = ellipsoidGeoms.get(snap.id());
            if (ellGeom == null) {
                var sphere = new com.jme3.scene.shape.Sphere(24, 24, 1f);
                ellGeom = new Geometry("ellipsoid-" + snap.id(), sphere);
                Material ellMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                var ec = snap.color();
                ellMat.setColor("Color", new ColorRGBA(ec.getRed() / 255f, ec.getGreen() / 255f, ec.getBlue() / 255f, 0.15f));
                ellMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
                ellMat.getAdditionalRenderState().setWireframe(true);
                ellGeom.setMaterial(ellMat);
                ellGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
                rootNode.attachChild(ellGeom);
                ellipsoidGeoms.put(snap.id(), ellGeom);
            }
            ellGeom.setCullHint(showCollisionEllipsoids ? Spatial.CullHint.Never : Spatial.CullHint.Always);
            if (showCollisionEllipsoids) {
                // Use model's interpolated position so ellipsoid tracks visual model
                // (not raw snapshot position, which leads at high sim speeds)
                Vector3f subModelPos = subNode.getLocalTranslation();
                Quaternion subModelRot = subNode.getLocalRotation();

                // Ellipsoid for hull body (excluding tower):
                // tighter vertical, offset aft since bow is longer than stern
                float semiLength = 38f;   // covers bow to stern body
                float semiBeam = 5.5f;    // slightly tighter than hullHalfBeam
                float semiHeight = 4.5f;  // hull body only, tower excluded
                float aftOffset = 2f;     // shift center slightly aft (sub local Y)

                ellGeom.setLocalRotation(subModelRot);
                // Offset in sub's local frame then transform to world
                Vector3f offset = subModelRot.mult(new Vector3f(0, aftOffset, 0));
                ellGeom.setLocalTranslation(subModelPos.add(offset));
                // In sub's local frame: X = beam, Y = forward (length), Z = up (height)
                ellGeom.setLocalScale(semiBeam, semiLength, semiHeight);
            }

            // Terrain collision points: 7 purple spheres
            // center, bow, stern, port, starboard, tower top, keel
            Geometry[] tpGeoms = terrainPointGeoms.get(snap.id());
            if (tpGeoms == null) {
                tpGeoms = new Geometry[7];
                var tpSphere = new com.jme3.scene.shape.Sphere(8, 8, 0.75f);
                Material tpMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                tpMat.setColor("Color", new ColorRGBA(0.7f, 0.2f, 0.9f, 0.8f));
                tpMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
                for (int tp = 0; tp < 7; tp++) {
                    tpGeoms[tp] = new Geometry("tp-" + snap.id() + "-" + tp, tpSphere);
                    tpGeoms[tp].setMaterial(tpMat);
                    tpGeoms[tp].setQueueBucket(RenderQueue.Bucket.Transparent);
                    rootNode.attachChild(tpGeoms[tp]);
                }
                terrainPointGeoms.put(snap.id(), tpGeoms);
            }
            for (var tpg : tpGeoms) {
                tpg.setCullHint(showCollisionEllipsoids ? Spatial.CullHint.Never : Spatial.CullHint.Always);
            }
            if (showCollisionEllipsoids) {
                double simX = snap.pose().position().x();
                double simY = snap.pose().position().y();
                double simZ = snap.pose().position().z();
                double hdg = snap.pose().heading();
                double pit = snap.pose().pitch();
                double cosH = Math.cos(hdg), sinH = Math.sin(hdg);
                double cosP = Math.cos(pit), sinP = Math.sin(pit);

                // Local frame vectors (sim coords: X=east, Y=north, Z=up)
                // Forward: heading=0 -> +Y (north), so fwd = (sinH, cosH, 0) * cosP + (0,0,sinP)
                double fwdX = sinH * cosP, fwdY = cosH * cosP, fwdZ = sinP;
                // Right: perpendicular in horizontal plane
                double rightX = cosH, rightY = -sinH, rightZ = 0;
                // Up: cross(fwd, right)
                double upX = -sinH * sinP, upY = -cosH * sinP, upZ = cosP;

                double bowDist = 33.5;
                double sternDist = 40.0;
                double beamDist = 6.0;
                double towerHeight = 6.5;
                double keelDepth = 5.0;

                // Offsets in local frame -> world coords
                double[][] pts = {
                    {simX, simY, simZ},                                                                     // center
                    {simX + fwdX * bowDist, simY + fwdY * bowDist, simZ + fwdZ * bowDist},                  // bow
                    {simX - fwdX * sternDist, simY - fwdY * sternDist, simZ - fwdZ * sternDist},            // stern
                    {simX + rightX * beamDist, simY + rightY * beamDist, simZ + rightZ * beamDist},          // port (actually starboard, sign doesn't matter for collision)
                    {simX - rightX * beamDist, simY - rightY * beamDist, simZ - rightZ * beamDist},          // starboard
                    {simX + upX * towerHeight, simY + upY * towerHeight, simZ + upZ * towerHeight},          // tower top
                    {simX - upX * keelDepth, simY - upY * keelDepth, simZ - upZ * keelDepth},                // keel
                };
                for (int tp = 0; tp < 7; tp++) {
                    // sim (X,Y,Z) -> JME (X, Z, -Y)
                    tpGeoms[tp].setLocalTranslation(
                            (float) pts[tp][0], (float) pts[tp][2], (float) -pts[tp][1]);
                }
            }
        }

        // ── Torpedo 3D rendering ──
        var torpSnapshots = latestTorpedoSnapshots;
        // Remove nodes for torpedoes that no longer exist
        var activeTorpIds = torpSnapshots.stream().mapToInt(TorpedoSnapshot::id)
                .boxed().collect(java.util.stream.Collectors.toSet());
        torpedoNodes.entrySet().removeIf(entry -> {
            if (!activeTorpIds.contains(entry.getKey())) {
                entry.getValue().removeFromParent();
                return true;
            }
            return false;
        });

        for (var ts : torpSnapshots) {
            if (!ts.alive()) continue;
            var tPos = ts.pose().position();
            float tx = (float) tPos.x();
            float ty = (float) tPos.z(); // sim Z -> JME Y
            float tz = (float) -tPos.y(); // sim Y -> JME -Z

            var targetPos = new Vector3f(tx, ty, tz);
            float tHeading = (float) -ts.pose().heading() + FastMath.PI;
            float tPitch = (float) ts.pose().pitch();
            var targetRot = new Quaternion().fromAngles(
                    -FastMath.HALF_PI - tPitch, tHeading, 0);

            Node torpNode = torpedoNodes.get(ts.id());
            if (torpNode == null) {
                torpNode = (Node) torpedoModelNode.deepClone();
                torpNode.setName("torpedo-" + ts.id());
                torpedoNodes.put(ts.id(), torpNode);
                rootNode.attachChild(torpNode);
                torpNode.setLocalTranslation(targetPos);
                torpNode.setLocalRotation(targetRot);
            } else {
                // Smooth interpolation (same lerpFactor as subs)
                Vector3f currentPos = torpNode.getLocalTranslation();
                torpNode.setLocalTranslation(currentPos.interpolateLocal(targetPos, lerpFactor));
                Quaternion currentRot = torpNode.getLocalRotation();
                currentRot.slerp(targetRot, lerpFactor);
                torpNode.setLocalRotation(currentRot);
            }
            // Spin torpedo propeller proportional to speed
            Spatial torpProp = findChild(torpNode, "g_Propeller_Plane");
            if (torpProp != null && ts.speed() > 1) {
                torpProp.rotate(0, tpf * (float) ts.speed() * 0.4f, 0);
            }

            // Torpedo wake bubbles (much more than sub: torpedo is loud and fast)
            ParticleEmitter torpBub = torpedoBubbles.get(ts.id());
            if (torpBub == null) {
                torpBub = new ParticleEmitter("torpWake-" + ts.id(),
                        ParticleMesh.Type.Triangle, 150);
                Material bMat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
                bMat.setTexture("Texture",
                        assetManager.loadTexture("Effects/Explosion/smoketrail.png"));
                bMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
                bMat.getAdditionalRenderState().setFaceCullMode(
                        com.jme3.material.RenderState.FaceCullMode.Off);
                torpBub.setMaterial(bMat);
                torpBub.setImagesX(1); torpBub.setImagesY(3);
                torpBub.setStartColor(new ColorRGBA(1f, 1f, 1f, 0.5f));
                torpBub.setEndColor(new ColorRGBA(0.7f, 0.85f, 1f, 0f));
                torpBub.setStartSize(0.3f);
                torpBub.setEndSize(2f);
                torpBub.setGravity(0, 3f, 0); // bubbles float up
                torpBub.setLowLife(0.3f);
                torpBub.setHighLife(1.5f);
                torpBub.getParticleInfluencer().setInitialVelocity(
                        new Vector3f(0, 2f, 0));
                torpBub.getParticleInfluencer().setVelocityVariation(0.6f);
                rootNode.attachChild(torpBub);
                torpedoBubbles.put(ts.id(), torpBub);
            }
            // Use the model's interpolated position (not raw targetPos) so that
            // bubbles and collision cylinder stay aligned with the visual model,
            // especially at high sim speeds where lerp causes lag.
            Vector3f modelPos = torpNode.getLocalTranslation();
            Quaternion modelRot = torpNode.getLocalRotation();

            // Position bubbles at the visual stern, accounting for pitch.
            // Stern = -forward direction in sim coords
            float simH = (float) ts.pose().heading();
            float simP = (float) ts.pose().pitch();
            float halfLen = TORP_HALF_LENGTH;
            float cosP = (float) Math.cos(simP);
            float sinP = (float) Math.sin(simP);
            float sternSimX = -(float) Math.sin(simH) * cosP * halfLen;
            float sternSimY = -(float) Math.cos(simH) * cosP * halfLen;
            float sternSimZ = -sinP * halfLen;
            torpBub.setLocalTranslation(
                    modelPos.x + sternSimX,
                    modelPos.y + sternSimZ,  // JME Y = sim Z
                    modelPos.z - sternSimY); // JME Z = -sim Y
            // Stop emission when paused (particles freeze in place)
            var simLoop = getActiveSim();
            boolean paused = simLoop != null && simLoop.isPaused();
            torpBub.setParticlesPerSec(paused ? 0 : Math.max(0, (int)(ts.speed() * 4)));
            // Torpedo collision cylinder (B key)
            Geometry torpCollGeom = torpedoCollisionGeoms.get(ts.id());
            if (torpCollGeom == null) {
                var cyl = new com.jme3.scene.shape.Cylinder(8, 12, TORP_RADIUS, TORP_HALF_LENGTH * 2, true);
                torpCollGeom = new Geometry("torpColl-" + ts.id(), cyl);
                Material collMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                collMat.setColor("Color", new ColorRGBA(1f, 1f, 0f, 0.3f));
                collMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
                collMat.getAdditionalRenderState().setWireframe(true);
                torpCollGeom.setMaterial(collMat);
                torpCollGeom.setQueueBucket(RenderQueue.Bucket.Transparent);
                rootNode.attachChild(torpCollGeom);
                torpedoCollisionGeoms.put(ts.id(), torpCollGeom);
            }
            torpCollGeom.setCullHint(showCollisionEllipsoids ? Spatial.CullHint.Never : Spatial.CullHint.Always);
            if (showCollisionEllipsoids) {
                torpCollGeom.setLocalTranslation(modelPos);
                var cylRot = modelRot.mult(new Quaternion().fromAngles(FastMath.HALF_PI, 0, 0));
                torpCollGeom.setLocalRotation(cylRot);
            }
        }

        // Clean up collision geoms and wake bubbles for removed torpedoes
        torpedoCollisionGeoms.entrySet().removeIf(entry -> {
            if (!activeTorpIds.contains(entry.getKey())) {
                if (entry.getValue() != null) entry.getValue().removeFromParent();
                return true;
            }
            return false;
        });
        torpedoBubbles.entrySet().removeIf(entry -> {
            if (!activeTorpIds.contains(entry.getKey())) {
                entry.getValue().removeFromParent();
                return true;
            }
            return false;
        });

        // Update intercept marker for selected torpedo
        {
            TorpedoSnapshot selTorp = null;
            for (var ts : torpSnapshots) {
                if (ts.id() == selectedSubId && ts.alive()) { selTorp = ts; break; }
            }
            if (selTorp != null && !Double.isNaN(selTorp.targetX()) && !Double.isNaN(selTorp.targetY())) {
                float ix = (float) selTorp.targetX();
                float iy = (float) (Double.isNaN(selTorp.targetZ()) ? selTorp.pose().position().z() : selTorp.targetZ());
                float iz = (float) -selTorp.targetY();
                interceptMarker.setLocalTranslation(ix, iy, iz);
                interceptMarker.setCullHint(Spatial.CullHint.Never);
                // Color matches torpedo owner
                var tc = selTorp.color();
                interceptMarker.getMaterial().setColor("Color",
                        new ColorRGBA(tc.getRed()/255f, tc.getGreen()/255f, tc.getBlue()/255f, 0.8f));
            } else {
                interceptMarker.setCullHint(Spatial.CullHint.Always);
            }
        }

        // Detect detonations: torpedoes that disappeared and were detonated
        for (var ts : torpSnapshots) {
            if (ts.detonated() && !knownTorpedoIds3D.contains(-ts.id())) { // use negative to mark seen
                knownTorpedoIds3D.add(-ts.id());
                var tPos = ts.pose().position();
                float jx = (float) tPos.x();
                float jy = (float) tPos.z();
                float jz = (float) -tPos.y();
                var ec = ts.color();
                activeExplosions.add(new Explosion3D(ts.id(),
                        new Vector3f(jx, jy, jz), appTime,
                        new ColorRGBA(ec.getRed()/255f, ec.getGreen()/255f, ec.getBlue()/255f, 1f)));
            }
        }
        knownTorpedoIds3D.retainAll(activeTorpIds.stream().map(i -> -i).collect(java.util.stream.Collectors.toSet()));

        // Render 3D explosions: fireball + shockwave ring + debris particles
        appTime += tpf;
        var explosionIter = activeExplosions.iterator();
        while (explosionIter.hasNext()) {
            var exp = explosionIter.next();
            float elapsed = appTime - exp.startTime();
            float t = elapsed / EXPLOSION_DURATION;

            if (t > 1.0f) {
                // Expired: clean up all effect geometries
                var geom = explosionGeoms.remove(exp.id());
                if (geom != null) geom.removeFromParent();
                var debris = debrisEmitters.remove(exp.id());
                if (debris != null) debris.removeFromParent();
                explosionIter.remove();
                continue;
            }

            // === Fireball (expanding hot sphere) ===
            Geometry geom = explosionGeoms.get(exp.id());
            if (geom == null) {
                var sphere = new com.jme3.scene.shape.Sphere(16, 16, 1f);
                geom = new Geometry("explosion-" + exp.id(), sphere);
                Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
                mat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
                mat.getAdditionalRenderState().setDepthWrite(false);
                geom.setMaterial(mat);
                geom.setQueueBucket(RenderQueue.Bucket.Transparent);
                rootNode.attachChild(geom);
                explosionGeoms.put(exp.id(), geom);
            }

            float fireRadius;
            float fade = (1.0f - t) * (1.0f - t);
            float r, g, b, a;
            if (elapsed < 0.15f) {
                // Intense white flash (very bright, visible through water)
                float flashT = elapsed / 0.15f;
                r = 2f; g = 2f; b = 1.5f; // >1 for extra glow with additive blending
                a = (1 - flashT * 0.3f);
                fireRadius = EXPLOSION_MAX_RADIUS * 0.5f * (0.5f + flashT);
            } else if (elapsed < 0.8f) {
                // Hot orange-yellow fireball expanding
                float phase = (elapsed - 0.15f) / 0.65f;
                r = 2f - phase * 0.5f; g = 1.2f - phase * 0.6f; b = 0.4f - phase * 0.3f;
                a = (1 - phase * 0.4f) * 0.9f;
                fireRadius = EXPLOSION_MAX_RADIUS * (0.5f + phase * 0.3f);
            } else {
                // Cooling dark cloud, expanding, fading
                r = 1.0f * fade + exp.color().r * (1 - fade);
                g = 0.4f * fade + exp.color().g * (1 - fade);
                b = 0.1f * fade;
                a = fade * 0.7f;
                fireRadius = EXPLOSION_MAX_RADIUS * (float) Math.sqrt(t);
            }

            geom.getMaterial().setColor("Color", new ColorRGBA(r, g, b, a));
            geom.setLocalTranslation(exp.position());
            geom.setLocalScale(fireRadius);

            // === Fire/smoke particles (flame texture, animated) ===
            ParticleEmitter fireEmitter = debrisEmitters.get(exp.id());
            if (fireEmitter == null) {
                fireEmitter = new ParticleEmitter("fire-" + exp.id(), ParticleMesh.Type.Triangle, 50);
                Material fireMat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
                fireMat.setTexture("Texture",
                        assetManager.loadTexture("Effects/Explosion/flame.png"));
                fireMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
                fireEmitter.setMaterial(fireMat);
                fireEmitter.setImagesX(2); fireEmitter.setImagesY(2);
                fireEmitter.setStartColor(new ColorRGBA(1f, 0.9f, 0.3f, 1f));
                fireEmitter.setEndColor(new ColorRGBA(1f, 0.2f, 0f, 0f));
                fireEmitter.setStartSize(5f);
                fireEmitter.setEndSize(20f);
                fireEmitter.setGravity(0, 2f, 0); // slight upward drift (hot gas rises)
                fireEmitter.setLowLife(0.5f);
                fireEmitter.setHighLife(1.5f);
                fireEmitter.getParticleInfluencer().setInitialVelocity(
                        new Vector3f(0, 12f, 0));
                fireEmitter.getParticleInfluencer().setVelocityVariation(1f);
                fireEmitter.setLocalTranslation(exp.position());
                fireEmitter.setParticlesPerSec(0); // burst mode
                fireEmitter.emitAllParticles();
                rootNode.attachChild(fireEmitter);
                debrisEmitters.put(exp.id(), fireEmitter);

                // Also spawn debris chunks
                var debrisEmitter = new ParticleEmitter("debris-" + exp.id(),
                        ParticleMesh.Type.Triangle, 15);
                Material debrisMat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
                debrisMat.setTexture("Texture",
                        assetManager.loadTexture("Effects/Explosion/Debris.png"));
                debrisEmitter.setMaterial(debrisMat);
                debrisEmitter.setImagesX(3); debrisEmitter.setImagesY(3);
                debrisEmitter.setSelectRandomImage(true);
                debrisEmitter.setRotateSpeed(4);
                debrisEmitter.setStartColor(ColorRGBA.White);
                debrisEmitter.setEndColor(new ColorRGBA(0.5f, 0.5f, 0.5f, 0f));
                debrisEmitter.setStartSize(2f);
                debrisEmitter.setEndSize(4f);
                debrisEmitter.setGravity(0, -3f, 0); // debris sinks in water
                debrisEmitter.setLowLife(1f);
                debrisEmitter.setHighLife(3f);
                debrisEmitter.getParticleInfluencer().setInitialVelocity(
                        new Vector3f(0, 15f, 0));
                debrisEmitter.getParticleInfluencer().setVelocityVariation(0.8f);
                debrisEmitter.setLocalTranslation(exp.position());
                debrisEmitter.setParticlesPerSec(0);
                debrisEmitter.emitAllParticles();
                rootNode.attachChild(debrisEmitter);
                // Note: debris emitter is not tracked separately; it self-cleans when particles die
            }
        }

        // Update orbit center tracking (used by Orbit mode and as fallback)
        // Check both sub nodes and torpedo nodes
        Node selected = subNodes.get(selectedSubId);
        if (selected == null) selected = torpedoNodes.get(selectedSubId);
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
                boolean isTorpedo = torpedoNodes.containsKey(selectedSubId);
                if (isTorpedo) {
                    // Torpedoes are small and fast: snap orbit center directly
                    orbitCenter.set(targetPos);
                } else {
                    float dist = orbitCenter.distance(targetPos);
                    float speed = dist > 100f ? 3f : 5f;
                    float factor = Math.min(1f, tpf * speed);
                    if (dist > 10f) factor = Math.max(factor, 0.05f);
                    orbitCenter.interpolateLocal(targetPos, factor);
                }
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
            if (isPressed && !dialogOpen && !latestSnapshots.isEmpty()) {
                // In Director mode, Tab exits to Orbit on the current entity
                if (cameraMode == CameraMode.DIRECTOR) {
                    cameraMode = CameraMode.ORBIT;
                    if (waterFilter != null) waterFilter.setEnabled(true);
                    if (fogFilter != null) fogFilter.setEnabled(true);
                    System.out.println("Camera: Orbit (exited Director)");
                    return;
                }
                // Merge sub + torpedo IDs for cycling
                var subIds = latestSnapshots.stream().mapToInt(SubmarineSnapshot::id).boxed().toList();
                var torpIds = latestTorpedoSnapshots.stream()
                        .filter(TorpedoSnapshot::alive).mapToInt(TorpedoSnapshot::id).boxed().toList();
                var allIds = new java.util.ArrayList<>(subIds);
                allIds.addAll(torpIds);
                var ids = allIds.stream().mapToInt(Integer::intValue).toArray();
                int currentIdx = 0;
                for (int i = 0; i < ids.length; i++) {
                    if (ids[i] == selectedSubId) { currentIdx = i; break; }
                }
                selectedSubId = ids[(currentIdx + 1) % ids.length];
                chaseInitialized = false;
                // Adjust orbit distance: closer for torpedoes (they're tiny)
                if (selectedSubId >= 1000) {
                    orbitDistance = Math.min(orbitDistance, 60f);
                }
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
            if (isPressed && !dialogOpen) {
                // Ignore V when Ctrl is held (Ctrl+V = paste in dialogs)
                long win = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
                if (org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT_CONTROL) == org.lwjgl.glfw.GLFW.GLFW_PRESS
                        || org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT_CONTROL) == org.lwjgl.glfw.GLFW.GLFW_PRESS)
                    return;
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
                // Restore water/fog when leaving director mode
                if (cameraMode != CameraMode.DIRECTOR) {
                    if (waterFilter != null) waterFilter.setEnabled(true);
                    if (fogFilter != null) fogFilter.setEnabled(true);
                }
                System.out.println("Camera: " + cameraMode.label());
            }
        }, "CycleCamera");

        // T/R/E/W/G/B to toggle overlays (unified across 2D and 3D)
        inputManager.addMapping("ToggleTrails", new KeyTrigger(KeyInput.KEY_T));
        inputManager.addMapping("ToggleRoute", new KeyTrigger(KeyInput.KEY_R));
        inputManager.addMapping("ToggleContacts", new KeyTrigger(KeyInput.KEY_E));
        inputManager.addMapping("ToggleWaypoints", new KeyTrigger(KeyInput.KEY_W));
        inputManager.addMapping("ToggleStrategic", new KeyTrigger(KeyInput.KEY_G));
        inputManager.addMapping("ToggleEllipsoids", new KeyTrigger(KeyInput.KEY_B));
        inputManager.addMapping("ToggleDetails", new KeyTrigger(KeyInput.KEY_I));
        inputManager.addMapping("ToggleMap", new KeyTrigger(KeyInput.KEY_M));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || dialogOpen) return;
            switch (name) {
                case "ToggleTrails" -> overlayConfig.trails = !overlayConfig.trails;
                case "ToggleRoute" -> overlayConfig.route = !overlayConfig.route;
                case "ToggleContacts" -> overlayConfig.contactEstimates = !overlayConfig.contactEstimates;
                case "ToggleWaypoints" -> overlayConfig.waypoints = !overlayConfig.waypoints;
                case "ToggleStrategic" -> overlayConfig.strategicWaypoints = !overlayConfig.strategicWaypoints;
                case "ToggleEllipsoids" -> showCollisionEllipsoids = !showCollisionEllipsoids;
                case "ToggleDetails" -> { showCompetitionDetails = !showCompetitionDetails; updateDetailHud(); }
                case "ToggleMap" -> {
                    var mapState = standalone ? stateManager.getState(NativeMapState.class) : null;
                    if (mapState != null) mapState.toggle();
                }
            }
        }, "ToggleTrails", "ToggleRoute", "ToggleContacts", "ToggleWaypoints", "ToggleStrategic", "ToggleEllipsoids", "ToggleDetails", "ToggleMap");

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
            // When 2D map is visible, redirect mouse events to map
            var mapState = standalone ? stateManager.getState(NativeMapState.class) : null;
            var cursor = inputManager.getCursorPosition();
            if (mapState != null && mapState.wantsMouseInput(cursor.x, cursor.y)) {
                switch (name) {
                    case "ZoomIn" -> mapState.zoomAt(1.20, cursor.x, cursor.y);
                    case "ZoomOut" -> mapState.zoomAt(1.0 / 1.20, cursor.x, cursor.y);
                    case "OrbitLeft" -> { if (dragging) mapState.pan(value * cam.getWidth(), 0); }
                    case "OrbitRight" -> { if (dragging) mapState.pan(-value * cam.getWidth(), 0); }
                    case "OrbitUp" -> { if (dragging) mapState.pan(0, value * cam.getHeight()); }
                    case "OrbitDown" -> { if (dragging) mapState.pan(0, -value * cam.getHeight()); }
                }
                return;
            }

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
                case "ZoomIn" -> { if (fl) freeLookDistance = Math.max(5f, freeLookDistance * 0.9f); else orbitDistance = Math.max(5f, orbitDistance * 0.9f); }
                case "ZoomOut" -> { if (fl) freeLookDistance *= 1.1f; else orbitDistance *= 1.1f; }
            }
        }, "OrbitLeft", "OrbitRight", "OrbitUp", "OrbitDown", "ZoomIn", "ZoomOut");
    }

    private void updateCamera(float tpf) {
        Vector3f desiredPos = new Vector3f();
        Vector3f desiredLookAt = new Vector3f();
        computeCameraForMode(desiredPos, desiredLookAt, tpf);

        // Choose the up vector based on camera orientation. When looking nearly
        // straight down (overhead shots), UNIT_Y is degenerate and causes the camera
        // to flicker. Use -Z (north in JME) as up instead, which gives stable north-up framing.
        Vector3f camPos, lookAt;
        if (modeTransTimer > 0) {
            modeTransTimer -= tpf;
            float t = 1f - Math.max(0f, modeTransTimer / MODE_TRANS_DURATION);
            t = t * t * (3f - 2f * t); // smoothstep
            camPos = new Vector3f(modeTransFromPos).interpolateLocal(desiredPos, t);
            lookAt = new Vector3f(modeTransFromLookAt).interpolateLocal(desiredLookAt, t);
        } else {
            camPos = desiredPos;
            lookAt = desiredLookAt;
        }
        cam.setLocation(camPos);
        // If camera is mostly above the lookAt point, use north as up to avoid gimbal flicker
        Vector3f toLook = lookAt.subtract(camPos);
        float downness = Math.abs(toLook.normalize().dot(Vector3f.UNIT_Y));
        Vector3f up = downness > 0.7f ? new Vector3f(0, 0, -1) : Vector3f.UNIT_Y;
        cam.lookAt(lookAt, up);
    }

    private void computeCameraForMode(Vector3f outPos, Vector3f outLookAt, float tpf) {
        switch (cameraMode) {
            case ORBIT     -> computeOrbitCamera(outPos, outLookAt);
            case CHASE     -> computeChaseCamera(outPos, outLookAt, tpf);
            case TARGET    -> computeTargetCamera(outPos, outLookAt, tpf);
            case PERISCOPE -> computePeriscopeCamera(outPos, outLookAt);
            case FREE_LOOK -> computeFreeLookCamera(outPos, outLookAt, tpf);
            case FLY_BY    -> computeFlyByCamera(outPos, outLookAt, tpf);
            case DIRECTOR  -> {
                if (cinematicDirector == null) {
                    cinematicDirector = new CinematicDirector(
                            () -> latestSnapshots,
                            () -> latestTorpedoSnapshots,
                            subNodes, torpedoNodes,
                            standaloneWorld != null ? standaloneWorld.terrain() : null);
                }
                int newId = cinematicDirector.update(tpf, outPos, outLookAt);
                if (newId != selectedSubId) {
                    selectedSubId = newId;
                    chaseInitialized = false;
                }
                // Water/fog opacity is handled by the atmosphere update method
                // via cinematicDirector.waterOpacity()
            }
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

    private void dumpSpatial(Spatial s, String indent) {
        System.out.println(indent + s.getName() + " (" + s.getClass().getSimpleName() + ")");
        if (s instanceof Node n) {
            for (var c : n.getChildren()) dumpSpatial(c, indent + "  ");
        }
    }

    private String camLabel() {
        if (cameraMode == CameraMode.DIRECTOR && cinematicDirector != null) {
            return "Director: " + cinematicDirector.currentShotLabel();
        }
        return cameraMode.label();
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

}
