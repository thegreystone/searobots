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
    private int selectedSubId = 0;
    private static final float LERP_SPEED = 8f; // higher = snappier tracking
    private static final float SURFACE_SPEED = 2f; // control surface actuator lerp rate
    private BitmapText hudText;
    private BitmapText keysText;

    // Lighting and atmosphere
    private DirectionalLight sun;
    private DirectionalLight fill;
    private AmbientLight ambient;
    private FogFilter fogFilter;
    private WaterFilter waterFilter;
    private LightScatteringFilter godRaysFilter;
    private FilterPostProcessor fpp;
    private volatile boolean atmosphereEnabled = true;

    // Sun simulation: equator, time of day in hours (12.0 = noon)
    // 1 real minute = 1 sim hour, so full day cycle in 24 real minutes
    private float sunHour = 12f;
    private static final float HOURS_PER_SECOND = 1f / 60f;

    // Orbit camera state
    private final Vector3f orbitCenter = new Vector3f();
    private float orbitAzimuth = FastMath.QUARTER_PI;   // radians, 0 = looking along +Z
    private float orbitElevation = 0.4f;                 // radians above horizon
    private float orbitDistance = 200f;
    private boolean dragging;

    // Tab transition: phase 1 = rotate to face target, phase 2 = move to target
    private float transitionTimer = 0f;
    private static final float LOOK_PHASE_DURATION = 0.6f; // seconds to rotate before moving

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
        setupOrbitInput();

        // Lighting
        DirectionalLight dl = new DirectionalLight();
        dl.setDirection(new Vector3f(-1, -1, -1).normalizeLocal());
        dl.setColor(ColorRGBA.White.mult(1.2f));
        rootNode.addLight(dl);

        DirectionalLight backLight = new DirectionalLight();
        backLight.setDirection(new Vector3f(1, 0.5f, 1).normalizeLocal());
        backLight.setColor(new ColorRGBA(0.3f, 0.4f, 0.5f, 1f));
        rootNode.addLight(backLight);

        AmbientLight al = new AmbientLight();
        al.setColor(new ColorRGBA(0.3f, 0.35f, 0.4f, 1f));
        rootNode.addLight(al);

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
                "[P] Pause  [N] Step  [Tab] Cycle sub  [Space] New map");
        float keysWidth = keysText.getLineWidth();
        float keysHeight = keysText.getHeight();
        keysText.setLocalTranslation(settings.getWidth() - keysWidth - 10, keysHeight + 10, 0);
        guiNode.attachChild(keysText);

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
        updateOrbitCamera();
        updateHud();
        // TODO: re-enable incrementally
        // sunHour += tpf * HOURS_PER_SECOND;
        // updateAtmosphere();
    }

    /**
     * Set the world to display terrain for. Safe to call from any thread.
     * The terrain mesh will be built and attached on the next jME update tick.
     */
    public void setWorld(GeneratedWorld world) {
        System.out.println("SubmarineScene3D.setWorld() called, world=" + (world != null));
        pendingWorld = world;
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

        // Clear previous vehicle models
        for (Node n : subNodes.values()) n.removeFromParent();
        subNodes.clear();
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
            godRaysFilter.setLightPosition(sunPos);
            sunBillboard.setLocalTranslation(sunPos.subtract(400, 400, 0));
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
        hudText.setText(String.format(
                "%s  |  Speed: %.1f kn  Depth: %.0f m  Throttle: %.0f%%\n" +
                "Heading: %03.0f\u00b0  Pitch: %+.1f\u00b0  Roll: %+.1f\u00b0  Rudder: %+.0f%%  Planes: %+.0f%%",
                snap.name(), snap.speed(), -pos.z(), snap.throttle() * 100,
                hdgDeg, pitchDeg, rollDeg,
                snap.rudder() * 100, snap.sternPlanes() * 100));
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

        // Smoothly move orbit camera center to follow selected sub
        Node selected = subNodes.get(selectedSubId);
        if (selected != null) {
            Vector3f targetPos = selected.getLocalTranslation();

            if (transitionTimer > 0) {
                // Phase 1: rotate orbit to look toward the new target, don't move yet
                transitionTimer -= tpf;
                Vector3f toTarget = targetPos.subtract(orbitCenter);
                float targetAzimuth = FastMath.atan2(toTarget.x, toTarget.z);
                // Smoothly rotate azimuth toward target direction
                float angleDiff = targetAzimuth - orbitAzimuth;
                // Normalize to -PI..PI
                while (angleDiff > FastMath.PI) angleDiff -= FastMath.TWO_PI;
                while (angleDiff < -FastMath.PI) angleDiff += FastMath.TWO_PI;
                orbitAzimuth += angleDiff * Math.min(1f, tpf * 5f);
            } else {
                // Phase 2 / normal tracking: move orbit center to follow sub
                float dist = orbitCenter.distance(targetPos);
                float speed = dist > 100f ? 3f : 5f;
                float factor = Math.min(1f, tpf * speed);
                if (dist > 10f) factor = Math.max(factor, 0.05f);
                orbitCenter.interpolateLocal(targetPos, factor);
            }
        }
    }

    // ---- orbit camera ----

    private void setupOrbitInput() {
        // Tab to cycle between submarines (with look-then-move transition)
        inputManager.addMapping("CycleSub", new KeyTrigger(KeyInput.KEY_TAB));
        inputManager.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (isPressed && !latestSnapshots.isEmpty()) {
                var ids = latestSnapshots.stream().mapToInt(SubmarineSnapshot::id).toArray();
                int currentIdx = 0;
                for (int i = 0; i < ids.length; i++) {
                    if (ids[i] == selectedSubId) { currentIdx = i; break; }
                }
                selectedSubId = ids[(currentIdx + 1) % ids.length];
                transitionTimer = LOOK_PHASE_DURATION;
                System.out.println("Tracking: " + latestSnapshots.stream()
                        .filter(s -> s.id() == selectedSubId)
                        .map(SubmarineSnapshot::name).findFirst().orElse("?"));
            }
        }, "CycleSub");

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
            if (dragging) {
                float speed = 2.5f;
                switch (name) {
                    case "OrbitLeft" -> orbitAzimuth -= value * speed;
                    case "OrbitRight" -> orbitAzimuth += value * speed;
                    case "OrbitUp" -> orbitElevation = Math.min(FastMath.HALF_PI - 0.01f,
                            orbitElevation + value * speed);
                    case "OrbitDown" -> orbitElevation = Math.max(-0.2f,
                            orbitElevation - value * speed);
                }
            }
            switch (name) {
                case "ZoomIn" -> orbitDistance = Math.max(20f, orbitDistance * 0.9f);
                case "ZoomOut" -> orbitDistance *= 1.1f;
            }
        }, "OrbitLeft", "OrbitRight", "OrbitUp", "OrbitDown", "ZoomIn", "ZoomOut");
    }

    private void updateOrbitCamera() {
        float x = orbitCenter.x + orbitDistance * FastMath.cos(orbitElevation) * FastMath.sin(orbitAzimuth);
        float y = orbitCenter.y + orbitDistance * FastMath.sin(orbitElevation);
        float z = orbitCenter.z + orbitDistance * FastMath.cos(orbitElevation) * FastMath.cos(orbitAzimuth);
        cam.setLocation(new Vector3f(x, y, z));
        cam.lookAt(orbitCenter, Vector3f.UNIT_Y);
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

    private Texture2D createSkyTexture() {
        // Sphere-mapped sky: gradient from deep blue (top) to bright horizon,
        // with a bright sun spot. The sphere map maps the center of the image
        // to "forward", edges to "behind", top to "up", bottom to "down".
        // For a sphere map, Y=0 is up (zenith), Y=size is down (nadir).
        int size = 512;
        ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
        float cx = size / 2f, cy = size / 2f;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float dx = (x - cx) / cx;
                float dy = (y - cy) / cy;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);

                // Elevation: center of sphere map = horizon, top = zenith
                // dy: -1 = top (zenith), +1 = bottom (nadir)
                float elevation = -dy; // +1 at top, -1 at bottom

                int r, g, b;
                if (elevation > 0) {
                    // Sky: gradient from bright horizon to deep blue at zenith
                    float t = elevation;
                    r = (int) (180 - t * 130);  // 180 -> 50
                    g = (int) (210 - t * 100);  // 210 -> 110
                    b = (int) (245 - t * 30);   // 245 -> 215
                } else {
                    // Below horizon: dark (underwater/ground)
                    float t = -elevation;
                    r = (int) (180 - t * 170);
                    g = (int) (210 - t * 200);
                    b = (int) (245 - t * 220);
                }

                // Clamp and outside sphere = black
                if (dist > 1f) { r = 0; g = 0; b = 0; }
                r = Math.max(0, Math.min(255, r));
                g = Math.max(0, Math.min(255, g));
                b = Math.max(0, Math.min(255, b));

                buf.put((byte) r).put((byte) g).put((byte) b).put((byte) 255);
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
