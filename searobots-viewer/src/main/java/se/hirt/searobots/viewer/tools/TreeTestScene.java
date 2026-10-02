/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.viewer.tools;

import com.jme3.app.SimpleApplication;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.*;
import com.jme3.post.FilterPostProcessor;
import com.jme3.scene.Geometry;
import com.jme3.system.AppSettings;
import se.hirt.searobots.viewer.TreeScatter;
import com.jme3.terrain.geomipmap.TerrainQuad;
import com.jme3.texture.Texture;
import com.jme3.water.WaterFilter;

/**
 * Tiny test island for evaluating tree billboard appearance and placement. Uses the same materials
 * and lighting as the full viewer. A small dome-shaped island surrounded by water, with clusters of
 * each tree type.
 * <p>
 * Launch: {@code java -cp searobots-viewer/target/searobots-viewer-*-SNAPSHOT.jar
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
		if (ssState == null)
			ssState = stateManager.getState(ScreenshotAppState.class);
		if (ssState != null && shotsTaken < 4) {
			if (frameCount == 60 || frameCount == 160 || frameCount == 260 || frameCount == 360) {
				ssState.takeScreenshot();
				shotsTaken++;
				System.out.println("TreeTest screenshot " + shotsTaken + "/4");
			}
		}
		orbitAzimuth += tpf * 0.12f;
		orbitElevation = frameCount > 300 ? 1.15f : 0.35f; // last shot from high above
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
				elev += (float) (Math.sin(c * 0.12 + r * 0.08) * 4 * Math.max(0, 1 - dist));
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
		Material mat = new Material(assetManager, "Common/MatDefs/Terrain/HeightBasedTerrain.j3md");
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

	/**
	 * Place labelled groups of each tree type around the island, built exactly as the viewer builds
	 * them.
	 */
	private void placeTreeGroups(TerrainQuad tq) {
		var rng = new java.util.Random(77);
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			float angle = type.ordinal() * FastMath.HALF_PI + 0.3f;
			float groupR = 60f;
			var trees = new java.util.ArrayList<TreeScatter.TreeInstance>();
			for (int i = 0; i < 15; i++) {
				float wx = (float) (Math.cos(angle) * groupR + (rng.nextFloat() - 0.5) * 35);
				float wz = (float) (Math.sin(angle) * groupR + (rng.nextFloat() - 0.5) * 35);
				// Raycast straight down to find exact terrain surface
				var ray = new com.jme3.math.Ray(new Vector3f(wx, 200, wz), new Vector3f(0, -1, 0));
				var results = new com.jme3.collision.CollisionResults();
				tq.collideWith(ray, results);
				if (results.size() == 0)
					continue;
				Vector3f hit = results.getClosestCollision().getContactPoint();
				if (hit.y < 2)
					continue;
				trees.add(TreeScatter.instance(type, rng, hit.x, hit.y, hit.z, 1f));
			}
			if (trees.isEmpty())
				continue;
			var side = new Geometry("trees_" + type, TreeScatter.buildCrossMesh(trees));
			side.setMaterial(TreeScatter.createTreeMaterial(assetManager, type.sidePath()));
			rootNode.attachChild(side);
			var top = new Geometry("trees_top_" + type, TreeScatter.buildTopMesh(trees, type));
			top.setMaterial(TreeScatter.createTopMaterial(assetManager, type.topPath()));
			rootNode.attachChild(top);
			System.out.printf("  %s: %d trees%n", type, trees.size());
		}
	}

	private Texture loadWrap(String path) {
		Texture t = assetManager.loadTexture(path);
		t.setWrap(Texture.WrapMode.Repeat);
		return t;
	}
}
