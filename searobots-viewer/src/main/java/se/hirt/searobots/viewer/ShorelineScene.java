/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain this copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce this copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package se.hirt.searobots.viewer;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

import com.jme3.app.SimpleApplication;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.environment.EnvironmentCamera;
import com.jme3.environment.LightProbeFactory;
import com.jme3.environment.generation.JobProgressAdapter;
import com.jme3.environment.util.EnvMapUtils;
import com.jme3.font.BitmapText;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.light.DirectionalLight;
import com.jme3.light.LightProbe;
import com.jme3.material.TechniqueDef;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.FXAAFilter;
import com.jme3.post.filters.FogFilter;
import com.jme3.post.filters.ToneMapFilter;
import com.jme3.post.ssao.SSAOFilter;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Node;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.material.Material;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.shadow.EdgeFilteringMode;
import com.jme3.system.AppSettings;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.TextureCubeMap;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.SkyFactory;
import com.jme3.water.WaterFilter;

import se.hirt.searobots.viewer.shoreline.CoastalVegetation;
import se.hirt.searobots.viewer.shoreline.ShorelineTerrain;

/**
 * Small, independent scenery benchmark: original 3D vegetation and rocks, detailed PBR terrain,
 * environment lighting, sun shadows and shallow coastal water. Pass {@code --capture <directory>}
 * to save the same five views and exit; otherwise explore with WASD and mouse drag.
 */
public final class ShorelineScene extends SimpleApplication {

	private static final String[] VIEW_NAMES = {"coast", "beach", "forest", "headland", "overview"};
	private static final int FRAMES_PER_VIEW = 100;
	private final Path captureDirectory;
	private final long seed;
	private final EnvironmentCamera environmentCamera = new EnvironmentCamera(128, new Vector3f(0, 8, 10));
	private Node environment;
	private LightProbe probe;
	private ScreenshotAppState screenshots;
	private int captureFrame;
	private int selectedView;
	private float sampleSeconds;
	private int sampleFrames;
	private final int[] renderedStatistics = new int[13];
	private boolean probeRequested;
	private boolean lightingReady;
	private BitmapText help;

	private ShorelineScene(Path captureDirectory, long seed) {
		this.captureDirectory = captureDirectory;
		this.seed = seed;
	}

	public static void main(String[] args) throws IOException {
		// PNG encoding needs Java2D, but an AWT Cocoa event loop conflicts with GLFW on macOS.
		System.setProperty("java.awt.headless", "true");
		Path output = null;
		long seed = 0x2c;
		for (int i = 0; i < args.length; i++) {
			switch (args[i]) {
			case "--capture" -> output = Path.of(args[++i]).toAbsolutePath();
			case "--seed" -> seed = Long.parseUnsignedLong(args[++i], 16);
			default -> throw new IllegalArgumentException("Usage: ShorelineScene [--capture directory] [--seed hex]");
			}
		}
		if (output != null)
			Files.createDirectories(output);
		var app = new ShorelineScene(output, seed);
		var settings = new AppSettings(true);
		settings.setTitle("SeaRobots · Shoreline");
		settings.setWidth(1600);
		settings.setHeight(900);
		settings.setSamples(4);
		settings.setGammaCorrection(true);
		settings.setVSync(output == null);
		settings.setFrameRate(output == null ? 60 : -1);
		settings.setResizable(output == null);
		app.setSettings(settings);
		app.setShowSettings(false);
		app.setPauseOnLostFocus(false);
		app.start();
	}

	@Override
	public void simpleInitApp() {
		setDisplayFps(false);
		setDisplayStatView(false);
		flyCam.setMoveSpeed(18f);
		flyCam.setDragToRotate(true);
		flyCam.setEnabled(captureDirectory == null);
		cam.setFrustumPerspective(54f, (float) cam.getWidth() / cam.getHeight(), 0.15f, 700f);
		renderManager.setPreferredLightMode(TechniqueDef.LightMode.SinglePass);
		renderManager.setSinglePassLightBatchSize(1);
		renderer.getStatistics().setEnabled(true);

		Vector3f towardSun = new Vector3f(-0.48f, 0.64f, -0.60f).normalizeLocal();
		var sun = new DirectionalLight();
		sun.setDirection(towardSun.negate());
		sun.setColor(new ColorRGBA(2.2f, 2.15f, 2f, 1f));
		rootNode.addLight(sun);

		environment = new Node("shoreline_sky");
		// SkyFactory's sphere normals face inwards; our HDR map uses outward world directions.
		Spatial sky = SkyFactory.createSky(assetManager, skyMap(towardSun), new Vector3f(-1, -1, -1),
				SkyFactory.EnvMapType.CubeMap);
		sky.setShadowMode(RenderQueue.ShadowMode.Off);
		environment.attachChild(sky);
		rootNode.attachChild(environment);
		// Continue the seafloor beyond the detailed patch so distant water has a consistent depth.
		var seabed = new Geometry("Offshore seabed", new Box(1500f, 1f, 1500f));
		var seabedMaterial = new Material(assetManager, "Common/MatDefs/Light/PBRLighting.j3md");
		seabedMaterial.setColor("BaseColor", new ColorRGBA(0.19f, 0.15f, 0.08f, 1f));
		seabedMaterial.setFloat("Metallic", 0f);
		seabedMaterial.setFloat("Roughness", 1f);
		seabed.setMaterial(seabedMaterial);
		seabed.setLocalTranslation(0f, -65f, 0f);
		seabed.setShadowMode(RenderQueue.ShadowMode.Off);
		rootNode.attachChild(seabed);
		rootNode.attachChild(ShorelineTerrain.create(assetManager, seed));
		rootNode.attachChild(CoastalVegetation.create(assetManager, seed, ShorelineTerrain::heightAt));
		stateManager.attach(environmentCamera);

		// Bind all four sampler slots, including on drivers that validate unused shadow samplers.
		var shadows = new DirectionalLightShadowRenderer(assetManager, 2048, 4);
		shadows.setLight(sun);
		shadows.setLambda(0.75f);
		shadows.setShadowIntensity(0.8f);
		shadows.setShadowZExtend(230f);
		shadows.setShadowZFadeLength(35f);
		shadows.setEdgeFilteringMode(EdgeFilteringMode.PCFPOISSON);
		shadows.setEdgesThickness(2);
		viewPort.addProcessor(shadows);

		FilterPostProcessor processor = SubmarineScene3D.createFilterPostProcessor(assetManager, renderer,
				settings.getSamples());
		var occlusion = new SSAOFilter(2f, 1.4f, 0.25f, 0.12f);
		occlusion.setApproximateNormals(true);
		processor.addFilter(occlusion);
		var water = new WaterFilter(rootNode, towardSun);
		WaterQualitySettings.configure(water);
		water.setWaterHeight(0f);
		water.setLightDirection(sun.getDirection());
		water.setLightColor(new ColorRGBA(1.25f, 1.12f, 0.95f, 1f));
		water.setWaterColor(new ColorRGBA(0.035f, 0.24f, 0.25f, 1f));
		water.setDeepWaterColor(new ColorRGBA(0.008f, 0.07f, 0.11f, 1f));
		water.setWaterTransparency(0.10f);
		water.setColorExtinction(new Vector3f(6f, 18f, 26f));
		water.setWaveScale(0.008f);
		water.setMaxAmplitude(0.25f);
		water.setSpeed(captureDirectory == null ? 0.35f : 0f);
		water.setSunScale(2f);
		water.setFoamIntensity(0.25f);
		water.setFoamExistence(new Vector3f(0.05f, 1.1f, 0.3f));
		water.setFoamHardness(1.2f);
		processor.addFilter(water);
		var fog = new FogFilter(new ColorRGBA(0.44f, 0.61f, 0.76f, 1f), 0.18f, 650f);
		processor.addFilter(fog);
		processor.addFilter(new ToneMapFilter(new Vector3f(4f, 4f, 4f)));
		var antialiasing = new FXAAFilter();
		antialiasing.setSubPixelShift(0f);
		processor.addFilter(antialiasing);
		viewPort.addProcessor(processor);

		String directory = captureDirectory == null ? "" : captureDirectory + "/";
		screenshots = new ScreenshotAppState(directory, "shoreline");
		screenshots.setIsNumbered(false);
		stateManager.attach(screenshots);
		setupControls();
		selectView(0);
		System.out.printf("Shoreline: seed=%x, capture=%s, filter samples=%d%n", seed,
				captureDirectory == null ? "interactive" : captureDirectory, processor.getNumSamples());
	}

	private void setupControls() {
		help = new BitmapText(guiFont);
		help.setText(
				"SHORELINE\n1 Coast  2 Beach  3 Forest  4 Headland  5 Overview\nWASD Move  ·  Q/Z Vertical  ·  Drag mouse Look  ·  F1 Capture  ·  H Hide help");
		help.setSize(16f);
		help.setColor(new ColorRGBA(0.96f, 0.96f, 0.89f, 1f));
		guiNode.attachChild(help);
		positionHelp();
		if (captureDirectory != null)
			guiNode.setCullHint(Spatial.CullHint.Always);
		for (int view = 0; view < VIEW_NAMES.length; view++) {
			final int index = view;
			String mapping = "shoreline_view_" + view;
			inputManager.addMapping(mapping, new KeyTrigger(KeyInput.KEY_1 + view));
			inputManager.addListener((com.jme3.input.controls.ActionListener) (name, pressed, tpf) -> {
				if (pressed)
					selectView(index);
			}, mapping);
		}
		inputManager.addMapping("shoreline_capture", new KeyTrigger(KeyInput.KEY_F1));
		inputManager.addMapping("shoreline_help", new KeyTrigger(KeyInput.KEY_H));
		inputManager.addListener((com.jme3.input.controls.ActionListener) (name, pressed, tpf) -> {
			if (!pressed)
				return;
			if (name.equals("shoreline_capture")) {
				screenshots.setFileName("shoreline-" + VIEW_NAMES[selectedView]);
				screenshots.takeScreenshot();
			} else {
				help.setCullHint(help.getCullHint() == Spatial.CullHint.Always ? Spatial.CullHint.Inherit
						: Spatial.CullHint.Always);
			}
		}, "shoreline_capture", "shoreline_help");
	}

	private void positionHelp() {
		help.setLocalTranslation(24f, cam.getHeight() - 24f, 0f);
	}

	private void selectView(int view) {
		selectedView = view;
		Vector3f[] positions = {new Vector3f(38, 7, -58), new Vector3f(-34, 3, -14),
				new Vector3f(-14, ShorelineTerrain.heightAt(-14, 45) + 2.2f, 45), new Vector3f(83, 8, -38),
				new Vector3f(61, 38, -79)};
		Vector3f[] targets = {new Vector3f(-6, 6, 22), new Vector3f(-4, 5, 24),
				new Vector3f(18, ShorelineTerrain.heightAt(18, 82) + 3f, 82), new Vector3f(52, 5, 8),
				new Vector3f(-10, 5, -5)};
		cam.setLocation(positions[view]);
		cam.lookAt(targets[view], Vector3f.UNIT_Y);
	}

	@Override
	public void simpleUpdate(float tpf) {
		positionHelp();
		if (!probeRequested && environmentCamera.isInitialized()) {
			// StatsAppState initializes after simpleInitApp and disables counters with its hidden HUD.
			renderer.getStatistics().setEnabled(true);
			probeRequested = true;
			probe = LightProbeFactory.makeProbe(environmentCamera, environment, new JobProgressAdapter<LightProbe>() {
				@Override
				public void done(LightProbe result) {
					lightingReady = true;
					System.out.println("Shoreline: environment lighting ready");
				}
			});
			probe.getArea().setRadius(450f);
			rootNode.addLight(probe);
		}
		if (!lightingReady || captureDirectory == null)
			return;
		int view = captureFrame / FRAMES_PER_VIEW;
		if (view >= VIEW_NAMES.length) {
			stop();
			return;
		}
		int frame = captureFrame % FRAMES_PER_VIEW;
		if (frame == 0) {
			selectView(view);
			sampleSeconds = 0;
			sampleFrames = 0;
		}
		if (frame >= 25 && frame < 70) {
			sampleSeconds += tpf;
			sampleFrames++;
		}
		if (frame == 75) {
			screenshots.setFileName(String.format("shoreline-%02d-%s", view + 1, VIEW_NAMES[view]));
			screenshots.takeScreenshot();
			System.out.printf("Shoreline capture: %s, %.1f FPS, %d triangles, %d draw objects%n", VIEW_NAMES[view],
					sampleFrames / sampleSeconds, renderedStatistics[1], renderedStatistics[3]);
		}
		captureFrame++;
	}

	@Override
	public void simpleRender(RenderManager manager) {
		// Sample the completed render before ResetStatsState clears the frame's counters.
		renderer.getStatistics().getData(renderedStatistics);
	}

	/** Original HDR sky: directional gradient, soft cloud bands and a warm afternoon sun. */
	private static TextureCubeMap skyMap(Vector3f towardSun) {
		int size = 256;
		ArrayList<ByteBuffer> faces = new ArrayList<>();
		Vector3f direction = new Vector3f();
		for (int face = 0; face < 6; face++) {
			ByteBuffer pixels = BufferUtils.createByteBuffer(size * size * 3 * 4).order(ByteOrder.nativeOrder());
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					EnvMapUtils.getVectorFromCubemapFaceTexCoord(x, y, size, face, direction,
							EnvMapUtils.FixSeamsMethod.Stretch);
					float height = Math.max(0f, direction.y);
					float zenith = (float) Math.pow(height, 0.45);
					float haze = (float) Math.exp(-height * 6f);
					float cloud = 0;
					if (direction.y > 0.07f) {
						float cx = direction.x / (direction.y + 0.15f), cz = direction.z / (direction.y + 0.15f);
						float noise = (float) (0.52 + 0.20 * Math.sin(cx * 2.1 + Math.sin(cz * 1.3))
								+ 0.15 * Math.sin(cz * 3.7 - cx * 0.8) + 0.08 * Math.sin(cx * 8.1 + cz * 6.5));
						cloud = FastMath.clamp((noise - 0.66f) * 3f, 0f, 0.75f)
								* FastMath.clamp((direction.y - 0.07f) * 5f, 0f, 1f);
					}
					float alignment = Math.max(0f, direction.dot(towardSun));
					float glow = 0.30f * (float) Math.pow(alignment, 32);
					float disc = 9f * (float) Math.pow(alignment, 6500);
					float red = 0.075f * zenith + 0.40f * (1 - zenith) + haze * 0.06f;
					float green = 0.23f * zenith + 0.60f * (1 - zenith);
					float blue = 0.62f * zenith + 0.82f * (1 - zenith);
					if (direction.y < 0) {
						// Keep the visible horizon continuous; the lower hemisphere represents open ocean.
						float belowHorizon = (float) Math.sqrt(-direction.y);
						red = 0.46f * (1 - belowHorizon) + 0.08f * belowHorizon;
						green = 0.60f * (1 - belowHorizon) + 0.17f * belowHorizon;
						blue = 0.82f * (1 - belowHorizon) + 0.28f * belowHorizon;
					}
					pixels.putFloat(red * (1 - cloud) + 0.93f * cloud + glow + disc);
					pixels.putFloat(green * (1 - cloud) + 0.91f * cloud + glow * 0.85f + disc * 0.82f);
					pixels.putFloat(blue * (1 - cloud) + 0.86f * cloud + glow * 0.65f + disc * 0.55f);
				}
			}
			pixels.flip();
			faces.add(pixels);
		}
		Image image = new Image(Image.Format.RGB32F, size, size, 1, faces, ColorSpace.Linear);
		var texture = new TextureCubeMap(image);
		texture.setMinFilter(Texture.MinFilter.BilinearNoMipMaps);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		return texture;
	}
}
