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

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Random;

import com.jme3.app.SimpleApplication;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.effect.Particle;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
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
import com.jme3.post.filters.TranslucentBucketFilter;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext;

/**
 * Deterministic wake comparisons through real water/fog postprocessing, without running a match.
 * Usage: {@code WakeRenderCheck <output-directory>}. Runs offscreen and writes paired before/after
 * PNGs from nearby, below, above, distant, and atmosphere-off views. Camera framing keeps the wake
 * at a comparable size at different ranges so its attenuation can be inspected.
 */
public final class WakeRenderCheck extends SimpleApplication {
	private record View(String name, Vector3f position, boolean atmosphere) {
	}

	private static final Vector3f TARGET = new Vector3f(0, -40, 12);
	private static final View[] VIEWS = {new View("near", new Vector3f(22, -34, 30), true),
			new View("below", new Vector3f(22, -72, 30), true), new View("above", new Vector3f(35, 35, 45), true),
			new View("far", new Vector3f(450, -40, 350), true),
			new View("far-clear", new Vector3f(450, -40, 350), false)};
	private final String directory;
	private ParticleEmitter wake;
	private Material before, after;
	private AtmosphereWaterFilter water;
	private FogFilter fog;
	private ScreenshotAppState screenshots;
	private int frame;

	private WakeRenderCheck(String directory) {
		this.directory = directory;
	}

	public static void main(String[] args) throws Exception {
		Path directory = Path.of(args.length == 0 ? "target/wake-check" : args[0]).toAbsolutePath();
		Files.createDirectories(directory);
		WakeRenderCheck app = new WakeRenderCheck(directory.toString() + "/");
		AppSettings settings = new AppSettings(true);
		settings.setResolution(1280, 720);
		settings.setSamples(0);
		settings.setAudioRenderer(null);
		settings.setFrameRate(30);
		app.setSettings(settings);
		app.setShowSettings(false);
		app.setPauseOnLostFocus(false);
		app.start(JmeContext.Type.OffscreenSurface);
	}

	@Override
	public void simpleInitApp() {
		flyCam.setEnabled(false);
		setDisplayFps(false);
		setDisplayStatView(false);
		viewPort.setBackgroundColor(new ColorRGBA(0.18f, 0.35f, 0.48f, 1f));
		rootNode.addLight(new AmbientLight(new ColorRGBA(0.45f, 0.5f, 0.6f, 1f)));
		Vector3f sunDirection = new Vector3f(-0.4f, -0.8f, -0.35f).normalizeLocal();
		rootNode.addLight(new DirectionalLight(sunDirection, ColorRGBA.White));

		Spatial torpedo = assetManager.loadModel("models/torpedo.obj");
		torpedo.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, FastMath.PI, 0));
		torpedo.setLocalTranslation(0, -40, 0);
		rootNode.attachChild(torpedo);

		Geometry seabed = new Geometry("seabed", new Box(3000, 1, 3000));
		Material sand = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
		sand.setColor("Color", new ColorRGBA(0.22f, 0.23f, 0.19f, 1f));
		seabed.setMaterial(sand);
		seabed.setLocalTranslation(0, -120, 0);
		rootNode.attachChild(seabed);

		fog = new FogFilter();
		fog.setFogColor(new ColorRGBA(0.03f, 0.07f, 0.17f, 1f));
		fog.setFogDistance(1800f);
		fog.setFogDensity(1.8f);
		water = new AtmosphereWaterFilter(rootNode, sunDirection);
		water.setWaterHeight(0f);
		water.setWaterTransparency(0.16f);
		water.setUnderWaterFogDistance(376f);
		water.setWaterColor(new ColorRGBA(0.025f, 0.17f, 0.28f, 1f));
		water.setDeepWaterColor(new ColorRGBA(0.006f, 0.045f, 0.12f, 1f));
		water.setWaveScale(0.006f);
		water.setMaxAmplitude(0.45f);
		water.setSpeed(0f); // identical waves in the paired images
		water.setUseCaustics(false);
		WaterQualitySettings.configure(water);
		FilterPostProcessor processor = new FilterPostProcessor(assetManager);
		processor.addFilter(fog);
		processor.addFilter(water);
		processor.addFilter(new TranslucentBucketFilter());
		viewPort.addProcessor(processor);

		before = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
		before.setTexture("Texture", assetManager.loadTexture("Effects/Explosion/smoketrail.png"));
		before.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Additive);
		before.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
		after = WakeMaterial.create(assetManager);
		wake = new ParticleEmitter("frozen-wake", ParticleMesh.Type.Triangle, 150);
		wake.setMaterial(after);
		wake.setQueueBucket(RenderQueue.Bucket.Translucent);
		wake.setCullHint(Spatial.CullHint.Never); // fixed particle positions bypass the emitter's bound update
		wake.setImagesX(1);
		wake.setImagesY(3);
		wake.setParticlesPerSec(0f);
		wake.setLowLife(10f);
		wake.setHighLife(10f);
		rootNode.attachChild(wake);
		rootNode.updateGeometricState();
		wake.emitAllParticles();
		Random random = new Random(42);
		int index = 0;
		for (Particle particle : wake.getParticles()) {
			float age = (index++ + 0.5f) / wake.getMaxNumParticles();
			float spread = 0.08f + 0.9f * age;
			particle.position.set((random.nextFloat() - 0.5f) * spread, -40f + age * 0.8f, 3f + age * 30f);
			particle.size = 0.3f + 1.7f * age;
			particle.color.set(1f - 0.3f * age, 1f - 0.15f * age, 1f, 0.5f * (1f - age));
			particle.velocity.set(0, 0, 0);
		}
		wake.setEnabled(false);
		screenshots = new ScreenshotAppState(directory);
		screenshots.setIsNumbered(false);
		stateManager.attach(screenshots);
	}

	@Override
	public void simpleUpdate(float tpf) {
		int pair = frame / 6;
		if (pair >= VIEWS.length * 2) {
			stop();
			return;
		}
		View view = VIEWS[pair / 2];
		boolean legacy = pair % 2 == 0;
		cam.setLocation(view.position());
		cam.lookAt(TARGET, Vector3f.UNIT_Y);
		float fov = 2f * FastMath.atan(22f / cam.getLocation().distance(TARGET)) * FastMath.RAD_TO_DEG;
		cam.setFrustumPerspective(fov, (float) cam.getWidth() / cam.getHeight(), 1f, 5000f);
		fog.setEnabled(view.atmosphere());
		water.setFogEnabled(view.atmosphere());
		WakeMaterial.update(after, water, fog, view.atmosphere());
		wake.setMaterial(legacy ? before : after);
		if (frame % 6 == 3) {
			screenshots.setFileName("wake-" + view.name() + (legacy ? "-before" : "-after"));
			screenshots.takeScreenshot();
		}
		frame++;
	}
}
