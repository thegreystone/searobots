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
package se.hirt.searobots.viewer.tools;

import com.jme3.app.SimpleApplication;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.BloomFilter;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Quad;
import com.jme3.system.AppSettings;

import se.hirt.searobots.viewer.ExplosionEffects;
import se.hirt.searobots.viewer.SubmarineModelSupport;

/**
 * Sets off a torpedo hit next to a submarine with {@link ExplosionEffects}, under a bloom filter as
 * in SubmarineScene3D, and saves screenshots at moments through the blast, then exits. Lets you
 * tune the explosion without running a match; needs a display and an OpenGL context (a window opens
 * briefly).
 * <p>
 * Usage: {@code ExplosionRenderCheck <out-dir> [under|above]}: the camera sits underwater near the
 * blast (default) or above the surface, to see the spray. Classpath as for
 * {@link ModelRenderCheck}.
 */
public final class ExplosionRenderCheck extends SimpleApplication {
	// Seconds after the detonation at which to take a screenshot
	private static final float[] SHOTS = {0.03f, 0.1f, 0.25f, 0.5f, 0.8f, 1.0f, 1.3f, 1.8f, 2.6f, 4f, 6f};
	private static final Vector3f BLAST = new Vector3f(0, -45, 0);
	static String outDir;
	static boolean above;
	ExplosionEffects explosions;
	ScreenshotAppState shots;
	float time = -0.5f; // let the scene settle first
	int next;

	public static void main(String[] args) {
		outDir = args[0].endsWith("/") ? args[0] : args[0] + "/";
		above = args.length > 1 && args[1].equals("above");
		var app = new ExplosionRenderCheck();
		var settings = new AppSettings(true);
		settings.setWidth(1280);
		settings.setHeight(720);
		settings.setTitle("explosion render check");
		settings.setSamples(4);
		settings.setFrameRate(60);
		app.setSettings(settings);
		app.setShowSettings(false);
		app.setPauseOnLostFocus(false);
		app.start();
	}

	@Override
	public void simpleInitApp() {
		flyCam.setEnabled(false);
		setDisplayStatView(false);
		setDisplayFps(false);
		viewPort.setBackgroundColor(
				above ? new ColorRGBA(0.55f, 0.70f, 0.85f, 1f) : new ColorRGBA(0.02f, 0.08f, 0.16f, 1f));

		Spatial sub = assetManager.loadModel("models/submarine-hybrid.obj");
		SubmarineModelSupport.prepare(assetManager, sub);
		sub.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0.4f, 0));
		sub.setLocalTranslation(BLAST.add(-6, 4, 12));
		rootNode.attachChild(sub);

		rootNode.attachChild(plane("floor", -140f, new ColorRGBA(0.35f, 0.33f, 0.28f, 1f)));
		if (above)
			rootNode.attachChild(plane("sea", 0f, new ColorRGBA(0.05f, 0.25f, 0.40f, 1f)));

		var sun = new DirectionalLight();
		sun.setDirection(new Vector3f(-0.4f, -0.8f, -0.35f).normalizeLocal());
		sun.setColor(ColorRGBA.White.mult(above ? 1.1f : 0.35f));
		rootNode.addLight(sun);
		var ambient = new AmbientLight();
		ambient.setColor(ColorRGBA.White.mult(above ? 0.35f : 0.15f));
		rootNode.addLight(ambient);

		var fpp = new FilterPostProcessor(assetManager);
		var bloom = new BloomFilter(BloomFilter.GlowMode.Objects);
		bloom.setBloomIntensity(2.5f);
		bloom.setBlurScale(1.6f);
		fpp.addFilter(bloom);
		viewPort.addProcessor(fpp);

		cam.setFrustumPerspective(50f, (float) cam.getWidth() / cam.getHeight(), 1f, 5000f);
		explosions = new ExplosionEffects(assetManager, rootNode, guiNode, cam);
		shots = new ScreenshotAppState(outDir, above ? "explosion-above" : "explosion-under", 0);
		stateManager.attach(shots);
	}

	private Geometry plane(String name, float y, ColorRGBA color) {
		var g = new Geometry(name, new Quad(3000, 3000));
		var m = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
		m.setBoolean("UseMaterialColors", true);
		m.setColor("Diffuse", color);
		m.setColor("Ambient", color);
		g.setMaterial(m);
		g.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0));
		g.setLocalTranslation(-1500, y, 1500);
		return g;
	}

	@Override
	public void simpleUpdate(float tpf) {
		float before = time;
		time += tpf;
		if (before < 0 && time >= 0)
			explosions.spawn(BLAST, new ColorRGBA(0.9f, 0.3f, 0.2f, 1f), true);
		explosions.update(time >= 0 ? tpf : 0, false);

		Vector3f camPos = above ? new Vector3f(420, 140, 520) : new Vector3f(170, -20, 230);
		Vector3f lookAt = above ? new Vector3f(0, 10, 0) : BLAST.clone();
		explosions.applyShake(camPos, lookAt);
		cam.setLocation(camPos);
		cam.lookAt(lookAt, Vector3f.UNIT_Y);

		if (next >= SHOTS.length) {
			stop();
		} else if (time >= SHOTS[next]) {
			shots.takeScreenshot();
			next++;
		}
	}
}
