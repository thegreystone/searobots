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
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;

/**
 * Renders a vehicle OBJ the way SubmarineScene3D does (same rotation, back-face culling off, lit by
 * a directional light) from a few camera angles, saves screenshots, then exits. Lets you check a
 * model under real jME lighting without launching the full viewer; needs a display and an OpenGL
 * context (a window opens briefly).
 * <p>
 * Usage: {@code ModelRenderCheck <out-dir> [model path]}, default model
 * {@code models/surface-ship.obj}, e.g.
 * {@code mvn -q dependency:build-classpath -Dmdep.outputFile=cp.txt} then
 * {@code java -cp "target/classes;$(cat cp.txt)" se.hirt.searobots.viewer.tools.ModelRenderCheck out/}.
 * Regenerate the ship itself with {@link ShipModelGenerator}.
 */
public final class ModelRenderCheck extends SimpleApplication {
	static String outDir;
	static String modelPath = "models/surface-ship.obj";
	int frame;
	Node ship;
	ScreenshotAppState shots;
	final Vector3f[] camPos = {new Vector3f(140, 45, 190), // starboard bow, above
			new Vector3f(-160, 25, -120), // port quarter, low
			new Vector3f(0, 40, -220), // dead astern (propeller and rudder)
			new Vector3f(10, 260, 10), // top down
			new Vector3f(-45, 12, 125), // port bow, close and low
			new Vector3f(0, 4, 150), // dead ahead at the waterline
	};

	public static void main(String[] args) {
		outDir = args[0].endsWith("/") ? args[0] : args[0] + "/";
		if (args.length > 1)
			modelPath = args[1];
		var app = new ModelRenderCheck();
		var settings = new AppSettings(true);
		settings.setWidth(1280);
		settings.setHeight(720);
		settings.setTitle("ship render check");
		settings.setSamples(4);
		app.setSettings(settings);
		app.setShowSettings(false);
		app.setPauseOnLostFocus(false);
		app.start();
	}

	@Override
	public void simpleInitApp() {
		flyCam.setEnabled(false);
		viewPort.setBackgroundColor(new ColorRGBA(0.55f, 0.70f, 0.85f, 1f));
		Spatial hull = assetManager.loadModel(modelPath);
		hull.depthFirstTraversal(s -> {
			if (s instanceof Geometry g && g.getMaterial() != null)
				g.getMaterial().getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
		});
		ship = new Node("shipTemplate");
		ship.attachChild(hull);
		ship.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0)); // as in the viewer
		rootNode.attachChild(ship);

		// Sea plane at the waterline so the draft reads correctly
		var sea = new Geometry("sea", new com.jme3.scene.shape.Quad(1200, 1200));
		var seaMat = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
		seaMat.setBoolean("UseMaterialColors", true);
		seaMat.setColor("Diffuse", new ColorRGBA(0.05f, 0.25f, 0.40f, 1f));
		seaMat.setColor("Ambient", new ColorRGBA(0.05f, 0.25f, 0.40f, 1f));
		sea.setMaterial(seaMat);
		sea.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0));
		sea.setLocalTranslation(-600, 0, 600);
		rootNode.attachChild(sea);

		var sun = new DirectionalLight();
		sun.setDirection(new Vector3f(-0.4f, -0.8f, -0.35f).normalizeLocal());
		sun.setColor(ColorRGBA.White.mult(1.1f));
		rootNode.addLight(sun);
		var ambient = new AmbientLight();
		ambient.setColor(ColorRGBA.White.mult(0.35f));
		rootNode.addLight(ambient);

		cam.setFrustumPerspective(45f, (float) cam.getWidth() / cam.getHeight(), 1f, 3000f);
		shots = new ScreenshotAppState(outDir, "ship-view", 0);
		stateManager.attach(shots);
	}

	@Override
	public void simpleUpdate(float tpf) {
		int view = frame / 6; // six frames per view: settle, then capture on the third
		if (view >= camPos.length) {
			stop();
			return;
		}
		cam.setLocation(camPos[view]);
		cam.lookAt(view >= 4 ? new Vector3f(0, 6, 60) : new Vector3f(0, 8, 0), Vector3f.UNIT_Y);
		if (frame % 6 == 3)
			shots.takeScreenshot();
		frame++;
	}
}
