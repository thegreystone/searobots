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
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.TranslucentBucketFilter;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.engine.GeneratedWorld;

import java.awt.Color;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/** Deterministic offscreen check of bottom contact, terrain shadows and the sonar toggle. */
public final class SonarRenderCheck extends SimpleApplication {
	private static final int GRID = 121;
	private final String directory;
	private TerrainMap terrain;
	private MapRenderer data;
	private SonarPingEffects effects;
	private ScreenshotAppState screenshots;
	private int frame;

	private SonarRenderCheck(String directory) {
		this.directory = directory;
	}

	public static void main(String[] args) throws Exception {
		Path output = Path.of(args.length == 0 ? "target/sonar-check" : args[0]).toAbsolutePath();
		Files.createDirectories(output);
		var app = new SonarRenderCheck(output.toString() + "/");
		var settings = new AppSettings(true);
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
		viewPort.setBackgroundColor(new ColorRGBA(0.015f, 0.035f, 0.07f, 1));
		double[] elevations = new double[GRID * GRID];
		float[] positions = new float[elevations.length * 3];
		float[] colors = new float[elevations.length * 4];
		for (int row = 0; row < GRID; row++) {
			for (int col = 0; col < GRID; col++) {
				int i = row * GRID + col;
				float x = -600 + col * 10, y = -600 + row * 10;
				float ridge = Math.max(0, 1 - Math.abs(x - 80) / 80);
				float z = -300 + ridge * 240;
				elevations[i] = z;
				positions[i * 3] = x;
				positions[i * 3 + 1] = z;
				positions[i * 3 + 2] = -y;
				float grid = (row % 10 == 0 || col % 10 == 0) ? 0.045f : 0;
				colors[i * 4] = 0.12f + ridge * 0.11f + grid;
				colors[i * 4 + 1] = 0.19f + ridge * 0.08f + grid;
				colors[i * 4 + 2] = 0.21f + ridge * 0.07f + grid;
				colors[i * 4 + 3] = 1;
			}
		}
		terrain = new TerrainMap(elevations, GRID, GRID, -600, -600, 10);
		int[] indices = new int[(GRID - 1) * (GRID - 1) * 6];
		int next = 0;
		for (int row = 0; row < GRID - 1; row++) {
			for (int col = 0; col < GRID - 1; col++) {
				int a = row * GRID + col, b = a + 1, c = a + GRID, d = c + 1;
				indices[next++] = a;
				indices[next++] = b;
				indices[next++] = d;
				indices[next++] = a;
				indices[next++] = d;
				indices[next++] = c;
			}
		}
		Mesh mesh = new Mesh();
		mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
		mesh.setBuffer(VertexBuffer.Type.Color, 4, colors);
		mesh.setBuffer(VertexBuffer.Type.Index, 3, indices);
		mesh.updateBound();
		Geometry floor = new Geometry("terrain", mesh);
		Material material = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setBoolean("VertexColor", true);
		floor.setMaterial(material);
		rootNode.attachChild(floor);
		Geometry source = new Geometry("ping-origin", new Sphere(12, 20, 10));
		Material marker = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
		marker.setColor("Color", new ColorRGBA(1, 0.7f, 0.15f, 1));
		source.setMaterial(marker);
		source.setLocalTranslation(-150, -100, 0);
		rootNode.attachChild(source);
		var world = new GeneratedWorld(MatchConfig.withDefaults(0), terrain, List.of(), new CurrentField(List.of()),
				List.of());
		data = new MapRenderer(world, new OverlayConfig());
		data.pingAnimations.add(new MapRenderer.PingAnimation(-150, 0, -100, 0, Color.CYAN, 0));
		effects = new SonarPingEffects(assetManager, rootNode);
		cam.setLocation(new Vector3f(-650, 550, 850));
		cam.lookAt(new Vector3f(0, -180, 0), Vector3f.UNIT_Y);
		cam.setFrustumPerspective(52, 1280f / 720, 1, 4000);
		var processor = new FilterPostProcessor(assetManager);
		processor.addFilter(new TranslucentBucketFilter());
		viewPort.addProcessor(processor);
		screenshots = new ScreenshotAppState(directory);
		screenshots.setIsNumbered(false);
		stateManager.attach(screenshots);
	}

	@Override
	public void simpleUpdate(float tpf) {
		int stage = frame / 8;
		if (stage >= 4) {
			stop();
			return;
		}
		data.simTick = stage == 0 ? 3 : stage == 1 ? 8 : 15;
		effects.update(data, terrain, 50, stage != 3);
		if (frame % 8 == 5) {
			String name = switch (stage) {
			case 0 -> "before-contact";
			case 1 -> "bottom-contact";
			case 2 -> "ridge-shadow";
			default -> "disabled";
			};
			screenshots.setFileName("sonar-" + name);
			screenshots.takeScreenshot();
		}
		frame++;
	}
}
