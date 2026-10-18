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

import com.jme3.asset.DesktopAssetManager;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.engine.GeneratedWorld;
import java.util.List;

import java.awt.Color;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class SonarPingEffectsTest {
	@Test
	void bottomContactWaitsForVerticalTravelThenUsesSphereIntersection() {
		var geometry = new SonarPingGeometry(flat(), 0, 0, -100);
		var lines = new SonarPingGeometry.Lines();
		geometry.bottom(199, lines);
		assertEquals(0, lines.size, "A 200 m bottom clearance needs 200 m of wave travel");
		geometry.bottom(250, lines);
		assertTrue(lines.size > 0);
		for (int i = 0; i < lines.size; i += 3) {
			assertEquals(150, Math.hypot(lines.positions[i], lines.positions[i + 2]), 0.25,
					"Bottom radius is sqrt(250^2 - 200^2), not the sphere radius");
			assertEquals(-299.2, lines.positions[i + 1], 0.001);
		}
	}

	@Test
	void sourceRaysAllowFirstBottomContactButRejectAnEarlierRidge() {
		assertTrue(SonarPingGeometry.clearPath(flat(), -150, 0, -100, 150, 0, -300));
		assertFalse(SonarPingGeometry.clearPath(ridge(), -150, 0, -100, 150, 0, -300));
		assertTrue(SonarPingGeometry.clearPath(ridge(), -150, 0, -100, -10, 0, -50),
				"The first surface itself can receive the pulse");
	}

	@Test
	void sphereAndBottomStayInTheSourcesVisibleWater() {
		TerrainMap terrain = ridge();
		var geometry = new SonarPingGeometry(terrain, -150, 0, -100);
		var lines = new SonarPingGeometry.Lines();
		geometry.sphere(350, lines);
		assertTrue(lines.size > 0);
		for (int i = 0; i < lines.size; i += 3) {
			double x = lines.positions[i], y = -lines.positions[i + 2], z = lines.positions[i + 1];
			assertTrue(z <= 0.001, "The pulse stays underwater");
			assertTrue(z >= terrain.elevationAt(x, y) - 0.001);
			assertTrue(SonarPingGeometry.clearPath(terrain, -150, 0, -100, x, y, z));
		}
		geometry.bottom(350, lines);
		assertTrue(lines.size > 0);
		for (int i = 0; i < lines.size; i += 3) {
			assertTrue(lines.positions[i] <= 0, "The bright contour cannot reappear behind the full-height ridge");
		}
	}

	@Test
	void simulationClockControlsRadiusAtNonDefaultTickRates() {
		assertEquals(1500, SonarPingGeometry.radius(150, 50, 100));
		assertEquals(1500, SonarPingGeometry.radius(100, 50, 50));
		assertEquals(0, SonarPingGeometry.radius(50, 50, 50));
	}

	@Test
	void retainedEventsFreezeWhenPausedAndDisappearWhenDisabledOrExpired() {
		var terrain = flat();
		var data = new MapRenderer(world(terrain), new OverlayConfig());
		var root = new Node();
		var effects = new SonarPingEffects(new DesktopAssetManager(true), root);
		data.pingAnimations.add(new MapRenderer.PingAnimation(0, 0, -100, 0, Color.CYAN, 7));
		data.simTick = 10;
		effects.update(data, terrain, 50, true);
		Node node = (Node) root.getChild("sonar-pings");
		assertEquals(2, node.getQuantity());
		Geometry bottom = (Geometry) node.getChild("sonar-bottom");
		var positions = bottom.getMesh().getBuffer(VertexBuffer.Type.Position).getData();
		int vertices = bottom.getMesh().getVertexCount();
		assertTrue(vertices > 0);
		assertEquals(RenderQueue.Bucket.Translucent, bottom.getQueueBucket());
		assertTrue(bottom.getMaterial().getAdditionalRenderState().isDepthTest());
		assertFalse(bottom.getMaterial().getAdditionalRenderState().isDepthWrite());
		effects.update(data, terrain, 50, true);
		assertSame(bottom, node.getChild("sonar-bottom"));
		assertSame(positions, bottom.getMesh().getBuffer(VertexBuffer.Type.Position).getData());
		assertEquals(vertices, bottom.getMesh().getVertexCount());
		effects.update(data, terrain, 50, false);
		assertEquals(0, node.getQuantity());
		effects.update(data, terrain, 50, true);
		assertEquals(2, node.getQuantity());
		data.simTick = 1000;
		effects.update(data, terrain, 50, true);
		assertEquals(0, node.getQuantity(), "Fast-forward must not allocate meshes for stale pulses");
	}

	@Test
	void resettingRetainedMapEventsClearsExistingThreeDimensionalPulses() {
		var terrain = flat();
		var data = new MapRenderer(world(terrain), new OverlayConfig());
		var root = new Node();
		var effects = new SonarPingEffects(new DesktopAssetManager(true), root);
		data.pingAnimations.add(new MapRenderer.PingAnimation(0, 0, -100, 0, Color.CYAN, 7));
		data.simTick = 10;
		effects.update(data, terrain, 50, true);
		data.setWorld(world(terrain));
		effects.update(data, terrain, 50, true);
		assertEquals(0, ((Node) root.getChild("sonar-pings")).getQuantity());
	}

	@Test
	void acceleratedTerrainRaysMatchHalfCellSamplingAcrossTileBoundaries() {
		var random = new java.util.Random(42);
		double[] heights = new double[81 * 81];
		for (int row = 0; row < 81; row++)
			for (int col = 0; col < 81; col++)
				heights[row * 81 + col] = -300 + 100 * Math.sin(col / 4.0) * Math.cos(row / 5.0);
		var terrain = new TerrainMap(heights, 81, 81, -400, -400, 10);
		var visibility = SonarTerrainVisibility.forTerrain(terrain);
		assertSame(visibility, SonarTerrainVisibility.forTerrain(terrain), "Pulses share the terrain index");
		for (int ray = 0; ray < 1000; ray++) {
			double sx = random.nextDouble(-400, 400), sy = random.nextDouble(-400, 400);
			double tx = ray % 4 == 0 ? sx : random.nextDouble(-400, 400);
			double ty = ray % 4 == 1 ? sy : random.nextDouble(-400, 400);
			double sz = terrain.elevationAt(sx, sy) + random.nextDouble(1, 250);
			double tz = terrain.elevationAt(tx, ty) + (ray % 3 == 0 ? 0 : random.nextDouble(1, 250));
			int samples = Math.max(2, (int) Math.ceil(Math.hypot(tx - sx, ty - sy) / 5));
			boolean expected = true;
			for (int i = 1; i < samples; i++) {
				double t = (double) i / samples;
				if (terrain.elevationAt(sx + (tx - sx) * t, sy + (ty - sy) * t) > sz + (tz - sz) * t + 0.02) {
					expected = false;
					break;
				}
			}
			assertEquals(expected, visibility.clear(sx, sy, sz, tx, ty, tz), "Ray " + ray);
		}
	}

	private static GeneratedWorld world(TerrainMap terrain) {
		return new GeneratedWorld(MatchConfig.withDefaults(0), terrain, List.of(), new CurrentField(List.of()),
				List.of());
	}

	private static TerrainMap flat() {
		double[] elevations = new double[81 * 81];
		Arrays.fill(elevations, -300);
		return new TerrainMap(elevations, 81, 81, -400, -400, 10);
	}

	private static TerrainMap ridge() {
		double[] elevations = new double[81 * 81];
		Arrays.fill(elevations, -300);
		for (int row = 0; row < 81; row++) {
			for (int col = 39; col <= 41; col++)
				elevations[row * 81 + col] = -50;
		}
		return new TerrainMap(elevations, 81, 81, -400, -400, 10);
	}
}
