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

import java.nio.FloatBuffer;
import java.util.List;
import java.util.Random;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.terrain.geomipmap.TerrainPatch;
import com.jme3.terrain.geomipmap.TerrainQuad;
import org.junit.jupiter.api.Test;

import se.hirt.searobots.api.TerrainMap;

import static org.junit.jupiter.api.Assertions.*;

class IslandSceneryControlTest {

	@Test
	void heightSnapshotPreservesPendingAncestorLightingAndUsesNestedWorldTransforms() {
		int size = 65;
		float[] heights = new float[size * size];
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++)
				heights[row * size + col] = 3f + col * .25f + row * .75f;
		}
		Node root = new Node("pending lit root");
		root.addLight(new DirectionalLight());
		root.addLight(new DirectionalLight());
		root.addLight(new AmbientLight());
		Node island = new Node("transformed island");
		island.setLocalScale(2f);
		island.setLocalTranslation(200f, 7f, -100f);
		TerrainQuad quad = TerrainQuadBuilder.createGeometry(heights, 17, size, 4f);
		quad.setLocalTranslation(10f, 5f, -20f);
		quad.updateGeometricState();
		island.attachChild(quad);
		root.attachChild(island);
		assertEquals(0, root.getWorldLightList().size(), "Ancestor lights have not been refreshed yet");

		// Preparing background height data may refresh transforms, but must not consume the
		// terrain's pending light refresh before its ancestor world light lists are populated.
		IslandScenerySurface snapshot = new IslandScenerySurface(quad);
		assertEquals(0, root.getWorldLightList().size());
		for (Vector2f grid : List.of(new Vector2f(18.25f, 23.75f), new Vector2f(32.5f, 32.75f),
				new Vector2f(47.25f, 18.5f))) {
			float x = 220f + (grid.x - 32f) * 8f;
			float z = -140f + (grid.y - 32f) * 8f;
			float expected = 17f + (3f + grid.x * .25f + grid.y * .75f) * 2f;
			assertEquals(expected, snapshot.height(x, z), .0002,
					"Snapshot must include parent and terrain translations and scales");
		}
		root.updateGeometricState();
		assertEquals(3, root.getWorldLightList().size());
		assertEquals(3, quad.getWorldLightList().size());
		assertEquals(16, assertTerrainPatchLighting(quad, 3));
	}

	@Test
	void backgroundHeightSnapshotMatchesRenderedTrianglesIncludingPatchCorners() {
		int size = 65;
		float[] heights = new float[size * size];
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++)
				heights[row * size
						+ col] = (float) (3 * Math.sin(col * .51) * Math.cos(row * .23) + col * .02 + row * .08);
		}
		TerrainQuad quad = TerrainQuadBuilder.createGeometry(heights, 17, size, 4);
		quad.setLocalTranslation(1000, 7, -2000);
		quad.updateGeometricState();
		IslandScenerySurface snapshot = new IslandScenerySurface(quad);
		Random random = new Random(42);
		for (int sample = 0; sample < 500; sample++) {
			float x = 872 + random.nextFloat() * 255, z = -2128 + random.nextFloat() * 255;
			assertEquals(quad.getHeight(new Vector2f(x, z)) + 7, snapshot.height(x, z), .0002,
					"Scenery must follow rendered triangles rather than simulation bilinear interpolation");
		}
		for (int corner : new int[] {0, 15, 16, 31, 32, 47}) {
			for (float fx : new float[] {.2f, .8f}) {
				float x = 872 + (corner + fx) * 4, z = -2128 + (corner + .4f) * 4;
				assertEquals(quad.getHeight(new Vector2f(x, z)) + 7, snapshot.height(x, z), .0002);
			}
		}
		float before = snapshot.height(1000, -2000);
		quad.setHeight(new Vector2f(1000, -2000), 100);
		assertEquals(before, snapshot.height(1000, -2000), "Background workers retain an immutable snapshot");
		assertTrue(Float.isNaN(snapshot.height(5000, -2000)));
	}

	@Test
	void seededBatchesKeepBeachesAndWaterOpenAndAnchorGrassToTheRenderedSurface() {
		Fixture fixture = coast();
		var assets = new DesktopAssetManager(true);
		var library = new IslandSceneryControl.Library(assets);
		var key = new IslandSceneryControl.Key(0, 2);
		var first = IslandSceneryControl.buildCell(key, fixture.surface, fixture.map, library, 42);
		var again = IslandSceneryControl.buildCell(key, fixture.surface, fixture.map, library, 42);
		var other = IslandSceneryControl.buildCell(key, fixture.surface, fixture.map, library, 123);
		assertTrue(first.grassCount > 0);
		assertTrue(first.shrubCount > 0);
		assertEquals(signature(first.node), signature(again.node));
		assertNotEquals(signature(first.node), signature(other.node));
		var beach = IslandSceneryControl.buildCell(new IslandSceneryControl.Key(0, 0), fixture.surface, fixture.map,
				library, 42);
		assertEquals(0, beach.grassCount);
		assertEquals(0, beach.shrubCount);
		assertTrue(beach.rockCount > 0, "Exercise real tide boulders as well as foliage");
		var water = IslandSceneryControl.buildCell(new IslandSceneryControl.Key(0, -2), fixture.surface, fixture.map,
				library, 42);
		assertEquals(0, water.triangles);
		List<Spatial> batches = new java.util.ArrayList<>(first.node.getChildren());
		batches.addAll(beach.node.getChildren());
		for (Spatial child : batches) {
			Geometry geometry = (Geometry) child;
			FloatBuffer positions = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
			FloatBuffer normals = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Normal);
			FloatBuffer uv = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.TexCoord);
			FloatBuffer tangents = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Tangent);
			for (int vertex = 0; vertex < geometry.getMesh().getVertexCount(); vertex++) {
				float x = positions.get(vertex * 3), y = positions.get(vertex * 3 + 1),
						z = positions.get(vertex * 3 + 2);
				assertTrue(Float.isFinite(x) && Float.isFinite(y) && Float.isFinite(z));
				Vector3f normal = new Vector3f(normals.get(vertex * 3), normals.get(vertex * 3 + 1),
						normals.get(vertex * 3 + 2));
				assertEquals(1, normal.length(), .003);
				if (tangents != null) {
					Vector3f tangent = new Vector3f(tangents.get(vertex * 4), tangents.get(vertex * 4 + 1),
							tangents.get(vertex * 4 + 2));
					assertTrue(Float.isFinite(tangent.lengthSquared()));
					if (tangent.lengthSquared() > .000001f) {
						assertEquals(1, tangent.length(), .003);
						assertEquals(0, tangent.dot(normal), .0002);
					}
				}
				if (geometry == first.grass && uv.get(vertex * 2 + 1) == 0)
					assertEquals(fixture.quad.getHeight(new Vector2f(x, z)), y, .0002);
			}
		}
	}

	@Test
	void stationaryUpdatesPreserveAttachmentsAndInheritedLightingAfterStreamingSettles() throws Exception {
		Fixture fixture = coast();
		var assets = new DesktopAssetManager(true);
		Camera camera = new Camera(1600, 900);
		camera.setFrustumPerspective(54, 16f / 9, .2f, 1000);
		camera.setLocation(new Vector3f(0, 22, 70));
		camera.lookAt(new Vector3f(0, 20, 130), Vector3f.UNIT_Y);
		var scenery = new AttachmentTrackingNode();
		var control = new IslandSceneryControl(scenery, fixture.map, fixture.surface, assets, camera, 42);
		scenery.addControl(control);
		Node root = new Node("lit scenery root");
		root.addLight(new AmbientLight());
		root.addLight(new DirectionalLight());
		root.attachChild(scenery);
		try {
			long deadline = System.nanoTime() + 6_000_000_000L;
			int quiet = 0;
			while (quiet < 40 && System.nanoTime() < deadline) {
				root.updateLogicalState(1f / 60);
				root.updateGeometricState();
				quiet = control.statistics().pendingJobs() == 0 ? quiet + 1 : 0;
				Thread.sleep(2);
			}
			assertTrue(quiet >= 40, "Preparation must finish before checking stationary render work");
			assertTrue(control.statistics().activeCells() > 0);
			assertTrue(control.statistics().grassClumps() > 0);
			int attachments = scenery.attachments, detachments = scenery.detachments;
			for (int frame = 0; frame < 90; frame++) {
				root.updateLogicalState(1f / 60);
				assertEquals(attachments, scenery.attachments,
						"Settled camera selection must not reattach unchanged scenery cells");
				assertEquals(detachments, scenery.detachments);
				assertEquals(0, scenery.pendingRefreshFlags(),
						"Stationary scenery must not invalidate transforms, bounds or inherited lights");
				for (Spatial cell : scenery.getChildren()) {
					assertSame(scenery, cell.getParent());
					for (Spatial geometry : ((Node) cell).getChildren())
						assertEquals(2, geometry.getWorldLightList().size());
				}
				root.updateGeometricState();
				assertLimits(control);
			}
		} finally {
			control.close();
		}
		assertEquals(0, scenery.getQuantity());
		assertEquals(0, control.statistics().residentCells());
	}

	@Test
	void backgroundStreamingBoundsGeometryAndCacheAcrossMovementAndDisposal() throws Exception {
		Fixture fixture = coast();
		var assets = new DesktopAssetManager(true);
		Camera camera = new Camera(1600, 900);
		camera.setFrustumPerspective(54, 16f / 9, .2f, 1000);
		Node scenery = new Node("test scenery");
		var control = new IslandSceneryControl(scenery, fixture.map, fixture.surface, assets, camera, 42);
		scenery.addControl(control);
		assertEquals(0, control.statistics().residentCells());
		try {
			for (Vector3f position : List.of(new Vector3f(0, 22, 70), new Vector3f(160, 22, 70),
					new Vector3f(-160, 35, 145), new Vector3f(80, 38, 160), new Vector3f(-80, 22, 70),
					new Vector3f(0, 22, 70))) {
				camera.setLocation(position);
				camera.lookAt(position.add(0, -2, 60), Vector3f.UNIT_Y);
				camera.setPlaneState(23);
				long deadline = System.nanoTime() + 6_000_000_000L;
				int quiet = 0;
				while (quiet < 40 && System.nanoTime() < deadline) {
					int before = control.statistics().completedCells();
					scenery.updateLogicalState(1f / 60);
					assertEquals(23, camera.getPlaneState(), "Selection must preserve later reflection/scene culling");
					assertTrue(control.statistics().completedCells() - before <= 1);
					assertLimits(control);
					quiet = control.statistics().pendingJobs() == 0 ? quiet + 1 : 0;
					Thread.sleep(2);
				}
				assertTrue(quiet >= 40, "Background preparation should settle");
				assertTrue(control.statistics().activeTriangles() > 0);
			}
			assertTrue(control.statistics().completedCells() > IslandSceneryControl.MAX_RESIDENT_CELLS,
					"Camera movement must exercise actual cache eviction");
			camera.setLocation(new Vector3f(5000, 500, 5000));
			scenery.updateLogicalState(1f / 60);
			assertEquals(0, control.statistics().activeTriangles());
		} finally {
			control.close();
		}
		assertEquals(0, scenery.getQuantity());
		assertEquals(0, control.statistics().residentCells());
		assertEquals(0, control.statistics().pendingJobs());
		scenery.updateLogicalState(1f / 60);
		assertEquals(0, control.statistics().activeTriangles());
	}

	private static void assertLimits(IslandSceneryControl control) {
		var statistics = control.statistics();
		assertTrue(statistics.activeCells() <= IslandSceneryControl.MAX_ACTIVE_CELLS);
		assertTrue(statistics.activeTriangles() <= IslandSceneryControl.MAX_ACTIVE_TRIANGLES);
		assertTrue(statistics.activeDraws() <= IslandSceneryControl.MAX_ACTIVE_DRAWS);
		assertTrue(statistics.residentCells() <= IslandSceneryControl.MAX_RESIDENT_CELLS);
		assertTrue(statistics.residentTriangles() <= IslandSceneryControl.MAX_RESIDENT_TRIANGLES);
		assertTrue(statistics.pendingJobs() <= 1);
	}

	private static int assertTerrainPatchLighting(Spatial spatial, int expectedLights) {
		if (spatial instanceof TerrainPatch patch) {
			assertEquals(expectedLights, patch.getWorldLightList().size(),
					"Every rendered terrain patch must inherit the pending root lights");
			return 1;
		}
		int patches = 0;
		if (spatial instanceof Node node) {
			for (Spatial child : node.getChildren())
				patches += assertTerrainPatchLighting(child, expectedLights);
		}
		return patches;
	}

	private static long signature(Node node) {
		long result = 1;
		for (Spatial child : node.getChildren()) {
			FloatBuffer positions = ((Geometry) child).getMesh().getFloatBuffer(VertexBuffer.Type.Position);
			for (int index = 0; index < positions.limit(); index++)
				result = result * 31 + Float.floatToIntBits(positions.get(index));
		}
		return result;
	}

	private static Fixture coast() {
		int size = 129;
		float cell = 4;
		float[] heights = new float[size * size];
		double[] elevations = new double[size * size];
		for (int row = 0; row < size; row++) {
			for (int col = 0; col < size; col++) {
				float z = (row - 64) * cell;
				heights[row * size + col] = z * .22f;
				elevations[(size - 1 - row) * size + col] = heights[row * size + col];
			}
		}
		TerrainQuad quad = TerrainQuadBuilder.createGeometry(heights, 17, size, cell);
		quad.updateGeometricState();
		return new Fixture(new TerrainMap(elevations, size, size, -256, -256, cell), quad,
				new IslandScenerySurface(quad));
	}

	private record Fixture(TerrainMap map, TerrainQuad quad, IslandScenerySurface surface) {
	}

	private static final class AttachmentTrackingNode extends Node {
		int attachments, detachments;

		@Override
		public int attachChild(Spatial child) {
			attachments++;
			return super.attachChild(child);
		}

		@Override
		public int detachChild(Spatial child) {
			detachments++;
			return super.detachChild(child);
		}

		int pendingRefreshFlags() {
			return refreshFlags;
		}
	}
}
