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
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.system.JmeSystem;
import org.junit.jupiter.api.Test;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class IslandForestControlTest {

	@Test
	void loadsLocalModelsLazilyAndBoundsTheCacheAcrossCameraMovesAndTierChanges() throws Exception {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		Camera camera = camera();
		Node forest = new Node("forest");
		IslandForestControl control = new IslandForestControl(forest, placements(), assets, camera, 42);
		forest.addControl(control);
		assertTrue(control.statistics().totalTrees() > 10_000);
		assertEquals(0, control.statistics().residentCells(), "Distant forests must not allocate modeled cells");
		assertRepresentationAndLimits(forest, control);

		for (Vector3f position : List.of(new Vector3f(0f, 21f, -25f), new Vector3f(120f, 21f, 60f),
				new Vector3f(240f, 22f, 195f), new Vector3f(-200f, 21f, 280f), new Vector3f(0f, 21f, -100f),
				new Vector3f(0f, 21f, -25f))) {
			camera.setLocation(position);
			camera.lookAt(position.add(0f, 0f, 140f), Vector3f.UNIT_Y);
			for (int frame = 0; frame < 28; frame++) {
				int before = control.statistics().cellBuilds();
				assertTrue(control.awaitPreparation(5000), "Bounded worker must complete preparation");
				forest.updateLogicalState(1f / 60f);
				assertTrue(control.statistics().cellBuilds() - before <= 1, "Build at most one local cell per frame");
				assertRepresentationAndLimits(forest, control);
			}
			assertTrue(control.statistics().modeledTrees() > 0, "Visible forest needs actual modeled crowns");
		}
		assertTrue(control.statistics().cellBuilds() > IslandForestControl.MAX_RESIDENT_CELLS,
				"Movement must exercise eviction, not merely an initially small scene");
		assertTrue(control.statistics().nearCells() > 0);
		assertTrue(control.statistics().middleCells() > 0);
		camera.setLocation(new Vector3f(5000f, -300f, 5000f));
		forest.updateLogicalState(1f / 60f);
		assertEquals(0, control.statistics().modeledTrees());
		assertRepresentationAndLimits(forest, control);
		control.close();
		assertTrue(control.awaitShutdown(5000));
	}

	@Test
	void selectionRestoresCameraPlaneStateForLaterSceneAndReflectionCulling() throws Exception {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		Camera camera = camera();
		Node forest = new Node("forest");
		var trees = new ArrayList<List<TreeScatter.TreeInstance>>();
		for (TreeScatter.Type ignored : TreeScatter.Type.values())
			trees.add(new ArrayList<>());
		trees.get(0).add(new TreeScatter.TreeInstance(0f, 20f, 30f, 8f, 12f, 0f, 1f, 1f, 1f));
		IslandForestControl control = new IslandForestControl(forest, trees, assets, camera, 42);
		forest.addControl(control);
		camera.setPlaneState(23);
		assertTrue(control.awaitPreparation(5000));
		forest.updateLogicalState(1f / 60f);
		assertEquals(23, camera.getPlaneState());
		assertTrue(control.awaitPreparation(5000));
		forest.updateLogicalState(1f / 60f);
		assertEquals(23, camera.getPlaneState());
		assertEquals(1, control.statistics().modeledTrees());
		assertRepresentationAndLimits(forest, control);
		control.close();
		assertTrue(control.awaitShutdown(5000));
	}

	@Test
	void backgroundPreparationCannotAttachAfterTheCameraLeavesOrTheWorldIsDisposed() throws Exception {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		Camera camera = camera();
		Node forest = new Node("forest");
		IslandForestControl control = new IslandForestControl(forest, placements(), assets, camera, 42);
		forest.addControl(control);
		assertTrue(control.awaitPreparation(5000));
		forest.updateLogicalState(1f / 60f);
		assertEquals(1, control.statistics().pendingJobs());
		assertEquals(0, control.statistics().modeledTrees(), "Mesh construction does not mutate the visible scene");
		camera.setLocation(new Vector3f(5000f, -300f, 5000f));
		assertTrue(control.awaitPreparation(5000));
		forest.updateLogicalState(1f / 60f);
		assertEquals(0, control.statistics().modeledTrees(),
				"An obsolete camera job remains only in the bounded cache");
		assertRepresentationAndLimits(forest, control);
		camera.setLocation(new Vector3f(200f, 21f, -25f));
		camera.lookAt(new Vector3f(200f, 21f, 120f), Vector3f.UNIT_Y);
		forest.updateLogicalState(1f / 60f);
		assertEquals(1, control.statistics().pendingJobs());
		control.close();
		assertTrue(control.awaitShutdown(5000), "World replacement terminates its worker and cancels queued work");
		forest.updateLogicalState(1f / 60f);
		assertTrue(control.isClosed());
		assertEquals(0, control.statistics().pendingJobs());
		assertEquals(0, control.statistics().residentCells());
		assertEquals(0, control.statistics().modeledTrees());
		assertRepresentationAndLimits(forest, control);
	}

	@Test
	void distantCardsUseRegionalDrawsAndIgnoreTheGamesLongFarPlane() throws Exception {
		var assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		Camera camera = camera();
		camera.setFrustumFar(51_000f);
		Node forest = new Node("forest");
		var trees = placements();
		trees.get(0).add(new TreeScatter.TreeInstance(10_000f, 20f, 30f, 8f, 12f, 0f, 1f, 1f, 1f));
		IslandForestControl control = new IslandForestControl(forest, trees, assets, camera, 42);
		forest.addControl(control);
		assertEquals(121, control.statistics().cells());
		assertEquals(5, control.statistics().cardRegions(), "512m grouping also handles negative cell coordinates");
		assertEquals(4, control.statistics().visibleCardRegions());
		assertEquals(32, control.statistics().cardDrawObjects(), "120 populated local cells need only 32 card draws");
		assertEquals(Spatial.CullHint.Always, forest.getChild("distant forest region 19:0").getCullHint());
		assertRepresentationAndLimits(forest, control);

		camera.setLocation(new Vector3f(10_000f, 21f, -25f));
		camera.lookAt(new Vector3f(10_000f, 21f, 120f), Vector3f.UNIT_Y);
		forest.updateLogicalState(1f / 60f);
		assertEquals(1, control.statistics().visibleCardRegions());
		assertEquals(2, control.statistics().cardDrawObjects());
		assertRepresentationAndLimits(forest, control);
		control.close();
		assertTrue(control.awaitShutdown(5000));
	}

	private static void assertRepresentationAndLimits(Node forest, IslandForestControl control) {
		var stats = control.statistics();
		assertTrue(stats.nearCells() + stats.middleCells() <= IslandForestControl.MAX_ACTIVE_CELLS);
		assertTrue(stats.modeledTrees() <= IslandForestControl.MAX_ACTIVE_TREES);
		assertTrue(stats.modeledTriangles() <= IslandForestControl.MAX_ACTIVE_TRIANGLES);
		assertTrue(stats.residentCells() <= IslandForestControl.MAX_RESIDENT_CELLS);
		assertTrue(stats.residentTriangles() + stats.pendingTriangles() <= IslandForestControl.MAX_RESIDENT_TRIANGLES,
				"Reserve native mesh capacity for the single in-flight batch too");
		assertTrue(stats.pendingJobs() >= 0 && stats.pendingJobs() <= 1);
		assertTrue(stats.cardDrawObjects() <= stats.visibleCardRegions() * 8);
		int modeledTrees = 0, sideTrees = 0, topTrees = 0, regions = 0;
		var modeledCells = new HashSet<String>();
		for (Spatial spatial : forest.getChildren()) {
			if (!spatial.getName().startsWith("forest cell "))
				continue;
			Node cell = (Node) spatial;
			modeledCells.add(cell.getName().substring("forest cell ".length()));
			assertEquals(1, cell.getQuantity(), "Only active modeled cells have their own render node");
			Node modeled = (Node) cell.getChild(0);
			assertEquals(2, modeled.getQuantity(), "One bark batch and one foliage batch per cell");
			assertEquals(RenderQueue.ShadowMode.CastAndReceive, modeled.getShadowMode());
			modeledTrees += (int) modeled.getUserData("treeCount");
		}
		for (Spatial spatial : forest.getChildren()) {
			if (!spatial.getName().startsWith("distant forest region "))
				continue;
			Node region = (Node) spatial;
			regions++;
			assertTrue(region.getQuantity() <= 8, "A region submits at most two draws per tree type");
			assertEquals(RenderQueue.ShadowMode.Off, region.getShadowMode());
			for (Spatial card : region.getChildren()) {
				Geometry geometry = (Geometry) card;
				assertEquals(RenderQueue.ShadowMode.Off, geometry.getShadowMode());
				var indexBuffer = geometry.getMesh().getBuffer(VertexBuffer.Type.Index);
				assertEquals(VertexBuffer.Usage.Dynamic, indexBuffer.getUsage());
				IntBuffer indices = (IntBuffer) indexBuffer.getData();
				boolean top = card.getName().startsWith("distant crowns");
				int verticesPerTree = top ? 9 : 8, indicesPerTree = top ? 24 : 12;
				FloatBuffer positions = geometry.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
				for (int tree = 0; tree < geometry.getMesh().getVertexCount() / verticesPerTree; tree++) {
					int vertex = tree * verticesPerTree * 3;
					float x = top ? positions.get(vertex) : (positions.get(vertex) + positions.get(vertex + 3)) * .5f;
					float z = top ? positions.get(vertex + 2)
							: (positions.get(vertex + 2) + positions.get(vertex + 5)) * .5f;
					String cell = (int) Math.floor(x / IslandForestControl.CELL_SIZE) + ":"
							+ (int) Math.floor(z / IslandForestControl.CELL_SIZE);
					boolean hidden = modeledCells.contains(cell);
					for (int i = tree * indicesPerTree; i < (tree + 1) * indicesPerTree; i += 3) {
						int a = indices.get(i), b = indices.get(i + 1), c = indices.get(i + 2);
						assertEquals(hidden, a == b && b == c,
								"Only the exact modeled cell's card triangles may be degenerate");
					}
					if (!hidden) {
						if (top)
							topTrees++;
						else
							sideTrees++;
					}
				}
			}
		}
		assertEquals(stats.cardRegions(), regions);
		assertEquals(stats.modeledTrees(), modeledTrees);
		assertEquals(sideTrees, topTrees, "Side and crown representations change atomically");
		assertEquals(stats.totalTrees(), modeledTrees + sideTrees,
				"Every tree has exactly one representation at every transition, including culled regions");
	}

	private static Camera camera() {
		Camera camera = new Camera(1600, 900);
		camera.setFrustumPerspective(54f, 16f / 9f, .15f, 1000f);
		camera.setLocation(new Vector3f(0f, 21f, -25f));
		camera.lookAt(new Vector3f(0f, 21f, 120f), Vector3f.UNIT_Y);
		return camera;
	}

	private static List<List<TreeScatter.TreeInstance>> placements() {
		var trees = new ArrayList<List<TreeScatter.TreeInstance>>();
		for (TreeScatter.Type ignored : TreeScatter.Type.values())
			trees.add(new ArrayList<>());
		Random random = new Random(42);
		for (int z = 0; z < 10; z++) {
			for (int x = -6; x < 6; x++) {
				for (int tree = 0; tree < 96; tree++) {
					TreeScatter.Type type = TreeScatter.Type.values()[tree % 4];
					trees.get(type.ordinal())
							.add(TreeScatter.instance(type, random,
									(x + random.nextFloat()) * IslandForestControl.CELL_SIZE, 20f,
									(z + random.nextFloat()) * IslandForestControl.CELL_SIZE, 1f));
				}
			}
		}
		return trees;
	}
}
