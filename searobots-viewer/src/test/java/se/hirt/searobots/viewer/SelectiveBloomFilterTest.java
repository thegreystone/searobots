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
import com.jme3.light.LightList;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.post.FilterPostProcessor;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.system.NullRenderer;
import com.jme3.texture.Image;
import com.jme3.texture.Texture2D;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;

import static org.junit.jupiter.api.Assertions.*;

class SelectiveBloomFilterTest {
	private static final DesktopAssetManager ASSETS = new DesktopAssetManager(true);

	@Test
	void detectsEmittingMaterialsIncludingLiveTeamLightChanges() {
		Material teamLight = new Material(ASSETS, "Common/MatDefs/Misc/Unshaded.j3md");
		teamLight.setColor("Color", ColorRGBA.White);
		assertFalse(SelectiveBloomFilter.emitsLight(teamLight));
		teamLight.setColor("GlowColor", ColorRGBA.Blue.mult(6));
		assertTrue(SelectiveBloomFilter.emitsLight(teamLight));
		teamLight.setColor("GlowColor", ColorRGBA.Black);
		assertFalse(SelectiveBloomFilter.emitsLight(teamLight));
		Material foliage = new Material(ASSETS, "Common/MatDefs/Light/PBRLighting.j3md");
		foliage.setColor("BaseColor", ColorRGBA.Green);
		assertFalse(SelectiveBloomFilter.emitsLight(foliage));
		foliage.setColor("Emissive", ColorRGBA.Red);
		assertTrue(SelectiveBloomFilter.emitsLight(foliage));
		foliage.clearParam("Emissive");
		foliage.setTexture("EmissiveMap", new Texture2D(1, 1, Image.Format.RGBA8));
		assertTrue(SelectiveBloomFilter.emitsLight(foliage));
	}

	@Test
	void vehicleMarkerFollowsTheModelHierarchyAndSurvivesDeepCloning() {
		Node template = new Node("vehicle template");
		Node model = new Node("nested model parts");
		Geometry hull = opaque("hull");
		model.attachChild(hull);
		template.attachChild(model);
		assertFalse(SelectiveBloomFilter.contributesToGlow(hull));
		SelectiveBloomFilter.markOccluder(template);
		assertTrue(SelectiveBloomFilter.contributesToGlow(hull), "A non-emitting hull must still occlude rear lights");

		Node vehicle = (Node) template.deepClone();
		Geometry clonedHull = (Geometry) ((Node) vehicle.getChild("nested model parts")).getChild("hull");
		assertNotSame(hull, clonedHull);
		assertTrue(SelectiveBloomFilter.contributesToGlow(clonedHull));
		Geometry newPart = opaque("replacement part");
		((Node) vehicle.getChild("nested model parts")).attachChild(newPart);
		assertTrue(SelectiveBloomFilter.contributesToGlow(newPart), "New parts inherit their vehicle's occlusion role");

		Node landscape = new Node("unmarked landscape");
		landscape.attachChild(clonedHull);
		assertFalse(SelectiveBloomFilter.contributesToGlow(clonedHull),
				"Reparenting must not retain a stale vehicle role");
		assertTrue(SelectiveBloomFilter.contributesToGlow(hull), "The original template remains independently marked");
		assertFalse(SelectiveBloomFilter.contributesToGlow(opaque("terrain")));
	}

	@Test
	void vehicleOccludersExcludeTransparentAndNonDepthWritingPartsButKeepTheirEmitters() {
		Node vehicle = new Node("vehicle");
		SelectiveBloomFilter.markOccluder(vehicle);
		Geometry hull = opaque("hull");
		vehicle.attachChild(hull);
		assertTrue(SelectiveBloomFilter.contributesToGlow(hull));
		hull.setQueueBucket(RenderQueue.Bucket.Transparent);
		assertFalse(SelectiveBloomFilter.contributesToGlow(hull));
		hull.setQueueBucket(RenderQueue.Bucket.Translucent);
		assertFalse(SelectiveBloomFilter.contributesToGlow(hull));
		hull.setQueueBucket(RenderQueue.Bucket.Inherit);
		vehicle.setQueueBucket(RenderQueue.Bucket.Transparent);
		assertFalse(SelectiveBloomFilter.contributesToGlow(hull), "Inherited transparency must be respected");
		vehicle.setQueueBucket(RenderQueue.Bucket.Opaque);
		hull.getMaterial().getAdditionalRenderState().setDepthWrite(false);
		assertFalse(SelectiveBloomFilter.contributesToGlow(hull));

		Geometry light = emitter("team light");
		vehicle.attachChild(light);
		light.setQueueBucket(RenderQueue.Bucket.Translucent);
		light.getMaterial().getAdditionalRenderState().setDepthWrite(false);
		assertTrue(SelectiveBloomFilter.contributesToGlow(light), "Glow emitters do not need to be opaque occluders");
	}

	@Test
	void realGlowPassAdmitsHullAndLightWithoutRedrawingTheLandscapeOrBypassingPreviousFilter() {
		try (GlowPass pass = new GlowPass()) {
			Geometry hull = opaque("hull"), light = emitter("visible light"), blockedLight = emitter("filtered light"),
					terrain = opaque("terrain");
			Node vehicle = new Node("vehicle");
			vehicle.attachChild(hull);
			SelectiveBloomFilter.markOccluder(vehicle);
			Predicate<Geometry> previous = geometry -> geometry != blockedLight;
			pass.manager.setRenderFilter(previous);
			pass.enqueue(hull, light, blockedLight, terrain);

			pass.processor.postQueue(pass.viewPort.getQueue());

			assertEquals(Set.of(hull, light), Set.copyOf(pass.manager.rendered));
			assertEquals(2, pass.manager.rendered.size());
			assertSame(previous, pass.manager.getRenderFilter());
			assertFalse(pass.viewPort.getQueue().isQueueEmpty(RenderQueue.Bucket.Opaque),
					"The glow pass must retain the main scene's render queue");

			pass.manager.setRenderFilter(null);
			pass.manager.rendered.clear();
			pass.processor.postQueue(pass.viewPort.getQueue());
			assertEquals(Set.of(hull, light, blockedLight), Set.copyOf(pass.manager.rendered));
			assertEquals(3, pass.manager.rendered.size());
			assertNull(pass.manager.getRenderFilter());
		}
	}

	@Test
	void restoresThePreviousRenderFilterWhenGlowRenderingThrows() {
		try (GlowPass pass = new GlowPass()) {
			Predicate<Geometry> previous = geometry -> true;
			pass.manager.setRenderFilter(previous);
			pass.enqueue(emitter("light"));
			IllegalStateException failure = new IllegalStateException("simulated draw failure");
			pass.manager.failure = failure;

			assertSame(failure, assertThrows(IllegalStateException.class,
					() -> pass.processor.postQueue(pass.viewPort.getQueue())));
			assertSame(previous, pass.manager.getRenderFilter());
		}
	}

	private static Geometry opaque(String name) {
		Geometry geometry = new Geometry(name, new Box(0.5f, 0.5f, 0.5f));
		geometry.setMaterial(new Material(ASSETS, "Common/MatDefs/Light/Lighting.j3md"));
		return geometry;
	}

	private static Geometry emitter(String name) {
		Geometry geometry = new Geometry(name, new Box(0.1f, 0.1f, 0.1f));
		Material material = new Material(ASSETS, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setColor("Color", ColorRGBA.Blue);
		material.setColor("GlowColor", ColorRGBA.Blue.mult(6f));
		geometry.setMaterial(material);
		return geometry;
	}

	private static final class GlowPass implements AutoCloseable {
		final RecordingRenderManager manager;
		final ViewPort viewPort;
		final FilterPostProcessor processor;

		GlowPass() {
			NullRenderer renderer = new NullRenderer();
			renderer.initialize();
			manager = new RecordingRenderManager(renderer);
			viewPort = new ViewPort("glow regression", new Camera(32, 32));
			manager.setCamera(viewPort.getCamera(), false);
			processor = new FilterPostProcessor(ASSETS);
			processor.addFilter(new SelectiveBloomFilter());
			processor.initialize(manager, viewPort);
		}

		void enqueue(Geometry ... geometries) {
			for (Geometry geometry : geometries) {
				Spatial scene = geometry;
				while (scene.getParent() != null)
					scene = scene.getParent();
				scene.updateGeometricState();
				viewPort.getQueue().addToQueue(geometry, geometry.getQueueBucket());
			}
		}

		@Override
		public void close() {
			processor.cleanup();
		}
	}

	private static final class RecordingRenderManager extends RenderManager {
		final List<Geometry> rendered = new ArrayList<>();
		RuntimeException failure;

		RecordingRenderManager(NullRenderer renderer) {
			super(renderer);
		}

		@Override
		public void renderGeometry(Geometry geometry, LightList lights) {
			// The real render queue and RenderManager.renderGeometry(Geometry) apply the filter
			// before this final submission point; replace only the GPU work with a recording.
			assertEquals("Glow", getForcedTechnique());
			if (failure != null)
				throw failure;
			rendered.add(geometry);
		}
	}
}
