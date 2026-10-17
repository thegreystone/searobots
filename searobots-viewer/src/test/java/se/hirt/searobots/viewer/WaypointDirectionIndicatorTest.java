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
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.engine.HullGeometry;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class WaypointDirectionIndicatorTest {

	private static final DesktopAssetManager ASSETS = new DesktopAssetManager(true);
	private static final HullGeometry.Envelope HULL = new HullGeometry.Envelope(38, 5.5, 4.5, 0, 0.11);

	@Test
	void startsOutsideTheHullOnTheSideOfTheActiveWaypointWithoutDrawingAnOval() {
		WaypointDirectionIndicator indicator = indicator(HULL);
		assertEquals(1, indicator.getQuantity());
		assertNull(indicator.getChild("oval"));
		for (Vector3f target : new Vector3f[] {new Vector3f(0, -1000, 0.11f), new Vector3f(0, 1000, 0.11f),
				new Vector3f(-1000, 0, 0.11f), new Vector3f(1000, 0, 0.11f)}) {
			indicator.update(Vector3f.ZERO, Quaternion.IDENTITY, target, camera(), false);
			Geometry arrow = arrow(indicator);
			assertEquals(Spatial.CullHint.Never, arrow.getCullHint());
			Vector3f start = arrow.getLocalTranslation();
			assertTrue(start.x * target.x + start.y * target.y > 0,
					"A target behind the sub must place the arrow behind it");
			float radius = arrow.getLocalScale().x;
			assertTrue(
					square(start.x / ((float) HULL.semiBeam() + radius))
							+ square(start.y / ((float) HULL.semiLength() + radius)) > 1,
					"The arrow starts outside the hull with clearance for its head");
			assertEquals(0.11f, start.z, 0.0001);
			assertAimsAt(arrow, target);
			assertEquals(20.9f, arrow.getLocalScale().z, 0.0001);
		}
	}

	@Test
	void followsTheDisplayedVehiclesPositionPitchAndRollWhileAimingAtTheFullThreeDimensionalTarget() {
		WaypointDirectionIndicator indicator = indicator(HULL);
		Vector3f position = new Vector3f(170, -80, 650);
		Quaternion rotation = new Quaternion().fromAngles(-1.8f, 0.6f, 0.4f);
		Vector3f target = new Vector3f(600, -900, 300);
		indicator.update(position, rotation, target, camera(), false);
		assertEquals(position, indicator.getLocalTranslation());
		assertEquals(rotation, indicator.getLocalRotation());
		Geometry arrow = arrow(indicator);
		Vector3f worldStart = rotation.mult(arrow.getLocalTranslation()).addLocal(position);
		Vector3f worldCenter = rotation.mult(new Vector3f(0, 0, 0.11f)).addLocal(position);
		Vector3f worldDirection = rotation.mult(arrow.getLocalRotation().mult(Vector3f.UNIT_Z));
		assertEquals(1, worldDirection.dot(target.subtract(worldCenter).normalizeLocal()), 0.0001);
		assertTrue(arrow.getLocalScale().z < target.distance(worldStart) * 0.1f,
				"The direction cue must remain short instead of connecting to the waypoint");
	}

	@Test
	void handlesVerticalTargetsAndSuppressesArrivedTargetsInsideTheHull() {
		WaypointDirectionIndicator indicator = indicator(HULL);
		for (float height : new float[] {-300, 300}) {
			Vector3f target = new Vector3f(0, 0, 0.11f + height);
			indicator.update(Vector3f.ZERO, Quaternion.IDENTITY, target, camera(), false);
			Geometry arrow = arrow(indicator);
			assertEquals(Spatial.CullHint.Never, arrow.getCullHint());
			assertTrue(arrow.getLocalTranslation().y < 0, "A vertical target consistently uses the bow");
			assertAimsAt(arrow, target);
			assertFinite(arrow);
		}
		for (Vector3f target : new Vector3f[] {new Vector3f(0, 0, 0.11f), new Vector3f(1, 0, 0.11f),
				new Vector3f(0, -20, 0.11f), new Vector3f(Float.NaN, 0, 0)}) {
			indicator.update(Vector3f.ZERO, Quaternion.IDENTITY, target, camera(), false);
			assertEquals(Spatial.CullHint.Always, arrow(indicator).getCullHint());
			assertFinite(arrow(indicator));
		}
		Vector3f nearbyTarget = new Vector3f(10, 0, 0.11f);
		indicator.update(Vector3f.ZERO, Quaternion.IDENTITY, nearbyTarget, camera(), false);
		Geometry arrow = arrow(indicator);
		assertEquals(Spatial.CullHint.Never, arrow.getCullHint());
		assertAimsAt(arrow, nearbyTarget);
		assertEquals(20.9f, arrow.getLocalScale().z, 0.0001, "A direction cue retains its short fixed length");
	}

	@Test
	void nearbyAftTargetKeepsTheArrowPointingAftWhenTacticalSizingMovesItsBaseBeyondTheTarget() {
		WaypointDirectionIndicator indicator = indicator(HULL);
		Vector3f position = new Vector3f(0, 0, 9000);
		indicator.update(position, Quaternion.IDENTITY, position.add(0, 100, 0.11f), camera(), true);
		Geometry arrow = arrow(indicator);
		assertEquals(Spatial.CullHint.Never, arrow.getCullHint());
		assertTrue(arrow.getLocalTranslation().y > 100, "The tactical oval may extend past a nearby waypoint");
		assertEquals(1, arrow.getLocalRotation().mult(Vector3f.UNIT_Z).dot(Vector3f.UNIT_Y), 0.0001,
				"A waypoint aft of the hull must keep the arrow behind the sub and pointing aft");
	}

	@Test
	void normalViewsRemainIndependentOfTheCameraAndRestoreTheirDimensionsAfterTacticalSizing() {
		WaypointDirectionIndicator indicator = indicator(HULL);
		Camera camera = camera();
		Vector3f position = new Vector3f(0, 0, 5000), target = new Vector3f(10000, 0, 5000.11f);
		indicator.update(position, Quaternion.IDENTITY, target, camera, false);
		Geometry arrow = arrow(indicator);
		Vector3f ordinaryStart = arrow.getLocalTranslation().clone(), ordinaryScale = arrow.getLocalScale().clone();
		Quaternion ordinaryRotation = arrow.getLocalRotation().clone();
		var mesh = arrow.getMesh();
		var positions = mesh.getBuffer(VertexBuffer.Type.Position);
		positions.clearUpdateNeeded();
		camera.setLocation(new Vector3f(100, 700, -600));
		camera.lookAtDirection(Vector3f.UNIT_Z.negate(), Vector3f.UNIT_Y);
		indicator.update(position, Quaternion.IDENTITY, target, camera, false);
		assertEquals(ordinaryStart, arrow.getLocalTranslation());
		assertEquals(ordinaryScale, arrow.getLocalScale());
		assertEquals(ordinaryRotation, arrow.getLocalRotation());
		assertTrue(arrow.getMaterial().getAdditionalRenderState().isDepthTest());
		assertFalse(arrow.getMaterial().getAdditionalRenderState().isDepthWrite());
		assertEquals(RenderQueue.Bucket.Translucent, arrow.getQueueBucket());
		camera.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
		indicator.update(position, Quaternion.IDENTITY, target, camera, true);
		assertTrue(arrow.getLocalScale().z > ordinaryScale.z);
		assertFalse(arrow.getMaterial().getAdditionalRenderState().isDepthTest());
		indicator.update(position, Quaternion.IDENTITY, target, camera, false);
		assertEquals(ordinaryStart, arrow.getLocalTranslation());
		assertEquals(ordinaryScale, arrow.getLocalScale());
		assertEquals(ordinaryRotation, arrow.getLocalRotation());
		assertTrue(arrow.getMaterial().getAdditionalRenderState().isDepthTest());
		assertSame(mesh, arrow.getMesh());
		assertSame(positions, arrow.getMesh().getBuffer(VertexBuffer.Type.Position));
		assertFalse(positions.isUpdateNeeded(), "Changing cameras or moving the arrow never rebuilds its mesh");
	}

	@Test
	void tacticalDimensionsUseCameraDepthAndRespondToResizeAndPartialViewports() {
		WaypointDirectionIndicator indicator = indicator(HULL);
		Camera camera = camera();
		Vector3f position = new Vector3f(0, 0, 5000);
		indicator.update(position, Quaternion.IDENTITY, position.add(10000, 0, 0.11f), camera, true);
		Vector3f scale = arrow(indicator).getLocalScale().clone();
		Vector3f start = arrow(indicator).getLocalTranslation().clone();
		Vector3f offAxis = new Vector3f(1800, 1100, 5000);
		indicator.update(offAxis, Quaternion.IDENTITY, offAxis.add(10000, 0, 0.11f), camera, true);
		assertEquals(scale, arrow(indicator).getLocalScale(), "Off-axis distance must not enlarge tactical arrows");
		camera.resize(800, 300, true);
		indicator.update(position, Quaternion.IDENTITY, position.add(10000, 0, 0.11f), camera, true);
		assertEquals(scale.z * 2, arrow(indicator).getLocalScale().z, 0.0001);
		assertEquals(scale.x * 2, arrow(indicator).getLocalScale().x, 0.0001);
		assertEquals(start.x * 2, arrow(indicator).getLocalTranslation().x, 0.0001);
		camera.setViewPort(0, 1, 0.25f, 0.75f);
		indicator.update(position, Quaternion.IDENTITY, position.add(10000, 0, 0.11f), camera, true);
		assertEquals(scale.z * 4, arrow(indicator).getLocalScale().z, 0.0001);
	}

	@Test
	void parallelProjectionKeepsSizeAtDifferentDepthsAndShipOffsetsAreBodyRelative() {
		WaypointDirectionIndicator indicator = indicator(new HullGeometry.Envelope(90, 19, 7, 1, -2.5));
		Camera camera = camera();
		Vector3f target = new Vector3f(0, 1000, -2.5f);
		indicator.update(Vector3f.ZERO, Quaternion.IDENTITY, target, camera, false);
		assertEquals(0, arrow(indicator).getLocalTranslation().x, 0.0001);
		assertEquals(102.5f, arrow(indicator).getLocalTranslation().y, 0.0001);
		assertEquals(-2.5f, arrow(indicator).getLocalTranslation().z, 0.0001);
		assertEquals(1, arrow(indicator).getLocalRotation().mult(Vector3f.UNIT_Z).dot(Vector3f.UNIT_Y), 0.0001);
		camera.setParallelProjection(true);
		camera.setFrustum(1, 10000, -6000, 6000, 4500, -4500);
		Vector3f position = new Vector3f(0, 0, 100);
		indicator.update(position, Quaternion.IDENTITY, position.add(10000, 0, -2.5f), camera, true);
		Vector3f scale = arrow(indicator).getLocalScale().clone();
		position.z = 9000;
		indicator.update(position, Quaternion.IDENTITY, position.add(10000, 0, -2.5f), camera, true);
		assertEquals(scale, arrow(indicator).getLocalScale());
	}

	@Test
	void broadShipRemainsVisibleWhenTacticalHeadWidthExceedsTheOrdinaryHullMargin() {
		WaypointDirectionIndicator indicator = indicator(new HullGeometry.Envelope(90, 19, 7, 1, -2.5));
		Vector3f position = new Vector3f(0, 0, 4000);
		indicator.update(position, Quaternion.IDENTITY, position.add(10000, -1, -2.5f), camera(), true);
		Geometry arrow = arrow(indicator);
		assertEquals(Spatial.CullHint.Never, arrow.getCullHint());
		assertTrue(arrow.getLocalTranslation().x > 19 + arrow.getLocalScale().x);
	}

	@Test
	void solidArrowHasClosedNonDegenerateFacesForViewsFromBelowAndEdgeOn() {
		Geometry arrow = arrow(indicator(HULL));
		assertEquals(RenderState.FaceCullMode.Off, arrow.getMaterial().getAdditionalRenderState().getFaceCullMode());
		var mesh = arrow.getMesh();
		Map<String, Integer> edges = new HashMap<>();
		int[] vertices = new int[3];
		Vector3f a = new Vector3f(), b = new Vector3f(), c = new Vector3f();
		for (int i = 0; i < mesh.getTriangleCount(); i++) {
			mesh.getTriangle(i, a, b, c);
			assertTrue(b.subtract(a).cross(c.subtract(a)).lengthSquared() > 0);
			mesh.getTriangle(i, vertices);
			for (int edge = 0; edge < 3; edge++) {
				int from = vertices[edge], to = vertices[(edge + 1) % 3];
				edges.merge(Math.min(from, to) + ":" + Math.max(from, to), 1, Integer::sum);
			}
		}
		assertTrue(edges.values().stream().allMatch(count -> count == 2), "Each solid face has a neighbouring face");
	}

	private static WaypointDirectionIndicator indicator(HullGeometry.Envelope hull) {
		return new WaypointDirectionIndicator(ASSETS, "direction", hull, ColorRGBA.Cyan);
	}

	private static Geometry arrow(WaypointDirectionIndicator indicator) {
		return (Geometry) indicator.getChild("arrow");
	}

	private static void assertAimsAt(Geometry arrow, Vector3f target) {
		Vector3f desired = target.subtract(new Vector3f(0, 0, 0.11f)).normalizeLocal();
		assertEquals(1, arrow.getLocalRotation().mult(Vector3f.UNIT_Z).dot(desired), 0.0001);
	}

	private static void assertFinite(Geometry arrow) {
		for (Vector3f vector : new Vector3f[] {arrow.getLocalTranslation(), arrow.getLocalScale()}) {
			assertTrue(Float.isFinite(vector.x));
			assertTrue(Float.isFinite(vector.y));
			assertTrue(Float.isFinite(vector.z));
		}
		Quaternion rotation = arrow.getLocalRotation();
		assertTrue(Float.isFinite(rotation.getX()) && Float.isFinite(rotation.getY()) && Float.isFinite(rotation.getZ())
				&& Float.isFinite(rotation.getW()));
	}

	private static float square(float value) {
		return value * value;
	}

	private static Camera camera() {
		Camera camera = new Camera(800, 600);
		camera.setFrustumPerspective(45, 800f / 600, 1, 10000);
		camera.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
		return camera;
	}
}
