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
import com.jme3.math.Quaternion;
import com.jme3.material.RenderState;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.Pose;
import se.hirt.searobots.api.Velocity;
import se.hirt.searobots.api.Waypoint;
import se.hirt.searobots.engine.SubmarineSnapshot;

import java.awt.Color;
import java.lang.reflect.Field;
import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class SubmarineScene3DOverlayTest {

	@Test
	void markersKeepWorldSizeNearbyAndMinimumPixelRadiusFarAway() {
		Camera camera = camera();
		assertEquals(1f, SubmarineScene3D.overlayMarkerScale(camera, new Vector3f(0, 0, 100), 18, 9));
		for (float depth : new float[] {2500, 5000, 9000}) {
			Vector3f position = new Vector3f(0, 0, depth);
			assertProjectedRadius(camera, position, 18, 9);
			assertProjectedRadius(camera, position, 10, 6);
			assertProjectedRadius(camera, position, 25, 12);
			assertProjectedRadius(camera, position, 16, 9);
		}
	}

	@Test
	void markerScaleUsesCameraDepthForOffAxisAndRotatedViews() {
		Camera camera = camera();
		Vector3f center = new Vector3f(0, 0, 5000);
		Vector3f offAxis = new Vector3f(1800, 1000, 5000);
		assertEquals(SubmarineScene3D.overlayMarkerScale(camera, center, 18, 9),
				SubmarineScene3D.overlayMarkerScale(camera, offAxis, 18, 9), 0.0001);
		assertProjectedRadius(camera, offAxis, 18, 9);

		camera.setLocation(new Vector3f(500, 700, -300));
		camera.lookAtDirection(new Vector3f(1, -2, 3).normalizeLocal(), Vector3f.UNIT_Y);
		Vector3f position = camera.getLocation().add(camera.getDirection().mult(5000))
				.addLocal(camera.getLeft().mult(1200)).addLocal(camera.getUp().mult(700));
		assertProjectedRadius(camera, position, 18, 9);
	}

	@Test
	void minimumRadiusSurvivesResizeAndPartialViewport() {
		Camera camera = camera();
		Vector3f position = new Vector3f(100, 200, 9000);
		assertProjectedRadius(camera, position, 18, 9);
		camera.resize(1600, 1200, true);
		assertProjectedRadius(camera, position, 18, 9);
		camera.setViewPort(0.25f, 0.75f, 0.2f, 0.8f);
		assertProjectedRadius(camera, position, 18, 9);
	}

	@Test
	void parallelProjectionIgnoresDepthAndSupportsResizeAndPartialViewport() {
		Camera camera = camera();
		camera.setParallelProjection(true);
		camera.setFrustum(1, 10000, -6000, 6000, 4500, -4500);
		Vector3f position = new Vector3f(100, 200, 100);
		float nearbyScale = SubmarineScene3D.overlayMarkerScale(camera, position, 18, 9);
		assertProjectedRadius(camera, position, 18, 9);
		position.z = 9000;
		assertEquals(nearbyScale, SubmarineScene3D.overlayMarkerScale(camera, position, 18, 9));
		assertProjectedRadius(camera, position, 18, 9);
		camera.resize(1600, 1200, true);
		camera.setViewPort(0, 1, 0.1f, 0.9f);
		assertProjectedRadius(camera, position, 18, 9);
	}

	@Test
	void cachedPlanFollowsMovingSubAndRescalesDuringPausedCameraChanges() throws Exception {
		Camera camera = camera();
		SubmarineScene3D scene = scene(camera);
		List<Waypoint> waypoints = List.of(new Waypoint(0, -500, -100, false), new Waypoint(200, -5000, -100, true),
				new Waypoint(300, -6000, -100, false));
		update(scene, snapshot(100, -1000, -100, waypoints), 0);
		Node planNode = (Node) scene.getRootNode().getChild("waypoints-7");
		assertEquals(4, planNode.getQuantity(), "Only active and future markers, the plan and direction arrow remain");
		Geometry activeMarker = (Geometry) planNode.getChild("wp");
		assertEquals(new Vector3f(200, -100, 5000), activeMarker.getLocalTranslation());
		TacticalPath plan = (TacticalPath) planNode.getChild("wpPlan");
		WaypointDirectionIndicator direction = (WaypointDirectionIndicator) planNode.getChild("wpDirection");
		assertEquals(6, plan.getMesh().getVertexCount());
		assertNull(planNode.getChild("wpActiveLeg"), "There is no full line from the sub to its first waypoint");
		assertNull(direction.getChild("oval"), "The oval is only an invisible placement boundary");
		assertEquals(1, direction.getQuantity());
		assertEquals(new Vector3f(100, -100, 1000), direction.getLocalTranslation());
		assertEquals(new Vector3f(200, -100, 5000), direction.getUserData("target"));

		Node displayedVehicle = new Node("displayed-sub");
		displayedVehicle.setLocalTranslation(125, -115, 1080);
		displayedVehicle.setLocalRotation(new Quaternion().fromAngles(-1.7f, 2.8f, 0.2f));
		@SuppressWarnings("unchecked")
		Map<Integer, Node> vehicles = (Map<Integer, Node>) field(scene.getClass(), "subNodes").get(scene);
		vehicles.put(7, displayedVehicle);

		update(scene, snapshot(130, -1100, -120, waypoints), 1);
		assertSame(plan, planNode.getChild("wpPlan"));
		assertSame(direction, planNode.getChild("wpDirection"));
		assertSame(activeMarker, planNode.getChild("wp"));
		assertEquals(displayedVehicle.getLocalTranslation(), direction.getLocalTranslation());
		assertEquals(displayedVehicle.getLocalRotation(), direction.getLocalRotation());
		float previousScale = activeMarker.getLocalScale().x;
		camera.resize(800, 300, true);
		update(scene, snapshot(130, -1100, -120, waypoints), 1);
		assertEquals(previousScale * 2, activeMarker.getLocalScale().x, 0.0001,
				"Paused simulation still responds to camera and viewport changes");
	}

	@Test
	void hiddenWaypointChangesRefreshOnEnableAndEmptyPlanRemovesOldGeometry() throws Exception {
		SubmarineScene3D scene = scene(camera());
		List<Waypoint> activePlan = List.of(new Waypoint(200, -5000, -100, true));
		update(scene, snapshot(100, -1000, -100, activePlan), 0);
		Node node = (Node) scene.getRootNode().getChild("waypoints-7");
		assertNotNull(node.getChild("wpDirection"));
		OverlayConfig config = (OverlayConfig) field(SubmarineScene3D.class, "overlayConfig").get(scene);
		config.waypoints = false;
		List<Waypoint> inactivePlan = List.of(new Waypoint(300, -5000, -100, false),
				new Waypoint(400, -6000, -100, false));
		update(scene, snapshot(100, -1000, -100, inactivePlan), 1);
		assertEquals(Spatial.CullHint.Always, node.getCullHint());
		config.waypoints = true;
		update(scene, snapshot(100, -1000, -100, inactivePlan), 1);
		assertEquals(Spatial.CullHint.Never, node.getCullHint());
		assertNull(node.getChild("wpDirection"), "An arrow needs an active target");
		assertNotNull(node.getChild("wpPlan"));
		assertEquals(3, node.getQuantity());
		update(scene, snapshot(100, -1000, -100, List.of()), 1);
		assertEquals(0, node.getQuantity());
	}

	@Test
	void directionArrowOnlyAppearsWithWaypointsAndRefreshesItsActiveTargetAfterBeingHidden() throws Exception {
		SubmarineScene3D scene = scene(camera());
		OverlayConfig config = (OverlayConfig) field(scene.getClass(), "overlayConfig").get(scene);
		List<Waypoint> ahead = List.of(new Waypoint(100, 2000, -100, true), new Waypoint(100, -4000, -100, false));
		config.waypoints = false;
		update(scene, snapshot(100, -1000, -100, ahead), 0);
		Node node = (Node) scene.getRootNode().getChild("waypoints-7");
		assertEquals(0, node.getQuantity());
		config.waypoints = true;
		update(scene, snapshot(100, -1000, -100, ahead), 0);
		WaypointDirectionIndicator direction = (WaypointDirectionIndicator) node.getChild("wpDirection");
		assertNotNull(direction);
		assertEquals(new Vector3f(100, -100, -2000), direction.getUserData("target"));
		config.waypoints = false;
		List<Waypoint> astern = List.of(new Waypoint(100, 2000, -100, false), new Waypoint(100, -4000, -100, true));
		update(scene, snapshot(100, -1000, -100, astern), 1);
		assertEquals(Spatial.CullHint.Always, node.getCullHint());
		config.waypoints = true;
		update(scene, snapshot(100, -1000, -100, astern), 1);
		assertEquals(Spatial.CullHint.Never, node.getCullHint());
		direction = (WaypointDirectionIndicator) node.getChild("wpDirection");
		assertEquals(new Vector3f(100, -100, 4000), direction.getUserData("target"));
		assertNull(node.getChild("wpActiveLeg"));
	}

	@Test
	void reenablingHistoryWhilePausedShowsSamplesCollectedWhileHidden() throws Exception {
		SubmarineScene3D scene = scene(camera());
		OverlayConfig config = (OverlayConfig) field(SubmarineScene3D.class, "overlayConfig").get(scene);
		config.route = true;
		update(scene, snapshot(100, -1000, -100, List.of()), 0);
		update(scene, snapshot(200, -2000, -100, List.of()), 100);
		Node trailNode = (Node) scene.getRootNode().getChild("trail-7");
		Node routeNode = (Node) scene.getRootNode().getChild("route-7");
		TacticalPath trail = (TacticalPath) trailNode.getChild("trailLine");
		TacticalPath route = (TacticalPath) routeNode.getChild("routeLine");
		assertEquals(6, trail.getMesh().getVertexCount());
		assertEquals(6, route.getMesh().getVertexCount());
		config.trails = false;
		config.route = false;
		update(scene, snapshot(300, -3000, -100, List.of()), 200);
		update(scene, snapshot(400, -4000, -100, List.of()), 300);
		config.trails = true;
		config.route = true;
		update(scene, snapshot(400, -4000, -100, List.of()), 300);
		assertSame(trail, trailNode.getChild("trailLine"));
		assertSame(route, routeNode.getChild("routeLine"));
		assertEquals(18, trail.getMesh().getVertexCount());
		assertEquals(18, route.getMesh().getVertexCount());
		assertEndpoint(trail, 14, 17, new Vector3f(400, -100, 4000));
		assertEndpoint(route, 14, 17, new Vector3f(400, -100, 4000));
	}

	@Test
	void normalViewsKeepWorldHistoryAndRestoreItAfterAPausedTacticalShot() throws Exception {
		Camera camera = camera();
		SubmarineScene3D scene = scene(camera);
		Field tacticalMode = field(scene.getClass(), "screenshotTacticalMode");
		tacticalMode.setBoolean(scene, false);
		OverlayConfig config = (OverlayConfig) field(scene.getClass(), "overlayConfig").get(scene);
		config.route = true;
		List<Waypoint> waypoints = List.of(new Waypoint(200, -5000, -100, true));
		update(scene, snapshot(100, -1000, -100, waypoints), 0);
		SubmarineSnapshot latest = snapshot(200, -2000, -100, waypoints);
		update(scene, latest, 100);
		TacticalPath trail = (TacticalPath) ((Node) scene.getRootNode().getChild("trail-7")).getChild("trailLine");
		TacticalPath route = (TacticalPath) ((Node) scene.getRootNode().getChild("route-7")).getChild("routeLine");
		Geometry marker = (Geometry) ((Node) scene.getRootNode().getChild("waypoints-7")).getChild("wp");
		assertEquals(Mesh.Mode.LineStrip, trail.getMesh().getMode());
		assertEquals(RenderState.BlendMode.Additive, trail.getMaterial().getAdditionalRenderState().getBlendMode());
		assertEquals(RenderQueue.Bucket.Translucent, trail.getQueueBucket());
		assertTrue(route.getMaterial().getAdditionalRenderState().isDepthTest());
		assertTrue(route.getMaterial().getAdditionalRenderState().isDepthWrite());
		assertEquals(RenderQueue.Bucket.Opaque, route.getQueueBucket());
		assertEquals(1f, marker.getLocalScale().x);
		assertTrue(marker.getMaterial().getAdditionalRenderState().isDepthTest());
		float[] worldRoute = positions(route);
		camera.lookAtDirection(Vector3f.UNIT_Z.negate(), Vector3f.UNIT_Y);
		update(scene, latest, 100);
		assertArrayEquals(worldRoute, positions(route), "Normal geometry remains available to other render cameras");

		camera.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
		tacticalMode.setBoolean(scene, true);
		update(scene, latest, 100);
		assertEquals(Mesh.Mode.Triangles, trail.getMesh().getMode());
		assertEquals(RenderQueue.Bucket.Transparent, trail.getQueueBucket());
		assertFalse(route.getMaterial().getAdditionalRenderState().isDepthTest());
		assertFalse(route.getMaterial().getAdditionalRenderState().isDepthWrite());
		assertEquals(RenderQueue.Bucket.Transparent, route.getQueueBucket());
		assertTrue(marker.getLocalScale().x > 1f);
		tacticalMode.setBoolean(scene, false);
		update(scene, latest, 100);
		assertEquals(Mesh.Mode.LineStrip, trail.getMesh().getMode());
		assertArrayEquals(worldRoute, positions(route));
		assertEquals(RenderQueue.Bucket.Translucent, trail.getQueueBucket());
		assertEquals(1f, marker.getLocalScale().x);
		assertTrue(route.getMaterial().getAdditionalRenderState().isDepthWrite());
	}

	@Test
	void tacticalSizingRequiresTheTacticalDirectorShotWithWaterHidden() throws Exception {
		SubmarineScene3D scene = scene(camera());
		field(scene.getClass(), "screenshotTacticalMode").setBoolean(scene, false);
		CinematicDirector director = new CinematicDirector(List::of, List::of, new HashMap<>(), new HashMap<>(), null);
		field(scene.getClass(), "cinematicDirector").set(scene, director);
		Field shot = field(director.getClass(), "currentShot");
		shot.set(director, new CinematicDirector.Shot(CinematicDirector.ShotType.CONTEXT_WIDE, 7, -1, 10, 5,
				CinematicDirector.TransitionType.HARD_CUT));
		SubmarineSnapshot latest = snapshot(100, -1000, -100, List.of(new Waypoint(200, -5000, -100, true)));
		update(scene, latest, 0);
		Geometry marker = (Geometry) ((Node) scene.getRootNode().getChild("waypoints-7")).getChild("wp");
		assertEquals(1f, marker.getLocalScale().x, "Water reflections still need ordinary world geometry");
		field(director.getClass(), "waterOpacity").setFloat(director, 0.4f);
		update(scene, latest, 0);
		assertTrue(marker.getLocalScale().x > 1f);
		shot.set(director, new CinematicDirector.Shot(CinematicDirector.ShotType.IDLE_HERO, 7, -1, 10, 5,
				CinematicDirector.TransitionType.HARD_CUT));
		update(scene, latest, 0);
		assertEquals(1f, marker.getLocalScale().x);
		shot.set(director, new CinematicDirector.Shot(CinematicDirector.ShotType.CONTEXT_WIDE, 7, -1, 10, 5,
				CinematicDirector.TransitionType.HARD_CUT));
		Field mode = field(scene.getClass(), "cameraMode");
		for (Object value : mode.getType().getEnumConstants()) {
			if (value.toString().equals("ORBIT"))
				mode.set(scene, value);
		}
		update(scene, latest, 0);
		assertEquals(1f, marker.getLocalScale().x, "Manual cameras must not inherit the director's last shot");
	}

	private static float[] positions(TacticalPath path) {
		FloatBuffer buffer = path.getMesh().getFloatBuffer(VertexBuffer.Type.Position).duplicate();
		float[] values = new float[buffer.remaining()];
		buffer.get(values);
		return values;
	}

	private static Camera camera() {
		Camera camera = new Camera(800, 600);
		camera.setFrustumPerspective(45, 800f / 600, 1, 10000);
		camera.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
		return camera;
	}

	private static void assertProjectedRadius(Camera camera, Vector3f position, float radius, float pixels) {
		float scale = SubmarineScene3D.overlayMarkerScale(camera, position, radius, pixels);
		Vector3f center = camera.getScreenCoordinates(position);
		Vector3f edge = camera.getScreenCoordinates(position.add(camera.getUp().mult(radius * scale)));
		assertEquals(pixels, Math.hypot(edge.x - center.x, edge.y - center.y), 0.002);
	}

	private static SubmarineScene3D scene(Camera camera) throws Exception {
		SubmarineScene3D scene = new SubmarineScene3D();
		field(scene.getClass(), "cam").set(scene, camera);
		field(scene.getClass(), "assetManager").set(scene, new DesktopAssetManager(true));
		field(scene.getClass(), "screenshotTacticalMode").setBoolean(scene, true);
		return scene;
	}

	private static Field field(Class<?> type, String name) throws NoSuchFieldException {
		for (Class<?> owner = type; owner != null; owner = owner.getSuperclass()) {
			try {
				Field field = owner.getDeclaredField(name);
				field.setAccessible(true);
				return field;
			} catch (NoSuchFieldException ignored) {
			}
		}
		throw new NoSuchFieldException(name);
	}

	private static void update(SubmarineScene3D scene, SubmarineSnapshot snapshot, long tick) throws Exception {
		field(scene.getClass(), "latestSnapshots").set(scene, List.of(snapshot));
		field(scene.getClass(), "latestTick").setLong(scene, tick);
		var update = SubmarineScene3D.class.getDeclaredMethod("updateOverlays");
		update.setAccessible(true);
		update.invoke(scene);
	}

	private static SubmarineSnapshot snapshot(double x, double y, double z, List<Waypoint> waypoints) {
		return new SubmarineSnapshot(7, "test", "TE", Pose.at(x, y, z, 0), Velocity.ZERO, 0, Color.CYAN, false, 100, 0,
				0, 0, 0, 0, "", false, 4, List.of(), waypoints, List.of(), null, false);
	}

	private static void assertEndpoint(TacticalPath path, int first, int second, Vector3f expected) {
		var positions = path.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
		for (int axis = 0; axis < 3; axis++) {
			float midpoint = (positions.get(first * 3 + axis) + positions.get(second * 3 + axis)) / 2;
			assertEquals(expected.get(axis), midpoint, 0.001);
		}
	}
}
