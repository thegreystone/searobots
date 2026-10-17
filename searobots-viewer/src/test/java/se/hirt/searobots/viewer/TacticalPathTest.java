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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
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
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.nio.FloatBuffer;

import static org.junit.jupiter.api.Assertions.*;

class TacticalPathTest {

	private static final DesktopAssetManager ASSETS = new DesktopAssetManager(true);

	@Test
	void retainsPixelWidthAtDifferentDepthsAfterResizeAndCameraRotation() {
		Camera camera = camera();
		TacticalPath path = path(List.of(new Vector3f(-10, 0, 100), new Vector3f(100, 20, 5000)));
		path.update(camera);
		assertWidth(path, camera, 7);
		camera.resize(1600, 1200, true);
		path.update(camera);
		assertWidth(path, camera, 7);
		camera.setLocation(new Vector3f(50, -100, -200));
		camera.lookAt(new Vector3f(100, 20, 5000), Vector3f.UNIT_Y);
		path.update(camera);
		assertWidth(path, camera, 7);
	}

	@Test
	void handlesVerticalPathsParallelProjectionAndPartialViewport() {
		Camera camera = camera();
		camera.setParallelProjection(true);
		camera.setFrustum(1, 10000, -50, 50, 50, -50);
		camera.setViewPort(0.1f, 0.9f, 0.2f, 0.8f);
		TacticalPath path = path(List.of(new Vector3f(0, -10, 100), new Vector3f(0, 10, 5000)));
		path.update(camera);
		assertWidth(path, camera, 7);
	}

	@Test
	void clipsAtNearPlaneAndPreservesTheFadeAtTheClippedEndpoint() {
		Camera camera = camera();
		TacticalPath path = new TacticalPath(ASSETS, "test", List.of(new Vector3f(-5, 0, -10), new Vector3f(5, 2, 10)),
				ColorRGBA.Cyan, 7, 0.2f, 0.9f);
		path.update(camera);
		assertEquals(6, path.getMesh().getVertexCount());
		var positions = path.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
		for (int i = 0; i < positions.limit(); i += 3) {
			assertTrue(Float.isFinite(positions.get(i)));
			assertTrue(positions.get(i + 2) >= camera.getFrustumNear());
		}
		float clippedFraction = (1.001f + 10) / 20;
		assertEquals(0.2f + 0.7f * clippedFraction, path.getMesh().getFloatBuffer(VertexBuffer.Type.Color).get(3),
				0.0001);
		assertEquals(0.9f, path.getMesh().getFloatBuffer(VertexBuffer.Type.Color).get(11), 0.0001);
		assertWidth(path, camera, 7);
	}

	@Test
	void ignoresInvisibleOrDegenerateSegmentsAndReusesBuffersForMovingConnectors() {
		Camera camera = camera();
		TacticalPath path = path(List.of());
		path.update(camera);
		assertEquals(0, path.getMesh().getVertexCount());
		path.setEndpoints(new Vector3f(0, 0, -10), new Vector3f(10, 0, -20));
		path.update(camera);
		assertEquals(0, path.getMesh().getVertexCount());
		path.setEndpoints(new Vector3f(0, 0, 10), new Vector3f(0, 0, 20));
		path.update(camera);
		assertEquals(0, path.getMesh().getVertexCount());
		path.setEndpoints(new Vector3f(0, 0, 10), new Vector3f(0, 0, 10));
		path.update(camera);
		assertEquals(0, path.getMesh().getVertexCount());
		var mesh = path.getMesh();
		var positions = mesh.getFloatBuffer(VertexBuffer.Type.Position);
		var material = path.getMaterial();
		path.setEndpoints(new Vector3f(-1, 0, 10), new Vector3f(1, 0, 20));
		path.update(camera);
		assertSame(mesh, path.getMesh());
		assertSame(positions, mesh.getFloatBuffer(VertexBuffer.Type.Position));
		assertSame(material, path.getMaterial());
		assertWidth(path, camera, 7);
	}

	@Test
	void normalGeometryRetainsHistoryBehindTheCameraAndDoesNotMoveWithIt() {
		for (TacticalPath.WorldStyle style : TacticalPath.WorldStyle.values()) {
			Camera camera = camera();
			TacticalPath path = worldPath(List.of(new Vector3f(-5, 0, -10), new Vector3f(5, 20, -20)), style);
			path.update(camera, false);
			assertEquals(style == TacticalPath.WorldStyle.LINE ? 2 : style == TacticalPath.WorldStyle.TUBE ? 24 : 6,
					path.getMesh().getVertexCount());
			float[] before = positions(path);
			camera.setLocation(new Vector3f(100, -200, 500));
			camera.lookAtDirection(Vector3f.UNIT_X, Vector3f.UNIT_Y);
			path.update(camera, false);
			assertArrayEquals(before, positions(path));
			// A cached ordinary path must not upload new geometry just because the camera moved.
			path.getMesh().getBuffer(VertexBuffer.Type.Position).clearUpdateNeeded();
			path.update(camera, false);
			assertFalse(path.getMesh().getBuffer(VertexBuffer.Type.Position).isUpdateNeeded());
		}
	}

	@Test
	void normalRoutesHaveSolidDepthAndAStableCrossSectionThroughVerticalDives() {
		TacticalPath path = worldPath(List.of(new Vector3f(1, 2, -10), new Vector3f(1, 50, -10)),
				TacticalPath.WorldStyle.TUBE);
		path.update(camera(), false);
		assertEquals(RenderQueue.Bucket.Opaque, path.getQueueBucket());
		var state = path.getMaterial().getAdditionalRenderState();
		assertTrue(state.isDepthTest());
		assertTrue(state.isDepthWrite());
		assertEquals(RenderState.BlendMode.Off, state.getBlendMode());
		float[] vertices = positions(path);
		for (int i = 0; i < vertices.length; i += 3) {
			assertEquals(2, Math.abs(vertices[i] - 1), 0.0001);
			assertTrue(vertices[i + 1] == 2 || vertices[i + 1] == 50);
			assertEquals(2, Math.abs(vertices[i + 2] + 10), 0.0001);
		}
		for (int i = 0; i < vertices.length; i += 9) {
			Vector3f a = new Vector3f(vertices[i], vertices[i + 1], vertices[i + 2]);
			Vector3f b = new Vector3f(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
			Vector3f c = new Vector3f(vertices[i + 6], vertices[i + 7], vertices[i + 8]);
			assertEquals(4 * 48, b.subtract(a).cross(c.subtract(a)).length(), 0.0001);
		}
		FloatBuffer colors = path.getMesh().getFloatBuffer(VertexBuffer.Type.Color);
		assertEquals(ColorRGBA.Cyan.g, colors.get(1));
		assertEquals(1, colors.get(3));
	}

	@Test
	void normalTrailDrawsAfterWaterAndFogWithItsThinAdditiveFade() {
		TacticalPath path = worldPath(
				List.of(new Vector3f(0, 0, -10), new Vector3f(10, 0, 20), new Vector3f(20, 0, 40)),
				TacticalPath.WorldStyle.LINE);
		path.update(camera(), false);
		assertEquals(Mesh.Mode.LineStrip, path.getMesh().getMode());
		assertEquals(RenderQueue.Bucket.Translucent, path.getQueueBucket());
		var state = path.getMaterial().getAdditionalRenderState();
		assertFalse(state.isDepthTest());
		assertFalse(state.isDepthWrite());
		assertEquals(RenderState.BlendMode.Additive, state.getBlendMode());
		FloatBuffer colors = path.getMesh().getFloatBuffer(VertexBuffer.Type.Color);
		assertEquals(2, colors.get(1));
		assertEquals(0, colors.get(3));
		assertEquals(0.4f, colors.get(11), 0.0001);
	}

	@Test
	void restoresWorldGeometryAfterTacticalViewsWhilePausedAndReusesBuffers() {
		for (TacticalPath.WorldStyle style : TacticalPath.WorldStyle.values()) {
			Camera camera = camera();
			TacticalPath path = worldPath(List.of(new Vector3f(-10, 0, 100), new Vector3f(20, 5, 500)), style);
			path.update(camera, false);
			float[] world = positions(path);
			var mesh = path.getMesh();
			var buffer = mesh.getFloatBuffer(VertexBuffer.Type.Position);
			var material = path.getMaterial();
			path.update(camera, true);
			assertWidth(path, camera, 7);
			assertEquals(RenderQueue.Bucket.Transparent, path.getQueueBucket());
			assertFalse(material.getAdditionalRenderState().isDepthTest());
			camera.setLocation(new Vector3f(50, -100, -200));
			camera.lookAt(new Vector3f(100, 20, 5000), Vector3f.UNIT_Y);
			path.update(camera, false);
			assertArrayEquals(world, positions(path));
			assertSame(mesh, path.getMesh());
			assertSame(buffer, mesh.getFloatBuffer(VertexBuffer.Type.Position));
			assertSame(material, path.getMaterial());
			assertEquals(style != TacticalPath.WorldStyle.LINE, material.getAdditionalRenderState().isDepthWrite());
			assertEquals(
					style == TacticalPath.WorldStyle.LINE ? RenderQueue.Bucket.Translucent : RenderQueue.Bucket.Opaque,
					path.getQueueBucket());
		}
	}

	@Test
	void pointUpdatesInvalidateCachedWorldGeometry() {
		TacticalPath path = worldPath(List.of(new Vector3f(0, 0, 10), new Vector3f(10, 0, 20)),
				TacticalPath.WorldStyle.RIBBON);
		path.update(camera(), false);
		path.setEndpoints(new Vector3f(0, 10, 10), new Vector3f(10, 10, 20));
		path.update(camera(), false);
		float[] vertices = positions(path);
		for (int i = 1; i < vertices.length; i += 3)
			assertEquals(10, vertices[i]);
		path.setPoints(List.of(new Vector3f(0, 20, 10), new Vector3f(10, 20, 20)));
		path.update(camera(), false);
		vertices = positions(path);
		for (int i = 1; i < vertices.length; i += 3)
			assertEquals(20, vertices[i]);
	}

	private static TacticalPath worldPath(List<Vector3f> points, TacticalPath.WorldStyle style) {
		return new TacticalPath(ASSETS, "test", points, ColorRGBA.Cyan, 7, 0.1f, 0.9f, style);
	}

	private static float[] positions(TacticalPath path) {
		FloatBuffer buffer = path.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
		float[] values = new float[buffer.limit()];
		buffer.duplicate().rewind().get(values);
		return values;
	}

	private static Camera camera() {
		Camera camera = new Camera(800, 600);
		camera.setFrustumPerspective(45, 800f / 600, 1, 10000);
		camera.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
		return camera;
	}

	private static TacticalPath path(List<Vector3f> points) {
		return new TacticalPath(ASSETS, "test", points, ColorRGBA.Cyan, 7, 0.1f, 0.9f);
	}

	private static void assertWidth(TacticalPath path, Camera camera, float width) {
		assertEquals(6, path.getMesh().getVertexCount());
		assertEquals(width, screenDistance(path, camera, 0, 1), 0.002);
		assertEquals(width, screenDistance(path, camera, 2, 5), 0.002);
	}

	private static double screenDistance(TacticalPath path, Camera camera, int a, int b) {
		var positions = path.getMesh().getFloatBuffer(VertexBuffer.Type.Position);
		Vector3f first = camera.getScreenCoordinates(
				new Vector3f(positions.get(a * 3), positions.get(a * 3 + 1), positions.get(a * 3 + 2)));
		Vector3f second = camera.getScreenCoordinates(
				new Vector3f(positions.get(b * 3), positions.get(b * 3 + 1), positions.get(b * 3 + 2)));
		return Math.hypot(first.x - second.x, first.y - second.y);
	}
}
