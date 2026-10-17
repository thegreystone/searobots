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

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;

import java.nio.FloatBuffer;
import java.util.List;

/**
 * A path with readable screen-width ribbons in tactical views and camera-independent geometry in
 * ordinary 3D views. Attach directly under the untransformed scene root and update after moving the
 * camera.
 */
final class TacticalPath extends Geometry {

	enum WorldStyle {
		LINE, TUBE, RIBBON
	}

	private final ColorRGBA color;
	private final float widthPixels, startAlpha, endAlpha;
	private final WorldStyle worldStyle;
	private Boolean tacticalMode;
	private boolean pointsChanged = true;
	private float[] points = new float[0];
	private int pointCount;
	private FloatBuffer positions, colors;
	private final Vector3f start = new Vector3f(), end = new Vector3f();
	private final Vector3f screenStart = new Vector3f(), screenEnd = new Vector3f();
	private final Vector3f startOffset = new Vector3f(), endOffset = new Vector3f();
	private final Vector3f direction = new Vector3f(), side = new Vector3f(), up = new Vector3f();

	TacticalPath(AssetManager assets, String name, List<Vector3f> points, ColorRGBA color, float widthPixels,
			float startAlpha, float endAlpha) {
		this(assets, name, points, color, widthPixels, startAlpha, endAlpha, WorldStyle.RIBBON);
	}

	TacticalPath(AssetManager assets, String name, List<Vector3f> points, ColorRGBA color, float widthPixels,
			float startAlpha, float endAlpha, WorldStyle worldStyle) {
		super(name, new Mesh());
		this.color = color.clone();
		this.widthPixels = widthPixels;
		this.startAlpha = startAlpha;
		this.endAlpha = endAlpha;
		this.worldStyle = worldStyle;
		Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setBoolean("VertexColor", true);
		var state = material.getAdditionalRenderState();
		state.setBlendMode(RenderState.BlendMode.Alpha);
		state.setDepthWrite(false);
		state.setDepthTest(false);
		state.setFaceCullMode(RenderState.FaceCullMode.Off);
		setMaterial(material);
		setQueueBucket(RenderQueue.Bucket.Transparent);
		setCullHint(CullHint.Never);
		setPoints(points);
	}

	/** Copies the supplied coordinates; buffers grow only when their capacity is exceeded. */
	void setPoints(List<Vector3f> source) {
		ensureCapacity(source.size());
		pointCount = source.size();
		for (int i = 0; i < pointCount; i++) {
			Vector3f point = source.get(i);
			points[i * 3] = point.x;
			points[i * 3 + 1] = point.y;
			points[i * 3 + 2] = point.z;
		}
		pointsChanged = true;
	}

	/** Updates a moving two-point connector without allocating a list or vectors. */
	void setEndpoints(Vector3f from, Vector3f to) {
		ensureCapacity(2);
		pointCount = 2;
		points[0] = from.x;
		points[1] = from.y;
		points[2] = from.z;
		points[3] = to.x;
		points[4] = to.y;
		points[5] = to.z;
		pointsChanged = true;
	}

	private void ensureCapacity(int count) {
		if (points.length < count * 3)
			points = new float[count * 3];
		int vertexCount = Math.max(1, count - 1) * (worldStyle == WorldStyle.TUBE ? 24 : 6);
		if (positions == null || positions.capacity() < vertexCount * 3) {
			positions = BufferUtils.createFloatBuffer(vertexCount * 3);
			colors = BufferUtils.createFloatBuffer(vertexCount * 4);
			positions.limit(0);
			colors.limit(0);
			getMesh().setBuffer(VertexBuffer.Type.Position, 3, positions);
			getMesh().setBuffer(VertexBuffer.Type.Color, 4, colors);
			getMesh().setDynamic();
		}
	}

	/** Reuses the mesh and buffers, including while the simulation is paused. */
	void update(Camera camera) {
		update(camera, true);
	}

	void update(Camera camera, boolean tactical) {
		boolean modeChanged = tacticalMode == null || tacticalMode != tactical;
		if (modeChanged) {
			tacticalMode = tactical;
			boolean line = !tactical && worldStyle == WorldStyle.LINE;
			getMesh().setMode(line ? Mesh.Mode.LineStrip : Mesh.Mode.Triangles);
			var state = getMaterial().getAdditionalRenderState();
			state.setBlendMode(line ? RenderState.BlendMode.Additive
					: tactical ? RenderState.BlendMode.Alpha : RenderState.BlendMode.Off);
			state.setDepthTest(!tactical && !line);
			state.setDepthWrite(!tactical && !line);
			// Trails have no scene depth. Draw them after water and fog so those filters
			// cannot obscure the line using the sky or seabed depth behind it.
			setQueueBucket(line ? RenderQueue.Bucket.Translucent
					: tactical ? RenderQueue.Bucket.Transparent : RenderQueue.Bucket.Opaque);
		}
		if (!tactical && !modeChanged && !pointsChanged)
			return;
		positions.clear();
		colors.clear();
		if (tactical)
			updateTactical(camera);
		else
			updateWorld();
		positions.flip();
		colors.flip();
		getMesh().getBuffer(VertexBuffer.Type.Position).updateData(positions);
		getMesh().getBuffer(VertexBuffer.Type.Color).updateData(colors);
		getMesh().updateCounts();
		if (positions.hasRemaining())
			updateModelBound();
		pointsChanged = false;
	}

	private void updateWorld() {
		if (worldStyle == WorldStyle.LINE) {
			for (int i = 0; i < pointCount; i++) {
				positions.put(points[i * 3]).put(points[i * 3 + 1]).put(points[i * 3 + 2]);
				colors.put(color.r * 2).put(color.g * 2).put(color.b * 2).put(color.a * 0.6f * i / pointCount);
			}
			return;
		}
		for (int i = 0; i < pointCount - 1; i++) {
			start.set(points[i * 3], points[i * 3 + 1], points[i * 3 + 2]);
			end.set(points[i * 3 + 3], points[i * 3 + 4], points[i * 3 + 5]);
			end.subtract(start, direction);
			if (direction.lengthSquared() < 0.000001f)
				continue;
			direction.normalizeLocal();
			side.set(direction.z, 0, -direction.x);
			if (side.lengthSquared() < 0.000001f)
				direction.cross(Vector3f.UNIT_Z, side);
			side.normalizeLocal();
			if (worldStyle == WorldStyle.RIBBON) {
				startOffset.set(side).multLocal(2);
				endOffset.set(side).multLocal(-2);
				worldQuad();
			} else {
				// A local orthonormal cross-section stays square through climbs and dives.
				side.cross(direction, up).normalizeLocal();
				for (int wall = 0; wall < 4; wall++) {
					float sideFrom = wall < 2 ? -2 : 2;
					float upFrom = wall == 0 || wall == 3 ? -2 : 2;
					float sideTo = wall == 1 || wall == 3 ? -sideFrom : sideFrom;
					float upTo = wall == 0 || wall == 2 ? -upFrom : upFrom;
					startOffset.set(side).multLocal(sideFrom).addLocal(up.x * upFrom, up.y * upFrom, up.z * upFrom);
					endOffset.set(side).multLocal(sideTo).addLocal(up.x * upTo, up.y * upTo, up.z * upTo);
					worldQuad();
				}
			}
		}
	}

	private void worldQuad() {
		worldVertex(start, startOffset);
		worldVertex(start, endOffset);
		worldVertex(end, startOffset);
		worldVertex(end, startOffset);
		worldVertex(start, endOffset);
		worldVertex(end, endOffset);
	}

	private void worldVertex(Vector3f point, Vector3f offset) {
		positions.put(point.x + offset.x).put(point.y + offset.y).put(point.z + offset.z);
		float brightness = worldStyle == WorldStyle.RIBBON ? 0.5f : 1;
		colors.put(color.r * brightness).put(color.g * brightness).put(color.b * brightness).put(1);
	}

	private void updateTactical(Camera camera) {
		float viewportWidth = camera.getWidth() * (camera.getViewPortRight() - camera.getViewPortLeft());
		float viewportHeight = camera.getHeight() * (camera.getViewPortTop() - camera.getViewPortBottom());
		// Stay just inside the near plane to avoid round-off clipping of the expanded corners.
		float near = camera.getFrustumNear() * 1.001f;
		if (viewportWidth > 0 && viewportHeight > 0 && widthPixels > 0) {
			for (int i = 0; i < pointCount - 1; i++) {
				start.set(points[i * 3], points[i * 3 + 1], points[i * 3 + 2]);
				end.set(points[i * 3 + 3], points[i * 3 + 4], points[i * 3 + 5]);
				float startDepth = depth(camera, start), endDepth = depth(camera, end);
				if (startDepth < near && endDepth < near)
					continue;
				float from = i, to = i + 1;
				if (startDepth < near) {
					float t = (near - startDepth) / (endDepth - startDepth);
					start.interpolateLocal(end, t);
					startDepth = near;
					from += t;
				} else if (endDepth < near) {
					float t = (near - endDepth) / (startDepth - endDepth);
					end.interpolateLocal(start, t);
					endDepth = near;
					to -= t;
				}
				camera.getScreenCoordinates(start, screenStart);
				camera.getScreenCoordinates(end, screenEnd);
				float dx = screenEnd.x - screenStart.x, dy = screenEnd.y - screenStart.y;
				float length = (float) Math.sqrt(dx * dx + dy * dy);
				if (!Float.isFinite(length) || length < 0.001f)
					continue;
				float normalX = -dy / length * widthPixels * 0.5f;
				float normalY = dx / length * widthPixels * 0.5f;
				offset(camera, normalX, normalY, startDepth, viewportWidth, viewportHeight, startOffset);
				offset(camera, normalX, normalY, endDepth, viewportWidth, viewportHeight, endOffset);
				float alphaFrom = alpha(from), alphaTo = alpha(to);
				vertex(start, startOffset, 1, alphaFrom);
				vertex(start, startOffset, -1, alphaFrom);
				vertex(end, endOffset, 1, alphaTo);
				vertex(end, endOffset, 1, alphaTo);
				vertex(start, startOffset, -1, alphaFrom);
				vertex(end, endOffset, -1, alphaTo);
			}
		}
	}

	private float alpha(float index) {
		return color.a * (startAlpha + (endAlpha - startAlpha) * index / (pointCount - 1));
	}

	private static float depth(Camera camera, Vector3f point) {
		Vector3f origin = camera.getLocation(), direction = camera.getDirection();
		return (point.x - origin.x) * direction.x + (point.y - origin.y) * direction.y
				+ (point.z - origin.z) * direction.z;
	}

	private static void offset(
		Camera camera, float x, float y, float depth, float viewportWidth, float viewportHeight, Vector3f result) {
		float scale = camera.isParallelProjection() ? 1 : depth / camera.getFrustumNear();
		float worldX = x * scale * (camera.getFrustumRight() - camera.getFrustumLeft()) / viewportWidth;
		float worldY = y * scale * (camera.getFrustumTop() - camera.getFrustumBottom()) / viewportHeight;
		Vector3f left = camera.getLeft(), up = camera.getUp();
		result.set(-left.x * worldX + up.x * worldY, -left.y * worldX + up.y * worldY,
				-left.z * worldX + up.z * worldY);
	}

	private void vertex(Vector3f point, Vector3f offset, float side, float alpha) {
		positions.put(point.x + offset.x * side).put(point.y + offset.y * side).put(point.z + offset.z * side);
		colors.put(color.r).put(color.g).put(color.b).put(alpha);
	}
}
