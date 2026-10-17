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

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import se.hirt.searobots.engine.HullGeometry;

/**
 * A short, solid arrow towards the active waypoint, starting on an invisible oval outside the hull.
 * OBJ coordinates are X across, Y aft and Z up; the supplied transform follows the displayed
 * vehicle, including its interpolated pitch and roll.
 */
final class WaypointDirectionIndicator extends Node {

	private final HullGeometry.Envelope hull;
	private final Geometry arrow;
	private final Quaternion inverseRotation = new Quaternion(), arrowRotation = new Quaternion();
	private final Vector3f target = new Vector3f(), direction = new Vector3f(), up = new Vector3f();
	private Boolean tacticalMode;

	WaypointDirectionIndicator(AssetManager assets, String name, HullGeometry.Envelope hull, ColorRGBA color) {
		super(name);
		this.hull = hull;
		arrow = new Geometry("arrow", arrowMesh());
		Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		ColorRGBA arrowColor = color.clone();
		arrowColor.a *= 0.95f;
		material.setColor("Color", arrowColor);
		var state = material.getAdditionalRenderState();
		state.setBlendMode(RenderState.BlendMode.Alpha);
		state.setDepthWrite(false);
		state.setFaceCullMode(RenderState.FaceCullMode.Off);
		arrow.setMaterial(material);
		// Water and fog must not apply the seabed's depth to this translucent navigation cue.
		arrow.setQueueBucket(RenderQueue.Bucket.Translucent);
		arrow.setShadowMode(RenderQueue.ShadowMode.Off);
		arrow.setCullHint(CullHint.Always);
		attachChild(arrow);
	}

	void update(
		Vector3f vehicleWorldPosition, Quaternion vehicleWorldRotation, Vector3f targetWorld, Camera camera,
		boolean tactical) {
		setLocalTranslation(vehicleWorldPosition);
		setLocalRotation(vehicleWorldRotation);
		if (tacticalMode == null || tacticalMode != tactical) {
			tacticalMode = tactical;
			arrow.getMaterial().getAdditionalRenderState().setDepthTest(!tactical);
		}
		arrow.setCullHint(CullHint.Always);
		if (!Float.isFinite(targetWorld.x) || !Float.isFinite(targetWorld.y) || !Float.isFinite(targetWorld.z))
			return;

		targetWorld.subtract(vehicleWorldPosition, target);
		// Displayed vehicle rotations are normalized; their conjugate maps world back to OBJ axes.
		inverseRotation.set(-vehicleWorldRotation.getX(), -vehicleWorldRotation.getY(), -vehicleWorldRotation.getZ(),
				vehicleWorldRotation.getW());
		inverseRotation.mult(target, target);
		float centerY = (float) -hull.forwardOffset(), centerZ = (float) hull.upOffset();
		float x = target.x, y = target.y - centerY, z = target.z - centerZ;
		float hullX = x / (float) hull.semiBeam(), hullY = y / (float) hull.semiLength();
		float hullZ = z / (float) hull.semiHeight();
		if (hullX * hullX + hullY * hullY + hullZ * hullZ <= 1)
			return;

		// The oval is only a placement boundary. Padding keeps even a sideways arrow clear of the body.
		float halfLength = (float) hull.semiLength() * 1.15f;
		float halfBeam = (float) hull.semiBeam() * 1.5f;
		float length = Math.max(12f, (float) hull.semiLength() * 0.55f);
		float radius = Math.max(1.7f, (float) hull.semiBeam() * 0.42f);
		if (tactical) {
			float units = unitsPerPixel(camera, vehicleWorldPosition);
			halfLength = Math.max(halfLength, 14f * units);
			halfBeam = Math.max(halfBeam, 4.5f * units);
			length = Math.max(length, 11f * units);
			radius = Math.max(radius, 2.5f * units);
		}
		// Wide tactical heads must still start outside the hull on broad ships as well as submarines.
		halfLength = Math.max(halfLength, (float) hull.semiLength() + radius + 0.5f);
		halfBeam = Math.max(halfBeam, (float) hull.semiBeam() + radius + 0.5f);

		float bearing = x * x / (halfBeam * halfBeam) + y * y / (halfLength * halfLength);
		float rimX, rimY;
		if (bearing > 0.000001f) {
			float scale = 1f / (float) Math.sqrt(bearing);
			rimX = x * scale;
			rimY = y * scale;
		} else {
			// A vertical waypoint has no horizontal bearing; use the bow consistently.
			rimX = 0;
			rimY = -halfLength;
		}
		arrow.setLocalTranslation(rimX, centerY + rimY, centerZ);
		// The oval offsets a direction cue, not a connector. In particular, a nearby target inside
		// the enlarged tactical oval must keep its bearing rather than flip the arrow towards the hull.
		direction.set(x, y, z).normalizeLocal();
		up.set(0, 0, 1);
		if (Math.abs(direction.z) > 0.9f)
			up.set(1, 0, 0);
		arrowRotation.lookAt(direction, up);
		arrow.setLocalRotation(arrowRotation);
		arrow.setLocalScale(radius, radius, length);
		arrow.setCullHint(CullHint.Never);
	}

	private static float unitsPerPixel(Camera camera, Vector3f position) {
		Vector3f origin = camera.getLocation(), direction = camera.getDirection();
		float depth = (position.x - origin.x) * direction.x + (position.y - origin.y) * direction.y
				+ (position.z - origin.z) * direction.z;
		float height = camera.getHeight() * (camera.getViewPortTop() - camera.getViewPortBottom());
		float units = (camera.getFrustumTop() - camera.getFrustumBottom()) / Math.max(1f, height);
		return camera.isParallelProjection() ? units
				: units * Math.max(camera.getFrustumNear(), depth) / camera.getFrustumNear();
	}

	/**
	 * Closed square shaft and pyramid head along +Z, reused through transforms without mesh
	 * uploads.
	 */
	private static Mesh arrowMesh() {
		Mesh mesh = new Mesh();
		mesh.setBuffer(VertexBuffer.Type.Position, 3,
				new float[] {-0.3f, -0.3f, 0, 0.3f, -0.3f, 0, 0.3f, 0.3f, 0, -0.3f, 0.3f, 0, -0.3f, -0.3f, 0.6f, 0.3f,
						-0.3f, 0.6f, 0.3f, 0.3f, 0.6f, -0.3f, 0.3f, 0.6f, -1, -1, 0.6f, 1, -1, 0.6f, 1, 1, 0.6f, -1, 1,
						0.6f, 0, 0, 1});
		mesh.setBuffer(VertexBuffer.Type.Index, 3,
				new int[] {0, 2, 1, 0, 3, 2, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7, 4,
						5, 6, 4, 6, 7, 8, 10, 9, 8, 11, 10, 8, 9, 12, 9, 10, 12, 10, 11, 12, 11, 8, 12});
		mesh.updateBound();
		mesh.setStatic();
		return mesh;
	}
}
