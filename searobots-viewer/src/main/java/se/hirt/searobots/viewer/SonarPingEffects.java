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
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.api.TerrainMap;

import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

/** Optional 3D information overlay sharing the map's retained, replay-safe pulse events. */
final class SonarPingEffects {
	private static final int MAX_VISIBLE_PULSES = 4;
	private final AssetManager assets;
	private final Node node = new Node("sonar-pings");
	private final Map<MapRenderer.PingAnimation, Pulse> pulses = new HashMap<>();
	private TerrainMap terrain;
	private SonarTerrainVisibility visibility;

	SonarPingEffects(AssetManager assets, Node sceneRoot) {
		this.assets = assets;
		sceneRoot.attachChild(node);
	}

	void update(MapRenderer data, TerrainMap terrain, double tickRateHz, boolean enabled) {
		if (this.terrain != terrain) {
			clear();
			this.terrain = terrain;
			visibility = terrain == null ? null : SonarTerrainVisibility.forTerrain(terrain);
		}
		node.setCullHint(enabled ? Spatial.CullHint.Inherit : Spatial.CullHint.Always);
		if (!enabled || terrain == null) {
			clear();
			return;
		}
		long tick = data.simTick;
		// CopyOnWriteArrayList supplies a stable snapshot while the simulation publishes ticks.
		var events = List.copyOf(data.pingAnimations);
		var keep = new HashSet<MapRenderer.PingAnimation>();
		int first = Math.max(0, events.size() - MAX_VISIBLE_PULSES);
		for (int i = first; i < events.size(); i++) {
			var event = events.get(i);
			double radius = SonarPingGeometry.radius(tick, event.startTick(), tickRateHz);
			// Avoid building stale geometry when the simulation fast-forwards between frames.
			if (radius <= 0 || radius >= SonarPingGeometry.MAX_RADIUS)
				continue;
			keep.add(event);
			Pulse pulse = pulses.computeIfAbsent(event, e -> new Pulse(e, terrain));
			pulse.update(radius);
		}
		pulses.entrySet().removeIf(entry -> {
			if (keep.contains(entry.getKey()))
				return false;
			entry.getValue().sphere.removeFromParent();
			entry.getValue().bottom.removeFromParent();
			return true;
		});
	}

	void clear() {
		pulses.clear();
		node.detachAllChildren();
	}

	private final class Pulse {
		final SonarPingGeometry geometry;
		final SonarPingGeometry.Lines lines = new SonarPingGeometry.Lines();
		final Geometry sphere, bottom;
		final FloatBuffer spherePositions = BufferUtils.createFloatBuffer(lines.positions.length);
		final FloatBuffer bottomPositions = BufferUtils.createFloatBuffer(lines.positions.length);
		final ColorRGBA sphereColor, bottomColor;
		double previousRadius = -1;

		Pulse(MapRenderer.PingAnimation event, TerrainMap terrain) {
			geometry = new SonarPingGeometry(terrain, event.x(), event.y(), event.z(), visibility);
			var c = event.color();
			sphereColor = new ColorRGBA(c.getRed() / 255f, c.getGreen() / 255f, c.getBlue() / 255f, 0.16f);
			bottomColor = new ColorRGBA((c.getRed() / 255f + 1) * 0.5f, (c.getGreen() / 255f + 1) * 0.5f,
					(c.getBlue() / 255f + 1) * 0.5f, 0.75f);
			sphere = createLines("sonar-sphere", sphereColor, 1, spherePositions);
			bottom = createLines("sonar-bottom", bottomColor, 2, bottomPositions);
			node.attachChild(sphere);
			node.attachChild(bottom);
		}

		void update(double radius) {
			if (radius == previousRadius)
				return;
			previousRadius = radius;
			float fade = (float) (1 - radius / SonarPingGeometry.MAX_RADIUS);
			sphereColor.a = 0.16f * fade;
			bottomColor.a = 0.75f * fade;
			sphere.getMaterial().setColor("Color", sphereColor);
			bottom.getMaterial().setColor("Color", bottomColor);
			geometry.sphere(radius, lines);
			upload(sphere, spherePositions, lines);
			geometry.bottom(radius, lines);
			upload(bottom, bottomPositions, lines);
		}
	}

	private Geometry createLines(String name, ColorRGBA color, float width, FloatBuffer positions) {
		positions.limit(0);
		Mesh mesh = new Mesh();
		mesh.setMode(Mesh.Mode.Lines);
		mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
		mesh.setDynamic();
		Geometry result = new Geometry(name, mesh);
		Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setColor("Color", color);
		var state = material.getAdditionalRenderState();
		state.setBlendMode(RenderState.BlendMode.Alpha);
		state.setDepthTest(true);
		state.setDepthWrite(false);
		state.setLineWidth(width);
		result.setMaterial(material);
		// Informational wavefronts remain readable after water/fog, but respect scene depth.
		result.setQueueBucket(RenderQueue.Bucket.Translucent);
		result.setShadowMode(RenderQueue.ShadowMode.Off);
		result.setCullHint(Spatial.CullHint.Always);
		return result;
	}

	private static void upload(Geometry geometry, FloatBuffer buffer, SonarPingGeometry.Lines lines) {
		buffer.clear();
		buffer.put(lines.positions, 0, lines.size);
		buffer.flip();
		geometry.getMesh().getBuffer(VertexBuffer.Type.Position).updateData(buffer);
		geometry.getMesh().updateCounts();
		geometry.setCullHint(lines.size == 0 ? Spatial.CullHint.Always : Spatial.CullHint.Inherit);
		if (lines.size > 0)
			geometry.updateModelBound();
	}
}
