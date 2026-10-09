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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import com.jme3.asset.AssetManager;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Quad;
import com.jme3.texture.Texture;

/**
 * What a dead submarine gives off as it sinks: oily black smoke-like fuel and lubricant seeping
 * from breaches along the hull, a steady stream of fine air bubbles, and irregular belches of air
 * from flooding compartments, big bubbles mixed with oily ones. It is worst just after the hull is
 * breached and dies down over the following minutes, never quite stopping. The oil that reaches the
 * surface spreads into a dark slick with a rainbow sheen over the wreck.
 * <p>
 * Coordinates are jME's (Y up, water surface at Y = 0); hull positions are in the submarine model's
 * own coordinates (Y fore and aft, Z up), turned into the world by the submarine's node.
 */
final class WreckEmissions {
	private static final String PARTICLE = "Common/MatDefs/Misc/Particle.j3md";
	// Breaches along the hull (model coordinates: X to starboard, Y fore and aft, Z up), where things leak from
	private static final Vector3f[] BREACHES = {new Vector3f(1.2f, -22f, 3.2f), new Vector3f(-1.5f, -9f, 3.6f),
			new Vector3f(0.4f, 4f, 3.8f), new Vector3f(-1.0f, 17f, 3.0f), new Vector3f(1.6f, 25f, 2.4f)};
	// How strong the leaking is, falling from 1 at the hull breach to LINGER, by e every DECAY seconds
	private static final float LINGER = 0.25f, DECAY = 35f;
	// Per second at full strength: oil, fine bubbles; seconds between belches of air at full strength
	private static final float OIL_RATE = 7f, FINE_RATE = 30f, BELCH_INTERVAL = 1.2f;
	// Oil slick: when it shows (seconds per metre of depth, and the cap), how big it grows (metres) and how fast
	private static final float SLICK_DELAY_PER_M = 0.08f, SLICK_DELAY_MAX = 15f, SLICK_START = 25f, SLICK_SIZE = 140f,
			SLICK_GROWTH = 70f;
	// Height the slick lies at, clear of the water filter's waves (WaterFilter max amplitude 1.5 m)
	private static final float SLICK_HEIGHT = 1.8f;
	private static final ColorRGBA OIL_START = new ColorRGBA(0.05f, 0.042f, 0.03f, 0.75f);
	private static final ColorRGBA OIL_END = new ColorRGBA(0.09f, 0.085f, 0.07f, 0f);
	private static final ColorRGBA AIR_START = new ColorRGBA(0.85f, 0.95f, 1f, 0.95f);
	private static final ColorRGBA AIR_END = new ColorRGBA(0.85f, 0.95f, 1f, 0f);
	private static final ColorRGBA OILY_START = new ColorRGBA(0.22f, 0.18f, 0.13f, 0.85f);
	private static final ColorRGBA OILY_END = new ColorRGBA(0.35f, 0.33f, 0.3f, 0f);

	private final AssetManager assets;
	private final Node root;
	private final Camera cam;
	private final Random random = new Random();
	private final Map<Integer, Wreck> wrecks = new HashMap<>();

	WreckEmissions(AssetManager assets, Node root, Camera cam) {
		this.assets = assets;
		this.root = root;
		this.cam = cam;
	}

	/**
	 * Keeps the wreck of submarine {@code id}, whose model is {@code sub}, leaking; call each frame
	 * while it is dead. While {@code paused}, everything holds still.
	 */
	void update(int id, Node sub, float tpf, boolean paused) {
		var w = wrecks.get(id);
		if (w != null && w.sub != sub) {
			// A new match (the same seed, say) reuses the id for a new model: the old wreck is gone
			w.dispose();
			w = null;
		}
		if (w == null) {
			w = new Wreck(sub);
			wrecks.put(id, w);
		}
		w.setFrozen(paused);
		w.fadeWithDistance(); // the camera may move while time stands still
		if (!paused)
			w.update(Math.min(tpf, ExplosionEffects.MAX_STEP));
	}

	/**
	 * Clears away the wrecks of submarines not in {@code ids}, such as after a new match starts.
	 */
	void retain(Set<Integer> ids) {
		for (Iterator<Map.Entry<Integer, Wreck>> it = wrecks.entrySet().iterator(); it.hasNext();) {
			var e = it.next();
			if (!ids.contains(e.getKey())) {
				e.getValue().dispose();
				it.remove();
			}
		}
	}

	private final class Wreck {
		final Node sub;
		final List<Spatial> parts = new ArrayList<>();
		final ParticleEmitter oil, fine, belch, swarm, oily;
		Node slick;
		float age, oilDue, fineDue, nextBelch, slickAt = -1;
		int nextBreach;
		boolean frozen;

		Wreck(Node sub) {
			this.sub = sub;
			oil = emitter("wreckOil", null, 8, 260);
			oil.getMaterial().setTexture("Texture", EffectTextures.puffs());
			oil.setSelectRandomImage(true);
			oil.setRandomAngle(true);
			oil.setRotateSpeed(0.25f);
			oil.setStartSize(2f);
			oil.setEndSize(16f);
			oil.setGravity(0, -0.25f, 0); // it floats, slowly
			oil.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 1.5f, 0));
			oil.getParticleInfluencer().setVelocityVariation(0.5f);

			// Seeping air: each particle a little swarm of small bubbles, spreading as it rises
			fine = emitter("wreckBubbles", null, 4, 400);
			fine.getMaterial().setTexture("Texture", EffectTextures.bubbleSwarm());
			fine.setSelectRandomImage(true);
			fine.setStartSize(2f);
			fine.setEndSize(5f);
			fine.setGravity(0, -1.5f, 0); // buoyancy
			fine.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 3f, 0));
			fine.getParticleInfluencer().setVelocityVariation(0.35f);

			// A belch of air: a few big bubbles, flattened into wobbly caps (upright, so no random angle), in a
			// swarm of smaller ones
			belch = emitter("wreckBelch", null, 4, 120);
			belch.getMaterial().setTexture("Texture", EffectTextures.bigBubbles());
			belch.setSelectRandomImage(true);
			belch.setStartSize(1.4f);
			belch.setEndSize(3.2f);
			belch.setGravity(0, -2f, 0);
			belch.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 5f, 0));
			belch.getParticleInfluencer().setVelocityVariation(0.45f);

			swarm = emitter("wreckSwarm", null, 4, 400);
			swarm.getMaterial().setTexture("Texture", EffectTextures.bubbleSwarm());
			swarm.setSelectRandomImage(true);
			swarm.setStartSize(3f);
			swarm.setEndSize(8f);
			swarm.setGravity(0, -1.8f, 0);
			swarm.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 4.5f, 0));
			swarm.getParticleInfluencer().setVelocityVariation(0.6f);

			// Bubbles of oily air: caps like the others, upright, tinted with oil
			oily = emitter("wreckOilyBubbles", null, 4, 90);
			oily.getMaterial().setTexture("Texture", EffectTextures.bigBubbles());
			oily.setSelectRandomImage(true);
			oily.setStartSize(1.2f);
			oily.setEndSize(3.5f);
			oily.setGravity(0, -1.2f, 0);
			oily.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 4f, 0));
			oily.getParticleInfluencer().setVelocityVariation(0.5f);

			// The hull giving way: a big belch from every breach at once
			for (int i = 0; i < BREACHES.length; i++) {
				emitAt(belch, breach(i), 5, 5f, 2f);
				emitAt(swarm, breach(i), 20, 4.5f, 1.8f);
				emitAt(oily, breach(i), 3, 4f, 1.2f);
				emitAt(oil, breach(i), 6, 1.5f, 0f);
			}
			nextBelch = 0.5f;
		}

		/** Fades everything by how much of the wreck the camera can see through the water. */
		void fadeWithDistance() {
			float vis = ExplosionEffects.visibility(cam, sub.getWorldTranslation());
			fade(oil, OIL_START, OIL_END, vis);
			fade(fine, AIR_START, AIR_END, vis);
			fade(belch, AIR_START, AIR_END, vis);
			fade(swarm, AIR_START, AIR_END, vis);
			fade(oily, OILY_START, OILY_END, vis);
			for (var e : new ParticleEmitter[] {oil, fine, belch, swarm, oily})
				ExplosionEffects.refreshFrozen(e);
			if (slick != null)
				((Geometry) slick.getChild(0)).getMaterial().setColor("Color",
						new ColorRGBA(1f, 1f, 1f, ExplosionEffects.visibility(cam, slick.getLocalTranslation())));
		}

		void update(float tpf) {
			age += tpf;
			float k = LINGER + (1f - LINGER) * FastMath.exp(-age / DECAY);

			// Steady seeping, round the breaches in turn
			oilDue += OIL_RATE * k * tpf;
			fineDue += FINE_RATE * k * tpf;
			while (oilDue >= 1f) {
				oilDue -= 1f;
				emitAt(oil, breach(nextBreach++), 1, 1.5f, 0f);
			}
			while (fineDue >= 1f) {
				fineDue -= 1f;
				emitAt(fine, breach(random.nextInt(BREACHES.length)), 1, 3f, 1.5f);
			}

			// Now and then a compartment gives up its air: a belch of big bubbles in a swarm of small ones, with oily ones
			// and a gout of oil
			nextBelch -= tpf;
			if (nextBelch <= 0f) {
				Vector3f at = breach(random.nextInt(BREACHES.length));
				emitAt(belch, at, 2 + (int) (6 * k * random.nextFloat()), 5f, 2f);
				emitAt(swarm, at, (int) (10 + 25 * k * random.nextFloat()), 4.5f, 1.8f);
				emitAt(oily, at, 1 + (int) (5 * k * random.nextFloat()), 4f, 1.2f);
				emitAt(oil, at, 1 + (int) (3 * k), 1.5f, 0f);
				nextBelch = BELCH_INTERVAL * (0.4f + 1.6f * random.nextFloat()) / k;
			}

			updateSlick(tpf);
		}

		/**
		 * The slick: once the first oil has had time to rise, it spreads over the wreck and drifts
		 * with it.
		 */
		private void updateSlick(float tpf) {
			Vector3f p = sub.getWorldTranslation();
			if (slickAt < 0)
				slickAt = Math.min(SLICK_DELAY_MAX, Math.max(0f, -p.y) * SLICK_DELAY_PER_M);
			if (age < slickAt)
				return;
			if (slick == null) {
				Geometry quad = new Geometry("wreckSlick", new Quad(1, 1));
				Material m = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
				m.setTexture("ColorMap", EffectTextures.oilSlick());
				m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
				m.getAdditionalRenderState().setDepthWrite(false);
				m.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
				quad.setMaterial(m);
				quad.setQueueBucket(ExplosionEffects.EFFECTS);
				quad.setLocalTranslation(-0.5f, -0.5f, 0f);
				slick = new Node("wreckSlickNode");
				slick.attachChild(quad);
				slick.setLocalTranslation(p.x, SLICK_HEIGHT, p.z);
				root.attachChild(slick);
				parts.add(slick);
			}
			float grown = 1f - FastMath.exp(-(age - slickAt) / SLICK_GROWTH);
			slick.setLocalScale(SLICK_START + (SLICK_SIZE - SLICK_START) * grown);
			slick.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0.01f * age, 0));
			Vector3f at = slick.getLocalTranslation();
			slick.setLocalTranslation(at.x + (p.x - at.x) * Math.min(1f, tpf * 0.1f), SLICK_HEIGHT,
					at.z + (p.z - at.z) * Math.min(1f, tpf * 0.1f));
		}

		/** Where breach {@code i} is now, in the world. */
		private Vector3f breach(int i) {
			return sub.localToWorld(BREACHES[i % BREACHES.length], null);
		}

		/**
		 * Emits {@code n} particles at {@code at}, living until they would reach the surface rising
		 * at {@code speed} m/s, speeding up at {@code lift} m/s^2.
		 */
		private void emitAt(ParticleEmitter e, Vector3f at, int n, float speed, float lift) {
			float depth = Math.max(1f, -at.y);
			float t = lift > 0f ? (-speed + FastMath.sqrt(speed * speed + 2f * lift * depth)) / lift : depth / speed;
			float life = FastMath.clamp(t, 0.5f, e == oil ? 14f : 10f);
			e.setHighLife(life);
			e.setLowLife(life * 0.5f);
			e.setLocalTranslation(at);
			e.updateGeometricState(); // so new particles start from here, not from where it was last frame
			e.emitParticles(n);
		}

		private void fade(ParticleEmitter e, ColorRGBA start, ColorRGBA end, float v) {
			e.setStartColor(new ColorRGBA(start.r, start.g, start.b, start.a * v));
			e.setEndColor(new ColorRGBA(end.r, end.g, end.b, end.a * v));
		}

		void setFrozen(boolean frozen) {
			if (this.frozen == frozen)
				return;
			this.frozen = frozen;
			for (var e : new ParticleEmitter[] {oil, fine, belch, swarm, oily})
				e.setEnabled(!frozen);
		}

		void dispose() {
			for (var p : parts)
				p.removeFromParent();
		}

		private ParticleEmitter emitter(String name, String texture, int imagesX, int count) {
			var e = new ParticleEmitter(name, ParticleMesh.Type.Triangle, count);
			Material m = new Material(assets, PARTICLE);
			if (texture != null) {
				Texture t = assets.loadTexture(texture);
				m.setTexture("Texture", t);
			}
			m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
			e.setMaterial(m);
			e.setImagesX(imagesX);
			e.setImagesY(1);
			e.setParticlesPerSec(0);
			e.setQueueBucket(ExplosionEffects.EFFECTS);
			root.attachChild(e);
			parts.add(e);
			return e;
		}
	}
}
