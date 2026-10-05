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

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import com.jme3.asset.AssetManager;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
import com.jme3.effect.shapes.EmitterSphereShape;
import com.jme3.light.PointLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.BillboardControl;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Sphere;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;

/**
 * Torpedo detonations in the 3D view, staged as an underwater blast: a white-hot flash with glare
 * and a point light that lights up the hulls and the sea floor nearby, a shockwave ring, a gas
 * bubble that swells, collapses with a second flash and rebounds before breaking up into a column
 * of rising bubbles, plus sparks, fire, murk and sinking debris. Shallow blasts also throw a spray
 * column and a foam ring up at the surface. Nearby blasts shake the camera and flash the screen.
 * <p>
 * Time is in seconds from the detonation; sizes are in metres and scale with a blast's strength (1
 * for a torpedo hitting the bottom or running out, more when it hits a submarine). Coordinates are
 * jME's (Y up, water surface at Y = 0).
 */
public final class ExplosionEffects {
	private static final String PARTICLE = "Common/MatDefs/Misc/Particle.j3md";
	private static final String UNSHADED = "Common/MatDefs/Misc/Unshaded.j3md";
	// How long a blast's effects live in all, enough for the last rising bubbles to fade
	private static final float LIFETIME = 12f;
	// Gas bubble: radius at its largest per unit strength, and its keyframes (time, fraction of that radius):
	// swells, collapses, rebounds, then breaks up while drifting upwards
	private static final float BUBBLE_RADIUS = 30f;
	private static final float[] BUBBLE_T = {0f, 0.45f, 1.0f, 1.45f, 3.0f};
	private static final float[] BUBBLE_R = {0.1f, 1f, 0.25f, 0.6f, 0.75f};
	private static final float COLLAPSE_TIME = 1.0f, BREAKUP_TIME = 1.45f, FADE_TIME = 3.0f;
	// Shockwave ring: final diameter per unit strength and how long it takes to get there
	private static final float SHOCK_SIZE = 260f, SHOCK_TIME = 0.7f;
	// Blasts this shallow (per unit strength) break the surface
	private static final float SURFACE_DEPTH = 90f;
	// Camera shake in metres at point-blank range, and the range at which it is halved
	private static final float SHAKE = 2.5f, SHAKE_RANGE = 250f;
	// Shake frequencies (radians per second): two incommensurate sines per axis keep it from looking periodic
	private static final float[] SHAKE_F1 = {47f, 53f, 41f, 37f, 43f, 59f};
	private static final float[] SHAKE_F2 = {83f, 71f, 97f, 89f, 67f, 79f};

	// Colours above 1 saturate to white in the middle of additive blends
	private static final ColorRGBA HOT = new ColorRGBA(2.2f, 2.0f, 1.7f, 1f);
	private static final ColorRGBA FIRE = new ColorRGBA(1.8f, 0.7f, 0.2f, 1f);
	private static final ColorRGBA PULSE = new ColorRGBA(1.6f, 1.1f, 0.7f, 1f);
	private static final ColorRGBA BUBBLE = new ColorRGBA(0.7f, 0.82f, 0.95f, 1f);
	private static final ColorRGBA LIGHT = new ColorRGBA(1f, 0.72f, 0.42f, 1f);

	private final AssetManager assets;
	private final Node root;
	private final Camera cam;
	private final Random random = new Random();
	private final List<Blast> blasts = new ArrayList<>();
	private final List<Blast> pending = new ArrayList<>(); // secondary blasts waiting out their delay
	private final Geometry screenFlash;
	private final Texture2D bubbleTexture;
	private final float[] shakePhase = new float[SHAKE_F1.length * 2];
	private float clock;

	public ExplosionEffects(AssetManager assets, Node root, Node gui, Camera cam) {
		this.assets = assets;
		this.root = root;
		this.cam = cam;
		this.bubbleTexture = createBubbleTexture();
		for (int i = 0; i < shakePhase.length; i++)
			shakePhase[i] = random.nextFloat() * FastMath.TWO_PI;

		screenFlash = new Geometry("explosionScreenFlash", new Quad(1, 1));
		Material m = new Material(assets, UNSHADED);
		m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
		m.getAdditionalRenderState().setDepthTest(false);
		m.getAdditionalRenderState().setDepthWrite(false);
		screenFlash.setMaterial(m);
		screenFlash.setCullHint(Spatial.CullHint.Always);
		gui.attachChild(screenFlash);
	}

	/**
	 * Starts a blast at {@code position} in the colour of the submarine whose torpedo it was. A
	 * torpedo that hits a submarine makes a bigger blast, followed by secondary explosions about
	 * the hull.
	 */
	public void spawn(Vector3f position, ColorRGBA teamColor, boolean submarineHit) {
		blasts.add(new Blast(position.clone(), teamColor.clone(), submarineHit ? 1.5f : 1f, true));
		if (submarineHit) {
			int secondaries = 2 + random.nextInt(2);
			for (int i = 0; i < secondaries; i++) {
				Vector3f offset = new Vector3f(random.nextFloat() - 0.5f, (random.nextFloat() - 0.5f) * 0.4f,
						random.nextFloat() - 0.5f).multLocal(28f);
				var b = new Blast(position.add(offset), teamColor.clone(), 0.4f + random.nextFloat() * 0.15f, false);
				b.age = -(0.3f + 0.25f * i + random.nextFloat() * 0.2f);
				pending.add(b);
			}
		}
	}

	/** Advances all blasts by {@code tpf}; while {@code paused} they hold still. */
	public void update(float tpf, boolean paused) {
		for (var b : blasts)
			b.setFrozen(paused);
		if (paused) {
			screenFlash.setCullHint(Spatial.CullHint.Always);
			return;
		}
		clock += tpf;
		for (Iterator<Blast> it = pending.iterator(); it.hasNext();) {
			var b = it.next();
			b.age += tpf;
			if (b.age >= 0) {
				it.remove();
				b.age = 0;
				blasts.add(b);
			}
		}
		float flash = 0;
		for (Iterator<Blast> it = blasts.iterator(); it.hasNext();) {
			var b = it.next();
			if (!b.started)
				b.start();
			else
				b.age += tpf;
			if (b.age > LIFETIME) {
				b.dispose();
				it.remove();
				continue;
			}
			b.animate();
			flash += b.screenFlash();
		}
		updateScreenFlash(Math.min(0.8f, flash));
	}

	/**
	 * Shakes the camera: moves {@code camPos} and, by a different amount, {@code lookAt}, so the
	 * view both jolts and twists, scaled by how strong and how close the blasts are.
	 */
	public void applyShake(Vector3f camPos, Vector3f lookAt) {
		float amp = 0;
		for (var b : blasts) {
			float d = camPos.distance(b.pos) / SHAKE_RANGE;
			float decay = FastMath.exp(-3.2f * b.age) + 0.35f * pulse(b.age);
			amp += SHAKE * b.strength * decay / (1f + d * d);
		}
		if (amp < 0.005f)
			return;
		camPos.addLocal(amp * wobble(0), amp * wobble(1), amp * wobble(2));
		lookAt.addLocal(amp * 0.7f * wobble(3), amp * 0.7f * wobble(4), amp * 0.7f * wobble(5));
	}

	private float wobble(int axis) {
		return 0.6f * FastMath.sin(clock * SHAKE_F1[axis] + shakePhase[axis])
				+ 0.4f * FastMath.sin(clock * SHAKE_F2[axis] + shakePhase[axis + SHAKE_F1.length]);
	}

	private void updateScreenFlash(float alpha) {
		if (alpha < 0.005f) {
			screenFlash.setCullHint(Spatial.CullHint.Always);
			return;
		}
		screenFlash.setCullHint(Spatial.CullHint.Never);
		screenFlash.setLocalScale(cam.getWidth(), cam.getHeight(), 1f);
		screenFlash.getMaterial().setColor("Color", new ColorRGBA(1f, 0.92f, 0.8f, alpha));
	}

	/**
	 * Flash brightness over time: a near-instant rise, a fast decay, and a lesser flash at bubble
	 * collapse.
	 */
	private static float flashCurve(float age) {
		float main = age < 0.02f ? age / 0.02f : FastMath.exp(-(age - 0.02f) * 9f);
		return Math.max(main, 0.4f * pulse(age));
	}

	/** A short bell around the moment the gas bubble collapses. */
	private static float pulse(float age) {
		float x = (age - COLLAPSE_TIME) / 0.08f;
		return FastMath.exp(-x * x);
	}

	private static float smooth(float x) {
		x = FastMath.clamp(x, 0f, 1f);
		return x * x * (3f - 2f * x);
	}

	/** 0 at the start, 1 at the end, fast at first and slowing down. */
	private static float easeOut(float u) {
		u = FastMath.clamp(u, 0f, 1f);
		return 1f - (1f - u) * (1f - u);
	}

	/**
	 * Eased interpolation through keyframes {@code values} at {@code times}, held beyond the ends.
	 */
	private static float keyframes(float t, float[] times, float[] values) {
		if (t <= times[0])
			return values[0];
		for (int i = 1; i < times.length; i++) {
			if (t < times[i]) {
				float u = smooth((t - times[i - 1]) / (times[i] - times[i - 1]));
				return values[i - 1] + (values[i] - values[i - 1]) * u;
			}
		}
		return values[values.length - 1];
	}

	private static ColorRGBA scaled(ColorRGBA c, float k, float alpha) {
		return new ColorRGBA(c.r * k, c.g * k, c.b * k, alpha);
	}

	private static void show(Spatial s, boolean visible) {
		s.setCullHint(visible ? Spatial.CullHint.Never : Spatial.CullHint.Always);
	}

	/** One blast and everything it puts in the scene. */
	private final class Blast {
		final Vector3f pos;
		final ColorRGBA team;
		final float strength;
		final boolean primary; // secondary blasts skip the surface effects and the rising bubble column
		final List<Spatial> parts = new ArrayList<>();
		float age;
		boolean started, frozen, plumeBurst;

		Geometry core, bubble;
		Node flare, shock, foam;
		PointLight light;
		ParticleEmitter sparks, fire, murk, debris, plume, spray;

		Blast(Vector3f pos, ColorRGBA team, float strength, boolean primary) {
			this.pos = pos;
			this.team = team;
			this.strength = strength;
			this.primary = primary;
		}

		void start() {
			started = true;
			float s = strength;

			// White-hot core: an additive sphere that also glows for the bloom filter
			core = new Geometry("blastCore", new Sphere(20, 20, 1f));
			Material cm = new Material(assets, UNSHADED);
			cm.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
			cm.getAdditionalRenderState().setDepthWrite(false);
			core.setMaterial(cm);
			attach(core);

			// Glare: a camera-facing flash sprite much larger than the core, and the shockwave, a ring racing outwards
			flare = billboard("blastFlare", "Effects/Explosion/flash.png", 2, true);
			shock = billboard("blastShock", "Effects/Explosion/shockwave.png", 1, false);

			// Gas bubble: a shiny, translucent lit sphere
			bubble = new Geometry("blastBubble", new Sphere(24, 24, 1f));
			Material bm = new Material(assets, "Common/MatDefs/Light/Lighting.j3md");
			bm.setBoolean("UseMaterialColors", true);
			bm.setColor("Specular", ColorRGBA.White);
			bm.setFloat("Shininess", 48f);
			bm.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
			bm.getAdditionalRenderState().setDepthWrite(false);
			bubble.setMaterial(bm);
			attach(bubble);

			light = new PointLight(pos.clone(), ColorRGBA.Black, 300f * s);
			root.addLight(light);

			sparks = emitter("blastSparks", "Effects/Explosion/spark.png", (int) (90 * s), 1, 1, true);
			sparks.setFacingVelocity(true);
			sparks.setStartColor(new ColorRGBA(1f, 0.95f, 0.7f, 1f));
			sparks.setEndColor(new ColorRGBA(1f, 0.35f, 0.05f, 0f));
			sparks.setStartSize(2.2f * s);
			sparks.setEndSize(0.4f * s);
			sparks.setLowLife(0.35f);
			sparks.setHighLife(1.0f);
			sparks.setGravity(0, 6f, 0); // jME gravity points the other way: positive Y pulls down
			sparks.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 75f * s, 0));
			sparks.getParticleInfluencer().setVelocityVariation(1f);

			fire = emitter("blastFire", "Effects/Explosion/flame.png", (int) (40 * s), 2, 2, true);
			fire.setRandomAngle(true);
			fire.setRotateSpeed(1.5f);
			fire.setStartColor(new ColorRGBA(1f, 0.85f, 0.4f, 1f));
			fire.setEndColor(new ColorRGBA(0.6f + 0.4f * team.r, 0.15f + 0.3f * team.g, 0.4f * team.b, 0f));
			fire.setStartSize(6f * s);
			fire.setEndSize(24f * s);
			fire.setLowLife(0.6f);
			fire.setHighLife(1.6f);
			fire.setGravity(0, -3f, 0); // hot gas rises
			fire.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 20f * s, 0));
			fire.getParticleInfluencer().setVelocityVariation(1f);

			// Murk: churned-up water and soot that hangs about after the flash
			murk = emitter("blastMurk", "Effects/Smoke/Smoke.png", (int) (18 * s), 15, 1, false);
			murk.setSelectRandomImage(true);
			murk.setRandomAngle(true);
			murk.setRotateSpeed(0.3f);
			murk.setStartColor(new ColorRGBA(0.18f, 0.16f, 0.14f, 0.55f));
			murk.setEndColor(new ColorRGBA(0.1f, 0.14f, 0.18f, 0f));
			murk.setStartSize(12f * s);
			murk.setEndSize(45f * s);
			murk.setLowLife(3f);
			murk.setHighLife(5f);
			murk.setGravity(0, -1.5f, 0);
			murk.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 9f * s, 0));
			murk.getParticleInfluencer().setVelocityVariation(1f);

			debris = emitter("blastDebris", "Effects/Explosion/Debris.png", (int) (24 * s), 3, 3, false);
			debris.setSelectRandomImage(true);
			debris.setRandomAngle(true);
			debris.setRotateSpeed(5f);
			debris.setStartColor(new ColorRGBA(0.5f, 0.45f, 0.4f, 1f));
			debris.setEndColor(new ColorRGBA(0.15f, 0.15f, 0.15f, 0f));
			debris.setStartSize(1.2f * s);
			debris.setEndSize(1.8f * s);
			debris.setLowLife(2f);
			debris.setHighLife(4f);
			debris.setGravity(0, 5f, 0); // debris sinks
			debris.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 35f * s, 0));
			debris.getParticleInfluencer().setVelocityVariation(0.9f);

			for (var e : new ParticleEmitter[] {sparks, fire, murk, debris})
				e.emitAllParticles();

			if (primary) {
				// Column of bubbles from the broken-up gas bubble, each living only as long as it takes to reach the
				// surface
				plume = emitter("blastPlume", null, (int) (500 * s), 1, 1, false);
				plume.getMaterial().setTexture("Texture", bubbleTexture);
				plume.setShape(new EmitterSphereShape(Vector3f.ZERO, BUBBLE_RADIUS * 0.5f * s));
				plume.setStartColor(new ColorRGBA(0.85f, 0.95f, 1f, 0.85f));
				plume.setEndColor(new ColorRGBA(0.85f, 0.95f, 1f, 0f));
				plume.setStartSize(1.2f);
				plume.setEndSize(3.5f);
				plume.setGravity(0, -2f, 0); // buoyancy: bubbles speed up as they rise
				plume.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 6f, 0));
				plume.getParticleInfluencer().setVelocityVariation(0.4f);
				// Rising d metres at 6 m/s, accelerating at 2 m/s^2, takes t where d = 6t + t^2
				float depth = Math.max(1f, -pos.y - BUBBLE_RADIUS * 0.5f * s);
				float toSurface = FastMath.clamp((-6f + FastMath.sqrt(36f + 4f * depth)) / 2f, 0.5f, 8f);
				plume.setHighLife(toSurface);
				plume.setLowLife(toSurface * 0.5f);
			}
		}

		/** Sets everything for the current {@link #age}. */
		void animate() {
			float s = strength, t = age;

			// Gas bubble: its centre starts drifting up once it breaks up
			float rise = t > BREAKUP_TIME ? 3f * (t - BREAKUP_TIME) * (t - BREAKUP_TIME) : 0f;
			Vector3f centre = pos.add(0, rise, 0);
			float bubbleA = 0.35f * (1f - smooth((t - BREAKUP_TIME) / (FADE_TIME - BREAKUP_TIME)));
			show(bubble, bubbleA > 0.01f);
			if (bubbleA > 0.01f) {
				bubble.setLocalTranslation(centre);
				bubble.setLocalScale(BUBBLE_RADIUS * s * keyframes(t, BUBBLE_T, BUBBLE_R));
				ColorRGBA tint = BUBBLE.clone().interpolateLocal(team, 0.2f);
				bubble.getMaterial().setColor("Diffuse", scaled(tint, 1f, bubbleA));
				bubble.getMaterial().setColor("Ambient", scaled(tint, 0.6f, bubbleA));
			}

			// Core: the main flash and fireball, then a smaller re-flash when the bubble collapses
			float main = t < 0.02f ? t / 0.02f : FastMath.exp(-(t - 0.02f) * 5f);
			float collapse = 0.8f * pulse(t);
			float coreA = Math.max(main, collapse);
			show(core, coreA > 0.01f);
			if (coreA > 0.01f) {
				boolean fireball = main >= collapse;
				ColorRGBA c = fireball ? HOT.clone().interpolateLocal(FIRE, smooth(t / 0.35f)) : PULSE;
				core.setLocalTranslation(centre);
				core.setLocalScale(s * (fireball ? 5f + 20f * smooth(t / 0.5f) : 5f + 5f * collapse));
				core.getMaterial().setColor("Color", scaled(c, 1f, Math.min(1f, coreA)));
				core.getMaterial().setColor("GlowColor", scaled(c, 0.8f * coreA, 1f));
			}

			// Glare: biggest in the first instant, shrinking as it fades; back briefly at the collapse
			float early = t < 0.35f ? FastMath.pow(1f - t / 0.35f, 1.5f) : 0f;
			float flareA = Math.max(early, 0.6f * pulse(t));
			show(flare, flareA > 0.01f);
			if (flareA > 0.01f) {
				flare.setLocalTranslation(centre);
				flare.setLocalScale(s * (t < 0.35f ? 120f - 60f * t / 0.35f : 50f));
				setBillboardColor(flare, new ColorRGBA(1.5f, 1.3f, 1.0f, flareA));
			}

			// Shockwave ring
			float u = t / SHOCK_TIME;
			show(shock, u < 1f);
			if (u < 1f) {
				shock.setLocalTranslation(pos);
				shock.setLocalScale(s * SHOCK_SIZE * (0.05f + 0.95f * easeOut(u)));
				setBillboardColor(shock, new ColorRGBA(0.8f, 0.9f, 1f, 0.9f * (1f - u) * (1f - u)));
			}

			// Light: what the flash and fireball throw on everything around
			if (light != null) {
				if (t > FADE_TIME) {
					root.removeLight(light);
					light = null;
				} else {
					float k = s * (4f * flashCurve(t) + 0.5f * FastMath.exp(-t * 1.5f));
					light.setColor(LIGHT.clone().interpolateLocal(team, 0.15f).multLocal(k));
					light.setPosition(centre);
				}
			}

			if (plume != null) {
				plume.setLocalTranslation(centre);
				if (t >= BREAKUP_TIME && !plumeBurst) {
					plumeBurst = true;
					plume.emitParticles((int) (160 * s));
				}
				plume.setParticlesPerSec(t < BREAKUP_TIME ? 0f : 110f * s * (1f - smooth((t - BREAKUP_TIME) / 3.5f)));
			}

			if (primary && pos.y > -SURFACE_DEPTH * s) {
				// The surface heaves a moment after the shock reaches it: the shallower, the sooner and the higher
				float depthFrac = Math.max(0f, -pos.y) / (SURFACE_DEPTH * s);
				float heaveAt = 0.15f + 0.6f * depthFrac;
				if (spray == null && t >= heaveAt)
					startSurface(1f - depthFrac);
				if (foam != null) {
					float f = (t - heaveAt) / 6f;
					show(foam, f < 1f);
					foam.setLocalScale(s * (40f + 120f * easeOut(f)));
					((Geometry) foam.getChild(0)).getMaterial().setColor("Color",
							new ColorRGBA(0.95f, 0.98f, 1f, 0.8f * (1f - f)));
				}
			}
		}

		/**
		 * Spray column and foam ring where the blast breaks the surface; {@code power} 0..1 by
		 * shallowness.
		 */
		private void startSurface(float power) {
			float s = strength;
			spray = emitter("blastSpray", "Effects/Smoke/Smoke.png", (int) (180 * s), 15, 1, false);
			spray.setSelectRandomImage(true);
			spray.setRandomAngle(true);
			spray.setRotateSpeed(0.5f);
			spray.setShape(new EmitterSphereShape(Vector3f.ZERO, 5f * s));
			spray.setStartColor(new ColorRGBA(0.95f, 0.97f, 1f, 0.95f));
			spray.setEndColor(new ColorRGBA(0.9f, 0.93f, 0.97f, 0f));
			spray.setStartSize(5f * s);
			spray.setEndSize(18f * s);
			spray.setLowLife(3.5f);
			spray.setHighLife(6f);
			spray.setGravity(0, 9.8f, 0);
			spray.getParticleInfluencer().setVelocityVariation(0.12f);
			spray.setLocalTranslation(pos.x, 1f, pos.z);
			// Launch it in waves of falling speed so it rises as a column rather than a single puff
			float top = (12f + 33f * power) * s;
			int waves = 6, perWave = spray.getMaxNumParticles() / waves;
			for (int i = 0; i < waves; i++) {
				spray.getParticleInfluencer().setInitialVelocity(new Vector3f(0, top * (1f - 0.12f * i), 0));
				spray.emitParticles(perWave);
			}

			// Foam ring lying flat on the water, centred over the blast
			Geometry ring = new Geometry("blastFoam", new Quad(1, 1));
			Material m = new Material(assets, UNSHADED);
			m.setTexture("ColorMap", assets.loadTexture("Effects/Explosion/shockwave.png"));
			m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
			m.getAdditionalRenderState().setDepthWrite(false);
			m.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
			ring.setMaterial(m);
			ring.setQueueBucket(RenderQueue.Bucket.Transparent);
			ring.setLocalTranslation(-0.5f, -0.5f, 0f);
			foam = new Node("blastFoamNode");
			foam.attachChild(ring);
			foam.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0));
			foam.setLocalTranslation(pos.x, 0.8f, pos.z);
			attach(foam);
		}

		float screenFlash() {
			float d = cam.getLocation().distance(pos) / 300f;
			Vector3f screen = cam.getScreenCoordinates(pos);
			boolean inView = screen.z > 0f && screen.z < 1f && screen.x >= 0 && screen.y >= 0
					&& screen.x <= cam.getWidth() && screen.y <= cam.getHeight();
			return 0.45f * strength * flashCurve(age) * (inView ? 1f : 0.3f) / (1f + d * d);
		}

		void setFrozen(boolean frozen) {
			if (this.frozen == frozen)
				return;
			this.frozen = frozen;
			for (var e : new ParticleEmitter[] {sparks, fire, murk, debris, plume, spray})
				if (e != null)
					e.setEnabled(!frozen);
		}

		void dispose() {
			for (var p : parts)
				p.removeFromParent();
			if (light != null)
				root.removeLight(light);
		}

		private void attach(Spatial s) {
			if (s instanceof Geometry)
				s.setQueueBucket(RenderQueue.Bucket.Transparent);
			s.setCullHint(Spatial.CullHint.Always);
			root.attachChild(s);
			parts.add(s);
		}

		/**
		 * A unit quad centred on a node that always faces the camera, showing the first of
		 * {@code cells} by {@code cells} images in {@code texture}.
		 */
		private Node billboard(String name, String texture, int cells, boolean glow) {
			var mesh = new Quad(1, 1);
			mesh.scaleTextureCoordinates(new Vector2f(1f / cells, 1f / cells));
			Geometry quad = new Geometry(name, mesh);
			Material m = new Material(assets, UNSHADED);
			Texture tex = assets.loadTexture(texture);
			m.setTexture("ColorMap", tex);
			if (glow)
				m.setTexture("GlowMap", tex);
			m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
			m.getAdditionalRenderState().setDepthWrite(false);
			m.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
			quad.setMaterial(m);
			quad.setQueueBucket(RenderQueue.Bucket.Transparent);
			quad.setLocalTranslation(-0.5f, -0.5f, 0f);
			Node n = new Node(name + "Node");
			n.attachChild(quad);
			n.addControl(new BillboardControl());
			attach(n);
			return n;
		}

		private void setBillboardColor(Node n, ColorRGBA c) {
			Material m = ((Geometry) n.getChild(0)).getMaterial();
			m.setColor("Color", c);
			if (m.getParam("GlowMap") != null)
				m.setColor("GlowColor", new ColorRGBA(c.r * c.a, c.g * c.a, c.b * c.a, 1f));
		}

		private ParticleEmitter emitter(
			String name, String texture, int count, int imagesX, int imagesY, boolean additive) {
			var e = new ParticleEmitter(name, ParticleMesh.Type.Triangle, Math.max(1, count));
			Material m = new Material(assets, PARTICLE);
			if (texture != null)
				m.setTexture("Texture", assets.loadTexture(texture));
			m.getAdditionalRenderState()
					.setBlendMode(additive ? RenderState.BlendMode.AlphaAdditive : RenderState.BlendMode.Alpha);
			e.setMaterial(m);
			e.setImagesX(imagesX);
			e.setImagesY(imagesY);
			e.setParticlesPerSec(0);
			e.setLocalTranslation(pos);
			e.setQueueBucket(RenderQueue.Bucket.Transparent);
			root.attachChild(e);
			parts.add(e);
			return e;
		}
	}

	/**
	 * A round bubble: nearly clear in the middle, a bright rim, and a highlight up and to the left.
	 */
	private static Texture2D createBubbleTexture() {
		int size = 64;
		ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
		float c = size / 2f;
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				float dx = (x + 0.5f - c) / c, dy = (y + 0.5f - c) / c;
				float d = FastMath.sqrt(dx * dx + dy * dy);
				float a = 0f;
				if (d <= 1f) {
					float rim = FastMath.pow(d, 6f) * smooth((1f - d) / 0.08f);
					float hx = dx + 0.35f, hy = dy - 0.35f;
					float highlight = Math.max(0f, 1f - FastMath.sqrt(hx * hx + hy * hy) / 0.25f);
					a = Math.min(1f, 0.12f + rim + highlight * highlight);
				}
				buf.put((byte) 255).put((byte) 255).put((byte) 255).put((byte) (255 * a));
			}
		}
		buf.flip();
		return new Texture2D(new Image(Image.Format.RGBA8, size, size, buf, ColorSpace.sRGB));
	}
}
