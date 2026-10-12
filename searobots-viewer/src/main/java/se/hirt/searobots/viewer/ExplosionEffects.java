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
import java.util.Iterator;
import java.util.List;
import java.util.Random;

import com.jme3.asset.AssetManager;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
import com.jme3.effect.shapes.EmitterBoxShape;
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
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.BillboardControl;
import com.jme3.scene.shape.Dome;
import com.jme3.scene.shape.Quad;
import com.jme3.scene.shape.Sphere;
import com.jme3.texture.Texture;

/**
 * Torpedo detonations in the 3D view, staged as an underwater blast: a white-hot flash with glare
 * and a point light that lights up the hulls and the sea floor nearby, a shockwave ring, a cluster
 * of smoky gas bubbles that swell, collapse with a second flash and rebound before breaking up into
 * a column of rising bubbles, clean and smoky, plus sparks, fire, murk and sinking debris. Shallow
 * blasts also break the surface: the shock flashes across it in a white ring and heaves it up in a
 * dome of spray, then the gas bursts through in a column of spray and jets, which falls back and
 * rolls out as a ring of mist, leaving a ring of foam. Nearby blasts shake the camera and flash the
 * screen.
 * <p>
 * The effects draw in the translucent bucket, which needs a TranslucentBucketFilter after the water
 * and fog filters to come out on top of them (see {@link #EFFECTS}).
 * <p>
 * Time is in seconds from the detonation; sizes are in metres and scale with a blast's strength (1
 * for a torpedo hitting the bottom or running out, more when it hits a submarine). Coordinates are
 * jME's (Y up, water surface at Y = 0).
 */
public final class ExplosionEffects {
	private static final String PARTICLE = "Common/MatDefs/Misc/Particle.j3md";
	private static final String UNSHADED = "Common/MatDefs/Misc/Unshaded.j3md";
	// The effects draw in the translucent bucket, which a TranslucentBucketFilter puts after the water and fog filters.
	// Underwater, they are lights and murk in the water, not things for the haze to bleach. Above it, they are see-through
	// and write no depth, so the water filter would take them for the sea floor behind them and paint the surface over
	// them. Their own fading (see visibility) stands in for the fog and the surface. Only the glare stays in the
	// transparent bucket, for the bloom filter's glow pass, which skips the translucent one.
	static final RenderQueue.Bucket EFFECTS = RenderQueue.Bucket.Translucent;
	// Visibility: effects fade by e every VISIBLE_RANGE metres beyond the first VISIBLE_NEAR, and, seen through the
	// surface (from above it, or from below), by e every SURFACE_FADE metres of the water between
	private static final float VISIBLE_NEAR = 150f, VISIBLE_RANGE = 700f, SURFACE_FADE = 60f;
	// The longest step (seconds) the effects take in one frame, however long the frame took
	static final float MAX_STEP = 0.05f;
	// How long a blast's effects live in all, enough for the last rising bubbles to fade
	private static final float LIFETIME = 12f;
	// Gas bubbles: radius of the main one at its largest per unit strength, and its keyframes (time, fraction of that
	// radius): swells, collapses, rebounds, then breaks up while drifting upwards. Satellite bubbles follow the same
	// curve, smaller, a little late, and spread round the main one.
	private static final float BUBBLE_RADIUS = 26f;
	private static final float[] BUBBLE_T = {0f, 0.45f, 1.0f, 1.45f, 3.0f};
	private static final float[] BUBBLE_R = {0.1f, 1f, 0.25f, 0.6f, 0.75f};
	private static final int SATELLITES = 6;
	private static final float COLLAPSE_TIME = 1.0f, BREAKUP_TIME = 1.45f, FADE_TIME = 3.0f;
	// Shockwave ring: final diameter per unit strength and how long it takes to get there
	private static final float SHOCK_SIZE = 260f, SHOCK_TIME = 0.7f;
	// Blasts this shallow (per unit strength) break the surface
	private static final float SURFACE_DEPTH = 90f;
	// Speed of sound in sea water (m/s): when the shock reaches the surface
	private static final float SOUND_SPEED = 1500f;
	// Where the shock reaches the surface, it flashes white in a ring (final diameter per unit strength, and how long it
	// takes to get there) and heaves the water up in a dome of spray (how long it takes to rise). Seconds after the gas
	// breaks through in a column, the column falls back and rolls out as a ring of mist across the water.
	private static final float SPALL_SIZE = 300f, SPALL_TIME = 0.45f, DOME_TIME = 0.6f, SURGE_DELAY = 1.6f;
	// The flash ring fades by e every SPALL_FADE metres of depth (per unit strength). Blasts too deep to break the
	// surface send their gas up to boil at it: after BOIL_DELAY seconds plus a second per BOIL_RISE metres of depth
	// (quicker than real bubbles, so it comes while the shot still lasts), unless that is later than BOIL_LATEST.
	private static final float SPALL_FADE = 400f, BOIL_DELAY = 2f, BOIL_RISE = 40f, BOIL_LATEST = 14f;
	// Camera shake in metres at point-blank range, and the range at which it is halved
	private static final float SHAKE = 2.5f, SHAKE_RANGE = 250f;
	// Shake frequencies (radians per second): two incommensurate sines per axis keep it from looking periodic
	private static final float[] SHAKE_F1 = {47f, 53f, 41f, 37f, 43f, 59f};
	private static final float[] SHAKE_F2 = {83f, 71f, 97f, 89f, 67f, 79f};

	// Colours above 1 saturate to white in the middle of additive blends
	private static final ColorRGBA HOT = new ColorRGBA(2.2f, 2.0f, 1.7f, 1f);
	private static final ColorRGBA FIRE = new ColorRGBA(1.8f, 0.7f, 0.2f, 1f);
	private static final ColorRGBA PULSE = new ColorRGBA(1.6f, 1.1f, 0.7f, 1f);
	private static final ColorRGBA GAS = new ColorRGBA(0.62f, 0.6f, 0.58f, 1f); // smoky detonation products
	private static final ColorRGBA LIGHT = new ColorRGBA(1f, 0.72f, 0.42f, 1f);

	private final AssetManager assets;
	private final Node root;
	private final Camera cam;
	private final Random random = new Random();
	private final List<Blast> blasts = new ArrayList<>();
	private final List<Blast> pending = new ArrayList<>(); // secondary blasts waiting out their delay
	private final Geometry screenFlash;
	private final float[] shakePhase = new float[SHAKE_F1.length * 2];
	private float clock;

	public ExplosionEffects(AssetManager assets, Node root, Node gui, Camera cam) {
		this.assets = assets;
		this.root = root;
		this.cam = cam;
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
	 * the hull. Returns the main blast, to follow it.
	 */
	public Handle spawn(Vector3f position, ColorRGBA teamColor, boolean submarineHit) {
		var main = new Blast(position.clone(), teamColor.clone(), submarineHit ? 1.5f : 1f, true);
		blasts.add(main);
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
		return main;
	}

	/**
	 * Builds a blast and its surface effects out of sight and has {@code renderManager} prepare
	 * them (generate the textures, compile the shaders, upload the meshes), so the first real
	 * explosion does not stall the frame it goes off in.
	 */
	public void preload(RenderManager renderManager) {
		// Far off to the side, but shallow, so that it has every surface effect, too
		var b = new Blast(new Vector3f(1e5f, -1f, 1e5f), ColorRGBA.White.clone(), 1f, true);
		b.start();
		b.startSurface();
		b.startPlume();
		for (var p : b.parts)
			renderManager.preloadScene(p);
		b.dispose();
	}

	/**
	 * Advances all blasts by {@code tpf}; while {@code paused} they hold still. A step is at most
	 * {@link #MAX_STEP}, so a hitch in the frame rate slows a blast down rather than skipping part
	 * of it.
	 */
	public void update(float tpf, boolean paused) {
		for (var b : blasts)
			b.setFrozen(paused);
		if (paused) {
			// Time stands still (the screen flash, too, holds as it is), but the camera may move: keep the fading
			// with distance and the like up to date
			for (var b : blasts)
				if (b.started) {
					b.animate();
					for (var f : b.faded)
						refreshFrozen(f.emitter());
					for (var f : b.surfaceFaded)
						refreshFrozen(f.emitter());
				}
			return;
		}
		tpf = Math.min(tpf, MAX_STEP);
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
			if (b.age > b.lifetime()) {
				b.dispose();
				it.remove();
				continue;
			}
			b.animate();
			flash += b.screenFlash();
		}
		updateScreenFlash(Math.min(0.6f, flash));
	}

	/**
	 * Recolours the particles of a frozen (disabled) emitter after its colours changed, without
	 * moving or ageing them: a frozen emitter does not update its particles at all otherwise.
	 */
	static void refreshFrozen(ParticleEmitter e) {
		if (e.isEnabled())
			return;
		e.setEnabled(true);
		e.updateFromControl(0f);
		e.setEnabled(false);
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

	/**
	 * How much of an underwater effect at {@code pos} the camera sees, 0..1: it fades with distance
	 * through the water, and quickly with depth when the camera is above the surface.
	 */
	static float visibility(Camera cam, Vector3f pos) {
		float d = Math.max(0f, cam.getLocation().distance(pos) - VISIBLE_NEAR);
		float v = FastMath.exp(-d / VISIBLE_RANGE);
		float camY = cam.getLocation().y;
		if (camY > 0f && pos.y < 0f)
			v *= FastMath.exp(pos.y / SURFACE_FADE); // looking down into the water
		else if (camY < 0f && pos.y > 0f)
			v *= FastMath.exp(camY / SURFACE_FADE); // looking up through it
		return v;
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
		// Underwater the flash comes through blue-green water
		ColorRGBA c = cam.getLocation().y < 0f ? new ColorRGBA(0.85f, 0.95f, 1f, alpha)
				: new ColorRGBA(1f, 0.92f, 0.8f, alpha);
		screenFlash.getMaterial().setColor("Color", c);
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
		return EffectTextures.smooth(x);
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

	/** A blast that has been set off, seen from outside. */
	public interface Handle {
		/** Seconds since the detonation, counting only while not paused. */
		float age();

		/** Where it went off. */
		Vector3f position();
	}

	/** A particle emitter with the colours it was given, so they can be faded with visibility. */
	private record Faded(ParticleEmitter emitter, ColorRGBA start, ColorRGBA end) {
		void apply(float v) {
			emitter.setStartColor(new ColorRGBA(start.r, start.g, start.b, start.a * v));
			emitter.setEndColor(new ColorRGBA(end.r, end.g, end.b, end.a * v));
		}
	}

	/**
	 * One gas bubble of the cluster: where it sits relative to the blast, how big, how late, how it
	 * turns and where in its wobble it starts.
	 */
	private record GasBubble(Geometry geometry, Vector3f offset, float size, float delay, Vector3f axis, float spin,
			float phase) {
	}

	/** One blast and everything it puts in the scene. */
	private final class Blast implements Handle {
		final Vector3f pos;
		final ColorRGBA team;
		final float strength;
		final boolean primary; // secondary blasts skip the surface effects and the rising bubble column
		final List<Spatial> parts = new ArrayList<>();
		final List<Faded> faded = new ArrayList<>();
		final List<GasBubble> gas = new ArrayList<>();
		// The surface effects fade by where they are, at the surface, not by where the blast was
		final List<Faded> surfaceFaded = new ArrayList<>();
		final float flickerPhase = random.nextFloat() * FastMath.TWO_PI;
		// How deep it went off; whether it breaks the surface, and how violently (0..1); when the shock reaches the
		// surface, and when the gas does, bursting through or, from deep down, boiling up; and whether anything shows
		// at the surface at all
		final float depth, power, shockAt, heaveAt;
		final boolean breaks, surfaces;
		float age;
		boolean started, frozen, plumeBurst, surged;

		Geometry core, dome;
		Node flare, shock, foam, spall;
		PointLight light;
		ParticleEmitter sparks, fire, murk, debris, plume, smoke, spray, jets, surge;

		Blast(Vector3f pos, ColorRGBA team, float strength, boolean primary) {
			this.pos = pos;
			this.team = team;
			this.strength = strength;
			this.primary = primary;
			depth = Math.max(0f, -pos.y);
			breaks = depth < SURFACE_DEPTH * strength;
			power = breaks ? 1f - depth / (SURFACE_DEPTH * strength) : 0f;
			shockAt = depth / SOUND_SPEED;
			heaveAt = breaks ? 0.15f + 0.6f * (1f - power) : BOIL_DELAY + depth / BOIL_RISE;
			surfaces = primary && heaveAt < BOIL_LATEST;
		}

		/**
		 * How long its effects live, long enough for the boil of a deep blast to rise and settle.
		 */
		float lifetime() {
			return surfaces ? Math.max(LIFETIME, heaveAt + SURGE_DELAY + 9f) : LIFETIME;
		}

		@Override
		public float age() {
			return age;
		}

		@Override
		public Vector3f position() {
			return pos.clone();
		}

		void start() {
			started = true;
			float s = strength;

			// White-hot core: an additive sphere
			core = new Geometry("blastCore", new Sphere(20, 20, 1f));
			Material cm = new Material(assets, UNSHADED);
			cm.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
			cm.getAdditionalRenderState().setDepthWrite(false);
			core.setMaterial(cm);
			attach(core, EFFECTS);

			// Glare: a camera-facing flash sprite much larger than the core, glowing for the bloom filter; and the
			// shockwave, a ring racing outwards
			flare = billboard("blastFlare", "Effects/Explosion/flash.png", 2, true, RenderQueue.Bucket.Transparent);
			shock = billboard("blastShock", "Effects/Explosion/shockwave.png", 1, false, EFFECTS);

			// Gas bubbles: a main one and satellites round it, lit, smoky and lumpy, each turning slowly
			int count = primary ? SATELLITES + 1 : 3;
			for (int i = 0; i < count; i++) {
				var g = new Geometry("blastGas", new Sphere(20, 28, 1f));
				Material bm = new Material(assets, "Common/MatDefs/Light/Lighting.j3md");
				bm.setTexture("DiffuseMap", EffectTextures.smokySkin());
				bm.setBoolean("UseMaterialColors", true);
				bm.setColor("Specular", new ColorRGBA(0.6f, 0.6f, 0.6f, 1f));
				bm.setFloat("Shininess", 24f);
				bm.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
				bm.getAdditionalRenderState().setDepthWrite(false);
				g.setMaterial(bm);
				attach(g, EFFECTS);
				Vector3f offset = i == 0 ? Vector3f.ZERO.clone()
						: randomDirection().multLocal(0.55f + 0.35f * random.nextFloat()).multLocal(1f, 0.7f, 1f);
				float size = i == 0 ? 1f : 0.3f + 0.3f * random.nextFloat();
				float delay = i == 0 ? 0f : 0.04f + 0.18f * random.nextFloat();
				gas.add(new GasBubble(g, offset, size, delay, randomDirection(), 0.3f + 0.5f * random.nextFloat(),
						random.nextFloat() * FastMath.TWO_PI));
			}

			light = new PointLight(pos.clone(), ColorRGBA.Black, 300f * s);
			root.addLight(light);

			sparks = emitter("blastSparks", "Effects/Explosion/spark.png", (int) (90 * s), 1, 1, true, EFFECTS);
			sparks.setFacingVelocity(true);
			colours(sparks, new ColorRGBA(1f, 0.95f, 0.7f, 1f), new ColorRGBA(1f, 0.35f, 0.05f, 0f));
			sparks.setStartSize(2.2f * s);
			sparks.setEndSize(0.4f * s);
			sparks.setLowLife(0.35f);
			sparks.setHighLife(1.0f);
			sparks.setGravity(0, 6f, 0); // jME gravity points the other way: positive Y pulls down
			sparks.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 75f * s, 0));
			sparks.getParticleInfluencer().setVelocityVariation(1f);

			fire = emitter("blastFire", "Effects/Explosion/flame.png", (int) (40 * s), 2, 2, true, EFFECTS);
			fire.setRandomAngle(true);
			fire.setRotateSpeed(1.5f);
			colours(fire, new ColorRGBA(1f, 0.85f, 0.4f, 0.75f),
					new ColorRGBA(0.6f + 0.4f * team.r, 0.15f + 0.3f * team.g, 0.4f * team.b, 0f));
			fire.setStartSize(4f * s);
			fire.setEndSize(13f * s);
			fire.setLowLife(0.5f);
			fire.setHighLife(1.2f);
			fire.setGravity(0, -3f, 0); // hot gas rises
			fire.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 20f * s, 0));
			fire.getParticleInfluencer().setVelocityVariation(1f);

			// Murk: churned-up water and soot that hangs about after the flash
			murk = emitter("blastMurk", null, (int) (18 * s), 8, 1, false, EFFECTS);
			murk.getMaterial().setTexture("Texture", EffectTextures.puffs());
			murk.setSelectRandomImage(true);
			murk.setRandomAngle(true);
			murk.setRotateSpeed(0.3f);
			colours(murk, new ColorRGBA(0.14f, 0.15f, 0.15f, 0.22f), new ColorRGBA(0.1f, 0.15f, 0.2f, 0f));
			murk.setStartSize(8f * s);
			murk.setEndSize(30f * s);
			murk.setLowLife(3f);
			murk.setHighLife(5f);
			murk.setGravity(0, -0.5f, 0);
			murk.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 4f * s, 0));
			murk.getParticleInfluencer().setVelocityVariation(1f);

			debris = emitter("blastDebris", null, (int) (24 * s), 2, 2, false, EFFECTS);
			debris.getMaterial().setTexture("Texture", EffectTextures.debris());
			debris.setSelectRandomImage(true);
			debris.setRandomAngle(true);
			debris.setRotateSpeed(5f);
			colours(debris, new ColorRGBA(0.9f, 0.85f, 0.8f, 1f), new ColorRGBA(0.6f, 0.6f, 0.6f, 0f));
			debris.setStartSize(0.7f * s);
			debris.setEndSize(1.1f * s);
			debris.setLowLife(2f);
			debris.setHighLife(4f);
			debris.setGravity(0, 5f, 0); // debris sinks
			debris.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 35f * s, 0));
			debris.getParticleInfluencer().setVelocityVariation(0.9f);

			for (var e : new ParticleEmitter[] {sparks, fire, murk, debris})
				e.emitAllParticles();

			if (primary) {
				// Columns of bubbles from the broken-up gas, clean air and smoky, each living only as long as it takes
				// to reach the surface (see bubbleLife)

				plume = emitter("blastPlume", null, (int) (350 * s), 4, 1, false, EFFECTS);
				plume.getMaterial().setTexture("Texture", EffectTextures.bubbleSwarm());
				plume.setSelectRandomImage(true);
				plume.setShape(new EmitterSphereShape(Vector3f.ZERO, BUBBLE_RADIUS * 0.5f * s));
				colours(plume, new ColorRGBA(0.8f, 0.9f, 0.95f, 0.45f), new ColorRGBA(0.8f, 0.9f, 0.95f, 0f));
				plume.setStartSize(1.5f);
				plume.setEndSize(4f);
				plume.setGravity(0, -2f, 0); // buoyancy: bubbles speed up as they rise
				plume.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 6f, 0));
				plume.getParticleInfluencer().setVelocityVariation(0.4f);

				smoke = emitter("blastSmokyBubbles", null, (int) (140 * s), 4, 1, false, EFFECTS);
				// Big bubbles of detonation products: wobbly caps like the air bubbles, upright, tinted smoky
				smoke.getMaterial().setTexture("Texture", EffectTextures.bigBubbles());
				smoke.setSelectRandomImage(true);
				smoke.setShape(new EmitterSphereShape(Vector3f.ZERO, BUBBLE_RADIUS * 0.4f * s));
				colours(smoke, new ColorRGBA(0.5f, 0.47f, 0.43f, 0.8f), new ColorRGBA(0.62f, 0.64f, 0.66f, 0f));
				smoke.setStartSize(1.5f * s);
				smoke.setEndSize(4.5f * s);
				smoke.setGravity(0, -1.5f, 0);
				smoke.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 5f, 0));
				smoke.getParticleInfluencer().setVelocityVariation(0.35f);
				bubbleLife(pos);
			}
		}

		/**
		 * Has bubbles from now on live as long as it takes them to reach the surface from the top
		 * of the sphere round {@code centre} that the columns start from: rising d metres at 6 m/s,
		 * accelerating at 2 m/s^2, takes t where d = 6t + t^2.
		 */
		private void bubbleLife(Vector3f centre) {
			float depth = Math.max(1f, -centre.y - BUBBLE_RADIUS * 0.5f * strength);
			float toSurface = FastMath.clamp((-6f + FastMath.sqrt(36f + 4f * depth)) / 2f, 0.3f, 8f);
			plume.setHighLife(toSurface);
			plume.setLowLife(toSurface * 0.5f);
			smoke.setHighLife(toSurface);
			smoke.setLowLife(toSurface * 0.6f);
		}

		/** Sets everything for the current {@link #age}. */
		void animate() {
			float s = strength, t = age;
			float vis = visibility(cam, pos);
			for (var f : faded)
				f.apply(vis);

			// Gas bubbles: the cluster's centre starts drifting up once it breaks up, but stays under water, as the
			// bubble columns rising from it must start there
			float rise = t > BREAKUP_TIME ? 3f * (t - BREAKUP_TIME) * (t - BREAKUP_TIME) : 0f;
			Vector3f centre = pos.add(0, Math.min(rise, Math.min(pos.y, -BUBBLE_RADIUS * 0.5f * s) - pos.y), 0);
			float gasA = 0.55f * vis * (1f - smooth((t - BREAKUP_TIME) / (FADE_TIME - BREAKUP_TIME)));
			float mainR = BUBBLE_RADIUS * s;
			for (var b : gas) {
				var g = b.geometry();
				show(g, gasA > 0.01f);
				if (gasA <= 0.01f)
					continue;
				float r = mainR * b.size() * keyframes(t - b.delay(), BUBBLE_T, BUBBLE_R);
				// Satellites sit on the main bubble's surface as it swells and shrinks
				float reach = mainR * keyframes(t, BUBBLE_T, BUBBLE_R);
				g.setLocalTranslation(centre.add(b.offset().mult(reach)));
				// They wobble out of round, a little at first and more once the collapse has unsettled them, keeping
				// roughly the same volume
				float wob = 0.06f + 0.16f * smooth((t - 0.7f) / 0.6f);
				float w1 = wob * FastMath.sin(7.3f * t + b.phase()), w2 = wob * FastMath.sin(9.1f * t + 2f * b.phase());
				g.setLocalScale(r * (1f + w1), r * (1f - 0.5f * (w1 + w2)), r * (1f + w2));
				g.setLocalRotation(new Quaternion().fromAngleAxis(b.spin() * t, b.axis()));
				// Inner fire lights the gas early on, then it goes grey and smoky
				float glow = FastMath.exp(-t * 4f) + 0.6f * pulse(t);
				ColorRGBA tint = GAS.clone().interpolateLocal(team, 0.12f);
				ColorRGBA lit = new ColorRGBA(tint.r + 1.2f * glow, tint.g + 0.7f * glow, tint.b + 0.3f * glow, 1f);
				g.getMaterial().setColor("Diffuse", scaled(tint, 1f, gasA));
				g.getMaterial().setColor("Ambient", scaled(lit, 0.8f, gasA));
			}

			// Core: the main flash and fireball, then a smaller re-flash when the bubble collapses
			float main = t < 0.02f ? t / 0.02f : FastMath.exp(-(t - 0.02f) * 5f);
			float collapse = 0.8f * pulse(t);
			float coreA = Math.max(main, collapse) * Math.max(vis, 0.3f);
			show(core, coreA > 0.01f);
			if (coreA > 0.01f) {
				boolean fireball = main >= collapse;
				ColorRGBA c = fireball ? HOT.clone().interpolateLocal(FIRE, smooth(t / 0.35f)) : PULSE;
				core.setLocalTranslation(centre);
				core.setLocalScale(s * (fireball ? 5f + 20f * smooth(t / 0.5f) : 5f + 5f * collapse));
				core.getMaterial().setColor("Color", scaled(c, 1f, Math.min(1f, coreA)));
			}

			// Glare: biggest in the first instant, shrinking as it fades; back briefly at the collapse
			float early = t < 0.35f ? FastMath.pow(1f - t / 0.35f, 1.5f) : 0f;
			float flareA = Math.max(early, 0.6f * pulse(t)) * Math.max(vis, 0.3f);
			show(flare, flareA > 0.01f);
			if (flareA > 0.01f) {
				flare.setLocalTranslation(centre);
				flare.setLocalScale(s * (t < 0.35f ? 120f - 60f * t / 0.35f : 50f));
				setBillboardColor(flare, new ColorRGBA(1.5f, 1.3f, 1.0f, flareA));
			}

			// Shockwave ring. Seen from above the water, the billboard would stand up out of it into the sky: there the
			// shock shows where it reaches the surface instead.
			float u = cam.getLocation().y > 0f ? 1f : t / SHOCK_TIME;
			show(shock, u < 1f);
			if (u < 1f) {
				shock.setLocalTranslation(pos);
				shock.setLocalScale(s * SHOCK_SIZE * (0.05f + 0.95f * easeOut(u)));
				setBillboardColor(shock, new ColorRGBA(0.8f, 0.9f, 1f, 0.9f * vis * (1f - u) * (1f - u)));
			}

			// Light: what the flash and fireball throw on everything around
			if (light != null) {
				if (t > FADE_TIME) {
					root.removeLight(light);
					light = null;
				} else {
					// The fireball flickers as it burns down
					float flicker = 1f + 0.35f * FastMath.exp(-t * 2f) * FastMath.sin(t * 61f + flickerPhase)
							* FastMath.sin(t * 37f);
					float k = s * (4f * flashCurve(t) + 0.5f * FastMath.exp(-t * 1.5f) * flicker);
					light.setColor(LIGHT.clone().interpolateLocal(team, 0.15f).multLocal(k));
					light.setPosition(centre);
				}
			}

			if (plume != null) {
				plume.setLocalTranslation(centre);
				bubbleLife(centre);
				smoke.setLocalTranslation(centre);
				if (t >= BREAKUP_TIME && !plumeBurst) {
					plumeBurst = true;
					plume.emitParticles((int) (60 * s));
					smoke.emitParticles((int) (20 * s));
				}
				float k = t < BREAKUP_TIME ? 0f : 1f - smooth((t - BREAKUP_TIME) / 3.5f);
				plume.setParticlesPerSec(90f * s * k);
				smoke.setParticlesPerSec(22f * s * k);
			}

			if (surfaces) {
				// The shock reaches the surface first: it flashes white in a ring racing outwards and, unless the blast
				// is deep, heaves up in a dome of spray. A moment later, the shallower the sooner and the higher, the gas
				// breaks through in a column of spray and jets (or, from deep down, boils up seconds later), and as that
				// falls back, a ring of mist rolls out across the water.
				if (spall == null && t >= shockAt)
					startSurface();
				if (spray == null && t >= heaveAt)
					startPlume();
				if (surge != null && !surged && t >= heaveAt + SURGE_DELAY) {
					surged = true;
					emitSurge();
				}
				// From under the water, what goes on above it shows only dimly, through the surface
				float sv = visibility(cam, new Vector3f(pos.x, 0f, pos.z)) * (cam.getLocation().y < 0f ? 0.15f : 1f);
				for (var f : surfaceFaded)
					f.apply(sv);
				if (dome != null) {
					// It swells up and out, then slumps and thins as the column bursts through it
					float swell = easeOut((t - shockAt) / DOME_TIME);
					float left = 1f - smooth((t - heaveAt) / 1.4f);
					show(dome, left > 0.01f);
					float r = s * (8f + 30f * power) * (0.3f + 0.7f * swell) * (1.15f - 0.15f * left);
					dome.setLocalScale(r, r * (0.15f + 0.65f * swell) * (0.4f + 0.6f * left), r);
					float a = left * sv;
					dome.getMaterial().setColor("Diffuse", new ColorRGBA(1.1f, 1.12f, 1.15f, a));
					dome.getMaterial().setColor("Ambient", new ColorRGBA(1.9f, 1.95f, 2f, a));
				}
				if (spall != null) {
					float f = (t - shockAt) / SPALL_TIME;
					show(spall, f < 1f);
					if (f < 1f) {
						spall.setLocalScale(s * SPALL_SIZE * (0.5f + 0.5f * power) * (0.03f + 0.97f * easeOut(f)));
						float a = (0.8f + 0.2f * power) * FastMath.exp(-depth / (SPALL_FADE * s)) * (1f - f);
						((Geometry) spall.getChild(0)).getMaterial().setColor("Color",
								new ColorRGBA(1.6f, 1.6f, 1.6f, a * sv));
					}
				}
				if (foam != null) {
					float f = (t - heaveAt) / 6f;
					show(foam, f < 1f);
					foam.setLocalScale(s * (40f + 120f * easeOut(f)));
					((Geometry) foam.getChild(0)).getMaterial().setColor("Color",
							new ColorRGBA(0.95f, 0.98f, 1f, 0.8f * (1f - f) * sv));
				}
			}
		}

		/**
		 * The flash ring where the shock reaches the surface and, if the blast breaks it, the dome
		 * of spray.
		 */
		private void startSurface() {
			spall = flatRing("blastSpall", 2.2f);
			if (!breaks)
				return;
			// A hemisphere of white water, lit by the sun, mottled and frothy
			dome = new Geometry("blastDome", new Dome(Vector3f.ZERO, 12, 32, 1f, true));
			Material m = new Material(assets, "Common/MatDefs/Light/Lighting.j3md");
			m.setTexture("DiffuseMap", EffectTextures.froth());
			m.setBoolean("UseMaterialColors", true);
			m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
			m.getAdditionalRenderState().setDepthWrite(false);
			dome.setMaterial(m);
			dome.setLocalTranslation(pos.x, 0f, pos.z);
			attach(dome, EFFECTS);
		}

		/**
		 * Spray column, jets, mist and foam ring where the gas breaks the surface: tall and violent
		 * for a shallow blast, a low boil for a deep one.
		 */
		private void startPlume() {
			float s = strength;
			spray = emitter("blastSpray", null, (int) ((breaks ? 180 : 90) * s), 8, 1, false, EFFECTS);
			spray.getMaterial().setTexture("Texture", EffectTextures.puffs());
			spray.setSelectRandomImage(true);
			spray.setRandomAngle(true);
			spray.setRotateSpeed(0.5f);
			// A column from a blast that breaks the surface; from deep down, the gas only heaves up a low, wide
			// mound of churning water. Either is spawned clear of the water: the effects draw over it, so any spray
			// below would show on top of it.
			if (breaks) {
				spray.setShape(new EmitterSphereShape(Vector3f.ZERO, 5f * s));
				spray.setLocalTranslation(pos.x, 1f + 5f * s, pos.z);
			} else {
				spray.setShape(
						new EmitterBoxShape(new Vector3f(-14f * s, 0f, -14f * s), new Vector3f(14f * s, 2f, 14f * s)));
				spray.setLocalTranslation(pos.x, 1f, pos.z);
			}
			surfaceColours(spray, new ColorRGBA(0.92f, 0.96f, 1f, 0.95f), new ColorRGBA(0.8f, 0.85f, 0.9f, 0f));
			spray.setStartSize((breaks ? 5f : 4f) * s);
			spray.setEndSize((breaks ? 18f : 11f) * s);
			spray.setLowLife(3.5f);
			spray.setHighLife(6f);
			spray.setGravity(0, 9.8f, 0);
			spray.getParticleInfluencer().setVelocityVariation(0.12f);
			// Launch it in waves of falling speed so it rises as a column rather than a single puff
			float top = (breaks ? 12f + 33f * power : 5f) * s;
			int waves = 6, perWave = spray.getMaxNumParticles() / waves;
			for (int i = 0; i < waves; i++) {
				spray.getParticleInfluencer().setInitialVelocity(new Vector3f(0, top * (1f - 0.12f * i), 0));
				spray.emitParticles(perWave);
			}

			// Jets of water thrown out of the column at all angles, fanning out into a crown as they fall back
			jets = emitter("blastJets", null, (int) ((breaks ? 56 : 24) * s), 1, 1, false, EFFECTS);
			jets.getMaterial().setTexture("Texture", EffectTextures.streak());
			jets.setFacingVelocity(true);
			jets.setShape(new EmitterSphereShape(Vector3f.ZERO, 3f * s));
			surfaceColours(jets, new ColorRGBA(1f, 1f, 1f, 1f), new ColorRGBA(0.88f, 0.92f, 0.96f, 0f));
			jets.setStartSize(5f * s);
			jets.setEndSize(14f * s);
			jets.setLowLife(2.5f);
			jets.setHighLife(4.5f);
			jets.setGravity(0, 9.8f, 0);
			jets.getParticleInfluencer().setVelocityVariation(0f);
			jets.setLocalTranslation(pos.x, 1f + 3f * s, pos.z);
			for (int i = 0; i < jets.getMaxNumParticles(); i++) {
				float tilt = 0.12f + 0.6f * random.nextFloat(), heading = random.nextFloat() * FastMath.TWO_PI;
				float speed = top * (0.6f + 0.55f * random.nextFloat());
				jets.getParticleInfluencer()
						.setInitialVelocity(new Vector3f(FastMath.sin(tilt) * FastMath.cos(heading) * speed,
								FastMath.cos(tilt) * speed, FastMath.sin(tilt) * FastMath.sin(heading) * speed));
				jets.emitParticles(1);
			}

			// Mist rolling out low over the water once the column falls back (see emitSurge)
			surge = emitter("blastSurge", null, (int) (48 * s), 8, 1, false, EFFECTS);
			surge.getMaterial().setTexture("Texture", EffectTextures.puffs());
			surge.setSelectRandomImage(true);
			surge.setRandomAngle(true);
			surge.setRotateSpeed(0.15f);
			surge.setShape(new EmitterBoxShape(new Vector3f(-6f * s, 0f, -6f * s), new Vector3f(6f * s, 2f, 6f * s)));
			surfaceColours(surge, new ColorRGBA(0.93f, 0.95f, 0.98f, 0.45f), new ColorRGBA(0.85f, 0.89f, 0.93f, 0f));
			surge.setStartSize((breaks ? 12f : 6f) * s);
			surge.setEndSize((breaks ? 40f : 20f) * s);
			surge.setLowLife(6f);
			surge.setHighLife(9f);
			surge.getParticleInfluencer().setVelocityVariation(0.1f);
			// Low over the water, which its puffs, as they grow, lie on rather than rise from
			surge.setLocalTranslation(pos.x, 1f + 0.4f * surge.getStartSize(), pos.z);

			// Foam ring lying flat on the water, centred over the blast
			foam = flatRing("blastFoam", 1.8f);
		}

		/** Sends the mist out from the foot of the column in all directions across the water. */
		private void emitSurge() {
			float speed = strength * (6f + 8f * power);
			int n = surge.getMaxNumParticles();
			for (int i = 0; i < n; i++) {
				float heading = FastMath.TWO_PI * (i + random.nextFloat()) / n;
				float v = speed * (0.7f + 0.6f * random.nextFloat());
				surge.getParticleInfluencer()
						.setInitialVelocity(new Vector3f(FastMath.cos(heading) * v, 0.4f, FastMath.sin(heading) * v));
				surge.emitParticles(1);
			}
		}

		/** A ring lying flat on the water {@code height} above it, centred over the blast. */
		private Node flatRing(String name, float height) {
			Geometry ring = new Geometry(name, new Quad(1, 1));
			Material m = new Material(assets, UNSHADED);
			m.setTexture("ColorMap", assets.loadTexture("Effects/Explosion/shockwave.png"));
			m.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.AlphaAdditive);
			m.getAdditionalRenderState().setDepthWrite(false);
			m.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
			ring.setMaterial(m);
			ring.setQueueBucket(EFFECTS);
			ring.setLocalTranslation(-0.5f, -0.5f, 0f);
			var n = new Node(name + "Node");
			n.attachChild(ring);
			n.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, 0, 0));
			n.setLocalTranslation(pos.x, height, pos.z);
			attach(n, EFFECTS);
			return n;
		}

		float screenFlash() {
			float d = cam.getLocation().distance(pos) / 300f;
			Vector3f screen = cam.getScreenCoordinates(pos);
			boolean inView = screen.z > 0f && screen.z < 1f && screen.x >= 0 && screen.y >= 0
					&& screen.x <= cam.getWidth() && screen.y <= cam.getHeight();
			return 0.3f * strength * flashCurve(age) * (inView ? 1f : 0.3f) / (1f + d * d);
		}

		void setFrozen(boolean frozen) {
			if (this.frozen == frozen)
				return;
			this.frozen = frozen;
			for (var e : new ParticleEmitter[] {sparks, fire, murk, debris, plume, smoke, spray, jets, surge})
				if (e != null)
					e.setEnabled(!frozen);
		}

		void dispose() {
			for (var p : parts)
				p.removeFromParent();
			if (light != null)
				root.removeLight(light);
		}

		private Vector3f randomDirection() {
			return new Vector3f(random.nextFloat() - 0.5f, random.nextFloat() - 0.5f, random.nextFloat() - 0.5f)
					.normalizeLocal();
		}

		private void attach(Spatial s, RenderQueue.Bucket bucket) {
			if (s instanceof Geometry)
				s.setQueueBucket(bucket);
			s.setCullHint(Spatial.CullHint.Always);
			root.attachChild(s);
			parts.add(s);
		}

		/**
		 * A unit quad centred on a node that always faces the camera, showing the first of
		 * {@code cells} by {@code cells} images in {@code texture}.
		 */
		private Node billboard(String name, String texture, int cells, boolean glow, RenderQueue.Bucket bucket) {
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
			quad.setQueueBucket(bucket);
			quad.setLocalTranslation(-0.5f, -0.5f, 0f);
			Node n = new Node(name + "Node");
			n.attachChild(quad);
			n.addControl(new BillboardControl());
			attach(n, bucket);
			return n;
		}

		private void setBillboardColor(Node n, ColorRGBA c) {
			Material m = ((Geometry) n.getChild(0)).getMaterial();
			m.setColor("Color", c);
			if (m.getParam("GlowMap") != null)
				m.setColor("GlowColor", new ColorRGBA(c.r * c.a, c.g * c.a, c.b * c.a, 1f));
		}

		/** Gives an underwater emitter its colours, to be faded with visibility from then on. */
		private void colours(ParticleEmitter e, ColorRGBA start, ColorRGBA end) {
			e.setStartColor(start);
			e.setEndColor(end);
			faded.add(new Faded(e, start, end));
		}

		/** Gives a surface emitter its colours, to be faded with the visibility of the surface. */
		private void surfaceColours(ParticleEmitter e, ColorRGBA start, ColorRGBA end) {
			e.setStartColor(start);
			e.setEndColor(end);
			surfaceFaded.add(new Faded(e, start, end));
		}

		private ParticleEmitter emitter(
			String name, String texture, int count, int imagesX, int imagesY, boolean additive,
			RenderQueue.Bucket bucket) {
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
			e.setQueueBucket(bucket);
			root.attachChild(e);
			parts.add(e);
			return e;
		}
	}
}
