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

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

import com.jme3.app.state.AppStateManager;
import com.jme3.app.state.ScreenshotAppState;
import com.jme3.font.BitmapText;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;

import se.hirt.searobots.engine.SimClock;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

/**
 * Scripted screenshots of torpedo detonations in the running viewer, for looking at the explosion
 * effects in the real scene (water, fog, terrain, bloom). Runs the match fast until a torpedo is in
 * the water, slower while one is, and when a detonation comes, follows it: at each requested time
 * after the detonation it freezes the simulation and the explosion, takes a screenshot from each
 * requested camera, and resumes. After the requested number of detonations it quits.
 * <p>
 * Enabled by {@code -Dsearobots.capture.dir=DIR}; the other settings, all optional:
 * <ul>
 * <li>{@code searobots.capture.ages}: seconds after the detonation, comma separated (default
 * {@code 0.03,0.15,0.4,0.8,1.0,1.5,2.5,4})
 * <li>{@code searobots.capture.cams}: cameras, semicolon separated, each either {@code view} (the
 * viewer's own camera mode, as it would be) or {@code distance,azimuth,elevation[,lookUp]}: metres
 * from the blast, compass bearing from the blast to the camera in degrees (0 north, clockwise),
 * degrees above (negative: below) the blast, and how far above the blast to aim (default
 * {@code 160,30,5;350,120,20;view})
 * <li>{@code searobots.capture.target}: {@code blast} to place fixed cameras round the blast, or
 * {@code victim} to place them round the submarine nearest it and follow that as it sinks (default
 * {@code blast})
 * <li>{@code searobots.capture.skip}: detonations to let pass before capturing (default 0)
 * <li>{@code searobots.capture.count}: detonations to capture (default 1)
 * <li>{@code searobots.capture.hitsOnly}: only capture detonations that hit a submarine (default
 * false)
 * <li>{@code searobots.capture.fast}, {@code searobots.capture.approach}: simulation speed with no
 * torpedo in the water, and with one (defaults 40 and 6; much faster and a detonation can fall
 * between two frames)
 * <li>{@code searobots.capture.hud}: keep the HUD text in the shots (default false)
 * </ul>
 * Shots are saved as {@code blast<n>-t<age>-cam<k>.png}, listed with the actual blast age in
 * {@code capture.txt}.
 */
final class ExplosionCapture {
	private static final String PREFIX = "searobots.capture.";
	private static final int SETTLE_FRAMES = 2; // frames between moving the camera and taking the shot
	private static final int WARMUP_FRAMES = 60; // frames to let the loaded scene settle before starting the match

	private enum State {
		WAITING, FOLLOWING, SHOOTING, DONE
	}

	private final Path dir;
	private final float[] ages;
	private final List<float[]> cams = new ArrayList<>(); // null entry: the viewer's own camera
	private final int skip, count;
	private final boolean hitsOnly, hud, followVictim;
	private final double fastSpeed, approachSpeed;
	private final ScreenshotAppState shots;
	private final Node gui;
	private final Supplier<SimClock> sim;
	private final Supplier<List<TorpedoSnapshot>> torpedoes;
	private final Supplier<List<SubmarineSnapshot>> submarines;
	private final Runnable quit;
	private final PrintWriter log;

	private State state = State.WAITING;
	private ExplosionEffects.Handle blast;
	private int seen, captured, ageIndex, camIndex, settle, victim = -1;
	private int warmup = WARMUP_FRAMES;
	private float elapsed, lastAge;
	private java.util.function.BooleanSupplier sceneReady = () -> true;

	/** Holds the match paused until {@code ready} says the scene is loaded, and a little after. */
	void waitFor(java.util.function.BooleanSupplier ready) {
		this.sceneReady = ready;
	}

	/** Returns a capture set up from the system properties, or null if capture is not enabled. */
	static ExplosionCapture fromSystemProperties(
		AppStateManager states, Node gui, Supplier<SimClock> sim, Supplier<List<TorpedoSnapshot>> torpedoes,
		Supplier<List<SubmarineSnapshot>> submarines, Runnable quit) {
		String dir = System.getProperty(PREFIX + "dir");
		if (dir == null || dir.isBlank())
			return null;
		try {
			return new ExplosionCapture(Path.of(dir), states, gui, sim, torpedoes, submarines, quit);
		} catch (IOException e) {
			System.err.println("Explosion capture disabled, cannot write to " + dir + ": " + e);
			return null;
		}
	}

	private ExplosionCapture(Path dir, AppStateManager states, Node gui, Supplier<SimClock> sim,
			Supplier<List<TorpedoSnapshot>> torpedoes, Supplier<List<SubmarineSnapshot>> submarines, Runnable quit)
			throws IOException {
		this.dir = dir;
		this.gui = gui;
		this.sim = sim;
		this.torpedoes = torpedoes;
		this.submarines = submarines;
		this.quit = quit;
		String[] a = System.getProperty(PREFIX + "ages", "0.03,0.15,0.4,0.8,1.0,1.5,2.5,4").split(",");
		ages = new float[a.length];
		for (int i = 0; i < a.length; i++)
			ages[i] = Float.parseFloat(a[i].trim());
		java.util.Arrays.sort(ages);
		for (String c : System.getProperty(PREFIX + "cams", "160,30,5;350,120,20;view").split(";")) {
			c = c.trim();
			if (c.equals("view")) {
				cams.add(null);
				continue;
			}
			String[] v = c.split(",");
			float[] cam = new float[4];
			for (int i = 0; i < v.length && i < 4; i++)
				cam[i] = Float.parseFloat(v[i].trim());
			cams.add(cam);
		}
		skip = Integer.getInteger(PREFIX + "skip", 0);
		count = Integer.getInteger(PREFIX + "count", 1);
		hitsOnly = Boolean.getBoolean(PREFIX + "hitsOnly");
		hud = Boolean.getBoolean(PREFIX + "hud");
		followVictim = System.getProperty(PREFIX + "target", "blast").equals("victim");
		fastSpeed = Double.parseDouble(System.getProperty(PREFIX + "fast", "40"));
		approachSpeed = Double.parseDouble(System.getProperty(PREFIX + "approach", "6"));

		Files.createDirectories(dir);
		log = new PrintWriter(Files.newBufferedWriter(dir.resolve("capture.txt")), true);
		shots = new ScreenshotAppState(dir.toAbsolutePath() + java.io.File.separator);
		shots.setIsNumbered(false);
		states.attach(shots);
		System.out.println("Explosion capture: " + ages.length + " moments x " + cams.size() + " cameras, into "
				+ dir.toAbsolutePath());
	}

	/** A detonation has been set off in the scene. */
	void onDetonation(ExplosionEffects.Handle handle, boolean submarineHit) {
		if (state != State.WAITING || (hitsOnly && !submarineHit))
			return;
		if (seen++ < skip)
			return;
		blast = handle;
		ageIndex = 0;
		elapsed = lastAge = 0;
		victim = followVictim ? nearestSubmarine(blast.position()) : -1;
		state = State.FOLLOWING;
		var clock = sim.get();
		if (clock != null)
			clock.setSpeedMultiplier(1.0);
		log.printf(Locale.ROOT, "# blast %d at %s%s%n", captured + 1, blast.position(),
				submarineHit ? " (submarine hit)" : "");
	}

	/**
	 * Seconds since the detonation being followed: the blast's own age while it lasts, so shots
	 * match the effects exactly, and counted on from there once it has been cleared away.
	 */
	private float elapsed(float tpf) {
		float age = blast.age();
		if (age > lastAge) {
			lastAge = age;
			elapsed = age;
		} else {
			elapsed += Math.min(tpf, ExplosionEffects.MAX_STEP);
		}
		return elapsed;
	}

	/** True while a moment is being shot: the simulation and the explosions should hold still. */
	boolean freezing() {
		return state == State.SHOOTING;
	}

	/** Advances the script; call once a frame, after the scene has been updated. */
	void update(float tpf) {
		var clock = sim.get();
		switch (state) {
		case WAITING -> {
			// Hold the match until the scene is loaded and running smoothly: what happens while the viewer is
			// stalled (building the terrain, say) all lands in one late frame
			if (warmup > 0) {
				if (sceneReady.getAsBoolean())
					warmup--;
				if (clock != null)
					clock.setPaused(warmup > 0);
				break;
			}
			if (clock != null) {
				boolean torpedoInWater = torpedoes.get().stream().anyMatch(TorpedoSnapshot::alive);
				clock.setSpeedMultiplier(torpedoInWater ? approachSpeed : fastSpeed);
			}
		}
		case FOLLOWING -> {
			if (elapsed(tpf) >= ages[ageIndex]) {
				state = State.SHOOTING;
				camIndex = 0;
				settle = SETTLE_FRAMES;
				if (clock != null)
					clock.setPaused(true);
			}
		}
		case SHOOTING -> {
			if (settle-- > 0)
				break;
			String name = String.format(Locale.ROOT, "blast%d-t%.2f-cam%d", captured + 1, ages[ageIndex], camIndex + 1);
			shots.setFileName(name);
			shots.takeScreenshot();
			log.printf(Locale.ROOT, "%s.png\tage=%.3f\tcam=%s%n", name, elapsed, describe(cams.get(camIndex)));
			settle = SETTLE_FRAMES;
			if (++camIndex < cams.size())
				break;
			if (++ageIndex < ages.length) {
				state = State.FOLLOWING;
			} else if (++captured < count) {
				state = State.WAITING;
			} else {
				state = State.DONE;
				settle = SETTLE_FRAMES + 2; // let the last shot be written
			}
			if (clock != null)
				clock.setPaused(false);
		}
		case DONE -> {
			if (settle-- <= 0) {
				log.close();
				System.out.println("Explosion capture done: " + dir.toAbsolutePath());
				quit.run();
			}
		}
		}
		if (!hud)
			for (Spatial s : gui.getChildren())
				if (s instanceof BitmapText)
					s.setCullHint(Spatial.CullHint.Always);
	}

	/**
	 * While following a blast with a fixed camera, sets {@code camPos} and {@code lookAt} for it
	 * and returns true; otherwise leaves them to the viewer's camera mode.
	 */
	boolean overrideCamera(Vector3f camPos, Vector3f lookAt) {
		if (state != State.FOLLOWING && state != State.SHOOTING)
			return false;
		// While following, hold the camera of the next shot so the scene settles from it
		float[] cam = cams.get(state == State.SHOOTING ? camIndex : 0);
		if (cam == null)
			return false;
		Vector3f p = focus();
		float az = cam[1] * FastMath.DEG_TO_RAD, el = cam[2] * FastMath.DEG_TO_RAD;
		// jME: north is -Z, east is +X
		camPos.set(p.x + cam[0] * FastMath.cos(el) * FastMath.sin(az), p.y + cam[0] * FastMath.sin(el),
				p.z - cam[0] * FastMath.cos(el) * FastMath.cos(az));
		lookAt.set(p.x, p.y + cam[3], p.z);
		return true;
	}

	/** Where fixed cameras look: the blast, or the submarine being followed, where it is now. */
	private Vector3f focus() {
		if (victim >= 0)
			for (var s : submarines.get())
				if (s.id() == victim) {
					var p = s.pose().position();
					return new Vector3f((float) p.x(), (float) p.z(), (float) -p.y());
				}
		return blast.position();
	}

	/** The id of the submarine nearest {@code at} (jME coordinates), or -1 if there are none. */
	private int nearestSubmarine(Vector3f at) {
		int best = -1;
		double bestDist = Double.MAX_VALUE;
		for (var s : submarines.get()) {
			var p = s.pose().position();
			double dx = p.x() - at.x, dy = p.y() + at.z, dz = p.z() - at.y;
			double d = dx * dx + dy * dy + dz * dz;
			if (d < bestDist) {
				bestDist = d;
				best = s.id();
			}
		}
		return best;
	}

	private static String describe(float[] cam) {
		return cam == null ? "view"
				: String.format(Locale.ROOT, "dist %.0f az %.0f el %.0f lookUp %.0f", cam[0], cam[1], cam[2], cam[3]);
	}
}
