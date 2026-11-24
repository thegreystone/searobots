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
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalTime;
import java.util.Locale;
import java.util.concurrent.locks.LockSupport;

import com.jme3.math.Vector3f;
import com.jme3.system.Timer;

import se.hirt.searobots.engine.SimulationLoop;
import se.hirt.searobots.viewer.CinematicDirector.ShotType;
import se.hirt.searobots.viewer.CinematicDirector.TransitionType;
import se.hirt.searobots.viewer.showreel.FrameRecorder;
import se.hirt.searobots.viewer.showreel.ShowreelOverlay;
import se.hirt.searobots.viewer.showreel.ShowreelPlan;
import se.hirt.searobots.viewer.showreel.ShowreelPlan.Beat;
import se.hirt.searobots.viewer.showreel.ShowreelScout;

/**
 * Makes the showreel: a short, cinematic film of a match, rendered offline from a replay. Finds a
 * match worth filming ({@link ShowreelScout}), plans the edit ({@link ShowreelPlan}), then plays
 * the replay in the viewer one video frame at a time: each frame advances the simulation by exactly
 * the planned amount, the Cinematic Director frames the planned shot, and the finished frame goes
 * to ffmpeg. The result does not depend on how fast the machine renders.
 * <p>
 * Run {@code Showreel --help} for the options.
 */
final class Showreel {
	private static final String URL = "github.com/thegreystone/searobots";
	/** Frames to let the scene settle before the first recorded frame. */
	private static final int WARMUP_MIN = 90, WARMUP_MAX = 900;
	/** At most this many extra renders per frame while scenery streams in near the camera. */
	private static final int MAX_HOLD = 12;

	private final ShowreelPlan plan;
	private final Path output;
	private final int fps, crf;
	private final String ffmpeg;
	private final LocalTime timeOfDay;
	private final String title, tagline, url;
	private final PrintStream log;

	private enum Phase {
		LOADING, WARMUP, RECORDING, FINISHED
	}

	private SubmarineScene3D scene;
	private ShowreelOverlay overlay;
	private FrameRecorder recorder;
	private CinematicDirector director;
	private FixedTimer timer;
	private Phase phase = Phase.LOADING;
	private int frame, totalFrames, warmup, settled, holds;
	private int beatIndex = -1;
	private double beatStart;
	private boolean cuePending;
	private long started;

	Showreel(ShowreelPlan plan, Path output, int fps, int crf, String ffmpeg, LocalTime timeOfDay, PrintStream log) {
		this(plan, output, fps, crf, ffmpeg, timeOfDay, ShowreelOverlay.DEFAULT_TITLE, ShowreelOverlay.DEFAULT_TAGLINE,
				URL, log);
	}

	Showreel(ShowreelPlan plan, Path output, int fps, int crf, String ffmpeg, LocalTime timeOfDay, String title,
			String tagline, String url, PrintStream log) {
		this.plan = plan;
		this.output = output;
		this.fps = fps;
		this.crf = crf;
		this.ffmpeg = ffmpeg;
		this.timeOfDay = timeOfDay;
		this.title = title;
		this.tagline = tagline;
		this.url = url;
		this.log = log;
		this.totalFrames = (int) Math.round(plan.length() * fps);
	}

	LocalTime timeOfDay() {
		return timeOfDay;
	}

	// ── Called by SubmarineScene3D, on the render thread ──

	/** Before the vehicles are updated: advances the simulation to this frame's moment. */
	void beforeVehicles() {
		switch (phase) {
		case LOADING -> {
			if (!scene.showreelSceneReady())
				return;
			startWarmup();
		}
		case WARMUP -> {
			warmup++;
			settled = scene.sceneryBusy() ? 0 : settled + 1;
			if (warmup >= WARMUP_MAX || (warmup >= WARMUP_MIN && settled >= 20))
				startRecording();
		}
		case RECORDING -> {
			double time = frame / (double) fps;
			int index = beatAt(time);
			boolean jumped = false;
			if (index != beatIndex) {
				beatIndex = index;
				beatStart = startOf(index);
				jumped = enterBeat(plan.beats().get(index));
			}
			var beat = plan.beats().get(beatIndex);
			double local = time - beatStart;
			double exact = beat.fromTick() + beat.simSecondsAt(local) * plan.tickRateHz();
			advanceTo(Math.round(exact));
			scene.showreelTickTime(exact);
			director.setFilmProgress((float) Math.min(1, local / beat.seconds()));
			// Smooth sub-tick motion at real time and slower; at time-lapse speed follow the simulation
			// exactly, and after a cut that skips ahead, put the hulls where they are instead of gliding
			scene.showreelVehicleLerp(jumped || beat.speedAt(local) > 2.5 ? 1f : -1f);
		}
		case FINISHED -> {
		}
		}
	}

	/**
	 * After the vehicles, before the camera: cues a new beat's shot, now that the hulls are in
	 * place.
	 */
	void beforeCamera() {
		if (!cuePending)
			return;
		cuePending = false;
		cue(plan.beats().get(beatIndex));
	}

	/** At the end of the frame's update: overlay, HUD, capture, and the next frame's time step. */
	void afterUpdate() {
		scene.hideForShowreel(overlay != null ? overlay.root() : null);
		if (phase != Phase.RECORDING || beatIndex < 0)
			return; // recording starts with the next frame
		if (frame >= totalFrames) {
			finish(); // the last frame was grabbed when the previous update was rendered
			return;
		}
		var beat = plan.beats().get(beatIndex);
		double time = frame / (double) fps;
		updateOverlay(beat, time - beatStart, time);

		boolean aerial = beat.shot() == ShowreelPlan.Shot.AERIAL || beat.shot() == ShowreelPlan.Shot.OUTRO
				|| scene.showreelCamera().getLocation().y > 0;
		if (aerial && scene.sceneryBusy() && holds < MAX_HOLD) {
			holds++; // render again, unrecorded and frozen, while trees stream in
			timer.next = 0;
			return;
		}
		holds = 0;
		recorder.captureNextFrame();
		frame++;
		if (frame % (fps * 5) == 0 && !healthy())
			return;
		if (frame >= totalFrames) {
			timer.next = 0; // one more update, frozen, to finish once this frame is grabbed
			return;
		}
		var next = plan.beats().get(beatAt(frame / (double) fps));
		double nextLocal = frame / (double) fps - startOf(beatAt(frame / (double) fps));
		// Slow motion slows everything on screen; time-lapse only speeds the simulation
		timer.next = Math.min(1.0, next.speedAt(nextLocal)) / fps;
	}

	// ── Phases ──

	private void startWarmup() {
		phase = Phase.WARMUP;
		var cam = scene.showreelCamera();
		overlay = new ShowreelOverlay(scene.getAssetManager(), cam.getWidth(), cam.getHeight(), title, tagline, url);
		scene.getGuiNode().attachChild(overlay.root());
		var views = scene.getRenderManager().getPostViews();
		recorder = new FrameRecorder(output, ffmpeg, fps, crf);
		views.get(views.size() - 1).addProcessor(recorder);
		timer = new FixedTimer(fps);
		scene.setTimer(timer);
		scene.showreelEmphasizePings();
		director = scene.showreelDirector();
		director.setScripted(true);
		director.reseed(plan.seed());
		var overlays = scene.overlayConfig();
		overlays.contactEstimates = false;
		overlays.waypoints = false;
		overlays.strategicWaypoints = false;
		overlays.route = false;
		advanceTo(Math.max(0, plan.beats().getFirst().fromTick()));
		beatIndex = 0;
		beatStart = 0;
		cuePending = true;
		log.printf(Locale.ROOT, "Scene loaded; settling before recording (%s)%n", overlay);
	}

	private void startRecording() {
		phase = Phase.RECORDING;
		frame = 0;
		beatIndex = -1;
		started = System.nanoTime();
		log.printf(Locale.ROOT, "Recording %d frames (%.1f s at %d fps) to %s%n", totalFrames, plan.length(), fps,
				output.toAbsolutePath());
	}

	private void finish() {
		phase = Phase.FINISHED;
		overlay.setFade(1);
		try {
			recorder.finish();
			double seconds = (System.nanoTime() - started) / 1e9;
			log.printf(Locale.ROOT, "Done: %d frames in %.0f s (%.1f fps) -> %s%s%n", recorder.frames(), seconds,
					recorder.frames() / seconds, output.toAbsolutePath(), describeSize());
		} catch (IOException e) {
			log.println("Recording failed: " + e.getMessage());
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
		}
		scene.stop();
	}

	private String describeSize() {
		try {
			return Files.isRegularFile(output)
					? String.format(Locale.ROOT, " (%.1f MB)", Files.size(output) / 1024.0 / 1024.0) : "";
		} catch (IOException e) {
			return "";
		}
	}

	/** Logs progress; if the recording has failed, stops (closing ffmpeg) and returns false. */
	private boolean healthy() {
		double seconds = (System.nanoTime() - started) / 1e9;
		double rate = frame / seconds;
		log.printf(Locale.ROOT, "  %5.1f%%  frame %d/%d  %.1f fps  about %.0f s to go%n", 100.0 * frame / totalFrames,
				frame, totalFrames, rate, (totalFrames - frame) / Math.max(0.1, rate));
		try {
			recorder.checkHealthy();
			return true;
		} catch (IOException e) {
			log.println(e.getMessage());
			finish(); // ends the encoder and ffmpeg too, so the process can exit
			return false;
		}
	}

	// ── Beats ──

	private int beatAt(double time) {
		double at = 0;
		var beats = plan.beats();
		for (int i = 0; i < beats.size(); i++) {
			at += beats.get(i).seconds();
			if (time < at)
				return i;
		}
		return beats.size() - 1;
	}

	private double startOf(int index) {
		double at = 0;
		for (int i = 0; i < index; i++)
			at += plan.beats().get(i).seconds();
		return at;
	}

	/** Sets up a new beat; true if it skips more than a second ahead. */
	private boolean enterBeat(Beat beat) {
		var overlays = scene.overlayConfig();
		overlays.trails = beat.trails();
		overlays.pings = beat.pings();
		// A shot about one ping shows that ping alone
		if (beat.shot() == ShowreelPlan.Shot.SONAR)
			scene.showreelFeaturePing(beat.subject(), beat.fromTick());
		else
			scene.showreelFeaturePing(-1, 0);
		// The overview also shows where each submarine means to go
		overlays.waypoints = beat.shot() == ShowreelPlan.Shot.TACTICAL;
		long jump = beat.fromTick() - scene.showreelTick();
		if (jump > 20L * plan.tickRateHz())
			scene.clearTrails();
		cuePending = true;
		log.printf(Locale.ROOT, "  beat %-18s %-13s at %5.1f s%n", beat.label(), beat.shot(), beatStart);
		return jump > plan.tickRateHz();
	}

	private void cue(Beat beat) {
		TransitionType transition = switch (beat.entry()) {
		case SMOOTH -> TransitionType.SMOOTH_PAN;
		case DESCENT -> TransitionType.SLOW_DESCENT;
		case CUT, FADE_IN, FLASH, CONTINUE -> TransitionType.HARD_CUT;
		};
		if (beat.entry() == ShowreelPlan.Entry.CONTINUE) {
			director.extendShot((float) beat.seconds());
			return;
		}
		ShotType type = switch (beat.shot()) {
		case AERIAL -> ShotType.AERIAL;
		case HERO -> ShotType.IDLE_HERO;
		case FLY_BY -> ShotType.IDLE_FLY_BY;
		case SCENIC -> beat.backdrop() != null ? ShotType.SCENIC : ShotType.IDLE_HERO;
		case TACTICAL -> ShotType.CONTEXT_WIDE;
		case LAUNCH -> ShotType.TORPEDO_LAUNCH;
		case TORPEDO_CHASE -> ShotType.TORPEDO_CHASE;
		case SONAR -> ShotType.SONAR_WIDE;
		case TERMINAL -> ShotType.TORPEDO_TERMINAL;
		case IMPACT -> ShotType.TORPEDO_POV;
		case DETONATION -> ShotType.DETONATION;
		case SINKING -> ShotType.SUB_DEATH;
		case OUTRO -> ShotType.OUTRO;
		};
		int subject = beat.subject();
		if (type == ShotType.CONTEXT_WIDE && subject < 0)
			subject = scene.showreelFirstSubmarine();
		director.cue(type, subject, beat.secondary(), (float) beat.seconds(), transition, jme(beat.focus()),
				jme(beat.backdrop()));
	}

	private static Vector3f jme(double[] sim) {
		return sim == null ? null : new Vector3f((float) sim[0], (float) sim[2], (float) -sim[1]);
	}

	private void updateOverlay(Beat beat, double local, double time) {
		// Fades: in from black at the start, out to black at the end
		float fade = 0;
		if (beat.entry() == ShowreelPlan.Entry.FADE_IN)
			fade = (float) Math.max(0, 1 - local / 1.4);
		double fadeOut = plan.fadeOutSeconds();
		if (time > plan.length() - fadeOut)
			fade = Math.max(fade, (float) ((time - (plan.length() - fadeOut)) / fadeOut));
		overlay.setFade(smooth(fade));
		overlay.setFlash(beat.entry() == ShowreelPlan.Entry.FLASH
				? 0.85f * (float) Math.pow(Math.max(0, 1 - local / 0.5), 1.6) : 0);

		switch (beat.card()) {
		case TITLE -> overlay.showCard(false, smooth(
				(float) Math.min(ramp(local, 1.4, 2.6), 1 - ramp(local, beat.seconds() - 2.2, beat.seconds() - 1.0))));
		case END -> overlay.showCard(true, smooth((float) ramp(local, 1.0, 2.2)));
		case NONE -> overlay.showCard(null, 0);
		}

		if (beat.caption() != null && beat.seconds() >= 2.5) {
			double in = ramp(local, 0.35, 0.8), out = 1 - ramp(local, beat.seconds() - 0.45, beat.seconds() - 0.1);
			overlay.showCaption(beat.caption(), smooth((float) Math.min(in, out)), (float) ramp(local, 0.35, 1.0));
		} else {
			overlay.showCaption(null, 0, 0);
		}
		double speed = beat.shot() == ShowreelPlan.Shot.TACTICAL ? beat.speedAt(local) : 0;
		overlay.showSpeed(speed,
				(float) Math.min(ramp(local, 0, 0.25), 1 - ramp(local, beat.seconds() - 0.15, beat.seconds())));
	}

	private static double ramp(double x, double from, double to) {
		return Math.max(0, Math.min(1, (x - from) / (to - from)));
	}

	private static float smooth(float x) {
		x = Math.max(0, Math.min(1, x));
		return x * x * (3 - 2 * x);
	}

	// ── Simulation stepping ──

	/** Steps the paused replay until it has emitted {@code target}. */
	private void advanceTo(long target) {
		var clock = scene.showreelClock();
		if (clock == null)
			return;
		long now = scene.showreelHasTicked() ? scene.showreelTick() : -1;
		if (target <= now)
			return;
		for (long i = now; i < target; i++)
			clock.stepOnce();
		long deadline = System.nanoTime() + 60_000_000_000L;
		while ((!scene.showreelHasTicked() || scene.showreelTick() < target)
				&& clock.getState() != SimulationLoop.State.STOPPED) {
			if (System.nanoTime() > deadline)
				throw new IllegalStateException("Replay did not reach tick " + target);
			LockSupport.parkNanos(20_000);
		}
	}

	/** Hands out the planned time step instead of measuring the wall clock. */
	static final class FixedTimer extends Timer {
		private final int fps;
		volatile double next;
		private double step;
		private long nanos;

		FixedTimer(int fps) {
			this.fps = fps;
			this.next = 1.0 / fps;
		}

		@Override
		public long getTime() {
			return nanos;
		}

		@Override
		public long getResolution() {
			return 1_000_000_000L;
		}

		@Override
		public float getFrameRate() {
			return fps;
		}

		@Override
		public float getTimePerFrame() {
			return (float) step;
		}

		@Override
		public void update() {
			step = next;
			nanos += Math.round(step * 1e9);
		}

		@Override
		public void reset() {
			nanos = 0;
		}
	}

	void attach(SubmarineScene3D scene) {
		this.scene = scene;
	}

	// ── Command line ──

	public static void main(String[] args) throws Exception {
		var options = Options.parse(args);
		if (options == null)
			return;
		var console = System.out;
		Path work = options.out.toAbsolutePath().getParent().resolve("showreel-work");
		Files.createDirectories(work);

		ShowreelScout.Candidate candidate;
		var controllers = ShowreelScout.parseShips(options.ships);
		var scout = new ShowreelScout(work.resolve("scout"),
				Math.max(1, Runtime.getRuntime().availableProcessors() / 2), 25L * 60 * 50, controllers, console);
		if (options.replay != null) {
			console.println("Indexing " + options.replay);
			candidate = ShowreelScout.quietly(work.resolve("index.log"), () -> ShowreelScout.index(options.replay));
		} else if (options.seed != null) {
			console.println("Simulating seed " + Long.toHexString(options.seed));
			candidate = ShowreelScout.quietly(work.resolve("simulate.log"), () -> scout.simulate(options.seed));
		} else {
			var ranked = ShowreelScout.quietly(work.resolve("scout.log"),
					() -> scout.scout(options.master, options.terrain, options.matches));
			candidate = ranked.getFirst();
		}
		console.println("Filming " + candidate);
		console.println("  replay " + candidate.replay().toAbsolutePath());

		var plan = ShowreelPlan.create(candidate.index(), candidate.seed(), options.length, options.time);
		console.print(plan.describe());
		Path planFile = options.out.resolveSibling(options.out.getFileName() + ".plan.txt");
		Files.writeString(planFile, plan.describe());
		if (options.planOnly)
			return;

		String ffmpeg = options.frames != null ? null : FrameRecorder.findFfmpeg(options.ffmpeg);
		Path target = options.out;
		if (ffmpeg == null) {
			target = options.frames != null ? options.frames : options.out
					.resolveSibling(options.out.getFileName().toString().replaceFirst("\\.[^.]+$", "") + "-frames");
			console.println(
					(options.frames == null ? "No ffmpeg found (install it, set FFMPEG or pass --ffmpeg); " : "")
							+ "writing PNG frames to " + target.toAbsolutePath());
			console.printf(Locale.ROOT,
					"  encode later with: ffmpeg -framerate %d -i %s -c:v libx264 -crf %d -pix_fmt yuv420p %s%n",
					options.fps, target.resolve("frame-%05d.png"), options.crf, options.out);
		}
		var showreel = new Showreel(plan, target, options.fps, options.crf, ffmpeg, options.time, options.title,
				options.tagline, options.url, console);
		SubmarineScene3D.startShowreel(candidate.replay(), showreel, options.width, options.height);
	}

	static final class Options {
		Path out = Path.of("target", "showreel", "searobots-showreel.mp4");
		Path replay, frames;
		Long seed;
		long master = 0x5eab07L;
		int terrain = 64, matches = 16, width = 1920, height = 960, fps = 60, crf = 20;
		double length = 85;
		String ffmpeg, ships;
		String title = ShowreelOverlay.DEFAULT_TITLE;
		String tagline = ShowreelOverlay.DEFAULT_TAGLINE;
		String url = URL;
		LocalTime time = LocalTime.of(15, 0);
		boolean planOnly;

		static Options parse(String[] args) {
			var o = new Options();
			for (int i = 0; i < args.length; i++) {
				String a = args[i];
				switch (a) {
				case "--out" -> o.out = Path.of(args[++i]);
				case "--replay" -> o.replay = Path.of(args[++i]);
				case "--seed" -> o.seed = Long.parseUnsignedLong(strip(args[++i]), 16);
				case "--ships" -> o.ships = args[++i];
				case "--master" -> o.master = Long.parseUnsignedLong(strip(args[++i]), 16);
				case "--scout" -> o.terrain = Integer.parseInt(args[++i]);
				case "--matches" -> o.matches = Integer.parseInt(args[++i]);
				case "--length" -> o.length = Double.parseDouble(args[++i]);
				case "--size" -> {
					String[] wh = args[++i].toLowerCase(Locale.ROOT).split("x");
					o.width = Integer.parseInt(wh[0]);
					o.height = Integer.parseInt(wh[1]);
				}
				case "--fps" -> o.fps = Integer.parseInt(args[++i]);
				case "--crf" -> o.crf = Integer.parseInt(args[++i]);
				case "--ffmpeg" -> o.ffmpeg = args[++i];
				case "--frames" -> o.frames = Path.of(args[++i]);
				case "--time" -> o.time = LocalTime.parse(args[++i]);
				case "--title" -> o.title = args[++i];
				case "--tagline" -> o.tagline = args[++i];
				case "--url" -> o.url = args[++i];
				case "--plan-only" -> o.planOnly = true;
				case "--help", "-h" -> {
					System.out.println(HELP);
					return null;
				}
				default -> throw new IllegalArgumentException("Unknown option " + a + "\n" + HELP);
				}
			}
			return o;
		}

		private static String strip(String hex) {
			return hex.startsWith("0x") || hex.startsWith("0X") ? hex.substring(2) : hex;
		}
	}

	private static final String HELP = """
			Showreel: renders a short cinematic film of a SeaRobots match.

			By default it scouts for a photogenic seed with an exciting fight, simulates it,
			plans the edit and renders it with ffmpeg (on the PATH, in FFMPEG, or --ffmpeg).

			  --out FILE        video to write (default target/showreel/searobots-showreel.mp4)
			  --replay FILE     film this .srl replay instead of scouting
			  --seed HEX        simulate and film this seed instead of scouting
			  --ships A:B       submarines to pit against each other: claude, codex, gemini, default
			                    (default claude:codex)
			  --scout N         seeds whose terrain to survey (default 64)
			  --matches N       of those, how many to simulate (default 16)
			  --master HEX      seed for the scouting draw (default 5eab07)
			  --length SECONDS  target length (default 85)
			  --size WxH        frame size (default 1920x960, a 2:1 cinemascope frame)
			  --fps N           frame rate (default 60)
			  --crf N           x264 quality, lower is better (default 20)
			  --time HH:MM      time of day for the light (default 15:00)
			  --title TEXT      title card heading (default SEAROBOTS)
			  --tagline TEXT    title card subtitle (default AUTONOMOUS UNDERWATER COMBAT)
			  --url TEXT        end card URL (default github.com/thegreystone/searobots)
			  --ffmpeg PATH     ffmpeg executable
			  --frames DIR      write PNG frames instead of a video
			  --plan-only       scout and plan, print the plan, render nothing
			""";
}
