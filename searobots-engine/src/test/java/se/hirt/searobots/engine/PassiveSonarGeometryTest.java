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
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.ThermalLayer;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.ToDoubleFunction;

import static org.junit.jupiter.api.Assertions.*;
import static se.hirt.searobots.api.VehicleConfig.submarine;

/**
 * Verifies the passive-sonar geometry rules that a stalking submarine relies on, by driving
 * {@link SonarModel} tick-by-tick with scripted poses (no physics loop, no controllers):
 * <ul>
 * <li>The detection arc is the full forward 260 degrees with uniform signal; only the stern baffles
 * are blind.</li>
 * <li>A contact that is only heard through the baffles yields no TMA solution.</li>
 * <li>Lateral (cross-bearing) motion builds a range solution; driving straight down the bearing
 * does not.</li>
 * <li>More lateral baseline and more course changes give progressively better solutions.</li>
 * <li>Terrain between listener and source degrades bearing and speed estimates before it kills
 * detection.</li>
 * <li>A hunter sitting in the target's baffles hears the target and builds a firing solution while
 * never being heard.</li>
 * <li>Terrain cover costs both sides the same dB, so the quieter sub is the one that
 * disappears.</li>
 * </ul>
 * Noise model used for scripted entities mirrors SubmarinePhysics below cavitation: SL = 80 + 2 *
 * speed dB, and the hydrophones hear own noise at SL - 35 dB (VehicleConfig.submarine()).
 */
class PassiveSonarGeometryTest {

	private static final List<ThermalLayer> NO_LAYERS = List.of();
	private static final double DEPTH = -200;
	private static final double DT = 0.02; // 50 ticks per second

	// ── Terrain ──────────────────────────────────────────────────────

	/** Deep flat ocean, 100 m cells, +-5 km. */
	private static TerrainMap deepFlat() {
		int size = 101;
		double cellSize = 100;
		double origin = -(size / 2) * cellSize;
		double[] data = new double[size * size];
		Arrays.fill(data, -500);
		return new TerrainMap(data, size, size, origin, origin, cellSize);
	}

	/**
	 * Deep flat ocean (-500 m, 10 m cells, +-2 km) with a thin east-west ridge. The ridge occupies
	 * {@code rows} consecutive grid rows starting at y = 0 and rises to {@code elevation}. A
	 * one-row ridge at -100 m costs a path at -200 m about 7.5 dB; each additional row adds roughly
	 * 15 dB.
	 */
	private static TerrainMap thinRidge(int rows, double elevation) {
		int size = 401;
		double cellSize = 10;
		double origin = -(size / 2) * cellSize;
		double[] data = new double[size * size];
		Arrays.fill(data, -500);
		int midY = size / 2;
		for (int r = 0; r < rows; r++) {
			for (int x = 0; x < size; x++) {
				data[(midY + r) * size + x] = elevation;
			}
		}
		return new TerrainMap(data, size, size, origin, origin, cellSize);
	}

	private static final TerrainMap FLAT = deepFlat();

	// ── Entities ─────────────────────────────────────────────────────

	private static double slForSpeed(double speed) {
		return 80.0 + 2.0 * speed;
	}

	private static SubmarineEntity makeSub(int id, Vec3 pos, double heading, double speed) {
		return makeSub(id, pos, heading, speed, slForSpeed(speed));
	}

	private static SubmarineEntity makeSub(int id, Vec3 pos, double heading, double speed, double sourceLevelDb) {
		var sub = new SubmarineEntity(submarine(), id, new DefaultAttackSub(), pos, heading, Color.GREEN, 1000);
		sub.setSpeed(speed);
		sub.setSourceLevelDb(sourceLevelDb);
		return sub;
	}

	private static void place(SubmarineEntity sub, double x, double y, double heading, double speed) {
		sub.setX(x);
		sub.setY(y);
		sub.setHeading(heading);
		sub.setSpeed(speed);
		sub.setSourceLevelDb(slForSpeed(speed));
	}

	private static double distance(SubmarineEntity a, SubmarineEntity b) {
		double dx = b.x() - a.x(), dy = b.y() - a.y(), dz = b.z() - a.z();
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	private static double bearingTo(SubmarineEntity from, SubmarineEntity to) {
		double b = Math.atan2(to.x() - from.x(), to.y() - from.y());
		return b < 0 ? b + 2 * Math.PI : b;
	}

	/** Signed angle difference a - b wrapped to [-pi, pi]. */
	private static double angleDiff(double a, double b) {
		double d = (a - b) % (2 * Math.PI);
		if (d > Math.PI)
			d -= 2 * Math.PI;
		if (d < -Math.PI)
			d += 2 * Math.PI;
		return d;
	}

	private static List<SonarContact> passive(
		SonarModel sonar, long tick, SubmarineEntity listener, SubmarineEntity source, TerrainMap terrain) {
		return sonar.computeContacts(tick, List.of(listener, source), terrain, NO_LAYERS).get(listener.id())
				.passiveContacts();
	}

	// ── Statistics ───────────────────────────────────────────────────

	private static double median(double[] values) {
		double[] sorted = values.clone();
		Arrays.sort(sorted);
		int n = sorted.length;
		return n % 2 == 1 ? sorted[n / 2] : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
	}

	private static double stdDev(List<Double> values) {
		double mean = values.stream().mapToDouble(Double::doubleValue).average().orElse(0);
		double ss = 0;
		for (double v : values)
			ss += (v - mean) * (v - mean);
		return Math.sqrt(ss / Math.max(1, values.size() - 1));
	}

	private static double errorPct(double estimated, double actual) {
		return Math.abs(estimated - actual) / actual * 100.0;
	}

	/**
	 * Well-mixed seed for the i-th independent sonar model (splitmix64 finaliser). java.util.Random
	 * scrambles its seed poorly: the first draws of consecutive seeds are strongly correlated,
	 * which would bias any statistic taken over many single-draw models.
	 */
	private static long seedFor(long i) {
		long z = (i + 1) * 0x9E3779B97F4A7C15L;
		z = (z ^ (z >>> 30)) * 0xBF58476D1CE4E5B9L;
		z = (z ^ (z >>> 27)) * 0x94D049BB133111EBL;
		return z ^ (z >>> 31);
	}

	// ── Scripted listener tracks ─────────────────────────────────────

	/** Listener pose at a tick: {x, y, heading, speed}. */
	private interface Motion {
		double[] at(int tick);
	}

	/** Constant heading and speed from (x0, y0). */
	private static Motion straight(double x0, double y0, double heading, double speed) {
		return tick -> {
			double s = speed * DT * tick;
			return new double[] {x0 + s * Math.sin(heading), y0 + s * Math.cos(heading), heading, speed};
		};
	}

	/** Alternates between two headings every {@code legTicks}, instant turns. */
	private static Motion zigzag(double x0, double y0, double headingA, double headingB, int legTicks, double speed) {
		return tick -> {
			double x = x0, y = y0;
			int full = tick / legTicks;
			int rem = tick % legTicks;
			for (int i = 0; i < full; i++) {
				double h = (i % 2 == 0) ? headingA : headingB;
				x += speed * DT * legTicks * Math.sin(h);
				y += speed * DT * legTicks * Math.cos(h);
			}
			double h = (full % 2 == 0) ? headingA : headingB;
			x += speed * DT * rem * Math.sin(h);
			y += speed * DT * rem * Math.cos(h);
			return new double[] {x, y, h, speed};
		};
	}

	private record Sample(int tick, SonarContact contact, double actualRange) {
	}

	/**
	 * Moves the listener along {@code motion} against a stationary target (speed only sets its
	 * radiated level) and records the listener's passive contact each tick.
	 */
	private static List<Sample> track(long seed, int ticks, Motion motion, Vec3 targetPos, double targetSpeed) {
		var sonar = new SonarModel(seed);
		double[] p0 = motion.at(0);
		var listener = makeSub(0, new Vec3(p0[0], p0[1], DEPTH), p0[2], p0[3]);
		var target = makeSub(1, targetPos, Math.PI, targetSpeed);
		var samples = new ArrayList<Sample>(ticks);
		for (int t = 0; t < ticks; t++) {
			double[] p = motion.at(t);
			place(listener, p[0], p[1], p[2], p[3]);
			var contacts = passive(sonar, t, listener, target, FLAT);
			samples.add(new Sample(t, contacts.isEmpty() ? null : contacts.getFirst(), distance(listener, target)));
		}
		return samples;
	}

	private static Sample last(List<Sample> samples) {
		return samples.getLast();
	}

	/** Median over seeds of some statistic of the final sample. */
	private static double medianOverSeeds(
		int seeds, int ticks, Motion motion, Vec3 targetPos, double targetSpeed, ToDoubleFunction<Sample> stat) {
		double[] values = new double[seeds];
		for (int s = 0; s < seeds; s++) {
			var samples = track(1000 + s, ticks, motion, targetPos, targetSpeed);
			var end = last(samples);
			assertNotNull(end.contact(), "Target must remain detected for seed " + s);
			values[s] = stat.applyAsDouble(end);
		}
		return median(values);
	}

	// ══════════════════════════════════════════════════════════════════
	// Baffles
	// ══════════════════════════════════════════════════════════════════

	@Test
	void detectionArcIsUniformForwardAndBlindAstern() {
		// Listener creeping north at 2 m/s (SL 84, self-noise 49 < 55 ambient).
		// Target at 8 m/s (SL 96) at 800 m: SE = 96 - 10*log10(800) - 55 = 12 dB.
		// In the baffles NL rises by 20 dB, so SE = -8 dB: no contact.
		double range = 800;
		var referenceListener = makeSub(0, new Vec3(0, 0, DEPTH), 0, 2);
		var referenceTarget = makeSub(1, new Vec3(0, range, DEPTH), Math.PI, 8);
		double referenceSe = passive(new SonarModel(7), 0, referenceListener, referenceTarget, FLAT).getFirst()
				.signalExcess();
		double baffleDeg = Math.toDegrees(SonarModel.BAFFLE_HALF_ARC);

		for (int deg = 0; deg <= 180; deg += 5) {
			if (deg == (int) Math.round(baffleDeg))
				continue; // exactly on the boundary: floating point decides, not the model
			boolean expectDetected = deg < baffleDeg;

			Boolean[] detectedBySide = new Boolean[2];
			int side = 0;
			for (int sign : new int[] {-1, 1}) {
				double rel = Math.toRadians(sign * deg);
				var sonar = new SonarModel(7);
				var listener = makeSub(0, new Vec3(0, 0, DEPTH), 0, 2);
				var target = makeSub(1, new Vec3(range * Math.sin(rel), range * Math.cos(rel), DEPTH), Math.PI, 8);

				var contacts = passive(sonar, 0, listener, target, FLAT);
				detectedBySide[side++] = !contacts.isEmpty();
				assertEquals(expectDetected, !contacts.isEmpty(),
						"Relative bearing " + sign * deg + " deg: expected detected=" + expectDetected);
				if (expectDetected) {
					var c = contacts.getFirst();
					assertEquals(referenceSe, c.signalExcess(), 0.05,
							"Measured strength for the same pair and seed must not depend on forward bearing ("
									+ sign * deg + " deg)");
					assertEquals(SonarModel.bearingStdDev(c.signalExcess()), c.bearingUncertainty(), 1e-9,
							"Bearing uncertainty must follow the measured strength");
				}
			}
			assertEquals(detectedBySide[0], detectedBySide[1], "Baffles must be symmetric port/starboard at " + deg);
		}
	}

	@Test
	void bafflesFollowOwnHeadingNotCompass() {
		// Same geometry rotated to an arbitrary heading: the blind cone must rotate with the hull.
		double heading = Math.toRadians(237);
		double range = 800;
		var sonar = new SonarModel(7);
		var listener = makeSub(0, new Vec3(0, 0, DEPTH), heading, 2);

		var ahead = makeSub(1, new Vec3(range * Math.sin(heading), range * Math.cos(heading), DEPTH), 0, 8);
		var astern = makeSub(2, new Vec3(-range * Math.sin(heading), -range * Math.cos(heading), DEPTH), 0, 8);
		// Due north of the listener: relative bearing 123 deg, inside the arc
		var north = makeSub(3, new Vec3(0, range, DEPTH), 0, 8);

		assertFalse(passive(sonar, 0, listener, ahead, FLAT).isEmpty(), "Target dead ahead must be heard");
		assertTrue(passive(sonar, 0, listener, astern, FLAT).isEmpty(), "Target dead astern must not be heard");
		assertFalse(passive(sonar, 0, listener, north, FLAT).isEmpty(),
				"Target at 123 deg relative (compass north) must be heard");
	}

	@Test
	void contactHeardOnlyThroughBafflesGivesNoSolution() {
		// A very loud source (SL 130) astern at 300 m punches through the 20 dB baffle
		// penalty (SE = 130 - 24.8 - 75 = 30 dB) so it is detected, but a baffled
		// bearing is not usable for TMA: no range, no quality, unbounded uncertainty.
		var baffledSonar = new SonarModel(7);
		var forwardSonar = new SonarModel(7);
		var listener = makeSub(0, new Vec3(0, 0, DEPTH), 0, 2);
		var astern = makeSub(1, new Vec3(0, -300, DEPTH), 0, 8, 130);
		var ahead = makeSub(1, new Vec3(0, 300, DEPTH), Math.PI, 8, 130);

		SonarContact baffled = null, forward = null;
		for (int t = 0; t < 100; t++) {
			var b = passive(baffledSonar, t, listener, astern, FLAT);
			var f = passive(forwardSonar, t, listener, ahead, FLAT);
			assertFalse(b.isEmpty(), "Loud source astern should still be detected at 300 m");
			assertFalse(f.isEmpty(), "Loud source ahead should be detected at 300 m");
			baffled = b.getFirst();
			forward = f.getFirst();
			if (t == 0) {
				// The same pair and seed share their initial calibration and strength error.
				// Later draws differ because only the forward observation updates TMA.
				assertEquals(forward.signalExcess() - SonarModel.BAFFLE_PENALTY_DB, baffled.signalExcess(), 0.01,
						"Baffles attenuate paired measurements by the baffle penalty");
			}
		}

		assertTrue(baffled.signalExcess() > SonarModel.DETECTION_THRESHOLD_DB);
		assertEquals(0.0, baffled.range(), "Baffled contact must carry no range estimate");
		assertEquals(0.0, baffled.solutionQuality(), "Baffled contact must carry no solution quality");
		assertEquals(Double.MAX_VALUE, baffled.rangeUncertainty(), "Baffled contact range is fully uncertain");

		assertTrue(forward.range() > 0, "Forward contact builds a range estimate");
		assertTrue(forward.solutionQuality() > 0, "Forward contact has a solution quality");
	}

	// ══════════════════════════════════════════════════════════════════
	// TMA geometry
	// ══════════════════════════════════════════════════════════════════

	private static final Vec3 TMA_TARGET = new Vec3(0, 1500, DEPTH);
	private static final double TMA_TARGET_SPEED = 8; // SL 96: SE 12 dB at 1500 m for a quiet listener
	private static final double TMA_LISTENER_SPEED = 4; // SL 88, self-noise 53 < ambient
	private static final int TMA_TICKS = 12_000; // 240 s, 960 m of travel
	private static final int TMA_SEEDS = 9;

	@Test
	void drivingDownTheBearingDoesNotResolveRange() {
		// Listener starts 1500 m south of the target and drives straight at it.
		// Range closes to ~540 m, signal gets strong, yet there is no cross-track
		// baseline: quality stays at the floor and range error stays large.
		var along = straight(0, 0, 0, TMA_LISTENER_SPEED);

		double[] quality = new double[TMA_SEEDS];
		double[] uncertainty = new double[TMA_SEEDS];
		double[] error = new double[TMA_SEEDS];
		for (int s = 0; s < TMA_SEEDS; s++) {
			var end = last(track(1000 + s, TMA_TICKS, along, TMA_TARGET, TMA_TARGET_SPEED));
			assertNotNull(end.contact(), "Target must remain detected for seed " + s);
			quality[s] = end.contact().solutionQuality();
			uncertainty[s] = end.contact().rangeUncertainty() / end.actualRange();
			error[s] = errorPct(end.contact().range(), end.actualRange());
			System.out.printf(
					"[TMA along-bearing seed %d] est=%.0f actual=%.0f error=%.0f%% uncertainty=%.0f%% quality=%.2f%n",
					s, end.contact().range(), end.actualRange(), error[s], uncertainty[s] * 100, quality[s]);

			// Honesty: the tracker must never claim a tight solution it does not have.
			// A reported uncertainty below half the true error would let a bot fire
			// on a phantom range.
			assertTrue(end.contact().rangeUncertainty() >= 0.5 * Math.abs(end.contact().range() - end.actualRange()),
					"Seed " + s + ": reported uncertainty " + end.contact().rangeUncertainty()
							+ " m under-reports a true error of " + Math.abs(end.contact().range() - end.actualRange())
							+ " m");
		}

		assertTrue(median(quality) < 0.15,
				"Closing straight down the bearing must not build quality, got " + median(quality));
		assertTrue(median(error) > 40, "Range error must stay large without geometry, got " + median(error) + "%");
		assertTrue(median(uncertainty) > 0.4,
				"Reported range uncertainty must stay large without geometry, got " + median(uncertainty));
	}

	@Test
	void lateralMotionResolvesRangeWhereClosingDoesNot() {
		// Same target, same speed, same duration. One listener closes along the
		// bearing (north), the other drives across it (east). Only the lateral
		// run accumulates cross-track baseline, so it must end with the better
		// solution on every metric.
		var along = straight(0, 0, 0, TMA_LISTENER_SPEED);
		var lateral = straight(0, 0, Math.toRadians(90), TMA_LISTENER_SPEED);

		ToDoubleFunction<Sample> quality = s -> s.contact().solutionQuality();
		ToDoubleFunction<Sample> uncertainty = s -> s.contact().rangeUncertainty() / s.actualRange();
		ToDoubleFunction<Sample> error = s -> errorPct(s.contact().range(), s.actualRange());

		double qAlong = medianOverSeeds(TMA_SEEDS, TMA_TICKS, along, TMA_TARGET, TMA_TARGET_SPEED, quality);
		double qLateral = medianOverSeeds(TMA_SEEDS, TMA_TICKS, lateral, TMA_TARGET, TMA_TARGET_SPEED, quality);
		double uAlong = medianOverSeeds(TMA_SEEDS, TMA_TICKS, along, TMA_TARGET, TMA_TARGET_SPEED, uncertainty);
		double uLateral = medianOverSeeds(TMA_SEEDS, TMA_TICKS, lateral, TMA_TARGET, TMA_TARGET_SPEED, uncertainty);
		double eAlong = medianOverSeeds(TMA_SEEDS, TMA_TICKS, along, TMA_TARGET, TMA_TARGET_SPEED, error);
		double eLateral = medianOverSeeds(TMA_SEEDS, TMA_TICKS, lateral, TMA_TARGET, TMA_TARGET_SPEED, error);

		System.out.printf(
				"[TMA along vs lateral] quality %.2f vs %.2f, uncertainty %.0f%% vs %.0f%%, error %.0f%% vs %.0f%%%n",
				qAlong, qLateral, uAlong * 100, uLateral * 100, eAlong, eLateral);

		assertTrue(qLateral > qAlong + 0.1,
				"Lateral motion must build clearly more quality: along=" + qAlong + " lateral=" + qLateral);
		assertTrue(uLateral < uAlong,
				"Lateral motion must reduce range uncertainty: along=" + uAlong + " lateral=" + uLateral);
		assertTrue(eLateral < eAlong,
				"Lateral motion must reduce range error: along=" + eAlong + " lateral=" + eLateral);
	}

	@Test
	void moreLateralBaselineGivesBetterSolution() {
		// Snapshot one lateral run at 240 m, 480 m, 960 m of baseline. Quality
		// must rise and relative uncertainty must fall at every step. The range
		// estimate wanders by up to a quarter of the range at low quality, so
		// the per-seed uncertainty ratio is noisy and more seeds are needed for
		// a stable median than in the other TMA tests.
		var lateral = straight(0, 0, Math.toRadians(90), TMA_LISTENER_SPEED);
		int[] checkpoints = {3000, 6000, 12000};
		int seeds = 21;

		double[][] quality = new double[checkpoints.length][seeds];
		double[][] uncertainty = new double[checkpoints.length][seeds];
		for (int s = 0; s < seeds; s++) {
			var samples = track(2000 + s, TMA_TICKS, lateral, TMA_TARGET, TMA_TARGET_SPEED);
			for (int i = 0; i < checkpoints.length; i++) {
				var sample = samples.get(checkpoints[i] - 1);
				assertNotNull(sample.contact(), "Target must remain detected while opening laterally");
				quality[i][s] = sample.contact().solutionQuality();
				uncertainty[i][s] = sample.contact().rangeUncertainty() / sample.actualRange();
			}
		}

		for (int i = 0; i < checkpoints.length; i++) {
			System.out.printf("[TMA baseline %4d m] quality=%.2f uncertainty=%.0f%%%n",
					(int) (checkpoints[i] * DT * TMA_LISTENER_SPEED), median(quality[i]), median(uncertainty[i]) * 100);
		}
		for (int i = 1; i < checkpoints.length; i++) {
			assertTrue(median(quality[i]) > median(quality[i - 1]) + 0.03,
					"Quality must improve with baseline (step " + i + ")");
			assertTrue(median(uncertainty[i]) < median(uncertainty[i - 1]),
					"Uncertainty must shrink with baseline (step " + i + ")");
		}
	}

	@Test
	void courseChangesAddInformationBeyondBaselineAlone() {
		// Zig-zag east/west every 60 s covers the same cross-track distance as the
		// straight lateral run but adds leg changes, which the model rewards.
		var lateral = straight(0, 0, Math.toRadians(90), TMA_LISTENER_SPEED);
		var zigzag = zigzag(0, 0, Math.toRadians(90), Math.toRadians(270), 3000, TMA_LISTENER_SPEED);

		ToDoubleFunction<Sample> quality = s -> s.contact().solutionQuality();
		ToDoubleFunction<Sample> error = s -> errorPct(s.contact().range(), s.actualRange());

		double qLateral = medianOverSeeds(TMA_SEEDS, TMA_TICKS, lateral, TMA_TARGET, TMA_TARGET_SPEED, quality);
		double qZigzag = medianOverSeeds(TMA_SEEDS, TMA_TICKS, zigzag, TMA_TARGET, TMA_TARGET_SPEED, quality);
		double eLateral = medianOverSeeds(TMA_SEEDS, TMA_TICKS, lateral, TMA_TARGET, TMA_TARGET_SPEED, error);
		double eZigzag = medianOverSeeds(TMA_SEEDS, TMA_TICKS, zigzag, TMA_TARGET, TMA_TARGET_SPEED, error);

		System.out.printf("[TMA lateral vs zigzag] quality %.2f vs %.2f, error %.0f%% vs %.0f%%%n", qLateral, qZigzag,
				eLateral, eZigzag);

		assertTrue(qZigzag > qLateral + 0.1, "Legs must add quality: lateral=" + qLateral + " zigzag=" + qZigzag);
		assertTrue(eZigzag <= eLateral, "Legs must not worsen range error: lateral=" + eLateral + " zigzag=" + eZigzag);
		assertTrue(eZigzag < 20, "Three legs at 1500 m should give a usable range, got " + eZigzag + "%");
	}

	// ══════════════════════════════════════════════════════════════════
	// Error correlation
	// ══════════════════════════════════════════════════════════════════

	/**
	 * Standard deviation, across {@code seeds} independent sonar models, of the bearing error
	 * averaged over {@code windowTicks} consecutive ticks of a static geometry. Each error is
	 * normalized by its own reported sigma before averaging, since measured strength wanders.
	 */
	private static double averagedBearingErrorFraction(int seeds, int windowTicks) {
		var listener = makeSub(0, new Vec3(0, 0, DEPTH), 0, 2);
		var source = makeSub(1, new Vec3(0, 1500, DEPTH), Math.PI, 8);
		double trueBearing = bearingTo(listener, source);

		var means = new ArrayList<Double>();
		for (int s = 0; s < seeds; s++) {
			var sonar = new SonarModel(seedFor(9000 + s));
			double sum = 0;
			for (int t = 0; t < windowTicks; t++) {
				var c = passive(sonar, t, listener, source, FLAT).getFirst();
				sum += angleDiff(c.bearing(), trueBearing) / c.bearingUncertainty();
			}
			means.add(sum / windowTicks);
		}
		return stdDev(means);
	}

	@Test
	void bearingErrorCannotBeAveragedAwayWithinTheCorrelationTime() {
		// Independent per-tick errors would shrink as 1/sqrt(n): a 1 s (50 sample)
		// average would keep 14% of the error, a 60 s average 2%. Real bearing
		// error wanders over tens of seconds instead, so short averages buy
		// almost nothing and even a five minute average keeps a good third.
		double single = averagedBearingErrorFraction(200, 1);
		double oneSecond = averagedBearingErrorFraction(200, 50);
		double oneMinute = averagedBearingErrorFraction(100, 3000);
		double fiveMinutes = averagedBearingErrorFraction(40, 15_000);

		System.out.printf("[Bearing averaging] retained error: 1 tick %.0f%%, 1 s %.0f%%, 60 s %.0f%%, 300 s %.0f%%%n",
				single * 100, oneSecond * 100, oneMinute * 100, fiveMinutes * 100);

		assertEquals(1.0, single, 0.2, "Single-sample scatter must match the reported sigma");
		assertTrue(oneSecond > 0.7, "A one second average must keep most of the error, kept " + oneSecond);
		assertTrue(oneMinute > 0.4, "A one minute average must still keep a large share, kept " + oneMinute);
		assertTrue(fiveMinutes > 0.15 && fiveMinutes < oneMinute,
				"Five minutes should help, but far less than 1/sqrt(n): kept " + fiveMinutes);
	}

	@Test
	void consecutiveBearingSamplesAreStronglyCorrelated() {
		// Lag-1 autocorrelation of the error on a static geometry. White noise
		// gives ~0; the model's slow wander gives about the correlated fraction.
		var listener = makeSub(0, new Vec3(0, 0, DEPTH), 0, 2);
		var source = makeSub(1, new Vec3(0, 1500, DEPTH), Math.PI, 8);
		double trueBearing = bearingTo(listener, source);
		var sonar = new SonarModel(77);

		int n = 30_000; // 10 minutes
		double[] err = new double[n];
		for (int t = 0; t < n; t++)
			err[t] = angleDiff(passive(sonar, t, listener, source, FLAT).getFirst().bearing(), trueBearing);

		double mean = Arrays.stream(err).average().orElse(0);
		double var = 0, cov = 0;
		for (int t = 0; t < n; t++)
			var += (err[t] - mean) * (err[t] - mean);
		for (int t = 1; t < n; t++)
			cov += (err[t] - mean) * (err[t - 1] - mean);
		double lag1 = cov / var;
		System.out.printf("[Bearing autocorrelation] lag-1 %.2f%n", lag1);
		assertTrue(lag1 > 0.6, "Consecutive bearing errors must be strongly correlated, got " + lag1);
		assertTrue(lag1 < 0.95, "Some tick-to-tick jitter must remain, got " + lag1);
	}

	// ══════════════════════════════════════════════════════════════════
	// Terrain
	// ══════════════════════════════════════════════════════════════════

	@Test
	void terrainBetweenSubsDegradesBearingAndSpeedBeforeKillingDetection() {
		// Listener and source 1000 m apart at -200 m, facing each other. A two-row
		// ridge at -100 m sits between them. The source is loud (SL 115) so it is
		// still heard through the ridge, but with much less signal excess.
		var ridge = thinRidge(2, -100);
		var src = new Vec3(0, -500, DEPTH);
		var dst = new Vec3(0, 500, DEPTH);
		double occlusion = SonarModel.terrainOcclusionDb(src, dst, ridge);
		assertTrue(occlusion > 10 && occlusion < 40, "Ridge should partially occlude, got " + occlusion + " dB");

		int n = 400;
		var flat = sampleStaticContact(FLAT, src, dst, 115, n);
		var behind = sampleStaticContact(ridge, src, dst, 115, n);

		assertEquals(n, flat.count, "Flat: always detected");
		assertEquals(n, behind.count, "Behind ridge: still detected (loud source)");
		for (int i = 0; i < n; i++) {
			// Paired seeds have the same calibration and strength wander. The ridge still
			// subtracts its physical loss, but a detected contact's display has a floor.
			assertEquals(Math.max(SonarModel.DETECTION_THRESHOLD_DB, flat.measuredStrengths[i] - occlusion),
					behind.measuredStrengths[i], 0.05,
					"Ridge attenuates each paired measurement, subject to the display floor (seed " + i + ")");
		}
		assertTrue(flat.se - behind.se > occlusion * 0.5, "Ridge must substantially lower mean measured strength");

		System.out.printf(
				"[Terrain] occlusion=%.1f dB  SE %.1f -> %.1f dB  bearing sigma %.2f -> %.2f deg (reported %.2f -> %.2f)  speed sigma %.2f -> %.2f m/s%n",
				occlusion, flat.se, behind.se, Math.toDegrees(flat.bearingSigma), Math.toDegrees(behind.bearingSigma),
				Math.toDegrees(flat.reportedBearingSigma), Math.toDegrees(behind.reportedBearingSigma), flat.speedSigma,
				behind.speedSigma);

		// Reported uncertainty grows
		assertTrue(behind.reportedBearingSigma > flat.reportedBearingSigma * 1.5,
				"Reported bearing uncertainty must grow behind terrain");
		// Actual scatter grows and matches what is reported
		assertTrue(behind.bearingSigma > flat.bearingSigma * 1.5, "Actual bearing scatter must grow behind terrain");
		assertEquals(1.0, flat.normalizedBearingSigma, 0.2, "Flat: per-observation reported sigma is honest");
		assertEquals(1.0, behind.normalizedBearingSigma, 0.2, "Ridge: per-observation reported sigma is honest");
		// Blade-rate speed analysis suffers too
		assertTrue(behind.speedSigma > flat.speedSigma * 1.5, "Speed estimate scatter must grow behind terrain");

		// Thicker terrain kills the contact entirely
		var thick = thinRidge(5, -100);
		var gone = sampleStaticContact(thick, src, dst, 115, 10);
		assertEquals(0, gone.count, "Five rows of ridge should hide even a loud source");
	}

	private record StaticStats(int count, double se, double bearingSigma, double reportedBearingSigma,
			double normalizedBearingSigma, double speedSigma, double[] measuredStrengths) {
	}

	/**
	 * Static listener at {@code lPos} facing {@code sPos}; samples the detection {@code n} times
	 * with an independent sonar model (seed) each time. Consecutive ticks of one model share most
	 * of their error by design, so the marginal error distribution has to be measured across
	 * independent realisations.
	 */
	private static StaticStats sampleStaticContact(TerrainMap terrain, Vec3 lPos, Vec3 sPos, double sourceSl, int n) {
		double heading = Math.atan2(sPos.x() - lPos.x(), sPos.y() - lPos.y());
		var listener = makeSub(0, lPos, heading, 2);
		var source = makeSub(1, sPos, heading + Math.PI, 8, sourceSl);
		double trueBearing = bearingTo(listener, source);

		var bearingErrors = new ArrayList<Double>();
		var normalizedBearingErrors = new ArrayList<Double>();
		var speedErrors = new ArrayList<Double>();
		double[] measuredStrengths = new double[n];
		double strengthSum = 0, reportedSigmaSquares = 0;
		int count = 0;
		for (int t = 0; t < n; t++) {
			var sonar = new SonarModel(seedFor(t));
			var contacts = passive(sonar, 0, listener, source, terrain);
			if (contacts.isEmpty())
				continue;
			var c = contacts.getFirst();
			count++;
			measuredStrengths[t] = c.signalExcess();
			strengthSum += c.signalExcess();
			reportedSigmaSquares += c.bearingUncertainty() * c.bearingUncertainty();
			double bearingError = angleDiff(c.bearing(), trueBearing);
			bearingErrors.add(bearingError);
			normalizedBearingErrors.add(bearingError / c.bearingUncertainty());
			if (c.estimatedSpeed() >= 0)
				speedErrors.add(c.estimatedSpeed() - source.speed());
		}
		return new StaticStats(count, count > 0 ? strengthSum / count : Double.NaN, stdDev(bearingErrors),
				count > 0 ? Math.sqrt(reportedSigmaSquares / count) : Double.NaN, stdDev(normalizedBearingErrors),
				stdDev(speedErrors), measuredStrengths);
	}

	@Test
	void terrainCoverHidesTheQuieterSubFirst() {
		// Hunter creeping at 3 m/s (SL 86), target at 6 m/s (SL 92), 200 m apart,
		// facing each other. In open water both hear each other. Put a thin ridge
		// (~7.5 dB) between them: the ridge costs both sides the same, but only the
		// loud target stays above threshold. The quiet hunter vanishes.
		var ridge = thinRidge(1, -100);
		var hunterPos = new Vec3(0, -100, DEPTH);
		var targetPos = new Vec3(0, 100, DEPTH);
		double occlusion = SonarModel.terrainOcclusionDb(hunterPos, targetPos, ridge);
		assertTrue(occlusion > 5 && occlusion < 10, "Thin ridge should cost 5-10 dB, got " + occlusion);

		for (TerrainMap terrain : new TerrainMap[] {FLAT, ridge}) {
			var sonar = new SonarModel(3);
			var hunter = makeSub(0, hunterPos, 0, 3);
			var target = makeSub(1, targetPos, Math.PI, 6);
			var results = sonar.computeContacts(0, List.of(hunter, target), terrain, NO_LAYERS);
			boolean hunterHears = !results.get(0).passiveContacts().isEmpty();
			boolean targetHears = !results.get(1).passiveContacts().isEmpty();

			if (terrain == FLAT) {
				assertTrue(hunterHears, "Open water: hunter hears target");
				assertTrue(targetHears, "Open water: target hears hunter at 200 m");
			} else {
				assertTrue(hunterHears, "Behind ridge: hunter still hears the louder target");
				assertFalse(targetHears, "Behind ridge: target no longer hears the quiet hunter");
			}
		}
	}

	// ══════════════════════════════════════════════════════════════════
	// Stalking scenario
	// ══════════════════════════════════════════════════════════════════

	@Test
	void hunterInTargetsBafflesTracksAndIsNeverHeard() {
		// Target runs north at 8 m/s. Hunter holds 450 m astern, matching speed and
		// weaving +-200 m laterally over a 300 s period (peak 4.2 m/s lateral, so
		// hunter speed <= 9.1 m/s, SL <= 98). Radiated levels follow the same
		// SL = 80 + 2*speed rule for both, so noise alone does not favour the hunter:
		// the target simply cannot hear astern, while the hunter has the target in
		// its forward arc the entire time and builds a firing solution.
		int ticks = 15_000;
		double standoff = 450, amplitude = 200, period = 300;
		double targetSpeed = 8;
		int seeds = 5;

		double[] finalQuality = new double[seeds];
		double[] finalError = new double[seeds];
		double[] initialUncertainty = new double[seeds];
		double[] finalUncertainty = new double[seeds];

		for (int s = 0; s < seeds; s++) {
			var sonar = new SonarModel(500 + s);
			var target = makeSub(1, new Vec3(0, -1200, DEPTH), 0, targetSpeed);
			var hunter = makeSub(0, new Vec3(0, -1200 - standoff, DEPTH), 0, targetSpeed);

			int hunterContacts = 0, targetContacts = 0;
			SonarContact first = null, lastContact = null;
			double actualRange = 0;

			for (int t = 0; t < ticks; t++) {
				double time = t * DT;
				double ty = -1200 + targetSpeed * time;
				place(target, 0, ty, 0, targetSpeed);

				double hx = amplitude * Math.sin(2 * Math.PI * time / period);
				double hvx = amplitude * 2 * Math.PI / period * Math.cos(2 * Math.PI * time / period);
				double hHeading = Math.atan2(hvx, targetSpeed);
				double hSpeed = Math.sqrt(hvx * hvx + targetSpeed * targetSpeed);
				place(hunter, hx, ty - standoff, hHeading, hSpeed);

				// Sanity on the intended geometry
				assertTrue(SonarModel.isInBaffles(target.heading(), bearingTo(target, hunter)),
						"Hunter must stay inside the target's baffles (tick " + t + ")");
				assertFalse(SonarModel.isInBaffles(hunter.heading(), bearingTo(hunter, target)),
						"Target must stay in the hunter's forward arc (tick " + t + ")");

				var results = sonar.computeContacts(t, List.of(hunter, target), FLAT, NO_LAYERS);
				var h = results.get(0).passiveContacts();
				var tc = results.get(1).passiveContacts();
				if (!h.isEmpty()) {
					hunterContacts++;
					if (first == null)
						first = h.getFirst();
					lastContact = h.getFirst();
				}
				if (!tc.isEmpty())
					targetContacts++;
				actualRange = distance(hunter, target);
			}

			assertEquals(ticks, hunterContacts, "Hunter must hold contact every tick (seed " + s + ")");
			assertEquals(0, targetContacts, "Target must never hear the hunter (seed " + s + ")");
			assertNotNull(lastContact);
			finalQuality[s] = lastContact.solutionQuality();
			finalError[s] = errorPct(lastContact.range(), actualRange);
			initialUncertainty[s] = first.rangeUncertainty() / standoff;
			finalUncertainty[s] = lastContact.rangeUncertainty() / actualRange;

			System.out.printf(
					"[Stalk seed %d] quality %.2f, range %.0f m (actual %.0f m, error %.0f%%), uncertainty %.0f%% -> %.0f%%%n",
					s, lastContact.solutionQuality(), lastContact.range(), actualRange, finalError[s],
					initialUncertainty[s] * 100, finalUncertainty[s] * 100);
		}

		assertTrue(median(finalQuality) > 0.5,
				"Weaving in the baffles should reach firing-solution quality, got " + median(finalQuality));
		assertTrue(median(finalError) < 20,
				"Range error after 300 s of weaving should be under 20%, got " + median(finalError) + "%");
		assertTrue(median(finalUncertainty) < median(initialUncertainty) / 2,
				"Range uncertainty should at least halve while weaving");
	}

	@Test
	void sameHunterAbeamIsHeardImmediately() {
		// Control for the stalk scenario: identical hunter and target, identical
		// speeds and noise, but the hunter sits 450 m off the target's beam instead
		// of astern. Now the target hears it at once. Geometry, not noise, hides
		// the stalker.
		var sonar = new SonarModel(500);
		var target = makeSub(1, new Vec3(0, 0, DEPTH), 0, 8);
		var abeam = makeSub(0, new Vec3(450, 0, DEPTH), Math.toRadians(270), 8);
		var astern = makeSub(2, new Vec3(0, -450, DEPTH), 0, 8);

		assertFalse(passive(sonar, 0, target, abeam, FLAT).isEmpty(), "Target hears an equal sub 450 m abeam");
		assertTrue(passive(sonar, 0, target, astern, FLAT).isEmpty(), "Target does not hear the same sub 450 m astern");
	}
}
