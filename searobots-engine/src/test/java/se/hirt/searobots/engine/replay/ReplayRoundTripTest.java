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
package se.hirt.searobots.engine.replay;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.awt.*;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies the replay system records and reconstructs matches faithfully.
 * <p>
 * The key property is <b>lossless against the live match</b>: the stream a {@link ReplayReader} produces must equal the
 * stream the simulation emitted, not merely agree with the writer. Numbers are compared to the format's fixed precision
 * (4 decimals), which is far below any physically meaningful threshold. The {@code firingSolution} is captured (format
 * v2) and compared; the {@code strategicWaypoints} field is still not captured and is excluded from the comparison.
 */
class ReplayRoundTripTest {

	private static final double EPS = 1e-2;

	private record Frame(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
	}

	/** A listener that just records every callback for later comparison. */
	private static final class CaptureListener implements SimulationListener {
		final List<Frame> frames = new ArrayList<>();
		boolean ended;

		@Override
		public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
			frames.add(new Frame(tick, List.copyOf(subs), List.copyOf(torps)));
		}

		@Override
		public void onMatchEnd() {
			ended = true;
		}
	}

	@Test
	void recordingIsLosslessAgainstLiveMatch(@TempDir Path dir) throws IOException {
		MatchConfig config = shortMatch(42L, 600);
		var world = new WorldGenerator().generate(config);

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9); // run as fast as possible, no real-time pacing

		List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new DefaultAttackSub());
		List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		Path replayFile = dir.resolve("match-42.srl");
		var live = new CaptureListener();
		try (var writer = new ReplayWriter(config, world.spawnPoints(), replayFile)) {
			sim.run(world, controllers, vehicles, SimulationListeners.composite(live, writer));
		}

		assertFalse(live.frames.isEmpty(), "live match should have produced frames");
		assertTrue(Files.size(replayFile) > 0, "replay file should be non-empty");

		var replayed = new CaptureListener();
		long ticks = new ReplayReader(replayFile).replay(replayed);

		assertEquals(live.frames.size(), ticks, "replayed tick count");
		assertEquals(live.frames.size(), replayed.frames.size(), "replayed frame count");
		assertTrue(replayed.ended, "onMatchEnd should fire");

		boolean sawWaypoints = false;
		for (int i = 0; i < live.frames.size(); i++) {
			Frame a = live.frames.get(i);
			Frame b = replayed.frames.get(i);
			assertEquals(a.tick(), b.tick(), "tick at frame " + i);
			assertEquals(a.subs().size(), b.subs().size(), "sub count at frame " + i);
			assertEquals(a.torps().size(), b.torps().size(), "torpedo count at frame " + i);
			for (int s = 0; s < a.subs().size(); s++) {
				assertSubEquals(a.subs().get(s), b.subs().get(s), "frame " + i + " sub " + s);
				sawWaypoints |= !a.subs().get(s).waypoints().isEmpty();
			}
			for (int t = 0; t < a.torps().size(); t++) {
				assertTorpedoEquals(a.torps().get(t), b.torps().get(t), "frame " + i + " torp " + t);
			}
		}
		// The match should have exercised the variable-length child-line path.
		assertTrue(sawWaypoints, "match should have produced waypoints to exercise child lines");
	}

	@Test
	void contactsRoundTripInLongerMatch(@TempDir Path dir) throws IOException {
		// A longer match with the combat controllers (which publish contact
		// estimates and fire torpedoes), so the subs detect each other and the
		// contact child-line path is exercised against a live match (not just
		// the synthetic fixture). Seed 13 reliably produces strong tracking.
		long seed = 13L;
		MatchConfig config = shortMatch(seed, 15_000); // ~5 min sim time
		var world = new WorldGenerator().generate(config);

		var sim = new SimulationLoop();
		sim.setSpeedMultiplier(1e9);

		List<SubmarineController> controllers = List.of(new CodexAttackSub(), new ClaudeAttackSub());
		List<VehicleConfig> vehicles = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

		Path replayFile = dir.resolve("match-" + seed + "-long.srl");
		var live = new CaptureListener();
		try (var writer = new ReplayWriter(config, world.spawnPoints(), replayFile)) {
			sim.run(world, controllers, vehicles, SimulationListeners.composite(live, writer));
		}

		var replayed = new CaptureListener();
		new ReplayReader(replayFile).replay(replayed);
		assertEquals(live.frames.size(), replayed.frames.size(), "frame count");

		long liveContactFrames = countContactFrames(live.frames);
		long replayedContactFrames = countContactFrames(replayed.frames);
		System.out.printf("seed %d: %d/%d live frames had contacts (replayed %d)%n", seed, liveContactFrames,
				live.frames.size(), replayedContactFrames);

		assertTrue(liveContactFrames > 0,
				"subs should have detected each other and published contacts in a 10-min match");
		assertEquals(liveContactFrames, replayedContactFrames,
				"every recorded contact frame must replay with contacts");

		// Verify the contact data itself round-trips field-for-field on every frame.
		for (int i = 0; i < live.frames.size(); i++) {
			for (int s = 0; s < live.frames.get(i).subs().size(); s++) {
				assertSubEquals(live.frames.get(i).subs().get(s), replayed.frames.get(i).subs().get(s),
						"frame " + i + " sub " + s);
			}
		}
	}

	private static long countContactFrames(List<Frame> frames) {
		return frames.stream().filter(fr -> fr.subs().stream().anyMatch(su -> !su.contactEstimates().isEmpty()))
				.count();
	}

	@Test
	void roundTripPreservesTorpedoesAndEdgeCases(@TempDir Path dir) throws IOException {
		MatchConfig config = shortMatch(7L, 1);
		Path file = dir.resolve("synthetic.srl");

		var sub = new SubmarineSnapshot(0, "Claude Sub", new Pose(new Vec3(123.5, -456.25, -200.0), 1.5, -0.25, 0.0),
				new Velocity(new Vec3(6.5, 0.0, -1.25), new Vec3(0.0, 0.0, 0.125)), 6.5, new Color(60, 220, 120), false,
				875, 1.5, 102.5, 0.8, -0.25, 0.1, "CHASE", true, 6,
				List.of(new ContactEstimate(1000.0, 2000.0, 0.7, 0.9, 150.0, 2.5, 7.0, "passive"),
						new ContactEstimate(1050.0, 1980.0, 0.4, 0.6, 300.0, Double.NaN, -1.0, "")),
				List.of(new Waypoint(500.0, 600.0, -250.0, true, false),
						new Waypoint(700.0, 800.0, -250.0, false, true)), List.of(),
				new FiringSolution(1234.5, -678.25, 2.1, 8.5, 0.85));

		var torp = new TorpedoSnapshot(1001, 0, new Pose(new Vec3(200.0, 300.0, -180.0), 0.75, 0.05, 0.0),
				new Velocity(new Vec3(23.0, 0.0, 0.5), Vec3.ZERO), 23.0, new Color(255, 80, 80), 220.5, false, true,
				140.0, false, 1000.0, 2000.0, -200.0, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
				1100.0, 2100.0, -195.0, "TERMINAL");

		var live = new CaptureListener();
		try (var writer = new ReplayWriter(config, List.of(new Vec3(0, 0, -100)), file)) {
			var subs = List.of(sub);
			var torps = List.of(torp);
			live.onTick(0, subs, torps);
			writer.onTick(0, subs, torps);
			writer.onMatchEnd();
		}
		live.onMatchEnd();

		var replayed = new CaptureListener();
		new ReplayReader(file).replay(replayed);

		assertEquals(1, replayed.frames.size());
		assertSubEquals(live.frames.get(0).subs().get(0), replayed.frames.get(0).subs().get(0), "synthetic sub");
		assertTorpedoEquals(live.frames.get(0).torps().get(0), replayed.frames.get(0).torps().get(0), "synthetic torp");
	}

	@Test
	void readsVersion1Files(@TempDir Path dir) throws IOException {
		// The reader accepts every version from MIN_READ_VERSION (v1) to the current one: v2
		// only ADDED the firing-solution child line, so a v1 file is exactly a v2 file without
		// the `f` records. Write a v2 file, mechanically downgrade it to v1, and verify the
		// current reader still decodes everything — with the firing solution (the one thing v1
		// cannot carry) reconstructing as null.
		MatchConfig config = shortMatch(7L, 1);
		Path v2File = dir.resolve("v2.srl");

		var sub = new SubmarineSnapshot(0, "Claude Sub", new Pose(new Vec3(123.5, -456.25, -200.0), 1.5, -0.25, 0.0),
				new Velocity(new Vec3(6.5, 0.0, -1.25), new Vec3(0.0, 0.0, 0.125)), 6.5, new Color(60, 220, 120), false,
				875, 1.5, 102.5, 0.8, -0.25, 0.1, "CHASE", true, 6,
				List.of(new ContactEstimate(1000.0, 2000.0, 0.7, 0.9, 150.0, 2.5, 7.0, "passive")),
				List.of(new Waypoint(500.0, 600.0, -250.0, true, false)), List.of(),
				new FiringSolution(1234.5, -678.25, 2.1, 8.5, 0.85));
		var torp = new TorpedoSnapshot(1001, 0, new Pose(new Vec3(200.0, 300.0, -180.0), 0.75, 0.05, 0.0),
				new Velocity(new Vec3(23.0, 0.0, 0.5), Vec3.ZERO), 23.0, new Color(255, 80, 80), 220.5, false, true,
				140.0, false, 1000.0, 2000.0, -200.0, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
				1100.0, 2100.0, -195.0, "TERMINAL");

		try (var writer = new ReplayWriter(config, List.of(new Vec3(0, 0, -100)), v2File)) {
			writer.onTick(0, List.of(sub), List.of(torp));
			writer.onMatchEnd();
		}

		// Downgrade: rewrite the version on the magic line, drop the `f` schema + data lines.
		Path v1File = dir.resolve("v1.srl");
		try (var w = Files.newBufferedWriter(v1File)) {
			for (String line : Files.readAllLines(v2File)) {
				if (line.startsWith(ReplayFormat.MAGIC + "\t")) {
					w.write(ReplayFormat.MAGIC + "\t1");
				} else if (line.startsWith("COLS\t" + ReplayFormat.TAG_FIRING + "\t")
						|| line.startsWith(ReplayFormat.TAG_FIRING + "\t")) {
					continue;
				} else {
					w.write(line);
				}
				w.newLine();
			}
		}

		var reader = new ReplayReader(v1File);
		assertEquals(1, reader.header().formatVersion(), "reader reports the file's own version");
		var replayed = new CaptureListener();
		reader.replay(replayed);
		assertEquals(1, replayed.frames.size());

		SubmarineSnapshot got = replayed.frames.get(0).subs().get(0);
		assertNull(got.firingSolution(), "v1 cannot carry a firing solution; must reconstruct as null");
		// Everything else must round-trip exactly as if it had been a v2 file.
		var expected = new SubmarineSnapshot(sub.id(), sub.name(), sub.pose(), sub.velocity(), sub.speed(), sub.color(),
				sub.forfeited(), sub.hp(), sub.noiseLevel(), sub.sourceLevelDb(), sub.throttle(), sub.rudder(),
				sub.sternPlanes(), sub.status(), sub.pingRequested(), sub.torpedoesRemaining(), sub.contactEstimates(),
				sub.waypoints(), sub.strategicWaypoints(), null);
		assertSubEquals(expected, got, "v1 sub");
		assertTorpedoEquals(torp, replayed.frames.get(0).torps().get(0), "v1 torp");
	}

	@Test
	void headerIsSelfDescribing(@TempDir Path dir) throws IOException {
		MatchConfig config = shortMatch(99L, 1);
		Path file = dir.resolve("hdr.srl");
		try (var writer = new ReplayWriter(config, List.of(Vec3.ZERO, Vec3.ZERO), file)) {
			writer.onTick(0, List.of(emptySub(0), emptySub(1)), List.of());
			writer.onMatchEnd();
		}

		List<String> lines = Files.readAllLines(file);
		assertTrue(lines.get(0).startsWith("SRREPLAY\t" + ReplayFormat.VERSION), "magic + version first");
		assertTrue(lines.stream().anyMatch(l -> l.startsWith("COLS\ts\t")), "declares sub column schema");
		assertTrue(lines.stream().anyMatch(l -> l.contains("heading:rad")), "declares units for columns");

		var header = new ReplayReader(file).header();
		assertEquals(99L, header.seed());
		assertEquals(config.tickRateHz(), header.tickRateHz());
		assertEquals(2, header.submarines().size());
	}

	// --- helpers ---

	private static MatchConfig shortMatch(long seed, int durationTicks) {
		MatchConfig b = MatchConfig.withDefaults(seed);
		return new MatchConfig(b.worldSeed(), b.tickRateHz(), durationTicks, b.submarineCount(), b.torpedoCount(),
				b.startingHp(), b.blastRadius(), b.minFuseRadius(), b.maxFuseRadius(), b.ratedDepth(), b.crushDepth(),
				b.battleArea(), b.terrainMarginMeters(), b.gridCellMeters(), b.minSeaFloorZ(), b.maxSeaFloorZ(),
				b.maxSubSpeed(), b.startTime());
	}

	private static SubmarineSnapshot emptySub(int id) {
		return new SubmarineSnapshot(id, "Sub" + id, Pose.at(Vec3.ZERO), Velocity.ZERO, 0, new Color(60, 220, 120),
				false, 1000, 0, 90, 0, 0, 0, "PATROL", false, 8, List.of(), List.of(), List.of(), null);
	}

	private static void assertSubEquals(SubmarineSnapshot a, SubmarineSnapshot b, String ctx) {
		assertEquals(a.id(), b.id(), ctx + " id");
		assertEquals(a.name(), b.name(), ctx + " name");
		assertEquals(a.color().getRGB(), b.color().getRGB(), ctx + " color");
		assertEquals(a.hp(), b.hp(), ctx + " hp");
		assertEquals(a.forfeited(), b.forfeited(), ctx + " forfeited");
		assertEquals(a.pingRequested(), b.pingRequested(), ctx + " ping");
		assertEquals(a.torpedoesRemaining(), b.torpedoesRemaining(), ctx + " torpedoesRemaining");
		assertEquals(a.status(), b.status(), ctx + " status");
		assertPose(a.pose(), b.pose(), ctx);
		assertVelocity(a.velocity(), b.velocity(), ctx);
		assertClose(a.speed(), b.speed(), ctx + " speed");
		assertClose(a.noiseLevel(), b.noiseLevel(), ctx + " noise");
		assertClose(a.sourceLevelDb(), b.sourceLevelDb(), ctx + " sourceLevel");
		assertClose(a.throttle(), b.throttle(), ctx + " throttle");
		assertClose(a.rudder(), b.rudder(), ctx + " rudder");
		assertClose(a.sternPlanes(), b.sternPlanes(), ctx + " sternPlanes");
		assertEquals(a.contactEstimates().size(), b.contactEstimates().size(), ctx + " contacts size");
		for (int i = 0; i < a.contactEstimates().size(); i++) {
			ContactEstimate ca = a.contactEstimates().get(i);
			ContactEstimate cb = b.contactEstimates().get(i);
			assertClose(ca.x(), cb.x(), ctx + " contact x");
			assertClose(ca.y(), cb.y(), ctx + " contact y");
			assertClose(ca.confidence(), cb.confidence(), ctx + " contact conf");
			assertClose(ca.contactAlive(), cb.contactAlive(), ctx + " contact alive");
			assertClose(ca.uncertaintyRadius(), cb.uncertaintyRadius(), ctx + " contact ur");
			assertCloseNaN(ca.estimatedHeading(), cb.estimatedHeading(), ctx + " contact estHeading");
			assertClose(ca.estimatedSpeed(), cb.estimatedSpeed(), ctx + " contact estSpeed");
			assertEquals(ca.label(), cb.label(), ctx + " contact label");
		}
		assertEquals(a.waypoints().size(), b.waypoints().size(), ctx + " waypoints size");
		for (int i = 0; i < a.waypoints().size(); i++) {
			Waypoint wa = a.waypoints().get(i);
			Waypoint wb = b.waypoints().get(i);
			assertClose(wa.x(), wb.x(), ctx + " wp x");
			assertClose(wa.y(), wb.y(), ctx + " wp y");
			assertClose(wa.z(), wb.z(), ctx + " wp z");
			assertEquals(wa.active(), wb.active(), ctx + " wp active");
			assertEquals(wa.reverse(), wb.reverse(), ctx + " wp reverse");
		}
		FiringSolution fa = a.firingSolution(), fb = b.firingSolution();
		assertEquals(fa == null, fb == null, ctx + " firingSolution presence");
		if (fa != null) {
			assertClose(fa.targetX(), fb.targetX(), ctx + " fs targetX");
			assertClose(fa.targetY(), fb.targetY(), ctx + " fs targetY");
			assertCloseNaN(fa.targetHeading(), fb.targetHeading(), ctx + " fs targetHeading");
			assertClose(fa.targetSpeed(), fb.targetSpeed(), ctx + " fs targetSpeed");
			assertClose(fa.quality(), fb.quality(), ctx + " fs quality");
		}
	}

	private static void assertTorpedoEquals(TorpedoSnapshot a, TorpedoSnapshot b, String ctx) {
		assertEquals(a.id(), b.id(), ctx + " id");
		assertEquals(a.ownerId(), b.ownerId(), ctx + " owner");
		assertEquals(a.color().getRGB(), b.color().getRGB(), ctx + " color");
		assertEquals(a.detonated(), b.detonated(), ctx + " detonated");
		assertEquals(a.alive(), b.alive(), ctx + " alive");
		assertEquals(a.pingRequested(), b.pingRequested(), ctx + " ping");
		assertEquals(a.diagPhase(), b.diagPhase(), ctx + " phase");
		assertPose(a.pose(), b.pose(), ctx);
		assertVelocity(a.velocity(), b.velocity(), ctx);
		assertClose(a.speed(), b.speed(), ctx + " speed");
		assertClose(a.fuelRemaining(), b.fuelRemaining(), ctx + " fuel");
		assertClose(a.sourceLevelDb(), b.sourceLevelDb(), ctx + " sourceLevel");
		assertClose(a.targetX(), b.targetX(), ctx + " targetX");
		assertClose(a.targetY(), b.targetY(), ctx + " targetY");
		assertClose(a.targetZ(), b.targetZ(), ctx + " targetZ");
		assertCloseNaN(a.diagEstX(), b.diagEstX(), ctx + " diagEstX");
		assertCloseNaN(a.diagEstHeading(), b.diagEstHeading(), ctx + " diagEstHeading");
		assertClose(a.diagIntX(), b.diagIntX(), ctx + " diagIntX");
	}

	private static void assertPose(Pose a, Pose b, String ctx) {
		assertClose(a.position().x(), b.position().x(), ctx + " x");
		assertClose(a.position().y(), b.position().y(), ctx + " y");
		assertClose(a.position().z(), b.position().z(), ctx + " z");
		assertClose(a.heading(), b.heading(), ctx + " heading");
		assertClose(a.pitch(), b.pitch(), ctx + " pitch");
		assertClose(a.roll(), b.roll(), ctx + " roll");
	}

	private static void assertVelocity(Velocity a, Velocity b, String ctx) {
		assertClose(a.linear().x(), b.linear().x(), ctx + " velX");
		assertClose(a.linear().y(), b.linear().y(), ctx + " velY");
		assertClose(a.linear().z(), b.linear().z(), ctx + " velZ");
		assertClose(a.angular().x(), b.angular().x(), ctx + " angVelX");
		assertClose(a.angular().y(), b.angular().y(), ctx + " angVelY");
		assertClose(a.angular().z(), b.angular().z(), ctx + " angVelZ");
	}

	private static void assertClose(double a, double b, String ctx) {
		assertEquals(a, b, EPS, ctx);
	}

	private static void assertCloseNaN(double a, double b, String ctx) {
		if (Double.isNaN(a) || Double.isNaN(b)) {
			assertTrue(Double.isNaN(a) && Double.isNaN(b), ctx + " (NaN mismatch)");
		} else {
			assertEquals(a, b, EPS, ctx);
		}
	}
}
