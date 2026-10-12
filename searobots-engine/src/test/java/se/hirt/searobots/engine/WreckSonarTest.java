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
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoLaunchContext;
import se.hirt.searobots.api.TorpedoOutput;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Solid wrecks reflect active sonar without restarting their machinery or sensors. */
class WreckSonarTest {

	private static final TerrainMap TERRAIN = deepFlat();

	@Test
	void livingAndDestroyedHullsHaveTheSameActiveEchoAtTheSameGeometry() {
		for (var config : List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip())) {
			SonarContact livingEcho = submarineEcho(config, 1000);
			SonarContact wreckEcho = submarineEcho(config, 0);
			assertEquals(livingEcho, wreckEcho,
					"A target's HP must not change its echo or disclose whether its hull is alive.");
			assertTrue(wreckEcho.isActive());
			double actualRange = config.surfaceLocked() ? Math.sqrt(20_000) : 100;
			assertNotEquals(actualRange, wreckEcho.range(), 1e-6, "Wreck returns retain ordinary ranging noise.");
			assertEquals(wreckEcho.range() * SonarModel.RANGE_NOISE_FRACTION, wreckEcho.rangeUncertainty(), 1e-9);
		}
	}

	@Test
	void submarinePassiveSonarDoesNotHearWreckMachineryOrDeadPingRequests() {
		var listener = submarine(0, new Vec3(0, 0, -100));
		var wreck = submarine(1, new Vec3(0, 100, -100));
		wreck.setSourceLevelDb(200);
		wreck.activeSonarPing();
		wreck.setHp(0);
		var results = new SonarModel(42).computeContacts(0, List.of(listener, wreck), TERRAIN, List.of());
		assertTrue(results.get(listener.id()).passiveContacts().isEmpty());
		assertTrue(results.get(listener.id()).activeReturns().isEmpty(), "Echoes require the listener's own ping.");
		assertEquals(0, wreck.activeSonarCooldown(), "A dead hull cannot emit its queued ping.");
		assertFalse(wreck.pingRequested(), "An abandoned ping request should still be consumed.");
	}

	@Test
	void destroyedListenersHaveNoSonarReturnsAndEmitNoPing() {
		var deadListener = submarine(0, new Vec3(0, 0, -100));
		deadListener.activeSonarPing();
		deadListener.setHp(0);
		var living = submarine(1, new Vec3(0, 100, -100));
		living.setSourceLevelDb(200);
		var results = new SonarModel(42).computeContacts(0, List.of(deadListener, living), TERRAIN, List.of());
		assertTrue(results.get(deadListener.id()).passiveContacts().isEmpty());
		assertTrue(results.get(deadListener.id()).activeReturns().isEmpty());
		assertTrue(results.get(living.id()).passiveContacts().isEmpty(), "Living listeners must not hear a dead ping.");
		assertEquals(0, deadListener.activeSonarCooldown());
		assertFalse(deadListener.pingRequested());
	}

	@Test
	void torpedoActiveSonarReflectsLivingAndDestroyedHullsEqually() {
		for (var config : List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip())) {
			for (int hp : new int[] {1000, 0}) {
				var echo = torpedoEcho(config, hp);
				double actualRange = config.surfaceLocked() ? Math.sqrt(20_000) : 100;
				assertTrue(echo.isActive());
				assertEquals(actualRange, echo.range(), actualRange * 0.1,
						"The living hull and wreck must each give a useful echo at the same physical range.");
				assertEquals(echo.range() * SonarModel.RANGE_NOISE_FRACTION, echo.rangeUncertainty(), 1e-9);
			}
		}
	}

	@Test
	void torpedoPassiveSonarKeepsWrecksSilent() {
		for (int hp : new int[] {1000, 0}) {
			var torpedo = torpedo();
			var source = submarine(1, new Vec3(0, 100, -100));
			source.setSourceLevelDb(200);
			source.setHp(hp);
			var result = new SonarModel(42).computeContacts(0, List.of(source), List.of(torpedo), TERRAIN, List.of())
					.get(torpedo.id());
			assertEquals(hp > 0 ? 1 : 0, result.passiveContacts().size(),
					"Only a living source emits its machinery noise.");
			assertTrue(result.activeReturns().isEmpty());
		}
	}

	@Test
	void deathStopsPassiveTrackRefreshWhileActiveEchoCanStillFixRange() throws ReflectiveOperationException {
		var listener = submarine(0, new Vec3(0, 0, -100));
		var source = submarine(1, new Vec3(0, 100, -100));
		source.setSourceLevelDb(130);
		var sonar = new SonarModel(42);
		assertEquals(1, sonar.computeContacts(0, List.of(listener, source), TERRAIN, List.of()).get(0).passiveContacts()
				.size());
		var tracks = tracks(sonar);
		var track = tracks.get(1L);
		assertNotNull(track);
		double quality = track.solutionQuality(), uncertainty = track.rangeUncertainty();
		source.setHp(0);
		var silentResult = sonar.computeContacts(50, List.of(listener, source), TERRAIN, List.of()).get(0);
		assertTrue(silentResult.passiveContacts().isEmpty());
		assertEquals(0, track.lastObservationTick(), "Death must not produce a new passive bearing observation.");
		assertEquals(quality - 0.01, track.solutionQuality(), 1e-9);
		assertEquals(uncertainty + 15, track.rangeUncertainty(), 1e-9,
				"The missing observation should grow uncertainty by one second of target travel.");

		listener.activeSonarPing();
		var pingResult = sonar.computeContacts(51, List.of(listener, source), TERRAIN, List.of()).get(0);
		assertTrue(pingResult.passiveContacts().isEmpty());
		assertEquals(1, pingResult.activeReturns().size());
		assertEquals(51, track.lastObservationTick(), "An actual echo may refresh the range fix.");
		assertEquals(0.95, track.solutionQuality(), 1e-9);
		assertEquals(pingResult.activeReturns().getFirst().range(), track.estimatedRange(), 1e-9);

		sonar.computeContacts(1552, List.of(listener, source), TERRAIN, List.of());
		assertNull(tracks.get(1L), "An unobserved wreck track still expires after the ordinary timeout.");
	}

	@Test
	void forfeitedHullsRemainAbsentFromActiveSonar() {
		var listener = submarine(0, new Vec3(0, 0, -100));
		listener.activeSonarPing();
		var removedHull = submarine(1, new Vec3(0, 100, -100));
		removedHull.setHp(0);
		removedHull.setForfeited(true);
		var torpedo = torpedo();
		torpedo.createOutput().activeSonarPing();
		var result = new SonarModel(42).computeContacts(0, List.of(listener, removedHull), List.of(torpedo), TERRAIN,
				List.of());
		// The submarine sees the real torpedo; the torpedo sees the real submarine.
		assertEquals(1, result.get(listener.id()).activeReturns().size());
		assertEquals(1, result.get(torpedo.id()).activeReturns().size());
	}

	private static SonarContact submarineEcho(VehicleConfig targetConfig, int hp) {
		var listener = submarine(0, new Vec3(0, 0, -100));
		listener.activeSonarPing();
		var target = target(targetConfig, hp);
		var result = new SonarModel(42).computeContacts(0, List.of(listener, target), TERRAIN, List.of()).get(0);
		assertEquals(1, result.activeReturns().size());
		return result.activeReturns().getFirst();
	}

	private static SonarContact torpedoEcho(VehicleConfig targetConfig, int hp) {
		var listener = torpedo();
		listener.createOutput().activeSonarPing();
		var result = new SonarModel(42)
				.computeContacts(0, List.of(target(targetConfig, hp)), List.of(listener), TERRAIN, List.of())
				.get(listener.id());
		assertEquals(1, result.activeReturns().size());
		assertTrue(result.passiveContacts().isEmpty());
		return result.activeReturns().getFirst();
	}

	private static SubmarineEntity target(VehicleConfig config, int hp) {
		var target = new SubmarineEntity(config, 1, null, new Vec3(0, 100, -100), 0, Color.RED, 1000);
		target.setHp(hp);
		// A silent living target and a wreck isolate reflected sound from passive machinery.
		target.setSourceLevelDb(0);
		return target;
	}

	private static SubmarineEntity submarine(int id, Vec3 position) {
		var submarine = new SubmarineEntity(VehicleConfig.submarine(), id, null, position, 0, Color.BLUE, 1000);
		submarine.setSourceLevelDb(90);
		return submarine;
	}

	private static TorpedoEntity torpedo() {
		var torpedo = new TorpedoEntity(1000, 99, VehicleConfig.torpedo(), new TorpedoController() {
			@Override
			public void onLaunch(TorpedoLaunchContext context) {
			}

			@Override
			public void onTick(TorpedoInput input, TorpedoOutput output) {
			}
		}, new Vec3(0, 0, -100), 0, 0, 30, Color.GREEN);
		torpedo.setSourceLevelDb(101);
		return torpedo;
	}

	private static TerrainMap deepFlat() {
		var heights = new double[441];
		Arrays.fill(heights, -1000);
		return new TerrainMap(heights, 21, 21, -1000, -1000, 100);
	}

	@SuppressWarnings("unchecked")
	private static Map<Long, ContactTracker> tracks(SonarModel sonar) throws ReflectiveOperationException {
		var field = SonarModel.class.getDeclaredField("trackers");
		field.setAccessible(true);
		return (Map<Long, ContactTracker>) field.get(sonar);
	}
}
