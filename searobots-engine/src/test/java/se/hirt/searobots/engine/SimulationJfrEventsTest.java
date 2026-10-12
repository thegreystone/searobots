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

import jdk.jfr.Recording;
import jdk.jfr.consumer.RecordedEvent;
import jdk.jfr.consumer.RecordingFile;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class SimulationJfrEventsTest {

	private static final long SEED = 0x5EAF017L;
	private static final int LIVE_TICKS = 170;
	private static final int WRECK_TICKS = 3;

	@Test
	void recordingCarriesRunTickControllerAndWreckContextWithoutChangingTheSimulation(@TempDir
	Path directory) throws IOException {
		Path enabledFile = directory.resolve("enabled.jfr");
		List<Frame> enabledLiving;
		List<Frame> enabledWreck;
		try (var recording = recording(true)) {
			recording.start();
			enabledLiving = run(false);
			enabledWreck = run(true);
			recording.stop();
			recording.dump(enabledFile);
		}

		Path disabledFile = directory.resolve("disabled.jfr");
		try (var recording = recording(false)) {
			recording.start();
			assertNull(SimulationJfrEvents.beginTick(), "Disabled tick recording must avoid event allocation");
			assertNull(SimulationJfrEvents.beginPhase(), "Disabled phase recording must avoid event allocation");
			assertNull(SimulationJfrEvents.beginTerrainMotion(),
					"Disabled terrain recording must avoid event allocation");
			assertEquals(enabledLiving, run(false), "Recording must not change live simulation results");
			assertEquals(enabledWreck, run(true), "Recording must not change wreck contact response");
			recording.stop();
			recording.dump(disabledFile);
		}
		assertTrue(engineEvents(disabledFile).isEmpty());

		var events = engineEvents(enabledFile);
		var ticks = events.stream()
				.filter(event -> event.getEventType().getName().equals(SimulationJfrEvents.TICK_EVENT_NAME)).toList();
		assertEquals(LIVE_TICKS + WRECK_TICKS, ticks.size());
		var runs = ticks.stream().collect(Collectors.groupingBy(event -> event.getLong("correlationId")));
		assertEquals(2, runs.size(), "Repeated matches with the same seed need distinct run IDs");
		long livingRun = runWithTicks(runs, LIVE_TICKS);
		long wreckRun = runWithTicks(runs, WRECK_TICKS);
		for (var run : runs.entrySet()) {
			assertTrue(run.getKey() > 0);
			var byTick = run.getValue().stream()
					.collect(Collectors.toMap(event -> event.getLong("tick"), Function.identity()));
			for (long tick = 0; tick < run.getValue().size(); tick++) {
				assertTrue(byTick.containsKey(tick), "Every executed tick must be represented exactly once");
				assertEquals(1, byTick.get(tick).getInt("vehicleCount"));
			}
		}
		for (var event : events) {
			assertEquals(SEED, event.getLong("worldSeed"));
			assertTrue(runs.containsKey(event.getLong("correlationId")));
			var matchingTick = runs.get(event.getLong("correlationId")).stream()
					.filter(tick -> tick.getLong("tick") == event.getLong("tick")).findFirst().orElseThrow();
			assertFalse(event.getStartTime().isBefore(matchingTick.getStartTime()),
					"Tick correlation must identify the enclosing tick, including terrain motion");
			assertFalse(event.getEndTime().isAfter(matchingTick.getEndTime()));
			assertFalse(event.getDuration().isNegative());
			assertNull(event.getStackTrace(), "Frequent timing events should not capture redundant stacks");
		}

		var phases = events.stream()
				.filter(event -> event.getEventType().getName().equals(SimulationJfrEvents.PHASE_EVENT_NAME)).toList();
		var livingPhases = phases.stream().filter(event -> event.getLong("correlationId") == livingRun).toList();
		for (String global : List.of("sonar", "collisions", "listener")) {
			var recorded = phase(livingPhases, global);
			assertEquals(LIVE_TICKS, recorded.size());
			for (var event : recorded) {
				assertEquals(-1, event.getInt("entityId"));
				assertEquals(1, event.getInt("vehicleCount"));
				assertTrue(Double.isNaN(event.getDouble("x")));
			}
		}
		for (String stage : List.of("submarine controller", "submarine physics")) {
			var recorded = phase(livingPhases, stage);
			assertEquals(LIVE_TICKS, recorded.size());
			for (var event : recorded) {
				assertEquals(0, event.getInt("entityId"));
				assertEquals(CaptureController.class.getName(), event.getString("controllerClass"));
				assertEquals(1000, event.getInt("hp"));
				assertFinitePosition(event, "x", "y", "z");
			}
		}
		var torpedoControllers = phase(livingPhases, "torpedo controller");
		assertFalse(torpedoControllers.isEmpty(), "The test must let the launched torpedo leave its tube");
		assertEquals(torpedoControllers.size(), phase(livingPhases, "torpedo physics").size());
		for (var event : torpedoControllers) {
			assertEquals(1000, event.getInt("entityId"));
			assertEquals(0, event.getInt("ownerId"));
			assertEquals(CaptureTorpedoController.class.getName(), event.getString("controllerClass"));
			assertFinitePosition(event, "x", "y", "z");
		}
		assertTrue(ticks.stream().filter(event -> event.getLong("correlationId") == livingRun)
				.anyMatch(event -> event.getInt("torpedoCount") == 1));

		var wreckPhases = phases.stream().filter(event -> event.getLong("correlationId") == wreckRun).toList();
		assertTrue(phase(wreckPhases, "submarine controller").isEmpty(),
				"Destroyed vehicles must not run a controller");
		assertEquals(WRECK_TICKS, phase(wreckPhases, "submarine physics").size(),
				"Wreck physics must remain observable");
		for (var event : phase(wreckPhases, "submarine physics")) {
			assertEquals(0, event.getInt("hp"));
			assertEquals(0, event.getInt("entityId"));
			assertEquals(CaptureController.class.getName(), event.getString("controllerClass"));
			assertFinitePosition(event, "x", "y", "z");
		}
		var terrainEvents = events.stream()
				.filter(event -> event.getEventType().getName().equals(SimulationJfrEvents.TERRAIN_EVENT_NAME))
				.toList();
		assertEquals(LIVE_TICKS + WRECK_TICKS, terrainEvents.size());
		for (var event : terrainEvents) {
			assertEquals(0, event.getInt("entityId"));
			assertEquals(0.02, event.getDouble("tickDurationSeconds"), 1e-12);
			assertTrue(event.getInt("iterations") >= 1 && event.getInt("iterations") <= 6);
			assertFinitePosition(event, "startX", "startY", "startZ");
			assertFinitePosition(event, "endX", "endY", "endZ");
		}
		var wreckTerrain = terrainEvents.stream().filter(event -> event.getLong("correlationId") == wreckRun).toList();
		assertEquals(WRECK_TICKS, wreckTerrain.size());
		assertTrue(wreckTerrain.stream().anyMatch(event -> event.getInt("contactPointCount") > 0),
				"Actual terrain overlap must expose the material candidates examined by the response");
		assertTrue(wreckTerrain.stream()
				.allMatch(event -> event.getInt("hpAtStart") == 0 && event.getInt("hpAtEnd") == 0));
		assertTrue(wreckTerrain.stream().anyMatch(event -> event.getDouble("endZ") > event.getDouble("startZ")),
				"The wreck fixture must exercise real terrain projection");
	}

	@Test
	void terrainEventReportsActualSweptContactsAndPreImpactContext(@TempDir
	Path directory) throws IOException {
		var config = MatchConfig.withDefaults(SEED);
		var submarine = new SubmarineEntity(VehicleConfig.submarine(), 7, null, new Vec3(0, 0, -94.99), 0, Color.BLUE,
				1000);
		submarine.setVerticalSpeed(-2);
		Path file = directory.resolve("terrain-contact.jfr");
		try (var recording = recording(true)) {
			recording.start();
			new SubmarinePhysics(config).step(submarine, 0.02, flatTerrain(-100), new CurrentField(List.of()),
					config.battleArea());
			recording.stop();
			recording.dump(file);
		}
		var events = engineEvents(file);
		assertEquals(1, events.size(), "A standalone physics step should record only its terrain motion");
		var event = events.getFirst();
		assertEquals(SimulationJfrEvents.TERRAIN_EVENT_NAME, event.getEventType().getName());
		assertEquals(0, event.getLong("correlationId"));
		assertEquals(-1, event.getLong("tick"), "Standalone physics without a supplied tick must report unknown");
		assertEquals(0.02, event.getDouble("tickDurationSeconds"), 1e-12);
		assertEquals(SEED, event.getLong("worldSeed"));
		assertEquals(7, event.getInt("entityId"));
		assertEquals(1000, event.getInt("hpAtStart"));
		assertEquals(submarine.hp(), event.getInt("hpAtEnd"));
		assertEquals(-94.99, event.getDouble("startZ"), 1e-12);
		assertEquals(submarine.z(), event.getDouble("endZ"), 1e-12);
		assertTrue(event.getInt("sweptContacts") > 0,
				"The initially clear descending keel must hit the floor during the tick");
		assertTrue(event.getInt("iterations") >= event.getInt("sweptContacts"));
		assertTrue(event.getInt("contactPointCount") > 0);
		assertTrue(event.getDouble("lastContactFraction") >= 0 && event.getDouble("lastContactFraction") <= 1);
		assertTrue(event.getDouble("remainingSeconds") >= 0 && event.getDouble("remainingSeconds") <= 0.02);
	}

	@Test
	void standalonePhysicsCanSupplyItsTickIndex(@TempDir
	Path directory) throws IOException {
		var config = MatchConfig.withDefaults(SEED);
		var submarine = new SubmarineEntity(VehicleConfig.submarine(), 7, null, new Vec3(0, 0, -100), 0, Color.BLUE,
				1000);
		Path file = directory.resolve("explicit-tick.jfr");
		try (var recording = recording(true)) {
			recording.start();
			new SubmarinePhysics(config).step(submarine, 0.04, flatTerrain(-1000), new CurrentField(List.of()),
					config.battleArea(), 83);
			recording.stop();
			recording.dump(file);
		}
		var events = engineEvents(file);
		assertEquals(1, events.size());
		assertEquals(83, events.getFirst().getLong("tick"));
		assertEquals(0.04, events.getFirst().getDouble("tickDurationSeconds"), 1e-12);
	}

	@Test
	void failingControllerStillCompletesItsPhaseAndEnclosingTickEvent(@TempDir
	Path directory) throws IOException {
		var config = MatchConfig.withDefaults(SEED).withMatchDurationTicks(2);
		var world = new GeneratedWorld(config, flatTerrain(-1000), List.of(), new CurrentField(List.of()),
				List.of(new Vec3(0, 0, -100)));
		var loop = new SimulationLoop();
		loop.setSpeedMultiplier(1e9);
		var listener = new CaptureListener();
		Path file = directory.resolve("failure.jfr");
		try (var recording = recording(true)) {
			recording.start();
			assertThrows(IllegalStateException.class, () -> loop.run(world, List.of(new FailingController()),
					List.of(VehicleConfig.submarine()), List.of(0.0), listener));
			recording.stop();
			recording.dump(file);
		}
		assertTrue(listener.ended, "Instrumentation must preserve normal cleanup after an exception");
		var events = engineEvents(file);
		var ticks = events.stream()
				.filter(event -> event.getEventType().getName().equals(SimulationJfrEvents.TICK_EVENT_NAME)).toList();
		assertEquals(1, ticks.size());
		var phases = events.stream()
				.filter(event -> event.getEventType().getName().equals(SimulationJfrEvents.PHASE_EVENT_NAME)).toList();
		var controller = phase(phases, "submarine controller");
		assertEquals(1, controller.size(), "An exceptional controller call must still close its duration event");
		assertEquals(FailingController.class.getName(), controller.getFirst().getString("controllerClass"));
		assertEquals(0, controller.getFirst().getInt("entityId"));
		assertEquals(0, controller.getFirst().getLong("tick"));
		assertEquals(ticks.getFirst().getLong("correlationId"), controller.getFirst().getLong("correlationId"));
		assertTrue(phase(phases, "submarine physics").isEmpty());
	}

	private static Recording recording(boolean enabled) {
		var recording = new Recording();
		for (var type : List.of(SimulationJfrEvents.SimulationTick.class, SimulationJfrEvents.SimulationPhase.class,
				SimulationJfrEvents.TerrainMotion.class)) {
			if (enabled)
				recording.enable(type).withoutThreshold().withoutStackTrace();
			else
				recording.disable(type);
		}
		return recording;
	}

	private static List<RecordedEvent> engineEvents(Path file) throws IOException {
		return RecordingFile
				.readAllEvents(file).stream().filter(
						event -> List
								.of(SimulationJfrEvents.TICK_EVENT_NAME, SimulationJfrEvents.PHASE_EVENT_NAME,
										SimulationJfrEvents.TERRAIN_EVENT_NAME)
								.contains(event.getEventType().getName()))
				.toList();
	}

	private static long runWithTicks(Map<Long, List<RecordedEvent>> runs, int ticks) {
		return runs.entrySet().stream().filter(entry -> entry.getValue().size() == ticks).findFirst().orElseThrow()
				.getKey();
	}

	private static List<RecordedEvent> phase(List<RecordedEvent> events, String phase) {
		return events.stream().filter(event -> event.getString("phase").equals(phase)).toList();
	}

	private static void assertFinitePosition(RecordedEvent event, String x, String y, String z) {
		assertTrue(Double.isFinite(event.getDouble(x)));
		assertTrue(Double.isFinite(event.getDouble(y)));
		assertTrue(Double.isFinite(event.getDouble(z)));
	}

	private static List<Frame> run(boolean wreck) {
		var defaults = MatchConfig.withDefaults(SEED);
		var config = wreck ? new MatchConfig(SEED, defaults.tickRateHz(), WRECK_TICKS, 1, 0, 0, defaults.blastRadius(),
				defaults.minFuseRadius(), defaults.maxFuseRadius(), defaults.ratedDepth(), defaults.crushDepth(),
				defaults.battleArea(), defaults.terrainMarginMeters(), defaults.gridCellMeters(),
				defaults.minSeaFloorZ(), defaults.maxSeaFloorZ(), defaults.maxSubSpeed(), defaults.startTime())
				: defaults.withMatchDurationTicks(LIVE_TICKS);
		var world = new GeneratedWorld(config, flatTerrain(wreck ? -120 : -1000), List.of(),
				new CurrentField(List.of()), List.of(new Vec3(0, 0, wreck ? -118 : -100)));
		var listener = new CaptureListener();
		var loop = new SimulationLoop();
		loop.setSpeedMultiplier(1e9);
		loop.run(world, List.of(new CaptureController()), List.of(VehicleConfig.submarine()), List.of(0.0), listener);
		assertTrue(listener.ended);
		return listener.frames;
	}

	private static TerrainMap flatTerrain(double floor) {
		return new TerrainMap(new double[] {floor, floor, floor, floor}, 2, 2, -3000, -3000, 6000);
	}

	private record Frame(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
	}

	private static final class CaptureListener implements SimulationListener {
		private final List<Frame> frames = new ArrayList<>();
		private boolean ended;

		@Override
		public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
			frames.add(new Frame(tick, submarines, torpedoes));
		}

		@Override
		public void onMatchEnd() {
			ended = true;
		}
	}

	private static final class CaptureController implements SubmarineController {
		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			output.setThrottle(0);
			output.setBallast(0.5);
			if (input.tick() == 0)
				output.launchTorpedo(new TorpedoLaunchCommand(0, 0, 5, ""));
		}

		@Override
		public TorpedoController createTorpedoController() {
			return new CaptureTorpedoController();
		}
	}

	private static final class CaptureTorpedoController implements TorpedoController {
		@Override
		public void onLaunch(TorpedoLaunchContext context) {
		}

		@Override
		public void onTick(TorpedoInput input, TorpedoOutput output) {
			output.setThrottle(1);
		}
	}

	private static final class FailingController implements SubmarineController {
		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			throw new IllegalStateException("Intentional controller failure");
		}
	}
}
