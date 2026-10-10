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
import se.hirt.searobots.api.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class GroundVelocityTelemetryTest {

	private static final double EPSILON = 1e-9;
	private static final Vec2 CURRENT = new Vec2(2, -3);
	private static final Vec3 CURRENT_3D = new Vec3(2, -3, 0);
	private static final Vec3 SPAWN = new Vec3(0, 0, -200);
	private static final CurrentField CURRENTS = new CurrentField(
			List.of(new CurrentField.CurrentBand(-1000, 0, CURRENT)));
	private static final TerrainMap TERRAIN = flatTerrain();

	@Test
	void opposingCurrentChangesGroundMotionWithoutChangingWaterSpeed() {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, null, SPAWN, 3 * Math.PI / 2, Color.GREEN, 1000);
		sub.setSpeed(8);
		sub.setPitch(Math.PI / 6);
		sub.setVerticalSpeed(-1);
		var input = input(sub.state(), CURRENTS);
		var before = input.self().velocity();

		var ground = input.groundVelocity();

		assertVectorEquals(new Vec3(-4 * Math.sqrt(3) + 2, -3, 3), ground);
		assertEquals(before, input.self().velocity(), "Ground telemetry must not mutate water-relative telemetry");
		assertEquals(8, input.self().surgeSpeed(), EPSILON);
		assertEquals(Math.sqrt(57), input.self().velocity().speed(), EPSILON);
		assertNotEquals(input.self().velocity().speed(), ground.length());
	}

	@Test
	void submarineGroundVelocityUsesCurrentAtItsOwnDepth() {
		var currents = new CurrentField(List.of(new CurrentField.CurrentBand(-100, 0, new Vec2(3, 0)),
				new CurrentField.CurrentBand(-400, -101, new Vec2(-2, 1))));
		var water = new Velocity(new Vec3(1, 5, -0.5), new Vec3(0, 0.1, -0.2));

		assertVectorEquals(new Vec3(4, 5, -0.5), input(stateAtDepth(-50, water), currents).groundVelocity());
		assertVectorEquals(new Vec3(-1, 6, -0.5), input(stateAtDepth(-250, water), currents).groundVelocity());
		assertVectorEquals(water.linear(), input(stateAtDepth(-800, water), currents).groundVelocity());
	}

	@Test
	void zeroCurrentPreservesLinearVelocityIncludingVerticalMotion() {
		var water = new Velocity(new Vec3(-4, 6, 1.5), new Vec3(0, -0.03, 0.08));
		var input = input(stateAtDepth(-200, water), new CurrentField(List.of()));

		assertEquals(water.linear(), input.groundVelocity());
		assertEquals(water, input.self().velocity());
	}

	@Test
	void legacyTorpedoInputWithoutGroundOverrideRetainsWaterVelocity() {
		var water = new Velocity(new Vec3(3, -4, -1), new Vec3(0, 0.2, -0.1));
		var legacy = new LegacyTorpedoInput(17, 0.02, Pose.at(SPAWN), water, 5, 120);

		assertEquals(water.linear(), legacy.groundVelocity());
		assertEquals(water, legacy.velocity());
		assertEquals(5, legacy.speed(), EPSILON);
	}

	@Test
	void simulationSubmarineControllerReceivesTheVelocityThatCausesCurrentDrift() {
		var controller = new CaptureSubmarineController(false);
		var frames = run(controller, 4);
		double dt = 1.0 / MatchConfig.withDefaults(42).tickRateHz();

		assertEquals(4, controller.samples.size());
		for (int i = 0; i < controller.samples.size(); i++) {
			var sample = controller.samples.get(i);
			var snapshot = frames.get(i).submarines().getFirst();
			assertVectorEquals(Vec3.ZERO, sample.water().linear());
			assertVectorEquals(CURRENT_3D, sample.ground());
			assertVectorEquals(sample.position().add(sample.ground().scale(dt)), snapshot.pose().position());
			assertEquals(0, snapshot.velocity().speed(), EPSILON,
					"A stationary submarine drifting with the current has zero speed through water");
		}
	}

	@Test
	void simulationTorpedoControllerReceivesCurrentWithoutChangingItsWaterTelemetry() {
		var controller = new CaptureSubmarineController(true);
		var frames = run(controller, 210);
		var samples = controller.weapon.samples;
		double dt = 1.0 / MatchConfig.withDefaults(42).tickRateHz();

		assertTrue(samples.size() > 10, "The torpedo must leave its tube and execute its controller");
		for (var sample : samples) {
			var previous = frames.get((int) sample.tick() - 1).torpedoes().getFirst();
			var after = frames.get((int) sample.tick()).torpedoes().getFirst();
			assertVectorEquals(previous.pose().position(), sample.position());
			assertEquals(previous.velocity(), sample.water(), "The controller must retain through-water telemetry");
			assertEquals(previous.speed(), sample.speed(), EPSILON);
			assertVectorEquals(previous.velocity().linear().add(CURRENT_3D), sample.ground());
			assertVectorEquals(after.velocity().linear().add(CURRENT_3D),
					after.pose().position().subtract(sample.position()).scale(1.0 / dt));
		}
	}

	private static SubmarineState stateAtDepth(double depth, Velocity water) {
		return new SubmarineState(Pose.at(0, 0, depth, 0), water, 5, 1000, 0);
	}

	private static TestHelpers.TestInput input(SubmarineState state, CurrentField currents) {
		return new TestHelpers.TestInput(0, 0.02, state, new EnvironmentSnapshot(TERRAIN, List.of(), currents),
				List.of(), List.of(), 0);
	}

	private static List<Frame> run(CaptureSubmarineController controller, int ticks) {
		var config = MatchConfig.withDefaults(42).withMatchDurationTicks(ticks);
		var world = new GeneratedWorld(config, TERRAIN, List.of(), CURRENTS, List.of(SPAWN));
		var frames = new ArrayList<Frame>();
		var loop = new SimulationLoop();
		loop.setSpeedMultiplier(1e9);
		loop.run(world, List.of(controller), List.of(VehicleConfig.submarine()), List.of(3 * Math.PI / 2),
				new SimulationListener() {
					@Override
					public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
						frames.add(new Frame(tick, submarines, torpedoes));
					}

					@Override
					public void onMatchEnd() {
					}
				});
		return frames;
	}

	private static TerrainMap flatTerrain() {
		double[] depths = new double[21 * 21];
		Arrays.fill(depths, -1000);
		return new TerrainMap(depths, 21, 21, -2000, -2000, 200);
	}

	private static void assertVectorEquals(Vec3 expected, Vec3 actual) {
		assertAll(() -> assertEquals(expected.x(), actual.x(), EPSILON, "world x"),
				() -> assertEquals(expected.y(), actual.y(), EPSILON, "world y"),
				() -> assertEquals(expected.z(), actual.z(), EPSILON, "world z"));
	}

	private record Frame(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
	}

	private record Sample(long tick, Vec3 position, Velocity water, Vec3 ground, double speed) {
	}

	private record LegacyTorpedoInput(long tick, double deltaTimeSeconds, Pose self, Velocity velocity, double speed,
			double fuelRemaining) implements TorpedoInput {
	}

	private static final class CaptureSubmarineController implements SubmarineController {
		private final boolean launch;
		private final ArrayList<Sample> samples = new ArrayList<>();
		private final CaptureTorpedoController weapon = new CaptureTorpedoController();

		private CaptureSubmarineController(boolean launch) {
			this.launch = launch;
		}

		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			var self = input.self();
			samples.add(new Sample(input.tick(), self.pose().position(), self.velocity(), input.groundVelocity(),
					self.surgeSpeed()));
			output.setThrottle(0);
			output.setBallast(0.5);
			if (launch && input.tick() == 0) {
				output.launchTorpedo(new TorpedoLaunchCommand(self.pose().heading(), 0, 5, ""));
			}
		}

		@Override
		public TorpedoController createTorpedoController() {
			return weapon;
		}
	}

	private static final class CaptureTorpedoController implements TorpedoController {
		private final ArrayList<Sample> samples = new ArrayList<>();

		@Override
		public void onLaunch(TorpedoLaunchContext context) {
		}

		@Override
		public void onTick(TorpedoInput input, TorpedoOutput output) {
			samples.add(new Sample(input.tick(), input.self().position(), input.velocity(), input.groundVelocity(),
					input.speed()));
			output.setThrottle(1);
		}
	}
}
