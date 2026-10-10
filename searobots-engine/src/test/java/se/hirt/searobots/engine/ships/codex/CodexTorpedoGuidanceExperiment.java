/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.TorpedoEntity;
import se.hirt.searobots.engine.TorpedoPhysics;

import java.awt.Color;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/** Isolated physics experiments with a moving target, independent of an opponent implementation. */
public final class CodexTorpedoGuidanceExperiment {
	private static final double DT = 0.02;

	private CodexTorpedoGuidanceExperiment() {
	}

	public static void main(String[] args) {
		for (double distance : new double[] {700.0, 1_200.0, 2_200.0}) {
			for (double depth : new double[] {-120.0, -280.0}) {
				for (double vx : new double[] {0.0, 7.0}) {
					print(distance, depth, vx, false, simulate(distance, depth, vx, 0.0, false));
					print(distance, depth, vx, true, simulate(distance, depth, vx, 0.0, true));
				}
			}
		}
	}

	static GuidanceOutcome simulate(double distance, double depth, double vx, double vy, boolean noisy) {
		var world = GeneratedWorld.deepFlat();
		var controller = new CodexTorpedoController();
		var torpedo = new TorpedoEntity(1, 0, VehicleConfig.torpedo(), controller, new Vec3(0, 0, -100), 0, 0, 30,
				Color.GREEN);
		torpedo.setSpeed(23);
		controller.onLaunch(new TorpedoLaunchContext(MatchConfig.withDefaults(0), world.terrain(), new Vec3(0, 0, -100),
				0, 0, String.format(Locale.US, "0;%.1f;%.1f;%.6f;%.1f", distance, depth, Math.atan2(vx, vy),
						Math.hypot(vx, vy))));
		var physics = new TorpedoPhysics();
		var rng = new Random(314159);
		double best = Double.POSITIVE_INFINITY;
		double bestAtFirstPass = Double.POSITIVE_INFINITY;
		boolean passed = false;
		double previousDistance = Double.POSITIVE_INFINITY;
		int recedingTicks = 0;
		int firstActiveTick = -1;
		int cooldown = 0;
		boolean pingPending = false;
		for (int tick = 0; tick < 8_000 && torpedo.alive(); tick++) {
			var target = new Vec3(vx * tick * DT, distance + vy * tick * DT, depth);
			var pos = torpedo.pose().position();
			double bearing = Math.atan2(target.x() - pos.x(), target.y() - pos.y());
			double slantRange = pos.distanceTo(target);
			List<SonarContact> fixes = List.of();
			if (pingPending && cooldown == 0) {
				if (firstActiveTick < 0) {
					firstActiveTick = tick;
				}
				fixes = List.of(new SonarContact(bearing + (noisy ? Math.toRadians(0.3) * rng.nextGaussian() : 0), 35,
						slantRange * (1 + (noisy ? 0.02 * rng.nextGaussian() : 0)), true, Math.hypot(vx, vy),
						Math.toRadians(0.3), slantRange * 0.02, 0, 1, Double.NaN,
						depth + (noisy ? Math.max(0.5, slantRange * 0.02) * rng.nextGaussian() : 0),
						SonarContact.Classification.SUBMARINE));
				cooldown = 50;
				pingPending = false;
			}
			List<SonarContact> passive = noisy ? List.of(new SonarContact(bearing + Math.toRadians(2.5), 35, 0, false,
					Math.hypot(vx, vy), Math.toRadians(2.5), Double.MAX_VALUE, 95, 0, Double.NaN, Double.NaN,
					SonarContact.Classification.SUBMARINE)) : List.of();
			controller.onTick(new Input(tick, torpedo.pose(), torpedo.velocity(), torpedo.speed(),
					torpedo.fuelRemaining(), passive, fixes, cooldown), torpedo.createOutput());
			if (torpedo.pingRequested()) {
				pingPending = true;
				torpedo.clearPingRequested();
			}
			physics.step(torpedo, DT, world.terrain(), null, world.config().battleArea());
			if (cooldown > 0) {
				cooldown--;
			}
			double currentDistance = torpedo.pose().position().distanceTo(target);
			best = Math.min(best, currentDistance);
			if (!passed) {
				bestAtFirstPass = Math.min(bestAtFirstPass, currentDistance);
				if (currentDistance > previousDistance && bestAtFirstPass < 500) {
					recedingTicks++;
				} else {
					recedingTicks = 0;
				}
				passed = recedingTicks > 50;
			}
			previousDistance = currentDistance;
		}
		return new GuidanceOutcome(bestAtFirstPass, best, firstActiveTick);
	}

	private static void print(double distance, double depth, double vx, boolean noisy, GuidanceOutcome result) {
		System.out.printf(Locale.US,
				"distance=%.0f depth=%.0f vx=%.0f noisy=%s firstActive=%d firstPass=%.1f best=%.1f%n", distance, depth,
				vx, noisy, result.firstActiveTick(), result.firstPassDistance(), result.bestDistance());
	}

	record GuidanceOutcome(double firstPassDistance, double bestDistance, int firstActiveTick) {
	}

	private record Input(long tick, Pose self, Velocity velocity, double speed, double fuelRemaining,
			List<SonarContact> sonarContacts, List<SonarContact> activeSonarReturns,
			int activeSonarCooldownTicks) implements TorpedoInput {
		@Override
		public double deltaTimeSeconds() {
			return DT;
		}
	}
}
