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

import se.hirt.searobots.api.ContactEstimate;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

/**
 * Reproducible headless combat diagnostics. The opponent is used only as a black-box controller;
 * diagnostics record simulation snapshots, never inspect its controller state or implementation.
 * Numeric seeds are decimal unless prefixed with {@code 0x}.
 */
public final class CodexCombatDiagnostics {

	private CodexCombatDiagnostics() {
	}

	public static void main(String[] args) {
		int ticks = SubmarineCompetition.STANDARD_COMBAT_DURATION_SECONDS * SubmarineCompetition.TICKS_PER_SECOND;
		int count = 20;
		Long master = null;
		boolean seedsOnly = false;
		List<Long> seeds = new ArrayList<>();
		for (String arg : args) {
			if (arg.startsWith("--ticks=")) {
				ticks = Integer.parseInt(arg.substring(8));
			} else if (arg.startsWith("--count=")) {
				count = Integer.parseInt(arg.substring(8));
			} else if (arg.startsWith("--master=")) {
				master = parseSeed(arg.substring(9));
			} else if (arg.equals("--seeds-only")) {
				seedsOnly = true;
			} else {
				seeds.add(parseSeed(arg));
			}
		}
		if (master != null) {
			Random random = new Random(master);
			for (int i = 0; i < count; i++) {
				seeds.add(random.nextLong());
			}
		}
		if (seeds.isEmpty() || ticks <= 0 || count <= 0) {
			throw new IllegalArgumentException(
					"Provide seeds or --master=<seed> --count=20; ticks/count must be positive.");
		}
		System.out.printf("CONFIG master=%s ticks=%d count=%d%n",
				master == null ? "explicit" : "0x" + Long.toHexString(master), ticks, seeds.size());
		for (long seed : seeds) {
			System.out.printf("SEED decimal=%d hex=0x%s%n", seed, Long.toHexString(seed));
		}
		if (seedsOnly) {
			return;
		}
		for (long seed : seeds) {
			run(seed, ticks);
		}
	}

	private static long parseSeed(String value) {
		return value.startsWith("0x") || value.startsWith("0X") ? Long.parseUnsignedLong(value.substring(2), 16)
				: Long.parseLong(value);
	}

	private static void run(long seed, int ticks) {
		MatchConfig config = MatchConfig.withDefaults(seed).withMatchDurationTicks(ticks);
		GeneratedWorld world = new WorldGenerator().generate(config);
		SimulationLoop simulation = new SimulationLoop();
		simulation.setSpeedMultiplier(1_000_000);
		System.out.printf("BEGIN seed=%d%n", seed);
		var recorder = new Recorder(world);
		simulation.run(world, List.<SubmarineController> of(new CodexAttackSub(), new ClaudeAttackSub()),
				List.of(VehicleConfig.submarine(), VehicleConfig.submarine()), recorder);
		System.out.printf(Locale.US,
				"RESULT seed=%d ticks=%d codexHp=%d opponentHp=%d codexAlive=%s opponentAlive=%s codexLaunches=%d opponentLaunches=%d minCodexBoundary=%.1f minCodexFloorClearance=%.1f%n",
				seed, recorder.lastTick + 1, recorder.codexHp, recorder.opponentHp, recorder.codexAlive,
				recorder.opponentAlive, recorder.launches[0], recorder.launches[1], recorder.minBoundary,
				recorder.minFloorClearance);
	}

	private static final class Flight {
		private final long firstTick;
		private double closest = Double.POSITIVE_INFINITY;
		private long closestTick;
		private String phase = "";
		private double lastTargetX = Double.NaN;
		private double lastTargetY = Double.NaN;

		private Flight(long firstTick) {
			this.firstTick = firstTick;
		}
	}

	private static final class Recorder implements SimulationListener {
		private final GeneratedWorld world;
		private final Map<Integer, Flight> flights = new LinkedHashMap<>();
		private final int[] launches = new int[2];
		private long lastTick;
		private int codexHp;
		private int opponentHp;
		private boolean codexAlive = true;
		private boolean opponentAlive = true;
		private double minBoundary = Double.POSITIVE_INFINITY;
		private double minFloorClearance = Double.POSITIVE_INFINITY;

		private Recorder(GeneratedWorld world) {
			this.world = world;
			codexHp = opponentHp = world.config().startingHp();
		}

		@Override
		public void onTick(long tick, List<SubmarineSnapshot> submarines, List<TorpedoSnapshot> torpedoes) {
			lastTick = tick;
			if (submarines.size() < 2) {
				return;
			}
			SubmarineSnapshot codex = submarines.get(0);
			SubmarineSnapshot opponent = submarines.get(1);
			double floorClearance = codex.pose().position().z()
					- world.terrain().elevationAt(codex.pose().position().x(), codex.pose().position().y());
			double boundary = world.config().battleArea().distanceToBoundary(codex.pose().position().x(),
					codex.pose().position().y());
			if (codex.hp() > 0 && !codex.forfeited()) {
				minBoundary = Math.min(minBoundary, boundary);
				minFloorClearance = Math.min(minFloorClearance, floorClearance);
			}
			boolean damage = codex.hp() != codexHp || opponent.hp() != opponentHp;
			if (tick % 1000 == 0 || damage) {
				ContactEstimate estimate = codex.contactEstimates().isEmpty() ? null
						: codex.contactEstimates().getFirst();
				double estimateError = estimate == null ? Double.NaN : Math.hypot(
						estimate.x() - opponent.pose().position().x(), estimate.y() - opponent.pose().position().y());
				System.out.printf(Locale.US,
						"STATE tick=%d range=%.1f codexHp=%d opponentHp=%d codexPos=(%.1f,%.1f,%.1f) opponentPos=(%.1f,%.1f,%.1f) codexSpeed=%.1f codexHeading=%.1f opponentHeading=%.1f throttle=%.2f rudder=%.2f planes=%.2f boundary=%.1f floorClearance=%.1f torps=%d trackError=%.1f trackConfidence=%.3f trackUncertainty=%.1f status=%s%n",
						tick, codex.pose().position().distanceTo(opponent.pose().position()), codex.hp(), opponent.hp(),
						codex.pose().position().x(), codex.pose().position().y(), codex.pose().position().z(),
						opponent.pose().position().x(), opponent.pose().position().y(), opponent.pose().position().z(),
						codex.speed(), Math.toDegrees(codex.pose().heading()),
						Math.toDegrees(opponent.pose().heading()), codex.throttle(), codex.rudder(),
						codex.sternPlanes(), boundary, floorClearance, codex.torpedoesRemaining(), estimateError,
						estimate == null ? Double.NaN : estimate.confidence(),
						estimate == null ? Double.NaN : estimate.uncertaintyRadius(), codex.status());
			}
			if (damage) {
				String detonations = torpedoes.stream().filter(TorpedoSnapshot::detonated)
						.map(t -> "id=" + t.id() + "/owner=" + t.ownerId() + "/codexRange="
								+ Math.round(t.pose().position().distanceTo(codex.pose().position())))
						.reduce((a, b) -> a + "," + b).orElse("none");
				System.out.printf("DAMAGE tick=%d codexDelta=%d opponentDelta=%d detonations=%s%n", tick,
						codex.hp() - codexHp, opponent.hp() - opponentHp, detonations);
			}
			codexHp = codex.hp();
			opponentHp = opponent.hp();
			codexAlive = codexHp > 0 && !codex.forfeited();
			opponentAlive = opponentHp > 0 && !opponent.forfeited();
			for (TorpedoSnapshot torpedo : torpedoes) {
				SubmarineSnapshot target = torpedo.ownerId() == codex.id() ? opponent : codex;
				Flight flight = flights.get(torpedo.id());
				if (flight == null) {
					flight = new Flight(tick);
					flights.put(torpedo.id(), flight);
					if (torpedo.ownerId() >= 0 && torpedo.ownerId() < launches.length) {
						launches[torpedo.ownerId()]++;
					}
					System.out.printf(Locale.US,
							"LAUNCH tick=%d id=%d owner=%d range=%.1f pos=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f)%n",
							tick, torpedo.id(), torpedo.ownerId(),
							torpedo.pose().position().distanceTo(target.pose().position()),
							torpedo.pose().position().x(), torpedo.pose().position().y(), torpedo.pose().position().z(),
							torpedo.targetX(), torpedo.targetY(), torpedo.targetZ());
				}
				double range = torpedo.pose().position().distanceTo(target.pose().position());
				if (range < flight.closest) {
					flight.closest = range;
					flight.closestTick = tick;
				}
				String phase = torpedo.diagPhase() == null ? "" : torpedo.diagPhase();
				double ownerRange = torpedo.pose().position().distanceTo(codex.pose().position());
				double targetJump = Math.hypot(torpedo.targetX() - flight.lastTargetX,
						torpedo.targetY() - flight.lastTargetY);
				if (torpedo.ownerId() == codex.id() && (!phase.equals(flight.phase) || targetJump > 200
						|| tick % 500 == 0 || (range < 500 || ownerRange < 150) && tick % 250 == 0)) {
					System.out.printf(Locale.US,
							"GUIDANCE tick=%d id=%d range=%.1f ownerRange=%.1f speed=%.1f fuel=%.1f phase=%s pos=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f) targetError=%.1f estError=%.1f intercept=(%.1f,%.1f,%.1f)%n",
							tick, torpedo.id(), range, ownerRange, torpedo.speed(), torpedo.fuelRemaining(), phase,
							torpedo.pose().position().x(), torpedo.pose().position().y(), torpedo.pose().position().z(),
							torpedo.targetX(), torpedo.targetY(), torpedo.targetZ(),
							Math.hypot(torpedo.targetX() - target.pose().position().x(),
									torpedo.targetY() - target.pose().position().y()),
							Math.hypot(torpedo.diagEstX() - target.pose().position().x(),
									torpedo.diagEstY() - target.pose().position().y()),
							torpedo.diagIntX(), torpedo.diagIntY(), torpedo.diagIntZ());
				}
				flight.phase = phase;
				flight.lastTargetX = torpedo.targetX();
				flight.lastTargetY = torpedo.targetY();
				if (!torpedo.alive()) {
					System.out.printf(Locale.US,
							"TORPEDO_END tick=%d id=%d owner=%d ageTicks=%d detonated=%s range=%.1f closest=%.1f closestTick=%d speed=%.1f fuel=%.1f floorClearance=%.1f%n",
							tick, torpedo.id(), torpedo.ownerId(), tick - flight.firstTick, torpedo.detonated(), range,
							flight.closest, flight.closestTick, torpedo.speed(), torpedo.fuelRemaining(),
							torpedo.pose().position().z() - world.terrain().elevationAt(torpedo.pose().position().x(),
									torpedo.pose().position().y()));
				}
			}
		}

		@Override
		public void onMatchEnd() {
		}
	}
}
