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

import se.hirt.searobots.api.*;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public final class SimulationLoop implements SimClock {

	private static final Color[] SUB_COLORS = {new Color(60, 220, 120), new Color(220, 80, 80), new Color(80, 140, 255),
			new Color(255, 180, 40), new Color(200, 80, 220), new Color(80, 220, 220)};

	public enum State {
		CREATED, INITIALIZING, RUNNING, PAUSED, STOPPED
	}

	// Per-tick torpedo terminal-approach trace. Enable with -Dtorp.trace=true.
	// Logs the continuously-computed intercept/estimate point vs. the true target
	// so the final-approach geometry (and aim-point jitter) can be analysed.
	private static final boolean TORP_TRACE = Boolean.getBoolean("torp.trace");
	private static final double TORP_TRACE_RANGE = 500.0; // only trace inside this true range

	private volatile double speedMultiplier = 1.0;
	private volatile boolean paused;
	private volatile boolean stepOnce;
	private volatile boolean stopped;
	private volatile State state = State.CREATED;

	private final List<SubmarineEntity> entities = new ArrayList<>();
	private final List<TorpedoEntity> torpedoes = new ArrayList<>();
	private final List<TorpedoEntity> pendingLaunches = new ArrayList<>();
	private int nextTorpedoId = 1000; // torpedo IDs start at 1000 to avoid conflict with sub IDs
	private final java.util.Map<Integer, Integer> nextTube = new java.util.HashMap<>(); // per submarine: tubes fired
	private final TorpedoPhysics torpedoPhysics = new TorpedoPhysics();

	public void run(
		GeneratedWorld world, List<SubmarineController> controllers, List<VehicleConfig> vehicleConfigs,
		SimulationListener listener) {
		run(world, controllers, vehicleConfigs, null, listener);
	}

	/**
	 * Run the simulation with optional per-entity headings.
	 *
	 * @param headings
	 *            optional list of initial headings in radians, or null to use the default (face
	 *            toward center). Individual entries may be Double.NaN to use the default for that
	 *            entity.
	 */
	public void run(
		GeneratedWorld world, List<SubmarineController> controllers, List<VehicleConfig> vehicleConfigs,
		List<Double> headings, SimulationListener listener) {
		Objects.requireNonNull(vehicleConfigs, "vehicleConfigs must not be null");
		state = State.INITIALIZING;
		var config = world.config();
		var physics = new SubmarinePhysics(config);
		var sonar = new SonarModel(config.worldSeed(), config.maxSubSpeed());
		double dt = 1.0 / config.tickRateHz();

		var envSnapshot = new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
		var matchContext = new MatchContext(config, world.terrain(), world.thermalLayers(), world.currentField());

		// Create entities at spawn points
		entities.clear();
		torpedoes.clear();
		pendingLaunches.clear();
		var spawns = world.spawnPoints();
		for (int i = 0; i < controllers.size(); i++) {
			var spawn = i < spawns.size() ? spawns.get(i) : spawns.getFirst();
			double heading;
			if (headings != null && i < headings.size() && !Double.isNaN(headings.get(i))) {
				heading = headings.get(i);
			} else {
				// Pick a heading that doesn't point into shallow water
				double safeHeading = WorldGenerator.findSafeHeading(world.terrain(), spawn.x(), spawn.y());
				if (!Double.isNaN(safeHeading)) {
					heading = safeHeading;
				} else {
					heading = Math.atan2(-spawn.x(), -spawn.y()); // fallback: face center
					if (heading < 0)
						heading += 2 * Math.PI;
				}
			}
			var vCfg = i < vehicleConfigs.size() ? vehicleConfigs.get(i) : VehicleConfig.submarine();
			var entity = new SubmarineEntity(vCfg, i, controllers.get(i), spawn, heading,
					SUB_COLORS[i % SUB_COLORS.length], config.startingHp());
			entity.setTorpedoesRemaining(config.torpedoCount());
			entities.add(entity);
		}

		// Main loop (try-finally ensures onMatchEnd always fires,
		// even if the thread is interrupted during sleep)
		boolean matchStarted = false;
		try {
			for (long tick = 0; tick < config.matchDurationTicks() && !stopped; tick++) {
				// Handle pause (sim may start paused to let viewers register)
				if (paused)
					state = State.PAUSED;
				while (paused && !stepOnce && !stopped) {
					try {
						Thread.sleep(50);
					} catch (InterruptedException ex) {
						break;
					}
				}
				if (stopped)
					break;
				stepOnce = false;
				state = State.RUNNING;

				// Notify controllers on first active tick, not during setup pause.
				// This prevents controllers from gaining extra processing time
				// while viewers are initializing.
				if (!matchStarted) {
					for (var e : entities) {
						e.controller().onMatchStart(matchContext);
					}
					matchStarted = true;
				}

				final long currentTick = tick;

				// Compute sonar contacts (based on current positions, before controller runs)
				var sonarResults = sonar.computeContacts(currentTick, entities, torpedoes, world.terrain(),
						world.thermalLayers());

				// Build input, call controller, advance physics
				for (var entity : entities) {
					if (entity.forfeited())
						continue;

					if (entity.hp() <= 0) {
						// Dead sub: no controller, but physics still runs for sinking.
						// Disengage clutch (prop freewheels), flood ballast (sink).
						entity.setThrottle(0);
						entity.setEngineClutch(false);
						entity.setBallast(0); // full flood = maximum negative buoyancy
						entity.setRudder(0);
						entity.setSternPlanes(0);
						physics.step(entity, dt, world.terrain(), world.currentField(), config.battleArea());
						continue;
					}

					var sr = sonarResults.getOrDefault(entity.id(),
							new SonarModel.SonarResult(List.of(), List.of(), 0));
					var explosionEvs = entity.drainExplosionEvents();
					var input = new SubmarineInput() {
						@Override
						public long tick() {
							return currentTick;
						}

						@Override
						public double deltaTimeSeconds() {
							return dt;
						}

						@Override
						public SubmarineState self() {
							return entity.state();
						}

						@Override
						public EnvironmentSnapshot environment() {
							return envSnapshot;
						}

						@Override
						public List<SonarContact> sonarContacts() {
							return sr.passiveContacts();
						}

						@Override
						public List<SonarContact> activeSonarReturns() {
							return sr.activeReturns();
						}

						@Override
						public int activeSonarCooldownTicks() {
							return sr.cooldownTicks();
						}

						@Override
						public List<ExplosionEvent> explosionEvents() {
							return explosionEvs;
						}
					};

					entity.controller().onTick(input, entity);
					physics.step(entity, dt, world.terrain(), world.currentField(), config.battleArea());
				}

				// Process pending torpedo launches
				for (var entity : entities) {
					var cmd = entity.pendingTorpedoLaunch();
					if (cmd != null) {
						entity.clearPendingTorpedoLaunch();
						// Load the torpedo into the submarine's next tube. It rides in the tube (see TorpedoTubes) and
						// only gets its launch context, controller and physics once it has cleared the muzzle.
						int tubeIndex = nextTube.merge(entity.id(), 1, Integer::sum) - 1;
						var tube = TorpedoTubes.TUBES.get(tubeIndex % TorpedoTubes.TUBES.size());
						double fuseR = Math.clamp(cmd.fuseRadius(), config.minFuseRadius(), config.maxFuseRadius());

						var customCtrl = entity.controller().createTorpedoController();
						var torpCtrl = customCtrl != null ? customCtrl
								: new se.hirt.searobots.engine.ships.SimpleTorpedoController();
						var torp = new TorpedoEntity(nextTorpedoId++, entity.id(), VehicleConfig.torpedo(), torpCtrl,
								new Vec3(entity.x(), entity.y(), entity.z()), entity.heading(), entity.pitch(), fuseR,
								entity.color());
						torp.loadIntoTube(entity, tube, dt, cmd.missionData() != null ? cmd.missionData() : "");
						torp.setLaunchTick(tick);
						pendingLaunches.add(torp);
					}
					entity.decrementLaunchTransient();
				}
				torpedoes.addAll(pendingLaunches);
				pendingLaunches.clear();

				// Torpedo controller + physics
				for (var torp : torpedoes) {
					if (!torp.alive())
						continue;

					// Still in its tube: carried by the submarine until it has cleared the muzzle, then released
					if (torp.inTube()) {
						var owner = torp.tubeOwner();
						boolean ownerGone = owner.hp() <= 0 || owner.forfeited();
						if (torp.advanceInTube(dt) || ownerGone) {
							torp.leaveTube();
							torp.setLaunchTick(tick);
							var launchPos = new Vec3(torp.x(), torp.y(), torp.z());
							torp.controller().onLaunch(new TorpedoLaunchContext(config, world.terrain(), launchPos,
									torp.heading(), torp.pitch(), torp.missionData()));
						}
						continue;
					}

					// Manual detonation request
					if (torp.detonateRequested()) {
						handleDetonation(torp, entities, config, false);
						continue;
					}

					// Build torpedo input with sonar contacts
					var torpOutput = torp.createOutput();
					final long torpTick = tick;
					var torpSonar = sonarResults.getOrDefault(torp.id(),
							new SonarModel.SonarResult(List.of(), List.of(), 0));
					var torpInput = new TorpedoInput() {
						@Override
						public long tick() {
							return torpTick;
						}

						@Override
						public double deltaTimeSeconds() {
							return dt;
						}

						@Override
						public Pose self() {
							return torp.pose();
						}

						@Override
						public Velocity velocity() {
							return torp.velocity();
						}

						@Override
						public Vec3 groundVelocity() {
							var current = world.currentField().currentAt(torp.z());
							return torp.velocity().linear().add(new Vec3(current.x(), current.y(), 0));
						}

						@Override
						public double speed() {
							return torp.speed();
						}

						@Override
						public double fuelRemaining() {
							return torp.fuelRemaining();
						}

						@Override
						public List<SonarContact> sonarContacts() {
							return torpSonar.passiveContacts();
						}

						@Override
						public List<SonarContact> activeSonarReturns() {
							return torpSonar.activeReturns();
						}

						@Override
						public int activeSonarCooldownTicks() {
							return torpSonar.cooldownTicks();
						}
					};
					torp.controller().onTick(torpInput, torpOutput);

					if (TORP_TRACE) {
						traceTorpedoApproach(tick, torp, entities);
					}

					// Check detonation request from controller
					if (torp.detonateRequested()) {
						handleDetonation(torp, entities, config, false);
						continue;
					}

					// Torpedo physics
					torpedoPhysics.step(torp, dt, world.terrain(), world.currentField(), config.battleArea());
				}

				// Proximity fuse check against sub ellipsoid (only after arming delay)
				for (var torp : torpedoes) {
					if (!torp.alive() || torp.inTube())
						continue;
					torp.decrementArmingDelay();
					if (!torp.fuseArmed())
						continue;
					for (var sub : entities) {
						if (sub.hp() <= 0 || sub.forfeited())
							continue;
						// Skip owner during first 5 seconds (250 ticks) to let torpedo clear
						if (sub.id() == torp.ownerId() && tick - torp.launchTick() < 250)
							continue;
						// Use torpedo bow (nose) point: warhead is in the front
						double hullDist = HullGeometry.bowDistanceToHull(torp.x(), torp.y(), torp.z(), torp.heading(),
								torp.pitch(), torp.vehicleConfig().hullHalfLength(), sub.x(), sub.y(), sub.z(),
								sub.heading(), sub.pitch());
						if (hullDist < torp.fuseRadius()) {
							System.out.printf(
									"[Torpedo %d] PROXIMITY FUSE at tick %d, hull dist=%.1fm to sub %d (%s)%n",
									torp.id(), tick, hullDist, sub.id(), sub.controller().name());
							handleDetonation(torp, entities, config, true);
							break;
						}
					}
				}

				// Handle detonations that weren't from proximity fuse
				// (terrain impact, manual detonation, stall)
				for (var t : torpedoes) {
					if (t.detonated() && !t.explosionProcessed()) {
						handleDetonation(t, entities, config, false);
					}
				}

				// Sub-to-sub collision (ramming)
				checkSubCollisions(entities, world.currentField());

				// Snapshots BEFORE removing dead torpedoes, so the viewer sees
				// detonated torpedoes for one tick and can create explosion effects.
				var snapshots = entities.stream().map(SubmarineEntity::snapshot).toList();
				var torpSnapshots = torpedoes.stream().map(TorpedoEntity::snapshot).toList();

				// Clear per-tick published data after snapshot
				entities.forEach(SubmarineEntity::clearContactEstimates);
				entities.forEach(SubmarineEntity::clearWaypoints);
				entities.forEach(SubmarineEntity::clearStrategicWaypoints);
				entities.forEach(SubmarineEntity::clearFiringSolution);
				// Note: torpedo pings are NOT cleared here. They're cleared by the sonar
				// model on the next tick's computeContacts(), same as submarine pings.

				// Post-tick: consume pings, tick cooldowns
				sonar.postTick(entities);
				torpedoes.forEach(TorpedoEntity::decrementSonarCooldown);

				// Notify listener
				listener.onTick(tick, snapshots, torpSnapshots);

				// Remove dead torpedoes (after snapshot and listener notification)
				final long logTick = tick;
				torpedoes.removeIf(t -> {
					if (t.alive())
						return false;
					String fate = t.detonated() ? "DETONATED" : "LOST";
					if (!t.detonated()) {
						if (t.speed() < TorpedoEntity.minimumSpeed())
							fate = "STALLED (sank)";
						else if (t.fuelRemaining() <= 0 && t.speed() < 1)
							fate = "FUEL DEPLETED (sank)";
						else
							fate = "TERRAIN/BOUNDARY";
					}
					System.out.printf("[Torpedo %d] %s at tick %d  pos=(%.0f,%.0f,%.0f) speed=%.1f fuel=%.1f%n", t.id(),
							fate, logTick, t.x(), t.y(), t.z(), t.speed(), t.fuelRemaining());
					return true;
				});

				// End the match as soon as the outcome is decided: no point simulating
				// a lone survivor for the remaining duration.
				if (matchDecided(entities, torpedoes)) {
					System.out.printf("[Match] Outcome decided at tick %d (%ds): ending early%n", tick,
							tick / config.tickRateHz());
					break;
				}

				// Timing
				long sleepMs = (long) (1000.0 / config.tickRateHz() / speedMultiplier);
				if (sleepMs > 0 && !paused) {
					try {
						Thread.sleep(sleepMs);
					} catch (InterruptedException ex) {
						break;
					}
				}
			}
		} finally {
			state = State.STOPPED;
			// Match end: always called regardless of how the loop exits
			for (var e : entities) {
				e.controller().onMatchEnd(new MatchResult(e.forfeited()));
			}
			listener.onMatchEnd();
		}
	}

	/**
	 * True once a multi-submarine match can no longer change outcome: at most one submarine is
	 * still alive and no torpedo is still running. Any live torpedo counts, not only those of dead
	 * submarines: the blast damage in {@link #handleDetonation} has no owner exemption and the
	 * proximity fuse only spares the launcher for its first five seconds, so a lone survivor can
	 * still be sunk by its own weapon. Solo runs (navigation scenarios) never end early.
	 */
	static boolean matchDecided(List<SubmarineEntity> entities, List<TorpedoEntity> torpedoes) {
		if (entities.size() < 2)
			return false;
		int alive = 0;
		for (var e : entities) {
			if (e.hp() > 0 && !e.forfeited())
				alive++;
		}
		if (alive > 1)
			return false;
		for (var t : torpedoes) {
			if (t.alive())
				return false;
		}
		return true;
	}

	/**
	 * Distance from a torpedo to the nearest point on a sub's hull ellipsoid.
	 */
	private static double distanceToHullSurface(TorpedoEntity torp, SubmarineEntity sub) {
		return HullGeometry.distanceToHull(torp, sub);
	}

	// ── Torpedo terminal-approach trace ──────────────────────────────

	// Tracks the previously logged intercept point per torpedo to report
	// tick-to-tick aim-point movement (the "jitter").
	private final java.util.Map<Integer, double[]> lastInt = new java.util.HashMap<>();

	private void traceTorpedoApproach(long tick, TorpedoEntity torp, List<SubmarineEntity> subs) {
		// Nearest living enemy sub = presumed target; report true closing geometry.
		SubmarineEntity target = null;
		double bestRange = Double.MAX_VALUE;
		for (var sub : subs) {
			if (sub.id() == torp.ownerId() || sub.hp() <= 0 || sub.forfeited())
				continue;
			double dx = sub.x() - torp.x(), dy = sub.y() - torp.y(), dz = sub.z() - torp.z();
			double r = Math.sqrt(dx * dx + dy * dy + dz * dz);
			if (r < bestRange) {
				bestRange = r;
				target = sub;
			}
		}
		if (target == null || bestRange > TORP_TRACE_RANGE)
			return;

		double intX = torp.diagIntX(), intY = torp.diagIntY(), intZ = torp.diagIntZ();
		double estX = torp.diagEstX(), estY = torp.diagEstY(), estZ = torp.diagEstZ();

		// Aim-point movement since last tick (how much the intercept point jumped).
		double intJump = Double.NaN;
		double[] prev = lastInt.get(torp.id());
		if (prev != null && !Double.isNaN(intX)) {
			intJump = Math.sqrt((intX - prev[0]) * (intX - prev[0]) + (intY - prev[1]) * (intY - prev[1])
					+ (intZ - prev[2]) * (intZ - prev[2]));
		}
		if (!Double.isNaN(intX))
			lastInt.put(torp.id(), new double[] {intX, intY, intZ});

		// Estimate error: how far the published target estimate is from truth.
		double estErr = Double.NaN;
		if (!Double.isNaN(estX)) {
			estErr = Math.sqrt((estX - target.x()) * (estX - target.x()) + (estY - target.y()) * (estY - target.y())
					+ (estZ - target.z()) * (estZ - target.z()));
		}

		// Heading error from torpedo heading to the intercept bearing.
		double hdgErr = Double.NaN;
		if (!Double.isNaN(intX)) {
			double brg = Math.atan2(intX - torp.x(), intY - torp.y());
			hdgErr = Math.toDegrees(normalizeAngle(brg - torp.heading()));
		}

		System.out.printf(
				"[TT] tick=%d id=%d phase=%-8s spd=%.1f rTrue=%.0f hdgErrInt=%s intJump=%s estErr=%s "
						+ "pos=(%.0f,%.0f,%.0f) est=(%.0f,%.0f,%.0f) int=(%.0f,%.0f,%.0f) tgt=(%.0f,%.0f,%.0f)%n",
				tick, torp.id(), torp.diagPhase(), torp.speed(), bestRange, fmt(hdgErr), fmt(intJump), fmt(estErr),
				torp.x(), torp.y(), torp.z(), estX, estY, estZ, intX, intY, intZ, target.x(), target.y(), target.z());
	}

	private static String fmt(double v) {
		return Double.isNaN(v) ? "  -  " : String.format("%.1f", v);
	}

	private static double normalizeAngle(double a) {
		while (a > Math.PI)
			a -= 2 * Math.PI;
		while (a < -Math.PI)
			a += 2 * Math.PI;
		return a;
	}

	// ── Torpedo explosion ────────────────────────────────────────────

	private static final int EXPLOSION_BASE_DAMAGE = 1200; // instant kill at point blank, heavy damage at close range
	private static final double EXPLOSION_IMPULSE = 15.0; // m/s velocity change at zero range (before mass division)

	private static void handleDetonation(
		TorpedoEntity torp, List<SubmarineEntity> subs, MatchConfig config, boolean submarineHit) {
		torp.detonate();
		torp.setExplosionProcessed();
		double blastRadius = config.blastRadius();

		// Blast center is at the torpedo's bow (warhead in the nose)
		double cosP = Math.cos(torp.pitch()), sinP = Math.sin(torp.pitch());
		double halfLen = torp.vehicleConfig().hullHalfLength();
		double blastX = torp.x() + Math.sin(torp.heading()) * cosP * halfLen;
		double blastY = torp.y() + Math.cos(torp.heading()) * cosP * halfLen;
		double blastZ = torp.z() + sinP * halfLen;

		// Queue explosion events for all living submarines (delivered next tick)
		for (var sub : subs) {
			if (sub.hp() <= 0 || sub.forfeited())
				continue;
			double edx = blastX - sub.x();
			double edy = blastY - sub.y();
			double edz = blastZ - sub.z();
			double eRange = Math.sqrt(edx * edx + edy * edy + edz * edz);
			double eBearing = Math.atan2(edx, edy);
			if (eBearing < 0)
				eBearing += 2 * Math.PI;
			sub.addExplosionEvent(
					new ExplosionEvent(eBearing, eRange, sub.id() == torp.ownerId(), torp.id(), submarineHit));
		}

		for (var sub : subs) {
			if (sub.hp() <= 0 || sub.forfeited())
				continue;
			double dx = sub.x() - blastX;
			double dy = sub.y() - blastY;
			double dz = sub.z() - blastZ;
			double centerDist = Math.sqrt(dx * dx + dy * dy + dz * dz);

			// Use hull surface distance from blast point for damage
			double hullDist = HullGeometry.distanceToHull(blastX, blastY, blastZ, sub.x(), sub.y(), sub.z(),
					sub.heading(), sub.pitch());
			if (hullDist < blastRadius) {
				// Quadratic falloff based on distance to hull surface
				double falloff = (1.0 - hullDist / blastRadius);
				falloff *= falloff;

				// Damage
				int damage = Math.max(1, (int) (EXPLOSION_BASE_DAMAGE * falloff));
				sub.setHp(Math.max(0, sub.hp() - damage));

				// Impulse: push sub away from blast. Divide by mass so
				// heavier subs resist more. A submarine is thousands of tons,
				// so the velocity change is modest (shove, not launch).
				if (centerDist > 0.1) {
					double force = EXPLOSION_IMPULSE * falloff;
					double mass = sub.vehicleConfig().massSurge();
					double dv = force / (mass * 0.001); // scale: mass is in kg, want moderate push
					double nx = dx / centerDist, ny = dy / centerDist, nz = dz / centerDist;

					// Linear impulse: velocity change along sub heading
					double along = nx * Math.sin(sub.heading()) + ny * Math.cos(sub.heading());
					sub.setSpeed(sub.speed() + dv * along);
					sub.setVerticalSpeed(sub.verticalSpeed() + dv * nz * 0.5);

					// Angular impulse: very small. A torpedo blast nudges a
					// submarine's heading, it doesn't spin it around.
					double fwdX = Math.sin(sub.heading());
					double fwdY = Math.cos(sub.heading());
					double crossZ = nx * fwdY - ny * fwdX;
					double rotInertia = mass * sub.vehicleConfig().rotationalInertia();
					sub.setYawRate(sub.yawRate() + crossZ * force / rotInertia * 0.5);
					sub.setPitchRate(sub.pitchRate() + nz * force / rotInertia * 0.2);
				}
			}
		}
	}

	// ── Sub-to-sub collision ───────────────────────────────────────

	static void checkSubCollisions(List<SubmarineEntity> entities) {
		checkSubCollisions(entities, new CurrentField(List.of()));
	}

	static void checkSubCollisions(List<SubmarineEntity> entities, CurrentField currentField) {
		for (int i = 0; i < entities.size(); i++) {
			var a = entities.get(i);
			if (a.forfeited() || a.hp() <= 0)
				continue;
			for (int j = i + 1; j < entities.size(); j++) {
				var b = entities.get(j);
				if (b.forfeited() || b.hp() <= 0)
					continue;

				var contact = HullOverlap.contact(a, b);
				if (contact != null) {
					SubCollisionResponse.resolve(a, b, contact, currentField);
				}
			}
		}
	}

	/** Check whether the full oriented hull ellipsoids intersect, including tangent contact. */
	static boolean ellipsoidsOverlap(SubmarineEntity a, SubmarineEntity b) {
		return HullOverlap.overlaps(a, b);
	}

	public double getSpeedMultiplier() {
		return speedMultiplier;
	}

	public void setSpeedMultiplier(double m) {
		this.speedMultiplier = m;
	}

	public State getState() {
		return state;
	}

	public void setPaused(boolean p) {
		this.paused = p;
	}

	public boolean isPaused() {
		return paused;
	}

	public void stepOnce() {
		this.stepOnce = true;
	}

	public void stop() {
		this.stopped = true;
	}

	public boolean isStopped() {
		return stopped;
	}
}
