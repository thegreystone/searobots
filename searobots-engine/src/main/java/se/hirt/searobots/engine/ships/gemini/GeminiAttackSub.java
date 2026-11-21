/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.FiringSolution;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.Purpose;
import se.hirt.searobots.api.SonarTrack;
import se.hirt.searobots.api.StrategicWaypoint;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoLaunchCommand;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Waypoint;

import java.util.ArrayList;
import java.util.List;

/**
 * Green-field autonomous attack submarine controller for Gemini.
 * <p>
 * Key design pillars:
 * <ol>
 * <li><b>Silent Pulse-and-Glide Stalking &amp; Disciplined Active Pinging:</b> Cruises below the
 * thermocline with disengaged clutch glides to minimize radiated noise and maximize passive sonar
 * horizon, taking terrain-checked snapshot pings only when TMA stalls or an enemy pings first.</li>
 * <li><b>Staggered Two-Torpedo Kill-Salvo:</b> Fires a tight-fuse primary ({@code 5.0m}) followed
 * 7.2s later by a wider-fuse follow-up ({@code 13.0m}), plus a {@code 22.0m} {@code FINISHER} mode
 * against already damaged targets, eliminating enemy retaliation windows.</li>
 * <li><b>Exact-Range Fuel-Starvation Evasion:</b> Pings during flank evasion to measure exact
 * incoming torpedo ranges, holding straight-line {@code 15 m/s} sprint to starve distant torpedoes
 * of fuel and triggering crisp 3D beam-cut jukes only at genuine point-blank range (&lt;260m).</li>
 * <li><b>Island-Safe A* Navigation:</b> Projects bearing-only/stale tracks and mandatory navigation
 * objectives through {@link se.hirt.searobots.api.PathPlanner} with lateral hull clearance.</li>
 * </ol>
 */
public final class GeminiAttackSub implements SubmarineController {

	private enum CombatState {
		OBJECTIVES, PATROL, STALK_TMA, ALIGN_FIRE, TUBE_CLEAR, SALVO_WAIT, EGRESS, EVADE
	}

	private MatchContext context;
	private GeminiSubmarineHelm helm;
	private GeminiSonarSuite sonar;

	private final List<StrategicWaypoint> objectives = new ArrayList<>();
	private int objectiveIndex;
	private double navSpawnDepth = Double.NaN;

	private CombatState state = CombatState.PATROL;
	private double tmaLegSide = 1.0;
	private long lastTmaFlipTick = -10_000;
	private int tmaLegSubPhase;

	private long pendingLaunchTick = -10_000;
	private double fireAimBearing;
	private double fireAimDepth;
	private boolean salvoSecondPending;
	private long salvoSecondDueTick = -10_000;
	private double egressBearing;
	private long egressUntilTick = -10_000;
	private long lastTacticalPingTick = -10_000;

	private double evadeJukeSign = 1.0;
	private double evadeDepthSign = 1.0;
	private long jukeBurstStartTick = -10_000;
	private long jukeCooldownUntilTick = -10_000;
	private long evadeStartTick = -10_000;
	private double cachedSprintBearing = Double.NaN;
	private long lastSprintBearingTick = -100;

	private Vec3 patrolTarget;
	private int patrolSectorIndex;
	private long lastSweepTick = -10_000;
	private boolean sweepingUpperLayer;
	private long sweepStartTick;
	private long lastProbePingTick = -10_000;

	@Override
	public String name() {
		return "Gemini";
	}

	@Override
	public String shortName() {
		return "GM";
	}

	@Override
	public TorpedoController createTorpedoController() {
		return new GeminiTorpedoController();
	}

	@Override
	public void setObjectives(List<StrategicWaypoint> objectives) {
		this.objectives.clear();
		if (objectives != null) {
			this.objectives.addAll(objectives);
		}
		this.objectiveIndex = 0;
	}

	@Override
	public void onMatchStart(MatchContext context) {
		this.context = context;
		this.helm = new GeminiSubmarineHelm(context);
		this.sonar = new GeminiSonarSuite(context);
		this.state = objectives.isEmpty() ? CombatState.PATROL : CombatState.OBJECTIVES;
		this.patrolTarget = null;
		this.patrolSectorIndex = 0;
		this.lastTmaFlipTick = 0;
		this.lastSweepTick = 0;
		this.lastProbePingTick = 0;
		this.lastTacticalPingTick = -10_000;
		this.salvoSecondPending = false;
		this.navSpawnDepth = Double.NaN;
	}

	@Override
	public void onTick(SubmarineInput input, SubmarineOutput output) {
		if (helm == null) {
			return;
		}
		long tick = input.tick();
		var self = input.self();
		var pos = self.pose().position();
		if (Double.isNaN(navSpawnDepth)) {
			navSpawnDepth = pos.z();
		}

		// 1. Update sonar intelligence
		sonar.update(input, output);

		// 2. Mandatory navigation objectives mode
		if (objectiveIndex < objectives.size()) {
			state = CombatState.OBJECTIVES;
			runObjectives(input, output);
			return;
		}

		// 3. Tube-clearance hold (~3.2s)
		if (tick - pendingLaunchTick < 165) {
			state = CombatState.TUBE_CLEAR;
			helm.steerDirect(input, output, fireAimBearing, fireAimDepth, GeminiSubmarineHelm.PropulsionMode.PURE_DRIFT,
					0.0, false);
			output.setStatus("LAUNCH CLEAR");
			return;
		}

		// 4. Staggered 2nd-torpedo salvo follow-up
		var threats = sonar.threatPicture(tick, self.pose().heading(), self.surgeSpeed());
		if (salvoSecondPending) {
			if ((threats != null && threats.anyImminent()) || self.torpedoesRemaining() <= 0) {
				salvoSecondPending = false;
			} else if (tick >= salvoSecondDueTick) {
				var estOpt = sonar.bestEnemyEstimate();
				if (estOpt.isPresent()) {
					launchSingleTorpedo(input, output, estOpt.get(), GeminiMission.Mode.KILL, 13.0, false);
					salvoSecondPending = false;
					egressUntilTick = tick + 1600;
					return;
				} else {
					salvoSecondPending = false;
				}
			} else {
				state = CombatState.SALVO_WAIT;
				helm.steerDirect(input, output, fireAimBearing, fireAimDepth,
						GeminiSubmarineHelm.PropulsionMode.PURE_DRIFT, 0.0, false);
				output.setStatus("SALVO SPACING");
				return;
			}
		}

		// 5. Highest combat priority: Incoming torpedo defense
		if (threats != null) {
			if (state != CombatState.EVADE) {
				state = CombatState.EVADE;
				evadeStartTick = tick;
				evadeJukeSign = (GeminiMath.angleDiff(threats.swarmBearing(), self.pose().heading()) >= 0) ? -1.0 : 1.0;
				evadeDepthSign = (pos.z() > -230.0) ? -1.0 : 1.0;
				cachedSprintBearing = Double.NaN;
			}
			runEvade(input, output, threats);
			return;
		} else if (state == CombatState.EVADE) {
			state = CombatState.PATROL;
			helm.clearRoute();
		}

		// 6. Post-launch silent egress
		if (tick < egressUntilTick) {
			state = CombatState.EGRESS;
			double egressDepth = chooseStealthDepth(pos.x(), pos.y(), true);
			helm.steerDirect(input, output, egressBearing, egressDepth,
					GeminiSubmarineHelm.PropulsionMode.STEALTH_GLIDE, 6.2, false);
			output.setStatus("EGRESS");
			return;
		}

		// 7. Target engagement / stalking / TMA
		var estOpt = sonar.bestEnemyEstimate();
		boolean hasBearing = Double.isFinite(sonar.lastKnownBearing()) && sonar.ticksSinceLastContact(tick) < 2500;

		if (estOpt.isPresent() || hasBearing) {
			runEngage(input, output, estOpt.orElse(null));
			return;
		}

		// 8. Silent deep search & patrol
		state = CombatState.PATROL;
		runPatrol(input, output);
	}

	private void runObjectives(SubmarineInput input, SubmarineOutput output) {
		var pos = input.self().pose().position();
		var obj = objectives.get(objectiveIndex);

		output.publishStrategicWaypoint(new Waypoint(obj.x(), obj.y(), obj.preferredDepth(), true), obj.purpose());

		double floorHere = context.terrain().elevationAt(pos.x(), pos.y());
		double deepTarget = Math.max(helm.maxSafeDepth(), floorHere + 38.0);
		deepTarget = Math.min(deepTarget, Math.min(navSpawnDepth, obj.preferredDepth()));
		deepTarget = Math.max(deepTarget, helm.maxSafeDepth());

		double dist = helm.navigateTo(input, output, obj.x(), obj.y(), deepTarget,
				GeminiSubmarineHelm.PropulsionMode.NAV_GLIDE, 8.4, false);
		if (dist < Math.max(obj.arrivalRadius(), 320.0)) {
			objectiveIndex++;
			helm.clearRoute();
		}
		output.setStatus("TRANSIT WP" + (objectiveIndex + 1));
	}

	private void runEvade(SubmarineInput input, SubmarineOutput output, GeminiSonarSuite.ThreatPicture threats) {
		long tick = input.tick();
		var self = input.self();
		var pos = self.pose().position();
		double heading = self.pose().heading();

		// During flank evasion we are already cavitating at ~135 dB (zero stealth left), so ping every
		// ~11s to measure exact active ranges to every incoming torpedo (preventing premature jukes)
		// AND simultaneously refresh an exact active fix on the enemy submarine for our counter-attack!
		if (input.activeSonarCooldownTicks() <= 0 && tick - lastTacticalPingTick >= 550) {
			output.activeSonarPing();
			lastTacticalPingTick = tick;
		}

		// Crisp 8s last-second 3D beam-cut juke ONLY when a fast torpedo is genuinely point-blank (<260m).
		// At longer ranges, pure 15 m/s straight-line sprint maximizes opening speed and starves torp fuel!
		boolean inJukeBurst = tick - jukeBurstStartTick < 400;
		if (!inJukeBurst && threats.anyImminent() && tick >= jukeCooldownUntilTick) {
			jukeBurstStartTick = tick;
			jukeCooldownUntilTick = tick + 1000;
			evadeJukeSign = -evadeJukeSign;
			evadeDepthSign = (pos.z() > -220.0) ? -1.0 : 1.0;
			inJukeBurst = true;
		}

		if (inJukeBurst) {
			double cutBearing = GeminiMath
					.normalizeBearing(threats.primary().bearing + evadeJukeSign * Math.toRadians(76));
			double floor = context.terrain().elevationAt(pos.x(), pos.y());
			double targetZ = evadeDepthSign < 0 ? Math.max(helm.maxSafeDepth(), floor + 65.0)
					: Math.min(-95.0, pos.z() + 100.0);
			if (Math.abs(pos.z() - targetZ) < 20.0) {
				targetZ = Math.max(helm.maxSafeDepth(), floor + 65.0);
			}
			helm.steerDirect(input, output, cutBearing, targetZ, GeminiSubmarineHelm.PropulsionMode.FLANK_SPRINT, 15.0,
					true);
			output.setStatus("TORP JUKE");
		} else {
			if (Double.isNaN(cachedSprintBearing) || tick - lastSprintBearingTick >= 50) {
				cachedSprintBearing = helm.findBestEscapeSprintBearing(pos.x(), pos.y(), threats.swarmBearing(),
						heading);
				lastSprintBearingTick = tick;
			}
			double gentleWeave = Math.sin((tick - evadeStartTick) * 0.0025) * Math.toRadians(8);
			double runBearing = GeminiMath.normalizeBearing(cachedSprintBearing + gentleWeave);
			double deepEvadeZ = chooseStealthDepth(pos.x(), pos.y(), true);

			helm.steerDirect(input, output, runBearing, deepEvadeZ, GeminiSubmarineHelm.PropulsionMode.FLANK_SPRINT,
					15.0, true);
			output.setStatus("EVADE SPRINT");
		}
	}

	private void runEngage(SubmarineInput input, SubmarineOutput output, SonarTrack.Estimate est) {
		long tick = input.tick();
		var self = input.self();
		var pos = self.pose().position();
		double heading = self.pose().heading();
		int torpsLeft = self.torpedoesRemaining();

		double contactBearing = sonar.currentActiveEnemy() != null ? sonar.currentActiveEnemy().bearing()
				: (sonar.currentPassiveEnemy() != null ? sonar.currentPassiveEnemy().bearing() : (est != null
						? GeminiMath.bearing(pos.x(), pos.y(), est.x(), est.y()) : sonar.lastKnownBearing()));

		double estRange = est != null ? GeminiMath.dist2D(pos.x(), pos.y(), est.x(), est.y()) : Double.NaN;
		double uncRadius = est != null ? est.uncertainty().uncertaintyRadius() : Double.MAX_VALUE;
		double quality = est != null ? est.solutionQuality() : 0.0;
		double fixAge = est != null ? est.secondsSinceFix() : 999.0;

		boolean isFinisher = sonar.enemyEstimatedHp() <= 450;
		double minSafeFireRange = isFinisher ? 300.0 : 520.0;

		// 1. Evaluate firing solution matched to our torpedo's wide active-sonar acquisition envelope!
		// Prevents mid-alignment aborts while turning the bow onto the shot vector.
		boolean hasActiveFix = est != null && fixAge < 38.0 && uncRadius < 680.0 && estRange < 3600.0;
		boolean hasMaturePassive = est != null && fixAge < 20.0 && quality >= 0.60 && estRange < 3100.0
				&& uncRadius < Math.max(380.0, estRange * 0.25);
		boolean hasCloseSnap = est != null && fixAge < 18.0 && estRange < 1700.0 && quality >= 0.35
				&& uncRadius < 550.0;

		boolean firingSolutionReady = est != null && (hasActiveFix || hasMaturePassive || hasCloseSnap)
				&& estRange >= minSafeFireRange;

		// 2. Decisive, terrain-checked snapshot active ping policy (including instant counter-punch!)
		boolean enemyPingedRecently = sonar.ticksSinceEnemyPing(tick) < 350;
		boolean canPingNow = torpsLeft > 0 && input.activeSonarCooldownTicks() <= 0
				&& sonar.activeOwnTorpedoCount(tick) == 0;

		if (canPingNow) {
			long minPingSpacing = (isFinisher || enemyPingedRecently || (!Double.isNaN(estRange) && estRange < 2200.0))
					? 900L : 1800L; // 18s urgent / 36s normal
			if (tick - lastTacticalPingTick >= minPingSpacing) {
				double contSec = sonar.continuousContactSeconds(tick);
				boolean needInitialRange = (est == null || fixAge > 45.0) && contSec > 24.0;
				boolean counterPunchPing = enemyPingedRecently && (!firingSolutionReady || uncRadius > 320.0);
				boolean finishingDamaged = isFinisher && sonar.ticksSinceOwnHit(tick) < 3000 && uncRadius > 220.0;
				boolean inKillZoneLowQ = !Double.isNaN(estRange) && estRange < 3200.0 && contSec > 25.0
						&& !firingSolutionReady;
				boolean tmaStalled = contSec > 42.0 && !firingSolutionReady
						&& (Double.isNaN(estRange) || estRange < 3800.0);
				boolean refreshDuringAlign = state == CombatState.ALIGN_FIRE && fixAge > 16.0 && uncRadius > 340.0;

				if (needInitialRange || counterPunchPing || finishingDamaged || inKillZoneLowQ || tmaStalled
						|| refreshDuringAlign) {
					double probeRange = Double.isFinite(estRange) ? Math.min(estRange, 3600.0) : 2600.0;
					double tx = pos.x() + Math.sin(contactBearing) * probeRange;
					double ty = pos.y() + Math.cos(contactBearing) * probeRange;
					double tz = estimateTargetDepth(tx, ty, pos.z(), tick);
					double occDb = GeminiMath.estimateTerrainOcclusionDb(context.terrain(), pos, new Vec3(tx, ty, tz));
					if (occDb < 48.0) {
						output.activeSonarPing();
						lastTacticalPingTick = tick;
					}
				}
			}
		}

		if (firingSolutionReady) {
			output.publishFiringSolution(
					new FiringSolution(est.x(), est.y(), est.estimatedHeading(), est.estimatedSpeed(), quality));
		}

		// 3. Weapon slot check & FAST ALIGN / LAUNCH
		boolean canFireNewSalvo = torpsLeft > 0 && sonar.ticksSinceLastOwnLaunch(tick) > 400
				&& (sonar.activeOwnTorpedoCount(tick) == 0 || (isFinisher && sonar.activeOwnTorpedoCount(tick) < 2
						&& sonar.ticksSinceLastOwnLaunch(tick) > 500));

		if (firingSolutionReady && canFireNewSalvo) {
			double tgtVx = (Double.isFinite(est.estimatedHeading()) && est.estimatedSpeed() > 0)
					? est.estimatedSpeed() * Math.sin(est.estimatedHeading()) : 0.0;
			double tgtVy = (Double.isFinite(est.estimatedHeading()) && est.estimatedSpeed() > 0)
					? est.estimatedSpeed() * Math.cos(est.estimatedHeading()) : 0.0;
			double targetZ = estimateTargetDepth(est.x(), est.y(), pos.z(), tick);
			var current = input.environment().currentField().currentAt(pos.z());

			Vec3 dogleg = helm.findTorpedoDoglegWaypoint(pos.x(), pos.y(), pos.z(), est.x(), est.y(), targetZ);
			Vec3 intercept = GeminiMath.solveIntercept3D(pos.x(), pos.y(), pos.z(), current.x(), current.y(), 21.0,
					est.x(), est.y(), targetZ, tgtVx, tgtVy, 0.0, Math.min(45.0, estRange / 18.0));

			double aimX = dogleg != null ? dogleg.x() : intercept.x();
			double aimY = dogleg != null ? dogleg.y() : intercept.y();
			double desiredShotBearing = GeminiMath.bearing(pos.x(), pos.y(), aimX, aimY);
			double bowError = Math.abs(GeminiMath.angleDiff(desiredShotBearing, heading));
			double pitchAbs = Math.abs(self.pose().pitch());

			// Generous off-axis shot cone matching our torpedo's crisp turn kinematics -> shoots 10-15s sooner!
			double maxBowErr = estRange < 850.0 ? Math.toRadians(24) : Math.toRadians(38);
			boolean clearImmediateFront = GeminiMath.maxFloorAlongSegment(context.terrain(), pos.x(), pos.y(),
					pos.x() + Math.sin(heading) * 240.0, pos.y() + Math.cos(heading) * 240.0, 35.0) < pos.z() - 14.0;

			if (bowError <= maxBowErr && pitchAbs < Math.toRadians(18) && clearImmediateFront) {
				GeminiMission.Mode mode = isFinisher ? GeminiMission.Mode.FINISHER
						: (hasCloseSnap && !hasActiveFix ? GeminiMission.Mode.SNAP : GeminiMission.Mode.KILL);
				double fuseR = isFinisher ? Math.min(22.0, context.config().maxFuseRadius())
						: context.config().minFuseRadius();

				boolean scheduleFollowup = !isFinisher && torpsLeft >= 2 && (hasActiveFix || hasMaturePassive);
				launchSingleTorpedo(input, output, est, mode, fuseR, scheduleFollowup);
				return;
			} else {
				// Fast non-cavitating bow turn (7.2 m/s gives ~45% faster yaw rate than 4.8 m/s!)
				state = CombatState.ALIGN_FIRE;
				double alignDepth = chooseStealthDepth(pos.x(), pos.y(), false);
				helm.steerDirect(input, output, desiredShotBearing, alignDepth,
						GeminiSubmarineHelm.PropulsionMode.STEADY_CRUISE, 7.2, false);
				output.setStatus("ALIGN SHOT");
				return;
			}
		}

		// 4. STALK & BUILD PASSIVE TMA SILENTLY (with A* island routing when far or bearing-only!)
		state = CombatState.STALK_TMA;

		if (tick - lastTmaFlipTick > 1100) {
			tmaLegSubPhase = (tmaLegSubPhase + 1) % 3;
			if (tmaLegSubPhase == 0) {
				tmaLegSide = chooseSaferFlank(pos.x(), pos.y(), contactBearing);
			}
			lastTmaFlipTick = tick;
		}

		double stalkDepth = chooseStealthDepth(pos.x(), pos.y(), false);
		if (sonar.hasRecentDepth(tick) && sonar.ticksSinceLastContact(tick) > 150) {
			stalkDepth = Math.clamp(sonar.lastKnownDepth(), helm.maxSafeDepth(), -65.0);
		}

		// If track is missing/stale OR far away (>3000m), use A* pathfinding around islands!
		boolean needAstarApproach = est == null || fixAge > 90.0 || estRange > 3000.0;
		if (needAstarApproach) {
			double goalX, goalY;
			double flankAngle = (tmaLegSubPhase == 1 ? 0.75 : 0.30) * tmaLegSide;
			if (est != null && fixAge <= 90.0) {
				goalX = est.x() + Math.sin(GeminiMath.normalizeBearing(contactBearing + Math.PI + flankAngle)) * 1650.0;
				goalY = est.y() + Math.cos(GeminiMath.normalizeBearing(contactBearing + Math.PI + flankAngle)) * 1650.0;
			} else {
				// Bearing-only or stale track: route cleanly around islands 2400m along contactBearing!
				double approachBrg = GeminiMath.normalizeBearing(contactBearing + flankAngle * 0.45);
				goalX = pos.x() + Math.sin(approachBrg) * 2400.0;
				goalY = pos.y() + Math.cos(approachBrg) * 2400.0;
				// Clamp inside battle area
				double r = Math.hypot(goalX, goalY);
				double maxR = context.config().battleArea().extent() - 900.0;
				if (r > maxR && r > 1.0) {
					goalX *= maxR / r;
					goalY *= maxR / r;
				}
			}
			double approachSpeed = (!Double.isNaN(estRange) && estRange > 3500.0) || est == null ? 7.6 : 6.2;
			output.publishStrategicWaypoint(new Waypoint(goalX, goalY, stalkDepth, true), Purpose.INTERCEPT);
			helm.navigateTo(input, output, goalX, goalY, stalkDepth, GeminiSubmarineHelm.PropulsionMode.STEALTH_GLIDE,
					approachSpeed, false);
		} else {
			double relativeAngle;
			if (estRange > 2500.0) {
				relativeAngle = (tmaLegSubPhase == 1) ? Math.toRadians(66) : Math.toRadians(38);
			} else if (estRange < 680.0) {
				relativeAngle = Math.toRadians(118);
			} else if (estRange < 1100.0) {
				relativeAngle = (tmaLegSubPhase == 1) ? Math.toRadians(108) : Math.toRadians(86);
			} else {
				relativeAngle = switch (tmaLegSubPhase) {
				case 0 -> Math.toRadians(58);
				case 1 -> Math.toRadians(92);
				default -> Math.toRadians(70);
				};
			}
			double desiredBearing = GeminiMath.normalizeBearing(contactBearing + tmaLegSide * relativeAngle);
			helm.steerDirect(input, output, desiredBearing, stalkDepth,
					GeminiSubmarineHelm.PropulsionMode.STEALTH_GLIDE, 5.8, false);
		}
		output.setStatus(String.format("TMA Q%.0f", quality * 100.0));
	}

	private void launchSingleTorpedo(
		SubmarineInput input, SubmarineOutput output, SonarTrack.Estimate est, GeminiMission.Mode mode,
		double fuseRadius, boolean scheduleSecondSalvo) {
		long tick = input.tick();
		var self = input.self();
		var pos = self.pose().position();
		double heading = self.pose().heading();
		double contactBearing = GeminiMath.bearing(pos.x(), pos.y(), est.x(), est.y());
		double targetZ = estimateTargetDepth(est.x(), est.y(), pos.z(), tick);
		Vec3 dogleg = helm.findTorpedoDoglegWaypoint(pos.x(), pos.y(), pos.z(), est.x(), est.y(), targetZ);

		double escSide = chooseSaferFlank(pos.x(), pos.y(), contactBearing);
		double escBrg = GeminiMath.normalizeBearing(contactBearing + escSide * Math.toRadians(115));

		var mission = new GeminiMission(mode, est.x(), est.y(), targetZ, sonar.hasRecentDepth(tick),
				est.estimatedHeading(), est.estimatedSpeed(), est.uncertainty().uncertaintyRadius(), pos.x(), pos.y(),
				pos.z(), escBrg, dogleg != null ? dogleg.x() : Double.NaN, dogleg != null ? dogleg.y() : Double.NaN,
				tick);

		output.launchTorpedo(new TorpedoLaunchCommand(heading, 0.0, fuseRadius, mission.encode()));
		sonar.registerOwnLaunch(tick, pos, heading, est.x(), est.y(), fuseRadius);

		pendingLaunchTick = tick;
		fireAimBearing = heading;
		fireAimDepth = pos.z();
		egressBearing = escBrg;

		if (scheduleSecondSalvo) {
			salvoSecondPending = true;
			salvoSecondDueTick = tick + 360;
			egressUntilTick = tick + 1800;
		} else {
			egressUntilTick = tick + 1500;
		}

		state = CombatState.TUBE_CLEAR;
		helm.steerDirect(input, output, fireAimBearing, fireAimDepth, GeminiSubmarineHelm.PropulsionMode.PURE_DRIFT,
				0.0, false);
		output.setStatus("FIRE " + mode.name());
	}

	private void runPatrol(SubmarineInput input, SubmarineOutput output) {
		long tick = input.tick();
		var pos = input.self().pose().position();
		double silenceSeconds = sonar.ticksSinceLastContact(tick) * 0.02;

		double shallowTop = GeminiMath.shallowestThermoclineTop(context.thermalLayers());
		boolean hasLayer = Double.isFinite(shallowTop);
		if (hasLayer && !sweepingUpperLayer && silenceSeconds > 85.0 && (tick - lastSweepTick) > 6000) {
			sweepingUpperLayer = true;
			sweepStartTick = tick;
		}
		if (sweepingUpperLayer && (tick - sweepStartTick) > 2000) {
			sweepingUpperLayer = false;
			lastSweepTick = tick;
		}

		if (silenceSeconds > 160.0 && (tick - lastProbePingTick) > 7500 && input.activeSonarCooldownTicks() <= 0) {
			output.activeSonarPing();
			lastProbePingTick = tick;
			lastTacticalPingTick = tick;
		}

		if (patrolTarget == null || GeminiMath.dist2D(pos.x(), pos.y(), patrolTarget.x(), patrolTarget.y()) < 450.0) {
			patrolTarget = pickNextPatrolWaypoint(pos);
			helm.clearRoute();
		}

		double patrolDepth;
		if (sweepingUpperLayer && hasLayer) {
			patrolDepth = Math.clamp(shallowTop + 15.0, -140.0, -55.0);
		} else {
			patrolDepth = chooseStealthDepth(pos.x(), pos.y(), false);
		}

		output.publishStrategicWaypoint(new Waypoint(patrolTarget.x(), patrolTarget.y(), patrolDepth, true),
				Purpose.PATROL);
		helm.navigateTo(input, output, patrolTarget.x(), patrolTarget.y(), patrolDepth,
				GeminiSubmarineHelm.PropulsionMode.STEALTH_GLIDE, 6.4, false);
		output.setStatus(sweepingUpperLayer ? "LAYER SWEEP" : "SILENT PATROL");
	}

	private Vec3 pickNextPatrolWaypoint(Vec3 currentPos) {
		double extent = context.config().battleArea().extent();
		double radius = extent * 0.42;
		for (int attempt = 0; attempt < 16; attempt++) {
			double angle = ((patrolSectorIndex++ * 137.5) % 360.0) * Math.PI / 180.0;
			double r = (attempt % 2 == 0) ? radius : radius * 0.55;
			double wx = Math.sin(angle) * r;
			double wy = Math.cos(angle) * r;
			if (GeminiMath.dist2D(currentPos.x(), currentPos.y(), wx, wy) < 1600.0)
				continue;
			if (!helm.pathPlanner().isSafe(wx, wy))
				continue;
			double floor = context.terrain().elevationAt(wx, wy);
			if (floor > -160.0)
				continue;
			double wz = Math.clamp(Math.max(floor + 65.0, helm.maxSafeDepth()), helm.maxSafeDepth(), -90.0);
			return new Vec3(wx, wy, wz);
		}
		return new Vec3(0.0, 0.0, -180.0);
	}

	private double chooseStealthDepth(double x, double y, boolean preferDeepCover) {
		double floor = context.terrain().elevationAt(x, y);
		double maxPermitted = Math.max(helm.maxSafeDepth(), floor + 60.0);
		double deepBottom = GeminiMath.deepestThermoclineBottom(context.thermalLayers());

		double desired;
		if (preferDeepCover) {
			desired = helm.maxSafeDepth();
		} else if (Double.isFinite(deepBottom)) {
			desired = Math.min(deepBottom - 35.0, -240.0);
		} else {
			desired = -260.0;
		}
		return Math.clamp(Math.max(desired, maxPermitted), helm.maxSafeDepth(), -60.0);
	}

	private double estimateTargetDepth(double tx, double ty, double ownZ, long tick) {
		if (sonar.hasRecentDepth(tick)) {
			return sonar.lastKnownDepth();
		}
		double floorAtTarget = context.terrain().elevationAt(tx, ty);
		double deepBottom = GeminiMath.deepestThermoclineBottom(context.thermalLayers());
		double guess = Double.isFinite(deepBottom) ? Math.min(deepBottom - 30.0, -180.0) : -180.0;
		return Math.clamp(Math.max(guess, floorAtTarget + 45.0), context.config().ratedDepth() + 20.0, -40.0);
	}

	private double chooseSaferFlank(double x, double y, double refBearing) {
		double scorePos = scoreFlank(x, y, GeminiMath.normalizeBearing(refBearing + Math.PI / 2));
		double scoreNeg = scoreFlank(x, y, GeminiMath.normalizeBearing(refBearing - Math.PI / 2));
		return scorePos >= scoreNeg ? 1.0 : -1.0;
	}

	private double scoreFlank(double x, double y, double brg) {
		double score = 0.0;
		for (double d : new double[] {400.0, 900.0, 1400.0}) {
			double px = x + Math.sin(brg) * d;
			double py = y + Math.cos(brg) * d;
			double bDist = context.config().battleArea().distanceToBoundary(px, py);
			if (bDist < 500.0)
				score -= 400.0;
			else
				score += Math.min(bDist, 1500.0) * 0.05;
			double floor = context.terrain().elevationAt(px, py);
			if (floor > -90.0)
				score -= 350.0;
			else
				score += Math.min(-floor, 400.0) * 0.3;
		}
		return score;
	}
}
