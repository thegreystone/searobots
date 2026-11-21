/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.ExplosionEvent;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.SonarTrack;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.api.Vec2;
import se.hirt.searobots.api.Vec3;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

/**
 * Acoustic intelligence suite providing dual-anchor enemy submarine tracking, explosion hit
 * confirmation and HP tracking, own-torpedo launch ledger, exact submarine vs. torpedo contact
 * separation, and active-range multi-torpedo threat analysis with baffle-transition filtering.
 */
public final class GeminiSonarSuite {

	/** Tracked incoming torpedo threat. */
	public static final class TorpedoThreat {
		public double bearing;
		public double signalExcess;
		public double maxObservedSe;
		public double smoothedSeRate; // dB/s (positive = closing!)
		public double smoothedBearingRate; // rad/s (near zero = collision course!)
		public double estimatedSpeed;
		public double activeRange = Double.NaN;
		public double closingSpeed = Double.NaN; // m/s (positive = closing!)
		public long activeRangeTick = -10_000;
		public long firstSeenTick;
		public long lastSeenTick;
		public long lastPingTick = -10_000;
		public boolean possiblyOwnDeparting;
		public boolean isActiveThreat;
		public double threatScore;

		private double prevSe;
		private double prevBearing;
		private boolean prevInBaffles;
		private long prevSampleTick;
	}

	/** Aggregated threat summary across all active incoming torpedoes. */
	public record ThreatPicture(TorpedoThreat primary, double swarmBearing, int activeCount, double maxSe,
			double maxSeRate, double maxSpeed, boolean anyImminent) {
	}

	/** Record of an own torpedo launch for departing-weapon filtering and hit/salvo tracking. */
	public static final class OwnTorpedoRecord {
		public final long launchTick;
		public final Vec3 launchPos;
		public final double launchBearing;
		public final double targetX, targetY;
		public final double fuseRadius;
		public final long expectedFlightTicks;
		public boolean active = true;

		OwnTorpedoRecord(long launchTick, Vec3 launchPos, double launchBearing, double targetX, double targetY,
				double fuseRadius, long expectedFlightTicks) {
			this.launchTick = launchTick;
			this.launchPos = launchPos;
			this.launchBearing = launchBearing;
			this.targetX = targetX;
			this.targetY = targetY;
			this.fuseRadius = fuseRadius;
			this.expectedFlightTicks = expectedFlightTicks;
		}
	}

	private final MatchContext context;
	private final SonarTrack passiveTrack;
	private final SonarTrack anchorTrack;

	private SonarContact currentPassiveEnemy;
	private SonarContact currentActiveEnemy;

	private double lastKnownBearing = Double.NaN;
	private long lastContactTick = -10_000;
	private long firstContinuousContactTick = -1;
	private double lastKnownDepth = Double.NaN;
	private long lastDepthTick = -10_000;
	private long lastEnemyPingTick = -10_000;
	private long lastOwnHitTick = -10_000;
	private int enemyEstimatedHp;
	private double contactAliveBelief = 0.0;

	private final List<TorpedoThreat> torpedoThreats = new ArrayList<>();
	private final List<OwnTorpedoRecord> ownTorpedoes = new ArrayList<>();

	public GeminiSonarSuite(MatchContext context) {
		this.context = context;
		this.passiveTrack = new SonarTrack(context.config().maxSubSpeed());
		this.anchorTrack = new SonarTrack(context.config().maxSubSpeed());
		this.enemyEstimatedHp = context.config().startingHp();
	}

	public void registerOwnLaunch(
		long tick, Vec3 pos, double heading, double targetX, double targetY, double fuseRadius) {
		double dist = GeminiMath.dist2D(pos.x(), pos.y(), targetX, targetY);
		long expectedTicks = (long) ((dist / 19.0 + 45.0) * 50.0);
		ownTorpedoes.add(new OwnTorpedoRecord(tick, pos, heading, targetX, targetY, fuseRadius,
				Math.clamp(expectedTicks, 3500L, 12_500L)));
	}

	public int activeOwnTorpedoCount(long currentTick) {
		int count = 0;
		for (var r : ownTorpedoes) {
			if (r.active && currentTick - r.launchTick < r.expectedFlightTicks) {
				count++;
			}
		}
		return count;
	}

	public long ticksSinceLastOwnLaunch(long currentTick) {
		if (ownTorpedoes.isEmpty())
			return 1_000_000;
		return currentTick - ownTorpedoes.getLast().launchTick;
	}

	public void update(SubmarineInput input, SubmarineOutput output) {
		long tick = input.tick();
		double dt = input.deltaTimeSeconds();
		var pos = input.self().pose().position();
		double ownHeading = input.self().pose().heading();
		double ownSpeed = input.self().surgeSpeed();

		currentPassiveEnemy = null;
		currentActiveEnemy = null;

		// 1. Handle explosion events first
		processExplosions(tick, pos, input.explosionEvents());

		// 2. Partition passive contacts into enemy submarine vs torpedoes (ZERO false positives!)
		List<SonarContact> torpContacts = new ArrayList<>();
		for (var c : input.sonarContacts()) {
			if (isPassiveTorpedoContact(c)) {
				torpContacts.add(c);
			} else {
				if (c.signalExcess() > 70.0) {
					lastEnemyPingTick = tick;
				}
				if (currentPassiveEnemy == null || scoreSubContact(c) > scoreSubContact(currentPassiveEnemy)) {
					currentPassiveEnemy = c;
				}
			}
		}

		// 3. Partition active returns into enemy submarine vs torpedoes
		for (var c : input.activeSonarReturns()) {
			if (isActiveTorpedoContact(c)) {
				torpContacts.add(c);
			} else {
				if (currentActiveEnemy == null || c.range() < currentActiveEnemy.range()) {
					currentActiveEnemy = c;
				}
			}
		}

		// 4. Update dual-anchor enemy submarine tracks
		SonarContact bestFeed = currentActiveEnemy != null ? currentActiveEnemy : currentPassiveEnemy;
		passiveTrack.update(pos, bestFeed, dt);
		anchorTrack.update(pos, currentActiveEnemy, dt);

		if (currentActiveEnemy != null && Double.isFinite(currentActiveEnemy.estimatedDepth())) {
			lastKnownDepth = currentActiveEnemy.estimatedDepth();
			lastDepthTick = tick;
		}
		if (bestFeed != null) {
			// Tolerate brief (<15s) acoustic flickers without resetting engagement timer
			if (firstContinuousContactTick < 0 || tick - lastContactTick > 750) {
				firstContinuousContactTick = tick;
			}
			lastKnownBearing = bestFeed.bearing();
			lastContactTick = tick;
			contactAliveBelief = 1.0;

			if (currentActiveEnemy == null && currentPassiveEnemy != null
					&& Double.isFinite(currentPassiveEnemy.estimatedHeading())
					&& currentPassiveEnemy.estimatedSpeed() >= 0) {
				anchorTrack.observeMotion(currentPassiveEnemy.estimatedHeading(), currentPassiveEnemy.estimatedSpeed());
			}
		} else {
			double age = (tick - lastContactTick) * dt;
			if (age > 45.0) {
				contactAliveBelief = Math.max(0.25, Math.exp(-(age - 45.0) / 180.0));
			}
		}

		// 5. Update multi-torpedo threat tracks
		updateTorpedoThreats(tick, dt, pos, ownHeading, ownSpeed, torpContacts);

		// 6. Publish viewer contact estimate
		bestEnemyEstimate().ifPresent(est -> {
			output.publishContactEstimate(est.toContactEstimate(contactAliveBelief, "enemy"));
		});
	}

	private void processExplosions(long tick, Vec3 pos, List<ExplosionEvent> explosions) {
		if (explosions == null || explosions.isEmpty())
			return;
		for (var ev : explosions) {
			if (ev.ownTorpedo()) {
				for (var rec : ownTorpedoes) {
					if (rec.active) {
						rec.active = false;
						if (ev.submarineHit()) {
							double blastR = context.config().blastRadius();
							double falloff = Math.max(0.0, 1.0 - rec.fuseRadius / blastR);
							int estDmg = (int) (1200.0 * falloff * falloff);
							enemyEstimatedHp = Math.max(10, enemyEstimatedHp - Math.max(250, estDmg));
						}
						break;
					}
				}
			}

			TorpedoThreat closestThreat = null;
			double bestDiff = Math.toRadians(20);
			for (var t : torpedoThreats) {
				double d = Math.abs(GeminiMath.angleDiff(t.bearing, ev.bearing()));
				if (d < bestDiff) {
					bestDiff = d;
					closestThreat = t;
				}
			}
			if (closestThreat != null) {
				torpedoThreats.remove(closestThreat);
			}

			if (ev.ownTorpedo() && ev.submarineHit()) {
				lastOwnHitTick = tick;
				double ex = pos.x() + Math.sin(ev.bearing()) * ev.rangeMetres();
				double ey = pos.y() + Math.cos(ev.bearing()) * ev.rangeMetres();
				var ellipse = new SonarTrack.Ellipse(45.0, 45.0, ev.bearing());
				anchorTrack.observePosition(new Vec2(ex, ey), ellipse, Double.NaN, -1.0, 0.96);
				passiveTrack.observePosition(new Vec2(ex, ey), ellipse, Double.NaN, -1.0, 0.96);
				lastKnownBearing = ev.bearing();
				lastContactTick = tick;
				contactAliveBelief = 1.0;
			}
		}
	}

	private void updateTorpedoThreats(
		long tick, double dt, Vec3 pos, double ownHeading, double ownSpeed, List<SonarContact> torpContacts) {
		int existingCount = torpedoThreats.size();
		boolean[] matched = new boolean[existingCount];

		for (var c : torpContacts) {
			int bestIdx = -1;
			double bestDiff = Math.toRadians(16);
			for (int i = 0; i < existingCount; i++) {
				if (matched[i])
					continue;
				double diff = Math.abs(GeminiMath.angleDiff(c.bearing(), torpedoThreats.get(i).bearing));
				if (diff < bestDiff) {
					bestDiff = diff;
					bestIdx = i;
				}
			}

			boolean isPingTick = !c.isActive() && c.signalExcess() > 80.0;
			boolean inBaffles = Math.abs(GeminiMath.angleDiff(c.bearing(), ownHeading)) > Math.toRadians(128);
			TorpedoThreat t;
			if (bestIdx >= 0) {
				matched[bestIdx] = true;
				t = torpedoThreats.get(bestIdx);
				if (c.isActive() && c.range() > 0.0) {
					double rangeDt = (tick - t.activeRangeTick) * dt;
					if (Double.isFinite(t.activeRange) && rangeDt >= 2.0 && rangeDt <= 30.0) {
						double instClose = (t.activeRange - c.range()) / rangeDt;
						t.closingSpeed = Double.isFinite(t.closingSpeed) ? 0.55 * t.closingSpeed + 0.45 * instClose
								: instClose;
					}
					t.activeRange = c.range();
					t.activeRangeTick = tick;
				}
				double sampleDt = (tick - t.prevSampleTick) * dt;
				if (sampleDt >= 0.4) {
					if (!c.isActive() && !isPingTick && t.prevSe < 80.0 && inBaffles == t.prevInBaffles) {
						double dSe = (c.signalExcess() - t.prevSe) / sampleDt;
						t.smoothedSeRate = 0.70 * t.smoothedSeRate + 0.30 * dSe;
					}
					double dBrg = GeminiMath.angleDiff(c.bearing(), t.prevBearing) / sampleDt;
					t.smoothedBearingRate = 0.65 * t.smoothedBearingRate + 0.35 * dBrg;
					if (!c.isActive() && !isPingTick) {
						t.prevSe = c.signalExcess();
					}
					t.prevBearing = c.bearing();
					t.prevInBaffles = inBaffles;
					t.prevSampleTick = tick;
				}
				t.bearing = c.bearing();
				if (!c.isActive()) {
					if (!isPingTick) {
						t.signalExcess = c.signalExcess();
						if (c.signalExcess() > t.maxObservedSe)
							t.maxObservedSe = c.signalExcess();
					} else {
						t.lastPingTick = tick;
					}
				}
				if (c.estimatedSpeed() > 0)
					t.estimatedSpeed = 0.6 * t.estimatedSpeed + 0.4 * c.estimatedSpeed();
				t.lastSeenTick = tick;
			} else {
				t = new TorpedoThreat();
				t.bearing = c.bearing();
				if (c.isActive() && c.range() > 0.0) {
					t.activeRange = c.range();
					t.activeRangeTick = tick;
					t.signalExcess = 35.0;
				} else {
					t.signalExcess = isPingTick ? 35.0 : c.signalExcess();
				}
				t.maxObservedSe = t.signalExcess;
				t.estimatedSpeed = c.estimatedSpeed() > 0 ? c.estimatedSpeed() : 16.0;
				t.firstSeenTick = tick;
				t.lastSeenTick = tick;
				if (isPingTick)
					t.lastPingTick = tick;
				t.prevSe = t.signalExcess;
				t.prevBearing = c.bearing();
				t.prevInBaffles = inBaffles;
				t.prevSampleTick = tick;
				torpedoThreats.add(t);
			}

			t.possiblyOwnDeparting = isLikelyOwnDepartingTorpedo(tick, pos, t);
			evaluateThreat(tick, dt, t);
		}

		Iterator<TorpedoThreat> it = torpedoThreats.iterator();
		while (it.hasNext()) {
			var t = it.next();
			long unseen = tick - t.lastSeenTick;
			if (unseen > 900) { // 18s unseen -> expired
				it.remove();
			} else if (unseen > 0) {
				if (t.possiblyOwnDeparting && unseen > 150) {
					t.isActiveThreat = false;
					t.threatScore = 0.0;
				} else if (unseen > 250) {
					t.threatScore *= 0.996;
				}
			}
		}
	}

	private void evaluateThreat(long tick, double dt, TorpedoThreat t) {
		double age = (tick - t.firstSeenTick) * dt;
		boolean recentPing = (tick - t.lastPingTick) < 100;
		boolean closingSe = t.smoothedSeRate > 0.05;
		boolean constantBearing = Math.abs(t.smoothedBearingRate) < Math.toRadians(4.5);

		if (t.possiblyOwnDeparting && t.smoothedSeRate < 0.25 && !recentPing && t.signalExcess < 46.0) {
			t.isActiveThreat = false;
			t.threatScore = 0.0;
			return;
		}

		boolean clearlyReceding = age > 10.0 && t.smoothedSeRate < -0.35 && t.signalExcess < 36.0 && !recentPing;
		t.isActiveThreat = !clearlyReceding
				&& (t.signalExcess > 20.0 || closingSe || recentPing || (!t.possiblyOwnDeparting && age < 15.0));

		if (!t.isActiveThreat) {
			t.threatScore = 0.0;
			return;
		}

		double score = t.signalExcess;
		if (closingSe)
			score += 16.0 + Math.min(25.0, t.smoothedSeRate * 22.0);
		if (constantBearing && age > 2.5)
			score += 12.0;
		if (recentPing)
			score += 15.0;
		if (t.signalExcess > 46.0)
			score += 18.0;
		t.threatScore = score;
	}

	private boolean isLikelyOwnDepartingTorpedo(long tick, Vec3 pos, TorpedoThreat t) {
		for (var rec : ownTorpedoes) {
			if (!rec.active)
				continue;
			double elapsed = (tick - rec.launchTick) * 0.02;
			if (elapsed < 0 || elapsed > 140.0)
				continue;
			double travelDist = Math.max(0.0, elapsed - 3.0) * 21.0;
			double dirBrg = GeminiMath.bearing(rec.launchPos.x(), rec.launchPos.y(), rec.targetX, rec.targetY);
			double px = rec.launchPos.x() + Math.sin(dirBrg) * travelDist;
			double py = rec.launchPos.y() + Math.cos(dirBrg) * travelDist;
			double expectedBrg = GeminiMath.bearing(pos.x(), pos.y(), px, py);
			double distToUs = GeminiMath.dist2D(pos.x(), pos.y(), px, py);
			double brgTol = elapsed < 14.0 ? Math.toRadians(34) : Math.toRadians(18);

			if (Math.abs(GeminiMath.angleDiff(t.bearing, expectedBrg)) < brgTol
					&& (elapsed < 16.0 || (distToUs > 650.0 && t.smoothedSeRate <= 0.08))) {
				return true;
			}
		}
		return false;
	}

	/**
	 * Exact submarine vs torpedo classification for passive sonar contacts.
	 * <p>
	 * Per the {@link SonarContact} API contract:
	 * <ul>
	 * <li>Submarine contacts ALWAYS run through the engine's TMA tracker, so they always carry
	 * {@code solutionQuality > 0} or {@code rangeUncertainty > 0} (e.g. {@code Double.MAX_VALUE}
	 * through baffles). Even if a sprinting submarine's noisy blade-rate speed momentarily spikes
	 * above 18 m/s, it MUST NOT be misclassified as a torpedo.</li>
	 * <li>Passive torpedo contacts have no TMA tracker: {@code range == 0},
	 * {@code rangeUncertainty == 0}, {@code solutionQuality == 0}, and
	 * {@code estimatedSourceLevel == NaN} — regardless of whether the torpedo runs at 23 m/s or
	 * slows to 12 m/s.</li>
	 * </ul>
	 */
	private static boolean isPassiveTorpedoContact(SonarContact c) {
		if (c.solutionQuality() > 0.0 || c.rangeUncertainty() > 0.0) {
			return false; // Submarine with active TMA tracker (never a torpedo!)
		}
		return !c.isActive() && c.range() == 0.0 && c.rangeUncertainty() == 0.0 && c.solutionQuality() == 0.0
				&& Double.isNaN(c.estimatedSourceLevel());
	}

	private static boolean isActiveTorpedoContact(SonarContact c) {
		if (c.solutionQuality() > 0.0) {
			return false; // Submarine active echo sets solutionQuality = 0.95
		}
		return c.isActive() && c.solutionQuality() == 0.0;
	}

	private static double scoreSubContact(SonarContact c) {
		return c.solutionQuality() * 100.0 + c.signalExcess();
	}

	public Optional<SonarTrack.Estimate> bestEnemyEstimate() {
		var p = passiveTrack.estimate();
		var a = anchorTrack.estimate();
		if (a.isEmpty())
			return p;
		if (p.isEmpty())
			return a;
		var ae = a.get();
		var pe = p.get();
		if (ae.uncertainty().uncertaintyRadius() <= pe.uncertainty().uncertaintyRadius() * 1.15) {
			return a;
		}
		return p;
	}

	/**
	 * Aggregates all active incoming torpedo threats into a single tactical picture. Uses exact
	 * active-sonar range whenever available so Gemini NEVER breaks a winning 15 m/s fuel-starvation
	 * sprint for a premature beam juke at 800-1200m!
	 */
	public ThreatPicture threatPicture(long tick, double ownHeading, double ownSpeed) {
		TorpedoThreat primary = null;
		double sumX = 0.0, sumY = 0.0;
		int count = 0;
		double maxSe = 0.0;
		double maxRate = -10.0;
		double maxSpd = 0.0;
		boolean anyImminent = false;

		for (var t : torpedoThreats) {
			if (!t.isActiveThreat)
				continue;
			count++;
			double w = Math.max(1.0, t.threatScore);
			sumX += Math.sin(t.bearing) * w;
			sumY += Math.cos(t.bearing) * w;
			if (t.signalExcess > maxSe)
				maxSe = t.signalExcess;
			if (t.smoothedSeRate > maxRate)
				maxRate = t.smoothedSeRate;
			if (t.estimatedSpeed > maxSpd)
				maxSpd = t.estimatedSpeed;

			double relFromStern = Math
					.abs(GeminiMath.angleDiff(ownHeading, GeminiMath.normalizeBearing(t.bearing + Math.PI)));
			boolean runningStraightAwayFast = ownSpeed >= 13.5 && relFromStern < Math.toRadians(40);
			boolean hasFreshRange = Double.isFinite(t.activeRange) && (tick - t.activeRangeTick) < 800;

			if (hasFreshRange) {
				// Exact range known: ONLY juke if truly point-blank (<260m) and faster/closing!
				boolean fastOrClosing = t.estimatedSpeed > 15.3
						|| (Double.isFinite(t.closingSpeed) && t.closingSpeed > 1.5);
				if (t.activeRange < 260.0 && fastOrClosing) {
					anyImminent = true;
				}
			} else {
				// Fallback before first evasion ping: strict point-blank threshold so we hold straight 15 m/s sprint!
				boolean pointBlankFast = t.estimatedSpeed > 15.8 && t.signalExcess > 65.0 && t.smoothedSeRate > 0.70;
				boolean aheadPointBlank = !runningStraightAwayFast && t.signalExcess > 62.0 && t.smoothedSeRate > 0.80;
				if (pointBlankFast || aheadPointBlank) {
					anyImminent = true;
				}
			}
			if (primary == null || t.threatScore > primary.threatScore) {
				primary = t;
			}
		}

		if (primary == null)
			return null;
		double swarmBrg = (Math.hypot(sumX, sumY) > 1e-4) ? GeminiMath.normalizeBearing(Math.atan2(sumX, sumY))
				: primary.bearing;
		return new ThreatPicture(primary, swarmBrg, count, maxSe, maxRate, maxSpd, anyImminent);
	}

	public SonarContact currentPassiveEnemy() {
		return currentPassiveEnemy;
	}

	public SonarContact currentActiveEnemy() {
		return currentActiveEnemy;
	}

	public double lastKnownBearing() {
		return lastKnownBearing;
	}

	public long ticksSinceLastContact(long tick) {
		return tick - lastContactTick;
	}

	public double continuousContactSeconds(long tick) {
		if (firstContinuousContactTick < 0 || tick - lastContactTick > 750)
			return 0.0;
		return (tick - firstContinuousContactTick) * 0.02;
	}

	public double lastKnownDepth() {
		return lastKnownDepth;
	}

	public boolean hasRecentDepth(long tick) {
		return Double.isFinite(lastKnownDepth) && (tick - lastDepthTick) < 3000;
	}

	public long ticksSinceEnemyPing(long tick) {
		return tick - lastEnemyPingTick;
	}

	public long ticksSinceOwnHit(long tick) {
		return tick - lastOwnHitTick;
	}

	public int enemyEstimatedHp() {
		return enemyEstimatedHp;
	}
}
