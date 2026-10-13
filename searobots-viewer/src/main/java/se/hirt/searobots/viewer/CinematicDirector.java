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
package se.hirt.searobots.viewer;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.Node;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.util.*;
import java.util.function.Supplier;

/**
 * Automatic cinematic camera director. Creates dramatic, TV-broadcast-style coverage of submarine
 * battles with no player input.
 * <p>
 * The director detects events from snapshot data (torpedo launches, detonations, HP changes,
 * proximity), selects the most dramatic camera shot, and computes camera positions. Between action
 * events, it cycles through beauty shots (orbits, fly-bys, establishing shots).
 */
final class CinematicDirector {

	// ── Shot types, ordered by priority (lowest ordinal = highest priority) ──

	enum ShotType {
		DETONATION("Explosion"),
		SUB_DEATH("Sub Sinking"),
		TORPEDO_TERMINAL("Terminal Approach"),
		NEAR_MISS("Near Miss"),
		TORPEDO_LAUNCH("Torpedo Launch"),
		TORPEDO_TRACK("Torpedo Track"),
		PING_DETECTION("Ping"),
		OPENING_FLYAROUND("Opening"),
		CONTEXT_WIDE("Tactical View"),
		IDLE_HERO("Bow-quarter Pass"),
		IDLE_ORBIT("Orbit"),
		IDLE_FLY_BY("Fly-by"),
		IDLE_ESTABLISHING("Overview");

		final String label;

		ShotType(String label) {
			this.label = label;
		}
	}

	enum TransitionType {
		HARD_CUT(0f), DRAMATIC_ZOOM(0.5f), SMOOTH_PAN(1.0f), SLOW_DESCENT(2.0f), SLOW_PULLBACK(2.5f);

		final float duration;

		TransitionType(float duration) {
			this.duration = duration;
		}
	}

	record Shot(ShotType type, int subjectId, int secondaryId, float duration, float minDuration,
			TransitionType transition) {
	}

	// ── Scene access ──

	private final Supplier<List<SubmarineSnapshot>> snapshotsSupplier;
	private final Supplier<List<TorpedoSnapshot>> torpedoSnapshotsSupplier;
	private final Map<Integer, Node> subNodes;
	private final Map<Integer, Node> torpedoNodes;
	private final TerrainMap terrain; // nullable
	private final Camera camera; // nullable in headless callers

	// ── State tracking for event detection ──

	private final Set<Integer> prevTorpedoIds = new HashSet<>();
	private final Map<Integer, Integer> prevSubHp = new HashMap<>();
	private final Set<Integer> seenDetonations = new HashSet<>();
	private final Set<Integer> seenTerminal = new HashSet<>();
	private final Map<Integer, Vector3f> detonationPositions = new HashMap<>();

	// ── Current shot ──

	private Shot currentShot;
	private float shotTimer;
	private float transitionTimer;
	private boolean transitionFromAbove;
	private float overviewAge;
	private float overviewReadableTime;
	private float directorTime;
	private static final float OVERVIEW_INTERVAL = 28f;
	private static final float OVERVIEW_READABLE_HOLD = 4.5f;
	private static final float IMMINENT_APPROACH_RANGE = 160f;

	// Spline transition state: cubic Hermite for position, lookAt tracks the subject
	private final Vector3f fromPos = new Vector3f();
	private final Vector3f fromLookAt = new Vector3f();
	private final Vector3f fromTangent = new Vector3f();

	// Last frame's final camera position (after all blending), used as the
	// starting point for the next transition.
	private final Vector3f lastCamPos = new Vector3f(0, 50, 0);
	private final Vector3f lastCamLookAt = new Vector3f(0, 0, 0);

	// ── Idle sequence state ──
	// Alternate two close passes with a readable, north-up view of the battle.
	private int lastIdleEntityIndex = -1;
	private final List<Integer> idleEntityIds = new ArrayList<>();

	private enum IdlePhase {
		CLOSE_UNDERWATER, CLOSE_ALTERNATE, WIDE_OVERVIEW
	}

	private IdlePhase nextIdlePhase = IdlePhase.CLOSE_UNDERWATER;

	// ── Context shot: queued follow-up after wide view ──

	private Shot afterContextShot;

	// ── Shot-specific state ──

	private float idleOrbitAngle;
	private float closePassDelay;
	private float closeOrbitSide = 1f;
	private final Vector3f flyByStation = new Vector3f();
	private final Vector3f flyByDirection = new Vector3f();
	private final Vector3f terminalSide = new Vector3f();
	private final Vector3f detonationPos = new Vector3f();
	private float detonationPullback;

	// ── Pending shots (priority queue) ──

	private final PriorityQueue<Shot> pendingShots = new PriorityQueue<>(
			Comparator.comparingInt(s -> s.type.ordinal()));
	private final Map<Shot, Float> pendingSince = new HashMap<>();

	// ── Water opacity (establishing shots fade water out to see subs) ──
	// 1.0 = fully visible (normal), 0.0 = fully transparent/disabled
	private float waterOpacity = 1f;
	private float waterOpacityTarget = 1f;
	private float descentHoldTimer = 0f; // holds camera above surface while water fades in
	private static final float WATER_FADE_OUT_SPEED = 0.6f; // ~1.7s to go transparent
	private static final float WATER_FADE_IN_SPEED = 0.3f; // ~3.3s to go opaque (underwater-to-underwater)
	private static final float WATER_FADE_IN_DESCENT_SPEED = 1.5f; // ~0.67s when descending from overhead
	private static final float DESCENT_HOLD_DURATION = 0.8f; // hold above surface while water fades in

	// ── Opening fly-around state ──

	private float openingAngle;

	// ── Overview entry direction (true = entered from above, false = from below) ──
	private boolean overviewEnteredFromAbove = false;

	// ── Smoothed overview centroid (prevents jumps when entities appear/disappear) ──
	private final Vector3f smoothedCentroid = new Vector3f();
	private boolean centroidInitialized = false;

	// ── Terminal approach tracking (detect miss vs hit) ──
	private float terminalClosestDist = Float.MAX_VALUE;
	private float terminalPrevDist = Float.MAX_VALUE;
	private float terminalRecedingTime;

	// ── Random for idle variety ──

	private final Random rng = new Random();

	CinematicDirector(Supplier<List<SubmarineSnapshot>> snapshots, Supplier<List<TorpedoSnapshot>> torpedoSnapshots,
			Map<Integer, Node> subNodes, Map<Integer, Node> torpedoNodes, TerrainMap terrain) {
		this(snapshots, torpedoSnapshots, subNodes, torpedoNodes, terrain, null);
	}

	CinematicDirector(Supplier<List<SubmarineSnapshot>> snapshots, Supplier<List<TorpedoSnapshot>> torpedoSnapshots,
			Map<Integer, Node> subNodes, Map<Integer, Node> torpedoNodes, TerrainMap terrain, Camera camera) {
		this.snapshotsSupplier = snapshots;
		this.torpedoSnapshotsSupplier = torpedoSnapshots;
		this.subNodes = subNodes;
		this.torpedoNodes = torpedoNodes;
		this.terrain = terrain;
		this.camera = camera;
	}

	/**
	 * Called each frame. Detects events, evaluates shot changes, computes camera position. Returns
	 * the entity ID the camera is tracking (for HUD).
	 */
	int update(float tpf, Vector3f outPos, Vector3f outLookAt) {
		var snapshots = snapshotsSupplier.get();
		var torpSnapshots = torpedoSnapshotsSupplier.get();

		refreshEntityList(snapshots, torpSnapshots);
		directorTime += tpf;
		overviewAge += tpf;
		detectEvents(snapshots, torpSnapshots);
		prunePendingShots(snapshots, torpSnapshots);
		evaluateShot(tpf);

		if (currentShot == null) {
			// Start with a cinematic fly-around of the battle area:
			// camera circles the arena at the water surface, looking at the center.
			// Then smoothly descends to the first entity.
			int firstId = idleEntityIds.isEmpty() ? 0 : idleEntityIds.getFirst();
			lastIdleEntityIndex = 0;
			nextIdlePhase = IdlePhase.CLOSE_ALTERNATE;
			openingAngle = 0;
			afterContextShot = makeCloseShot(firstId);
			// Duration: one full lap. Arena radius ~7000m, at a nice angular speed
			// ~12 seconds for a full 360-degree sweep.
			currentShot = new Shot(ShotType.OPENING_FLYAROUND, firstId, -1, 12f, 5f, TransitionType.HARD_CUT);
			fromPos.set(0, 50, 0);
			fromLookAt.set(0, 0, 0);
		}

		shotTimer += tpf;
		computeShot(tpf, snapshots, torpSnapshots, outPos, outLookAt);

		// Water opacity: fade out for overhead shots (camera above water),
		// fade back in for all other shots. The fade-in is delayed until the
		// camera transition is mostly complete, so the water reappears while
		// the camera is already descending (smooth re-entry, not a hard pop).
		boolean overheadShot = currentShot.type == ShotType.IDLE_ESTABLISHING
				|| currentShot.type == ShotType.CONTEXT_WIDE;
		// When entering from above, disable water immediately (already above it).
		// When entering from below, wait 1 second for the camera to rise first.
		float fadeDelay = overviewEnteredFromAbove ? 0f : 1.0f;
		waterOpacityTarget = (overheadShot && shotTimer > fadeDelay) ? 0f : 1f;
		if (waterOpacity > waterOpacityTarget) {
			// Fading out (going transparent): normal speed
			waterOpacity = Math.max(waterOpacity - WATER_FADE_OUT_SPEED * tpf, waterOpacityTarget);
		} else if (waterOpacity < waterOpacityTarget) {
			// Fading in (becoming opaque). Two cases:
			// 1. Descending from overhead: fade in immediately and fast so water is fully
			//    opaque before the camera crosses the surface — the reverse of the fade-out
			//    delay used when rising from below.
			// 2. Transitioning between underwater shots: wait until nearly done to avoid
			//    popping water in while the spline briefly arcs through the surface.
			boolean descendingFromOverhead = transitionFromAbove;
			if (descendingFromOverhead) {
				waterOpacity = Math.min(waterOpacity + WATER_FADE_IN_DESCENT_SPEED * tpf, waterOpacityTarget);
			} else if (transitionTimer <= 0.3f) {
				waterOpacity = Math.min(waterOpacity + WATER_FADE_IN_SPEED * tpf, waterOpacityTarget);
			}
		}

		// Transition blending between shots.
		if (transitionTimer > 0) {
			float totalDur = currentShot.transition.duration;
			float raw = totalDur > 0 ? 1f - transitionTimer / totalDur : 1f;
			// Quintic smoothstep for smooth ease in/out
			float t = raw * raw * raw * (raw * (raw * 6f - 15f) + 10f);

			// Save the shot's computed destination before overwriting
			float destX = outPos.x, destY = outPos.y, destZ = outPos.z;
			float destLX = outLookAt.x, destLY = outLookAt.y, destLZ = outLookAt.z;

			boolean bothOverhead = !isUnderwaterShot(currentShot.type) && fromPos.y > 10f;

			if (bothOverhead) {
				// Overhead-to-overhead: simple linear lerp with easing.
				outPos.set(fromPos.x + (destX - fromPos.x) * t, fromPos.y + (destY - fromPos.y) * t,
						fromPos.z + (destZ - fromPos.z) * t);
				// Blend lookAt too (don't force a fixed Y, let it transition smoothly)
				outLookAt.set(fromLookAt.x + (destLX - fromLookAt.x) * t, fromLookAt.y + (destLY - fromLookAt.y) * t,
						fromLookAt.z + (destLZ - fromLookAt.z) * t);

			} else {
				// Spline transition: cubic Hermite for underwater/mixed transitions
				float t2 = t * t, t3 = t2 * t;
				float h00 = 2 * t3 - 3 * t2 + 1;
				float h10 = t3 - 2 * t2 + t;
				float h01 = -2 * t3 + 3 * t2;
				float h11 = t3 - t2;

				float transDist = fromPos.distance(new Vector3f(destX, destY, destZ));
				float tangentScale = Math.max(transDist * 0.4f, 30f);
				Vector3f t0 = new Vector3f(fromTangent).multLocal(tangentScale);

				Vector3f arriveDir = new Vector3f(destLX - destX, destLY - destY, destLZ - destZ);
				if (arriveDir.lengthSquared() > 0.01f)
					arriveDir.normalizeLocal();
				else
					arriveDir.set(0, 0, -1);
				Vector3f t1 = arriveDir.mult(tangentScale);

				outPos.set(h00 * fromPos.x + h10 * t0.x + h01 * destX + h11 * t1.x,
						h00 * fromPos.y + h10 * t0.y + h01 * destY + h11 * t1.y,
						h00 * fromPos.z + h10 * t0.z + h01 * destZ + h11 * t1.z);

				outLookAt.set(fromLookAt.x + (destLX - fromLookAt.x) * t, fromLookAt.y + (destLY - fromLookAt.y) * t,
						fromLookAt.z + (destLZ - fromLookAt.z) * t);
			}

			// Progressively update the start point toward the blended position.
			// This absorbs entity movement so the transition doesn't jitter
			// when the destination keeps shifting under a moving subject.
			float absorb = Math.min(tpf * 2f, 0.1f);
			fromPos.interpolateLocal(outPos, absorb);
			fromLookAt.interpolateLocal(outLookAt, absorb);

			descentHoldTimer = Math.max(0, descentHoldTimer - tpf);
			if (descentHoldTimer <= 0) {
				transitionTimer = Math.max(0, transitionTimer - tpf);
			}
		}

		// Underwater-to-underwater splines may skim the surface. A descent from an
		// overview must retain its altitude until the water has faded back in.
		if (isUnderwaterShot(currentShot.type) && (transitionTimer <= 0 || !transitionFromAbove)) {
			outPos.y = Math.min(outPos.y, transitionTimer <= 0 ? -3f : 5f);
		}

		// Terrain clearance takes precedence where there is no room below the surface.
		if (terrain != null) {
			float simX = outPos.x;
			float simY = -outPos.z;
			double floor = terrain.elevationAt(simX, simY);
			float minY = (float) floor + 10f; // JME Y = sim Z (up)
			if (outPos.y < minY)
				outPos.y = minY;
		}

		if (currentShot.type == ShotType.CONTEXT_WIDE && transitionTimer <= 0 && waterOpacity <= 0.05f) {
			overviewReadableTime += tpf;
			if (overviewReadableTime >= OVERVIEW_READABLE_HOLD)
				overviewAge = 0;
		}
		// Save the final camera position for next frame's transition start
		lastCamPos.set(outPos);
		lastCamLookAt.set(outLookAt);

		return currentShot.subjectId;
	}

	String currentShotLabel() {
		return currentShot != null ? currentShot.type.label : "---";
	}

	/**
	 * Water opacity: 1.0 = normal rendering, 0.0 = fully transparent. Establishing shots smoothly
	 * fade to 0 so the subs become visible from above. Other shots fade back to 1.0.
	 */
	float waterOpacity() {
		return waterOpacity;
	}

	// ── Entity list for idle cycling ──

	private void refreshEntityList(List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
		idleEntityIds.clear();
		for (var s : subs)
			idleEntityIds.add(s.id());
		for (var t : torps) {
			if (t.alive())
				idleEntityIds.add(t.id());
		}
		// Keep the index valid if entities disappeared
		if (lastIdleEntityIndex >= idleEntityIds.size()) {
			lastIdleEntityIndex = Math.max(0, idleEntityIds.size() - 1);
		}
	}

	private boolean isTorpedoEntity(int id) {
		return id >= 1000; // torpedo IDs start at 1000
	}

	// ── Event Detection ──

	private void detectEvents(List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
		// Torpedo launches
		Set<Integer> currentTorpIds = new HashSet<>();
		for (var ts : torps) {
			currentTorpIds.add(ts.id());
			if (ts.alive() && !prevTorpedoIds.contains(ts.id())) {
				// New torpedo: show launcher briefly, then track torpedo
				pendingShots.removeIf(s -> (s.type == ShotType.TORPEDO_LAUNCH && s.subjectId == ts.ownerId())
						|| s.type == ShotType.TORPEDO_TRACK);
				enqueueShot(new Shot(ShotType.TORPEDO_LAUNCH, ts.ownerId(), ts.id(), 3f, 2f, TransitionType.HARD_CUT));
				enqueueShot(new Shot(ShotType.TORPEDO_TRACK, ts.id(), -1, 8f, 2f, TransitionType.SMOOTH_PAN));
			}
		}
		prevTorpedoIds.clear();
		prevTorpedoIds.addAll(currentTorpIds);

		// Torpedo terminal phase + detonations
		for (var ts : torps) {
			if (ts.detonated() && seenDetonations.add(ts.id())) {
				var torpNode = torpedoNodes.get(ts.id());
				Vector3f position = torpNode != null ? torpNode.getLocalTranslation().clone()
						: new Vector3f((float) ts.pose().position().x(), (float) ts.pose().position().z(),
								(float) -ts.pose().position().y());
				detonationPositions.put(ts.id(), position);
				removeWeaponFollowups(ts.id());

				// Check if this detonation is near a sub (a hit vs terrain impact)
				int nearestSub = findNearestSub(subs, ts.pose().position().x(), ts.pose().position().y());
				double nearestDist = Double.MAX_VALUE;
				for (var s : subs) {
					double d = ts.pose().position().distanceTo(s.pose().position());
					if (d < nearestDist)
						nearestDist = d;
				}
				boolean isSubHit = nearestDist < 80; // within blast effect range

				// Sub hit: preempt any ongoing detonation camera (force min duration 0)
				// Terrain hit: normal priority, can be preempted by sub hits
				float minDur = isSubHit ? 0f : 3.0f;
				if (isSubHit && currentShot != null && currentShot.type == ShotType.DETONATION) {
					// Force the current terrain detonation to expire immediately
					shotTimer = currentShot.duration;
				}

				enqueueShot(new Shot(ShotType.DETONATION, ts.id(), nearestSub, 4.0f, minDur, TransitionType.HARD_CUT));
			}

			if (ts.alive() && seenTerminal.contains(ts.id()) && subs.stream().filter(s -> s.id() != ts.ownerId())
					.allMatch(s -> ts.pose().position().distanceTo(s.pose().position()) > 700)) {
				seenTerminal.remove(ts.id());
			}
			if (ts.alive() && !seenTerminal.contains(ts.id())) {
				// Check distance to nearest enemy sub (not the torpedo's intercept estimate)
				var torpPos = ts.pose().position();
				for (var s : subs) {
					if (s.id() == ts.ownerId())
						continue; // skip the launcher
					var sp = s.pose().position();
					double dx = torpPos.x() - sp.x(), dy = torpPos.y() - sp.y(), dz = torpPos.z() - sp.z();
					double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
					if (dist < 600) {
						seenTerminal.add(ts.id());
						removeWeaponFollowups(ts.id());
						enqueueShot(new Shot(ShotType.TORPEDO_TERMINAL, ts.id(), s.id(), 30f, 2f,
								TransitionType.DRAMATIC_ZOOM));
						break;
					}
				}
			}
		}

		// HP changes (hits and deaths)
		for (var s : subs) {
			int prev = prevSubHp.getOrDefault(s.id(), 1000);
			if (s.hp() < prev && s.hp() > 0) {
				pendingShots.removeIf(shot -> shot.type == ShotType.NEAR_MISS && shot.subjectId == s.id());
				// Hit: show the hit sub
				enqueueShot(new Shot(ShotType.NEAR_MISS, s.id(), -1, 3f, 1f, TransitionType.HARD_CUT));
			}
			if (s.hp() <= 0 && prev > 0) {
				pendingShots.removeIf(shot -> shot.type == ShotType.NEAR_MISS && shot.subjectId == s.id());
				// Death: dramatic slow orbit
				enqueueShot(new Shot(ShotType.SUB_DEATH, s.id(), -1, 5f, 2f, TransitionType.SLOW_PULLBACK));
			}
			prevSubHp.put(s.id(), s.hp());
		}
	}

	private void enqueueShot(Shot shot) {
		pendingShots.add(shot);
		pendingSince.put(shot, directorTime);
	}

	private void removeWeaponFollowups(int torpedoId) {
		pendingShots.removeIf(s -> ((s.type == ShotType.TORPEDO_TRACK || s.type == ShotType.TORPEDO_TERMINAL)
				&& s.subjectId == torpedoId) || (s.type == ShotType.TORPEDO_LAUNCH && s.secondaryId == torpedoId));
	}

	private void prunePendingShots(List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
		Set<Integer> aliveWeapons = new HashSet<>();
		for (var t : torps) {
			if (t.alive())
				aliveWeapons.add(t.id());
		}
		pendingShots.removeIf(s -> directorTime - pendingSince.getOrDefault(s, directorTime) > switch (s.type) {
		case TORPEDO_LAUNCH -> 5f;
		case TORPEDO_TRACK -> 12f;
		case NEAR_MISS -> 4f;
		case DETONATION -> 6f;
		case SUB_DEATH -> 8f;
		default -> 40f;
		});
		pendingShots.removeIf(s -> switch (s.type) {
		case TORPEDO_LAUNCH -> !aliveWeapons.contains(s.secondaryId) || !subNodes.containsKey(s.subjectId);
		case TORPEDO_TRACK -> !aliveWeapons.contains(s.subjectId) || !torpedoNodes.containsKey(s.subjectId);
		case TORPEDO_TERMINAL -> !aliveWeapons.contains(s.subjectId) || !torpedoNodes.containsKey(s.subjectId)
				|| subs.stream().noneMatch(sub -> sub.id() == s.secondaryId && sub.hp() > 0)
				|| terminalSeparation(s) > 700f;
		case NEAR_MISS -> !subNodes.containsKey(s.subjectId)
				|| subs.stream().noneMatch(sub -> sub.id() == s.subjectId && sub.hp() > 0);
		case SUB_DEATH, PING_DETECTION -> !subNodes.containsKey(s.subjectId);
		default -> false;
		});
		pendingSince.keySet().retainAll(pendingShots);
		detonationPositions.keySet().removeIf(id -> (currentShot == null || currentShot.subjectId != id)
				&& pendingShots.stream().noneMatch(s -> s.type == ShotType.DETONATION && s.subjectId == id));
	}

	// ── Shot Evaluation ──

	private void evaluateShot(float tpf) {
		// Force switch if current shot expired
		boolean expired = currentShot != null && shotTimer > currentShot.duration;
		// Can we be preempted?
		boolean canPreempt = currentShot == null || shotTimer > currentShot.minDuration || expired;
		if (currentShot != null && currentShot.type == ShotType.CONTEXT_WIDE
				&& overviewReadableTime < OVERVIEW_READABLE_HOLD) {
			canPreempt = false;
			expired = false;
		}

		Shot urgent = pendingShots.stream().filter(this::isUrgentShot)
				.min(Comparator.comparingInt(s -> s.type.ordinal())).orElse(null);
		if (urgent != null && (currentShot == null || urgent.type.ordinal() < currentShot.type.ordinal() || expired
				|| (urgent.type == ShotType.TORPEDO_TERMINAL && currentShot.type == ShotType.TORPEDO_TERMINAL
						&& !isUrgentShot(currentShot)))) {
			pendingShots.remove(urgent);
			afterContextShot = null;
			switchToShot(urgent);
			return;
		}

		if (currentShot != null && isUnderwaterShot(currentShot.type) && currentShot.type != ShotType.DETONATION
				&& findEntityNode(currentShot.subjectId) == null) {
			expired = true;
		}

		// Terminal approach: check if the torpedo resolved (missed or gone)
		if (currentShot != null && currentShot.type == ShotType.TORPEDO_TERMINAL && !expired) {
			var torps = torpedoSnapshotsSupplier.get();
			var subs = snapshotsSupplier.get();
			boolean torpGone = torps.stream().noneMatch(t -> t.id() == currentShot.subjectId && t.alive());
			boolean targetGone = subs.stream().noneMatch(s -> s.id() == currentShot.secondaryId && s.hp() > 0);

			if (torpGone || targetGone) {
				// Torpedo detonated, died, or target destroyed: hold 1.5s then move on
				if (shotTimer > currentShot.minDuration + 1.5f)
					expired = true;
			} else {
				// Track closing distance to detect a miss (torpedo receding)
				Node torpNode = torpedoNodes.get(currentShot.subjectId);
				Node targetNode = subNodes.get(currentShot.secondaryId);
				if (torpNode != null && targetNode != null) {
					float dist = torpNode.getLocalTranslation().distance(targetNode.getLocalTranslation());
					terminalClosestDist = Math.min(terminalClosestDist, dist);
					if (dist > terminalPrevDist && dist > terminalClosestDist + 5f) {
						terminalRecedingTime += tpf;
					} else {
						terminalRecedingTime = 0;
					}
					terminalPrevDist = dist;
					// Clear miss: receding for 3+ seconds after closest approach
					if (terminalRecedingTime > 3f && dist > terminalClosestDist + 100) {
						expired = true;
					}
				}
			}
		}

		// Tactical coverage has its own clock, independent of how busy the event queue is.
		// Only the few seconds around a real impact may delay an overdue overview.
		if (overviewAge >= OVERVIEW_INTERVAL && currentShot != null && currentShot.type != ShotType.CONTEXT_WIDE
				&& (expired || !isUrgentShot(currentShot)) && subNodes.size() >= 2) {
			afterContextShot = null;
			startOverview();
			return;
		}

		if (!pendingShots.isEmpty() && (canPreempt || expired)) {
			Shot next = pendingShots.peek();
			if (currentShot == null || next.type.ordinal() < currentShot.type.ordinal() || expired) {
				afterContextShot = null;
				switchToShot(pendingShots.poll());
				return;
			}
		}

		if (expired) {
			// If an overhead shot just finished with a queued follow-up,
			// transition with a slow descent back into the water
			if (afterContextShot != null
					&& (currentShot.type == ShotType.CONTEXT_WIDE || currentShot.type == ShotType.IDLE_ESTABLISHING
							|| currentShot.type == ShotType.OPENING_FLYAROUND)) {
				var next = afterContextShot;
				afterContextShot = null;
				switchToShot(new Shot(next.type, next.subjectId, next.secondaryId, next.duration, next.minDuration,
						TransitionType.SLOW_DESCENT));
			} else {
				switchToIdleShot();
			}
		}
	}

	private boolean isUrgentShot(Shot shot) {
		return shot.type == ShotType.DETONATION || shot.type == ShotType.SUB_DEATH
				|| (shot.type == ShotType.TORPEDO_TERMINAL && terminalSeparation(shot) < IMMINENT_APPROACH_RANGE);
	}

	private float terminalSeparation(Shot shot) {
		Node weapon = torpedoNodes.get(shot.subjectId);
		Node target = subNodes.get(shot.secondaryId);
		return weapon != null && target != null ? weapon.getLocalTranslation().distance(target.getLocalTranslation())
				: Float.MAX_VALUE;
	}

	private void startOverview() {
		nextIdlePhase = IdlePhase.CLOSE_UNDERWATER;
		float arrival = TransitionType.SLOW_PULLBACK.duration;
		int primarySub = currentShot != null && subNodes.containsKey(currentShot.subjectId) ? currentShot.subjectId
				: snapshotsSupplier.get().stream().filter(s -> subNodes.containsKey(s.id()))
						.mapToInt(SubmarineSnapshot::id).findFirst().orElse(-1);
		switchToShot(new Shot(ShotType.CONTEXT_WIDE, primarySub, -1, arrival + OVERVIEW_READABLE_HOLD + 1f,
				arrival + OVERVIEW_READABLE_HOLD, TransitionType.SLOW_PULLBACK));
	}

	private void switchToShot(Shot shot) {
		// If leaving an overhead shot, force a slow descent transition
		// so the camera glides back into the water smoothly.
		boolean leavingOverhead = currentShot != null && (currentShot.type == ShotType.IDLE_ESTABLISHING
				|| currentShot.type == ShotType.CONTEXT_WIDE || currentShot.type == ShotType.OPENING_FLYAROUND);
		if (leavingOverhead && shot.transition != TransitionType.HARD_CUT
				&& shot.transition != TransitionType.SLOW_DESCENT) {
			shot = new Shot(shot.type, shot.subjectId, shot.secondaryId, shot.duration, shot.minDuration,
					TransitionType.SLOW_DESCENT);
		}
		descentHoldTimer = (leavingOverhead && isUnderwaterShot(shot.type)) ? DESCENT_HOLD_DURATION : 0f;

		// Capture the actual camera position from last frame as the transition start
		fromPos.set(lastCamPos);
		fromLookAt.set(lastCamLookAt);
		transitionFromAbove = fromPos.y > 10f;

		// Track whether overhead shots are entered from above or below
		if (shot.type == ShotType.CONTEXT_WIDE || shot.type == ShotType.IDLE_ESTABLISHING) {
			overviewEnteredFromAbove = fromPos.y > 100f;
		}

		currentShot = shot;
		shotTimer = 0;
		transitionTimer = shot.transition.duration;
		overviewReadableTime = 0;
		// Save the lead-in before its timers count down, so the beauty pass starts after arrival.
		closePassDelay = transitionTimer + descentHoldTimer;
		if (shot.type == ShotType.IDLE_ORBIT || shot.type == ShotType.IDLE_HERO || shot.type == ShotType.IDLE_FLY_BY
				|| shot.type == ShotType.TORPEDO_TRACK || shot.type == ShotType.TORPEDO_LAUNCH)
			closeOrbitSide = rng.nextBoolean() ? 1f : -1f;

		// Compute tangent at transition start. For overhead shots (looking
		// straight down), use a horizontal tangent so the spline glides
		// laterally instead of arcing down and back up.
		boolean fromOverhead = fromPos.y > 10f; // above water = overhead
		if (fromOverhead) {
			Vector3f horiz = new Vector3f(fromLookAt.x - fromPos.x, 0, fromLookAt.z - fromPos.z);
			if (horiz.lengthSquared() > 0.01f)
				horiz.normalizeLocal();
			else
				horiz.set(0, 0, -1);
			fromTangent.set(horiz);
		} else {
			Vector3f lookDir = new Vector3f(fromLookAt).subtractLocal(fromPos);
			float lookDist = lookDir.length();
			if (lookDist > 0.1f)
				lookDir.divideLocal(lookDist);
			else
				lookDir.set(0, 0, -1);
			fromTangent.set(lookDir);
		}

		// Reset shot-specific state
		if (shot.type == ShotType.IDLE_FLY_BY) {
			pickFlyByStation(shot.subjectId);
		}
		if (shot.type == ShotType.DETONATION) {
			Vector3f position = detonationPositions.get(shot.subjectId);
			if (position != null)
				detonationPos.set(position);
			detonationPullback = 0;
		}
		if (shot.type == ShotType.TORPEDO_TERMINAL) {
			terminalClosestDist = Float.MAX_VALUE;
			terminalPrevDist = Float.MAX_VALUE;
			terminalRecedingTime = 0;
			Node torpedo = torpedoNodes.get(shot.subjectId);
			Node target = subNodes.get(shot.secondaryId);
			if (torpedo != null && target != null) {
				Vector3f center = torpedo.getLocalTranslation().add(target.getLocalTranslation()).multLocal(0.5f);
				terminalSide.set(lastCamPos).subtractLocal(center).setY(0);
				if (terminalSide.lengthSquared() < 0.01f)
					terminalSide.set(1, 0, 0);
				terminalSide.normalizeLocal();
			}
		}
	}

	private void switchToIdleShot() {
		if (idleEntityIds.isEmpty())
			return;
		if (nextIdlePhase == IdlePhase.WIDE_OVERVIEW && subNodes.size() >= 2) {
			startOverview();
			return;
		}
		lastIdleEntityIndex = (lastIdleEntityIndex + 1) % idleEntityIds.size();
		nextIdlePhase = nextIdlePhase == IdlePhase.CLOSE_UNDERWATER ? IdlePhase.CLOSE_ALTERNATE
				: IdlePhase.WIDE_OVERVIEW;
		switchToShot(makeCloseShot(idleEntityIds.get(lastIdleEntityIndex)));
	}

	private Shot makeCloseShot(int entityId) {
		boolean isTorp = isTorpedoEntity(entityId);
		// Torpedoes get shorter shots (they move fast, less time needed)
		float duration = isTorp ? 5f + rng.nextFloat() * 3f : 8f + rng.nextFloat() * 4f;
		ShotType[] closeTypes = {ShotType.IDLE_ORBIT, ShotType.IDLE_HERO, ShotType.IDLE_FLY_BY};
		// A distant static fly-by loses a fast, five-metre torpedo almost immediately.
		ShotType type = isTorp ? ShotType.IDLE_ORBIT : closeTypes[rng.nextInt(closeTypes.length)];
		return new Shot(type, entityId, -1, duration, 2f, TransitionType.SMOOTH_PAN);
	}

	// ── Camera Computation ──

	private void computeShot(
		float tpf, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps, Vector3f outPos, Vector3f outLookAt) {

		switch (currentShot.type) {
		case DETONATION -> computeDetonation(tpf, outPos, outLookAt);
		case SUB_DEATH -> computeSubDeath(tpf, subs, outPos, outLookAt);
		case TORPEDO_TERMINAL -> computeTorpedoTerminal(tpf, subs, torps, outPos, outLookAt);
		case NEAR_MISS -> computeChaseShot(subs, outPos, outLookAt, 150f, 30f);
		case TORPEDO_LAUNCH -> computeLaunch(subs, outPos, outLookAt);
		case TORPEDO_TRACK -> computeCloseOrbit(subs, outPos, outLookAt);
		case PING_DETECTION -> computeChaseShot(subs, outPos, outLookAt, 200f, 40f);
		case OPENING_FLYAROUND -> computeOpeningFlyaround(tpf, outPos, outLookAt);
		case CONTEXT_WIDE -> computeContextWide(tpf, subs, torps, outPos, outLookAt);
		case IDLE_ORBIT -> computeCloseOrbit(subs, outPos, outLookAt);
		case IDLE_HERO -> computeHeroPass(subs, outPos, outLookAt);
		case IDLE_FLY_BY -> computeIdleFlyBy(tpf, subs, outPos, outLookAt);
		case IDLE_ESTABLISHING -> computeIdleEstablishing(subs, outPos, outLookAt);
		}
	}

	private void computeOpeningFlyaround(float tpf, Vector3f outPos, Vector3f outLookAt) {
		// Camera circles the battle area looking toward center. Starts high
		// for a panoramic overview, gradually descends to skim the water surface.
		float arenaRadius = 5000f;
		float duration = currentShot.duration;

		// Angular speed: one full revolution over the shot duration
		openingAngle += (FastMath.TWO_PI / duration) * tpf;

		// Progress 0..1 over the shot duration
		float progress = Math.min(shotTimer / duration, 1f);
		// Smooth ease-in-out on the descent
		float eased = progress * progress * (3f - 2f * progress);

		// Start at 1500m altitude (panoramic overview), descend to 8m (skimming waves)
		float startHeight = 1500f;
		float endHeight = 8f;
		float height = startHeight + (endHeight - startHeight) * eased;

		// Also tighten the radius slightly as we descend (feels like spiraling in)
		float radius = arenaRadius - 800f * eased;

		outPos.set(radius * FastMath.sin(openingAngle), height, radius * FastMath.cos(openingAngle));

		// Look at center, aim slightly below the horizon when high,
		// at the surface when low
		float lookY = -50f * (1f - eased); // starts looking slightly down, ends at surface
		outLookAt.set(0, lookY, 0);
	}

	private void computeDetonation(float tpf, Vector3f outPos, Vector3f outLookAt) {
		// Slow orbit around the detonation point, pulling back gradually.
		// Picks a camera position with terrain LOS to the blast.
		detonationPullback += tpf * 25f;
		idleOrbitAngle += 0.5f * tpf; // slow dramatic orbit
		float dist = 80f + detonationPullback;
		float elev = 20f + detonationPullback * 0.4f; // rise slightly

		// Try the orbit position; if terrain blocks LOS, try higher
		for (int attempt = 0; attempt < 4; attempt++) {
			float camX = detonationPos.x + dist * FastMath.sin(idleOrbitAngle);
			float camY = detonationPos.y + elev;
			float camZ = detonationPos.z + dist * FastMath.cos(idleOrbitAngle);

			if (terrain != null) {
				// Check terrain height at camera position
				float floor = (float) terrain.elevationAt(camX, -camZ);
				if (camY < floor + 15f) {
					// Camera is inside terrain, go higher
					elev += 30f;
					continue;
				}
				// Simple LOS check: sample midpoint between camera and blast
				float midX = (camX + detonationPos.x) * 0.5f;
				float midZ = (camZ + detonationPos.z) * 0.5f;
				float midFloor = (float) terrain.elevationAt(midX, -midZ);
				float midCamY = (camY + detonationPos.y) * 0.5f;
				if (midCamY < midFloor + 10f) {
					// Terrain in the way, go higher
					elev += 30f;
					continue;
				}
			}
			outPos.set(camX, camY, camZ);
			outLookAt.set(detonationPos);
			return;
		}
		// Fallback: just go high above
		outPos.set(detonationPos.x + dist * FastMath.sin(idleOrbitAngle), detonationPos.y + elev + 50f,
				detonationPos.z + dist * FastMath.cos(idleOrbitAngle));
		outLookAt.set(detonationPos);
	}

	private void computeSubDeath(float tpf, List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt) {
		// Slow orbit around the dying sub
		Node node = subNodes.get(currentShot.subjectId);
		if (node == null) {
			computeIdleEstablishing(subs, outPos, outLookAt);
			return;
		}

		Vector3f subPos = node.getLocalTranslation();
		float orbitRate = 0.12f; // rad/s
		idleOrbitAngle += orbitRate * tpf;
		float dist = 85f + 40f * passProgress();
		float elev = FastMath.DEG_TO_RAD * 20f;

		outPos.set(subPos.x + dist * FastMath.cos(elev) * FastMath.sin(idleOrbitAngle),
				subPos.y + dist * FastMath.sin(elev),
				subPos.z + dist * FastMath.cos(elev) * FastMath.cos(idleOrbitAngle));
		outLookAt.set(subPos);
		frameEntities(subs, outPos, outLookAt, currentShot.subjectId);
	}

	private void computeTorpedoTerminal(
		float tpf, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps, Vector3f outPos, Vector3f outLookAt) {
		Node weapon = torpedoNodes.get(currentShot.subjectId);
		Node target = subNodes.get(currentShot.secondaryId);
		if (weapon == null || target == null) {
			computeCloseOrbit(subs, outPos, outLookAt);
			return;
		}
		Vector3f weaponPos = weapon.getLocalTranslation();
		Vector3f targetPos = target.getLocalTranslation();
		outLookAt.set(weaponPos).addLocal(targetPos).multLocal(0.5f);

		// Stay on the same side of the action axis, including after a missed pass.
		// A restrained arc keeps the weapon's direction and its target readable.
		Vector3f axis = targetPos.subtract(weaponPos).setY(0);
		if (axis.lengthSquared() > 1f) {
			Vector3f desiredSide = new Vector3f(-axis.z, 0, axis.x).normalizeLocal();
			if (desiredSide.dot(terminalSide) < 0)
				desiredSide.negateLocal();
			terminalSide.interpolateLocal(desiredSide, 1f - FastMath.exp(-tpf * 2f)).normalizeLocal();
		}
		float progress = passProgress();
		float arc = (progress - 0.5f) * FastMath.DEG_TO_RAD * 28f;
		Vector3f along = new Vector3f(terminalSide.z, 0, -terminalSide.x);
		Vector3f offset = terminalSide.mult(FastMath.cos(arc)).addLocal(along.mult(FastMath.sin(arc)));
		float range = Math.max(80f, weaponPos.distance(targetPos) * 0.6f + 45f);
		outPos.set(outLookAt).addLocal(offset.mult(range));
		outPos.y += 12f + 12f * progress;
		frameEntities(subs, outPos, outLookAt, currentShot.subjectId, currentShot.secondaryId);
	}

	private void computeLaunch(List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt) {
		Node launcher = subNodes.get(currentShot.subjectId);
		if (launcher == null) {
			computeCloseOrbit(subs, outPos, outLookAt);
			return;
		}
		Vector3f forward = launcher.getLocalRotation().mult(new Vector3f(0, -1, 0));
		Vector3f side = launcher.getLocalRotation().mult(Vector3f.UNIT_X).multLocal(closeOrbitSide);
		outLookAt.set(launcher.getLocalTranslation()).addLocal(forward.mult(12f));
		outPos.set(launcher.getLocalTranslation()).addLocal(forward.mult(65f)).addLocal(side.mult(75f));
		outPos.y += 10f;
		frameEntities(subs, outPos, outLookAt, currentShot.subjectId, currentShot.secondaryId);
	}

	private void computeChaseShot(
		List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt, float asternDist, float aboveHeight) {
		Node node = subNodes.get(currentShot.subjectId);
		if (node == null) {
			// Might be a torpedo ID that got used for launch shot
			node = torpedoNodes.get(currentShot.subjectId);
		}
		if (node == null) {
			outPos.set(0, 100, 0);
			outLookAt.set(0, 0, 0);
			return;
		}

		Vector3f pos = node.getLocalTranslation();

		// Find heading from snapshot
		float heading = 0;
		for (var s : subs) {
			if (s.id() == currentShot.subjectId) {
				heading = (float) s.pose().heading();
				break;
			}
		}

		float fwdX = FastMath.sin(heading);
		float fwdZ = -FastMath.cos(heading);

		outPos.set(pos.x - fwdX * asternDist, pos.y + aboveHeight, pos.z - fwdZ * asternDist);
		outLookAt.set(pos);
	}

	private void computeContextWide(
		float tpf, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps, Vector3f outPos, Vector3f outLookAt) {
		Vector3f center = new Vector3f();
		float minX = Float.MAX_VALUE, maxX = -Float.MAX_VALUE;
		float minZ = Float.MAX_VALUE, maxZ = -Float.MAX_VALUE;
		List<Integer> framedIds = new ArrayList<>();
		for (var sub : subs) {
			Node node = subNodes.get(sub.id());
			if (node == null)
				continue;
			Vector3f position = node.getLocalTranslation();
			center.addLocal(position);
			minX = Math.min(minX, position.x);
			maxX = Math.max(maxX, position.x);
			minZ = Math.min(minZ, position.z);
			maxZ = Math.max(maxZ, position.z);
			framedIds.add(sub.id());
		}
		if (framedIds.isEmpty()) {
			outPos.set(0, 300, 0);
			outLookAt.set(0, 0, 0);
			return;
		}
		center.divideLocal(framedIds.size());
		if (!centroidInitialized) {
			smoothedCentroid.set(center);
			centroidInitialized = true;
		} else {
			smoothedCentroid.interpolateLocal(center, 1f - FastMath.exp(-tpf * 1.5f));
		}

		// Include weapons crossing the battle, but don't let distant misses pull
		// the framing away from the two submarines that establish its geography.
		for (var torpedo : torps) {
			Node node = torpedoNodes.get(torpedo.id());
			if (!torpedo.alive() || node == null)
				continue;
			Vector3f position = node.getLocalTranslation();
			if (position.x >= minX - 100 && position.x <= maxX + 100 && position.z >= minZ - 100
					&& position.z <= maxZ + 100)
				framedIds.add(torpedo.id());
		}
		outLookAt.set(smoothedCentroid);
		Vector3f direction = new Vector3f(0, 1, 0.18f).normalizeLocal();
		float minimumRange = Math.max(400f, (280f - outLookAt.y) / direction.y);
		outPos.set(outLookAt).addLocal(direction.mult(minimumRange));
		frameEntities(subs, outPos, outLookAt, framedIds.stream().mapToInt(Integer::intValue).toArray());
		float pullback = 1f + 0.08f * passProgress();
		outPos.subtractLocal(outLookAt).multLocal(pullback).addLocal(outLookAt);
	}

	private void computeCloseOrbit(List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt) {
		Node node = findEntityNode(currentShot.subjectId);
		if (node == null) {
			outPos.set(0, 200, 0);
			outLookAt.set(0, 0, 0);
			return;
		}

		boolean isTorp = isTorpedoEntity(currentShot.subjectId);
		// A full push-in / pull-back after the transition, even on a short torpedo shot.
		float passDuration = Math.max(currentShot.duration - closePassDelay, 0.1f);
		float progress = passProgress();
		float pushIn = 0.5f - 0.5f * FastMath.cos(FastMath.TWO_PI * progress);
		float nearRange = isTorp ? 9f : 65f;
		float farRange = isTorp ? 16f : 100f;
		// Surface ships are twice the submarine's length and need room for the entire hull.
		boolean surfaceShip = subs.stream().anyMatch(s -> s.id() == currentShot.subjectId && s.surfaceLocked());
		float dist = (farRange + (nearRange - farRange) * pushIn) * (surfaceShip ? 2f : 1f);

		// Cross the beam at the closest point, where the sail code and hull name are most readable.
		// Use the rendered rotation so close views follow the same smoothing as the model.
		Vector3f side = node.getLocalRotation().mult(Vector3f.UNIT_X).multLocal(closeOrbitSide);
		float beamAngle = FastMath.atan2(side.x, side.z);
		float orbitRate = isTorp ? 0.25f : 0.15f;
		float angle = beamAngle + (progress - 0.5f) * passDuration * orbitRate;
		float elev = FastMath.DEG_TO_RAD * (6f + 16f * progress);
		Vector3f pos = node.getLocalTranslation();
		outLookAt.set(pos.x, pos.y + (isTorp ? 0f : 2f), pos.z);
		outPos.set(outLookAt.x + dist * FastMath.cos(elev) * FastMath.sin(angle),
				outLookAt.y + dist * FastMath.sin(elev), outLookAt.z + dist * FastMath.cos(elev) * FastMath.cos(angle));
		frameEntities(subs, outPos, outLookAt, currentShot.subjectId);
	}

	private void computeHeroPass(List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt) {
		Node node = findEntityNode(currentShot.subjectId);
		if (node == null) {
			computeCloseOrbit(subs, outPos, outLookAt);
			return;
		}
		float progress = passProgress();
		float eased = progress * progress * (3f - 2f * progress);
		float scale = subs.stream().anyMatch(s -> s.id() == currentShot.subjectId && s.surfaceLocked()) ? 2f : 1f;
		Vector3f forward = node.getLocalRotation().mult(new Vector3f(0, -1, 0));
		Vector3f side = node.getLocalRotation().mult(Vector3f.UNIT_X).multLocal(closeOrbitSide);
		outLookAt.set(node.getLocalTranslation()).addLocal(forward.mult(6f));
		outLookAt.y += 2f;
		// Sweep from the low bow quarter towards the beam, with a small crane rise.
		outPos.set(node.getLocalTranslation()).addLocal(forward.mult((70f - 95f * eased) * scale))
				.addLocal(side.mult((78f - 12f * FastMath.sin(FastMath.PI * progress)) * scale));
		outPos.y += (6f + 18f * eased) * scale;
		frameEntities(subs, outPos, outLookAt, currentShot.subjectId);
	}

	private void computeIdleFlyBy(float tpf, List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt) {
		Node node = findEntityNode(currentShot.subjectId);
		if (node == null) {
			computeCloseOrbit(subs, outPos, outLookAt);
			return;
		}
		float progress = passProgress();
		// A camera station ahead of the bow lets the sub pass by close enough to
		// read the hull. A modest lateral dolly and crane keep the pass alive.
		outPos.set(flyByStation).addLocal(flyByDirection.mult(16f * progress));
		outPos.y += 10f * progress;
		outLookAt.set(node.getLocalTranslation());
		outLookAt.y += 2f;
		frameEntities(subs, outPos, outLookAt, currentShot.subjectId);
	}

	private void computeIdleEstablishing(List<SubmarineSnapshot> subs, Vector3f outPos, Vector3f outLookAt) {
		Node node = findEntityNode(currentShot.subjectId);
		if (node == null) {
			outPos.set(0, 200, 500);
			outLookAt.set(0, 0, 0);
			return;
		}

		Vector3f subPos = node.getLocalTranslation();
		// Close top-down with dynamic altitude.
		// From below: rise from 100m to 180m (pulling up from underwater).
		// From above: descend from 180m to 100m (zooming in from overview).
		float progress = Math.min(shotTimer / currentShot.duration, 1f);
		float height;
		if (overviewEnteredFromAbove) {
			height = 180f - 80f * progress;
		} else {
			height = 100f + 80f * progress;
		}
		float southOffset = 50f;
		outPos.set(subPos.x, height, subPos.z + southOffset);
		outLookAt.set(subPos);
	}

	// ── Helpers ──

	private void pickFlyByStation(int entityId) {
		Node node = findEntityNode(entityId);
		if (node == null)
			return;
		Vector3f forward = node.getLocalRotation().mult(new Vector3f(0, -1, 0));
		flyByDirection.set(node.getLocalRotation().mult(Vector3f.UNIT_X)).multLocal(closeOrbitSide);
		boolean surfaceShip = snapshotsSupplier.get().stream().anyMatch(s -> s.id() == entityId && s.surfaceLocked());
		float scale = surfaceShip ? 2f : 1f;
		flyByStation.set(node.getLocalTranslation()).addLocal(forward.mult(60f * scale))
				.addLocal(flyByDirection.mult(72f * scale));
		flyByStation.y += 8f * scale;
	}

	private float passProgress() {
		return FastMath.clamp((shotTimer - closePassDelay) / Math.max(currentShot.duration - closePassDelay, 0.1f), 0f,
				1f);
	}

	/** Fit oriented hulls to the live lens, with room at the screen edges. */
	private void frameEntities(List<SubmarineSnapshot> subs, Vector3f position, Vector3f lookAt, int ... ids) {
		Vector3f offset = position.subtract(lookAt);
		float range = offset.length();
		if (range < 0.1f)
			return;
		offset.divideLocal(range);
		Vector3f forward = offset.negate();
		Vector3f up = Math.abs(offset.y) > 0.7f ? new Vector3f(0, 0, -1) : Vector3f.UNIT_Y;
		Vector3f right = forward.cross(up).normalizeLocal();
		up = right.cross(forward).normalizeLocal();
		float tanV = camera != null ? Math.abs(camera.getFrustumTop() / camera.getFrustumNear())
				: FastMath.tan(FastMath.DEG_TO_RAD * 22.5f);
		float tanH = camera != null ? Math.abs(camera.getFrustumRight() / camera.getFrustumNear()) : tanV * 16f / 9f;
		float padding = 0.88f;
		for (int id : ids) {
			Node node = findEntityNode(id);
			if (node == null)
				continue;
			boolean weapon = isTorpedoEntity(id);
			boolean ship = subs.stream().anyMatch(s -> s.id() == id && s.surfaceLocked());
			float halfBeam = weapon ? 0.5f : ship ? 17f : 7f;
			float halfLength = weapon ? 3f : ship ? 85f : 42f;
			float halfHeight = weapon ? 1f : ship ? 38f : 8f;
			// Node bounds include particle emitters; use the body envelope instead.
			for (int corner = 0; corner < 8; corner++) {
				Vector3f point = new Vector3f((corner & 1) == 0 ? -halfBeam : halfBeam,
						(corner & 2) == 0 ? -halfLength : halfLength, (corner & 4) == 0 ? -halfHeight : halfHeight);
				node.getLocalRotation().mult(point, point);
				point.addLocal(node.getLocalTranslation()).subtractLocal(lookAt);
				float screenRange = Math.max(Math.abs(right.dot(point)) / (tanH * padding),
						Math.abs(up.dot(point)) / (tanV * padding));
				range = Math.max(range, offset.dot(point) + screenRange);
			}
		}
		position.set(lookAt).addLocal(offset.mult(range));
	}

	private boolean isUnderwaterShot(ShotType type) {
		return switch (type) {
		case IDLE_ORBIT, IDLE_HERO, IDLE_FLY_BY, TORPEDO_TRACK, TORPEDO_LAUNCH, TORPEDO_TERMINAL, NEAR_MISS,
				PING_DETECTION, SUB_DEATH, DETONATION ->
			true;
		case OPENING_FLYAROUND, CONTEXT_WIDE, IDLE_ESTABLISHING -> false;
		};
	}

	private Node findEntityNode(int id) {
		Node node = subNodes.get(id);
		if (node == null)
			node = torpedoNodes.get(id);
		return node;
	}

	private int findNearestSub(List<SubmarineSnapshot> subs, double x, double y) {
		int best = subs.isEmpty() ? 0 : subs.getFirst().id();
		double bestDist = Double.MAX_VALUE;
		for (var s : subs) {
			var p = s.pose().position();
			double dx = p.x() - x, dy = p.y() - y;
			double d = dx * dx + dy * dy;
			if (d < bestDist) {
				bestDist = d;
				best = s.id();
			}
		}
		return best;
	}

	private int findNearestSubExcept(List<SubmarineSnapshot> subs, double x, double y, int exceptId) {
		int best = -1;
		double bestDist = Double.MAX_VALUE;
		for (var s : subs) {
			if (s.id() == exceptId)
				continue;
			var p = s.pose().position();
			double dx = p.x() - x, dy = p.y() - y;
			double d = dx * dx + dy * dy;
			if (d < bestDist) {
				bestDist = d;
				best = s.id();
			}
		}
		return best >= 0 ? best : (subs.isEmpty() ? 0 : subs.getFirst().id());
	}
}
