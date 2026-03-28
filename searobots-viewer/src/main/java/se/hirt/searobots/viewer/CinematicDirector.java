/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.viewer;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.util.*;
import java.util.function.Supplier;

/**
 * Automatic cinematic camera director. Creates dramatic, TV-broadcast-style
 * coverage of submarine battles with no player input.
 *
 * <p>The director detects events from snapshot data (torpedo launches,
 * detonations, HP changes, proximity), selects the most dramatic camera
 * shot, and computes camera positions. Between action events, it cycles
 * through beauty shots (orbits, fly-bys, establishing shots).
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
        IDLE_ORBIT("Orbit"),
        IDLE_FLY_BY("Fly-by"),
        IDLE_ESTABLISHING("Overview");

        final String label;
        ShotType(String label) { this.label = label; }
    }

    enum TransitionType {
        HARD_CUT(0f),
        DRAMATIC_ZOOM(0.5f),
        SMOOTH_PAN(1.0f),
        SLOW_DESCENT(2.0f),
        SLOW_PULLBACK(2.5f);

        final float duration;
        TransitionType(float duration) { this.duration = duration; }
    }

    record Shot(ShotType type, int subjectId, int secondaryId,
                float duration, float minDuration, TransitionType transition) {}

    // ── Scene access ──

    private final Supplier<List<SubmarineSnapshot>> snapshotsSupplier;
    private final Supplier<List<TorpedoSnapshot>> torpedoSnapshotsSupplier;
    private final Map<Integer, Node> subNodes;
    private final Map<Integer, Node> torpedoNodes;
    private final TerrainMap terrain; // nullable

    // ── State tracking for event detection ──

    private final Set<Integer> prevTorpedoIds = new HashSet<>();
    private final Map<Integer, Integer> prevSubHp = new HashMap<>();
    private final Set<Integer> seenDetonations = new HashSet<>();
    private final Set<Integer> seenTerminal = new HashSet<>();

    // ── Current shot ──

    private Shot currentShot;
    private float shotTimer;
    private float transitionTimer;

    // Spline transition state: cubic Hermite for position, lookAt tracks the subject
    private final Vector3f fromPos = new Vector3f();
    private final Vector3f fromLookAt = new Vector3f();
    private final Vector3f fromTangent = new Vector3f();

    // Last frame's final camera position (after all blending), used as the
    // starting point for the next transition.
    private final Vector3f lastCamPos = new Vector3f(0, 50, 0);
    private final Vector3f lastCamLookAt = new Vector3f(0, 0, 0);

    // ── Idle sequence state ──
    // Cycles through ALL entities (subs + alive torpedoes):
    //   1. Close underwater on entity N (orbit/fly-by)
    //   2. Close top-down on entity N (fade water)
    //   3. Full wide overview of everything
    //   4. Close top-down on entity N+1 (preview)
    //   5. Descend to close underwater on entity N+1
    //   ... round-robin
    private int lastIdleEntityIndex = -1;
    private final List<Integer> idleEntityIds = new ArrayList<>();
    private enum IdlePhase { CLOSE_UNDERWATER, CLOSE_TOPDOWN, WIDE_OVERVIEW, PREVIEW_TOPDOWN }
    private IdlePhase nextIdlePhase = IdlePhase.CLOSE_UNDERWATER;

    // ── Context shot: queued follow-up after wide view ──

    private Shot afterContextShot;

    // ── Shot-specific state ──

    private float idleOrbitAngle;
    private final Vector3f flyByStation = new Vector3f();
    private float flyByAge;
    private final Vector3f detonationPos = new Vector3f();
    private float detonationPullback;

    // ── Pending shots (priority queue) ──

    private final PriorityQueue<Shot> pendingShots = new PriorityQueue<>(
            Comparator.comparingInt(s -> s.type.ordinal()));

    // ── Water opacity (establishing shots fade water out to see subs) ──
    // 1.0 = fully visible (normal), 0.0 = fully transparent/disabled
    private float waterOpacity = 1f;
    private float waterOpacityTarget = 1f;
    private static final float WATER_FADE_OUT_SPEED = 0.6f;  // ~1.7s to go transparent
    private static final float WATER_FADE_IN_SPEED = 0.3f;   // ~3.3s to go opaque (slower = smoother re-entry)

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
    private int terminalRecedingTicks = 0; // ticks where distance is increasing

    // ── Random for idle variety ──

    private final Random rng = new Random();

    CinematicDirector(Supplier<List<SubmarineSnapshot>> snapshots,
                      Supplier<List<TorpedoSnapshot>> torpedoSnapshots,
                      Map<Integer, Node> subNodes,
                      Map<Integer, Node> torpedoNodes,
                      TerrainMap terrain) {
        this.snapshotsSupplier = snapshots;
        this.torpedoSnapshotsSupplier = torpedoSnapshots;
        this.subNodes = subNodes;
        this.torpedoNodes = torpedoNodes;
        this.terrain = terrain;
    }

    /**
     * Called each frame. Detects events, evaluates shot changes, computes
     * camera position. Returns the entity ID the camera is tracking (for HUD).
     */
    int update(float tpf, Vector3f outPos, Vector3f outLookAt) {
        var snapshots = snapshotsSupplier.get();
        var torpSnapshots = torpedoSnapshotsSupplier.get();

        detectEvents(snapshots, torpSnapshots);
        evaluateShot(tpf);

        // Refresh entity list (subs + alive torpedoes) each frame
        refreshEntityList(snapshots, torpSnapshots);

        if (currentShot == null) {
            // Start with a cinematic fly-around of the battle area:
            // camera circles the arena at the water surface, looking at the center.
            // Then smoothly descends to the first entity.
            int firstId = idleEntityIds.isEmpty() ? 0 : idleEntityIds.getFirst();
            lastIdleEntityIndex = 0;
            nextIdlePhase = IdlePhase.CLOSE_TOPDOWN;
            openingAngle = 0;
            afterContextShot = makeCloseShot(firstId);
            // Duration: one full lap. Arena radius ~7000m, at a nice angular speed
            // ~12 seconds for a full 360-degree sweep.
            currentShot = new Shot(ShotType.OPENING_FLYAROUND, firstId, -1,
                    12f, 5f, TransitionType.HARD_CUT);
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
            // Fading in (becoming opaque): only start after the camera transition
            // is mostly done, then fade slowly for a smooth splash-back effect.
            boolean transitionDone = transitionTimer <= 0.3f;
            if (transitionDone) {
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
                outPos.set(
                        fromPos.x + (destX - fromPos.x) * t,
                        fromPos.y + (destY - fromPos.y) * t,
                        fromPos.z + (destZ - fromPos.z) * t);
                // Blend lookAt too (don't force a fixed Y, let it transition smoothly)
                outLookAt.set(
                        fromLookAt.x + (destLX - fromLookAt.x) * t,
                        fromLookAt.y + (destLY - fromLookAt.y) * t,
                        fromLookAt.z + (destLZ - fromLookAt.z) * t);

            } else {
                // Spline transition: cubic Hermite for underwater/mixed transitions
                float t2 = t * t, t3 = t2 * t;
                float h00 = 2*t3 - 3*t2 + 1;
                float h10 = t3 - 2*t2 + t;
                float h01 = -2*t3 + 3*t2;
                float h11 = t3 - t2;

                float transDist = fromPos.distance(new Vector3f(destX, destY, destZ));
                float tangentScale = Math.max(transDist * 0.4f, 30f);
                Vector3f t0 = new Vector3f(fromTangent).multLocal(tangentScale);

                Vector3f arriveDir = new Vector3f(destLX - destX, destLY - destY, destLZ - destZ);
                if (arriveDir.lengthSquared() > 0.01f) arriveDir.normalizeLocal();
                else arriveDir.set(0, 0, -1);
                Vector3f t1 = arriveDir.mult(tangentScale);

                outPos.set(
                        h00*fromPos.x + h10*t0.x + h01*destX + h11*t1.x,
                        h00*fromPos.y + h10*t0.y + h01*destY + h11*t1.y,
                        h00*fromPos.z + h10*t0.z + h01*destZ + h11*t1.z);

                outLookAt.set(
                        fromLookAt.x + (destLX - fromLookAt.x) * t,
                        fromLookAt.y + (destLY - fromLookAt.y) * t,
                        fromLookAt.z + (destLZ - fromLookAt.z) * t);
            }

            // Progressively update the start point toward the blended position.
            // This absorbs entity movement so the transition doesn't jitter
            // when the destination keeps shifting under a moving subject.
            float absorb = Math.min(tpf * 2f, 0.1f);
            fromPos.interpolateLocal(outPos, absorb);
            fromLookAt.interpolateLocal(outLookAt, absorb);

            transitionTimer = Math.max(0, transitionTimer - tpf);
        }

        // Terrain avoidance: don't go below terrain
        if (terrain != null) {
            float simX = outPos.x;
            float simY = -outPos.z;
            double floor = terrain.elevationAt(simX, simY);
            float minY = (float) floor + 10f; // JME Y = sim Z (up)
            if (outPos.y < minY) outPos.y = minY;
        }

        // Underwater enforcement: shots that should be underwater stay below
        // the surface (JME Y=0 is the water plane). This prevents the camera
        // from popping above the water during transitions between underwater shots.
        if (isUnderwaterShot(currentShot.type) && transitionTimer <= 0) {
            outPos.y = Math.min(outPos.y, -3f); // at least 3m below surface
        } else if (isUnderwaterShot(currentShot.type) && transitionTimer > 0) {
            // During transitions TO an underwater shot, clamp more gently
            // (the spline may arc through the surface)
            outPos.y = Math.min(outPos.y, 5f); // allow brief surface skim during transition
        }

        // Log every frame for overhead shots to find the glitch
        // Save the final camera position for next frame's transition start
        lastCamPos.set(outPos);
        lastCamLookAt.set(outLookAt);

        return currentShot.subjectId;
    }

    String currentShotLabel() {
        return currentShot != null ? currentShot.type.label : "---";
    }

    /**
     * Water opacity: 1.0 = normal rendering, 0.0 = fully transparent.
     * Establishing shots smoothly fade to 0 so the subs become visible
     * from above. Other shots fade back to 1.0.
     */
    float waterOpacity() { return waterOpacity; }

    // ── Entity list for idle cycling ──

    private void refreshEntityList(List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
        idleEntityIds.clear();
        for (var s : subs) idleEntityIds.add(s.id());
        for (var t : torps) {
            if (t.alive()) idleEntityIds.add(t.id());
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
            if (!prevTorpedoIds.contains(ts.id())) {
                // New torpedo: show launcher briefly, then track torpedo
                pendingShots.add(new Shot(ShotType.TORPEDO_LAUNCH, ts.ownerId(), ts.id(),
                        2f, 1.5f, TransitionType.HARD_CUT));
                pendingShots.add(new Shot(ShotType.TORPEDO_TRACK, ts.id(), -1,
                        8f, 2f, TransitionType.SMOOTH_PAN));
            }
        }
        prevTorpedoIds.clear();
        prevTorpedoIds.addAll(currentTorpIds);

        // Torpedo terminal phase + detonations
        for (var ts : torps) {
            if (ts.detonated() && seenDetonations.add(ts.id())) {
                var torpNode = torpedoNodes.get(ts.id());
                if (torpNode != null) {
                    detonationPos.set(torpNode.getLocalTranslation());
                } else {
                    detonationPos.set((float) ts.pose().position().x(),
                            (float) ts.pose().position().z(),
                            (float) -ts.pose().position().y());
                }
                detonationPullback = 0;

                // Check if this detonation is near a sub (a hit vs terrain impact)
                int nearestSub = findNearestSub(subs, ts.pose().position().x(), ts.pose().position().y());
                double nearestDist = Double.MAX_VALUE;
                for (var s : subs) {
                    double d = ts.pose().position().distanceTo(s.pose().position());
                    if (d < nearestDist) nearestDist = d;
                }
                boolean isSubHit = nearestDist < 80; // within blast effect range

                // Sub hit: preempt any ongoing detonation camera (force min duration 0)
                // Terrain hit: normal priority, can be preempted by sub hits
                float minDur = isSubHit ? 0f : 3.0f;
                if (isSubHit && currentShot != null && currentShot.type == ShotType.DETONATION) {
                    // Force the current terrain detonation to expire immediately
                    shotTimer = currentShot.duration;
                }

                pendingShots.add(new Shot(ShotType.DETONATION, ts.id(), nearestSub,
                        4.0f, minDur, TransitionType.HARD_CUT));
            }

            if (ts.alive() && !seenTerminal.contains(ts.id())) {
                // Check distance to nearest enemy sub (not the torpedo's intercept estimate)
                var torpPos = ts.pose().position();
                for (var s : subs) {
                    if (s.id() == ts.ownerId()) continue; // skip the launcher
                    var sp = s.pose().position();
                    double dx = torpPos.x() - sp.x(), dy = torpPos.y() - sp.y(), dz = torpPos.z() - sp.z();
                    double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    if (dist < 600) {
                        seenTerminal.add(ts.id());
                        pendingShots.add(new Shot(ShotType.TORPEDO_TERMINAL, ts.id(), s.id(),
                                30f, 2f, TransitionType.DRAMATIC_ZOOM));
                        break;
                    }
                }
            }
        }

        // HP changes (hits and deaths)
        for (var s : subs) {
            int prev = prevSubHp.getOrDefault(s.id(), 1000);
            if (s.hp() < prev && s.hp() > 0) {
                // Hit: show the hit sub
                pendingShots.add(new Shot(ShotType.NEAR_MISS, s.id(), -1,
                        3f, 1f, TransitionType.HARD_CUT));
            }
            if (s.hp() <= 0 && prev > 0) {
                // Death: dramatic slow orbit
                pendingShots.add(new Shot(ShotType.SUB_DEATH, s.id(), -1,
                        5f, 2f, TransitionType.SLOW_PULLBACK));
            }
            prevSubHp.put(s.id(), s.hp());
        }
    }

    // ── Shot Evaluation ──

    private void evaluateShot(float tpf) {
        // Force switch if current shot expired
        boolean expired = currentShot != null && shotTimer > currentShot.duration;
        // Can we be preempted?
        boolean canPreempt = currentShot == null
                || shotTimer > currentShot.minDuration
                || expired;

        if (!pendingShots.isEmpty()) {
            var next = pendingShots.peek();
            // Detonations always preempt (priority 0)
            boolean forcePreempt = next.type == ShotType.DETONATION;
            if (canPreempt || forcePreempt) {
                if (currentShot == null
                        || next.type.ordinal() < currentShot.type.ordinal()
                        || expired) {
                    switchToShot(pendingShots.poll());
                    return;
                }
            }
        }

        // Terminal approach: check if the torpedo resolved (missed or gone)
        if (currentShot != null && currentShot.type == ShotType.TORPEDO_TERMINAL && !expired) {
            var torps = torpedoSnapshotsSupplier.get();
            var subs = snapshotsSupplier.get();
            boolean torpGone = torps.stream().noneMatch(t -> t.id() == currentShot.subjectId && t.alive());
            boolean targetGone = subs.stream().noneMatch(s -> s.id() == currentShot.secondaryId && s.hp() > 0);

            if (torpGone || targetGone) {
                // Torpedo detonated, died, or target destroyed: hold 1.5s then move on
                if (shotTimer > currentShot.minDuration + 1.5f) expired = true;
            } else {
                // Track closing distance to detect a miss (torpedo receding)
                Node torpNode = torpedoNodes.get(currentShot.subjectId);
                Node targetNode = subNodes.get(currentShot.secondaryId);
                if (torpNode != null && targetNode != null) {
                    float dist = torpNode.getLocalTranslation().distance(targetNode.getLocalTranslation());
                    terminalClosestDist = Math.min(terminalClosestDist, dist);
                    if (dist > terminalPrevDist + 1f) {
                        terminalRecedingTicks++;
                    } else {
                        terminalRecedingTicks = 0;
                    }
                    terminalPrevDist = dist;
                    // Clear miss: receding for 3+ seconds after closest approach
                    if (terminalRecedingTicks > 150 && dist > terminalClosestDist + 100) {
                        expired = true;
                    }
                }
            }
        }

        if (expired) {
            // If an overhead shot just finished with a queued follow-up,
            // transition with a slow descent back into the water
            if (afterContextShot != null
                    && (currentShot.type == ShotType.CONTEXT_WIDE
                        || currentShot.type == ShotType.IDLE_ESTABLISHING
                        || currentShot.type == ShotType.OPENING_FLYAROUND)) {
                var next = afterContextShot;
                afterContextShot = null;
                switchToShot(new Shot(next.type, next.subjectId, next.secondaryId,
                        next.duration, next.minDuration, TransitionType.SLOW_DESCENT));
            } else {
                switchToIdleShot();
            }
        }
    }

    private void switchToShot(Shot shot) {
        // If leaving an overhead shot, force a slow descent transition
        // so the camera glides back into the water smoothly.
        boolean leavingOverhead = currentShot != null
                && (currentShot.type == ShotType.IDLE_ESTABLISHING
                    || currentShot.type == ShotType.CONTEXT_WIDE
                    || currentShot.type == ShotType.OPENING_FLYAROUND);
        if (leavingOverhead && shot.transition != TransitionType.HARD_CUT
                && shot.transition != TransitionType.SLOW_DESCENT) {
            shot = new Shot(shot.type, shot.subjectId, shot.secondaryId,
                    shot.duration, shot.minDuration, TransitionType.SLOW_DESCENT);
        }

        // Capture the actual camera position from last frame as the transition start
        fromPos.set(lastCamPos);
        fromLookAt.set(lastCamLookAt);

        // Track whether overhead shots are entered from above or below
        if (shot.type == ShotType.CONTEXT_WIDE || shot.type == ShotType.IDLE_ESTABLISHING) {
            overviewEnteredFromAbove = fromPos.y > 100f;
        }


        currentShot = shot;
        shotTimer = 0;
        transitionTimer = shot.transition.duration;

        // Compute tangent at transition start. For overhead shots (looking
        // straight down), use a horizontal tangent so the spline glides
        // laterally instead of arcing down and back up.
        boolean fromOverhead = fromPos.y > 10f; // above water = overhead
        if (fromOverhead) {
            Vector3f horiz = new Vector3f(fromLookAt.x - fromPos.x, 0,
                    fromLookAt.z - fromPos.z);
            if (horiz.lengthSquared() > 0.01f) horiz.normalizeLocal();
            else horiz.set(0, 0, -1);
            fromTangent.set(horiz);
        } else {
            Vector3f lookDir = new Vector3f(fromLookAt).subtractLocal(fromPos);
            float lookDist = lookDir.length();
            if (lookDist > 0.1f) lookDir.divideLocal(lookDist);
            else lookDir.set(0, 0, -1);
            fromTangent.set(lookDir);
        }

        // Reset shot-specific state
        if (shot.type == ShotType.IDLE_FLY_BY) {
            pickFlyByStation(shot.subjectId);
            flyByAge = 0;
        }
        if (shot.type == ShotType.TORPEDO_TERMINAL) {
            terminalClosestDist = Float.MAX_VALUE;
            terminalPrevDist = Float.MAX_VALUE;
            terminalRecedingTicks = 0;
        }
    }

    private void switchToIdleShot() {
        if (idleEntityIds.isEmpty()) return;

        // Structured idle sequence:
        //   1. Close underwater on sub A (orbit/fly-by), 8-12s
        //   2. Close top-down on sub A (water fades, see A's surroundings), 5-8s
        //   3. Wide tactical overview (all entities), 4s
        //   4. Close top-down on sub B (preview B before diving in), 5-8s
        //   5. Descend to close underwater on sub B
        //   ... repeat, alternating subs

        if (idleEntityIds.isEmpty()) return;

        switch (nextIdlePhase) {
            case CLOSE_UNDERWATER -> {
                // Phase 1: close underwater view of next entity
                lastIdleEntityIndex = (lastIdleEntityIndex + 1) % idleEntityIds.size();
                nextIdlePhase = IdlePhase.CLOSE_TOPDOWN;
                switchToShot(makeCloseShot(idleEntityIds.get(lastIdleEntityIndex)));
            }
            case CLOSE_TOPDOWN -> {
                // Phase 2: pull up to close top-down of the SAME entity
                nextIdlePhase = IdlePhase.WIDE_OVERVIEW;
                int entityId = idleEntityIds.get(lastIdleEntityIndex % idleEntityIds.size());
                float duration = isTorpedoEntity(entityId) ? 3f + rng.nextFloat() * 2f
                        : 5f + rng.nextFloat() * 3f;
                switchToShot(new Shot(ShotType.IDLE_ESTABLISHING, entityId, -1,
                        duration, 2f, TransitionType.SMOOTH_PAN));
            }
            case WIDE_OVERVIEW -> {
                // Phase 3: wide overview showing everything.
                // Slow transition from birds-eye (both are overhead, smooth pull-up).
                if (idleEntityIds.size() >= 2) {
                    nextIdlePhase = IdlePhase.PREVIEW_TOPDOWN;
                    switchToShot(new Shot(ShotType.CONTEXT_WIDE, 0, -1,
                            10f, 4f, TransitionType.SLOW_PULLBACK));
                } else {
                    nextIdlePhase = IdlePhase.CLOSE_UNDERWATER;
                    switchToShot(makeCloseShot(idleEntityIds.getFirst()));
                }
            }
            case PREVIEW_TOPDOWN -> {
                // Phase 4: close top-down of the NEXT entity (preview before diving in).
                // Slow transition from wide overview (both overhead, smooth glide).
                nextIdlePhase = IdlePhase.CLOSE_UNDERWATER;
                int nextIdx = (lastIdleEntityIndex + 1) % idleEntityIds.size();
                int nextId = idleEntityIds.get(nextIdx);
                float duration = isTorpedoEntity(nextId) ? 3f + rng.nextFloat() * 2f
                        : 5f + rng.nextFloat() * 3f;
                afterContextShot = makeCloseShot(nextId);
                switchToShot(new Shot(ShotType.IDLE_ESTABLISHING, nextId, -1,
                        duration, 2f, TransitionType.SLOW_PULLBACK));
            }
        }
    }

    private Shot makeCloseShot(int entityId) {
        boolean isTorp = isTorpedoEntity(entityId);
        // Torpedoes get shorter shots (they move fast, less time needed)
        float duration = isTorp ? 5f + rng.nextFloat() * 3f : 8f + rng.nextFloat() * 4f;
        ShotType[] closeTypes = {ShotType.IDLE_ORBIT, ShotType.IDLE_FLY_BY};
        ShotType type = closeTypes[rng.nextInt(closeTypes.length)];
        return new Shot(type, entityId, -1, duration, 2f, TransitionType.SMOOTH_PAN);
    }

    // ── Camera Computation ──

    private void computeShot(float tpf, List<SubmarineSnapshot> subs,
                             List<TorpedoSnapshot> torps,
                             Vector3f outPos, Vector3f outLookAt) {

        switch (currentShot.type) {
            case DETONATION -> computeDetonation(tpf, outPos, outLookAt);
            case SUB_DEATH -> computeSubDeath(tpf, subs, outPos, outLookAt);
            case TORPEDO_TERMINAL -> computeTorpedoTerminal(tpf, subs, torps, outPos, outLookAt);
            case NEAR_MISS -> computeChaseShot(subs, outPos, outLookAt, 150f, 30f);
            case TORPEDO_LAUNCH -> computeChaseShot(subs, outPos, outLookAt, 200f, 40f);
            case TORPEDO_TRACK -> computeTorpedoTrack(tpf, torps, outPos, outLookAt);
            case PING_DETECTION -> computeChaseShot(subs, outPos, outLookAt, 200f, 40f);
            case OPENING_FLYAROUND -> computeOpeningFlyaround(tpf, outPos, outLookAt);
            case CONTEXT_WIDE -> computeContextWide(tpf, subs, torps, outPos, outLookAt);
            case IDLE_ORBIT -> computeIdleOrbit(tpf, subs, outPos, outLookAt);
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

        outPos.set(
                radius * FastMath.sin(openingAngle),
                height,
                radius * FastMath.cos(openingAngle));

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
        outPos.set(
                detonationPos.x + dist * FastMath.sin(idleOrbitAngle),
                detonationPos.y + elev + 50f,
                detonationPos.z + dist * FastMath.cos(idleOrbitAngle));
        outLookAt.set(detonationPos);
    }

    private void computeSubDeath(float tpf, List<SubmarineSnapshot> subs,
                                 Vector3f outPos, Vector3f outLookAt) {
        // Slow orbit around the dying sub
        Node node = subNodes.get(currentShot.subjectId);
        if (node == null) { computeIdleEstablishing(subs, outPos, outLookAt); return; }

        Vector3f subPos = node.getLocalTranslation();
        float orbitRate = 0.3f; // rad/s
        idleOrbitAngle += orbitRate * tpf;
        float dist = 150f;
        float elev = FastMath.DEG_TO_RAD * 20f;

        outPos.set(
                subPos.x + dist * FastMath.cos(elev) * FastMath.sin(idleOrbitAngle),
                subPos.y + dist * FastMath.sin(elev),
                subPos.z + dist * FastMath.cos(elev) * FastMath.cos(idleOrbitAngle));
        outLookAt.set(subPos);
    }

    private void computeTorpedoTerminal(float tpf, List<SubmarineSnapshot> subs,
                                        List<TorpedoSnapshot> torps,
                                        Vector3f outPos, Vector3f outLookAt) {
        // Elliptical orbit around the torpedo-target pair, keeping both in view.
        // The orbit is tighter near the torpedo, wider near the target.
        Node torpNode = torpedoNodes.get(currentShot.subjectId);
        Node targetNode = currentShot.secondaryId >= 0 ? subNodes.get(currentShot.secondaryId) : null;

        if (torpNode == null) { computeIdleOrbit(tpf, subs, outPos, outLookAt); return; }

        Vector3f torpPos = torpNode.getLocalTranslation();
        Vector3f targetPos = targetNode != null ? targetNode.getLocalTranslation() : torpPos;

        // Midpoint weighted toward the torpedo (camera stays closer to the action)
        float torpWeight = 0.65f;
        Vector3f focus = new Vector3f(
                torpPos.x * torpWeight + targetPos.x * (1 - torpWeight),
                torpPos.y * torpWeight + targetPos.y * (1 - torpWeight),
                torpPos.z * torpWeight + targetPos.z * (1 - torpWeight));

        // Separation between torpedo and target
        float separation = torpPos.distance(targetPos);
        float orbitRadius = Math.max(separation * 0.5f + 30f, 80f);

        // Orbit around the focus point. Use the torpedo-target axis to define
        // the ellipse orientation (wider perpendicular to the chase axis).
        float axisX = targetPos.x - torpPos.x;
        float axisZ = targetPos.z - torpPos.z;
        float axisLen = FastMath.sqrt(axisX * axisX + axisZ * axisZ);
        if (axisLen > 0.01f) { axisX /= axisLen; axisZ /= axisLen; }
        else { axisX = 1; axisZ = 0; }
        float perpX = -axisZ, perpZ = axisX; // perpendicular to chase axis

        // Slow orbit: ~0.4 rad/s
        idleOrbitAngle += 0.4f * tpf;
        float cosA = FastMath.cos(idleOrbitAngle);
        float sinA = FastMath.sin(idleOrbitAngle);

        // Elliptical: wider perpendicular (1.0x), narrower along chase (0.5x)
        float offAlongAxis = orbitRadius * 0.5f * cosA;
        float offPerpAxis = orbitRadius * sinA;

        outPos.set(
                focus.x + axisX * offAlongAxis + perpX * offPerpAxis,
                focus.y + 20f + orbitRadius * 0.15f, // slightly above
                focus.z + axisZ * offAlongAxis + perpZ * offPerpAxis);
        outLookAt.set(focus);
    }

    private void computeChaseShot(List<SubmarineSnapshot> subs,
                                  Vector3f outPos, Vector3f outLookAt,
                                  float asternDist, float aboveHeight) {
        Node node = subNodes.get(currentShot.subjectId);
        if (node == null) {
            // Might be a torpedo ID that got used for launch shot
            node = torpedoNodes.get(currentShot.subjectId);
        }
        if (node == null) { outPos.set(0, 100, 0); outLookAt.set(0, 0, 0); return; }

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

        outPos.set(
                pos.x - fwdX * asternDist,
                pos.y + aboveHeight,
                pos.z - fwdZ * asternDist);
        outLookAt.set(pos);
    }

    private void computeTorpedoTrack(float tpf, List<TorpedoSnapshot> torps,
                                     Vector3f outPos, Vector3f outLookAt) {
        Node node = torpedoNodes.get(currentShot.subjectId);
        if (node == null) { outPos.set(0, 50, 0); outLookAt.set(0, 0, 0); return; }

        Vector3f torpPos = node.getLocalTranslation();

        // Find torpedo heading
        float heading = 0;
        for (var ts : torps) {
            if (ts.id() == currentShot.subjectId) {
                heading = (float) ts.pose().heading();
                break;
            }
        }

        float fwdX = FastMath.sin(heading);
        float fwdZ = -FastMath.cos(heading);
        float perpX = fwdZ, perpZ = -fwdX;

        // Close follow: slightly to the side and behind, with a slow drift
        // to give a dynamic feel (the camera slowly orbits from one side to the other)
        float drift = FastMath.sin(shotTimer * 0.3f) * 20f; // gentle side-to-side
        outPos.set(
                torpPos.x - fwdX * 40f + perpX * drift,
                torpPos.y + 8f,
                torpPos.z - fwdZ * 40f + perpZ * drift);

        // Look slightly ahead of the torpedo (lead the subject)
        outLookAt.set(
                torpPos.x + fwdX * 30f,
                torpPos.y,
                torpPos.z + fwdZ * 30f);
    }

    private void computeContextWide(float tpf, List<SubmarineSnapshot> subs,
                                     List<TorpedoSnapshot> torps,
                                     Vector3f outPos, Vector3f outLookAt) {
        // High-altitude shot centered on the midpoint between all active subs.
        // Camera height scales with the distance between subs so both are visible.
        if (subs.isEmpty()) { outPos.set(0, 300, 0); outLookAt.set(0, 0, 0); return; }

        // Compute centroid and max spread of all subs
        float sumX = 0, sumZ = 0;
        float minX = Float.MAX_VALUE, maxX = -Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE, maxZ = -Float.MAX_VALUE;
        int count = 0;
        for (var s : subs) {
            Node node = subNodes.get(s.id());
            if (node == null) continue;
            Vector3f p = node.getLocalTranslation();
            sumX += p.x; sumZ += p.z;
            minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
            minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z);
            count++;
        }
        // Include active torpedoes in the framing
        for (var ts : torps) {
            if (!ts.alive()) continue;
            Node node = torpedoNodes.get(ts.id());
            if (node == null) continue;
            Vector3f p = node.getLocalTranslation();
            sumX += p.x; sumZ += p.z;
            minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
            minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z);
            count++;
        }
        if (count == 0) { outPos.set(0, 300, 0); outLookAt.set(0, 0, 0); return; }

        float rawMidX = sumX / count;
        float rawMidZ = sumZ / count;

        // Smooth the centroid to prevent jumps when entities appear/disappear
        if (!centroidInitialized) {
            smoothedCentroid.set(rawMidX, 0, rawMidZ);
            centroidInitialized = true;
        } else {
            smoothedCentroid.x += (rawMidX - smoothedCentroid.x) * Math.min(tpf * 1.5f, 0.15f);
            smoothedCentroid.z += (rawMidZ - smoothedCentroid.z) * Math.min(tpf * 1.5f, 0.15f);
        }
        float midX = smoothedCentroid.x;
        float midZ = smoothedCentroid.z;
        float spread = Math.max(maxX - minX, maxZ - minZ);

        // Height: enough to see the spread, with dynamic altitude animation.
        // Coming from below (underwater): start low and rise (zoom out).
        // Coming from above (birds-eye): start high and descend (zoom in).
        float baseHeight = Math.max(400f, spread * 1.2f + 200f);
        float altitudeRange = 500f; // total altitude change over the shot
        float progress = Math.min(shotTimer / currentShot.duration, 1f);
        float height;
        if (overviewEnteredFromAbove) {
            // Entered from above: start high, slowly zoom in
            height = baseHeight + altitudeRange * (1f - progress);
        } else {
            // Entered from below: start at base, slowly rise
            height = baseHeight + altitudeRange * progress;
        }

        // Fixed orientation: camera offset toward south (JME +Z).
        float southOffset = height * 0.1f;
        outPos.set(midX, height, midZ + southOffset);
        outLookAt.set(midX, -30f, midZ - 1f);
    }

    private void computeIdleOrbit(float tpf, List<SubmarineSnapshot> subs,
                                  Vector3f outPos, Vector3f outLookAt) {
        Node node = findEntityNode(currentShot.subjectId);
        if (node == null) { outPos.set(0, 200, 0); outLookAt.set(0, 0, 0); return; }

        Vector3f subPos = node.getLocalTranslation();
        boolean isTorp = isTorpedoEntity(currentShot.subjectId);
        idleOrbitAngle += (isTorp ? 0.25f : 0.15f) * tpf; // faster orbit for torpedoes
        float dist = isTorp ? 100f : 250f; // tighter for torpedoes
        float elev = FastMath.DEG_TO_RAD * 18f;

        outPos.set(
                subPos.x + dist * FastMath.cos(elev) * FastMath.sin(idleOrbitAngle),
                subPos.y + dist * FastMath.sin(elev),
                subPos.z + dist * FastMath.cos(elev) * FastMath.cos(idleOrbitAngle));
        outLookAt.set(subPos);
    }

    private void computeIdleFlyBy(float tpf, List<SubmarineSnapshot> subs,
                                  Vector3f outPos, Vector3f outLookAt) {
        Node node = findEntityNode(currentShot.subjectId);
        if (node == null) { computeIdleOrbit(tpf, subs, outPos, outLookAt); return; }

        Vector3f subPos = node.getLocalTranslation();
        flyByAge += tpf;

        // Reposition when sub passes the station
        if (subPos.distance(flyByStation) > 500f || flyByAge > 12f) {
            pickFlyByStation(currentShot.subjectId);
            flyByAge = 0;
        }

        outPos.set(flyByStation);
        outLookAt.set(subPos);
    }

    private void computeIdleEstablishing(List<SubmarineSnapshot> subs,
                                         Vector3f outPos, Vector3f outLookAt) {
        Node node = findEntityNode(currentShot.subjectId);
        if (node == null) { outPos.set(0, 200, 500); outLookAt.set(0, 0, 0); return; }

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
        if (node == null) return;

        Vector3f subPos = node.getLocalTranslation();
        // Pick a point 300m ahead and 150m to the side
        float angle = rng.nextFloat() * FastMath.TWO_PI;
        flyByStation.set(
                subPos.x + 300f * FastMath.sin(angle),
                subPos.y + 20f + rng.nextFloat() * 30f,
                subPos.z + 300f * FastMath.cos(angle));
    }

    private boolean isUnderwaterShot(ShotType type) {
        return switch (type) {
            case IDLE_ORBIT, IDLE_FLY_BY, TORPEDO_TRACK, TORPEDO_LAUNCH,
                 TORPEDO_TERMINAL, NEAR_MISS, PING_DETECTION, SUB_DEATH,
                 DETONATION -> true;
            case OPENING_FLYAROUND, CONTEXT_WIDE, IDLE_ESTABLISHING -> false;
        };
    }

    private Node findEntityNode(int id) {
        Node node = subNodes.get(id);
        if (node == null) node = torpedoNodes.get(id);
        return node;
    }

    private int findNearestSub(List<SubmarineSnapshot> subs, double x, double y) {
        int best = subs.isEmpty() ? 0 : subs.getFirst().id();
        double bestDist = Double.MAX_VALUE;
        for (var s : subs) {
            var p = s.pose().position();
            double dx = p.x() - x, dy = p.y() - y;
            double d = dx * dx + dy * dy;
            if (d < bestDist) { bestDist = d; best = s.id(); }
        }
        return best;
    }

    private int findNearestSubExcept(List<SubmarineSnapshot> subs, double x, double y, int exceptId) {
        int best = -1;
        double bestDist = Double.MAX_VALUE;
        for (var s : subs) {
            if (s.id() == exceptId) continue;
            var p = s.pose().position();
            double dx = p.x() - x, dy = p.y() - y;
            double d = dx * dx + dy * dy;
            if (d < bestDist) { bestDist = d; best = s.id(); }
        }
        return best >= 0 ? best : (subs.isEmpty() ? 0 : subs.getFirst().id());
    }
}
