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
import se.hirt.searobots.api.TerrainMap;

import java.util.ArrayList;
import java.util.List;

/**
 * Encapsulates the island orbit camera state: finding island peaks from
 * terrain data, transitioning between islands with a two-phase animation
 * (rotate toward target, then fly over), and managing the orbit center.
 *
 * <p>This class does not reference CameraMode. The caller (SubmarineScene3D)
 * checks the camera mode and delegates to this class when ISLAND_ORBIT
 * is active.
 */
final class IslandOrbitState {

    // Transition timing constants
    private static final float ISLAND_ROTATE_DURATION = 1.0f;
    private static final float ISLAND_FLY_DURATION = 2.5f;

    // Island peaks in jME coordinates (x=east, y=elevation, z=-north)
    private final List<Vector3f> islandPeaks = new ArrayList<>();
    private int currentIslandIdx = 0;

    // Transition state: phase 1 = rotate toward target, phase 2 = fly over
    private float transTimer = 0;
    private final Vector3f transFrom = new Vector3f();
    private final Vector3f transTo = new Vector3f();
    private float transTotalDuration = 0;
    private boolean transitionJustEnded = false;

    // The orbit center managed by this state during island orbit mode
    private final Vector3f orbitCenter = new Vector3f();

    /**
     * Scan the terrain for distinct island peaks (local maxima above sea level,
     * separated by at least 500m).
     */
    void initFromTerrain(TerrainMap terrain) {
        islandPeaks.clear();
        currentIslandIdx = 0;
        double minSep = 500;

        // Scan grid at coarse resolution, find local maxima
        int step = 20;
        var candidates = new ArrayList<double[]>(); // {x, y, elev}
        for (int r = step; r < terrain.getRows() - step; r += step) {
            for (int c = step; c < terrain.getCols() - step; c += step) {
                double e = terrain.elevationAtGrid(c, r);
                if (e < 10) continue; // must be well above water
                // Check if local maximum in neighborhood
                boolean isMax = true;
                for (int dr = -step; dr <= step && isMax; dr += step)
                    for (int dc = -step; dc <= step && isMax; dc += step)
                        if (dr != 0 || dc != 0)
                            if (terrain.elevationAtGrid(c + dc, r + dr) > e) isMax = false;
                if (isMax) {
                    double wx = terrain.getOriginX() + c * terrain.getCellSize();
                    double wy = terrain.getOriginY() + r * terrain.getCellSize();
                    candidates.add(new double[]{wx, wy, e});
                }
            }
        }

        // Sort by elevation (tallest first) and filter by minimum separation
        candidates.sort((a, b) -> Double.compare(b[2], a[2]));
        for (var cand : candidates) {
            boolean tooClose = false;
            for (var existing : islandPeaks) {
                double dx = cand[0] - existing.x;
                double dz = cand[1] + existing.z; // existing.z = -wy
                if (Math.sqrt(dx * dx + dz * dz) < minSep) { tooClose = true; break; }
            }
            if (!tooClose) {
                // Store as jME coords: x=east, y=elevation, z=-north
                islandPeaks.add(new Vector3f((float) cand[0], (float) cand[2], (float) -cand[1]));
            }
        }
    }

    /**
     * Activate island orbit mode. Sets the orbit center to the first island
     * peak and returns the recommended orbit distance and elevation.
     *
     * @param outOrbitCenter populated with the initial orbit center
     * @return float[2]: {orbitDistance, orbitElevation}, or null if no islands
     */
    float[] activate(Vector3f outOrbitCenter) {
        if (islandPeaks.isEmpty()) return null;
        currentIslandIdx = 0;
        var peak = islandPeaks.getFirst();
        orbitCenter.set(peak.x, peak.y * 0.4f, peak.z);
        outOrbitCenter.set(orbitCenter);
        float distance = Math.max(200, peak.y * 4);
        float elevation = 0.35f;
        System.out.printf("Island Orbit: island 1/%d (elev=%.0f)%n",
                islandPeaks.size(), peak.y);
        return new float[]{distance, elevation};
    }

    /**
     * Start a smooth fly-over transition to the next island.
     * Called when Tab is pressed in ISLAND_ORBIT mode.
     *
     * @param currentOrbitCenter the current orbit center (copied as transition start)
     */
    void cycleToNextIsland(Vector3f currentOrbitCenter) {
        if (islandPeaks.isEmpty()) return;
        transFrom.set(currentOrbitCenter);
        currentIslandIdx = (currentIslandIdx + 1) % islandPeaks.size();
        var peak = islandPeaks.get(currentIslandIdx);
        transTo.set(peak.x, peak.y * 0.4f, peak.z);
        // Duration scales with distance (longer for farther islands)
        float dist = transFrom.distance(transTo);
        transTotalDuration = ISLAND_ROTATE_DURATION
                + Math.min(ISLAND_FLY_DURATION, dist / 2000f + 1.0f);
        transTimer = transTotalDuration;
        System.out.printf("Island Orbit: flying to island %d/%d (dist=%.0f, elev=%.0f)%n",
                currentIslandIdx + 1, islandPeaks.size(), dist, peak.y);
    }

    /**
     * Update the island orbit animation. Handles the two-phase transition
     * (rotate toward target island, then fly over with ease-in-out).
     *
     * @param tpf time per frame
     * @param orbitAzimuth current orbit azimuth (modified via returned value)
     * @return the updated orbit azimuth, or NaN if no azimuth change needed.
     *         The caller should also read {@link #getOrbitCenter()} for the
     *         updated center, and check {@link #getOrbitDistance()} and
     *         {@link #getOrbitElevation()} after transition completes.
     */
    float update(float tpf, float orbitAzimuth) {
        transitionJustEnded = false;
        if (transTimer <= 0) {
            return Float.NaN;
        }

        transTimer -= tpf;
        float flyDuration = transTotalDuration - ISLAND_ROTATE_DURATION;
        float elapsed = transTotalDuration - transTimer;

        if (elapsed < ISLAND_ROTATE_DURATION) {
            // Phase 1: rotate to face the target island
            Vector3f dir = transTo.subtract(orbitCenter).normalizeLocal();
            float targetAzimuth = FastMath.atan2(dir.x, dir.z);
            float angleDiff = targetAzimuth - orbitAzimuth;
            while (angleDiff > FastMath.PI) angleDiff -= FastMath.TWO_PI;
            while (angleDiff < -FastMath.PI) angleDiff += FastMath.TWO_PI;
            orbitAzimuth += angleDiff * Math.min(1f, tpf * 4f);
        } else {
            // Phase 2: smooth fly-over with ease-in-out
            float flyT = (elapsed - ISLAND_ROTATE_DURATION) / Math.max(0.01f, flyDuration);
            flyT = Math.min(1f, flyT);
            // Smoothstep for ease-in-out
            float smooth = flyT * flyT * (3 - 2 * flyT);
            orbitCenter.interpolateLocal(transFrom, transTo, smooth);
            // Rise up during flight, descend at destination
            float arc = (float) Math.sin(smooth * Math.PI) * 100;
            orbitCenter.y += arc;
        }

        if (transTimer <= 0) {
            // Arrived: snap to final position
            orbitCenter.set(transTo);
            transitionJustEnded = true;
        }

        return orbitAzimuth;
    }

    /**
     * Returns the orbit center managed by this state. The caller should
     * copy this into its own orbit center when in ISLAND_ORBIT mode.
     */
    Vector3f getOrbitCenter() {
        return orbitCenter;
    }

    /**
     * Returns the recommended orbit distance after a transition completes.
     * Only meaningful when {@link #isTransitioning()} just became false.
     */
    float getOrbitDistance() {
        var peak = islandPeaks.get(currentIslandIdx);
        return Math.max(200, peak.y * 4);
    }

    /**
     * Returns the recommended orbit elevation after a transition completes.
     * Only meaningful when {@link #isTransitioning()} just became false.
     */
    float getOrbitElevation() {
        return 0.35f;
    }

    /** Whether a fly-over transition is currently in progress. */
    boolean isTransitioning() {
        return transTimer > 0;
    }

    /** Whether a transition just ended on the last {@link #update} call. */
    boolean transitionJustEnded() {
        return transitionJustEnded;
    }

    /** Number of detected island peaks. */
    int getIslandCount() {
        return islandPeaks.size();
    }

    /** Index of the current island (0-based). */
    int getCurrentIslandIdx() {
        return currentIslandIdx;
    }

    /** Direct access to the island peaks list (for screenshot tour). */
    List<Vector3f> getIslandPeaks() {
        return islandPeaks;
    }
}
