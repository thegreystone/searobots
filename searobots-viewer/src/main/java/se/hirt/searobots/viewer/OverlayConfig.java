/*
 * Copyright (C) 2026 Marcus Hirt
 *                    (see TerrainViewer.java for full license text)
 */
package se.hirt.searobots.viewer;

/**
 * Shared overlay toggle state. Both the 2D map renderer and the 3D scene
 * read from this so that T/R/E/W/G toggles are consistent across views.
 */
final class OverlayConfig {
    volatile boolean trails = true;
    volatile boolean route = false;
    volatile boolean contactEstimates = true;
    volatile boolean waypoints = true;          // nav / autopilot waypoints
    volatile boolean strategicWaypoints = true;
    volatile boolean contours = true;           // 2D only
    volatile boolean currents = false;          // 2D only
}
