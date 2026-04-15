/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.api;

/**
 * Output interface for torpedo controllers. Controls the torpedo's
 * steering, propulsion, sonar, and detonation.
 */
public interface TorpedoOutput {
    /** Set rudder deflection: -1 (full port) to +1 (full starboard). */
    void setRudder(double value);

    /** Set stern planes deflection: -1 (dive) to +1 (climb). */
    void setSternPlanes(double value);

    /** Set throttle: 0 (stop) to 1 (full ahead). Negative not supported. */
    void setThrottle(double value);

    /** Emit an active sonar ping. Reveals the torpedo's position to all listeners. */
    default void activeSonarPing() {}

    /** Manually detonate the torpedo's warhead at the current position. */
    default void detonate() {}

    /** Publish current target position for viewer visualization. */
    default void publishTarget(double x, double y, double z) {}

    /**
     * Publish torpedo guidance diagnostics for analysis.
     *
     * @param estX       estimated target X (where we think the target is)
     * @param estY       estimated target Y
     * @param estZ       estimated target Z (depth)
     * @param estHeading estimated target heading (radians, NaN if unknown)
     * @param estSpeed   estimated target speed (m/s)
     * @param intX       intercept point X (where we're steering toward)
     * @param intY       intercept point Y
     * @param intZ       intercept point Z
     * @param phase      guidance phase name (e.g. "TRANSIT", "ACQUISITION", "TERMINAL")
     */
    default void publishDiagnostics(double estX, double estY, double estZ,
                                     double estHeading, double estSpeed,
                                     double intX, double intY, double intZ,
                                     String phase) {}
}
