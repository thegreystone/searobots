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
}
