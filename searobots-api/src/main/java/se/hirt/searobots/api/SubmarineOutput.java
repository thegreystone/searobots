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
package se.hirt.searobots.api;

public interface SubmarineOutput {
    void setRudder(double value);
    void setSternPlanes(double value);
    void setThrottle(double value);
    void setBallast(double value);

    /**
     * Sets a short status string for debugging and match display (e.g. current
     * state machine state). Truncated to 40 characters by the engine.
     */
    default void setStatus(String status) {}

    /**
     * Emits an active sonar ping (~220 dB). Returns bearing + range for all
     * detected contacts, but reveals your position to everyone on the map.
     * Subject to cooldown (250 ticks / 5 seconds).
     */
    default void activeSonarPing() {}

    /**
     * Publishes an estimated contact position for viewer visualization and
     * replay recording. May be called multiple times per tick for multiple
     * contacts. Has no effect on the simulation.
     */
    default void publishContactEstimate(ContactEstimate estimate) {}

    /**
     * Publishes a navigation waypoint for viewer visualization.
     * Call once per waypoint per tick. The viewer renders them as
     * depth-colored circles with connecting lines. Has no effect on
     * the simulation.
     */
    default void publishWaypoint(Waypoint waypoint) {}

    /**
     * Publishes a torpedo firing solution for viewer visualization and
     * match recording. Call once per tick when the controller believes
     * it has a viable solution; do not call when there is no solution.
     *
     * <p>The controller decides its own criteria for what constitutes a
     * valid firing solution (range, uncertainty, target geometry, environment).
     */
    default void publishFiringSolution(FiringSolution solution) {}

    /**
     * Launches a torpedo. The torpedo is autonomous after launch: no
     * further communication is possible. The mission data string is the
     * only information the torpedo controller receives from the sub.
     * Torpedoes exit fixed forward tubes in the submarine's current
     * heading and pitch, then steer autonomously after ejection.
     *
     * <p>Requires torpedoesRemaining > 0. Ignored if no torpedoes left.
     * Creates a launch noise transient on the submarine.
     */
    default void launchTorpedo(TorpedoLaunchCommand command) {}

    /**
     * Engages or disengages the engine clutch. When disengaged, the prop
     * freewheels: no thrust, no engine braking, minimal machinery noise.
     * This gives the longest coast and quietest operation while moving.
     */
    default void setEngineClutch(boolean engaged) {}

    /**
     * Publishes a strategic waypoint for viewer visualization.
     * Strategic waypoints represent high-level mission goals (where and why),
     * while regular waypoints represent the A* navigation route (how).
     * Has no effect on the simulation.
     *
     * @param waypoint  the waypoint position and active flag
     * @param purpose   the tactical purpose (e.g. PATROL, INTERCEPT, EVADE)
     */
    default void publishStrategicWaypoint(Waypoint waypoint, Purpose purpose) {}
}
