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

public interface SubmarineController {
    /**
     * Short display name for this controller, shown in the viewer HUD
     * and match logs. Keep it brief (under 20 characters).
     */
    default String name() { return getClass().getSimpleName(); }

    default void onMatchStart(MatchContext context) {}
    void onTick(SubmarineInput input, SubmarineOutput output);
    default void onMatchEnd(MatchResult result) {}

    /**
     * Sets mandatory navigation objectives. When set, the controller MUST
     * navigate to these waypoints in order before generating its own patrol
     * waypoints. The objectives are not replanned or overridden until all
     * have been reached.
     *
     * <p>Called after {@link #onMatchStart} but before the first tick.
     * Used by the competition framework and test harnesses.
     *
     * @param objectives ordered list of waypoints to navigate to
     */
    default void setObjectives(java.util.List<StrategicWaypoint> objectives) {}
}
