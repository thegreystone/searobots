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
package se.hirt.searobots.engine.ships.codex;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.TestHelpers;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

class CodexExplosionFeedbackTest {
    private static final double DT = 1.0 / 50.0;

    @Test
    void ownTorpedoHitCreatesPrecisePingTrack() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        var self = submarineState(new Vec3(0.0, 0.0, -140.0), 0.0, 7.5);
        var output = new TestHelpers.CapturedOutput();
        controller.onTick(new ExplosionInput(
                600L,
                DT,
                self,
                environment(world),
                List.of(),
                List.of(),
                0,
                List.of(new ExplosionEvent(0.0, 1_500.0, true, 3, true))),
                output);

        assertTrue(!output.contactEstimates.isEmpty(), "Codex should publish a track after a confirmed torpedo hit.");
        ContactEstimate estimate = output.contactEstimates.get(0);
        assertTrue("ping".equals(estimate.label()),
                "A confirmed torpedo hit should count as a fresh high-confidence fix, got " + estimate.label());
        assertTrue(Math.abs(estimate.x()) < 30.0 && Math.abs(estimate.y() - 1_500.0) < 30.0,
                "The published track should sit on the explosion position, got (" + estimate.x() + ", " + estimate.y() + ")");
        assertTrue(estimate.uncertaintyRadius() <= 80.0,
                "A confirmed hit should collapse uncertainty, got " + estimate.uncertaintyRadius());
        assertTrue(estimate.contactAlive() > 0.95,
                "A confirmed hit should reset contact-alive confidence, got " + estimate.contactAlive());
    }

    private static EnvironmentSnapshot environment(GeneratedWorld world) {
        return new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
    }

    private static SubmarineState submarineState(Vec3 position, double heading, double speed) {
        var pose = new Pose(position, heading, 0.0, 0.0);
        var velocity = new Velocity(new Vec3(0.0, speed, 0.0), Vec3.ZERO);
        return new SubmarineState(pose, velocity, 1_000, 8);
    }

    private record ExplosionInput(long tick, double deltaTimeSeconds,
                                  SubmarineState self, EnvironmentSnapshot environment,
                                  List<SonarContact> sonarContacts,
                                  List<SonarContact> activeSonarReturns,
                                  int activeSonarCooldownTicks,
                                  List<ExplosionEvent> explosionEvents)
            implements SubmarineInput {}
}
