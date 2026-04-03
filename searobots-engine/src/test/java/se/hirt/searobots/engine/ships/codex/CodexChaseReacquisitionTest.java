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

class CodexChaseReacquisitionTest {
    private static final double DT = 1.0 / 50.0;

    @Test
    void recentActiveChaseRepingsInsteadOfFallingBackToPassiveTrack() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        var self = submarineState(new Vec3(0.0, 0.0, -140.0), 0.0, 7.5);

        controller.onTick(new TestHelpers.TestInput(
                0L,
                DT,
                self,
                environment(world),
                List.of(),
                List.of(activeContact(0.0, 3_400.0)),
                250),
                new TestHelpers.CapturedOutput());

        TestHelpers.CapturedOutput finalOutput = null;
        for (long tick = 1; tick <= 560; tick++) {
            var output = new TestHelpers.CapturedOutput();
            controller.onTick(new TestHelpers.TestInput(
                    tick,
                    DT,
                    self,
                    environment(world),
                    List.of(passiveContact(0.0)),
                    List.of(),
                    tick == 560 ? 0 : 250),
                    output);
            finalOutput = output;
        }

        assertTrue(finalOutput != null && finalOutput.pinged,
                "Codex should reping to rebuild a decaying chase track instead of drifting on passive.");
    }

    private static EnvironmentSnapshot environment(GeneratedWorld world) {
        return new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
    }

    private static SubmarineState submarineState(Vec3 position, double heading, double speed) {
        var pose = new Pose(position, heading, 0.0, 0.0);
        var velocity = new Velocity(new Vec3(0.0, speed, 0.0), Vec3.ZERO);
        return new SubmarineState(pose, velocity, 1_000, 8);
    }

    private static SonarContact activeContact(double bearing, double range) {
        return new SonarContact(
                bearing,
                28.0,
                range,
                true,
                8.0,
                Math.toRadians(0.5),
                75.0,
                220.0,
                0.72,
                0.0,
                -140.0);
    }

    private static SonarContact passiveContact(double bearing) {
        return new SonarContact(
                bearing,
                12.0,
                0.0,
                false,
                8.0,
                Math.toRadians(2.0),
                0.0,
                145.0,
                0.34,
                0.0,
                Double.NaN);
    }
}
