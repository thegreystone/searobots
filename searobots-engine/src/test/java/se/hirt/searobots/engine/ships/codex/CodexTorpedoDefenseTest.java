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
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.EnvironmentSnapshot;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.Pose;
import se.hirt.searobots.api.Purpose;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.SubmarineState;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Velocity;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.TestHelpers;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CodexTorpedoDefenseTest {
    private static final double DT = 1.0 / 50.0;

    @Test
    void classifiedTorpedoThreatOverridesShotWithEvadePosture() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        var self = submarineState(new Vec3(0.0, 0.0, -140.0), 0.0, 7.5);
        var output = new TestHelpers.CapturedOutput();
        controller.onTick(new TestHelpers.TestInput(
                700L,
                DT,
                self,
                environment(world),
                List.of(),
                List.of(
                        activeSubmarine(0.0, 1_000.0),
                        activeTorpedo(Math.toRadians(24.0), 700.0)),
                250),
                output);

        assertTrue(output.status != null && output.status.startsWith("!"),
                "A live torpedo threat should be visible in status, got " + output.status);
        assertFalse(output.strategicPurposes.isEmpty(),
                "Codex should publish a strategic response to the torpedo threat.");
        assertEquals(Purpose.EVADE, output.strategicPurposes.get(output.strategicPurposes.size() - 1),
                "Torpedo threat should override combat planning with an evade waypoint.");
        assertEquals(0, output.launchedTorpedoCount,
                "Codex should suppress torpedo launch while handling an incoming torpedo.");
        assertTrue(output.throttle >= 0.56,
                "Torpedo defense should command decisive throttle, got " + output.throttle);
    }

    @Test
    void faintPassiveTorpedoClassificationStillTriggersDefense() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        var self = submarineState(new Vec3(0.0, 0.0, -140.0), 0.0, 7.5);
        controller.onTick(new TestHelpers.TestInput(
                700L,
                DT,
                self,
                environment(world),
                List.of(passiveTorpedo(Math.toRadians(18.0), 4.0)),
                List.of(activeSubmarine(0.0, 1_000.0)),
                250),
                new TestHelpers.CapturedOutput());

        var output = new TestHelpers.CapturedOutput();
        controller.onTick(new TestHelpers.TestInput(
                701L,
                DT,
                self,
                environment(world),
                List.of(passiveTorpedo(Math.toRadians(18.0), 4.0)),
                List.of(activeSubmarine(0.0, 1_000.0)),
                250),
                output);

        assertTrue(output.status != null && output.status.startsWith("!"),
                "Codex should trust a classified torpedo even when the passive signal is faint.");
        assertEquals(Purpose.EVADE, output.strategicPurposes.get(output.strategicPurposes.size() - 1),
                "Faint classified torpedo contact should still trigger evade planning.");
        assertEquals(0, output.launchedTorpedoCount,
                "Faint classified torpedo contact should still suppress offensive launch.");
    }

    @Test
    void faintFarPassiveTorpedoDoesNotForceImmediateDefense() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        var self = submarineState(new Vec3(0.0, 0.0, -140.0), 0.0, 7.5);
        var output = new TestHelpers.CapturedOutput();
        controller.onTick(new TestHelpers.TestInput(
                700L,
                DT,
                self,
                environment(world),
                List.of(passiveTorpedo(Math.toRadians(18.0), 4.0)),
                List.of(activeSubmarine(0.0, 2_600.0)),
                250),
                output);

        assertTrue(output.status != null && !output.status.startsWith("!"),
                "A faint passive torpedo contact at long enemy range should not force immediate defense. Got " + output.status);
    }

    @Test
    void ownLaunchedTorpedoBearingDoesNotTriggerFalsePassiveDefense() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        var self = submarineState(new Vec3(0.0, 0.0, -140.0), 0.0, 7.5);
        var launchOutput = new TestHelpers.CapturedOutput();
        controller.onTick(new TestHelpers.TestInput(
                700L,
                DT,
                self,
                environment(world),
                List.of(),
                List.of(activeSubmarine(0.0, 1_000.0)),
                250),
                launchOutput);

        assertEquals(1, launchOutput.launchedTorpedoCount,
                "Setup should produce a Codex torpedo launch.");

        double ownTorpedoBearing = launchOutput.launchedTorpedo.bearing();
        var output = new TestHelpers.CapturedOutput();
        controller.onTick(new TestHelpers.TestInput(
                701L,
                DT,
                self,
                environment(world),
                List.of(passiveTorpedo(ownTorpedoBearing, 8.0)),
                List.of(activeSubmarine(0.0, 1_000.0)),
                250),
                output);

        assertTrue(output.status != null && !output.status.startsWith("!"),
                "Codex should not mistake its own outbound torpedo for an inbound threat. Got " + output.status);
    }

    @Test
    void torpedoLaunchBearingMatchesLeadTargetInMissionData() {
        var world = GeneratedWorld.deepFlat();
        var controller = new CodexAttackSub();
        controller.onMatchStart(new MatchContext(
                MatchConfig.withDefaults(0L),
                world.terrain(),
                world.thermalLayers(),
                new CurrentField(List.of())));

        Vec3 position = new Vec3(0.0, 0.0, -140.0);
        var self = submarineState(position, 0.0, 7.5);
        var output = new TestHelpers.CapturedOutput();
        controller.onTick(new TestHelpers.TestInput(
                700L,
                DT,
                self,
                environment(world),
                List.of(),
                List.of(activeSubmarine(0.0, 1_500.0, Math.toRadians(90.0), 12.0)),
                250),
                output);

        assertEquals(1, output.launchedTorpedoCount,
                "Setup should produce a Codex torpedo launch.");

        String[] missionParts = output.launchedTorpedo.missionData().split(";");
        double leadX = Double.parseDouble(missionParts[0]);
        double leadY = Double.parseDouble(missionParts[1]);
        double expectedBearing = Math.atan2(leadX - position.x(), leadY - position.y());
        if (expectedBearing < 0.0) {
            expectedBearing += 2.0 * Math.PI;
        }

        assertEquals(expectedBearing, output.launchedTorpedo.bearing(), 1.0e-6,
                "Launch bearing should follow the same lead point encoded in mission data.");
    }

    private static EnvironmentSnapshot environment(GeneratedWorld world) {
        return new EnvironmentSnapshot(world.terrain(), world.thermalLayers(), world.currentField());
    }

    private static SubmarineState submarineState(Vec3 position, double heading, double speed) {
        var pose = new Pose(position, heading, 0.0, 0.0);
        var velocity = new Velocity(
                new Vec3(Math.sin(heading) * speed, Math.cos(heading) * speed, 0.0),
                Vec3.ZERO);
        return new SubmarineState(pose, velocity, 1_000, 8);
    }

    private static SonarContact activeSubmarine(double bearing, double range) {
        return activeSubmarine(bearing, range, 0.0, 8.0);
    }

    private static SonarContact activeSubmarine(double bearing,
                                                double range,
                                                double estimatedHeading,
                                                double estimatedSpeed) {
        return new SonarContact(
                bearing,
                30.0,
                range,
                true,
                estimatedSpeed,
                Math.toRadians(0.4),
                70.0,
                210.0,
                0.86,
                estimatedHeading,
                -140.0,
                SonarContact.Classification.SUBMARINE);
    }

    private static SonarContact activeTorpedo(double bearing, double range) {
        return new SonarContact(
                bearing,
                14.0,
                range,
                true,
                25.0,
                Math.toRadians(0.3),
                45.0,
                115.0,
                0.65,
                bearing,
                -120.0,
                SonarContact.Classification.TORPEDO);
    }

    private static SonarContact passiveTorpedo(double bearing, double signalExcess) {
        return new SonarContact(
                bearing,
                signalExcess,
                0.0,
                false,
                25.0,
                Math.toRadians(1.2),
                0.0,
                112.0,
                0.20,
                bearing,
                Double.NaN,
                SonarContact.Classification.TORPEDO);
    }
}
