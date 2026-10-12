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
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.EnvironmentSnapshot;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.MatchContext;
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class DefaultAttackSubSonarTest {

	private static final TerrainMap TERRAIN = deepFlat();
	private static final CurrentField CURRENTS = new CurrentField(List.of());
	private static final EnvironmentSnapshot ENVIRONMENT = new EnvironmentSnapshot(TERRAIN, List.of(), CURRENTS);

	@Test
	void classifiedPassiveTorpedoTriggersEvasionWithoutASourceLevelEstimate() {
		var controller = controller();
		var listener = submarine(0, controller, new Vec3(0, 0, -200), 80);
		var weapon = new TorpedoEntity(100, 99, VehicleConfig.torpedo(), null, new Vec3(0, 1000, -200), Math.PI, 0, 20,
				Color.GREEN);
		weapon.setSpeed(23);
		weapon.setSourceLevelDb(140);
		var contacts = new SonarModel(42).computeContacts(0, List.of(listener), List.of(weapon), TERRAIN, List.of())
				.get(listener.id());
		var threat = contacts.passiveContacts().getFirst();
		assertEquals(SonarContact.Classification.TORPEDO, threat.classification());
		assertTrue(Double.isNaN(threat.estimatedSourceLevel()),
				"The acoustic classification remains available without independent radiated-level telemetry");

		controller.onTick(input(0, listener, contacts), new TestHelpers.CapturedOutput());

		assertEquals(DefaultAttackSub.State.EVADING, controller.state());
		assertFalse(controller.hasTrackedContact(), "An incoming weapon must not become the submarine target track");
	}

	@Test
	void ordinaryPassiveSubmarineContactStillBuildsATargetTrack() {
		var controller = controller();
		var listener = submarine(0, controller, new Vec3(0, 0, -200), 80);
		var source = submarine(1, null, new Vec3(0, 1000, -200), 104);
		source.setSpeed(8);
		var sonar = new SonarModel(42);
		for (int tick = 0; tick < 3; tick++) {
			var contacts = sonar.computeContacts(tick, List.of(listener, source), TERRAIN, List.of())
					.get(listener.id());
			assertNotEquals(SonarContact.Classification.TORPEDO,
					contacts.passiveContacts().getFirst().classification());
			controller.onTick(input(tick, listener, contacts), new TestHelpers.CapturedOutput());
		}

		assertEquals(DefaultAttackSub.State.TRACKING, controller.state());
		assertTrue(controller.hasTrackedContact(), "A confirmed submarine contact should remain an engagement target");
	}

	private static DefaultAttackSub controller() {
		var controller = new DefaultAttackSub();
		controller.onMatchStart(new MatchContext(MatchConfig.withDefaults(0), TERRAIN, List.of(), CURRENTS));
		return controller;
	}

	private static TestHelpers.TestInput input(long tick, SubmarineEntity listener, SonarModel.SonarResult contacts) {
		return new TestHelpers.TestInput(tick, 1.0 / 50, listener.state(), ENVIRONMENT, contacts.passiveContacts(),
				contacts.activeReturns(), contacts.cooldownTicks());
	}

	private static SubmarineEntity submarine(int id, DefaultAttackSub controller, Vec3 position, double sourceLevel) {
		var entity = new SubmarineEntity(VehicleConfig.submarine(), id, controller, position, 0, Color.GREEN, 1000);
		entity.setSourceLevelDb(sourceLevel);
		entity.setSpeed(5);
		return entity;
	}

	private static TerrainMap deepFlat() {
		int size = 201;
		double[] elevations = new double[size * size];
		Arrays.fill(elevations, -500);
		return new TerrainMap(elevations, size, size, -10000, -10000, 100);
	}
}
