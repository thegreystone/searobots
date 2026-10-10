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
import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.ThermalLayer;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/** Sonar interactions between submarines and torpedoes, including a ping's complete lifecycle. */
class SonarInteractionTest {

	private static final TerrainMap TERRAIN = terrain(false);

	@Test
	void submarinePingIsHeardByEveryTorpedoOnlyOnItsEmissionTick() {
		var sonar = new SonarModel(42);
		var source = submarine(0, new Vec3(0, 0, -200), 0);
		var north = torpedo(100, 99, new Vec3(0, 5000, -200), Math.PI);
		var east = torpedo(101, 99, new Vec3(5000, 0, -200), 1.5 * Math.PI);
		var subs = List.of(source);
		var torps = List.of(north, east);

		var silent = sonar.computeContacts(0, subs, torps, TERRAIN, List.of());
		assertTrue(silent.get(north.id()).passiveContacts().isEmpty());
		assertTrue(silent.get(east.id()).passiveContacts().isEmpty());

		source.activeSonarPing();
		var emission = sonar.computeContacts(1, subs, torps, TERRAIN, List.of());
		assertPingBearing(emission.get(north.id()).passiveContacts(), Math.PI);
		assertPingBearing(emission.get(east.id()).passiveContacts(), 1.5 * Math.PI);
		assertFalse(source.pingRequested(), "A ping must be consumed after all listeners hear it");

		var following = sonar.computeContacts(2, subs, torps, TERRAIN, List.of());
		assertTrue(following.get(north.id()).passiveContacts().isEmpty(), "The ping is not continuous noise");
		assertTrue(following.get(east.id()).passiveContacts().isEmpty());
	}

	@Test
	void torpedoPingIsHeardByEverySubmarineOnlyOnItsEmissionTick() {
		var sonar = new SonarModel(42);
		var source = torpedo(100, 99, new Vec3(0, 0, -200), 0);
		var north = submarine(0, new Vec3(0, 5000, -200), Math.PI);
		var east = submarine(1, new Vec3(5000, 0, -200), 1.5 * Math.PI);
		var subs = List.of(north, east);
		var torps = List.of(source);

		var silent = sonar.computeContacts(0, subs, torps, TERRAIN, List.of());
		assertTrue(silent.get(north.id()).passiveContacts().isEmpty());
		assertTrue(silent.get(east.id()).passiveContacts().isEmpty());

		source.createOutput().activeSonarPing();
		var emission = sonar.computeContacts(1, subs, torps, TERRAIN, List.of());
		assertPingBearing(emission.get(north.id()).passiveContacts(), Math.PI);
		assertPingBearing(emission.get(east.id()).passiveContacts(), 1.5 * Math.PI);
		assertFalse(source.pingRequested());

		var following = sonar.computeContacts(2, subs, torps, TERRAIN, List.of());
		assertTrue(following.get(north.id()).passiveContacts().isEmpty());
		assertTrue(following.get(east.id()).passiveContacts().isEmpty());
	}

	@Test
	void submarinePingReturnsNoisyRangeAndDepthForAFastTorpedo() {
		var position = new Vec3(120, 160, -260);
		var first = activeTorpedoEcho(42, position, 0, TERRAIN, List.of());
		var second = activeTorpedoEcho(43, position, 0, TERRAIN, List.of());
		double actualRange = new Vec3(0, 0, -200).distanceTo(position);

		assertTrue(first.isActive());
		assertEquals(actualRange, first.range(), 25, "The echo should resolve the nearby weapon's range");
		assertEquals(position.z(), first.estimatedDepth(), 25);
		assertTrue(first.rangeUncertainty() > 0, "Active measurements retain range uncertainty");
		assertEquals(SonarContact.Classification.TORPEDO, first.classification());
		assertNotEquals(first.range(), second.range(), "Range measurements must retain sensor noise");
		assertNotEquals(first.estimatedDepth(), second.estimatedDepth(), "Depth is measured, not exact telemetry");
	}

	@Test
	void submarineCanEchoItsOwnFreeWeapon() {
		var sonar = new SonarModel(42);
		var owner = submarine(0, new Vec3(0, 0, -200), 0);
		var weapon = torpedo(100, owner.id(), new Vec3(0, 500, -200), 0);
		owner.activeSonarPing();

		var result = sonar.computeContacts(0, List.of(owner), List.of(weapon), TERRAIN, List.of()).get(owner.id());
		assertEquals(1, result.activeReturns().size(), "Sonar has no perfect friend-or-foe filter");
		assertEquals(500, result.activeReturns().getFirst().range(), 40);
	}

	@Test
	void pingCooldownsAreReportedImmediatelyAndTickDownOnceWithoutRepeatedEmission() {
		var sonar = new SonarModel(42);
		var sub = submarine(0, new Vec3(0, 0, -200), 0);
		var torp = torpedo(100, 99, new Vec3(0, 5000, -200), Math.PI);
		var subs = List.of(sub);
		var torps = List.of(torp);
		sub.activeSonarPing();
		torp.createOutput().activeSonarPing();

		var emission = sonar.computeContacts(0, subs, torps, TERRAIN, List.of());
		assertEquals(250, emission.get(sub.id()).cooldownTicks());
		assertEquals(50, emission.get(torp.id()).cooldownTicks());
		assertEquals(sub.activeSonarCooldown(), emission.get(sub.id()).cooldownTicks());
		assertEquals(torp.activeSonarCooldown(), emission.get(torp.id()).cooldownTicks());
		assertFalse(sub.pingRequested());
		assertFalse(torp.pingRequested());

		sonar.postTick(subs);
		torp.decrementSonarCooldown();
		assertEquals(249, sub.activeSonarCooldown());
		assertEquals(49, torp.activeSonarCooldown());
		sub.activeSonarPing();
		torp.createOutput().activeSonarPing();
		var following = sonar.computeContacts(1, subs, torps, TERRAIN, List.of());
		for (var result : following.values()) {
			assertTrue(result.activeReturns().isEmpty(), "Cooldown prevents another echo cycle");
			assertTrue(result.passiveContacts().isEmpty(), "Cooldown also prevents another loud transmission");
		}
		assertEquals(249, following.get(sub.id()).cooldownTicks());
		assertEquals(49, following.get(torp.id()).cooldownTicks());
	}

	@Test
	void destroyedAndForfeitedVehiclesAreNeitherSourcesNorListeners() {
		var sonar = new SonarModel(42);
		var listener = submarine(0, new Vec3(0, 0, -200), 0);
		var destroyed = submarine(1, new Vec3(0, 100, -200), Math.PI);
		var forfeited = submarine(2, new Vec3(100, 0, -200), Math.PI);
		var weapon = torpedo(100, 99, new Vec3(0, 200, -200), Math.PI);
		destroyed.activeSonarPing();
		forfeited.activeSonarPing();
		weapon.createOutput().activeSonarPing();
		destroyed.setHp(0);
		forfeited.setForfeited(true);
		weapon.kill();
		listener.activeSonarPing();

		var results = sonar.computeContacts(0, List.of(listener, destroyed, forfeited), List.of(weapon), TERRAIN,
				List.of());
		assertTrue(results.get(listener.id()).passiveContacts().isEmpty());
		assertTrue(results.get(listener.id()).activeReturns().isEmpty());
		assertTrue(results.get(destroyed.id()).passiveContacts().isEmpty());
		assertTrue(results.get(destroyed.id()).activeReturns().isEmpty());
		assertTrue(results.get(forfeited.id()).passiveContacts().isEmpty());
		assertTrue(results.get(forfeited.id()).activeReturns().isEmpty());
		assertFalse(results.containsKey(weapon.id()));
	}

	@Test
	void containedTorpedoIsExcludedUntilItLeavesItsTube() {
		var sonar = new SonarModel(42);
		var listener = submarine(0, new Vec3(0, 0, -200), 0);
		var owner = submarine(1, new Vec3(0, 500, -200), Math.PI);
		var weapon = torpedo(100, owner.id(), owner.pose().position(), owner.heading());
		weapon.loadIntoTube(owner, TorpedoTubes.TUBES.getFirst(), 0.02, "");
		weapon.setSourceLevelDb(160);
		weapon.createOutput().activeSonarPing();
		listener.activeSonarPing();

		// The owner is omitted to isolate sonar interactions with the contained weapon.
		var contained = sonar.computeContacts(0, List.of(listener), List.of(weapon), TERRAIN, List.of());
		assertTrue(contained.get(listener.id()).passiveContacts().isEmpty(), "The hull contains the weapon");
		assertTrue(contained.get(listener.id()).activeReturns().isEmpty(), "No separate echo while in the tube");
		assertFalse(contained.containsKey(weapon.id()), "A contained weapon is not an independent listener");
		assertEquals(0, weapon.activeSonarCooldown(), "A contained weapon cannot consume a ping cycle");

		weapon.leaveTube();
		var nextListener = submarine(0, new Vec3(0, 0, -200), 0);
		nextListener.activeSonarPing();
		var released = sonar.computeContacts(1, List.of(nextListener), List.of(weapon), TERRAIN, List.of());
		assertEquals(1, released.get(nextListener.id()).passiveContacts().size());
		assertEquals(1, released.get(nextListener.id()).activeReturns().size());
		assertFalse(released.get(weapon.id()).passiveContacts().isEmpty(), "Its receiver works after release");
	}

	@Test
	void activeTorpedoEchoIsAttenuatedAcrossAThermalLayer() {
		var position = new Vec3(0, 1000, -80);
		var clear = activeTorpedoEcho(42, position, 0, TERRAIN, List.of());
		var attenuated = activeTorpedoEcho(42, position, 0, TERRAIN, List.of(new ThermalLayer(-120, 40, 15, 5)));

		assertTrue(clear.signalExcess() - attenuated.signalExcess() > 10,
				"An active echo crosses the thermocline in both directions");
		assertTrue(attenuated.signalExcess() > SonarModel.DETECTION_THRESHOLD_DB);
	}

	@Test
	void submarineBafflesDegradeActiveTorpedoEcho() {
		var position = new Vec3(0, 500, -200);
		var forward = activeTorpedoEcho(42, position, 0, TERRAIN, List.of());
		var behind = activeTorpedoEcho(42, position, Math.PI, TERRAIN, List.of());

		assertEquals(20, forward.signalExcess() - behind.signalExcess(), 0.01,
				"The submarine receiver's baffle penalty applies to torpedo echoes too");
	}

	@Test
	void islandBlocksActiveTorpedoEcho() {
		var listener = submarine(0, new Vec3(0, -400, -200), 0);
		var weapon = torpedo(100, 99, new Vec3(0, 400, -200), Math.PI);
		listener.activeSonarPing();
		var clear = new SonarModel(42).computeContacts(0, List.of(listener), List.of(weapon), TERRAIN, List.of());
		assertEquals(1, clear.get(listener.id()).activeReturns().size());

		var blockedListener = submarine(0, listener.pose().position(), 0);
		blockedListener.activeSonarPing();
		var blocked = new SonarModel(42).computeContacts(0, List.of(blockedListener), List.of(weapon), terrain(true),
				List.of());
		assertTrue(blocked.get(blockedListener.id()).activeReturns().isEmpty(), "Rock blocks the echo path");
	}

	@Test
	void queuedRequestsCannotEmitIfCooldownHasBecomeActive() {
		var sonar = new SonarModel(42);
		var source = submarine(0, new Vec3(0, 0, -200), 0);
		var listener = submarine(1, new Vec3(0, 5000, -200), Math.PI);
		var weapon = torpedo(100, 99, new Vec3(5000, 0, -200), 1.5 * Math.PI);
		source.activeSonarPing();
		weapon.createOutput().activeSonarPing();
		source.setActiveSonarCooldown(100);
		weapon.setActiveSonarCooldown(20);

		var results = sonar.computeContacts(0, List.of(source, listener), List.of(weapon), TERRAIN, List.of());
		for (var result : results.values()) {
			assertTrue(result.passiveContacts().isEmpty(), "A rejected request produces no source-level boost");
			assertTrue(result.activeReturns().isEmpty(), "A rejected request produces no echoes");
		}
		assertEquals(100, source.activeSonarCooldown());
		assertEquals(20, weapon.activeSonarCooldown());
		assertFalse(source.pingRequested());
		assertFalse(weapon.pingRequested());
	}

	private static void assertPingBearing(List<SonarContact> contacts, double expectedBearing) {
		assertEquals(1, contacts.size());
		var contact = contacts.getFirst();
		assertTrue(contact.signalExcess() > 100, "A quiet vehicle's active transmission is loud at 5 km");
		assertEquals(expectedBearing, contact.bearing(), Math.toRadians(2));
		assertFalse(contact.isActive(), "Hearing someone else's ping is a passive observation");
		assertEquals(0, contact.range(), "Hearing a ping does not reveal exact range");
		assertTrue(Double.isNaN(contact.estimatedDepth()));
	}

	private static SonarContact activeTorpedoEcho(
		long seed, Vec3 position, double heading, TerrainMap terrain, List<ThermalLayer> layers) {
		var listener = submarine(0, new Vec3(0, 0, -200), heading);
		var weapon = torpedo(100, 99, position, Math.PI);
		weapon.setSpeed(23);
		listener.activeSonarPing();
		var returns = new SonarModel(seed).computeContacts(0, List.of(listener), List.of(weapon), terrain, layers)
				.get(listener.id()).activeReturns();
		assertEquals(1, returns.size(), "A submarine ping should illuminate a free torpedo");
		return returns.getFirst();
	}

	private static SubmarineEntity submarine(int id, Vec3 position, double heading) {
		var entity = new SubmarineEntity(VehicleConfig.submarine(), id, null, position, heading, Color.GREEN, 1000);
		entity.setSourceLevelDb(80);
		return entity;
	}

	private static TorpedoEntity torpedo(int id, int ownerId, Vec3 position, double heading) {
		var entity = new TorpedoEntity(id, ownerId, VehicleConfig.torpedo(), null, position, heading, 0, 20,
				Color.GREEN);
		entity.setSourceLevelDb(80);
		return entity;
	}

	private static TerrainMap terrain(boolean island) {
		int size = 201;
		double[] elevations = new double[size * size];
		Arrays.fill(elevations, -500);
		if (island) {
			for (int row = 99; row <= 101; row++) {
				Arrays.fill(elevations, row * size, (row + 1) * size, 20);
			}
		}
		return new TerrainMap(elevations, size, size, -1000, -1000, 10);
	}
}
