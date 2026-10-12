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
import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.SimpleTorpedoController;

import java.awt.Color;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class WreckProximityFuseTest {

	private static final MatchConfig CONFIG = MatchConfig.withDefaults(17);

	@Test
	void armedWarheadInsideASubmarineOrSunkenShipWreckDetonates() {
		for (var config : List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip())) {
			var wreck = wreck(config);
			var torpedo = torpedo(new Vec3(0, 0, -100), 99, true);
			assertEquals(0, HullGeometry.bowDistanceToHull(torpedo, wreck), 1e-9);
			SimulationLoop.checkProximityFuse(torpedo, List.of(wreck), CONFIG, 1000);
			assertFalse(torpedo.alive());
			assertTrue(torpedo.detonated());
			assertTrue(torpedo.explosionProcessed());
			assertEquals(0, wreck.hp(), "An impact must not revive a wreck or drive its HP negative.");
		}
	}

	@Test
	void wreckDetonationDamagesNearbyLivingHullsOnceAndPublishesAnExplosion() {
		var wreck = wreck(VehicleConfig.submarine());
		var living = new SubmarineEntity(VehicleConfig.submarine(), 2, new DefaultAttackSub(), new Vec3(20, 0, -100), 0,
				Color.BLUE, 1000);
		var torpedo = torpedo(new Vec3(0, 0, -100), 99, true);
		var bodies = List.of(wreck, living);
		SimulationLoop.checkProximityFuse(torpedo, bodies, CONFIG, 1000);
		assertTrue(torpedo.detonated());
		assertTrue(living.hp() > 0 && living.hp() < 1000, "The blast must affect a nearby surviving hull.");
		int afterBlast = living.hp();
		var events = living.drainExplosionEvents();
		assertEquals(1, events.size());
		assertTrue(events.getFirst().submarineHit());
		assertFalse(events.getFirst().ownTorpedo());
		SimulationLoop.checkProximityFuse(torpedo, bodies, CONFIG, 1001);
		assertEquals(afterBlast, living.hp(), "The same warhead must not apply damage twice.");
		assertTrue(living.drainExplosionEvents().isEmpty());
	}

	@Test
	void aWreckDoesNotBypassTheArmingDelay() {
		var wreck = wreck(VehicleConfig.submarine());
		var torpedo = torpedo(new Vec3(0, 0, -100), 99, false);
		for (int tick = 0; tick < 99; tick++) {
			SimulationLoop.checkProximityFuse(torpedo, List.of(wreck), CONFIG, tick);
			assertTrue(torpedo.alive(), "An unarmed warhead must retain the existing launch safety delay.");
		}
		SimulationLoop.checkProximityFuse(torpedo, List.of(wreck), CONFIG, 99);
		assertTrue(torpedo.detonated(), "The physical wreck must trigger the fuse once it arms.");
	}

	@Test
	void ownerLaunchClearanceAppliesUntilTheGracePeriodEnds() {
		var wreck = wreck(VehicleConfig.submarine());
		var torpedo = torpedo(new Vec3(0, 0, -100), wreck.id(), true);
		torpedo.setLaunchTick(100);
		SimulationLoop.checkProximityFuse(torpedo, List.of(wreck), CONFIG, 349);
		assertTrue(torpedo.alive());
		SimulationLoop.checkProximityFuse(torpedo, List.of(wreck), CONFIG, 350);
		assertTrue(torpedo.detonated(), "The owner's wreck becomes a fuse target after launch clearance.");
	}

	@Test
	void hullsOutsideFuseRangeAndForfeitedHullsDoNotTriggerDetonation() {
		var wreck = wreck(VehicleConfig.submarine());
		var distant = torpedo(new Vec3(20, 0, -100), 99, true);
		assertTrue(HullGeometry.bowDistanceToHull(distant, wreck) > distant.fuseRadius());
		SimulationLoop.checkProximityFuse(distant, List.of(wreck), CONFIG, 1000);
		assertTrue(distant.alive());
		wreck.setForfeited(true);
		var inside = torpedo(new Vec3(0, 0, -100), 99, true);
		SimulationLoop.checkProximityFuse(inside, List.of(wreck), CONFIG, 1000);
		assertTrue(inside.alive(), "Forfeited entities are no longer physical participants.");
	}

	@Test
	void torpedoesInTheirTubesDoNotRunTheirProximityFuse() {
		var wreck = wreck(VehicleConfig.submarine());
		var torpedo = torpedo(new Vec3(0, 0, -100), wreck.id(), true);
		torpedo.loadIntoTube(wreck, TorpedoTubes.TUBES.getFirst(), 0.02, "");
		assertTrue(torpedo.inTube());
		SimulationLoop.checkProximityFuse(torpedo, List.of(wreck), CONFIG, 1000);
		assertTrue(torpedo.alive());
	}

	private static SubmarineEntity wreck(VehicleConfig config) {
		var wreck = new SubmarineEntity(config, 1, new DefaultAttackSub(), new Vec3(0, 0, -100), 0, Color.BLUE, 1000);
		wreck.setHp(0);
		wreck.setZ(-100);
		return wreck;
	}

	private static TorpedoEntity torpedo(Vec3 position, int ownerId, boolean armed) {
		var torpedo = new TorpedoEntity(1000, ownerId, VehicleConfig.torpedo(), new SimpleTorpedoController(), position,
				0, 0, 5, Color.BLUE);
		if (armed) {
			for (int tick = 0; tick < 100; tick++)
				torpedo.decrementArmingDelay();
		}
		return torpedo;
	}
}
