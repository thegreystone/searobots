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

import java.awt.Color;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;

import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.Vec2;
import se.hirt.searobots.api.VehicleConfig;

import static org.junit.jupiter.api.Assertions.*;

class SubmarineSweptTerrainTest {

	private static final VehicleConfig SUBMARINE = VehicleConfig.submarine();
	private static final MatchConfig MATCH = MatchConfig.withDefaults(42);
	private static final CurrentField STILL_WATER = new CurrentField(List.of());

	@Test
	void shallowFineGridCrestBetweenTwoClearEndpointPosesIsDetected() {
		var terrain = narrowCrest(0.1, -104.9999, -110, 0.1);
		var start = new Vec3(0, 0.02, -100);
		var end = new Vec3(0, 0.28, -100);
		assertClear(start, 0, 0, SUBMARINE, terrain);
		assertClear(end, 0, 0, SUBMARINE, terrain);

		var contact = SweptHullTerrain.firstContact(start, end, 0, 0, 0, 0, SUBMARINE, terrain);
		assertNotNull(contact, "A crest crossed by the physical keel must not disappear between ticks");
		assertTrue(contact.fraction() > 0 && contact.fraction() < 1);
		assertEquals(0.1, contact.position().y(), 3e-6);
		assertTrue(contact.localPoints().length > 0, "The response needs actual material points at first contact");
		assertTrue(
				HullGeometry.terrainPenetration(contact.position(), contact.heading(), contact.pitch(), SUBMARINE,
						terrain) < 1e-5,
				"The response must start at the crest's leading edge rather than after crossing it");
	}

	@Test
	void pointSweepStillFindsAnAlmostTangentCrestWhenTimeSamplingHitsItsBudget() {
		var terrain = narrowCrest(0.03, -104.999999, -110, 0.01);
		var start = new Vec3(0, -100, -100);
		var end = new Vec3(0, 100, -100);
		assertClear(start, 0, 0, SUBMARINE, terrain);
		assertClear(end, 0, 0, SUBMARINE, terrain);

		var contact = SweptHullTerrain.firstContact(start, end, 0, 0, 0, 0, SUBMARINE, terrain);
		assertNotNull(contact, "The exact material-point cell crossing must survive the bounded pose-sampling budget");
		assertEquals(0.03, contact.position().y(), 1e-7);
	}

	@Test
	void rotatingHullDetectsTerrainBetweenClearEndpointAttitudes() {
		var terrain = rowCrest(20, -102, -120);
		var position = new Vec3(0, 0, -100);
		assertClear(position, -Math.PI / 2, 0, SUBMARINE, terrain);
		assertClear(position, Math.PI / 2, 0, SUBMARINE, terrain);

		var contact = SweptHullTerrain.firstContact(position, position, -Math.PI / 2, Math.PI / 2, 0, 0, SUBMARINE,
				terrain);
		assertNotNull(contact, "The forebody must contact the ridge during its swept turn");
		assertTrue(contact.fraction() > 0 && contact.fraction() < 0.5);
		assertEquals(position, contact.position());
		assertTrue(contact.heading() > -Math.PI / 2 && contact.heading() < 0);
	}

	@Test
	void refinedTurnReturnsTheActualSupportingMaterialPointOnASlope() {
		var terrain = slopingFloor(1);
		var position = new Vec3(0, 0, -90);
		var envelope = HullGeometry.envelope(SUBMARINE);
		assertClear(position, 0, 0, SUBMARINE, terrain);

		var contact = SweptHullTerrain.firstContact(position, position, 0, Math.PI / 2, 0, 0, SUBMARINE, terrain);
		assertNotNull(contact);
		double beam = envelope.semiBeam(), length = envelope.semiLength(), height = envelope.semiHeight();
		// On z = x - 100, the ellipsoid's maximum x - z is the norm of the
		// scaled plane normal. This gives first contact independently of terrain sampling.
		double clearance = position.z() + 100 + envelope.upOffset();
		double expectedHeading = Math.asin(
				Math.sqrt((clearance * clearance - beam * beam - height * height) / (length * length - beam * beam)));
		assertEquals(expectedHeading, contact.heading(), 1e-7);
		assertEquals(position, contact.position());
		double sin = Math.sin(contact.heading()), cos = Math.cos(contact.heading());
		double supportLength = Math.sqrt(beam * beam * cos * cos + length * length * sin * sin + height * height);
		var supportingLocal = new Vec3(beam * beam * cos / supportLength,
				envelope.forwardOffset() + length * length * sin / supportLength,
				envelope.upOffset() - height * height / supportLength);
		assertTrue(
				Arrays.stream(contact.localPoints()).anyMatch(point -> point.subtract(supportingLocal).length() < 1e-8),
				"The response must use the supporting material point at the refined attitude");
		double penetration = 0;
		for (var local : contact.localPoints()) {
			double worldX = position.x() + cos * local.x() + sin * local.y();
			double worldZ = position.z() + local.z();
			penetration = Math.max(penetration, worldX - 100 - worldZ);
		}
		assertTrue(penetration > 0 && penetration < 1e-6,
				"The returned points must describe the penetrating side of first contact, within numerical precision");
	}

	@Test
	void pitchingHullStopsAtItsFirstPhysicalContactInsteadOfTheAttemptedAttitude() {
		var terrain = flat(-120);
		var position = new Vec3(0, 0, -100);
		double attemptedPitch = Math.PI / 3;
		assertClear(position, 0, 0, SUBMARINE, terrain);
		assertTrue(HullGeometry.terrainPenetration(position, 0, attemptedPitch, SUBMARINE, terrain) > 10);

		var contact = SweptHullTerrain.firstContact(position, position, 0, 0, 0, attemptedPitch, SUBMARINE, terrain);
		assertNotNull(contact);
		assertTrue(contact.fraction() > 0.3 && contact.fraction() < 0.8);
		assertEquals(attemptedPitch * contact.fraction(), contact.pitch(), 1e-12);
		assertTrue(HullGeometry.terrainPenetration(contact.position(), contact.heading(), contact.pitch(), SUBMARINE,
				terrain) < 1e-6, "The hull must reach the floor at its first contact attitude");
	}

	@Test
	void rotatingSternAppendageIsIncludedEvenWhereTheBodyClears() {
		var terrain = rowCrest(-30, -102.81, -120);
		var position = new Vec3(0, 4.6, -100);
		assertClear(position, -Math.PI / 2, 0, SUBMARINE, terrain);
		assertClear(position, Math.PI / 2, 0, SUBMARINE, terrain);

		var contact = SweptHullTerrain.firstContact(position, position, -Math.PI / 2, Math.PI / 2, 0, 0, SUBMARINE,
				terrain);
		assertNotNull(contact, "The lower stern fin must remain part of the rotating terrain collision shape");
		assertTrue(Arrays.stream(contact.localPoints())
				.anyMatch(p -> p.y() >= -35.4 - 1e-8 && p.y() <= -30.6 + 1e-8 && p.z() <= -2.82 + 1e-8));
	}

	@Test
	void wrappedHeadingFollowsTheShortTurnAndDoesNotSweepThroughARemoteRidge() {
		var terrain = columnCrest(20, -99, -120);
		var position = new Vec3(0, 0, -100);
		assertNull(
				SweptHullTerrain.firstContact(position, position, 2 * Math.PI - 0.01, 0.01, 0, 0, SUBMARINE, terrain),
				"Crossing heading zero is a small turn, rather than a full circle");
	}

	@Test
	void clearWaterInsideNavigationMarginDoesNotGenerateASweptContact() {
		var terrain = flat(-106);
		var start = new Vec3(0, 0, -100);
		var end = new Vec3(0, 100, -100);
		assertNull(SweptHullTerrain.firstContact(start, end, 0, 0, 0, 0, SUBMARINE, terrain));
		assertNull(SweptHullTerrain.firstContact(start, start, 0, 0, 0, 0, SUBMARINE, terrain));
	}

	@Test
	void preexistingPenetrationDoesNotPreventOutwardEscape() {
		var terrain = flat(-100);
		assertNull(SweptHullTerrain.firstContact(new Vec3(0, 0, -100), new Vec3(0, 0, -90), 0, 0, 0, 0, SUBMARINE,
				terrain), "An initial overlap is projected separately and must not stop motion at t=0");
	}

	@Test
	void descendingInclinedForebodyReachesBilinearCellInteriorBeforeTheEndpoint() {
		var terrain = new TerrainMap(new double[] {-129, -121, -87, -139}, 2, 2, 10, 10, 10);
		var start = new Vec3(4, 0, -90);
		var end = new Vec3(4, 0, -100);
		double heading = 0.7, pitch = -0.55;
		assertClear(start, heading, pitch, SUBMARINE, terrain);
		var contact = SweptHullTerrain.firstContact(start, end, heading, heading, pitch, pitch, SUBMARINE, terrain);
		assertNotNull(contact, "An interior body contact must also be visible to the swept pose queries");
		assertTrue(contact.fraction() > 0 && contact.fraction() < 1);
		var vertex = HullTerrainGeometry.worldPoint(new Vec3(-4.266573, 21.711071, -0.356215), contact.position(),
				contact.heading(), contact.pitch());
		assertTrue(vertex.z() >= terrain.elevationAt(vertex.x(), vertex.y()) - 1e-6);
	}

	@Test
	void physicsAndContactProjectionClearAnExistingInteriorForebodyOverlap() {
		var terrain = new TerrainMap(new double[] {-129, -121, -87, -139}, 2, 2, 10, 10, 10);
		var sub = new SubmarineEntity(SUBMARINE, 0, null, new Vec3(4, 0, -100), 0.7, Color.BLUE, 1000);
		sub.setPitch(-0.55);
		new SubmarinePhysics(MATCH).step(sub, 0.02, terrain, STILL_WATER, MATCH.battleArea());
		HullContactSolver.resolve(List.of(sub), STILL_WATER, terrain);

		var vertex = HullTerrainGeometry.worldPoint(new Vec3(-4.266573, 21.711071, -0.356215), sub.pose().position(),
				sub.heading(), sub.pitch());
		assertTrue(vertex.z() >= terrain.elevationAt(vertex.x(), vertex.y()) - 1e-6,
				"A reported clear pose must also clear the independently referenced rendered forebody");
		assertTrue(sub.z() > -97.23, "Static contact must remove the original 2.78 m mesh overlap");
	}

	@Test
	void physicsRespondsToTheFineCrestBeforeContinuingTheUnspentTick() {
		var terrain = narrowCrest(0.1, -104.99, -110, 0.1);
		for (double dt : new double[] {0.02, 0.1}) {
			var sub = new SubmarineEntity(SUBMARINE, 0, null, new Vec3(0, 0.02, -100), 0, Color.BLUE, 1000);
			var clearWater = new SubmarineEntity(SUBMARINE, 0, null, new Vec3(0, 0.02, -100), 0, Color.BLUE, 1000);
			sub.setSpeed(15);
			clearWater.setSpeed(15);
			new SubmarinePhysics(MATCH).step(sub, dt, terrain, STILL_WATER, MATCH.battleArea());
			new SubmarinePhysics(MATCH).step(clearWater, dt, flat(-110), STILL_WATER, MATCH.battleArea());

			assertTrue(sub.y() < clearWater.y() - 1e-4,
					"The swept impact must reduce forward progress before the remaining tick is integrated");
			assertTrue(sub.velocity().linear().length() < clearWater.velocity().linear().length(),
					"Crossing the crest after deflection must not preserve the unmodified incoming velocity");
			assertTrue(sub.z() >= clearWater.z() - 1e-6,
					"The terrain response may stop or deflect the hull, but must not drive it down through the crest");
			assertTrue(sub.hp() < 1000, "Actual inward motion at the swept contact must cause impact damage");
			assertTrue(HullGeometry.terrainPenetration(new Vec3(sub.x(), sub.y(), sub.z()), sub.heading(), sub.pitch(),
					SUBMARINE, terrain) < 1e-6, "The projected response pose must clear the physical terrain");
		}
	}

	@Test
	void groundedSinkingWreckRetainsTangentialCoastingAndCurrentDrift() {
		var terrain = flat(-100);
		var wreck = new SubmarineEntity(SUBMARINE, 0, null, new Vec3(0, 0, -95), 0, Color.BLUE, 1000);
		wreck.setHp(0);
		wreck.setBallast(0);
		wreck.setActualBallast(0);
		wreck.setSpeed(2);
		var current = new CurrentField(List.of(new CurrentField.CurrentBand(-1000, 0, new Vec2(1, 0))));
		var physics = new SubmarinePhysics(MATCH);
		for (int tick = 0; tick < 50; tick++) {
			physics.step(wreck, 0.02, terrain, current, MATCH.battleArea());
			assertEquals(-95, wreck.z(), 1e-6, "The flooded wreck's keel must remain supported by the floor");
			assertEquals(0, wreck.verticalSpeed(), 1e-6, "A supported wreck must settle without bouncing");
			assertTrue(HullGeometry.terrainPenetration(wreck.pose().position(), wreck.heading(), wreck.pitch(),
					SUBMARINE, terrain) < 1e-6);
		}
		assertTrue(wreck.x() > 0.95, "Seabed support must not consume the current's horizontal drift");
		assertTrue(wreck.y() > 1.5, "First contact at the start of a tick must not pin a coasting wreck in place");
		assertEquals(0, wreck.hp());
	}

	@Test
	void groundedSinkingWreckFollowsNonflatSupportWhileRetainingTangentialDrift() {
		var terrain = slopingFloor(0.05);
		var wreck = new SubmarineEntity(SUBMARINE, 0, null, new Vec3(0, 0, -95), 0, Color.BLUE, 1000);
		wreck.setHp(0);
		wreck.setBallast(0);
		wreck.setActualBallast(0);
		wreck.setSpeed(2);
		var current = new CurrentField(List.of(new CurrentField.CurrentBand(-1000, 0, new Vec2(1, 0))));
		var physics = new SubmarinePhysics(MATCH);
		for (int tick = 0; tick < 50; tick++) {
			physics.step(wreck, 0.02, terrain, current, MATCH.battleArea());
			var keel = HullTerrainGeometry.worldPoint(new Vec3(0, 0, -5), wreck.pose().position(), wreck.heading(),
					wreck.pitch());
			assertEquals(terrain.elevationAt(keel.x(), keel.y()), keel.z(), 1e-6,
					"Repeated support must follow the slope rather than retain a previous tick's contact pose");
			assertTrue(HullGeometry.terrainPenetration(wreck.pose().position(), wreck.heading(), wreck.pitch(),
					SUBMARINE, terrain) < 1e-6, "A supported wreck must remain physically clear each tick");
		}
		assertTrue(wreck.x() > 0.8, "Support must preserve drift along the sloping floor");
		assertTrue(wreck.y() > 1.5, "Support must preserve tangential coasting");
		assertEquals(0, wreck.hp());
	}

	@Test
	void surfaceShipUsesItsOwnDraftForACrestCrossedBetweenTicks() {
		var terrain = narrowCrest(1.1, -9.499999, -20, 0.01);
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, null, Vec3.ZERO, 0, Color.BLUE, 1000);
		ship.setSpeed(10);
		assertClear(Vec3.ZERO, 0, 0, ship.vehicleConfig(), terrain);
		assertClear(new Vec3(0, 0.2, 0), 0, 0, ship.vehicleConfig(), terrain);
		new SubmarinePhysics(MATCH).step(ship, 0.02, terrain, STILL_WATER, MATCH.battleArea());

		assertTrue(ship.y() < 0.101,
				"A ship's swept keel must hit the crest before returning to a clear endpoint pose");
		assertEquals(0, ship.z(), 1e-6);
		assertTrue(ship.hp() < 1000);
	}

	private static void assertClear(
		Vec3 position, double heading, double pitch, VehicleConfig config, TerrainMap terrain) {
		assertEquals(0, HullGeometry.terrainPenetration(position, heading, pitch, config, terrain), 1e-10);
	}

	private static TerrainMap narrowCrest(double crestY, double crestHeight, double background, double cell) {
		int cols = 5, rows = 21;
		double[] elevations = new double[cols * rows];
		Arrays.fill(elevations, background);
		for (int col = 0; col < cols; col++) {
			elevations[10 * cols + col] = crestHeight;
		}
		return new TerrainMap(elevations, cols, rows, -2 * cell, crestY - 10 * cell, cell);
	}

	private static TerrainMap rowCrest(double crestY, double crestHeight, double background) {
		double[] elevations = new double[441];
		Arrays.fill(elevations, background);
		for (int row = 0; row < 21; row++) {
			if (-100 + row * 10 == crestY) {
				for (int col = 0; col < 21; col++) {
					elevations[row * 21 + col] = crestHeight;
				}
			}
		}
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}

	private static TerrainMap columnCrest(double crestX, double crestHeight, double background) {
		double[] elevations = new double[441];
		Arrays.fill(elevations, background);
		for (int row = 0; row < 21; row++) {
			for (int col = 0; col < 21; col++) {
				if (-100 + col * 10 == crestX) {
					elevations[row * 21 + col] = crestHeight;
				}
			}
		}
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}

	private static TerrainMap flat(double height) {
		double[] elevations = new double[441];
		Arrays.fill(elevations, height);
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}

	private static TerrainMap slopingFloor(double slope) {
		double[] elevations = new double[441];
		for (int row = 0; row < 21; row++) {
			for (int col = 0; col < 21; col++) {
				elevations[row * 21 + col] = -100 + slope * (-100 + col * 10);
			}
		}
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}
}
