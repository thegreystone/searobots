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

import jdk.jfr.Recording;
import jdk.jfr.consumer.RecordingFile;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec2;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

/** Regression tests for living hulls grounded on shallow shoals at the water surface. */
class GroundedSubmarineTerrainTest {

	private static final double DT = 0.02;
	private static final MatchConfig CONFIG = MatchConfig.withDefaults(42);
	private static final CurrentField GENTLE_CURRENT = new CurrentField(
			List.of(new CurrentField.CurrentBand(-1000, 0, new Vec2(0.3, -0.2))));

	@Test
	void livingSubmarineGroundedOnFlatShoalAtSurfaceSlidesWithoutFreezingOrSpuriousDamage() {
		var shoal = flat(-2.0);
		var physics = new SubmarinePhysics(CONFIG);
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(0, 0, 0), 0, Color.RED, 1000);

		for (int tick = 0; tick < 500; tick++) {
			sub.setThrottle(0.3);
			sub.setRudder(0.5);
			sub.setSternPlanes(-0.45);
			sub.setBallast(0.18);
			physics.step(sub, DT, shoal, GENTLE_CURRENT, CONFIG.battleArea(), tick);
			assertTrue(HullGeometry.terrainPenetration(sub.pose().position(), sub.heading(), sub.pitch(),
					sub.vehicleConfig(), shoal) < 1e-6, "Hull must remain clear of the shoal on tick " + tick);
		}

		assertEquals(1000, sub.hp(), "Sustained surface-shoal scraping with gentle controls must not invent impacts");
		assertTrue(Math.hypot(sub.x(), sub.y()) > 5.0,
				"A pitched hull on a flat shoal must retain horizontal motion rather than freezing in place");
	}

	@Test
	void livingSubmarineOnUnevenShoalConsumesTickAndReturnsToWaterlineInDeepWater(@TempDir
	Path tempDir) throws IOException {
		var shoal = unevenShoal();
		var physics = new SubmarinePhysics(CONFIG);
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, null, new Vec3(-35, 0, 0), Math.PI / 2, Color.GREEN,
				1000);
		Path jfrFile = tempDir.resolve("grounded-sub.jfr");

		boolean sawAboveWaterLift = false;
		int ticks = 2500;
		try (var recording = new Recording()) {
			recording.enable(SimulationJfrEvents.TERRAIN_EVENT_NAME).withThreshold(Duration.ZERO).withoutStackTrace();
			recording.start();
			for (int tick = 0; tick < ticks; tick++) {
				sub.setThrottle(0.5);
				sub.setRudder(0.4);
				sub.setSternPlanes(0.5);
				sub.setBallast(0.95);
				physics.step(sub, DT, shoal, GENTLE_CURRENT, CONFIG.battleArea(), tick);
				assertTrue(
						HullGeometry.terrainPenetration(sub.pose().position(), sub.heading(), sub.pitch(),
								sub.vehicleConfig(), shoal) < 1e-6,
						"Hull must remain physically clear on tick " + tick);
				if (sub.z() > 0.1) {
					sawAboveWaterLift = true;
				}
			}
			recording.stop();
			recording.dump(jfrFile);
		}

		assertTrue(sawAboveWaterLift, "Fixture must exercise an above-water terrain lift on the shoal");
		assertTrue(shoal.elevationAt(sub.x(), sub.y()) < -15.0,
				"Submarine should drive off the shoal into deeper water, elev=" + shoal.elevationAt(sub.x(), sub.y()));
		assertTrue(sub.z() <= 1e-6,
				"Once clear of the shoal in deep water, the submarine must not remain lifted above z=0, z=" + sub.z());

		var events = RecordingFile.readAllEvents(jfrFile).stream()
				.filter(e -> e.getEventType().getName().equals(SimulationJfrEvents.TERRAIN_EVENT_NAME)).toList();
		assertEquals(ticks, events.size());
		long unspentTicks = events.stream().filter(e -> e.getDouble("remainingSeconds") > 1e-9).count();
		assertTrue(unspentTicks <= 5,
				"Grounded surface-shoal resolution must consume the tick budget rather than stalling, unspentTicks="
						+ unspentTicks);
	}

	@Test
	void surfaceShipMovingOffShoalRestoresWaterlineWithoutDamage() {
		// Shallow -5 m shelf for y <= 20, deep -50 m water for y >= 120
		int size = 41;
		double[] elevations = new double[size * size];
		for (int row = 0; row < size; row++) {
			double y = -200 + row * 10.0;
			double elev = y <= 20 ? -5.0 : -50.0;
			Arrays.fill(elevations, row * size, (row + 1) * size, elev);
		}
		var terrain = new TerrainMap(elevations, size, size, -200, -200, 10.0);
		var physics = new SubmarinePhysics(CONFIG);
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, null, new Vec3(0, 0, 0), 0, Color.BLUE, 1000);

		physics.step(ship, DT, terrain, new CurrentField(List.of()), CONFIG.battleArea(), 0);
		assertTrue(ship.z() > 0, "Grounded surface ship starts lifted above its waterline");
		assertEquals(1000, ship.hp());

		// Move cleanly into deep water where the entire 180 m hull clears -50 m
		ship.setY(150);
		ship.setSpeed(5);
		physics.step(ship, DT, terrain, new CurrentField(List.of()), CONFIG.battleArea(), 1);
		assertEquals(0.0, ship.z(), 1e-9, "Surface ship must return to z=0 once clear of the shoal");
		assertEquals(1000, ship.hp(), "Restoring the waterline must not cause impact damage");
	}

	private static TerrainMap flat(double elevation) {
		int size = 41;
		double[] elevations = new double[size * size];
		Arrays.fill(elevations, elevation);
		return new TerrainMap(elevations, size, size, -200, -200, 10.0);
	}

	private static TerrainMap unevenShoal() {
		int size = 41;
		double cell = 10.0;
		double origin = -200.0;
		double[] elev = new double[size * size];
		var rng = new Random(42);
		for (int r = 0; r < size; r++) {
			for (int c = 0; c < size; c++) {
				double x = origin + c * cell;
				double y = origin + r * cell;
				double dist = Math.hypot(x, y);
				elev[r * size + c] = 10.0 - 0.3 * dist + (rng.nextDouble() - 0.5) * 1.5;
			}
		}
		return new TerrainMap(elev, size, size, origin, origin, cell);
	}
}
