/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.TerrainMap;

import static org.junit.jupiter.api.Assertions.*;

class IslandBeachShaperTest {

	private static final int COLS = 101;
	private static final int ROWS = 61;
	private static final double CELL = 10;

	@Test
	void preservesWaterAndEveryMixedInterpolationCell() {
		double[] original = new double[COLS * ROWS];
		for (int row = 0; row < ROWS; row++) {
			for (int col = 0; col < COLS; col++) {
				double coast = 20.5 + 2.4 * Math.sin(row * .19);
				original[row * COLS + col] = (col - coast) * 2;
			}
		}
		double[] shaped = shape(original, 42);
		var before = new TerrainMap(original, COLS, ROWS, 0, 0, CELL);
		var after = new TerrainMap(shaped, COLS, ROWS, 0, 0, CELL);
		int changed = 0;
		int mixed = 0;
		for (int index = 0; index < original.length; index++) {
			assertEquals(original[index] > 0, shaped[index] > 0, "Land mask at " + index);
			assertTrue(Double.isFinite(shaped[index]));
			assertTrue(shaped[index] <= original[index], "Shaping only lowers land");
			if (original[index] <= 0)
				assertEquals(original[index], shaped[index]);
			if (original[index] != shaped[index])
				changed++;
		}
		for (int row = 0; row < ROWS - 1; row++) {
			for (int col = 0; col < COLS - 1; col++) {
				boolean hasWater = original[row * COLS + col] <= 0 || original[row * COLS + col + 1] <= 0
						|| original[(row + 1) * COLS + col] <= 0 || original[(row + 1) * COLS + col + 1] <= 0;
				if (!hasWater)
					continue;
				mixed++;
				for (double y : new double[] {0, .25, .5, .75, 1}) {
					for (double x : new double[] {0, .25, .5, .75, 1}) {
						assertEquals(before.elevationAt((col + x) * CELL, (row + y) * CELL),
								after.elevationAt((col + x) * CELL, (row + y) * CELL),
								"Water-bearing interpolation cell " + col + "," + row);
					}
				}
			}
		}
		assertTrue(mixed > 0);
		assertTrue(changed > 0, "Preservation must also exercise real island shaping");
	}

	@Test
	void formsARisingBeachAndRestoresTheInlandHills() {
		double[] original = planarCoast(.2);
		double[] shaped = shape(original, 42);
		int row = ROWS / 2;
		int coastCol = 21;
		assertEquals(original[row * COLS + coastCol], shaped[row * COLS + coastCol]);
		for (int col = coastCol + 1; col <= coastCol + 25; col++) {
			assertTrue(shaped[row * COLS + col] > shaped[row * COLS + col - 1],
					"Beach should rise inland without a berm or constant-height plateau");
		}
		double beachElevation = shaped[row * COLS + coastCol + 8];
		assertTrue(beachElevation > 3 && beachElevation < 8, "Low beach shelf: " + beachElevation);
		double beachSlope = (shaped[row * COLS + coastCol + 7] - shaped[row * COLS + coastCol + 4]) / (3 * CELL);
		assertTrue(beachSlope > .02 && beachSlope < .1, "Gentle, nonflat shelf: " + beachSlope);
		assertTrue(beachElevation < original[row * COLS + coastCol + 8] * .5);
		assertEquals(original[row * COLS + coastCol + 28], shaped[row * COLS + coastCol + 28],
				"Island relief beyond the coastal transition remains untouched");
	}

	@Test
	void leavesSteepHeadlandsAndMapEdgesUntouched() {
		double[] steep = planarCoast(.45);
		assertArrayEquals(steep, shape(steep, 42), "Steep coasts retain their rocky relief");
		double[] gentle = planarCoast(.2);
		double[] shaped = shape(gentle, 42);
		for (int col = 0; col < COLS; col++) {
			assertEquals(gentle[col], shaped[col]);
			assertEquals(gentle[(ROWS - 1) * COLS + col], shaped[(ROWS - 1) * COLS + col]);
		}
		for (int row = 0; row < ROWS; row++) {
			assertEquals(gentle[row * COLS], shaped[row * COLS]);
			assertEquals(gentle[row * COLS + COLS - 1], shaped[row * COLS + COLS - 1]);
		}
	}

	@Test
	void coastCharacterIsDeterministicAndVariesWithTheWorldSeed() {
		double[] original = planarCoast(.2);
		double[] first = shape(original, 42);
		assertArrayEquals(first, shape(original, 42));
		double[] second = shape(original, 123);
		boolean changed = false;
		for (int col = 25; col < 45; col++) {
			if (first[(ROWS / 2) * COLS + col] != second[(ROWS / 2) * COLS + col]) {
				changed = true;
				break;
			}
		}
		assertTrue(changed, "Seed affects beach width and inland rise");
	}

	@Test
	void smoothsTheCarveAcrossCoastSuitabilityChangesWithoutRemovingLowLand() {
		double[] original = planarCoast(.2);
		for (int row = 0; row < ROWS; row++) {
			for (int col = 0; col < COLS; col++) {
				int index = row * COLS + col;
				// Landward hills are continuous along this coast. A steep underwater approach in
				// its northern half switches beach suitability without an existing inland cliff.
				if (row >= ROWS / 2 && original[index] <= 0)
					original[index] *= 5;
				if (col > 24)
					original[index] += (col - 24) * 7;
			}
		}
		int lowLand = (ROWS / 2 + 2) * COLS + 29;
		original[lowLand] = .25;
		double[] shaped = shape(original, 42);
		assertEquals(original[lowLand], shaped[lowLand], "Borrowed carving cannot remove low land");
		double largestAddedLateralSlope = 0;
		for (int row = ROWS / 2 - 10; row < ROWS / 2 + 10; row++) {
			for (int col = 25; col < 36; col++) {
				int index = row * COLS + col;
				int next = index + COLS;
				double oldSlope = Math.abs(original[index] - original[next]) / CELL;
				double newSlope = Math.abs(shaped[index] - shaped[next]) / CELL;
				largestAddedLateralSlope = Math.max(largestAddedLateralSlope, newSlope - oldSlope);
			}
		}
		assertTrue(largestAddedLateralSlope > 0, "Exercise a transition between shelves and headlands");
		assertTrue(largestAddedLateralSlope < .751,
				"Avoid a deep, one-cell wall at a nearest-anchor suitability boundary: " + largestAddedLateralSlope);
		for (int row = ROWS / 2; row < ROWS / 2 + 3; row++) {
			int index = row * COLS + 31;
			assertTrue(shaped[index] < original[index], "The transition extends into the adjacent headland");
		}
		for (int index = 0; index < original.length; index++) {
			assertEquals(original[index] > 0, shaped[index] > 0);
			assertTrue(Double.isFinite(shaped[index]));
			assertTrue(shaped[index] <= original[index]);
			if (original[index] <= 0)
				assertEquals(original[index], shaped[index]);
		}
	}

	@Test
	void joinsLowBeachShelvesToHighCoastalAnchorsWithoutCuttingAQuarryWall() {
		double[] original = planarCoast(.2);
		for (int row = 0; row < ROWS; row++) {
			for (int col = 21; col < COLS; col++) {
				int index = row * COLS + col;
				if (col > 24)
					original[index] += (col - 24) * 7;
				// A rocky coast replaces the low anchor in the northern half. The inland hill
				// stays continuous, so a steep lateral cut there would be introduced by shaping.
				if (row >= ROWS / 2)
					original[index] += 45 * Math.max(0, 1 - (col - 21) / 6.0);
			}
		}
		double[] shaped = shape(original, 42);
		double largestAddedSlope = 0;
		for (int row = 1; row < ROWS - 1; row++) {
			for (int col = 1; col < COLS - 1; col++) {
				int index = row * COLS + col;
				for (int offset : new int[] {1, COLS, COLS + 1, COLS - 1}) {
					double distance = CELL * (offset == 1 || offset == COLS ? 1 : Math.sqrt(2));
					double oldSlope = Math.abs(original[index] - original[index + offset]) / distance;
					double newSlope = Math.abs(shaped[index] - shaped[index + offset]) / distance;
					largestAddedSlope = Math.max(largestAddedSlope, newSlope - oldSlope);
				}
			}
		}
		assertTrue(largestAddedSlope > .1, "Exercise the actual transition between beach and headland");
		assertTrue(largestAddedSlope < .751, "Bound the added grade across anchor transitions: " + largestAddedSlope);
		assertTrue(shaped[10 * COLS + 27] < 8, "Retain the low beach away from the headland");
		assertEquals(original[35 * COLS + 27], shaped[35 * COLS + 27], "Retain the neighboring rocky hill");
		for (int index = 0; index < original.length; index++) {
			assertEquals(original[index] > 0, shaped[index] > 0);
			assertTrue(shaped[index] <= original[index]);
		}
	}

	private static double[] planarCoast(double slope) {
		double[] elevations = new double[COLS * ROWS];
		for (int row = 0; row < ROWS; row++) {
			for (int col = 0; col < COLS; col++) {
				elevations[row * COLS + col] = (col - 20.5) * CELL * slope;
			}
		}
		return elevations;
	}

	private static double[] shape(double[] original, long seed) {
		double[] shaped = original.clone();
		IslandBeachShaper.shape(shaped, COLS, ROWS, 0, 0, CELL, seed);
		return shaped;
	}
}
