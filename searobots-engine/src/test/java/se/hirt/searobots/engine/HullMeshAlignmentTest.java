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
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HullMeshAlignmentTest {

	@Test
	void forebodyVertexOutsideTheFormerHullIsContained() {
		// Actual Body vertex in model coordinates: (0, -27.285818, 3.227062).
		// The former aft=2 m, up=0 hull left this upper forebody point outside its collision surface.
		assertEquals(0, HullGeometry.distanceToHull(0, 27.285818, 3.227062, 0, 0, 0, 0, 0), 1e-6);
	}

	@Test
	void collisionHullContainsEveryReferencedBodyVertexInTheCurrentModel() throws IOException {
		Path relative = Path.of("searobots-viewer", "src", "main", "resources", "models", "submarine-hybrid.obj");
		// Surefire normally runs from the engine module; direct JUnit runs can use the repository root.
		Path model = Files.isRegularFile(relative) ? relative : Path.of("..").resolve(relative);
		assertTrue(Files.isRegularFile(model), "The actual submarine model must be present: " + model);

		var vertices = new ArrayList<Vec3>();
		var hullIndices = new HashSet<Integer>();
		Set<String> hullGroups = Set.of("Body", "HullPlain");
		var seenGroups = new HashSet<String>();
		String group = "";
		try (var reader = Files.newBufferedReader(model)) {
			String line;
			while ((line = reader.readLine()) != null) {
				if (line.startsWith("v ")) {
					String[] fields = line.split("\\s+");
					vertices.add(new Vec3(Double.parseDouble(fields[1]), Double.parseDouble(fields[2]),
							Double.parseDouble(fields[3])));
				} else if (line.startsWith("g ")) {
					group = line.substring(2).trim();
					if (hullGroups.contains(group))
						seenGroups.add(group);
				} else if (line.startsWith("f ") && hullGroups.contains(group)) {
					for (String field : line.substring(2).trim().split("\\s+")) {
						int slash = field.indexOf('/');
						int index = Integer.parseInt(slash < 0 ? field : field.substring(0, slash));
						hullIndices.add(index > 0 ? index - 1 : vertices.size() + index);
					}
				}
			}
		}
		assertEquals(hullGroups, seenGroups, "Check both the tiled body and its untiled bow/keel surfaces");
		assertTrue(hullIndices.size() >= 3000, "Check the full mesh, rather than a sparse set of sample points");

		double maximumGap = 0;
		Vec3 worstPoint = Vec3.ZERO;
		for (int index : hullIndices) {
			Vec3 point = vertices.get(index);
			// OBJ: X port, Y aft, Z up. Engine at heading=0, pitch=0: X right, Y forward, Z up.
			double gap = HullGeometry.distanceToHull(-point.x(), -point.y(), point.z(), 0, 0, 0, 0, 0);
			assertTrue(Double.isFinite(gap), "The hull distance must be finite at OBJ " + point);
			if (gap > maximumGap) {
				maximumGap = gap;
				worstPoint = point;
			}
		}
		assertTrue(maximumGap <= 1e-6,
				"The collision hull must contain the rendered body; gap=" + maximumGap + " m at OBJ " + worstPoint);
	}

	@Test
	void surfaceShipHullContainsTheActualSubmergedMainHullAndBulb() throws IOException {
		Path relative = Path.of("searobots-viewer", "src", "main", "resources", "models", "surface-ship.obj");
		Path model = Files.isRegularFile(relative) ? relative : Path.of("..").resolve(relative);
		assertTrue(Files.isRegularFile(model), "The actual surface ship model must be present: " + model);
		var vertices = new ArrayList<Vec3>();
		var hullIndices = new HashSet<Integer>();
		Set<String> hullGroups = Set.of("HullBelow", "HullBoot", "TransomBoot", "Bulb");
		var seenGroups = new HashSet<String>();
		String group = "";
		try (var reader = Files.newBufferedReader(model)) {
			String line;
			while ((line = reader.readLine()) != null) {
				if (line.startsWith("v ")) {
					String[] fields = line.split("\\s+");
					vertices.add(new Vec3(Double.parseDouble(fields[1]), Double.parseDouble(fields[2]),
							Double.parseDouble(fields[3])));
				} else if (line.startsWith("g ")) {
					group = line.substring(2).trim();
					if (hullGroups.contains(group))
						seenGroups.add(group);
				} else if (line.startsWith("f ") && hullGroups.contains(group)) {
					for (String field : line.substring(2).trim().split("\\s+")) {
						int slash = field.indexOf('/');
						int index = Integer.parseInt(slash < 0 ? field : field.substring(0, slash));
						hullIndices.add(index > 0 ? index - 1 : vertices.size() + index);
					}
				}
			}
		}
		assertEquals(hullGroups, seenGroups);
		var ship = new SubmarineEntity(VehicleConfig.surfaceShip(), 0, (input, output) -> {
		}, Vec3.ZERO, 0, Color.BLUE, 1000);
		int submergedCount = 0;
		for (int index : hullIndices) {
			var point = vertices.get(index);
			if (point.z() > 0)
				continue;
			submergedCount++;
			// Ship OBJ: X starboard, Y aft, Z up; engine heading zero points north.
			double gap = HullGeometry.distanceToHull(point.x(), -point.y(), point.z(), ship);
			assertEquals(0, gap, 1e-6, "The submerged ship hull must contain OBJ " + point);
		}
		assertTrue(submergedCount >= 1300, "Check the main underwater mesh, including the bulb and transom");
	}
}
