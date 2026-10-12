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

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertTrue;

class EllipsoidTerrainCoverageTest {

	private static final int SCENES_PER_HULL = 80;
	private static final int LATITUDE_STEPS = 64;
	private static final int LONGITUDE_STEPS = 128;
	private static final double COVERAGE_TOLERANCE = 0.001;

	@ParameterizedTest(name = "dense terrain coverage, torpedo={0}")
	@ValueSource(booleans = {false, true})
	void contactCandidatesCoverIndependentlySampledHullPenetration(boolean torpedo) {
		var random = new Random(124);
		var config = torpedo ? VehicleConfig.torpedo() : VehicleConfig.submarine();
		var envelope = torpedo
				? new HullGeometry.Envelope(config.hullHalfLength(), config.hullHalfBeam(), config.hullHalfBeam(), 0, 0)
				: HullGeometry.envelope(config);
		// Preserve the original oracle's seeded submarine-then-torpedo sequence, including
		// its ridge and bilinear-cell counterexamples for both hulls.
		if (torpedo) {
			for (int scene = 0; scene < SCENES_PER_HULL; scene++) {
				nextScene(random);
			}
		}
		for (int index = 0; index < SCENES_PER_HULL; index++) {
			var scene = nextScene(random);
			double reported = 0;
			for (var local : HullTerrainGeometry.ellipsoidContactPoints(scene.position(), scene.heading(),
					scene.pitch(), envelope, scene.terrain())) {
				double right = local.x() / envelope.semiBeam();
				double forward = (local.y() - envelope.forwardOffset()) / envelope.semiLength();
				double up = (local.z() - envelope.upOffset()) / envelope.semiHeight();
				assertTrue(right * right + forward * forward + up * up <= 1 + 1e-8,
						"Terrain candidates must stay inside the physical ellipsoid");
				reported = Math.max(reported, penetration(worldPoint(local, scene), scene.terrain()));
			}
			double sampled = sampledPenetration(scene, envelope);
			assertTrue(reported + COVERAGE_TOLERANCE >= sampled,
					String.format("%s scene %d: dense hull penetration %.6f m exceeds contact candidates %.6f m",
							torpedo ? "torpedo" : "submarine", index, sampled, reported));
		}
	}

	private record Scene(TerrainMap terrain, Vec3 position, double heading, double pitch) {
	}

	private static Scene nextScene(Random random) {
		double[] depths = new double[4 * 4];
		for (int index = 0; index < depths.length; index++) {
			depths[index] = -100 + random.nextGaussian() * 8;
		}
		var terrain = new TerrainMap(depths, 4, 4, -15, -15, 10);
		var position = new Vec3(random.nextDouble() * 20 - 10, random.nextDouble() * 20 - 10,
				-100 + random.nextDouble() * 15);
		return new Scene(terrain, position, random.nextDouble() * Math.PI * 2, random.nextDouble() * 2 - 1);
	}

	private static double sampledPenetration(Scene scene, HullGeometry.Envelope envelope) {
		double maximum = 0;
		// Sample material points on the complete body, independent of terrain normals,
		// lattice seeds, cell solvers, and the geometry code's contact selection.
		for (int latitude = 0; latitude <= LATITUDE_STEPS; latitude++) {
			double theta = Math.PI * latitude / LATITUDE_STEPS;
			for (int longitude = 0; longitude < LONGITUDE_STEPS; longitude++) {
				double phi = Math.PI * 2 * longitude / LONGITUDE_STEPS;
				var local = new Vec3(envelope.semiBeam() * Math.sin(theta) * Math.cos(phi),
						envelope.forwardOffset() + envelope.semiLength() * Math.cos(theta),
						envelope.upOffset() + envelope.semiHeight() * Math.sin(theta) * Math.sin(phi));
				maximum = Math.max(maximum, penetration(worldPoint(local, scene), scene.terrain()));
			}
		}
		return maximum;
	}

	private static Vec3 worldPoint(Vec3 local, Scene scene) {
		double sinHeading = Math.sin(scene.heading()), cosHeading = Math.cos(scene.heading());
		double sinPitch = Math.sin(scene.pitch()), cosPitch = Math.cos(scene.pitch());
		return new Vec3(
				scene.position().x() + local.x() * cosHeading + local.y() * sinHeading * cosPitch
						- local.z() * sinHeading * sinPitch,
				scene.position().y() - local.x() * sinHeading + local.y() * cosHeading * cosPitch
						- local.z() * cosHeading * sinPitch,
				scene.position().z() + local.y() * sinPitch + local.z() * cosPitch);
	}

	private static double penetration(Vec3 point, TerrainMap terrain) {
		return terrain.elevationAt(point.x(), point.y()) - point.z();
	}
}
