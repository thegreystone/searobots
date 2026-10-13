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
package se.hirt.searobots.viewer;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.scene.Node;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;
import se.hirt.searobots.api.Pose;
import se.hirt.searobots.api.Velocity;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Regressions for readable tactical coverage during continuous action. */
class CinematicDirectorTest {

	private static final String OVERVIEW = "Tactical View";

	@ParameterizedTest
	@ValueSource(ints = {30, 60, 144})
	void continuousLaunchesPreserveReadableBattleOverviews(int fps) throws Exception {
		Scene scene = new Scene(1920, 1080);
		float dt = 1f / fps;
		double nextLaunch = 0;
		double lastReadable = 0;
		double longestAbsence = 0;
		double readableRun = 0;
		List<Double> completedReadableRuns = new ArrayList<>();
		boolean previousOverview = false;

		for (int frame = 0; frame < 125 * fps; frame++) {
			double now = frame / (double) fps;
			if (now >= nextLaunch) {
				scene.launch(new Vector3f(-2000, -100, 1500), now);
				nextLaunch += .5;
			}
			scene.expireTorpedoes(now - 12);
			scene.step(dt);
			boolean overview = OVERVIEW.equals(scene.director.currentShotLabel());
			if (overview && scene.director.waterOpacity() <= .05f && hullFits(scene.camera, scene.subNodes.get(1))
					&& hullFits(scene.camera, scene.subNodes.get(2))) {
				lastReadable = now;
				readableRun += dt;
			} else {
				longestAbsence = Math.max(longestAbsence, now - lastReadable);
			}
			if (previousOverview && !overview) {
				completedReadableRuns.add(readableRun);
				readableRun = 0;
			}
			previousOverview = overview;
		}

		assertTrue(completedReadableRuns.size() >= 3,
				"Continuous launches must still allow at least three complete battle overviews in 125 seconds: "
						+ completedReadableRuns);
		assertTrue(longestAbsence <= 36,
				"Both submarines disappeared from readable overview coverage for " + longestAbsence + " seconds");
		for (double duration : completedReadableRuns)
			assertTrue(duration >= 4.4,
					"Ordinary launches interrupted the readable overview after " + duration + " seconds");
	}

	@ParameterizedTest
	@CsvSource({"1920,1080", "1440,1080", "1080,1920"})
	void battleOverviewFramesBothSubmarinesAcrossViewportShapes(int width, int height) throws Exception {
		for (Vector3f spread : List.of(new Vector3f(6000, 0, 0), new Vector3f(0, 0, 6000),
				new Vector3f(1800, 0, 1800))) {
			Scene scene = new Scene(width, height);
			scene.subNodes.get(1).setLocalTranslation(spread.mult(-.5f).addLocal(0, -25, 0));
			scene.subNodes.get(2).setLocalTranslation(spread.mult(.5f).addLocal(0, -500, 0));
			// A weapon leaving the battle must not drag the submarines out of the overview.
			scene.launch(new Vector3f(3500, -150, -7000), 0);
			scene.waitForOverview();
			boolean readable = false;
			for (int frame = 0; frame < 8 * 60 && OVERVIEW.equals(scene.director.currentShotLabel()); frame++) {
				scene.step(1f / 60);
				if (frame < 3 * 60 || scene.director.waterOpacity() > .05f)
					continue;
				readable = true;
				assertHullFits(scene.camera, scene.subNodes.get(1));
				assertHullFits(scene.camera, scene.subNodes.get(2));
				Vector3f origin = scene.camera.getScreenCoordinates(scene.look);
				Vector3f north = scene.camera.getScreenCoordinates(scene.look.add(0, 0, -10));
				Vector3f east = scene.camera.getScreenCoordinates(scene.look.add(10, 0, 0));
				assertTrue(north.y > origin.y && east.x > origin.x,
						"The battle overview must keep north up and east right");
				assertEquals(origin.x, north.x, .1f, "The battle overview must preserve its orientation");
			}
			assertTrue(readable, "The overview never became readable");
		}
	}

	@Test
	void ordinaryTerminalApproachPreservesProtectedOverview() throws Exception {
		Scene scene = new Scene(1920, 1080);
		scene.waitForOverview();
		scene.launch(scene.subNodes.get(2).getLocalTranslation().add(300, 0, 0), 0);
		for (int frame = 0; frame < 4 * 60; frame++) {
			scene.step(1f / 60);
			assertEquals(OVERVIEW, scene.director.currentShotLabel(),
					"A distant terminal approach must allow the battle overview to become readable");
		}
	}

	@Test
	void imminentTorpedoApproachInterruptsProtectedOverview() throws Exception {
		Scene scene = new Scene(1920, 1080);
		scene.waitForOverview();
		scene.launch(scene.subNodes.get(2).getLocalTranslation().add(100, 0, 0), 0);
		for (int frame = 0; frame < 10; frame++)
			scene.step(1f / 60);
		assertEquals("Terminal Approach", scene.director.currentShotLabel(),
				"An imminent impact must interrupt the overview's normal minimum hold");
	}

	@Test
	void detonationInterruptsProtectedOverview() throws Exception {
		Scene scene = new Scene(1920, 1080);
		scene.waitForOverview();
		scene.launch(scene.subNodes.get(2).getLocalTranslation(), 0);
		scene.torpedoDetonated = true;
		scene.step(1f / 60);
		assertEquals("Explosion", scene.director.currentShotLabel(), "A hit must receive immediate coverage");
	}

	@Test
	void submarineDeathInterruptsProtectedOverview() throws Exception {
		Scene scene = new Scene(1920, 1080);
		scene.waitForOverview();
		scene.secondSubHp = 0;
		for (int frame = 0; frame < 10; frame++)
			scene.step(1f / 60);
		assertEquals("Sub Sinking", scene.director.currentShotLabel(),
				"A submarine's destruction must interrupt the overview's normal minimum hold");
	}

	@Test
	void imminentWeaponReplacesCurrentDistantTerminalApproach() throws Exception {
		Scene scene = new Scene(1920, 1080);
		int distantId = scene.nextTorpedoId;
		scene.launch(scene.subNodes.get(2).getLocalTranslation().add(500, 0, 0), 0);
		scene.step(1f / 60);
		assertEquals("Terminal Approach", scene.director.currentShotLabel());
		assertEquals(distantId, scene.trackedId);
		int imminentId = scene.nextTorpedoId;
		scene.launch(scene.subNodes.get(2).getLocalTranslation().add(100, 0, 0), 0);
		scene.step(1f / 60);
		assertEquals("Terminal Approach", scene.director.currentShotLabel());
		assertEquals(imminentId, scene.trackedId,
				"A newly imminent weapon must replace a distant approach even during its minimum hold");
	}

	@Test
	void removedTrackedWeaponMovesCoverageToExistingSubmarine() throws Exception {
		Scene scene = new Scene(1920, 1080);
		int weaponId = scene.nextTorpedoId;
		scene.launch(new Vector3f(-2000, -100, 1500), 0);
		for (int frame = 0; frame < 20 * 60 && !"Torpedo Track".equals(scene.director.currentShotLabel()); frame++)
			scene.step(1f / 60);
		assertEquals("Torpedo Track", scene.director.currentShotLabel());
		scene.torpedoNodes.remove(weaponId);
		scene.launches.remove(weaponId);
		scene.step(1f / 60);
		assertTrue(scene.subNodes.containsKey(scene.trackedId),
				"A vanished weapon must release its camera shot to an existing submarine");
		assertTrue(scene.position.length() > 500,
				"Removing a tracked torpedo must not point the camera at the world-origin fallback");
	}

	@Test
	void descentFromOverviewRetainsOverheadAltitudeDuringArrivalHold() throws Exception {
		Scene scene = new Scene(1920, 1080);
		scene.waitForOverview();
		for (int frame = 0; frame < 15 * 60; frame++) {
			float previousAltitude = scene.position.y;
			scene.step(1f / 60);
			if (OVERVIEW.equals(scene.director.currentShotLabel()))
				continue;
			assertTrue(previousAltitude > 100, "The battle overview must start above the water");
			assertTrue(scene.position.y > previousAltitude * .95f,
					"A smooth descent must begin at the completed overview's altitude");
			for (int holdFrame = 0; holdFrame < 25; holdFrame++) {
				scene.step(1f / 60);
				assertTrue(scene.position.y > 100,
						"The camera must remain overhead while water fades in during the arrival hold");
			}
			return;
		}
		throw new AssertionError("The battle overview never returned to a submarine shot");
	}

	private static void assertHullFits(Camera camera, Node node) {
		assertTrue(hullFits(camera, node), "Submarine hull clipped in " + camera.getWidth() + "x" + camera.getHeight());
	}

	private static boolean hullFits(Camera camera, Node node) {
		// The current 73m hull and sail fit inside this conservative body-space box.
		for (float x : new float[] {-6, 6})
			for (float y : new float[] {-35, 40})
				for (float z : new float[] {-4, 8}) {
					Vector3f world = node.getLocalRotation().mult(new Vector3f(x, y, z))
							.addLocal(node.getLocalTranslation());
					Vector3f screen = camera.getScreenCoordinates(world);
					float depth = world.subtract(camera.getLocation()).dot(camera.getDirection());
					if (depth <= camera.getFrustumNear() || screen.x < 0 || screen.x > camera.getWidth() || screen.y < 0
							|| screen.y > camera.getHeight())
						return false;
				}
		return true;
	}

	private static final class Scene {
		final Camera camera;
		final CinematicDirector director;
		final Map<Integer, Node> subNodes = new HashMap<>();
		final Map<Integer, Node> torpedoNodes = new HashMap<>();
		final Map<Integer, Double> launches = new HashMap<>();
		final Vector3f position = new Vector3f();
		final Vector3f look = new Vector3f();
		int trackedId;
		int nextTorpedoId = 1000;
		int secondSubHp = 1000;
		boolean torpedoDetonated;

		Scene(int width, int height) throws Exception {
			camera = new Camera(width, height);
			camera.setFrustumPerspective(45, width / (float) height, 1, 30000);
			subNodes.put(1, node(new Vector3f(-2000, -80, 0), 1.1f));
			subNodes.put(2, node(new Vector3f(2000, -120, 0), 3.7f));
			director = new CinematicDirector(this::submarines, this::torpedoes, subNodes, torpedoNodes, null, camera);
		}

		void launch(Vector3f position, double time) {
			torpedoNodes.put(nextTorpedoId, node(position, 0));
			launches.put(nextTorpedoId, time);
			nextTorpedoId++;
		}

		void expireTorpedoes(double beforeTime) {
			launches.entrySet().removeIf(entry -> {
				if (entry.getValue() >= beforeTime)
					return false;
				torpedoNodes.remove(entry.getKey());
				return true;
			});
		}

		void step(float dt) {
			trackedId = director.update(dt, position, look);
			camera.setLocation(position);
			float downness = Math.abs(look.subtract(position).normalizeLocal().dot(Vector3f.UNIT_Y));
			camera.lookAt(look, downness > .7f ? new Vector3f(0, 0, -1) : Vector3f.UNIT_Y);
			camera.update();
		}

		void waitForOverview() {
			for (int frame = 0; frame < 70 * 60; frame++) {
				step(1f / 60);
				if (OVERVIEW.equals(director.currentShotLabel()))
					return;
			}
			throw new AssertionError("The director never showed a battle overview");
		}

		List<SubmarineSnapshot> submarines() {
			return List.of(submarine(1, subNodes.get(1), 1.1f, 1000), submarine(2, subNodes.get(2), 3.7f, secondSubHp));
		}

		List<TorpedoSnapshot> torpedoes() {
			List<TorpedoSnapshot> result = new ArrayList<>();
			for (var entry : torpedoNodes.entrySet()) {
				result.add(new TorpedoSnapshot(entry.getKey(), 1, pose(entry.getValue(), 0), Velocity.ZERO, 20,
						Color.CYAN, 1000, torpedoDetonated, !torpedoDetonated, 0, false, Double.NaN, Double.NaN,
						Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
						Double.NaN, ""));
			}
			return result;
		}
	}

	private static Node node(Vector3f point, float heading) {
		Node node = new Node();
		node.setLocalTranslation(point);
		node.setLocalRotation(new Quaternion().fromAngles(-FastMath.HALF_PI, FastMath.PI - heading, 0));
		return node;
	}

	private static Pose pose(Node node, float heading) {
		Vector3f p = node.getLocalTranslation();
		return Pose.at(p.x, -p.z, p.y, heading);
	}

	private static SubmarineSnapshot submarine(int id, Node node, float heading, int hp) {
		return new SubmarineSnapshot(id, "Test" + id, "TS", pose(node, heading), Velocity.ZERO, 0, Color.CYAN, false,
				hp, 0, 0, 0, 0, 0, "", false, 0, List.of(), List.of(), List.of(), null, false);
	}

}
