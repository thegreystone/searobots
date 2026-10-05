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
import se.hirt.searobots.engine.ships.SimpleTorpedoController;

import java.awt.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * A torpedo in its tube rides along with the submarine, whatever it does, and is released only once
 * it has cleared the muzzle.
 */
public class TorpedoTubeTest {

	private static final double DT = 1.0 / 50;

	@Test
	void torpedoFollowsTheTubeWhileTheSubmarineManoeuvres() {
		var sub = new SubmarineEntity(VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
				new Vec3(100, 200, -80), 0.3, Color.RED, 1000);
		sub.setSpeed(8);
		sub.setPitch(0.05);
		var tube = TorpedoTubes.TUBES.get(1); // starboard upper
		var torp = new TorpedoEntity(1000, sub.id(), VehicleConfig.torpedo(), new SimpleTorpedoController(),
				new Vec3(sub.x(), sub.y(), sub.z()), sub.heading(), sub.pitch(), 20, Color.RED);
		torp.loadIntoTube(sub, tube, DT, "");

		double half = VehicleConfig.torpedo().hullHalfLength();
		double start = tube.muzzleForward() - half - TorpedoTubes.NOSE_GAP;
		double clear = tube.muzzleForward() + half + TorpedoTubes.CLEARANCE;
		int holdTicks = (int) Math.round(TorpedoTubes.DOOR_SECONDS / DT);
		int expectedTicks = holdTicks + (int) Math.ceil((clear - start) / (TorpedoTubes.EJECTION_SPEED * DT));

		int ticks = 0;
		boolean released = false;
		while (!released) {
			// The submarine turns hard, pitches and moves on, as it could right after firing
			sub.setHeading(sub.heading() + 0.1 * DT);
			sub.setPitch(sub.pitch() + 0.02 * DT);
			sub.setX(sub.x() + 8 * DT * Math.sin(sub.heading()));
			sub.setY(sub.y() + 8 * DT * Math.cos(sub.heading()));
			released = torp.advanceInTube(DT);
			ticks++;

			// Wherever the submarine is, the torpedo sits on the tube's line in its frame, pointing the same way
			double[] local = toSubFrame(sub, torp.x(), torp.y(), torp.z());
			assertEquals(tube.right(), local[0], 1e-9, "starboard offset");
			assertEquals(tube.up(), local[2], 1e-9, "up offset");
			assertEquals(sub.heading(), torp.heading(), 1e-12);
			assertEquals(sub.pitch(), torp.pitch(), 1e-12);
			// Waits behind the door, then slides out at the ejection speed
			double expectedForward = ticks <= holdTicks ? start
					: start + (ticks - holdTicks) * TorpedoTubes.EJECTION_SPEED * DT;
			assertEquals(expectedForward, local[1], 1e-9, "distance along the tube at tick " + ticks);
			assertTrue(torp.inTube());
			assertTrue(ticks <= expectedTicks, "still in the tube after " + ticks + " ticks");
		}
		assertEquals(expectedTicks, ticks);
		torp.leaveTube();
		assertFalse(torp.inTube());
		assertEquals(sub.speed() + TorpedoTubes.EJECTION_SPEED, torp.speed(), 1e-12);
	}

	/**
	 * {right, forward, up} of a world point in the submarine's frame (as TorpedoEntity places it).
	 */
	private static double[] toSubFrame(SubmarineEntity sub, double x, double y, double z) {
		double dx = x - sub.x(), dy = y - sub.y(), dz = z - sub.z();
		double sinH = Math.sin(sub.heading()), cosH = Math.cos(sub.heading());
		double sinP = Math.sin(sub.pitch()), cosP = Math.cos(sub.pitch());
		double right = dx * cosH - dy * sinH;
		double forward = dx * sinH * cosP + dy * cosH * cosP + dz * sinP;
		double up = -dx * sinH * sinP - dy * cosH * sinP + dz * cosP;
		return new double[] {right, forward, up};
	}
}
