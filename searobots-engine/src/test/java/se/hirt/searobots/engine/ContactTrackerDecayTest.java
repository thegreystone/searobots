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

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ContactTrackerDecayTest {

	@Test
	void oneSecondDropoutCoastsAtTheDeclaredRates() {
		var tracker = fixedTracker();
		for (long tick = 101; tick <= 150; tick++) {
			tracker.decay(tick, 15);
		}

		assertEquals(0.94, tracker.solutionQuality(), 1e-12);
		assertEquals(55, tracker.rangeUncertainty(), 1e-9);
	}

	@Test
	void dropoutDecayDependsOnElapsedTimeRatherThanUpdateCadence() {
		var everyTick = fixedTracker();
		var once = fixedTracker();
		for (long tick = 101; tick <= 700; tick++) {
			everyTick.decay(tick, 15);
		}
		once.decay(700, 15);

		assertEquals(0.83, everyTick.solutionQuality(), 1e-12);
		assertEquals(220, everyTick.rangeUncertainty(), 1e-9);
		assertEquals(once.solutionQuality(), everyTick.solutionQuality(), 1e-12);
		assertEquals(once.rangeUncertainty(), everyTick.rangeUncertainty(), 1e-9);
	}

	@Test
	void repeatedDecayAtTheSameTickIsIdempotent() {
		var tracker = fixedTracker();
		tracker.decay(150, 15);
		tracker.decay(150, 15);

		assertEquals(0.94, tracker.solutionQuality(), 1e-12);
		assertEquals(55, tracker.rangeUncertainty(), 1e-9);
	}

	@Test
	void newActiveFixStartsANewCoastingInterval() {
		var tracker = fixedTracker();
		tracker.decay(150, 15);
		tracker.updateFromPing(200, 1000);
		tracker.decay(201, 15);
		tracker.decay(202, 15);

		assertEquals(0.9496, tracker.solutionQuality(), 1e-12);
		assertEquals(20.6, tracker.rangeUncertainty(), 1e-9);
	}

	@Test
	void passiveReacquisitionStartsANewCoastingInterval() {
		var tracker = fixedTracker();
		tracker.decay(150, 15);
		tracker.update(200, 0, 20, 0, 0, 0, false, 2000, 0, 2000, new Random(42));
		double quality = tracker.solutionQuality();
		double uncertainty = tracker.rangeUncertainty();
		tracker.decay(201, 15);
		tracker.decay(202, 15);

		assertEquals(quality - 0.0004, tracker.solutionQuality(), 1e-12);
		assertEquals(uncertainty + 0.6, tracker.rangeUncertainty(), 1e-9);
	}

	@Test
	void expirationUsesLastObservationRatherThanLastCoastingUpdate() {
		var tracker = fixedTracker();
		tracker.decay(1600, 15);
		assertFalse(tracker.isExpired(1600));
		tracker.decay(1601, 15);
		assertTrue(tracker.isExpired(1601));
	}

	private static ContactTracker fixedTracker() {
		var tracker = new ContactTracker();
		tracker.updateFromPing(100, 2000);
		return tracker;
	}
}
