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
import org.junit.jupiter.api.Timeout;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

class ClockControlTest {

	@Test
	void waitUsesOnlyTheRemainingBudget() {
		var nanos = new AtomicLong(100);
		var waits = new ArrayList<Long>();
		var clock = new ClockControl(nanos::get, (wake, remaining) -> {
			waits.add(remaining);
			nanos.addAndGet(remaining);
		});
		long start = clock.frameStartNanos();
		nanos.addAndGet(5_000_000);

		assertTrue(clock.pace(start, 50));
		assertEquals(List.of(15_000_000L), waits);
	}

	@Test
	void overrunsDoNotWaitOrCreateDebtForTheNextFrame() {
		var nanos = new AtomicLong();
		var waits = new ArrayList<Long>();
		var clock = new ClockControl(nanos::get, (wake, remaining) -> {
			waits.add(remaining);
			nanos.addAndGet(remaining);
		});
		long start = clock.frameStartNanos();
		nanos.addAndGet(25_000_000);

		assertTrue(clock.pace(start, 50));
		assertEquals(List.of(), waits);
		assertTrue(clock.pace(clock.frameStartNanos(), 50));
		assertEquals(List.of(20_000_000L), waits);
	}

	@Test
	void speedChangeRecomputesWaitFromOriginalStart() {
		var nanos = new AtomicLong();
		var waits = new ArrayList<Long>();
		var clocks = new ClockControl[1];
		clocks[0] = new ClockControl(nanos::get, (wake, remaining) -> {
			waits.add(remaining);
			if (waits.size() == 1) {
				nanos.addAndGet(5_000_000);
				clocks[0].setSpeedMultiplier(2);
			} else {
				nanos.addAndGet(remaining);
			}
		});

		assertTrue(clocks[0].pace(clocks[0].frameStartNanos(), 50));
		assertEquals(List.of(20_000_000L, 5_000_000L), waits);
	}

	@Test
	void budgetsHandleExtremeSpeedsFractionalNanosAndNanoTimeWrap() {
		assertEquals(Long.MAX_VALUE, ClockControl.remainingNanos(0, 0, 50, Double.MIN_VALUE));
		assertEquals(Long.MAX_VALUE - 10, ClockControl.remainingNanos(0, 10, 50, Double.MIN_VALUE));
		assertEquals(0, ClockControl.remainingNanos(0, 0, 50, Double.MAX_VALUE));
		assertEquals(1, ClockControl.remainingNanos(0, 0, 50, 20_000_000));
		assertEquals(0, ClockControl.remainingNanos(0, 1, 50, 20_000_000));
		assertEquals(19_999_000, ClockControl.remainingNanos(Long.MAX_VALUE - 499, Long.MIN_VALUE + 500, 50, 1));
	}

	@Test
	@Timeout(value = 10, unit = TimeUnit.SECONDS)
	void realWaitKeepsSubMillisecondBudgets() {
		// Object.wait(millis, nanos) rounds any remainder up to the next whole millisecond, which
		// would stretch the 1.25 ms frames of a 16x fast-forward to 2 ms. The fake waiters above
		// cannot see that, so this test paces real 800 Hz frames and checks their median.
		var clock = new ClockControl();
		for (int i = 0; i < 10; i++) {
			clock.pace(clock.frameStartNanos(), 800);
		}
		long[] frames = new long[50];
		for (int i = 0; i < frames.length; i++) {
			long start = clock.frameStartNanos();
			assertTrue(clock.pace(start, 800));
			frames[i] = clock.frameStartNanos() - start;
			assertTrue(frames[i] >= 1_250_000, "a frame must use its whole 1.25 ms budget, took " + frames[i] + " ns");
		}
		Arrays.sort(frames);
		long median = frames[frames.length / 2];
		assertTrue(median < 1_900_000, "median frame of " + median + " ns indicates whole-millisecond rounding");
	}
}
