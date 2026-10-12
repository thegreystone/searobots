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

import java.util.Objects;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.LongSupplier;

/**
 * Wakeable playback controls shared by the live simulation and replay worker. Frame processing
 * stays outside the lock; the worker uses {@link #awaitFrame()} before processing and
 * {@link #pace(long, int)} afterwards.
 * <p>
 * Engine-internal: this class is public only because the replay worker lives in another package.
 * Viewers and other callers drive playback through {@link SimClock}.
 */
public final class ClockControl implements SimClock {

	@FunctionalInterface
	interface NanoWaiter {
		/** Waits on the condition for at most the given budget; spurious wakeups are tolerated. */
		void await(Condition wake, long remainingNanos) throws InterruptedException;
	}

	private final LongSupplier nanoTime;
	private final NanoWaiter nanoWaiter;
	private final ReentrantLock lock = new ReentrantLock();
	private final Condition wake = lock.newCondition();
	private double speedMultiplier = 1.0;
	private boolean paused;
	private long pendingSteps;
	private boolean stopped;
	private SimulationLoop.State state = SimulationLoop.State.CREATED;

	public ClockControl() {
		// Condition.awaitNanos keeps sub-millisecond budgets. Object.wait(millis, nanos) rounds any
		// remainder up to the next whole millisecond, which would stretch 16x frames to 2 ms.
		this(System::nanoTime, Condition::awaitNanos);
	}

	ClockControl(LongSupplier nanoTime, NanoWaiter nanoWaiter) {
		this.nanoTime = Objects.requireNonNull(nanoTime, "nanoTime");
		this.nanoWaiter = Objects.requireNonNull(nanoWaiter, "nanoWaiter");
	}

	/** Mark setup in progress without restarting a stopped clock. */
	public void initialize() {
		lock.lock();
		try {
			if (!stopped) {
				state = SimulationLoop.State.INITIALIZING;
			}
		} finally {
			lock.unlock();
		}
	}

	/** Wait until the worker may process another frame, or return false when it must exit. */
	public boolean awaitFrame() {
		lock.lock();
		try {
			if (interrupted()) {
				return false;
			}
			while (paused && pendingSteps == 0 && !stopped) {
				state = SimulationLoop.State.PAUSED;
				try {
					wake.await();
				} catch (InterruptedException ex) {
					return stopAfterInterrupt();
				}
			}
			if (stopped || interrupted()) {
				return false;
			}
			if (paused) {
				pendingSteps--;
			}
			state = SimulationLoop.State.RUNNING;
			return true;
		} finally {
			lock.unlock();
		}
	}

	/** Monotonic start time to use when budgeting a frame. */
	public long frameStartNanos() {
		return nanoTime.getAsLong();
	}

	/**
	 * Wait for the remainder of the frame's wall-clock budget. Control changes wake the wait; speed
	 * changes recompute the budget from the original start. Pause and overrun do not create timing
	 * debt for later frames.
	 *
	 * @return false if stopped or interrupted
	 */
	public boolean pace(long frameStartNanos, int tickRateHz) {
		lock.lock();
		try {
			while (!stopped && !paused) {
				if (interrupted()) {
					return false;
				}
				long remaining = remainingNanos(frameStartNanos, nanoTime.getAsLong(), tickRateHz, speedMultiplier);
				if (remaining <= 0) {
					return true;
				}
				try {
					nanoWaiter.await(wake, remaining);
				} catch (InterruptedException ex) {
					return stopAfterInterrupt();
				}
			}
			if (!stopped && paused) {
				state = SimulationLoop.State.PAUSED;
			}
			return !stopped;
		} finally {
			lock.unlock();
		}
	}

	static long remainingNanos(long frameStartNanos, long nowNanos, int tickRateHz, double speedMultiplier) {
		// The narrowing conversion saturates huge intervals and rounds sub-nanosecond ones to zero.
		long interval = (long) (1_000_000_000.0 / tickRateHz / speedMultiplier);
		long elapsed = nowNanos - frameStartNanos;
		return elapsed >= interval ? 0 : interval - elapsed;
	}

	private boolean interrupted() {
		if (Thread.currentThread().isInterrupted()) {
			stop();
			return true;
		}
		return false;
	}

	private boolean stopAfterInterrupt() {
		stop();
		Thread.currentThread().interrupt();
		return false;
	}

	@Override
	public void setPaused(boolean paused) {
		lock.lock();
		try {
			this.paused = paused;
			if (!paused) {
				pendingSteps = 0;
			}
			wake.signalAll();
		} finally {
			lock.unlock();
		}
	}

	@Override
	public boolean isPaused() {
		lock.lock();
		try {
			return paused;
		} finally {
			lock.unlock();
		}
	}

	@Override
	public void setSpeedMultiplier(double multiplier) {
		if (!Double.isFinite(multiplier) || multiplier <= 0) {
			throw new IllegalArgumentException("Speed multiplier must be finite and positive");
		}
		lock.lock();
		try {
			speedMultiplier = multiplier;
			wake.signalAll();
		} finally {
			lock.unlock();
		}
	}

	@Override
	public double getSpeedMultiplier() {
		lock.lock();
		try {
			return speedMultiplier;
		} finally {
			lock.unlock();
		}
	}

	@Override
	public void stepOnce() {
		lock.lock();
		try {
			if (paused && !stopped) {
				pendingSteps++;
				wake.signalAll();
			}
		} finally {
			lock.unlock();
		}
	}

	@Override
	public void stop() {
		lock.lock();
		try {
			stopped = true;
			pendingSteps = 0;
			state = SimulationLoop.State.STOPPED;
			wake.signalAll();
		} finally {
			lock.unlock();
		}
	}

	public boolean isStopped() {
		lock.lock();
		try {
			return stopped;
		} finally {
			lock.unlock();
		}
	}

	@Override
	public SimulationLoop.State getState() {
		lock.lock();
		try {
			return state;
		} finally {
			lock.unlock();
		}
	}
}
