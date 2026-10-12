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
package se.hirt.searobots.engine.replay;

import se.hirt.searobots.engine.ClockControl;
import se.hirt.searobots.engine.SimClock;
import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SimulationLoop;

import java.util.List;
import java.util.Objects;

/**
 * Plays a recorded match back through a {@link SimulationListener} under the same control surface
 * as a live {@link SimulationLoop}. Because the viewers are {@code SimulationListener}s and this
 * exposes {@link SimClock}, a replay is driven by the exact same pause / single-step / speed
 * controls as a live simulation, with no changes to the viewer's control code.
 * <p>
 * The whole match is pre-decoded into a {@link ReplayFrame} list (see
 * {@link ReplayReader#readAll()}), so playback is pure emission: {@link #run()} walks the frames on
 * a dedicated thread, emitting each at the recorded tick rate scaled by the speed multiplier,
 * honoring pause and single-step exactly like {@link SimulationLoop}. Fast-forward-to-event is just
 * "max speed until the fan-out pauses us", so no separate seek machinery is needed.
 */
public final class ReplayPlayer implements SimClock {

	private final List<ReplayFrame> frames;
	private final int tickRateHz;
	private final SimulationListener listener;

	private final ClockControl clockControl;
	private volatile int index; // next frame to emit
	private volatile long currentTick; // tick of the most recently emitted frame

	public ReplayPlayer(List<ReplayFrame> frames, int tickRateHz, SimulationListener listener) {
		this(frames, tickRateHz, listener, new ClockControl());
	}

	ReplayPlayer(List<ReplayFrame> frames, int tickRateHz, SimulationListener listener, ClockControl clockControl) {
		this.frames = List.copyOf(frames);
		this.tickRateHz = tickRateHz > 0 ? tickRateHz : 50;
		this.listener = Objects.requireNonNull(listener, "listener");
		this.clockControl = Objects.requireNonNull(clockControl, "clockControl");
	}

	/**
	 * Walks the recorded frames, emitting each to the listener at the recorded tick rate (scaled by
	 * the speed multiplier) and honoring pause / single-step / stop. Blocks until the match ends or
	 * {@link #stop()} is called. Always fires {@link SimulationListener#onMatchEnd()} on exit,
	 * mirroring {@link SimulationLoop}.
	 */
	public void run() {
		clockControl.initialize();
		try {
			while (index < frames.size()) {
				// Playback may start paused so viewers can register before frames flow.
				if (!clockControl.awaitFrame()) {
					break;
				}
				long frameStartNanos = clockControl.frameStartNanos();

				ReplayFrame frame = frames.get(index);
				currentTick = frame.tick();
				// The listener (the viewer's event-pausing fan-out) may pause us during emit,
				// exactly as it does for a live loop; the next iteration then blocks.
				frame.emit(listener);
				index++;

				if (index < frames.size() && !clockControl.pace(frameStartNanos, tickRateHz)) {
					break;
				}
			}
		} finally {
			clockControl.stop();
			listener.onMatchEnd();
		}
	}

	/** Tick of the most recently emitted frame. */
	public long currentTick() {
		return currentTick;
	}

	/** Total number of recorded frames. */
	public int frameCount() {
		return frames.size();
	}

	// ── SimClock ────────────────────────────────────────────────────

	@Override
	public void setPaused(boolean paused) {
		clockControl.setPaused(paused);
	}

	@Override
	public boolean isPaused() {
		return clockControl.isPaused();
	}

	@Override
	public void setSpeedMultiplier(double multiplier) {
		clockControl.setSpeedMultiplier(multiplier);
	}

	@Override
	public double getSpeedMultiplier() {
		return clockControl.getSpeedMultiplier();
	}

	@Override
	public void stepOnce() {
		clockControl.stepOnce();
	}

	@Override
	public void stop() {
		clockControl.stop();
	}

	@Override
	public SimulationLoop.State getState() {
		return clockControl.getState();
	}
}
