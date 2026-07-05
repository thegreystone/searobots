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

/**
 * The playback control surface shared by a live {@link SimulationLoop} and a
 * {@link se.hirt.searobots.engine.replay.ReplayPlayer}. Because a recorded match replays through a
 * {@link SimulationListener} exactly like a live one, the viewer can drive either with the same pause / step / speed
 * controls simply by holding a {@code SimClock} instead of a concrete loop.
 * <p>
 * The {@link SimulationLoop.State} enum is reused as the shared state type so existing viewer code that switches on it
 * needs no changes.
 */
public interface SimClock {

	/** Pause or unpause advancement. A paused clock emits no further frames until unpaused or stepped. */
	void setPaused(boolean paused);

	boolean isPaused();

	/** Wall-clock speed multiplier: {@code 1.0} is real time, higher is faster; very large means "as fast as possible". */
	void setSpeedMultiplier(double multiplier);

	double getSpeedMultiplier();

	/** While paused, advance exactly one frame. */
	void stepOnce();

	/** Stop for good; the clock transitions to {@link SimulationLoop.State#STOPPED} and cannot be restarted. */
	void stop();

	SimulationLoop.State getState();
}
