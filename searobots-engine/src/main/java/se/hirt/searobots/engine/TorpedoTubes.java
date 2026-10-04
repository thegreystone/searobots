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

import java.util.List;

/**
 * The submarine's torpedo tubes and the launch sequence through them.
 * <p>
 * A launched torpedo is first loaded into the submarine's next tube (each submarine cycles through
 * its own tubes), nose just behind the muzzle door. It rides along with the submarine while the
 * door opens ({@link #DOOR_SECONDS}), then slides out at {@link #EJECTION_SPEED} relative to the
 * submarine until its tail has cleared the muzzle by {@link #CLEARANCE}. Only then is it released:
 * from that moment its own controller and physics take over. That way the torpedo follows the tube
 * however the submarine manoeuvres, instead of the hull swinging through it.
 * <p>
 * Tube positions are in the submarine's frame: {@code right} is to starboard, {@code up} along the
 * submarine's up direction, {@code muzzleForward} how far ahead of the submarine's position the
 * tube meets the hull. They must match the 3D model: SubmarineModelGenerator builds the muzzles and
 * doors from the same right/up values and prints the muzzle positions used here.
 */
public final class TorpedoTubes {

	/**
	 * One tube: offsets to starboard and up from the submarine's position, and where it meets the
	 * hull.
	 */
	public record Tube(double right, double up, double muzzleForward) {
	}

	/** Port upper, starboard upper, port lower, starboard lower (TubeDoor1..4 in the model). */
	public static final List<Tube> TUBES = List.of(new Tube(-1.7, -0.6, 33.161), new Tube(1.7, -0.6, 33.161),
			new Tube(-1.25, -1.45, 32.601), new Tube(1.25, -1.45, 32.601));

	/** Time the torpedo waits in the tube for the door to open. */
	public static final double DOOR_SECONDS = 1.0;
	/** Speed the torpedo leaves the tube with, relative to the submarine (m/s). */
	public static final double EJECTION_SPEED = 3.0;
	/** How far the torpedo's tail must be past the muzzle before it is released (m). */
	public static final double CLEARANCE = 0.5;
	/** Gap between the torpedo's nose and the closed door when it is loaded (m). */
	public static final double NOSE_GAP = 0.3;

	private TorpedoTubes() {
	}
}
