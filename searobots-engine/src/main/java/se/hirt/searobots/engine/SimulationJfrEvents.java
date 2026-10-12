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

import jdk.jfr.Category;
import jdk.jfr.Description;
import jdk.jfr.Event;
import jdk.jfr.EventType;
import jdk.jfr.Label;
import jdk.jfr.Name;
import jdk.jfr.StackTrace;
import jdk.jfr.Threshold;
import jdk.jfr.Unsigned;

import java.util.concurrent.atomic.AtomicLong;

import se.hirt.searobots.api.Vec3;

/** Engine timing context for JFR. Disabled recordings allocate no per-stage event objects. */
final class SimulationJfrEvents {

	static final String TICK_EVENT_NAME = "se.hirt.searobots.SimulationTick";
	static final String PHASE_EVENT_NAME = "se.hirt.searobots.SimulationPhase";
	static final String TERRAIN_EVENT_NAME = "se.hirt.searobots.TerrainMotion";
	private static final EventType TICK_TYPE = EventType.getEventType(SimulationTick.class);
	private static final EventType PHASE_TYPE = EventType.getEventType(SimulationPhase.class);
	private static final EventType TERRAIN_TYPE = EventType.getEventType(TerrainMotion.class);
	private static final AtomicLong RUN_IDS = new AtomicLong();

	@Name(TICK_EVENT_NAME)
	@Label("Simulation Tick")
	@Category({"SeaRobots", "Engine"})
	@Description("Simulation work, including listener delivery, excluding deliberate pacing and pauses")
	@Threshold("1 ms")
	@StackTrace(false)
	static final class SimulationTick extends Event {
		@Label("Run ID")
		@Description("Unique SimulationLoop run within this JVM, including repeated runs of the same seed")
		long correlationId;
		@Label("World Seed")
		long worldSeed;
		@Description("Zero-based simulation tick index")
		@Label("Tick")
		@Unsigned
		long tick;
		@Description("Number of participating vehicles, including wrecks")
		@Unsigned
		int vehicleCount;
		@Description("Number of torpedoes retained at the end of this stage, including any still in tubes")
		@Unsigned
		int torpedoCount;
	}

	@Name(PHASE_EVENT_NAME)
	@Label("Simulation Phase")
	@Category({"SeaRobots", "Engine"})
	@Description("Engine stage duration and its post-stage context; entity fields are unavailable for global stages")
	@Threshold("1 ms")
	@StackTrace(false)
	static final class SimulationPhase extends Event {
		String phase;
		@Label("Run ID")
		@Description("Unique SimulationLoop run within this JVM, matching its enclosing SimulationTick")
		long correlationId;
		@Label("World Seed")
		long worldSeed;
		@Description("Zero-based simulation tick index")
		@Label("Tick")
		@Unsigned
		long tick;
		@Description("Number of participating vehicles, including wrecks")
		@Unsigned
		int vehicleCount;
		@Description("Number of torpedoes retained at the end of this stage, including any still in tubes")
		@Unsigned
		int torpedoCount;
		@Description("Submarine or torpedo ID; -1 for a global stage")
		int entityId = -1;
		@Description("Torpedo owner ID; -1 for submarine and global stages")
		int ownerId = -1;
		String controllerClass;
		@Description("Post-stage submarine hit points; -1 for torpedo and global stages")
		int hp = -1;
		@Description("Post-stage world X in metres; NaN for global stages")
		@Label("X (m)")
		double x = Double.NaN;
		@Description("Post-stage world Y in metres; NaN for global stages")
		@Label("Y (m)")
		double y = Double.NaN;
		@Description("Post-stage world Z in metres; NaN for global stages")
		@Label("Z (m)")
		double z = Double.NaN;
	}

	@Name(TERRAIN_EVENT_NAME)
	@Label("Terrain Motion")
	@Category({"SeaRobots", "Engine"})
	@Description("Submarine terrain sweep and response, including continuation of the unspent tick after impact")
	@Threshold("1 ms")
	@StackTrace(false)
	static final class TerrainMotion extends Event {
		@Label("Run ID")
		@Description("Enclosing SimulationLoop run ID; zero for standalone SubmarinePhysics")
		long correlationId;
		long worldSeed;
		@Label("Tick")
		@Description("Zero-based simulation tick index; -1 when a standalone physics caller supplies no tick")
		long tick;
		@Label("Tick Duration (s)")
		@Description("Simulated interval the physics step attempted, in seconds; remainingSeconds is the unspent part")
		double tickDurationSeconds;
		int entityId;
		@Description("Submarine hit points before the terrain sweep")
		int hpAtStart;
		@Description("Submarine hit points after terrain sweep and impact damage")
		int hpAtEnd;
		@Label("Start X (m)")
		@Description("World X at the start of the attempted motion, in metres")
		double startX;
		@Label("Start Y (m)")
		@Description("World Y at the start of the attempted motion, in metres")
		double startY;
		@Label("Start Z (m)")
		@Description("World Z at the start of the attempted motion, in metres")
		double startZ;
		@Label("End X (m)")
		@Description("World X after terrain response, in metres")
		double endX;
		@Label("End Y (m)")
		@Description("World Y after terrain response, in metres")
		double endY;
		@Label("End Z (m)")
		@Description("World Z after terrain response, in metres")
		double endZ;
		@Unsigned
		@Description("Number of sweep/response iterations, at most six")
		int iterations;
		@Unsigned
		@Description("Number of swept first-contact results, excluding static overlap projection")
		int sweptContacts;
		@Unsigned
		@Label("Support Projection Iterations")
		@Description("Finite-motion velocity projection passes for established seabed support")
		int supportIterations;
		@Unsigned
		@Description("Sum of material contact candidates examined over the response iterations")
		int contactPointCount;
		@Label("Remaining Time (s)")
		@Description("Unspent tick duration at exit, in seconds; zero after a contact-free continuation")
		double remainingSeconds;
		@Description("Last swept contact's fraction of its attempted segment, from zero to one; NaN if none")
		double lastContactFraction = Double.NaN;
	}

	private SimulationJfrEvents() {
	}

	static long nextRunId() {
		return RUN_IDS.incrementAndGet();
	}

	static SimulationTick beginTick() {
		if (!TICK_TYPE.isEnabled())
			return null;
		var event = new SimulationTick();
		event.begin();
		return event;
	}

	static SimulationPhase beginPhase() {
		if (!PHASE_TYPE.isEnabled())
			return null;
		var event = new SimulationPhase();
		event.begin();
		return event;
	}

	static TerrainMotion beginTerrainMotion() {
		if (!TERRAIN_TYPE.isEnabled())
			return null;
		var event = new TerrainMotion();
		event.begin();
		return event;
	}

	static void finishTerrainMotion(
		TerrainMotion event, long correlationId, long worldSeed, long tick, double tickDurationSeconds,
		SubmarineEntity entity, Vec3 start, int hpAtStart) {
		if (event == null)
			return;
		event.end();
		if (!event.shouldCommit())
			return;
		event.correlationId = correlationId;
		event.worldSeed = worldSeed;
		event.tick = tick;
		event.tickDurationSeconds = tickDurationSeconds;
		event.entityId = entity.id();
		event.hpAtStart = hpAtStart;
		event.hpAtEnd = entity.hp();
		event.startX = start.x();
		event.startY = start.y();
		event.startZ = start.z();
		event.endX = entity.x();
		event.endY = entity.y();
		event.endZ = entity.z();
		event.commit();
	}

	static void finishTick(
		SimulationTick event, long correlationId, long worldSeed, long tick, int vehicleCount, int torpedoCount) {
		if (event == null)
			return;
		event.end();
		if (!event.shouldCommit())
			return;
		event.correlationId = correlationId;
		event.worldSeed = worldSeed;
		event.tick = tick;
		event.vehicleCount = vehicleCount;
		event.torpedoCount = torpedoCount;
		event.commit();
	}

	static void finishPhase(
		SimulationPhase event, String phase, long correlationId, long worldSeed, long tick, int vehicleCount,
		int torpedoCount) {
		if (completePhase(event, phase, correlationId, worldSeed, tick, vehicleCount, torpedoCount))
			event.commit();
	}

	static void finishPhase(
		SimulationPhase event, String phase, long correlationId, long worldSeed, long tick, SubmarineEntity entity,
		int vehicleCount, int torpedoCount) {
		if (!completePhase(event, phase, correlationId, worldSeed, tick, vehicleCount, torpedoCount))
			return;
		event.entityId = entity.id();
		event.controllerClass = entity.controller().getClass().getName();
		event.hp = entity.hp();
		event.x = entity.x();
		event.y = entity.y();
		event.z = entity.z();
		event.commit();
	}

	static void finishPhase(
		SimulationPhase event, String phase, long correlationId, long worldSeed, long tick, TorpedoEntity entity,
		int vehicleCount, int torpedoCount) {
		if (!completePhase(event, phase, correlationId, worldSeed, tick, vehicleCount, torpedoCount))
			return;
		event.entityId = entity.id();
		event.ownerId = entity.ownerId();
		event.controllerClass = entity.controller().getClass().getName();
		event.x = entity.x();
		event.y = entity.y();
		event.z = entity.z();
		event.commit();
	}

	private static boolean completePhase(
		SimulationPhase event, String phase, long correlationId, long worldSeed, long tick, int vehicleCount,
		int torpedoCount) {
		if (event == null)
			return false;
		event.end();
		if (!event.shouldCommit())
			return false;
		event.phase = phase;
		event.correlationId = correlationId;
		event.worldSeed = worldSeed;
		event.tick = tick;
		event.vehicleCount = vehicleCount;
		event.torpedoCount = torpedoCount;
		return true;
	}
}
