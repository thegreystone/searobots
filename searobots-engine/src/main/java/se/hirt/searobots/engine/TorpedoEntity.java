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

import se.hirt.searobots.api.*;

import java.awt.*;

/**
 * A torpedo entity in the simulation. Autonomous after launch: controlled by its own
 * {@link TorpedoController}, receives sonar contacts independently, and has no communication link
 * with the launching submarine.
 * <p>
 * Torpedoes are fuel-limited, slightly negatively buoyant, and rely on hydrodynamic lift from
 * forward motion to maintain depth. Below minimum speed (~3 m/s), lift is insufficient and the
 * torpedo sinks.
 */
public final class TorpedoEntity {

	private static final double DEFAULT_FUEL_SECONDS = 240.0; // 4 minutes of powered run
	private static final double MINIMUM_SPEED = 3.0; // below this, torpedo sinks
	private static final double SINK_ACCELERATION = 2.0; // m/s^2 downward when stalled

	private final VehicleConfig vehicleConfig;
	private final int id;
	private final int ownerId;
	private final TorpedoController controller;
	private final Color color;
	private final double fuseRadius;
	private int armingTicks = 100; // fuse doesn't activate for 2 seconds
	private long launchTick; // tick when torpedo was created

	// Position and orientation
	private double x, y, z;
	private double heading;
	private double pitch;
	private double speed;
	private double verticalSpeed;
	private double yawRate;
	private double pitchRate;

	// Fuel
	private double fuelRemaining; // seconds

	// Actuator state (slew-limited)
	private double actualThrottle;
	private double actualRudder;
	private double actualSternPlanes;

	// Sonar
	private double sourceLevelDb;
	private boolean pingRequested;
	private int activeSonarCooldown;

	// Lifecycle
	private boolean alive = true;
	private boolean detonated;
	private boolean detonateRequested;
	private boolean explosionProcessed; // true after handleDetonation applies damage

	// Published target (for viewer visualization)
	private double pubTargetX = Double.NaN, pubTargetY = Double.NaN, pubTargetZ = Double.NaN;

	// Guidance diagnostics (for analysis)
	private double diagEstX = Double.NaN, diagEstY = Double.NaN, diagEstZ = Double.NaN;
	private double diagEstHeading = Double.NaN, diagEstSpeed = Double.NaN;
	private double diagIntX = Double.NaN, diagIntY = Double.NaN, diagIntZ = Double.NaN;
	private String diagPhase = "";

	// Pending launch command output (used by TorpedoOutput impl)
	private double cmdRudder, cmdSternPlanes, cmdThrottle = 1.0;

	public TorpedoEntity(int id, int ownerId, VehicleConfig vehicleConfig, TorpedoController controller, Vec3 launchPos,
			double heading, double pitch, double fuseRadius, Color color) {
		this.id = id;
		this.ownerId = ownerId;
		this.vehicleConfig = vehicleConfig;
		this.controller = controller;
		this.x = launchPos.x();
		this.y = launchPos.y();
		this.z = launchPos.z();
		this.heading = heading;
		this.pitch = pitch;
		this.fuseRadius = fuseRadius;
		this.color = color;
		this.fuelRemaining = DEFAULT_FUEL_SECONDS;
		this.speed = 5.0; // initial launch speed (ejected from tube)
		this.actualThrottle = 1.0; // full throttle at launch
	}

	// ── Accessors ──────────────────────────────────────────────────

	public int id() {
		return id;
	}

	public int ownerId() {
		return ownerId;
	}

	public VehicleConfig vehicleConfig() {
		return vehicleConfig;
	}

	public TorpedoController controller() {
		return controller;
	}

	public Color color() {
		return color;
	}

	public double fuseRadius() {
		return fuseRadius;
	}

	public double x() {
		return x;
	}

	public double y() {
		return y;
	}

	public double z() {
		return z;
	}

	public double heading() {
		return heading;
	}

	public double pitch() {
		return pitch;
	}

	public double speed() {
		return speed;
	}

	public double verticalSpeed() {
		return verticalSpeed;
	}

	public double yawRate() {
		return yawRate;
	}

	public double pitchRate() {
		return pitchRate;
	}

	public double fuelRemaining() {
		return fuelRemaining;
	}

	public double sourceLevelDb() {
		return sourceLevelDb;
	}

	public boolean pingRequested() {
		return pingRequested;
	}

	public int activeSonarCooldown() {
		return activeSonarCooldown;
	}

	public boolean alive() {
		return alive;
	}

	public boolean detonated() {
		return detonated;
	}

	public boolean detonateRequested() {
		return detonateRequested;
	}

	// ── Mutators (called by physics/engine) ────────────────────────

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public void setZ(double z) {
		this.z = z;
	}

	public void setHeading(double h) {
		this.heading = h;
	}

	public void setPitch(double p) {
		this.pitch = p;
	}

	public void setSpeed(double s) {
		this.speed = s;
	}

	public void setVerticalSpeed(double v) {
		this.verticalSpeed = v;
	}

	public void setYawRate(double r) {
		this.yawRate = r;
	}

	public void setPitchRate(double r) {
		this.pitchRate = r;
	}

	public void setSourceLevelDb(double db) {
		this.sourceLevelDb = db;
	}

	public void setActualThrottle(double t) {
		this.actualThrottle = t;
	}

	public void setActualRudder(double r) {
		this.actualRudder = r;
	}

	public void setActualSternPlanes(double p) {
		this.actualSternPlanes = p;
	}

	public double actualThrottle() {
		return actualThrottle;
	}

	public double actualRudder() {
		return actualRudder;
	}

	public double actualSternPlanes() {
		return actualSternPlanes;
	}

	public void consumeFuel(double seconds) {
		fuelRemaining = Math.max(0, fuelRemaining - seconds);
	}

	public void kill() {
		alive = false;
	}

	public void detonate() {
		detonated = true;
		alive = false;
	}

	public boolean explosionProcessed() {
		return explosionProcessed;
	}

	public void setExplosionProcessed() {
		explosionProcessed = true;
	}

	public void decrementSonarCooldown() {
		if (activeSonarCooldown > 0)
			activeSonarCooldown--;
	}

	/**
	 * True if the fuse is armed (arming delay has elapsed).
	 */
	public boolean fuseArmed() {
		return armingTicks <= 0;
	}

	public void decrementArmingDelay() {
		if (armingTicks > 0)
			armingTicks--;
	}

	public long launchTick() {
		return launchTick;
	}

	public void setLaunchTick(long tick) {
		this.launchTick = tick;
	}

	// ── Torpedo tube ───────────────────────────────────────────────

	private SubmarineEntity tubeOwner; // non-null while the torpedo is still in its tube
	private TorpedoTubes.Tube tube;
	private double tubeForward, tubeClearForward;
	private int tubeHoldTicks;
	private String missionData = "";

	/**
	 * Loads the torpedo into {@code tube} of {@code owner}, nose just behind the muzzle door, and
	 * places it there. It stays in the tube until {@link #advanceInTube} reports it clear.
	 */
	public void loadIntoTube(SubmarineEntity owner, TorpedoTubes.Tube tube, double dt, String missionData) {
		double half = vehicleConfig.hullHalfLength();
		this.tubeOwner = owner;
		this.tube = tube;
		this.tubeForward = tube.muzzleForward() - half - TorpedoTubes.NOSE_GAP;
		this.tubeClearForward = tube.muzzleForward() + half + TorpedoTubes.CLEARANCE;
		this.tubeHoldTicks = (int) Math.round(TorpedoTubes.DOOR_SECONDS / dt);
		this.missionData = missionData;
		placeInTube(0);
	}

	public boolean inTube() {
		return tubeOwner != null;
	}

	/** The submarine whose tube the torpedo is in, or null once it has left. */
	public SubmarineEntity tubeOwner() {
		return tubeOwner;
	}

	/**
	 * Mission data from the launch command, handed to the controller when the torpedo leaves the
	 * tube.
	 */
	public String missionData() {
		return missionData;
	}

	/**
	 * One tick in the tube: waits for the door, then slides out at the ejection speed, always
	 * placed in the owner's current frame so it follows every manoeuvre. Returns true once its tail
	 * has cleared the muzzle.
	 */
	public boolean advanceInTube(double dt) {
		double slide = 0;
		if (tubeHoldTicks > 0) {
			tubeHoldTicks--;
		} else {
			slide = TorpedoTubes.EJECTION_SPEED;
			tubeForward += slide * dt;
		}
		placeInTube(slide);
		return tubeForward >= tubeClearForward;
	}

	/** Releases the torpedo from its tube: from now on its own controller and physics move it. */
	public void leaveTube() {
		speed = Math.max(tubeOwner.speed(), 0) + TorpedoTubes.EJECTION_SPEED;
		tubeOwner = null;
	}

	/**
	 * Puts the torpedo in its tube in the owner's current frame, moving with it plus {@code slide}
	 * along the tube.
	 */
	private void placeInTube(double slide) {
		var o = tubeOwner;
		double sinH = Math.sin(o.heading()), cosH = Math.cos(o.heading());
		double sinP = Math.sin(o.pitch()), cosP = Math.cos(o.pitch());
		// Same frame as the launch geometry: forward along the bow, right to starboard, up across both
		double fx = sinH * cosP, fy = cosH * cosP, fz = sinP;
		double rx = cosH, ry = -sinH;
		double ux = -sinH * sinP, uy = -cosH * sinP, uz = cosP;
		x = o.x() + fx * tubeForward + rx * tube.right() + ux * tube.up();
		y = o.y() + fy * tubeForward + ry * tube.right() + uy * tube.up();
		z = o.z() + fz * tubeForward + uz * tube.up(); // right has no vertical component
		heading = o.heading();
		pitch = o.pitch();
		speed = Math.max(o.speed(), 0) + slide;
		verticalSpeed = 0;
		yawRate = 0;
		pitchRate = 0;
	}

	// ── Controller interface ───────────────────────────────────────

	/**
	 * Applies controller commands. Called after controller.onTick().
	 */
	public void applyCommands() {
		// Store commanded values for physics to slew toward
	}

	/**
	 * Returns a TorpedoOutput that captures commands from the controller.
	 */
	public TorpedoOutput createOutput() {
		return new TorpedoOutput() {
			@Override
			public void setRudder(double value) {
				cmdRudder = Math.clamp(value, -1, 1);
			}

			@Override
			public void setSternPlanes(double value) {
				cmdSternPlanes = Math.clamp(value, -1, 1);
			}

			@Override
			public void setThrottle(double value) {
				cmdThrottle = Math.clamp(value, 0, 1);
			}

			@Override
			public void activeSonarPing() {
				if (activeSonarCooldown <= 0) {
					pingRequested = true;
				}
			}

			@Override
			public void detonate() {
				detonateRequested = true;
			}

			@Override
			public void publishTarget(double x, double y, double z) {
				pubTargetX = x;
				pubTargetY = y;
				pubTargetZ = z;
			}

			@Override
			public void publishDiagnostics(
				double estX, double estY, double estZ, double estHeading, double estSpeed, double intX, double intY,
				double intZ, String phase) {
				diagEstX = estX;
				diagEstY = estY;
				diagEstZ = estZ;
				diagEstHeading = estHeading;
				diagEstSpeed = estSpeed;
				diagIntX = intX;
				diagIntY = intY;
				diagIntZ = intZ;
				diagPhase = phase;
			}
		};
	}

	public double publishedTargetX() {
		return pubTargetX;
	}

	public double publishedTargetY() {
		return pubTargetY;
	}

	public double publishedTargetZ() {
		return pubTargetZ;
	}

	// Guidance diagnostics
	public double diagEstX() {
		return diagEstX;
	}

	public double diagEstY() {
		return diagEstY;
	}

	public double diagEstZ() {
		return diagEstZ;
	}

	public double diagEstHeading() {
		return diagEstHeading;
	}

	public double diagEstSpeed() {
		return diagEstSpeed;
	}

	public double diagIntX() {
		return diagIntX;
	}

	public double diagIntY() {
		return diagIntY;
	}

	public double diagIntZ() {
		return diagIntZ;
	}

	public String diagPhase() {
		return diagPhase;
	}

	public double cmdRudder() {
		return cmdRudder;
	}

	public double cmdSternPlanes() {
		return cmdSternPlanes;
	}

	public double cmdThrottle() {
		return fuelRemaining > 0 ? cmdThrottle : 0;
	}

	public void clearPingRequested() {
		pingRequested = false;
	}

	public void setActiveSonarCooldown(int ticks) {
		activeSonarCooldown = ticks;
	}

	// ── Snapshot and state ─────────────────────────────────────────

	public Pose pose() {
		return new Pose(new Vec3(x, y, z), heading, pitch, 0);
	}

	public Velocity velocity() {
		double vx = speed * Math.sin(heading) * Math.cos(pitch);
		double vy = speed * Math.cos(heading) * Math.cos(pitch);
		double vz = speed * Math.sin(pitch) + verticalSpeed;
		return new Velocity(new Vec3(vx, vy, vz), new Vec3(0, pitchRate, yawRate));
	}

	public TorpedoSnapshot snapshot() {
		return new TorpedoSnapshot(id, ownerId, pose(), velocity(), speed, color, fuelRemaining, detonated, alive,
				sourceLevelDb, pingRequested, pubTargetX, pubTargetY, pubTargetZ, diagEstX, diagEstY, diagEstZ,
				diagEstHeading, diagEstSpeed, diagIntX, diagIntY, diagIntZ, diagPhase);
	}

	// ── Constants ──────────────────────────────────────────────────

	public static double minimumSpeed() {
		return MINIMUM_SPEED;
	}

	public static double sinkAcceleration() {
		return SINK_ACCELERATION;
	}

	public static double defaultFuelSeconds() {
		return DEFAULT_FUEL_SECONDS;
	}
}
