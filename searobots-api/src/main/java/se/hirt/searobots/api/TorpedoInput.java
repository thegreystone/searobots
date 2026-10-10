/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.api;

import java.util.List;

/**
 * Input provided to a torpedo controller each tick. The torpedo knows its own state and has its own
 * sonar contacts, but receives no information from the launching submarine.
 */
public interface TorpedoInput {
	/**
	 * Current simulation tick.
	 */
	long tick();

	/**
	 * Time delta in seconds (typically 1/50 = 0.02s).
	 */
	double deltaTimeSeconds();

	/**
	 * Torpedo's current pose (position, heading, pitch).
	 */
	Pose self();

	/**
	 * Torpedo's velocity relative to the water, in world axes, and pose-angle rates. Use
	 * {@link #groundVelocity()} for linear motion over the seabed, including current drift.
	 */
	Velocity velocity();

	/**
	 * Linear velocity over the seabed in world (x, y, z) axes, in m/s. The simulation includes the
	 * ocean current at the torpedo's depth. The default assumes still water.
	 */
	default Vec3 groundVelocity() {
		return velocity().linear();
	}

	/**
	 * Torpedo's current forward speed relative to the water, in m/s.
	 */
	double speed();

	/**
	 * Remaining fuel in seconds. When zero, the torpedo loses thrust.
	 */
	double fuelRemaining();

	/**
	 * Passive sonar detections (bearing, signal strength, estimated range).
	 */
	default List<SonarContact> sonarContacts() {
		return List.of();
	}

	/**
	 * Active sonar returns (bearing + range from own ping).
	 */
	default List<SonarContact> activeSonarReturns() {
		return List.of();
	}

	/**
	 * Ticks remaining before the next active sonar ping is allowed.
	 */
	default int activeSonarCooldownTicks() {
		return 0;
	}
}
