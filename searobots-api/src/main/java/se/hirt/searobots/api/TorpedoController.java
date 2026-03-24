/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.api;

/**
 * Controller for an autonomous torpedo. After launch, the torpedo operates
 * entirely on its own -- no communication with the launching submarine.
 * The controller receives sonar contacts and controls the torpedo's
 * rudder, stern planes, throttle, active sonar, and detonation.
 */
public interface TorpedoController {
    /**
     * Called once when the torpedo is launched. The context includes
     * the launch position, heading, terrain, and the mission data
     * string provided by the launching submarine.
     */
    void onLaunch(TorpedoLaunchContext context);

    /**
     * Called every simulation tick. The torpedo receives its own
     * sonar contacts (not the submarine's) and controls its actuators.
     */
    void onTick(TorpedoInput input, TorpedoOutput output);
}
