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
package se.hirt.searobots.api;

/**
 * Data-driven vehicle configuration. All physics constants that were previously
 * hardcoded in SubmarinePhysics are now per-entity parameters, enabling
 * submarines, surface ships, and (future) torpedoes to share the same physics
 * engine with different tuning.
 */
public record VehicleConfig(
    double dryMass,
    double addedMassSurge,
    double addedMassHeave,
    double maxThrust,
    double reverseThrustFactor,
    double maxReverseSpeed,
    double dragCoeff,
    double maxYawRate,
    double maxPitchRate,
    double refSpeed,
    double ballastSlewRate,
    double ballastForceMax,
    double verticalDragCoeff,
    double terrainClearance,
    double hullHalfLength,
    double hullHalfBeam,
    double collisionDamageFactor,
    double bounceSpeed,
    double propDragFactor,
    // Noise model
    double baseSlDb,
    double clutchDisengagedSlReduction,
    double speedNoiseDbPerMs,
    double baseCavitationSpeed,
    double cavitationDepthFactor,
    double cavitationMaxDb,
    double reverseCavitationDb,
    double surfaceNoiseDepth,
    double surfaceNoiseDb,
    double ballastNoiseDb,
    // Vehicle type flags
    boolean surfaceLocked,
    boolean hasBallast
) {
    /** Effective mass in surge (forward/aft) direction. */
    public double massSurge() { return dryMass + addedMassSurge; }

    /** Effective mass in heave (vertical) direction. */
    public double massHeave() { return dryMass + addedMassHeave; }

    /** Standard submarine configuration (reproduces the original SubmarinePhysics constants exactly). */
    public static VehicleConfig submarine() {
        return new VehicleConfig(
            700_000,                    // dryMass
            350_000,                    // addedMassSurge
            700_000 * 1.25,             // addedMassHeave
            870_000,                    // maxThrust
            0.3,                        // reverseThrustFactor
            3.0,                        // maxReverseSpeed
            3_864,                      // dragCoeff
            0.15,                       // maxYawRate
            0.08,                       // maxPitchRate
            10.0,                       // refSpeed
            1.0 / 20.0,                // ballastSlewRate
            0.03 * 700_000 * 9.81,     // ballastForceMax
            0.5 * 1025 * 1.0 * 520,    // verticalDragCoeff
            10.0,                       // terrainClearance
            32.5,                       // hullHalfLength
            4.0,                        // hullHalfBeam
            5.0,                        // collisionDamageFactor
            0.5,                        // bounceSpeed
            0.5,                        // propDragFactor
            80.0,                       // baseSlDb
            15.0,                       // clutchDisengagedSlReduction
            2.0,                        // speedNoiseDbPerMs
            5.0,                        // baseCavitationSpeed
            0.02,                       // cavitationDepthFactor
            15.0,                       // cavitationMaxDb
            12.0,                       // reverseCavitationDb
            -50.0,                      // surfaceNoiseDepth
            5.0,                        // surfaceNoiseDb
            5.0,                        // ballastNoiseDb
            false,                      // surfaceLocked
            true                        // hasBallast
        );
    }

    /** Large noisy transport ship, max ~8 m/s, surface locked. */
    public static VehicleConfig surfaceShip() {
        double shipMass = 5_000_000;
        double shipDrag = 0.5 * 1025 * 0.2 * Math.PI * 7 * 7;
        double shipThrust = shipDrag * 8 * 8;
        return new VehicleConfig(
            shipMass,
            shipMass * 0.3,             // addedMassSurge
            shipMass * 1.0,             // addedMassHeave (not used, surface locked)
            shipThrust,
            0.2,                        // reverseThrustFactor
            2.0,                        // maxReverseSpeed
            shipDrag,
            0.10,                       // maxYawRate (slower turning)
            0.0,                        // maxPitchRate (no pitch control)
            8.0,                        // refSpeed
            0,                          // ballastSlewRate (no ballast)
            0,                          // ballastForceMax
            0,                          // verticalDragCoeff (not used)
            5.0,                        // terrainClearance (shallow draft)
            75.0,                       // hullHalfLength (150m ship)
            15.0,                       // hullHalfBeam (30m beam)
            5.0,                        // collisionDamageFactor
            0.5,                        // bounceSpeed
            0.5,                        // propDragFactor
            100.0,                      // baseSlDb (very loud machinery)
            10.0,                       // clutchDisengagedSlReduction
            3.0,                        // speedNoiseDbPerMs (louder per m/s)
            3.0,                        // baseCavitationSpeed (surface props cavitate easily)
            0.01,                       // cavitationDepthFactor
            20.0,                       // cavitationMaxDb
            15.0,                       // reverseCavitationDb
            -10.0,                      // surfaceNoiseDepth
            8.0,                        // surfaceNoiseDb (always at surface)
            0,                          // ballastNoiseDb
            true,                       // surfaceLocked
            false                       // hasBallast
        );
    }
}
