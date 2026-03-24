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
    double addedMassSway,
    double addedMassHeave,
    double maxThrust,
    double reverseThrustFactor,
    double maxReverseSpeed,
    double dragCoeff,
    double swayDragCoeff,
    double hullMomentArm,
    double rudderArea,
    double rudderArm,
    double planesArea,
    double planesArm,
    double stallAngle,
    double rotationalInertia,
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
    // Engine dynamics
    double thrustSlewRate,
    // Vehicle type flags
    boolean surfaceLocked,
    boolean hasBallast
) {
    /** Effective mass in surge (forward/aft) direction. */
    public double massSurge() { return dryMass + addedMassSurge; }

    /** Effective mass in sway (lateral) direction. */
    public double massSway() { return dryMass + addedMassSway; }

    /** Effective mass in heave (vertical) direction. */
    public double massHeave() { return dryMass + addedMassHeave; }

    /**
     * Attack submarine: 75m length, 12m beam, ~2500 tonnes.
     * Max speed 15 m/s (~29 knots). Turn radius ~150-300m at patrol speed.
     */
    public static VehicleConfig submarine() {
        double mass = 2_500_000;         // 2500 tonnes
        double drag = 12_000;            // tuned for 15 m/s max: sqrt(thrust/drag) = 15
        return new VehicleConfig(
            mass,                           // dryMass
            mass / 2,                       // addedMassSurge (50% for streamlined hull)
            mass * 1.5,                     // addedMassSway (blunt cross-section)
            mass * 1.25,                    // addedMassHeave
            drag * 15 * 15,                 // maxThrust: drag * v_max^2 = 2,700,000
            0.3,                            // reverseThrustFactor
            3.0,                            // maxReverseSpeed
            drag,                           // dragCoeff
            drag * 3,                       // swayDragCoeff (lateral drag ~3x surge)
            6.0,                            // hullMomentArm (CG to lateral center of pressure)
            6.0,                            // rudderArea (m^2)
            30.0,                           // rudderArm (rudder near stern of 75m hull)
            5.0,                            // planesArea (m^2)
            30.0,                           // planesArm (stern planes near rudder)
            Math.toRadians(25),             // stallAngle
            15.0,                           // rotationalInertia (low base = tight turns at low speed)
            1.0 / 5.0,                     // ballastSlewRate (full blow in ~2.5s from neutral)
            0.03 * mass * 9.81,            // ballastForceMax
            0.5 * 1025 * 1.0 * 520 * 1.5, // verticalDragCoeff (scaled for hull)
            12.0,                           // terrainClearance
            37.5,                           // hullHalfLength (75m / 2)
            6.0,                            // hullHalfBeam (12m / 2)
            5.0,                            // collisionDamageFactor
            0.5,                            // bounceSpeed
            0.5,                            // propDragFactor
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
            0.25,                       // thrustSlewRate (full power in 4 seconds)
            false,                      // surfaceLocked
            true                        // hasBallast
        );
    }

    /**
     * Torpedo: 5m length, 0.5m diameter, ~300 kg.
     * Max speed ~20 m/s (~39 knots). Highly maneuverable at speed,
     * loses all control authority below ~3 m/s. Slightly negatively
     * buoyant: relies on hydrodynamic lift to maintain depth.
     * Very loud (~115 dB base), making passive sonar nearly useless
     * but trivially detectable by targets.
     */
    public static VehicleConfig torpedo() {
        double mass = 300;               // 300 kg
        // Low drag for streamlined body: coasts for ~30-40s after fuel runs out.
        // v_max = sqrt(thrust/drag) = 23 m/s (~45 knots).
        double drag = 3.0;              // very low drag, torpedo-shaped
        double vMax = 25.0;             // ~49 knots
        return new VehicleConfig(
            mass,                           // dryMass
            100,                            // addedMassSurge (slender body)
            mass * 1.0,                     // addedMassSway
            mass * 0.8,                     // addedMassHeave
            drag * vMax * vMax,             // maxThrust: drag * v_max^2
            0.0,                            // reverseThrustFactor (no reverse)
            0.0,                            // maxReverseSpeed
            drag,                           // dragCoeff
            drag * 5,                       // swayDragCoeff
            0.5,                            // hullMomentArm (short body)
            0.05,                           // rudderArea (m^2, very small fins)
            2.0,                            // rudderArm (fins near tail of 5m body)
            0.05,                           // planesArea (m^2, very small fins)
            2.0,                            // planesArm
            Math.toRadians(30),             // stallAngle
            8.0,                            // rotationalInertia (high: long narrow body resists turning)
            0,                              // ballastSlewRate (no ballast)
            0,                              // ballastForceMax
            0.5 * 1025 * 0.1 * 0.2 * 1.5,  // verticalDragCoeff (small cross-section)
            1.0,                            // terrainClearance (torpedo is small)
            2.5,                            // hullHalfLength (5m / 2)
            0.25,                           // hullHalfBeam (0.5m / 2)
            10.0,                           // collisionDamageFactor (torpedo is fragile)
            0.0,                            // bounceSpeed (destroyed on terrain hit)
            0.0,                            // propDragFactor
            115.0,                      // baseSlDb (very loud propulsion)
            5.0,                        // clutchDisengagedSlReduction
            1.0,                        // speedNoiseDbPerMs
            8.0,                        // baseCavitationSpeed
            0.01,                       // cavitationDepthFactor
            10.0,                       // cavitationMaxDb
            5.0,                        // reverseCavitationDb
            -30.0,                      // surfaceNoiseDepth
            3.0,                        // surfaceNoiseDb
            0,                          // ballastNoiseDb
            0.5,                        // thrustSlewRate (fast spool-up)
            false,                      // surfaceLocked
            false                       // hasBallast
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
            shipMass * 1.5,             // addedMassSway
            shipMass * 1.0,             // addedMassHeave (not used, surface locked)
            shipThrust,
            0.2,                        // reverseThrustFactor
            2.0,                        // maxReverseSpeed
            shipDrag,
            shipDrag * 8,               // swayDragCoeff (lateral ~8x surge for wide hull)
            20.0,                       // hullMomentArm (CG to lateral center of pressure)
            8.0,                        // rudderArea (m^2, larger ship)
            25.0,                       // rudderArm (m from CG)
            0.0,                        // planesArea (no pitch control)
            0.0,                        // planesArm
            Math.toRadians(16),         // stallAngle
            17.5,                       // rotationalInertia
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
            0.5,                        // thrustSlewRate (full power in 2 seconds, bigger engines)
            true,                       // surfaceLocked
            false                       // hasBallast
        );
    }
}
