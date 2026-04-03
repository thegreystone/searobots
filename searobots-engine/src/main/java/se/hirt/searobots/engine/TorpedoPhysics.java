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

import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.TerrainMap;

/**
 * Physics model for torpedoes. Simplified relative to submarine physics:
 * no ballast, no clutch, no reverse thrust. Torpedoes are slightly negatively
 * buoyant and rely on hydrodynamic lift from forward motion to maintain depth.
 * Below minimum speed (~3 m/s), lift fails and the torpedo sinks.
 */
public final class TorpedoPhysics {

    private static final double WATER_DENSITY = 1025.0; // kg/m^3
    private static final double NEGATIVE_BUOYANCY = 30.0; // N downward (slight)
    private static final double LIFT_COEFFICIENT = 0.3; // lift per m/s^2 at depth

    /**
     * Lift coefficient as a function of angle of attack, with stall.
     * Same formula as SubmarinePhysics.
     */
    private static double liftCoefficient(double alpha, double stallAngle) {
        if (Math.abs(alpha) <= stallAngle) {
            return 2 * Math.PI * Math.sin(alpha);
        }
        double sign = Math.signum(alpha);
        double postStall = 0.5 * Math.sin(2 * alpha);
        return sign * Math.max(Math.abs(postStall), 0.1);
    }

    /**
     * Steps the torpedo physics by one tick.
     *
     * @param torp torpedo entity to update
     * @param dt time step in seconds
     * @param terrain terrain map for collision checking
     * @param currentField ocean currents
     * @param battleArea battle area boundary
     */
    public void step(TorpedoEntity torp, double dt, TerrainMap terrain,
                     CurrentField currentField, BattleArea battleArea) {
        if (!torp.alive()) return;

        var cfg = torp.vehicleConfig();

        // 1. Fuel consumption
        double cmdThrottle = torp.cmdThrottle(); // already 0 if no fuel
        if (cmdThrottle > 0 && torp.fuelRemaining() > 0) {
            torp.consumeFuel(dt);
        }
        if (torp.fuelRemaining() <= 0) {
            cmdThrottle = 0;
        }

        // 2. Throttle slew
        double actualThrottle = torp.actualThrottle();
        double maxThrottleChange = cfg.thrustSlewRate() * dt;
        if (cmdThrottle > actualThrottle) {
            actualThrottle = Math.min(actualThrottle + maxThrottleChange, cmdThrottle);
        } else {
            actualThrottle = Math.max(actualThrottle - maxThrottleChange * 2, cmdThrottle); // fast decay
        }
        torp.setActualThrottle(actualThrottle);

        // 3. Thrust and drag
        double thrust = cfg.maxThrust() * actualThrottle;
        double speed = torp.speed();
        double drag = cfg.dragCoeff() * speed * Math.abs(speed);
        speed += (thrust - drag) / cfg.massSurge() * dt;
        if (speed < 0) speed = 0; // no reverse
        torp.setSpeed(speed);

        // 4. Control surface slew (faster than submarine: small fins, less inertia)
        double controlSlewRate = 1.5; // faster than sub's 0.57
        double maxControlChange = controlSlewRate * dt;

        double actualRudder = torp.actualRudder();
        double cmdRudder = torp.cmdRudder();
        if (cmdRudder > actualRudder) {
            actualRudder = Math.min(actualRudder + maxControlChange, cmdRudder);
        } else {
            actualRudder = Math.max(actualRudder - maxControlChange, cmdRudder);
        }
        torp.setActualRudder(actualRudder);

        double actualPlanes = torp.actualSternPlanes();
        double cmdPlanes = torp.cmdSternPlanes();
        if (cmdPlanes > actualPlanes) {
            actualPlanes = Math.min(actualPlanes + maxControlChange, cmdPlanes);
        } else {
            actualPlanes = Math.max(actualPlanes - maxControlChange, cmdPlanes);
        }
        torp.setActualSternPlanes(actualPlanes);

        // 5. Yaw dynamics (same first-order model as submarine, different coefficients)
        double rudderAngle = actualRudder * Math.PI / 4;
        double rudderCl = liftCoefficient(rudderAngle, cfg.stallAngle());
        double rudderMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed)
                * cfg.rudderArea() * rudderCl * cfg.rudderArm();

        double baseInertia = cfg.massSurge() * cfg.rotationalInertia();
        // Torpedo: long narrow body generates enormous rotational resistance at speed.
        // Speed damping grows with v^2, making the torpedo progressively less
        // maneuverable at higher speeds. At 20 m/s the turn radius is ~200m;
        // at 5 m/s it's much tighter (~30m) but control surfaces are also weaker.
        double speedDampingCoeff = 0.4; // higher than sub's 0.05, but allows reasonable turns
        double speedDamping = baseInertia * speedDampingCoeff * speed * Math.abs(speed);
        double effectiveInertia = baseInertia + speedDamping;

        double yawRateSteady = effectiveInertia > 0 ? rudderMoment / effectiveInertia : 0;

        double absSpeed = Math.max(speed, 0.5);
        double tau = effectiveInertia / (cfg.swayDragCoeff() * cfg.hullMomentArm() * absSpeed);
        tau = Math.clamp(tau, 0.5, 10.0); // torpedoes respond faster

        double yawRate = torp.yawRate();
        yawRate += (yawRateSteady - yawRate) * (1.0 - Math.exp(-dt / tau));
        torp.setYawRate(yawRate);

        double heading = torp.heading() + yawRate * dt;
        heading = heading % (2 * Math.PI);
        if (heading < 0) heading += 2 * Math.PI;
        torp.setHeading(heading);

        // 6. Pitch dynamics
        if (cfg.planesArea() > 0) {
            double planesAngle = actualPlanes * Math.PI / 4;
            double planesCl = liftCoefficient(planesAngle, cfg.stallAngle());
            double pitchMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed)
                    * cfg.planesArea() * planesCl * cfg.planesArm();

            // Torpedoes should answer depth commands faster than they answer yaw.
            // Keeping pitch damping as high as yaw made deep targets effectively
            // unreachable before the weapon had already overrun them.
            double pitchDampingCoeff = 0.18;
            double pitchSpeedDamping = baseInertia * pitchDampingCoeff * speed * Math.abs(speed);
            double pitchInertia = baseInertia * 1.5 + pitchSpeedDamping;
            double pitchRateSteady = pitchInertia > 0 ? pitchMoment / pitchInertia : 0;
            double pitchTau = Math.clamp(pitchInertia / (cfg.swayDragCoeff() * absSpeed), 1.2, 6.0);

            double pitchRate = torp.pitchRate();
            pitchRate += (pitchRateSteady - pitchRate) * (1.0 - Math.exp(-dt / pitchTau));
            torp.setPitchRate(pitchRate);

            double pitch = torp.pitch() + pitchRate * dt;
            pitch = Math.clamp(pitch, -Math.PI / 3, Math.PI / 3); // max 60 deg
            torp.setPitch(pitch);
        }

        // 7. Buoyancy and lift
        double verticalSpeed = torp.verticalSpeed();

        if (speed >= TorpedoEntity.minimumSpeed()) {
            // Sufficient speed: hydrodynamic lift counteracts negative buoyancy
            // Lift proportional to speed^2, counters the constant negative buoyancy
            double liftForce = LIFT_COEFFICIENT * speed * speed;
            double netVertical = (-NEGATIVE_BUOYANCY + liftForce) / cfg.massHeave();
            // Damp toward zero: lift maintains depth when pitch controls are neutral
            verticalSpeed += netVertical * dt;
            verticalSpeed *= Math.exp(-2.0 * dt); // vertical damping
        } else {
            // Below minimum speed: lift fails, torpedo sinks
            verticalSpeed -= TorpedoEntity.sinkAcceleration() * dt;
        }

        // Vertical drag
        double vDrag = cfg.verticalDragCoeff() * verticalSpeed * Math.abs(verticalSpeed);
        verticalSpeed -= vDrag / cfg.massHeave() * dt;
        torp.setVerticalSpeed(verticalSpeed);

        // 8. Position update
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        double cosP = Math.cos(torp.pitch());
        double sinP = Math.sin(torp.pitch());

        double vx = speed * sinH * cosP;
        double vy = speed * cosH * cosP;
        double vz = speed * sinP + verticalSpeed;

        // Apply ocean current
        double depth = torp.z();
        if (currentField != null) {
            var current = currentField.currentAt(depth);
            vx += current.x();
            vy += current.y();
        }

        double newX = torp.x() + vx * dt;
        double newY = torp.y() + vy * dt;
        double newZ = torp.z() + vz * dt;

        // 9. Surface clamp
        if (newZ > 0) {
            newZ = 0;
            torp.setVerticalSpeed(0);
        }

        // 10. Terrain collision: torpedo DETONATES on impact (can damage nearby subs)
        if (terrain != null) {
            double floor = terrain.elevationAt(newX, newY);
            if (newZ < floor + cfg.terrainClearance()) {
                torp.detonate(); // detonate, not just kill
                torp.setX(newX);
                torp.setY(newY);
                torp.setZ(floor + cfg.terrainClearance());
                return;
            }
        }

        // 11. Battle area: torpedo destroyed if outside
        if (battleArea != null && !battleArea.contains(newX, newY)) {
            torp.kill();
        }

        torp.setX(newX);
        torp.setY(newY);
        torp.setZ(newZ);

        // 12. Noise model (simplified: loud constant base + speed contribution)
        double baseNoise = cfg.baseSlDb();
        double speedNoise = cfg.speedNoiseDbPerMs() * speed;
        torp.setSourceLevelDb(baseNoise + speedNoise);
    }
}
