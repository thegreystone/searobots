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

import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.BattleArea;
import se.hirt.searobots.api.VehicleConfig;

public final class SubmarinePhysics {

    private static final double WATER_DENSITY = 1025.0; // kg/m^3 seawater
    private static final double CL_SLOPE = 2 * Math.PI; // thin airfoil theory

    /**
     * Lift coefficient with stall. Linear below stallAngle, smooth
     * rolloff above (drops to ~60% at full deflection).
     */
    static double liftCoefficient(double alpha, double stallAngle) {
        double absAlpha = Math.abs(alpha);
        double cl;
        if (absAlpha <= stallAngle) {
            cl = CL_SLOPE * absAlpha;
        } else {
            double clMax = CL_SLOPE * stallAngle;
            double maxDeflection = Math.PI / 4;
            double postStallFraction = (absAlpha - stallAngle) / (maxDeflection - stallAngle);
            cl = clMax * (1.0 - 0.4 * Math.min(postStallFraction, 1.0));
        }
        return Math.copySign(cl, alpha);
    }

    public void step(SubmarineEntity sub, double dt, TerrainMap terrain,
                     CurrentField currentField, BattleArea battleArea) {
        if (sub.forfeited() || sub.hp() <= 0) return;

        var cfg = sub.vehicleConfig();

        // 1. Thrust lag: actual throttle tracks commanded with slew limit
        double commandedThrottle = sub.throttle();
        double actualThrottle = sub.actualThrottle();
        double maxThrottleChange = cfg.thrustSlewRate() * dt;
        if (commandedThrottle > actualThrottle) {
            actualThrottle = Math.min(actualThrottle + maxThrottleChange, commandedThrottle);
        } else if (commandedThrottle < actualThrottle) {
            actualThrottle = Math.max(actualThrottle - maxThrottleChange, commandedThrottle);
        }
        sub.setActualThrottle(actualThrottle);

        // 2. Thrust and drag (with engine clutch mechanic)
        boolean clutchEngaged = sub.engineClutch();
        double thrust;
        if (!clutchEngaged) {
            // Clutch disengaged: prop freewheels, no thrust, no engine braking
            thrust = 0;
        } else if (actualThrottle >= 0) {
            thrust = cfg.maxThrust() * actualThrottle;
        } else {
            // Reverse thrust is weaker; props are optimized for forward
            thrust = cfg.maxThrust() * cfg.reverseThrustFactor() * actualThrottle;
        }
        double speed = sub.speed();
        double drag = cfg.dragCoeff() * speed * Math.abs(speed);
        // Extra drag from windmilling prop when clutch engaged at zero throttle
        if (clutchEngaged && actualThrottle == 0) {
            drag += cfg.dragCoeff() * cfg.propDragFactor() * speed * Math.abs(speed);
        }
        speed += (thrust - drag) / cfg.massSurge() * dt;
        // Cap reverse speed; hull form creates enormous drag going backwards
        if (speed < -cfg.maxReverseSpeed()) speed = -cfg.maxReverseSpeed();
        sub.setSpeed(speed);

        // 3. Yaw with sway coupling
        // Rudder produces yaw moment. Hull lateral drag opposes turning.
        double rudderAngle = sub.rudder() * Math.PI / 4;  // -1..1 maps to -45..+45 deg
        double rudderCl = liftCoefficient(rudderAngle, cfg.stallAngle());
        double rudderMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed)
                * cfg.rudderArea() * rudderCl * cfg.rudderArm();

        // Sway dynamics: turning creates centripetal sway, hull drag damps it
        double swaySpeed = sub.swaySpeed();
        double yawRate = sub.yawRate();

        // Net yaw moment = rudder moment - hull lateral drag moment
        // Hull lateral drag acts at hullMomentArm ahead of CG, creating a restoring moment
        double swayDrag = cfg.swayDragCoeff() * swaySpeed * Math.abs(swaySpeed);
        double hullDragMoment = swayDrag * cfg.hullMomentArm();

        double netYawMoment = rudderMoment - hullDragMoment;
        double yawInertia = cfg.massSurge() * cfg.rotationalInertia();
        double yawAccel = netYawMoment / yawInertia;
        yawRate += yawAccel * dt;
        sub.setYawRate(yawRate);

        double heading = sub.heading() + yawRate * dt;
        heading = heading % (2 * Math.PI);
        if (heading < 0) heading += 2 * Math.PI;
        sub.setHeading(heading);

        // Update sway: centripetal force from turning, minus lateral drag
        double centripetalForce = cfg.massSurge() * speed * yawRate;
        swaySpeed += (centripetalForce - swayDrag) / cfg.massSway() * dt;
        sub.setSwaySpeed(swaySpeed);

        // 3. Pitch (stern planes): lift-based model with stall
        if (!cfg.surfaceLocked() && cfg.planesArea() > 0) {
            double planesAngle = sub.sternPlanes() * Math.PI / 4;  // -1..1 maps to -45..+45 deg
            double planesCl = liftCoefficient(planesAngle, cfg.stallAngle());
            double pitchMoment = 0.5 * WATER_DENSITY * speed * Math.abs(speed)
                    * cfg.planesArea() * planesCl * cfg.planesArm();
            double pitchAccel = pitchMoment / (cfg.massHeave() * cfg.rotationalInertia());
            double pitchRate = sub.pitchRate() + pitchAccel * dt;
            sub.setPitchRate(pitchRate);
            double pitch = sub.pitch() + pitchRate * dt;
            pitch = Math.clamp(pitch, -Math.PI / 4, Math.PI / 4);
            sub.setPitch(pitch);
        }

        // 4. Ballast: slew actual ballast toward commanded ballast (tanks take time to flood/blow)
        double actual = sub.actualBallast();
        double buoyancyForce = 0;
        if (cfg.hasBallast()) {
            double commanded = sub.ballast();
            double maxChange = cfg.ballastSlewRate() * dt;
            if (commanded > actual) {
                actual = Math.min(actual + maxChange, commanded);
            } else if (commanded < actual) {
                actual = Math.max(actual - maxChange, commanded);
            }
            sub.setPreviousActualBallast(sub.actualBallast());
            sub.setActualBallast(actual);

            // Buoyancy force from ballast: 0.5 = neutral, <0.5 = heavy (sink), >0.5 = light (rise)
            buoyancyForce = (actual - 0.5) * 2.0 * cfg.ballastForceMax();
        }

        // Current vertical speed
        double verticalSpeed = sub.verticalSpeed();

        if (!cfg.surfaceLocked()) {
            // Vertical drag (proportional to v^2, large cross-section)
            double verticalDrag = cfg.verticalDragCoeff() * verticalSpeed * Math.abs(verticalSpeed);

            // Vertical acceleration: buoyancy force minus drag, divided by heave mass
            verticalSpeed += (buoyancyForce - verticalDrag) / cfg.massHeave() * dt;
            sub.setVerticalSpeed(verticalSpeed);
        } else {
            verticalSpeed = 0;
            sub.setVerticalSpeed(0);
        }

        // 5. Position update (surge + sway components)
        double pitch = sub.pitch();
        // Surge: along heading
        double vx = speed * Math.sin(heading) * Math.cos(pitch);
        double vy = speed * Math.cos(heading) * Math.cos(pitch);
        // Sway: perpendicular to heading (positive = starboard)
        double swayX = swaySpeed * Math.cos(heading);
        double swayY = -swaySpeed * Math.sin(heading);
        vx += swayX;
        vy += swayY;
        double vz = speed * Math.sin(pitch) + verticalSpeed;

        // Apply current
        var current = currentField.currentAt(sub.z());
        vx += current.x();
        vy += current.y();

        double newX = sub.x() + vx * dt;
        double newY = sub.y() + vy * dt;
        double newZ = sub.z() + vz * dt;

        // 6. Clamp: can't go above water
        if (newZ > 0) {
            newZ = 0;
        }

        // For surfaceLocked vehicles, force z=0 and zero vertical speed
        if (cfg.surfaceLocked()) {
            newZ = 0;
            sub.setVerticalSpeed(0);
        }

        // 7. Terrain collision: check hull cylinder (bow, stern, port, starboard, center)
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        // perpendicular (port direction): rotate heading 90 deg left
        double sinP = -cosH;
        double cosP = sinH;

        double floorCenter = terrain.elevationAt(newX, newY);
        double floorBow    = terrain.elevationAt(newX + sinH * cfg.hullHalfLength(),
                                                  newY + cosH * cfg.hullHalfLength());
        double floorStern  = terrain.elevationAt(newX - sinH * cfg.hullHalfLength(),
                                                  newY - cosH * cfg.hullHalfLength());
        double floorPort   = terrain.elevationAt(newX + sinP * cfg.hullHalfBeam(),
                                                  newY + cosP * cfg.hullHalfBeam());
        double floorStbd   = terrain.elevationAt(newX - sinP * cfg.hullHalfBeam(),
                                                  newY - cosP * cfg.hullHalfBeam());
        double floorZ = Math.max(floorCenter,
                         Math.max(Math.max(floorBow, floorStern),
                                  Math.max(floorPort, floorStbd)));

        double minZ = floorZ + cfg.terrainClearance();
        boolean scraping = false;
        if (newZ < minZ) {
            scraping = true;
            double penetration = minZ - newZ;
            double closingSpeed = penetration / dt;

            int damage = Math.max(1, (int) (cfg.collisionDamageFactor() * closingSpeed * closingSpeed));
            sub.setHp(Math.max(0, sub.hp() - damage));

            newZ = minZ;
            sub.setVerticalSpeed(cfg.bounceSpeed());
            sub.setPitch(Math.max(sub.pitch(), 0));

            // Terrain friction: scraping absorbs energy proportional to speed
            // Hard impact halves speed; continuous scraping bleeds 5% per tick
            if (closingSpeed > 3.0) {
                sub.setSpeed(speed * 0.5);
            } else {
                sub.setSpeed(speed * 0.95);
            }
        }

        sub.setX(newX);
        sub.setY(newY);
        sub.setZ(newZ);

        // 8. Noise model (dB-based source level)
        double sl = cfg.baseSlDb();

        // When clutch is disengaged and throttle is zero, machinery noise drops
        if (!clutchEngaged && actualThrottle == 0) {
            sl -= cfg.clutchDisengagedSlReduction();
        }

        // Speed-dependent: flow noise and propeller noise
        sl += cfg.speedNoiseDbPerMs() * Math.abs(speed);

        // Depth-dependent cavitation
        // Deeper = higher pressure = cavitation onset at higher speed
        // At -50m: cavitate above 6 m/s. At -200m: above 9 m/s. At -500m: above 15 m/s.
        double cavitationSpeed = cfg.baseCavitationSpeed() + (-newZ) * cfg.cavitationDepthFactor();
        if (Math.abs(speed) > cavitationSpeed) {
            double excess = (Math.abs(speed) - cavitationSpeed) / cavitationSpeed;
            sl += cfg.cavitationMaxDb() * Math.min(excess, 1.0);
        }

        // Reverse thrust cavitation (prop wash turbulence)
        if (actualThrottle < -0.1) {
            sl += cfg.reverseCavitationDb() * Math.abs(actualThrottle);
        }

        // Hull scraping noise (metal on rock is extremely loud)
        if (scraping && Math.abs(speed) > 0.5) {
            sl += 20.0 * Math.min(Math.abs(speed) / 5.0, 1.0);
        }

        // Surface proximity noise
        if (newZ > cfg.surfaceNoiseDepth()) {
            sl += cfg.surfaceNoiseDb() * (1.0 + newZ / (-cfg.surfaceNoiseDepth()));
        }

        // Ballast change noise (flooding/blowing tanks is audible)
        if (cfg.hasBallast()) {
            double ballastChangeRate = Math.abs(actual - sub.previousActualBallast()) / dt;
            if (ballastChangeRate > 0.001) {
                sl += cfg.ballastNoiseDb() * Math.min(ballastChangeRate / cfg.ballastSlewRate(), 1.0);
            }
        }

        sub.setSourceLevelDb(sl);

        // Also set linear noise level for viewer compatibility (80 dB = 1.0)
        sub.setNoiseLevel(Math.pow(10, (sl - 80) / 20.0));

        // 9. Battle area check
        if (!battleArea.contains(newX, newY)) {
            sub.setForfeited(true);
        }
    }
}
