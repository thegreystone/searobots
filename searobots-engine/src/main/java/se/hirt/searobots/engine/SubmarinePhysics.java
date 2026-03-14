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

public final class SubmarinePhysics {

    private static final double MAX_THRUST = 120_000;
    private static final double REVERSE_THRUST_FACTOR = 0.3;  // reverse is 30% of forward
    private static final double MAX_REVERSE_SPEED = 3.0;      // max ~3 m/s in reverse
    private static final double MASS = 8_000;
    private static final double DRAG_COEFF = 530;
    private static final double MAX_YAW_RATE = 0.15;
    private static final double MAX_PITCH_RATE = 0.08;
    private static final double REF_SPEED = 10.0;
    private static final double BALLAST_MAX_VERTICAL_SPEED = 2.0;
    // Ballast slew rate: how fast the physical ballast state can change.
    // Full flood/blow (0→1 or 1→0) takes ~20 seconds.
    private static final double BALLAST_SLEW_RATE = 1.0 / 20.0; // per second
    private static final double TERRAIN_CLEARANCE = 10.0;

    // Hull collision cylinder: the sub can't squeeze through gaps
    // narrower than its beam, and bow/stern check prevents straddling ridges
    private static final double HULL_HALF_LENGTH = 15.0;  // 30m bow to stern
    private static final double HULL_HALF_BEAM = 3.0;     // 6m wide

    // Damage = factor * closingSpeed^2, minimum 1 HP on any contact.
    // closingSpeed is estimated from penetration depth / dt.
    // At max speed (~15 m/s) into a cliff: ~5 * 225 = 1125 HP → instant death.
    // Gentle scrape (~0.5 m/s): ~5 * 0.25 = 1 HP → survivable.
    private static final double COLLISION_DAMAGE_FACTOR = 5.0;
    private static final double BOUNCE_SPEED = 0.5; // upward push after any collision

    // Noise model (dB-based source level)
    private static final double BASE_SL_DB = 80.0;              // machinery noise at dead stop
    private static final double SPEED_NOISE_DB_PER_MS = 2.0;    // +2 dB per m/s speed
    private static final double BASE_CAVITATION_SPEED = 5.0;    // m/s cavitation onset at surface
    private static final double CAVITATION_DEPTH_FACTOR = 0.02; // onset rises 0.02 m/s per meter depth
    private static final double CAVITATION_MAX_DB = 15.0;       // max cavitation penalty
    private static final double REVERSE_CAVITATION_DB = 12.0;   // reverse thrust cavitation
    private static final double SURFACE_NOISE_DEPTH = -50.0;    // noise starts here
    private static final double SURFACE_NOISE_DB = 5.0;         // shallow depth penalty
    private static final double BALLAST_NOISE_DB = 5.0;         // max ballast change noise

    public void step(SubmarineEntity sub, double dt, TerrainMap terrain,
                     CurrentField currentField, BattleArea battleArea) {
        if (sub.forfeited() || sub.hp() <= 0) return;

        // 1. Thrust and drag
        double throttle = sub.throttle();
        double thrust;
        if (throttle >= 0) {
            thrust = MAX_THRUST * throttle;
        } else {
            // Reverse thrust is weaker — props are optimized for forward
            thrust = MAX_THRUST * REVERSE_THRUST_FACTOR * throttle;
        }
        double speed = sub.speed();
        double drag = DRAG_COEFF * speed * Math.abs(speed);
        speed += (thrust - drag) / MASS * dt;
        // Cap reverse speed — hull form creates enormous drag going backwards
        if (speed < -MAX_REVERSE_SPEED) speed = -MAX_REVERSE_SPEED;
        sub.setSpeed(speed);

        // 2. Yaw (rudder)
        // Control surfaces reverse in reverse — water flows from the other direction.
        // Using signed speed preserves this: negative speed flips rudder/plane effect.
        // sqrt relationship: at low speed, turn radius shrinks (more realistic than linear).
        // At REF_SPEED: factor=1.0 (unchanged). At 1 m/s: factor=0.32 (was 0.1 with linear).
        double speedFactor = Math.signum(speed)
                * Math.clamp(Math.sqrt(Math.abs(speed) / REF_SPEED), 0.0, 1.0);
        double yawRate = MAX_YAW_RATE * sub.rudder() * speedFactor;
        double heading = sub.heading() + yawRate * dt;
        // Normalize to [0, 2pi)
        heading = heading % (2 * Math.PI);
        if (heading < 0) heading += 2 * Math.PI;
        sub.setHeading(heading);

        // 3. Pitch (stern planes)
        double pitchRate = MAX_PITCH_RATE * sub.sternPlanes() * speedFactor;
        double pitch = sub.pitch() + pitchRate * dt;
        pitch = Math.clamp(pitch, -Math.PI / 4, Math.PI / 4);
        sub.setPitch(pitch);

        // 4. Ballast: slew actual ballast toward commanded ballast (tanks take time to flood/blow)
        double commanded = sub.ballast();
        double actual = sub.actualBallast();
        double maxChange = BALLAST_SLEW_RATE * dt;
        if (commanded > actual) {
            actual = Math.min(actual + maxChange, commanded);
        } else if (commanded < actual) {
            actual = Math.max(actual - maxChange, commanded);
        }
        sub.setPreviousActualBallast(sub.actualBallast());
        sub.setActualBallast(actual);

        // Vertical speed from physical ballast state (not commanded)
        // 0.5 = neutral, <0.5 = sink, >0.5 = rise
        double ballastVerticalSpeed = (actual - 0.5) * 2.0 * BALLAST_MAX_VERTICAL_SPEED;
        sub.setVerticalSpeed(ballastVerticalSpeed);

        // 5. Position update
        double vx = speed * Math.sin(heading) * Math.cos(pitch);
        double vy = speed * Math.cos(heading) * Math.cos(pitch);
        double vz = speed * Math.sin(pitch) + ballastVerticalSpeed;

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

        // 7. Terrain collision — check hull cylinder (bow, stern, port, starboard, center)
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);
        // perpendicular (port direction): rotate heading 90° left
        double sinP = -cosH;
        double cosP = sinH;

        double floorCenter = terrain.elevationAt(newX, newY);
        double floorBow    = terrain.elevationAt(newX + sinH * HULL_HALF_LENGTH,
                                                  newY + cosH * HULL_HALF_LENGTH);
        double floorStern  = terrain.elevationAt(newX - sinH * HULL_HALF_LENGTH,
                                                  newY - cosH * HULL_HALF_LENGTH);
        double floorPort   = terrain.elevationAt(newX + sinP * HULL_HALF_BEAM,
                                                  newY + cosP * HULL_HALF_BEAM);
        double floorStbd   = terrain.elevationAt(newX - sinP * HULL_HALF_BEAM,
                                                  newY - cosP * HULL_HALF_BEAM);
        double floorZ = Math.max(floorCenter,
                         Math.max(Math.max(floorBow, floorStern),
                                  Math.max(floorPort, floorStbd)));

        double minZ = floorZ + TERRAIN_CLEARANCE;
        if (newZ < minZ) {
            double penetration = minZ - newZ;
            double closingSpeed = penetration / dt;

            int damage = Math.max(1, (int) (COLLISION_DAMAGE_FACTOR * closingSpeed * closingSpeed));
            sub.setHp(Math.max(0, sub.hp() - damage));

            newZ = minZ;
            sub.setVerticalSpeed(BOUNCE_SPEED);
            sub.setPitch(Math.max(sub.pitch(), 0));

            if (closingSpeed > 3.0) {
                sub.setSpeed(speed * 0.5);
            }
        }

        sub.setX(newX);
        sub.setY(newY);
        sub.setZ(newZ);

        // 8. Noise model (dB-based source level)
        double sl = BASE_SL_DB;

        // Speed-dependent: flow noise and propeller noise
        sl += SPEED_NOISE_DB_PER_MS * Math.abs(speed);

        // Depth-dependent cavitation
        // Deeper = higher pressure = cavitation onset at higher speed
        // At -50m: cavitate above 6 m/s. At -200m: above 9 m/s. At -500m: above 15 m/s.
        double cavitationSpeed = BASE_CAVITATION_SPEED + (-newZ) * CAVITATION_DEPTH_FACTOR;
        if (Math.abs(speed) > cavitationSpeed) {
            double excess = (Math.abs(speed) - cavitationSpeed) / cavitationSpeed;
            sl += CAVITATION_MAX_DB * Math.min(excess, 1.0);
        }

        // Reverse thrust cavitation (prop wash turbulence)
        if (sub.throttle() < -0.1) {
            sl += REVERSE_CAVITATION_DB * Math.abs(sub.throttle());
        }

        // Surface proximity noise
        if (newZ > SURFACE_NOISE_DEPTH) {
            sl += SURFACE_NOISE_DB * (1.0 + newZ / (-SURFACE_NOISE_DEPTH));
        }

        // Ballast change noise (flooding/blowing tanks is audible)
        double ballastChangeRate = Math.abs(actual - sub.previousActualBallast()) / dt;
        if (ballastChangeRate > 0.001) {
            sl += BALLAST_NOISE_DB * Math.min(ballastChangeRate / BALLAST_SLEW_RATE, 1.0);
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
