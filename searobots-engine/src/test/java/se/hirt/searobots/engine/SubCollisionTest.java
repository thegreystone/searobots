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

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import java.awt.*;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static se.hirt.searobots.api.VehicleConfig.submarine;

class SubCollisionTest {

    private SubmarineEntity makeSub(int id, Vec3 pos, double speed, double heading) {
        var controller = new DefaultAttackSub();
        var sub = new SubmarineEntity(submarine(), id, controller, pos, heading, Color.RED, 1000);
        sub.setSpeed(speed);
        return sub;
    }

    @Test
    void headOnRamIsMutuallyFatal() {
        // Two subs 20m apart, heading toward each other at 10 m/s each
        // Closing speed = 20 m/s → damage = 5 * 400 = 2000 → both dead
        var sub1 = makeSub(0, new Vec3(0, -10, -200), 10, 0);      // heading north
        var sub2 = makeSub(1, new Vec3(0, 10, -200), 10, Math.PI);  // heading south

        SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

        assertTrue(sub1.hp() <= 0, "Sub1 should be dead from head-on ram, hp=" + sub1.hp());
        assertTrue(sub2.hp() <= 0, "Sub2 should be dead from head-on ram, hp=" + sub2.hp());
    }

    @Test
    void glancingBlowIsSurvivable() {
        // Two subs close together, slow closing speed (~2 m/s)
        // damage = 5 * 4 = 20 HP
        var sub1 = makeSub(0, new Vec3(0, -10, -200), 2, 0);
        var sub2 = makeSub(1, new Vec3(0, 10, -200), 0, 0);  // stationary

        SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

        assertTrue(sub1.hp() > 0 && sub1.hp() < 1000,
                "Sub1 should survive glancing blow, hp=" + sub1.hp());
        assertTrue(sub2.hp() > 0 && sub2.hp() < 1000,
                "Sub2 should survive glancing blow, hp=" + sub2.hp());
    }

    @Test
    void stationarySubTakesLessDamage() {
        // Rammed while stopped: closing speed is only one sub's speed
        // At 5 m/s: damage = 5 * 25 = 125 → survivable
        var ramming = makeSub(0, new Vec3(0, -10, -200), 5, 0);
        var stationary = makeSub(1, new Vec3(0, 10, -200), 0, 0);

        SimulationLoop.checkSubCollisions(List.of(ramming, stationary));

        assertTrue(ramming.hp() > 0, "Ramming sub should survive at 5 m/s");
        assertTrue(stationary.hp() > 0, "Stationary sub should survive at 5 m/s");
        // Both take same damage (mutual)
        assertEquals(ramming.hp(), stationary.hp(),
                "Both subs should take equal damage from collision");
    }

    @Test
    void subsBouncedApartAfterCollision() {
        var sub1 = makeSub(0, new Vec3(0, -5, -200), 5, 0);
        var sub2 = makeSub(1, new Vec3(0, 5, -200), 5, Math.PI);

        double distBefore = new Vec3(sub1.x(), sub1.y(), sub1.z())
                .distanceTo(new Vec3(sub2.x(), sub2.y(), sub2.z()));

        SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

        double distAfter = new Vec3(sub1.x(), sub1.y(), sub1.z())
                .distanceTo(new Vec3(sub2.x(), sub2.y(), sub2.z()));

        assertTrue(distAfter > distBefore,
                "Subs should be pushed apart: before=" + distBefore + " after=" + distAfter);
    }

    @Test
    void noCollisionWhenFarApart() {
        // Subs 100m apart, well beyond collision radius (32.5m)
        var sub1 = makeSub(0, new Vec3(0, -50, -200), 10, 0);
        var sub2 = makeSub(1, new Vec3(0, 50, -200), 10, Math.PI);

        SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

        assertEquals(1000, sub1.hp(), "Sub1 should be undamaged");
        assertEquals(1000, sub2.hp(), "Sub2 should be undamaged");
    }

    @Test
    void separatingSubsDontCollide() {
        // Subs overlapping but moving apart → no damage
        var sub1 = makeSub(0, new Vec3(0, -5, -200), 5, Math.PI);  // heading south (away)
        var sub2 = makeSub(1, new Vec3(0, 5, -200), 5, 0);          // heading north (away)

        SimulationLoop.checkSubCollisions(List.of(sub1, sub2));

        assertEquals(1000, sub1.hp(), "Separating sub1 should be undamaged");
        assertEquals(1000, sub2.hp(), "Separating sub2 should be undamaged");
    }
}
