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
import se.hirt.searobots.api.VehicleConfig;

import java.awt.*;

/**
 * Measures coasting distance and time from max speed to 1 m/s
 * under various conditions: clutch engaged (prop drag), clutch
 * disengaged (freewheeling), and with rudder deflection.
 */
class CoastingTest {

    static final double DT = 1.0 / 50;
    static final SubmarinePhysics physics = new SubmarinePhysics();
    static final GeneratedWorld world = GeneratedWorld.deepFlat();

    @Test
    void coastFromMaxSpeed() {
        System.out.println("=== Coasting from max speed (15 m/s) to 1 m/s ===\n");

        System.out.printf("%-35s %8s %8s %8s%n", "Condition", "Time(s)", "Dist(m)", "Final");
        System.out.println("-".repeat(65));

        // 1. Clutch engaged, throttle zero (prop windmilling drag)
        coast("Throttle=0, clutch engaged", sub -> {
            sub.setThrottle(0);
        });

        // 2. Clutch disengaged (freewheeling, minimal drag)
        coast("Throttle=0, clutch disengaged", sub -> {
            sub.setThrottle(0);
            sub.setEngineClutch(false);
        });

        // 3. Full reverse (active braking)
        coast("Throttle=-1.0 (full reverse)", sub -> {
            sub.setThrottle(-1.0);
        });

        // 4. Coasting with 55% rudder (optimal turn while coasting)
        coast("Throttle=0, rudder=55%, clutch off", sub -> {
            sub.setThrottle(0);
            sub.setEngineClutch(false);
            sub.setRudder(0.55);
        });

        // 5. Coasting with full rudder (past stall)
        coast("Throttle=0, rudder=100%, clutch off", sub -> {
            sub.setThrottle(0);
            sub.setEngineClutch(false);
            sub.setRudder(1.0);
        });

        // Also measure from patrol speed
        System.out.println("\n=== Coasting from patrol speed (7 m/s) to 1 m/s ===\n");
        System.out.printf("%-35s %8s %8s %8s%n", "Condition", "Time(s)", "Dist(m)", "Final");
        System.out.println("-".repeat(65));

        coastFrom("Throttle=0, clutch disengaged", 7, sub -> {
            sub.setThrottle(0);
            sub.setEngineClutch(false);
        });

        coastFrom("Throttle=0, clutch engaged", 7, sub -> {
            sub.setThrottle(0);
        });
    }

    void coast(String label, java.util.function.Consumer<SubmarineEntity> setup) {
        coastFrom(label, 15, setup);
    }

    void coastFrom(String label, double fromSpeed, java.util.function.Consumer<SubmarineEntity> setup) {
        var sub = makeSub(-200);

        // Reach target speed
        double throttle = findThrottle(fromSpeed);
        sub.setThrottle(throttle);
        for (int t = 0; t < 50 * 120; t++) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
        }
        double startSpeed = sub.speed();
        double startX = sub.x();
        double startY = sub.y();

        // Apply coast condition
        setup.accept(sub);

        // Coast until speed drops to 1 m/s or 10 minutes
        int ticks = 0;
        int maxTicks = 50 * 600;
        while (sub.speed() > 1.0 && ticks < maxTicks) {
            physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
            ticks++;
        }

        double dist = Math.sqrt(Math.pow(sub.x() - startX, 2) + Math.pow(sub.y() - startY, 2));
        double time = ticks * DT;

        System.out.printf("%-35s %7.1fs %7.0fm  %.1f m/s%n",
                label, time, dist, sub.speed());
    }

    double findThrottle(double targetSpeed) {
        double lo = 0, hi = 1;
        for (int i = 0; i < 30; i++) {
            double mid = (lo + hi) / 2;
            var sub = makeSub(-200);
            sub.setThrottle(mid);
            for (int t = 0; t < 50 * 120; t++)
                physics.step(sub, DT, world.terrain(), world.currentField(), world.config().battleArea());
            if (sub.speed() < targetSpeed) lo = mid; else hi = mid;
        }
        return (lo + hi) / 2;
    }

    SubmarineEntity makeSub(double depth) {
        return new SubmarineEntity(
                VehicleConfig.submarine(), 0, new PhysicsCharacterization.DummyController(),
                new Vec3(0, 0, depth), 0, Color.RED, 1000);
    }
}
