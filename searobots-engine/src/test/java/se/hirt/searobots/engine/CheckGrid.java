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
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.PathPlanner;
class CheckGrid {
    @Test void check() {
        var world = new WorldGenerator().generate(MatchConfig.withDefaults(55555));
        var planner = new PathPlanner(world.terrain(), -80, 200, 75);
        // Check death location and surroundings
        double x = 873, y = 1063;
        System.out.printf("Death [%.0f,%.0f]: safe=%s cost=%.1f floor=%.0f%n",
                x, y, planner.isSafe(x, y), planner.costAt(x, y), world.terrain().elevationAt(x, y));
        for (int d = 0; d < 360; d += 45) {
            double brg = Math.toRadians(d);
            double px = x + Math.sin(brg) * 500;
            double py = y + Math.cos(brg) * 500;
            System.out.printf("  %3d deg +500m: safe=%s cost=%.1f floor=%.0f%n",
                    d, planner.isSafe(px, py), planner.costAt(px, py), world.terrain().elevationAt(px, py));
        }
    }
}
