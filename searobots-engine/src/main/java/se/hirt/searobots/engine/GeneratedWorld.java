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

import se.hirt.searobots.api.*;

import java.util.List;

/**
 * The result of procedural world generation: everything needed to run
 * or display a match. Can also be constructed directly for test scenarios.
 */
public record GeneratedWorld(
        MatchConfig config,
        TerrainMap terrain,
        List<ThermalLayer> thermalLayers,
        CurrentField currentField,
        List<Vec3> spawnPoints) {

    /**
     * Flat ocean floor at the given depth, no thermal layers, no currents.
     * Spawn points are placed 2000m apart at the specified operating depth.
     */
    public static GeneratedWorld flatOcean(double floorDepth, double operatingDepth) {
        var config = MatchConfig.withDefaults(0);
        double halfSize = config.battleArea() instanceof BattleArea.Circular c
                ? c.radius() : 7000;
        double margin = config.terrainMarginMeters();
        double totalSize = (halfSize + margin) * 2;
        double cellSize = config.gridCellMeters();
        int gridSize = (int) Math.ceil(totalSize / cellSize) + 1;
        double origin = -(gridSize / 2) * cellSize;

        double[] data = new double[gridSize * gridSize];
        java.util.Arrays.fill(data, floorDepth);
        var terrain = new TerrainMap(data, gridSize, gridSize, origin, origin, cellSize);

        var spawns = List.of(
                new Vec3(-1000, 0, operatingDepth),
                new Vec3(1000, 0, operatingDepth));

        return new GeneratedWorld(config, terrain, List.of(),
                new CurrentField(List.of()), spawns);
    }

    /**
     * Flat ocean at 1000m depth, subs at 100m depth. No terrain, no
     * thermoclines, no currents. Pure open-water physics testbed.
     */
    public static GeneratedWorld deepFlat() {
        return flatOcean(-1000, -100);
    }

    /**
     * L-shaped island on deep ocean for testing three-point turn recovery.
     * Sub 0 spawns 900m south of the island, facing north (into the wall).
     */
    public static GeneratedWorld lIslandRecovery() {
        var config = MatchConfig.withDefaults(0);
        double halfSize = config.battleArea() instanceof BattleArea.Circular c
                ? c.radius() : 7000;
        double margin = config.terrainMarginMeters();
        double totalSize = (halfSize + margin) * 2;
        double cellSize = config.gridCellMeters();
        int gridSize = (int) Math.ceil(totalSize / cellSize) + 1;
        double origin = -(gridSize / 2) * cellSize;

        double[] data = new double[gridSize * gridSize];
        for (int row = 0; row < gridSize; row++) {
            double worldY = origin + row * cellSize;
            for (int col = 0; col < gridSize; col++) {
                double worldX = origin + col * cellSize;
                boolean inHArm = worldX >= -2000 && worldX <= 2000
                        && worldY >= -500 && worldY <= 500;
                boolean inVArm = worldX >= -500 && worldX <= 500
                        && worldY >= -500 && worldY <= 2000;
                data[row * gridSize + col] = (inHArm || inVArm) ? 5.0 : -500;
            }
        }
        var terrain = new TerrainMap(data, gridSize, gridSize, origin, origin, cellSize);

        var spawns = List.of(
                new Vec3(0, -900, -150),    // sub 0: south of island, facing north
                new Vec3(5000, -5000, -200));

        return new GeneratedWorld(config, terrain, List.of(),
                new CurrentField(List.of()), spawns);
    }
}
