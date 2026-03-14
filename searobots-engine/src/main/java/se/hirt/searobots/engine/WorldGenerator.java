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

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Procedurally generates an underwater world from a {@link MatchConfig}.
 * Deterministic — the same config always produces the same world.
 */
public final class WorldGenerator {

    public GeneratedWorld generate(MatchConfig config) {
        var rng = new Random(config.worldSeed());

        double extent = config.battleArea().extent() + config.terrainMarginMeters();
        int gridSize = (int) Math.ceil(2 * extent / config.gridCellMeters()) + 1;
        double origin = -extent;

        var terrain = generateTerrain(config, rng, gridSize, origin);
        var thermals = generateThermals(config, rng);
        var currents = generateCurrents(config, rng);
        var spawns = generateSpawnPoints(config, terrain, rng);

        return new GeneratedWorld(config, terrain, thermals, currents, spawns);
    }

    private TerrainMap generateTerrain(MatchConfig config, Random rng,
                                       int gridSize, double origin) {
        var terrainNoise = new PerlinNoise(rng.nextLong());
        var blendNoise = new PerlinNoise(rng.nextLong());
        var warpNoise = new PerlinNoise(rng.nextLong());
        var shelfNoise = new PerlinNoise(rng.nextLong());
        var trenchNoise = new PerlinNoise(rng.nextLong());

        double cell = config.gridCellMeters();
        double[] elevations = new double[gridSize * gridSize];
        double baseFreq = 1.0 / 800.0;

        double minZ = config.maxSeaFloorZ();
        double maxZ = 150.0;
        double range = maxZ - minZ;
        // Bias midpoint upward — most terrain is moderate depth, deep
        // trenches are the exception rather than the average.
        double mid = minZ + range * 0.6;

        for (int row = 0; row < gridSize; row++) {
            for (int col = 0; col < gridSize; col++) {
                double wx = origin + col * cell;
                double wy = origin + row * cell;

                // domain warping for organic shapes
                double ws = 1.0 / 1200.0;
                double warpX = warpNoise.noise(wx * ws, wy * ws) * 150;
                double warpY = warpNoise.noise(wx * ws + 97, wy * ws + 131) * 150;

                double nx = (wx + warpX) * baseFreq;
                double ny = (wy + warpY) * baseFreq;

                // blend between smooth fBm and ridged noise
                double blend = blendNoise.fbm(wx * 0.0005, wy * 0.0005, 3, 2.0, 0.5) * 0.5 + 0.5;
                double smooth = terrainNoise.fbm(nx, ny, 6, 2.0, 0.5);
                double ridged = terrainNoise.ridged(nx, ny, 6, 2.0, 0.5);
                double n = smooth * (1 - blend) + ridged * blend;

                double elevation = mid + n * range / 2;

                // Continental shelf: low-frequency noise creates broad
                // elevated regions where islands are larger and more common.
                double shelf = shelfNoise.fbm(wx * 0.00025, wy * 0.00025, 2, 2.0, 0.5);
                if (shelf > 0.0) {
                    elevation += shelf * shelf * 600;
                }

                // Deep trenches: mirror of shelf — occasional deep gashes
                double trench = trenchNoise.fbm(wx * 0.0003, wy * 0.0003, 2, 2.0, 0.5);
                if (trench > 0.0) {
                    elevation -= trench * trench * 600;
                }

                elevation = Math.max(minZ, Math.min(maxZ, elevation));
                elevations[row * gridSize + col] = elevation;
            }
        }

        return new TerrainMap(elevations, gridSize, gridSize, origin, origin, cell);
    }

    private List<ThermalLayer> generateThermals(MatchConfig config, Random rng) {
        double shallowZ = config.minSeaFloorZ(); // e.g. -30
        double deepZ = config.maxSeaFloorZ();     // e.g. -500
        int layerCount = 1 + rng.nextInt(3);
        double surfaceTemp = 18.0;

        var layers = new ArrayList<ThermalLayer>();
        for (int i = 0; i < layerCount; i++) {
            double fraction = (i + 1.0) / (layerCount + 1.0);
            double layerZ = shallowZ + (deepZ - shallowZ) * fraction;
            layerZ += (rng.nextDouble() - 0.5) * 30;
            double tempAbove = surfaceTemp - i * 4.0;
            double tempBelow = tempAbove - 3.0 - rng.nextDouble() * 3.0;
            layers.add(new ThermalLayer(layerZ, tempAbove, tempBelow));
        }
        return List.copyOf(layers);
    }

    private CurrentField generateCurrents(MatchConfig config, Random rng) {
        double shallowZ = config.minSeaFloorZ();
        double deepZ = config.maxSeaFloorZ();
        int bandCount = 3 + rng.nextInt(3);

        var bands = new ArrayList<CurrentField.CurrentBand>();
        double bandHeight = (shallowZ - deepZ) / bandCount;

        for (int i = 0; i < bandCount; i++) {
            double maxZ = shallowZ - i * bandHeight;
            double minZ = maxZ - bandHeight;
            double angle = rng.nextDouble() * Math.PI * 2;
            double speed = 0.1 + rng.nextDouble() * 0.5;
            var current = new Vec2(Math.cos(angle) * speed, Math.sin(angle) * speed);
            bands.add(new CurrentField.CurrentBand(minZ, maxZ, current));
        }
        return new CurrentField(List.copyOf(bands));
    }

    private List<Vec3> generateSpawnPoints(MatchConfig config, TerrainMap terrain,
                                           Random rng) {
        int count = config.submarineCount();
        double areaExtent = config.battleArea().extent();
        double minSeparation = areaExtent * 0.5;
        double spawnDepth = -50;

        var points = new ArrayList<Vec3>();
        for (int attempts = 0; attempts < 1000 && points.size() < count; attempts++) {
            double angle = rng.nextDouble() * Math.PI * 2;
            double radius = areaExtent * 0.3 + rng.nextDouble() * areaExtent * 0.5;
            double x = Math.cos(angle) * radius;
            double y = Math.sin(angle) * radius;

            if (!config.battleArea().contains(x, y)) continue;

            double floorZ = terrain.elevationAt(x, y);
            if (spawnDepth < floorZ + 20) continue;

            boolean tooClose = false;
            for (var p : points) {
                if (p.horizontalDistanceTo(new Vec3(x, y, spawnDepth)) < minSeparation) {
                    tooClose = true;
                    break;
                }
            }
            if (tooClose) continue;

            points.add(new Vec3(x, y, spawnDepth));
        }
        return List.copyOf(points);
    }
}
