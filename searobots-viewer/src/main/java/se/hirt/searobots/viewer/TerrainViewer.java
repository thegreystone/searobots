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
package se.hirt.searobots.viewer;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.engine.*;

import javax.swing.*;
import java.awt.event.KeyEvent;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

public final class TerrainViewer {

    public static void main(String[] args) {
        long seed = args.length > 0
                ? Long.parseLong(args[0])
                : ThreadLocalRandom.current().nextLong();

        var generator = new WorldGenerator();
        var world = generator.generate(MatchConfig.withDefaults(seed));

        SwingUtilities.invokeLater(() -> {
            var frame = new JFrame("SeaRobots \u2014 Simulation Viewer");
            frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

            var panel = new MapPanel(world);
            frame.add(panel);

            var simHolder = new SimulationHolder();

            var im = panel.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
            var am = panel.getActionMap();

            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "newMap");
            am.put("newMap", action(() -> {
                simHolder.stop();
                long newSeed = ThreadLocalRandom.current().nextLong();
                var newWorld = generator.generate(MatchConfig.withDefaults(newSeed));
                panel.setWorld(newWorld);
                simHolder.start(newWorld, panel);
            }));

            im.put(KeyStroke.getKeyStroke('='), "zoomIn");
            im.put(KeyStroke.getKeyStroke('+'), "zoomIn");
            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_ADD, 0), "zoomIn");
            am.put("zoomIn", action(() -> panel.zoom(1.3)));

            im.put(KeyStroke.getKeyStroke('-'), "zoomOut");
            im.put(KeyStroke.getKeyStroke(KeyEvent.VK_SUBTRACT, 0), "zoomOut");
            am.put("zoomOut", action(() -> panel.zoom(1 / 1.3)));

            im.put(KeyStroke.getKeyStroke('c'), "contours");
            im.put(KeyStroke.getKeyStroke('C'), "contours");
            am.put("contours", action(panel::toggleContours));

            im.put(KeyStroke.getKeyStroke('f'), "currents");
            im.put(KeyStroke.getKeyStroke('F'), "currents");
            am.put("currents", action(panel::toggleCurrents));

            im.put(KeyStroke.getKeyStroke('t'), "trails");
            im.put(KeyStroke.getKeyStroke('T'), "trails");
            am.put("trails", action(panel::toggleTrails));

            im.put(KeyStroke.getKeyStroke('r'), "route");
            im.put(KeyStroke.getKeyStroke('R'), "route");
            am.put("route", action(panel::toggleRoute));

            im.put(KeyStroke.getKeyStroke('p'), "pause");
            im.put(KeyStroke.getKeyStroke('P'), "pause");
            am.put("pause", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) {
                    boolean p = !sim.isPaused();
                    sim.setPaused(p);
                    panel.setSimPaused(p);
                }
            }));

            im.put(KeyStroke.getKeyStroke('n'), "step");
            im.put(KeyStroke.getKeyStroke('N'), "step");
            am.put("step", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) sim.stepOnce();
            }));

            im.put(KeyStroke.getKeyStroke('1'), "speed1");
            am.put("speed1", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(1); panel.setSimSpeed(1); }
            }));
            im.put(KeyStroke.getKeyStroke('2'), "speed2");
            am.put("speed2", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(2); panel.setSimSpeed(2); }
            }));
            im.put(KeyStroke.getKeyStroke('3'), "speed5");
            am.put("speed5", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(5); panel.setSimSpeed(5); }
            }));
            im.put(KeyStroke.getKeyStroke('4'), "speed10");
            am.put("speed10", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(10); panel.setSimSpeed(10); }
            }));

            frame.setSize(1200, 900);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // Auto-start simulation
            simHolder.start(world, panel);
        });
    }

    private static class SimulationHolder {
        volatile SimulationLoop loop;
        volatile Thread thread;

        void start(GeneratedWorld world, MapPanel panel) {
            stop();
            var sim = new SimulationLoop();
            this.loop = sim;

            List<SubmarineController> controllers = List.of(
                    new ObstacleAvoidanceSub(), new ObstacleAvoidanceSub());

            MatchRecorder recorder = null;
            try {
                var logDir = java.nio.file.Path.of("logs");
                recorder = new MatchRecorder(world.config(), world.spawnPoints(), logDir);
                System.out.println("Recording match to " + recorder.logFile());
            } catch (java.io.IOException e) {
                System.err.println("Failed to create match recorder: " + e.getMessage());
            }
            final var rec = recorder;

            var listener = new SimulationListener() {
                @Override
                public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                    panel.updateSubmarines(tick, submarines);
                    if (rec != null) rec.onTick(tick, submarines);
                }

                @Override
                public void onMatchEnd() {
                    if (rec != null) rec.onMatchEnd();
                }
            };

            var t = Thread.ofPlatform().daemon().name("sim-loop").start(() ->
                    sim.run(world, controllers, listener));
            this.thread = t;
        }

        void stop() {
            var sim = loop;
            if (sim != null) {
                sim.stop();
                loop = null;
            }
            var t = thread;
            if (t != null) {
                t.interrupt();
                thread = null;
            }
        }
    }

    private static AbstractAction action(Runnable r) {
        return new AbstractAction() {
            @Override
            public void actionPerformed(java.awt.event.ActionEvent e) {
                r.run();
            }
        };
    }
}
