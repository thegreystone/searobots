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
import se.hirt.searobots.engine.ships.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.*;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.Image;
import java.awt.event.KeyEvent;
import java.io.IOException;
import java.util.ArrayList;
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
            var frame = new JFrame("SeaRobots: Simulation Viewer");
            frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
            loadIcons(frame);

            var panel = new MapPanel(world);
            frame.add(panel);

            var simHolder = new SimulationHolder();
            long[] currentSeed = {seed}; // mutable for lambda capture

            // Lazily created 3D scene (declared early so restartWith can reference it)
            final SubmarineScene3D[] scene3D = {null};
            final java.awt.Canvas[] canvas3D = {null};
            final GeneratedWorld[] lastWorld3D = {null};

            frame.addWindowListener(new java.awt.event.WindowAdapter() {
                @Override
                public void windowClosed(java.awt.event.WindowEvent e) {
                    simHolder.stop();
                    if (scene3D[0] != null) {
                        scene3D[0].stop();
                    }
                    System.exit(0);
                }
            });

            // Helper to start/restart with an arbitrary world
            java.util.function.Consumer<GeneratedWorld> restartWith = w -> {
                try {
                    simHolder.stop();
                    panel.setWorld(w);
                    panel.setSimPaused(false);
                    simHolder.start(w, panel);
                    frame.setTitle("SeaRobots: Simulation Viewer [seed: " + w.config().worldSeed() + "]");
                    if (scene3D[0] != null) {
                        scene3D[0].setWorld(w);
                        lastWorld3D[0] = w;
                    }
                    panel.requestFocusInWindow();
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            };

            // Helper to start/restart with a given seed
            Runnable restartWithCurrentSeed = () -> {
                restartWith.accept(generator.generate(MatchConfig.withDefaults(currentSeed[0])));
            };

            // Menu bar
            var menuBar = new JMenuBar();
            var simMenu = new JMenu("Simulation");
            var rerunItem = new JMenuItem("Re-run (same seed)");
            rerunItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_R, KeyEvent.CTRL_DOWN_MASK));
            rerunItem.addActionListener(e -> restartWithCurrentSeed.run());
            simMenu.add(rerunItem);

            var newMapItem = new JMenuItem("New map (random seed)");
            newMapItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0));
            newMapItem.addActionListener(e -> {
                currentSeed[0] = ThreadLocalRandom.current().nextLong();
                restartWithCurrentSeed.run();
            });
            simMenu.add(newMapItem);

            simMenu.addSeparator();

            var setSeedItem = new JMenuItem("Set seed...");
            setSeedItem.addActionListener(e -> {
                var input = JOptionPane.showInputDialog(frame, "Enter seed:", currentSeed[0]);
                if (input != null && !input.isBlank()) {
                    try {
                        currentSeed[0] = Long.parseLong(input.trim());
                        restartWithCurrentSeed.run();
                    } catch (NumberFormatException ex) {
                        JOptionPane.showMessageDialog(frame, "Invalid seed: " + input,
                                "Error", JOptionPane.ERROR_MESSAGE);
                    }
                }
            });
            simMenu.add(setSeedItem);

            var copySeedItem = new JMenuItem("Copy seed to clipboard");
            copySeedItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_C, KeyEvent.CTRL_DOWN_MASK | KeyEvent.SHIFT_DOWN_MASK));
            copySeedItem.addActionListener(e -> {
                var clipboard = java.awt.Toolkit.getDefaultToolkit().getSystemClipboard();
                clipboard.setContents(new java.awt.datatransfer.StringSelection(
                        String.valueOf(currentSeed[0])), null);
            });
            simMenu.add(copySeedItem);

            simMenu.addSeparator();

            var flatOceanItem = new JMenuItem("Deep flat ocean (no terrain)");
            flatOceanItem.addActionListener(e -> restartWith.accept(GeneratedWorld.deepFlat()));
            simMenu.add(flatOceanItem);

            var lIslandItem = new JMenuItem("Test: L-Island three-point turn");
            lIslandItem.addActionListener(e -> {
                var lWorld = GeneratedWorld.lIslandRecovery();
                try {
                    simHolder.stop();
                    panel.setWorld(lWorld);
                    panel.setSimPaused(false);
                    simHolder.startWithHeadings(lWorld, panel, List.of(0.0, 0.0));
                    frame.setTitle("SeaRobots: L-Island Recovery Test");
                    if (scene3D[0] != null) {
                        scene3D[0].setWorld(lWorld);
                        lastWorld3D[0] = lWorld;
                    }
                    panel.requestFocusInWindow();
                } catch (Exception ex) {
                    ex.printStackTrace();
                }
            });
            simMenu.add(lIslandItem);

            simMenu.addSeparator();

            var configItem = new JMenuItem("Configure Simulation...");
            configItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_F2, 0));
            configItem.addActionListener(e -> {
                var dialog = new SimConfigDialog(frame);
                dialog.setVisible(true);
                if (dialog.isConfirmed()) {
                    restartWithCurrentSeed.run();
                }
            });
            simMenu.add(configItem);

            simMenu.addSeparator();

            var objectivesItem = new JCheckBoxMenuItem("Inject competition objectives", false);
            objectivesItem.addActionListener(e -> {
                simHolder.injectObjectives = objectivesItem.isSelected();
                restartWithCurrentSeed.run();
            });
            simMenu.add(objectivesItem);

            simMenu.addSeparator();

            var pauseOnDeathItem = new JCheckBoxMenuItem("Pause on ship death", true);
            simHolder.pauseOnDeath = true;
            pauseOnDeathItem.addActionListener(e -> {
                simHolder.pauseOnDeath = pauseOnDeathItem.isSelected();
            });
            simMenu.add(pauseOnDeathItem);

            var pauseOnSolutionItem = new JCheckBoxMenuItem("Pause on torpedo solution", true);
            simHolder.pauseOnTorpedoSolution = true;
            pauseOnSolutionItem.addActionListener(e -> {
                simHolder.pauseOnTorpedoSolution = pauseOnSolutionItem.isSelected();
            });
            simMenu.add(pauseOnSolutionItem);

            menuBar.add(simMenu);

            // View menu: 2D / 3D toggle
            var viewMenu = new JMenu("View");
            var viewGroup = new ButtonGroup();
            var view2DItem = new JRadioButtonMenuItem("2D Map", true);
            var view3DItem = new JRadioButtonMenuItem("3D Scene");
            viewGroup.add(view2DItem);
            viewGroup.add(view3DItem);
            viewMenu.add(view2DItem);
            viewMenu.add(view3DItem);

            view2DItem.addActionListener(e -> {
                if (canvas3D[0] != null) canvas3D[0].setVisible(false);
                panel.setVisible(true);
                panel.requestFocusInWindow();
            });

            view3DItem.addActionListener(e -> {
                boolean firstTime = scene3D[0] == null;
                if (firstTime) {
                    scene3D[0] = SubmarineScene3D.create();
                    simHolder.scene3DRef = scene3D[0];
                    canvas3D[0] = scene3D[0].getCanvas();
                    canvas3D[0].setPreferredSize(new java.awt.Dimension(2560, 1080));
                    canvas3D[0].setFocusTraversalKeysEnabled(false);
                    // Add canvas to content pane (hidden initially, kept alive)
                    frame.getContentPane().add(canvas3D[0]);
                    scene3D[0].startCanvas();
                }
                panel.setVisible(false);
                canvas3D[0].setVisible(true);
                canvas3D[0].requestFocusInWindow();
                frame.revalidate();
                frame.repaint();
                // Set world if changed
                var w3d = panel.getWorld();
                if (w3d != lastWorld3D[0]) {
                    lastWorld3D[0] = w3d;
                    var s = scene3D[0];
                    Timer timer = new Timer(firstTime ? 500 : 50, evt -> s.setWorld(w3d));
                    timer.setRepeats(false);
                    timer.start();
                }
            });

            viewMenu.addSeparator();
            var debugModeItem = new JCheckBoxMenuItem("Debug Mode (no effects)", false);
            debugModeItem.addActionListener(e -> {
                if (scene3D[0] != null) {
                    scene3D[0].setDebugMode(debugModeItem.isSelected());
                }
            });
            viewMenu.add(debugModeItem);

            var atmosphereItem = new JCheckBoxMenuItem("Atmosphere", true);
            atmosphereItem.addActionListener(e -> {
                if (scene3D[0] != null) {
                    scene3D[0].setAtmosphereEnabled(atmosphereItem.isSelected());
                }
            });
            viewMenu.add(atmosphereItem);

            var godRaysItem = new JCheckBoxMenuItem("God Rays", true);
            godRaysItem.addActionListener(e -> {
                if (scene3D[0] != null) {
                    scene3D[0].setGodRaysEnabled(godRaysItem.isSelected());
                }
            });
            viewMenu.add(godRaysItem);

            var waterSettingsItem = new JMenuItem("Water Settings...");
            waterSettingsItem.addActionListener(e -> {
                if (scene3D[0] != null && scene3D[0].getWaterFilter() != null) {
                    new WaterSettingsDialog(frame, scene3D[0].getWaterFilter()).setVisible(true);
                } else {
                    JOptionPane.showMessageDialog(frame, "Switch to 3D view first.",
                            "Water Settings", JOptionPane.INFORMATION_MESSAGE);
                }
            });
            viewMenu.add(waterSettingsItem);

            var timeOfDayItem = new JMenuItem("Set Time of Day...");
            timeOfDayItem.addActionListener(e -> {
                if (scene3D[0] != null) {
                    var current = scene3D[0].getStartTime();
                    var input = JOptionPane.showInputDialog(frame,
                            "Enter start time of day (HH:mm):",
                            current.toString().substring(0, 5));
                    if (input != null && !input.isBlank()) {
                        try {
                            scene3D[0].setStartTime(java.time.LocalTime.parse(input.trim()));
                        } catch (Exception ex) {
                            JOptionPane.showMessageDialog(frame, "Invalid time: " + input,
                                    "Error", JOptionPane.ERROR_MESSAGE);
                        }
                    }
                } else {
                    JOptionPane.showMessageDialog(frame, "Switch to 3D view first.",
                            "Time of Day", JOptionPane.INFORMATION_MESSAGE);
                }
            });
            viewMenu.add(timeOfDayItem);

            menuBar.add(viewMenu);
            frame.setJMenuBar(menuBar);
            frame.setTitle("SeaRobots: Simulation Viewer [seed: " + currentSeed[0] + "]");

            // Global keybindings on root pane (work in both 2D and 3D views)
            var rootPane = frame.getRootPane();
            var gim = rootPane.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
            var gam = rootPane.getActionMap();

            // F11: toggle maximized window
            gim.put(KeyStroke.getKeyStroke(KeyEvent.VK_F11, 0), "maximize");
            gam.put("maximize", action(() -> {
                int state = frame.getExtendedState();
                if ((state & java.awt.Frame.MAXIMIZED_BOTH) != 0) {
                    frame.setExtendedState(state & ~java.awt.Frame.MAXIMIZED_BOTH);
                } else {
                    frame.setExtendedState(state | java.awt.Frame.MAXIMIZED_BOTH);
                }
            }));

            gim.put(KeyStroke.getKeyStroke(KeyEvent.VK_SPACE, 0), "newMap");
            gam.put("newMap", action(() -> {
                currentSeed[0] = ThreadLocalRandom.current().nextLong();
                restartWithCurrentSeed.run();
            }));

            gim.put(KeyStroke.getKeyStroke('p'), "pause");
            gim.put(KeyStroke.getKeyStroke('P'), "pause");
            gam.put("pause", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) {
                    boolean p = !sim.isPaused();
                    sim.setPaused(p);
                    panel.setSimPaused(p);
                }
            }));

            gim.put(KeyStroke.getKeyStroke('n'), "step");
            gim.put(KeyStroke.getKeyStroke('N'), "step");
            gam.put("step", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) sim.stepOnce();
            }));

            gim.put(KeyStroke.getKeyStroke('1'), "speed1");
            gam.put("speed1", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(1); panel.setSimSpeed(1); }
            }));
            gim.put(KeyStroke.getKeyStroke('2'), "speed2");
            gam.put("speed2", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(2); panel.setSimSpeed(2); }
            }));
            gim.put(KeyStroke.getKeyStroke('3'), "speed4");
            gam.put("speed4", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(4); panel.setSimSpeed(4); }
            }));
            gim.put(KeyStroke.getKeyStroke('4'), "speed8");
            gam.put("speed8", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(8); panel.setSimSpeed(8); }
            }));
            gim.put(KeyStroke.getKeyStroke('5'), "speed16");
            gam.put("speed16", action(() -> {
                var sim = simHolder.loop;
                if (sim != null) { sim.setSpeedMultiplier(16); panel.setSimSpeed(16); }
            }));

            // 2D-only keybindings on MapPanel
            var im = panel.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
            var am = panel.getActionMap();

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

            im.put(KeyStroke.getKeyStroke('e'), "contacts");
            im.put(KeyStroke.getKeyStroke('E'), "contacts");
            am.put("contacts", action(panel::toggleContactEstimates));

            im.put(KeyStroke.getKeyStroke('w'), "waypoints");
            im.put(KeyStroke.getKeyStroke('W'), "waypoints");
            am.put("waypoints", action(panel::toggleWaypoints));

            frame.setSize(2560, 1080);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // Auto-start simulation
            simHolder.start(world, panel);
        });
    }

    private static class SimulationHolder {
        volatile SimulationLoop loop;
        volatile Thread thread;
        volatile boolean pauseOnDeath;
        volatile boolean pauseOnTorpedoSolution;
        volatile boolean injectObjectives;
        volatile SubmarineScene3D scene3DRef;
        private boolean torpedoSolutionTriggered;
        private final java.util.Set<Integer> deadEntities = java.util.Collections.synchronizedSet(new java.util.HashSet<>());

        void startWithHeadings(GeneratedWorld world, MapPanel panel, List<Double> headings) {
            start(world, panel, headings);
        }

        void start(GeneratedWorld world, MapPanel panel) {
            start(world, panel, null);
        }

        private void start(GeneratedWorld world, MapPanel panel, List<Double> headings) {
            stop();
            var sim = new SimulationLoop();
            this.loop = sim;

            // Normal battle scenario: use procedural spawn points (face toward center)
            var simWorld = world;

            List<SubmarineController> controllers = SimConfigDialog.currentControllers();
            List<VehicleConfig> vehicleConfigs = SimConfigDialog.currentVehicleConfigs();
            if (controllers.isEmpty()) {
                System.out.println("No ships configured, skipping simulation.");
                return;
            }

            // Inject competition objectives if enabled
            if (injectObjectives) {
                var objectives = SubmarineCompetition.generateObjectives(
                        simWorld.config().worldSeed(), simWorld);
                var terrain = simWorld.terrain();
                double depth1 = Math.max(-300, terrain.elevationAt(objectives.x1(), objectives.y1()) + 90);
                double depth2 = Math.max(-300, terrain.elevationAt(objectives.x2(), objectives.y2()) + 90);
                var objList = java.util.List.of(
                        new StrategicWaypoint(objectives.x1(), objectives.y1(), depth1,
                                Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1),
                        new StrategicWaypoint(objectives.x2(), objectives.y2(), depth2,
                                Purpose.PATROL, NoisePolicy.NORMAL, MovementPattern.DIRECT, 300, -1)
                );
                for (var ctrl : controllers) {
                    ctrl.setObjectives(objList);
                }
                System.out.printf("Injected objectives: WP1=(%.0f,%.0f) WP2=(%.0f,%.0f)%n",
                        objectives.x1(), objectives.y1(), objectives.x2(), objectives.y2());
            }

            MatchRecorder recorder = null;
            try {
                var logDir = java.nio.file.Path.of("logs");
                recorder = new MatchRecorder(simWorld.config(), simWorld.spawnPoints(), logDir);
                System.out.println("Recording match to " + recorder.logFile());
            } catch (java.io.IOException e) {
                System.err.println("Failed to create match recorder: " + e.getMessage());
            }
            final var rec = recorder;

            deadEntities.clear();
            torpedoSolutionTriggered = false;

            var listener = new SimulationListener() {
                @Override
                public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                    panel.updateSubmarines(tick, submarines);
                    var s3d = scene3DRef;
                    if (s3d != null) s3d.updateSubmarines(tick, submarines);
                    if (rec != null) rec.onTick(tick, submarines);

                    // Pause on death detection
                    if (pauseOnDeath) {
                        for (var sub : submarines) {
                            if (sub.hp() <= 0 && !deadEntities.contains(sub.id())) {
                                deadEntities.add(sub.id());
                                sim.setPaused(true);
                                panel.setSimPaused(true);
                            }
                        }
                    }

                    // Pause on first torpedo solution (crosshair drawn by MapPanel from snapshot)
                    if (pauseOnTorpedoSolution && !torpedoSolutionTriggered) {
                        for (var sub : submarines) {
                            if (sub.firingSolution() != null) {
                                torpedoSolutionTriggered = true;
                                sim.setPaused(true);
                                panel.setSimPaused(true);
                                var sol = sub.firingSolution();
                                System.out.printf("TORPEDO SOLUTION at tick %d: %s target=[%.0f,%.0f] hdg=%.0f spd=%.1f q=%.2f%n",
                                        tick, sub.name(), sol.targetX(), sol.targetY(),
                                        Math.toDegrees(sol.targetHeading()), sol.targetSpeed(), sol.quality());
                                break;
                            }
                        }
                    }
                }

                @Override
                public void onMatchEnd() {
                    if (rec != null) rec.onMatchEnd();
                }
            };

            var t = Thread.ofPlatform().daemon().name("sim-loop").start(() ->
                    sim.run(simWorld, controllers, vehicleConfigs, headings, listener));
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

    private static void loadIcons(JFrame frame) {
        String[] sizes = {"16", "24", "32", "48", "64", "128", "256", "512", "1024"};
        var icons = new ArrayList<Image>();
        for (String size : sizes) {
            var url = TerrainViewer.class.getResource("/icons/searobots-" + size + ".png");
            if (url != null) {
                try {
                    icons.add(ImageIO.read(url));
                } catch (IOException e) {
                    // skip missing sizes
                }
            }
        }
        if (!icons.isEmpty()) {
            frame.setIconImages(icons);
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
