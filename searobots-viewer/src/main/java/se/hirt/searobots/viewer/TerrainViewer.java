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

            var simManager = new SimulationManager();
            simManager.addListener(panel); // 2D view subscribes to tick events
            long[] currentSeed = {seed}; // mutable for lambda capture

            // Lazily created 3D scene (declared early so restartWith can reference it)
            final SubmarineScene3D[] scene3D = {null};
            final java.awt.Canvas[] canvas3D = {null};
            // lastWorld3D removed: SimulationManager.setWorld handles all viewers

            frame.addWindowListener(new java.awt.event.WindowAdapter() {
                @Override
                public void windowClosed(java.awt.event.WindowEvent e) {
                    simManager.stop();
                    if (scene3D[0] != null) {
                        scene3D[0].stop();
                    }
                    System.exit(0);
                }
            });

            // Helper to start/restart with an arbitrary world
            java.util.function.Consumer<GeneratedWorld> restartWith = w -> {
                try {
                    // Stop any active competition
                    if (activeCompetition != null) {
                        activeCompetition.stopAll();
                        activeCompetition = null;
                    }
                    panel.clearCompetitionResults();
                    // Loading state is read from sim loop via state supplier
                    simManager.stop();
                    simManager.setWorld(w);
                    // Paused state is read from sim loop via supplier
                    // Start sim with configured controllers
                    var controllers = SimConfigDialog.currentControllers();
                    var vehicleConfigs = SimConfigDialog.currentVehicleConfigs();
                    if (!controllers.isEmpty()) {
                        simManager.start(w, controllers, vehicleConfigs);
                        simManager.play();
                    }
                    // Loading state cleared automatically when sim starts running
                    frame.setTitle("SeaRobots: Simulation Viewer [seed: " + Long.toHexString(w.config().worldSeed()) + "]");
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
                if (activeCompetition != null && activeCompetition.isRunning()) {
                    activeCompetition.skipToNext();
                } else {
                    currentSeed[0] = ThreadLocalRandom.current().nextLong();
                    restartWithCurrentSeed.run();
                }
            });
            simMenu.add(newMapItem);

            simMenu.addSeparator();

            var setSeedItem = new JMenuItem("Set seed...");
            setSeedItem.addActionListener(e -> {
                var input = JOptionPane.showInputDialog(frame, "Enter seed (hex or decimal):",
                        Long.toHexString(currentSeed[0]));
                if (input != null && !input.isBlank()) {
                    try {
                        String s = input.trim();
                        // Try hex first (no prefix needed), fall back to decimal
                        try {
                            currentSeed[0] = Long.parseUnsignedLong(s, 16);
                        } catch (NumberFormatException hex) {
                            currentSeed[0] = Long.parseLong(s);
                        }
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
                        Long.toHexString(currentSeed[0])), null);
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
                    if (activeCompetition != null) {
                        activeCompetition.stopAll();
                        activeCompetition = null;
                    }
                    panel.clearCompetitionResults();
                    simManager.stop();
                    simManager.setWorld(lWorld);
                    // Paused state is read from sim loop via supplier
                    var controllers = SimConfigDialog.currentControllers();
                    var vehicleConfigs = SimConfigDialog.currentVehicleConfigs();
                    if (!controllers.isEmpty()) {
                        simManager.start(lWorld, controllers, vehicleConfigs, List.of(0.0, 0.0));
                        simManager.play();
                    }
                    frame.setTitle("SeaRobots: L-Island Recovery Test");
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
                    System.out.println("Dialog confirmed. Competition mode: " + SimConfigDialog.isCompetitionMode());
                    if (SimConfigDialog.isCompetitionMode()) {
                        System.out.println(">>> Starting competition mode");
                        runCompetitionInViewer(frame, simManager, panel, scene3D);
                    } else {
                        restartWithCurrentSeed.run();
                    }
                }
            });
            simMenu.add(configItem);

            simMenu.addSeparator();

            var objectivesItem = new JCheckBoxMenuItem("Inject competition objectives", false);
            objectivesItem.addActionListener(e -> {
                simManager.injectObjectives = objectivesItem.isSelected();
                restartWithCurrentSeed.run();
            });
            simMenu.add(objectivesItem);

            simMenu.addSeparator();

            var pauseOnDeathItem = new JCheckBoxMenuItem("Pause on ship death", true);
            simManager.pauseOnDeath = true;
            pauseOnDeathItem.addActionListener(e -> {
                simManager.pauseOnDeath = pauseOnDeathItem.isSelected();
            });
            simMenu.add(pauseOnDeathItem);

            var pauseOnSolutionItem = new JCheckBoxMenuItem("Pause on torpedo solution", true);
            simManager.pauseOnTorpedoSolution = true;
            pauseOnSolutionItem.addActionListener(e -> {
                simManager.pauseOnTorpedoSolution = pauseOnSolutionItem.isSelected();
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
                    simManager.addListener(scene3D[0]); // 3D view subscribes to tick events
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
                // Sync world to 3D scene when switching views
                var w3d = panel.getWorld();
                if (w3d != null) {
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
            frame.setTitle("SeaRobots: Simulation Viewer [seed: " + Long.toHexString(currentSeed[0]) + "]");

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
                if (activeCompetition != null && activeCompetition.isRunning()) {
                    activeCompetition.skipToNext();
                } else {
                    currentSeed[0] = ThreadLocalRandom.current().nextLong();
                    restartWithCurrentSeed.run();
                }
            }));

            // Helper to get the active sim loop (competition or free patrol)
            java.util.function.Supplier<SimulationLoop> activeSim = () -> {
                if (activeCompetition != null && activeCompetition.isRunning()) {
                    return activeCompetition.currentSim();
                }
                return simManager.currentLoop();
            };

            // Suppliers: MapPanel reads actual state from the active sim loop
            panel.setSimSpeedSupplier(() -> {
                var sim = activeSim.get();
                return sim != null ? sim.getSpeedMultiplier() : 1.0;
            });
            panel.setSimPausedSupplier(() -> {
                var sim = activeSim.get();
                return sim != null && sim.isPaused();
            });
            panel.setSimStateSupplier(() -> {
                var sim = activeSim.get();
                return sim != null ? sim.getState() : SimulationLoop.State.STOPPED;
            });

            // Helper to set speed on the active sim and persist for competition
            java.util.function.IntConsumer setSimSpeed = (int mult) -> {
                var sim = activeSim.get();
                if (sim != null) sim.setSpeedMultiplier(mult);
                if (activeCompetition != null && activeCompetition.isRunning()) {
                    activeCompetition.setSpeed(mult);
                }
            };

            gim.put(KeyStroke.getKeyStroke('p'), "pause");
            gim.put(KeyStroke.getKeyStroke('P'), "pause");
            gam.put("pause", action(() -> {
                var sim = activeSim.get();
                if (sim != null) {
                    sim.setPaused(!sim.isPaused());
                }
            }));

            gim.put(KeyStroke.getKeyStroke('n'), "step");
            gim.put(KeyStroke.getKeyStroke('N'), "step");
            gam.put("step", action(() -> {
                var sim = activeSim.get();
                if (sim != null) sim.stepOnce();
            }));

            gim.put(KeyStroke.getKeyStroke('1'), "speed1");
            gam.put("speed1", action(() -> setSimSpeed.accept(1)));
            gim.put(KeyStroke.getKeyStroke('2'), "speed2");
            gam.put("speed2", action(() -> setSimSpeed.accept(2)));
            gim.put(KeyStroke.getKeyStroke('3'), "speed4");
            gam.put("speed4", action(() -> setSimSpeed.accept(4)));
            gim.put(KeyStroke.getKeyStroke('4'), "speed8");
            gam.put("speed8", action(() -> setSimSpeed.accept(8)));
            gim.put(KeyStroke.getKeyStroke('5'), "speed16");
            gam.put("speed16", action(() -> setSimSpeed.accept(16)));
            gim.put(KeyStroke.getKeyStroke('6'), "speed24");
            gam.put("speed24", action(() -> setSimSpeed.accept(24)));
            gim.put(KeyStroke.getKeyStroke('0'), "speedMax");
            gam.put("speedMax", action(() -> setSimSpeed.accept(1_000_000)));

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

            im.put(KeyStroke.getKeyStroke('g'), "strategicWaypoints");
            im.put(KeyStroke.getKeyStroke('G'), "strategicWaypoints");
            am.put("strategicWaypoints", action(panel::toggleStrategicWaypoints));

            frame.setSize(2560, 1080);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            // Auto-start simulation
            {
                var controllers = SimConfigDialog.currentControllers();
                var vehicleConfigs = SimConfigDialog.currentVehicleConfigs();
                if (!controllers.isEmpty()) {
                    simManager.start(world, controllers, vehicleConfigs);
                }
            }
        });
    }

    // SimulationHolder removed: SimulationManager handles all sim lifecycle.

    private static CompetitionRunner activeCompetition;

    private static void runCompetitionInViewer(JFrame frame, SimulationManager simManager,
                                                   MapPanel panel, SubmarineScene3D[] scene3D) {
        // Stop any existing simulation or competition completely
        simManager.stop();
        if (activeCompetition != null) {
            activeCompetition.stopAll();
            activeCompetition = null;
        }
        // Brief pause for old threads to finish
        try { Thread.sleep(150); } catch (InterruptedException ignored) {}

        var names = SimConfigDialog.currentNames();
        var factories = SimConfigDialog.currentFactories();
        if (factories.size() < 2) {
            JOptionPane.showMessageDialog(frame, "Need 2 ships for competition.", "Error",
                    JOptionPane.ERROR_MESSAGE);
            return;
        }

        var competitors = new java.util.ArrayList<SubmarineCompetition.Competitor>();
        for (int i = 0; i < factories.size(); i++) {
            competitors.add(new SubmarineCompetition.Competitor(names.get(i), factories.get(i)));
        }

        activeCompetition = new CompetitionRunner(frame, panel, simManager);
        activeCompetition.start(competitors, 5,
                SubmarineCompetition.DEFAULT_DURATION / SubmarineCompetition.TICKS_PER_SECOND);
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
