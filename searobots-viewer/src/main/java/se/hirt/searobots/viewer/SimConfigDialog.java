/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.viewer;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Dialog for configuring the simulation: which ships to use, and their types.
 * Settings persist across simulation restarts until the viewer is closed.
 */
final class SimConfigDialog extends JDialog {

    record ShipOption(String displayName, Supplier<SubmarineController> factory,
                      VehicleConfig vehicleConfig) {
        @Override public String toString() { return displayName; }
    }

    private static final List<ShipOption> SHIP_OPTIONS = List.of(
            new ShipOption("(empty)", null, null),
            new ShipOption("Codex Sub", CodexAttackSub::new, VehicleConfig.submarine()),
            new ShipOption("Claude Sub", ClaudeAttackSub::new, VehicleConfig.submarine()),
            new ShipOption("Default Sub", DefaultAttackSub::new, VehicleConfig.submarine()),
            new ShipOption("Sub Drone", SubmarineDrone::new, VehicleConfig.submarine()),
            new ShipOption("Ship Drone (surface)", TargetDrone::new, VehicleConfig.surfaceShip())
    );

    // Persistent selections (static so they survive dialog close/reopen)
    private static int selectedShip1Index = 2; // Claude Sub
    private static int selectedShip2Index = 1; // Codex Sub
    private static int selectedSpeedMultiplier = 1;
    private static int selectedSimType = 0; // 0=Free patrol, 1=Competition

    static final String[] SIM_TYPES = {"Free patrol", "Competition (nav + combat)"};

    private final JComboBox<ShipOption> ship1Combo;
    private final JComboBox<ShipOption> ship2Combo;
    private final JComboBox<String> speedCombo;
    private final JComboBox<String> simTypeCombo;
    private boolean confirmed = false;

    SimConfigDialog(JFrame parent) {
        super(parent, "Simulation Configuration", true);
        setLayout(new GridBagLayout());
        var gbc = new GridBagConstraints();
        gbc.insets = new Insets(6, 10, 6, 10);
        gbc.fill = GridBagConstraints.HORIZONTAL;

        // Simulation type
        gbc.gridx = 0; gbc.gridy = 0;
        add(new JLabel("Mode:"), gbc);
        gbc.gridx = 1; gbc.gridwidth = 2;
        simTypeCombo = new JComboBox<>(SIM_TYPES);
        simTypeCombo.setSelectedIndex(selectedSimType);
        add(simTypeCombo, gbc);

        // Ship 1
        gbc.gridx = 0; gbc.gridy = 1; gbc.gridwidth = 1;
        add(new JLabel("Ship 1 (Blue):"), gbc);
        gbc.gridx = 1; gbc.gridwidth = 2;
        ship1Combo = new JComboBox<>(SHIP_OPTIONS.toArray(new ShipOption[0]));
        ship1Combo.setSelectedIndex(selectedShip1Index);
        add(ship1Combo, gbc);

        // Ship 2
        gbc.gridx = 0; gbc.gridy = 2; gbc.gridwidth = 1;
        add(new JLabel("Ship 2 (Red):"), gbc);
        gbc.gridx = 1; gbc.gridwidth = 2;
        ship2Combo = new JComboBox<>(SHIP_OPTIONS.toArray(new ShipOption[0]));
        ship2Combo.setSelectedIndex(selectedShip2Index);
        add(ship2Combo, gbc);

        // Speed
        gbc.gridx = 0; gbc.gridy = 3; gbc.gridwidth = 1;
        add(new JLabel("Initial speed:"), gbc);
        gbc.gridx = 1; gbc.gridwidth = 2;
        String[] speeds = {"1x", "2x", "4x", "8x", "16x"};
        speedCombo = new JComboBox<>(speeds);
        speedCombo.setSelectedIndex(selectedSpeedMultiplier);
        add(speedCombo, gbc);

        // Buttons
        gbc.gridx = 0; gbc.gridy = 4; gbc.gridwidth = 3;
        gbc.fill = GridBagConstraints.NONE;
        gbc.anchor = GridBagConstraints.CENTER;
        var buttonPanel = new JPanel(new FlowLayout(FlowLayout.CENTER, 10, 0));
        var okButton = new JButton("Start");
        okButton.addActionListener(e -> {
            confirmed = true;
            selectedSimType = simTypeCombo.getSelectedIndex();
            selectedShip1Index = ship1Combo.getSelectedIndex();
            selectedShip2Index = ship2Combo.getSelectedIndex();
            selectedSpeedMultiplier = speedCombo.getSelectedIndex();
            dispose();
        });
        var cancelButton = new JButton("Cancel");
        cancelButton.addActionListener(e -> dispose());
        buttonPanel.add(okButton);
        buttonPanel.add(cancelButton);
        add(buttonPanel, gbc);

        getRootPane().setDefaultButton(okButton);
        pack();
        setMinimumSize(new Dimension(350, 200));
        setLocationRelativeTo(parent);
    }

    boolean isConfirmed() { return confirmed; }

    /** Returns the configured controllers (may have 0, 1, or 2 entries). */
    List<SubmarineController> getControllers() {
        var result = new ArrayList<SubmarineController>();
        var opt1 = (ShipOption) ship1Combo.getSelectedItem();
        var opt2 = (ShipOption) ship2Combo.getSelectedItem();
        if (opt1 != null && opt1.factory() != null) result.add(opt1.factory().get());
        if (opt2 != null && opt2.factory() != null) result.add(opt2.factory().get());
        return result;
    }

    /** Returns the vehicle configs matching the selected controllers. */
    List<VehicleConfig> getVehicleConfigs() {
        var result = new ArrayList<VehicleConfig>();
        var opt1 = (ShipOption) ship1Combo.getSelectedItem();
        var opt2 = (ShipOption) ship2Combo.getSelectedItem();
        if (opt1 != null && opt1.factory() != null) result.add(opt1.vehicleConfig());
        if (opt2 != null && opt2.factory() != null) result.add(opt2.vehicleConfig());
        return result;
    }

    int getSpeedMultiplier() {
        return new int[]{1, 2, 4, 8, 16}[speedCombo.getSelectedIndex()];
    }

    /** Returns true if competition mode is selected. */
    static boolean isCompetitionMode() { return selectedSimType == 1; }

    /** Returns display names for the current persistent settings. */
    static List<String> currentNames() {
        var result = new ArrayList<String>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.displayName());
        if (opt2.factory() != null) result.add(opt2.displayName());
        return result;
    }

    /** Returns controller factories for the current persistent settings. */
    static List<Supplier<SubmarineController>> currentFactories() {
        var result = new ArrayList<Supplier<SubmarineController>>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.factory());
        if (opt2.factory() != null) result.add(opt2.factory());
        return result;
    }

    /** Creates controllers using the current persistent settings (no dialog). */
    static List<SubmarineController> currentControllers() {
        var result = new ArrayList<SubmarineController>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.factory().get());
        if (opt2.factory() != null) result.add(opt2.factory().get());
        return result;
    }

    /** Returns vehicle configs for the current persistent settings. */
    static List<VehicleConfig> currentVehicleConfigs() {
        var result = new ArrayList<VehicleConfig>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.vehicleConfig());
        if (opt2.factory() != null) result.add(opt2.vehicleConfig());
        return result;
    }
}
