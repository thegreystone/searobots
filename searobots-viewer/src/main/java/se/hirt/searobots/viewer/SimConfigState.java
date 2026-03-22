/*
 * Copyright (C) 2026 Marcus Hirt
 *                    (see TerrainViewer.java for full license text)
 */
package se.hirt.searobots.viewer;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.BaseAppState;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.simsilica.lemur.*;
import com.simsilica.lemur.component.SpringGridLayout;
import com.simsilica.lemur.style.ElementId;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.*;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Lemur-based simulation configuration dialog, replacing the Swing SimConfigDialog.
 * Shown as a modal popup via PopupState.
 */
final class SimConfigState extends BaseAppState {

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

    private static final String[] SIM_TYPES = {"Free patrol", "Competition (nav + combat)"};
    private static final String[] SPEED_OPTIONS = {"1x", "2x", "4x", "8x", "16x"};

    // Persistent selections (survive dialog close/reopen)
    static int selectedShip1Index = 2; // Claude Sub
    static int selectedShip2Index = 1; // Codex Sub
    static int selectedSpeedMultiplier = 1;
    static int selectedSimType = 0;

    private Container window;
    private int ship1Index;
    private int ship2Index;
    private int speedIndex;
    private int simTypeIndex;
    private Runnable onConfirm;
    private TextField seedField;

    // Seed: null means "use current seed", non-null means "use this seed"
    private Long pendingSeed;
    // Callback to set the seed externally
    java.util.function.LongConsumer onSeedChanged;
    java.util.function.LongSupplier seedSupplier;
    SimulationManager simManager;

    SimConfigState(Runnable onConfirm) {
        this.onConfirm = onConfirm;
    }

    @Override
    protected void initialize(Application app) {
    }

    @Override
    protected void onEnable() {
        var app = (SimpleApplication) getApplication();

        ship1Index = selectedShip1Index;
        ship2Index = selectedShip2Index;
        speedIndex = selectedSpeedMultiplier;
        simTypeIndex = selectedSimType;

        window = new Container(new SpringGridLayout(Axis.Y, Axis.X, FillMode.Even, FillMode.Last));
        window.addChild(new Label("Simulation Configuration", new ElementId("title")));
        window.addChild(new Label("")); // spacer

        // Mode selector
        addSelector(window, "Mode:", SIM_TYPES, simTypeIndex, i -> simTypeIndex = i);

        // Ship selectors
        var shipNames = SHIP_OPTIONS.stream().map(ShipOption::displayName).toArray(String[]::new);
        addSelector(window, "Ship 1 (Blue):", shipNames, ship1Index, i -> ship1Index = i);
        addSelector(window, "Ship 2 (Red):", shipNames, ship2Index, i -> ship2Index = i);

        // Speed selector
        addSelector(window, "Initial speed:", SPEED_OPTIONS, speedIndex, i -> speedIndex = i);

        window.addChild(new Label("")); // spacer

        // Seed field
        var seedRow = window.addChild(new Container(new SpringGridLayout(Axis.X, Axis.Y, FillMode.None, FillMode.Last)));
        seedRow.addChild(new Label("Seed (hex):"));
        seedField = seedRow.addChild(new TextField(
                seedSupplier != null ? Long.toHexString(seedSupplier.getAsLong()) : ""));
        pendingSeed = null;

        window.addChild(new Label("")); // spacer

        // Simulation options
        if (simManager != null) {
            window.addChild(new Label("-- Options --"));
            addToggle(window, "Pause on ship death", simManager.pauseOnDeath,
                    v -> simManager.pauseOnDeath = v);
            addToggle(window, "Pause on torpedo solution", simManager.pauseOnTorpedoSolution,
                    v -> simManager.pauseOnTorpedoSolution = v);
            addToggle(window, "Inject competition objectives", simManager.injectObjectives,
                    v -> simManager.injectObjectives = v);
        }

        window.addChild(new Label("")); // spacer

        // Buttons
        var buttonRow = window.addChild(new Container(new SpringGridLayout(Axis.X, Axis.Y)));
        var startBtn = buttonRow.addChild(new Button("Start"));
        startBtn.addClickCommands(b -> confirm());
        var cancelBtn = buttonRow.addChild(new Button("Cancel"));
        cancelBtn.addClickCommands(b -> close());

        // Center on screen
        window.setLocalTranslation(
                app.getCamera().getWidth() * 0.3f,
                app.getCamera().getHeight() * 0.7f,
                10);

        app.getGuiNode().attachChild(window);

        // Ctrl+V paste into seed field (use GLFW to check Ctrl modifier)
        var im = app.getInputManager();
        im.addMapping("PasteSeed", new KeyTrigger(KeyInput.KEY_V));
        im.addListener((ActionListener) (name, isPressed, tpf) -> {
            if (!isPressed || seedField == null) return;
            long win = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
            int lctrl = org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT_CONTROL);
            int rctrl = org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT_CONTROL);
            if (lctrl != org.lwjgl.glfw.GLFW.GLFW_PRESS && rctrl != org.lwjgl.glfw.GLFW.GLFW_PRESS) return;
            String clip = org.lwjgl.glfw.GLFW.glfwGetClipboardString(win);
            if (clip != null && !clip.isBlank()) {
                seedField.setText(clip.trim());
            }
        }, "PasteSeed");
    }

    private void addToggle(Container parent, String label, boolean initial,
                           java.util.function.Consumer<Boolean> onChange) {
        var btn = parent.addChild(new Button((initial ? "[x] " : "[ ] ") + label));
        final boolean[] state = {initial};
        btn.addClickCommands(b -> {
            state[0] = !state[0];
            btn.setText((state[0] ? "[x] " : "[ ] ") + label);
            onChange.accept(state[0]);
        });
    }

    private void addSelector(Container parent, String label, String[] options,
                             int selectedIndex, java.util.function.IntConsumer onChange) {
        var row = parent.addChild(new Container(new SpringGridLayout(Axis.X, Axis.Y, FillMode.None, FillMode.Last)));
        row.addChild(new Label(label));

        // Cycle-button: click to advance to the next option
        var btn = row.addChild(new Button(options[selectedIndex]));
        final int[] current = {selectedIndex};
        btn.addClickCommands(b -> {
            current[0] = (current[0] + 1) % options.length;
            btn.setText(options[current[0]]);
            onChange.accept(current[0]);
        });
    }

    private void confirm() {
        selectedSimType = simTypeIndex;
        selectedShip1Index = ship1Index;
        selectedShip2Index = ship2Index;
        selectedSpeedMultiplier = speedIndex;

        // Parse seed if changed
        if (seedField != null && onSeedChanged != null) {
            String text = seedField.getText().trim();
            if (!text.isEmpty()) {
                try {
                    long seed = Long.parseUnsignedLong(text, 16);
                    onSeedChanged.accept(seed);
                } catch (NumberFormatException e) {
                    try {
                        long seed = Long.parseLong(text);
                        onSeedChanged.accept(seed);
                    } catch (NumberFormatException e2) {
                        System.out.println("Invalid seed: " + text);
                    }
                }
            }
        }

        close();
        if (onConfirm != null) onConfirm.run();
    }

    private void close() {
        if (window != null) {
            window.removeFromParent();
            window = null;
        }
        setEnabled(false);
    }

    @Override
    protected void onDisable() {
        if (window != null) {
            window.removeFromParent();
            window = null;
        }
        var im = getApplication().getInputManager();
        if (im.hasMapping("PasteSeed")) {
            im.deleteMapping("PasteSeed");
        }
    }

    @Override
    protected void cleanup(Application app) {
    }

    // --- Static helpers matching the old SimConfigDialog API ---

    static boolean isCompetitionMode() { return selectedSimType == 1; }

    static List<SubmarineController> currentControllers() {
        var result = new ArrayList<SubmarineController>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.factory().get());
        if (opt2.factory() != null) result.add(opt2.factory().get());
        return result;
    }

    static List<VehicleConfig> currentVehicleConfigs() {
        var result = new ArrayList<VehicleConfig>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.vehicleConfig());
        if (opt2.factory() != null) result.add(opt2.vehicleConfig());
        return result;
    }

    static List<String> currentNames() {
        var result = new ArrayList<String>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.displayName());
        if (opt2.factory() != null) result.add(opt2.displayName());
        return result;
    }

    static List<Supplier<SubmarineController>> currentFactories() {
        var result = new ArrayList<Supplier<SubmarineController>>();
        var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
        var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
        if (opt1.factory() != null) result.add(opt1.factory());
        if (opt2.factory() != null) result.add(opt2.factory());
        return result;
    }

    static int getSpeedMultiplier() {
        return new int[]{1, 2, 4, 8, 16}[selectedSpeedMultiplier];
    }
}
