/*
 * Copyright (C) 2026 Marcus Hirt
 *                    (see TerrainViewer.java for full license text)
 */
package se.hirt.searobots.viewer;

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.BaseAppState;
import com.simsilica.lemur.*;
import com.simsilica.lemur.component.SpringGridLayout;

/**
 * Lemur-based command palette shown on Escape. Provides quick access
 * to simulation actions and toggle states.
 */
final class CommandPaletteState extends BaseAppState {

    private Container window;

    // Callbacks wired by SubmarineScene3D
    Runnable onNewMap;
    Runnable onRerun;
    Runnable onConfigure;
    Runnable onFlatOcean;
    Runnable onLIsland;
    Runnable onSeedInput;

    // Toggle state references
    SimulationManager simManager;

    @Override
    protected void initialize(Application app) {
    }

    @Override
    protected void onEnable() {
        var app = (SimpleApplication) getApplication();

        window = new Container(new SpringGridLayout(Axis.Y, Axis.X, FillMode.Even, FillMode.Last));
        window.addChild(new Label("Command Palette"));
        window.addChild(new Label("")); // spacer

        // Simulation actions
        window.addChild(new Label("-- Simulation --"));
        addAction(window, "[Space] New random map", onNewMap);
        addAction(window, "[Ctrl+R] Re-run same seed", onRerun);
        addAction(window, "[F2] Configure simulation...", onConfigure);

        window.addChild(new Label("")); // spacer
        window.addChild(new Label("-- Scenarios --"));
        addAction(window, "Deep flat ocean", onFlatOcean);
        addAction(window, "L-Island recovery test", onLIsland);

        window.addChild(new Label("")); // spacer
        window.addChild(new Label("-- Options --"));

        // Toggles
        if (simManager != null) {
            addToggle(window, "Pause on ship death", simManager.pauseOnDeath,
                    v -> simManager.pauseOnDeath = v);
            addToggle(window, "Pause on torpedo solution", simManager.pauseOnTorpedoSolution,
                    v -> simManager.pauseOnTorpedoSolution = v);
            addToggle(window, "Inject competition objectives", simManager.injectObjectives,
                    v -> simManager.injectObjectives = v);
        }

        window.addChild(new Label("")); // spacer

        // Seed display/copy
        addAction(window, "Copy seed to clipboard", onSeedInput);

        window.addChild(new Label("")); // spacer
        var closeBtn = window.addChild(new Button("Close [Esc]"));
        closeBtn.addClickCommands(b -> setEnabled(false));

        window.addChild(new Label("")); // spacer
        var quitBtn = window.addChild(new Button("Quit"));
        quitBtn.addClickCommands(b -> getApplication().stop());

        // Center on screen
        window.setLocalTranslation(
                app.getCamera().getWidth() * 0.3f,
                app.getCamera().getHeight() * 0.75f,
                10);

        app.getGuiNode().attachChild(window);
    }

    private void addAction(Container parent, String label, Runnable action) {
        var btn = parent.addChild(new Button(label));
        btn.addClickCommands(b -> {
            setEnabled(false);
            if (action != null) action.run();
        });
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

    @Override
    protected void onDisable() {
        if (window != null) {
            window.removeFromParent();
            window = null;
        }
    }

    @Override
    protected void cleanup(Application app) {
    }
}
