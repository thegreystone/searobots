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
import se.hirt.searobots.engine.ControllerLoader;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Supplier;

/**
 * Lemur-based simulation configuration dialog, replacing the Swing SimConfigDialog. Shown as a
 * modal popup via PopupState.
 */
final class SimConfigState extends BaseAppState {

	record ShipOption(String displayName, String className, Supplier<SubmarineController> factory,
			VehicleConfig vehicleConfig) {
		@Override
		public String toString() {
			return displayName;
		}
	}

	private static final CopyOnWriteArrayList<ShipOption> SHIP_OPTIONS = initShipOptions();

	private static CopyOnWriteArrayList<ShipOption> initShipOptions() {
		var list = new CopyOnWriteArrayList<ShipOption>();
		list.add(new ShipOption("(empty)", null, null, null));
		for (var b : ControllerLoader.allBuiltIns()) {
			list.add(new ShipOption(b.displayName(), b.className(), b.factory(), b.vehicleConfig()));
		}
		ClassLoader extraLoader = resolveExtraClassLoader(System.getProperty("searobots.extraCp"));
		for (var sp : ControllerLoader.discoverServiceProviders(extraLoader)) {
			registerIfAbsent(list, sp);
		}
		String extraShips = System.getProperty("searobots.extraShips");
		if (extraShips != null && !extraShips.isBlank()) {
			for (String spec : extraShips.split(",")) {
				if (!spec.isBlank()) {
					try {
						registerIfAbsent(list, ControllerLoader.resolve(spec.trim(), extraLoader));
					} catch (RuntimeException e) {
						System.err
								.println("searobots.extraShips: cannot load '" + spec.trim() + "': " + e.getMessage());
					}
				}
			}
		}
		return list;
	}

	private static ClassLoader resolveExtraClassLoader(String extraCp) {
		ClassLoader base = SimConfigState.class.getClassLoader();
		if (extraCp == null || extraCp.isBlank()) {
			return base;
		}
		try {
			return ControllerLoader.createClassLoader(extraCp, base);
		} catch (Exception e) {
			System.err.println("Failed to load extra classpath '" + extraCp + "': " + e.getMessage());
			return base;
		}
	}

	private static int registerIfAbsent(List<ShipOption> list, ControllerLoader.ResolvedController rc) {
		for (int i = 0; i < list.size(); i++) {
			if (rc.className().equals(list.get(i).className())) {
				return i;
			}
		}
		list.add(new ShipOption(rc.displayName(), rc.className(), rc.factory(), rc.vehicleConfig()));
		return list.size() - 1;
	}

	private static final String[] SIM_TYPES = {"Free patrol", "Competition (nav + combat)"};
	private static final String[] SPEED_OPTIONS = {"1x", "2x", "4x", "8x", "16x"};

	// Persistent selections (survive dialog close/reopen)
	// TODO: Currently fixed to two vessel slots (2-way); extend to allow adding an arbitrary number
	// of boats into the water (multiple submarines, cargo ships / surface traffic, etc.).
	// Defaults: Claude vs Codex. Overridable via -Dsearobots.ship1 / -Dsearobots.ship2 or
	// --ship1 / --ship2 with a built-in name, index, or external SubmarineController FQCN.
	static int selectedShip1Index = shipIndexFromProperty("searobots.ship1", 2);
	static int selectedShip2Index = shipIndexFromProperty("searobots.ship2", 1);

	private static int shipIndexFromProperty(String property, int defaultIndex) {
		String value = System.getProperty(property);
		if (value == null || value.isBlank())
			return defaultIndex;
		int resolved = resolveOrRegisterShip(value, resolveExtraClassLoader(System.getProperty("searobots.extraCp")));
		if (resolved >= 0) {
			return resolved;
		}
		System.err.println(property + "=" + value + " matches no ship option; using default");
		return defaultIndex;
	}

	/**
	 * Resolves a ship selector (built-in display name substring, numeric index, or external
	 * {@link SubmarineController} FQCN) and dynamically registers external controllers into
	 * {@link #SHIP_OPTIONS} so they also appear in the interactive configuration dialog.
	 *
	 * @return index into {@link #SHIP_OPTIONS}, or {@code -1} if unresolved
	 */
	static int resolveOrRegisterShip(String spec, ClassLoader extraLoader) {
		if (spec == null || spec.isBlank()) {
			return -1;
		}
		String trimmed = spec.trim();
		String needle = trimmed.toLowerCase(Locale.ROOT);
		if ("empty".equals(needle) || "none".equals(needle)) {
			return 0;
		}
		// 1. Exact class-name match against already-registered options
		for (int i = 0; i < SHIP_OPTIONS.size(); i++) {
			if (trimmed.equals(SHIP_OPTIONS.get(i).className())) {
				return i;
			}
		}
		// 2. Display name substring match (for short names like "Claude", "Codex", "Gemini")
		if (!trimmed.contains(".")) {
			for (int i = 0; i < SHIP_OPTIONS.size(); i++) {
				if (SHIP_OPTIONS.get(i).displayName().toLowerCase(Locale.ROOT).contains(needle)) {
					return i;
				}
			}
		}
		// 3. Numeric index
		try {
			int index = Integer.parseInt(trimmed);
			if (index >= 0 && index < SHIP_OPTIONS.size()) {
				return index;
			}
		} catch (NumberFormatException ignored) {
		}
		// 4. External SubmarineController FQCN (or built-in alias via ControllerLoader)
		try {
			var rc = ControllerLoader.resolve(trimmed, extraLoader);
			return registerIfAbsent(SHIP_OPTIONS, rc);
		} catch (RuntimeException e) {
			return -1;
		}
	}

	static void applyCliOverrides(String ship1Spec, String ship2Spec, String extraCp, Integer speedMultiplier) {
		ClassLoader loader = resolveExtraClassLoader(
				extraCp != null ? extraCp : System.getProperty("searobots.extraCp"));
		if (ship1Spec != null && !ship1Spec.isBlank()) {
			int idx = resolveOrRegisterShip(ship1Spec, loader);
			if (idx >= 0) {
				selectedShip1Index = idx;
			} else {
				System.err.println("--ship1=" + ship1Spec + " matches no built-in ship or SubmarineController class");
			}
		}
		if (ship2Spec != null && !ship2Spec.isBlank()) {
			int idx = resolveOrRegisterShip(ship2Spec, loader);
			if (idx >= 0) {
				selectedShip2Index = idx;
			} else {
				System.err.println("--ship2=" + ship2Spec + " matches no built-in ship or SubmarineController class");
			}
		}
		if (speedMultiplier != null) {
			int[] speeds = {1, 2, 4, 8, 16};
			for (int i = 0; i < speeds.length; i++) {
				if (speeds[i] == speedMultiplier) {
					selectedSpeedMultiplier = i;
					break;
				}
			}
		}
	}

	static List<ShipOption> shipOptions() {
		return List.copyOf(SHIP_OPTIONS);
	}

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
	SubmarineScene3D scene; // for dialogOpen flag
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
		if (scene != null)
			scene.dialogOpen = true;
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
		var seedRow = window
				.addChild(new Container(new SpringGridLayout(Axis.X, Axis.Y, FillMode.None, FillMode.Last)));
		seedRow.addChild(new Label("Seed (hex):"));
		seedField = seedRow
				.addChild(new TextField(seedSupplier != null ? Long.toHexString(seedSupplier.getAsLong()) : ""));
		seedField.setPreferredSize(new com.jme3.math.Vector3f(200, 22, 0));
		pendingSeed = null;

		window.addChild(new Label("")); // spacer

		// Simulation options
		if (simManager != null) {
			window.addChild(new Label("-- Options --"));
			addToggle(window, "Pause on ship death", simManager.pauseOnDeath, v -> simManager.pauseOnDeath = v);
			addToggle(window, "Pause on torpedo solution", simManager.pauseOnTorpedoSolution,
					v -> simManager.pauseOnTorpedoSolution = v);
			addToggle(window, "Pause on torpedo launch", simManager.pauseOnTorpedoLaunch,
					v -> simManager.pauseOnTorpedoLaunch = v);
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
		window.setLocalTranslation(app.getCamera().getWidth() * 0.3f, app.getCamera().getHeight() * 0.7f, 10);

		app.getGuiNode().attachChild(window);

		// Ctrl+V paste into seed field (use GLFW to check Ctrl modifier)
		var im = app.getInputManager();
		im.addMapping("PasteSeed", new KeyTrigger(KeyInput.KEY_V));
		im.addListener((ActionListener) (name, isPressed, tpf) -> {
			if (!isPressed || seedField == null)
				return;
			long win = org.lwjgl.glfw.GLFW.glfwGetCurrentContext();
			int lctrl = org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT_CONTROL);
			int rctrl = org.lwjgl.glfw.GLFW.glfwGetKey(win, org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT_CONTROL);
			if (lctrl != org.lwjgl.glfw.GLFW.GLFW_PRESS && rctrl != org.lwjgl.glfw.GLFW.GLFW_PRESS)
				return;
			String clip = org.lwjgl.glfw.GLFW.glfwGetClipboardString(win);
			if (clip != null && !clip.isBlank()) {
				seedField.setText(clip.trim());
			}
		}, "PasteSeed");
	}

	private void addToggle(
		Container parent, String label, boolean initial, java.util.function.Consumer<Boolean> onChange) {
		var btn = parent.addChild(new Button((initial ? "[x] " : "[ ] ") + label));
		final boolean[] state = {initial};
		btn.addClickCommands(b -> {
			state[0] = !state[0];
			btn.setText((state[0] ? "[x] " : "[ ] ") + label);
			onChange.accept(state[0]);
		});
	}

	private void addSelector(
		Container parent, String label, String[] options, int selectedIndex, java.util.function.IntConsumer onChange) {
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
					onSeedChanged.accept(parseSeed(text));
				} catch (NumberFormatException e) {
					System.out.println("Invalid seed: " + text);
				}
			}
		}

		close();
		if (onConfirm != null)
			onConfirm.run();
	}

	/**
	 * A seed as typed by the user: hexadecimal, as the viewer shows seeds (title bar, this dialog),
	 * or failing that a signed decimal number.
	 *
	 * @throws NumberFormatException
	 *             if the text is neither
	 */
	static long parseSeed(String text) {
		try {
			return Long.parseUnsignedLong(text.trim(), 16);
		} catch (NumberFormatException e) {
			return Long.parseLong(text.trim());
		}
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
		if (scene != null)
			scene.dialogOpen = false;
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

	static boolean isCompetitionMode() {
		return selectedSimType == 1;
	}

	static List<SubmarineController> currentControllers() {
		var result = new ArrayList<SubmarineController>();
		var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
		var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
		if (opt1.factory() != null)
			result.add(opt1.factory().get());
		if (opt2.factory() != null)
			result.add(opt2.factory().get());
		return result;
	}

	static List<VehicleConfig> currentVehicleConfigs() {
		var result = new ArrayList<VehicleConfig>();
		var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
		var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
		if (opt1.factory() != null)
			result.add(opt1.vehicleConfig());
		if (opt2.factory() != null)
			result.add(opt2.vehicleConfig());
		return result;
	}

	static List<String> currentNames() {
		var result = new ArrayList<String>();
		var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
		var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
		if (opt1.factory() != null)
			result.add(opt1.displayName());
		if (opt2.factory() != null)
			result.add(opt2.displayName());
		return result;
	}

	static List<Supplier<SubmarineController>> currentFactories() {
		var result = new ArrayList<Supplier<SubmarineController>>();
		var opt1 = SHIP_OPTIONS.get(selectedShip1Index);
		var opt2 = SHIP_OPTIONS.get(selectedShip2Index);
		if (opt1.factory() != null)
			result.add(opt1.factory());
		if (opt2.factory() != null)
			result.add(opt2.factory());
		return result;
	}

	static int getSpeedMultiplier() {
		return new int[] {1, 2, 4, 8, 16}[selectedSpeedMultiplier];
	}
}
