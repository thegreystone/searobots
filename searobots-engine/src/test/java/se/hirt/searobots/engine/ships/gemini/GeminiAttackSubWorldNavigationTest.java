/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.engine.WorldNavigationTest;

class GeminiAttackSubWorldNavigationTest extends WorldNavigationTest {
	@Override
	protected SubmarineController createController() {
		return new GeminiAttackSub();
	}

	@Override
	protected String controllerName() {
		return "Gemini";
	}
}
