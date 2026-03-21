package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.engine.WorldNavigationTest;

class CodexWorldNavigationTest extends WorldNavigationTest {
    @Override
    protected SubmarineController createController() {
        return new CodexAttackSub();
    }

    @Override
    protected String controllerName() {
        return "CodexAttackSub";
    }
}
