package se.hirt.searobots.engine.ships.claude;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.engine.WorldNavigationTest;

class ClaudeWorldNavigationTest extends WorldNavigationTest {
    @Override protected SubmarineController createController() { return new ClaudeAttackSub(); }
    @Override protected String controllerName() { return "ClaudeAttackSub"; }
}
