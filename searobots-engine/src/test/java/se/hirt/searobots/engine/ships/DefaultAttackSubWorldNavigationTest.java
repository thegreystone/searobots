package se.hirt.searobots.engine.ships;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.engine.WorldNavigationTest;

class DefaultAttackSubWorldNavigationTest extends WorldNavigationTest {
    @Override protected SubmarineController createController() { return new DefaultAttackSub(); }
    @Override protected String controllerName() { return "DefaultAttackSub"; }
}
