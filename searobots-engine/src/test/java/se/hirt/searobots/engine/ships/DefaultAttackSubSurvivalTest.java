package se.hirt.searobots.engine.ships;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.engine.SurvivalTest;

class DefaultAttackSubSurvivalTest extends SurvivalTest {
    @Override protected SubmarineController createController() { return new DefaultAttackSub(); }
    @Override protected String controllerName() { return "DefaultAttackSub"; }
}
