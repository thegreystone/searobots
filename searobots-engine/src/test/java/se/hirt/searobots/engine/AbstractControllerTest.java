package se.hirt.searobots.engine;

import se.hirt.searobots.api.SubmarineController;

/**
 * Base class for tests that should run with any SubmarineController implementation.
 * Subclasses provide the controller factory; the test methods exercise general
 * submarine behavior (survival, navigation, terrain avoidance).
 */
public abstract class AbstractControllerTest {

    /** Creates a fresh instance of the controller under test. */
    protected abstract SubmarineController createController();

    /** Display name for the controller (used in test output). */
    protected abstract String controllerName();
}
