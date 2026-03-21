# SeaRobots

SeaRobots is a competitive autonomous underwater combat simulator, a spiritual
successor to [C-Robots](https://en.wikipedia.org/wiki/Crobots), set beneath the
waves. It is based on an idea and implementation from 2004, revised so that
submarines can participate in a language-agnostic manner and untrusted code
is isolated using modern sandboxing techniques rather than the JVM Security
Manager.

Participants submit programs that control autonomous submarines and their
torpedoes in a real-time simulation. Matches are sandboxed and fully
replayable in a 3D viewer.

---

## Vision

- Provide a safe arena for untrusted autonomous code to compete.
- Enforce strict real-time decision budgets: spend time thinking and you
  lose the chance to act.
- Enable deterministic simulation and full match replay. (Note: submarine
  implementations may use their own RNGs, so match outcomes are not
  guaranteed to be identical across runs.)
- Support rich 3D visualization of live and recorded matches.
- Encourage strategic tradeoffs between planning depth and responsiveness.

Each participant controls **one submarine** and **zero or more active
torpedoes**. All controlled entities share the same CPU budget per tick.

---

## Building

Requires Java 25+ and Maven 3.9+.

```bash
mvn clean install
```

### Running the Viewer

```bash
mvn clean install -DskipTests
mvn exec:java -pl searobots-viewer -Dexec.mainClass=se.hirt.searobots.viewer.TerrainViewer
```

To start with a specific seed:

```bash
mvn exec:java -pl searobots-viewer -Dexec.mainClass=se.hirt.searobots.viewer.TerrainViewer -Dexec.args="12345"
```

#### Viewer Controls

| Key | Action |
|-----|--------|
| Space | New random map |
| P | Pause / resume |
| N | Step one tick (when paused) |
| 1-5 | Speed: 1x, 2x, 4x, 8x, 16x |
| F2 | Configure simulation (ship picker) |
| F11 | Maximize window |
| +/- | Zoom in/out (2D view) |
| C | Toggle contour lines |
| T | Toggle trails |

#### Ship Configuration

Press **F2** (or Simulation > Configure Simulation) to open the ship
picker dialog. Select which controller to use for each ship slot:

- **Claude Sub** -- the Claude-authored attack submarine
- **Default Sub** -- the original reference implementation
- **Sub Drone** -- simple patrol submarine (no combat AI)
- **Ship Drone** -- noisy surface vessel (target practice)
- **(empty)** -- no ship in this slot

Settings persist across seed changes and simulation restarts.

### Running Engine Tests

```bash
mvn test -pl searobots-engine
```

### Running the Competition

The competition framework runs multiple controller implementations through
the same set of seeded maps and compares navigation performance metrics.

```bash
mvn exec:java -pl searobots-engine \
  -Dexec.mainClass=se.hirt.searobots.engine.SubmarineCompetition \
  -Dexec.classpathScope=test
```

This produces a table comparing metrics across 10 seeds:

- **Navigation**: waypoint chain length, hit accuracy, arrival rate, time to first waypoint
- **Stealth**: average/peak depth, average/peak noise (dB)
- **Efficiency**: average speed, path efficiency, time in normal patrol
- **Survivability**: time of first damage, time to death

To add a new competitor, edit `SubmarineCompetition.main()` and add your
controller factory to the competitors list.

---

## Project Structure

```
searobots/
├── searobots-api/           # Public Java API (SubmarineController, records)
├── searobots-engine/        # Simulation engine (physics, world, sensors)
│   └── ships/               # Controller implementations (temporary location)
│       ├── DefaultAttackSub     # Reference implementation
│       ├── SubmarineDrone       # Simple patrol drone
│       ├── TargetDrone          # Surface ship drone
│       └── claude/
│           └── ClaudeAttackSub  # Claude-authored implementation
├── searobots-viewer/        # 2D/3D viewer (Swing + jMonkeyEngine)
└── docs/                    # Design docs, implementation plan
```

### Writing a New Controller

1. Create a class implementing `SubmarineController` in a new subpackage
   under `se.hirt.searobots.engine.ships`.
2. Implement `name()`, `onMatchStart()`, and `onTick()`.
3. Add test classes extending `SurvivalTest` and `WorldNavigationTest`
   (provide your controller via `createController()`).
4. Add your controller to `SimConfigDialog.SHIP_OPTIONS` to make it
   available in the viewer.
5. Add it to `SubmarineCompetition.main()` to benchmark against others.

See the [implementation plan](docs/implementation-plan.md) for current
progress and next steps.

---

## Documentation

- **[Design Document](docs/DESIGN.md):** architecture, execution model,
  control interfaces, physics, sonar, damage model, and design principles.
- **[Default Submarine Design](docs/default-submarine-design.md):**
  behaviour design for the built-in `DefaultAttackSub`.
- **[Implementation Plan](docs/implementation-plan.md):** phased
  implementation roadmap.
- **[Physics Research](docs/physics-research.md):** research notes on
  submarine physics from games, simulations, and academic papers.
- **[Passive Sonar / TMA Design](docs/passive-sonar-tma-design.md):**
  engine-side Target Motion Analysis that provides controllers with
  progressively accurate contact data.
