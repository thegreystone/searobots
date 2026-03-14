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
cd searobots-viewer
mvn exec:java
```

### Running Engine Tests

```bash
cd searobots-engine
mvn test
```

---

## Project Structure

```
searobots/
├── searobots-engine/        # Simulation engine (physics, world, sensors, replay)
├── searobots-api/           # Public Java API (SubmarineController, records)
├── searobots-viewer/        # 3D viewer (jMonkeyEngine-based)
└── docs/                    # Design docs, implementation plan
```

See the [implementation plan](docs/implementation-plan.md) for current
progress and next steps.

---

## Documentation

- **[Design Document](docs/DESIGN.md):** architecture, execution model,
  control interfaces, physics, sonar, damage model, and design principles.
- **[Default Submarine Design](docs/default-submarine-design.md):**
  behaviour design for the built-in `ObstacleAvoidanceSub`.
- **[Implementation Plan](docs/implementation-plan.md):** phased
  implementation roadmap.
