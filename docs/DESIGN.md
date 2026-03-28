# SeaRobots: Design Document

This document describes the architecture, execution model, control interfaces,
physics, sonar, damage model, and design philosophy of SeaRobots.

For a project overview and build instructions, see the [README](../README.md).

---

## Legacy Project

SeaRobots was originally started in 2004–2006 as an in-process Java
simulation using the JVM Security Manager for isolation. That codebase
lives at `E:\Backups\Old\Studio\Data\opensource\searobots` and contains
early prototypes that never reached completion but informed this design.

**Reusable assets from the legacy project:**

- **3D models:** `submarine.obj`, `test-sub-reduced.obj`, `torpedo.obj`
  (with materials), which can be imported into modern jMonkeyEngine 3.x.
- **Design validations:** the original `Submersible` interface already used
  rudder `[-1.0, 1.0]`, elevator `[-1.0, 1.0]`, and engine power
  `[-1.0, 1.0]`, confirming the actuator range conventions. `Torpedoe`
  extended `Submersible`, validating the "torpedo is a mini-sub" decision.
- **Sonar model:** active ping vs passive sonar distinction, a good
  gameplay mechanic carried forward.
- **Thermal layers:** `DefaultTemperatureModel` had depth-based temperature
  zones, and the concept carries forward into the new physics model.
- **Countermeasures:** noted as a future idea (`releaseCounterMeasures()`),
  still a good extension point.

**What the new architecture replaces:**

- In-process threading (`SubmersibleContext` per sub in a `ThreadGroup`)
  → sandboxed processes over TCP.
- JVM Security Manager isolation → OS/VM-level sandboxing.
- Event-driven callbacks (`onSonarSignal`, `onHit`, `onHeartBeat`)
  → tick-based `BattleInput`/`BattleOutput` protocol.
- jMonkeyEngine 1.x API → jMonkeyEngine 3.x (completely different API).
- Ant build → modern build tooling (Maven).

---

## High-Level Architecture

```
┌──────────────────────┐
│   3D Viewer/Client   │  (jMonkeyEngine: replay & live stream consumer)
└──────────┬───────────┘
           │ replay file / live state stream
           ▼
┌──────────────────────┐
│  Simulation Engine   │  (authoritative, deterministic, headless)
└──────────┬───────────┘
           │ TCP (framed protocol)
           ▼
┌──────────────────────┐
│ Arena / Orchestrator  │  (sandbox lifecycle, resource limits)
└──────────┬───────────┘
           │ one sandbox per participant
           ▼
┌──────────────────────┐
│ Participant Process   │  (untrusted controller, Java SDK)
└──────────────────────┘
```

---

## Core Components

### Simulation Engine (Trusted)

The engine is the **sole authority** over world state.

Responsibilities:

- Procedural underwater world generation (terrain, thermals, currents).
- Fixed-step physics simulation (see [Physics Model](#physics-model)).
- Collision detection.
- Sonar and sensor modeling.
- Damage resolution (see [Damage Model](#damage-model)).
- Weapon lifecycle.
- Tick scheduling and deadline enforcement.
- Replay logging.

Controllers **cannot** mutate world state directly. They can only propose
actuator settings; the engine decides what happens.

### Arena / Orchestrator

The orchestrator manages participant isolation and match lifecycle.

- Launches one sandboxed VM (or container) per participant.
- Each VM runs a **guest agent** (arena-provided, trusted) that manages
  controller processes inside the VM and holds a single TCP connection
  back to the engine.
- Establishes the TCP connection between the engine and the guest agent.
- Enforces hard resource limits on the VM:
  - CPU quota (shared across submarine + all torpedo processes)
  - Memory ceiling
  - Process/thread cap
  - Read-only root filesystem, ephemeral scratch only
  - No outbound network access (except the engine connection)
- Terminates participants on resource violation or protocol breach.
- Schedules matches, collects results, destroys sandboxes.

Recommended isolation tiers (strongest first):

| Tier | Technology | Notes |
|------|-----------|-------|
| Strongest | MicroVM / Kata Containers | Hardware-virtualisation backed |
| Good | gVisor | Userspace kernel interposition |
| Minimum | Hardened Docker (seccomp + AppArmor + cgroups) | Acceptable for trusted environments |

### Participant Process (Untrusted)

Each participant runs in its own VM/container, which contains:

- A **guest agent** (trusted, arena-provided) that manages process
  lifecycle and bridges to the engine via TCP.
- A **submarine controller process** running the participant's
  `SubmarineController`, which communicates with the guest agent via a
  Unix domain socket.
- Zero or more **torpedo controller processes**, each running the
  participant's `TorpedoController`, spawned by the guest agent on
  launch, each with its own Unix domain socket.

All processes share a single **CPU quota**. More active torpedoes
means less compute time for every entity.

### 3D Viewer

The viewer is a standalone **jMonkeyEngine 3.8** application with
**Lemur GUI**. It renders live simulations with full camera control.

Features:

- Renders submarines, torpedoes, terrain, water surface, sonar markers,
  torpedo trails, waypoints, contact estimates, and explosion effects.
- Seven camera modes: Orbit, Chase, Target, Periscope, Free Look,
  Fly-by, and **Director** (automatic cinematic camera).
- 2D tactical map overlay (M key) with Java2D-to-texture rendering.
- Lemur-based GUI: simulation configuration (F2), render settings (F3),
  pause-on-event toggles, seed input with clipboard support.
- Particle effects: propeller wake bubbles, torpedo explosion fireballs
  with debris, death bubbles from sinking subs.
- Sub death animation: engine cuts, propeller freewheels to a stop,
  ballast floods, sub sinks and settles on the seabed matching the
  terrain slope.

**Cinematic Director** (default camera mode): event-driven automatic
camera system that creates TV-broadcast-style coverage. Structured idle
sequence cycles through all entities (close underwater, birds-eye,
tactical overview). Hard-cuts to torpedo launches, terminal approaches,
and explosions. Terrain-aware detonation camera with slow orbit. Smooth
spline transitions between shots. Water fades for overhead views.
North-up orientation for situational awareness.

The viewer never influences simulation state.

---

## Execution Model

### Fixed-Step Simulation

The engine advances in fixed discrete time steps at **50 Hz** (20 ms per tick).
Simulation time is separated from wall-clock time: physics uses a constant
`dt`, but the arena waits only a bounded wall-clock duration for each
controller response.

Per tick:

1. Engine computes current world state.
2. Engine derives sensor snapshots for each process (submarines and
   active torpedoes).
3. Engine sends `SubmarineInput` to each submarine process and
   `TorpedoInput` to each active torpedo process, **all in parallel**.
4. Deadline timer starts.
5. Each process computes and returns its output independently.
6. **If response arrives before deadline:** commands are applied.
7. **If deadline expires:** previous actuator settings remain in effect
   (per process, independently).
8. Engine advances physics using the accepted actuator vectors.
9. Engine handles torpedo launches (spawns new torpedo processes).
10. Engine writes tick to replay log.

Late responses are **discarded, never retroactively applied**.

### Deadline Semantics

The per-tick deadline creates a core strategic tension:

- **Reactive bots** respond quickly every tick.
- **Planning bots** may intentionally miss ticks to compute deeper strategies.
- **More torpedoes in the water = less CPU for everyone.** Every active
  torpedo runs in its own process, sharing the same CPU quota. Fire all
  your torpedoes at once and your submarine (and every torpedo) gets
  less time to think per tick.

Poor implementations are punished naturally; no special-case rules needed.

### Match Parameters

Matches are configured with parameters that define the environment and
rules. These are communicated to the submarine via `MatchContext` in
`onMatchStart()`.

Key configurable parameters:

| Parameter | Description | Example |
|-----------|-------------|---------|
| Torpedo count | Total torpedoes per submarine | 3–8 |
| Decoy count | Countermeasure decoys per submarine | 3 |
| Vehicle configs | Per-participant vehicle type and physics parameters | submarine, surface ship |
| Tick rate | Simulation ticks per second | 20–50 Hz |
| Match duration | Maximum ticks before draw | 6000 (2 min at 50 Hz) |
| World seed | Procedural generation seed | random long |
| Blast radius | Torpedo explosion radius | 50 m |
| Max fuse radius | Upper bound on configurable fuse radius | 30 m |
| Min fuse radius | Lower bound on configurable fuse radius | 5 m |
| Starting HP | Hit points per submarine | 200 |
| Rated depth | Submarine's rated max operating depth | −400 m |
| Crush depth | Absolute hull failure: instant destruction | −700 m |

The torpedo count is a deliberate balance lever. Low counts (e.g. 3)
create a tense, conservative game where every launch must count. High
counts (e.g. 8) enable aggressive tactics but increase CPU pressure.

---

## Control Model

### Submarine and Torpedo: Separate Controllers

Participants provide **two** controller implementations in their
submission:

1. **`SubmarineController`**: controls the submarine.
2. **`TorpedoController`**: controls each torpedo autonomously after
   launch.

Each runs in its **own process**. The submarine does not guide torpedoes
after launch. The torpedo controller is self-contained: it receives its
own sensor data and makes its own decisions.

All processes (submarine + all active torpedoes) share a single **CPU
quota** via cgroup. This is a core gameplay mechanic: every torpedo in
the water competes with the submarine and every other torpedo for compute
time. Launching more torpedoes degrades the responsiveness of all
controlled entities.

### SubmarineController

```java
public interface SubmarineController {

    default void onMatchStart(MatchContext context) {}

    void onTick(SubmarineInput input, SubmarineOutput output);

    default void onMatchEnd(MatchResult result) {}
}
```

### TorpedoController

```java
public interface TorpedoController {

    default void onLaunch(TorpedoLaunchContext context) {}

    void onTick(TorpedoInput input, TorpedoOutput output);
}
```

`onLaunch()` is called once when the torpedo process starts, before the
first tick. It receives the `missionData` from the `TorpedoLaunchCommand`
and the launch parameters, allowing the torpedo to configure its
behaviour based on instructions from the submarine.

A new process is spawned for each torpedo at launch time, running the
participant's `TorpedoController` implementation. The torpedo process is
destroyed when the torpedo's lifecycle ends.

### SubmarineInput

```java
public interface SubmarineInput {
    long tick();
    double deltaTimeSeconds();
    long deadlineNanos();

    SubmarineState self();              // pose, velocity, HP, torpedoes remaining
    SonarSnapshot sonar();              // no FFI
    List<TorpedoPosition> ownTorpedoes(); // exact positions, no control
    EnvironmentSnapshot environment();
}
```

The submarine always knows the exact position of its own active
torpedoes via `ownTorpedoes()`, but has no control over them. This is
telemetry only, useful for evasive action if your own torpedo starts
heading back toward you.

### SubmarineOutput

```java
public interface SubmarineOutput {
    void setRudder(double value);
    void setSternPlanes(double value);
    void setThrottle(double value);
    void setBallast(double value);
    void activeSonarPing();
    void launchTorpedo(TorpedoLaunchCommand command);
    void launchDecoy(double bearing);
    void publishContactEstimate(ContactEstimate estimate);
}
```

#### Contact Estimates

Submarines can optionally publish their best guesses for contact
positions via `publishContactEstimate()`. A `ContactEstimate` contains
a position, a confidence value, and an optional label. These estimates
have no effect on simulation: the engine simply records them into the
replay stream. The viewer renders them as sonar markers on the tactical
display, updated each tick with the submarine's latest estimates.

This gives spectators and replay viewers insight into what the submarine
"thinks" it knows, showing the gap between estimated and actual contact
positions.

### TorpedoInput

```java
public interface TorpedoInput {
    long tick();
    double deltaTimeSeconds();
    long deadlineNanos();

    TorpedoState self();
    SonarSnapshot sonar();    // no FFI: cannot distinguish friend from foe
    EnvironmentSnapshot environment();
}
```

### TorpedoOutput

```java
public interface TorpedoOutput {
    void setRudder(double value);
    void setPlanes(double value);
    void setThrottle(double value);
    void activeSonarPing();
    void detonate();
}
```

---

## Controlled Entities

### Submarine

**Sensors received:**

| Sensor | Description |
|--------|-------------|
| Pose | Position, orientation |
| Velocity | Linear and angular velocity |
| Sonar (passive) | Contacts with bearing, speed, signal excess, measurement uncertainty (**no FFI**) |
| Sonar (active ping) | Bearing + range with measurement uncertainty, but reveals own position (**no FFI**) |
| Own torpedo positions | Exact position of all own active torpedoes (see below) |
| Environment | Water temperature, pressure, current at own position |
| Damage | Current hit points, damage events |
| Weapons | Torpedoes remaining (of match-configured total) |

**Always available (from match start):**

| Information | Source |
|-------------|--------|
| Sea floor heightmap | `MatchContext`: full terrain topology |
| Thermal layer depths | `MatchContext`: thermocline boundaries |
| Current field | `MatchContext`: depth-varying current map |
| Own position, heading, depth | `SubmarineState` every tick |
| Own torpedo positions | Telemetry in `SubmarineInput` every tick |
| Torpedoes remaining | `SubmarineState` every tick |

**Never available directly:**

| Information | How to obtain |
|-------------|--------------|
| Enemy positions | Sonar only (bearing + signal strength) |
| Enemy count or identity | Cannot (no FFI) |
| Contact type (sub vs torpedo) | Cannot (no FFI) |

**Actuators:**

| Actuator | Range | Description |
|----------|-------|-------------|
| Rudder | \[-1.0, 1.0\] | Yaw control |
| Stern planes | \[-1.0, 1.0\] | Pitch/depth control |
| Throttle | \[0.0, 1.0\] | Engine power |
| Ballast | \[0.0, 1.0\] | Buoyancy control |
| Launch torpedo | (action) | Issues a `TorpedoLaunchCommand` |
| Launch decoy | (action) | Deploys a noisemaker countermeasure in a direction |
| Active sonar ping | (action) | Triggers a ping (reveals own position) |

*(See `SubmarineOutput` in the Control Model section for the Java
interface.)*

### Torpedoes

Torpedoes are **mini-submarines**: they have their own sonar, control
surfaces, and internal state. After launch, each torpedo runs
**autonomously** in its own process using the participant's
`TorpedoController`. The submarine has no control over it.

Unlike submarines, torpedoes have **no ballast system**. They are slightly
negatively buoyant and rely entirely on hydrodynamic lift from forward
motion to maintain depth. Below a minimum speed (~3 m/s), lift is
insufficient and the torpedo sinks. When fuel runs out, drag bleeds off
speed, lift fails, and the torpedo sinks to the sea floor and is
destroyed. This creates natural end-of-life without an arbitrary timer,
and means a torpedo that loses its target and slows down in a tight
search pattern will eventually sink on its own.

Fuel is limited (180 seconds at full throttle). At cruise speed
(~25 m/s), this gives roughly 4.5 km of powered range plus coast time.
Smart torpedo controllers reduce speed in the terminal phase for better
maneuverability (turn radius halves when speed halves), at the cost of
fuel efficiency. Long enough to execute a search pattern and re-acquire
after a miss, short enough that a well-executed evasion can outlast it.

Torpedo guidance must manage speed carefully: throttle down too much and
you lose the ability to steer, and eventually sink.

Torpedoes carry a **proximity fuse**: the engine automatically detonates
the warhead when the torpedo passes within the configured fuse radius of
any submarine (including the owner). The fuse radius is set at launch
time and cannot be changed in flight. The participant can also command
manual detonation at any time.

**There is no friend-or-foe identification (FFI).** The proximity fuse
triggers on *any* submarine, and active sonar pings from a torpedo are
indistinguishable from those of a submarine. This has important tactical
consequences:

- A torpedo sent to hunt with active sonar may come back toward its
  owner if it fails to find the enemy.
- The torpedo's own sonar cannot distinguish friend from foe, so it will
  chase whatever it detects.
- Tight formations or aggressive pursuit risk self-damage from your own
  weapons.

Torpedoes are **fire-and-forget**: there is no communication link
between the submarine and its launched torpedoes. The submarine can
only track its own torpedoes through passive sonar (they are loud).
This means:

- You hear your own torpedoes just like the enemy does.
- You can estimate their position from sonar contacts, but have no
  precise telemetry.
- A torpedo that circles back toward you is a real threat: you must
  evade your own weapon.
- The torpedo itself is "dumb" about friend-or-foe and will chase
  whatever it detects on sonar.

**Torpedo launch transient:** Launching a torpedo creates a detectable
noise event on the launching submarine. Tube flooding and ejection
produce a transient noise spike (~1-2 seconds of elevated source level)
that is audible on passive sonar, subject to normal detection rules
(baffles, terrain occlusion, thermocline). An alert defender who detects
the launch transient has a few seconds of warning before the torpedo
arrives. Tactically, this means:

- Launching during your own cavitation sprint masks the transient in
  your existing noise.
- Launching from the target's baffles means they may not hear it.
- Launching from a quiet tailing position is stealthy for you, but
  the launch transient briefly spikes your noise.

**Torpedoes as noise sources:** A running torpedo is loud (~115-140 dB
source level depending on speed). Everyone on the map can hear a
torpedo in the water.
This changes the tactical picture immediately: the target knows it is
under attack, and so does everyone else.

**Torpedoes as decoys:** Because torpedoes have sonar and are
indistinguishable from submarines on passive sonar (at similar speeds),
they can be used as decoys. Launch a torpedo at submarine-like speed with
active sonar pinging to draw enemy attention, waste their torpedoes, or
mask your real position. This costs a torpedo from your limited inventory
but can be worth the deception.

**Launch configuration:**

The current implementation launches torpedoes from fixed forward tubes.
They leave the submarine on the submarine's current heading and pitch.
`bearing` and `pitch` on `TorpedoLaunchCommand` describe the requested
shot geometry, but they do not rotate the torpedo at the instant of
ejection.

The `TorpedoLaunchCommand` currently has this shape:

```java
public record TorpedoLaunchCommand(
    double bearing,
    double pitch,
    double fuseRadius,
    String missionData
) {}
```

The `missionData` field is an opaque string passed to the torpedo
controller when it starts. The submarine can encode mission parameters
(e.g. "search pattern", "head to waypoint X", "decoy mode") in whatever
format it likes. The engine does not inspect or interpret this data;
it just delivers it. This is the only communication channel from
submarine to torpedo; after launch, there is no further contact.

The `TorpedoLaunchContext` given to the torpedo controller contains the
actual tube-exit pose (`launchHeading`, `launchPitch`), which is the
launching submarine's heading and pitch at the moment of launch.

### Decoys

Submarines carry a limited supply of **decoy countermeasures** (default
3, configurable in `MatchConfig`). Decoys are small noisemakers that
confuse incoming torpedoes and provide a defensive option separate from
the offensive torpedo inventory.

**Decoy characteristics:**
- Launched in a specified direction via `SubmarineOutput.launchDecoy(double bearing)`.
- Travel in a straight line at moderate speed, making noise.
- No guidance, no sonar, no controller. Fire-and-forget noisemakers.
- Source level ~95-100 dB (comparable to a moderate-speed submarine).
- Short lifespan (30-60 seconds), then they go quiet and sink.
- No FFI: an incoming torpedo's sonar cannot distinguish a decoy from
  a real submarine. It chases whatever is loudest.

**Tactical role:** Decoys are a limited defensive resource. The decision
to deploy one is a tradeoff: save it and try to evade with maneuvers
(free but risky), or spend it for a reliable escape (safe but costly,
only 3 per match).

Effective decoy use requires timing:
- Deploy too early: the torpedo may not have acquired the decoy, wasted.
- Deploy too late: the torpedo is too close to be drawn away.
- Deploy in the right geometry: the decoy should be between you and the
  torpedo, or offset enough to pull the torpedo's sonar away from you
  as you go quiet and slip away.

**Decoys vs torpedo-as-decoy:** The original design allowed launching
unarmed torpedoes as decoys. This remains possible, but dedicated decoys
are cheaper (don't consume torpedo inventory), simpler (no controller
needed), and purpose-built for the role. Torpedoes are too valuable to
waste as noisemakers when you only have 4.

**Distinguishing torpedoes from submarines on sonar:**
Torpedoes are significantly faster (~25 m/s) and louder (~140 dB)
than submarines. The blade-rate speed estimate available on sonar
contacts reports high speed for a torpedo, giving a smart controller
a strong clue that "this contact is probably a torpedo, not a submarine."
There is still no FFI (cannot tell whose torpedo), but the speed and
noise signature make torpedoes and submarines distinguishable in
practice.

**Sensors received:**

- Pose and velocity
- Sonar contacts (no FFI: cannot distinguish friend from foe)
- Status (fuel remaining, armed state, fuse radius, speed)
- Terrain map (received via `TorpedoLaunchContext` at launch, same
  heightmap as the submarine)

**Actuators:**

| Actuator | Range | Description |
|----------|-------|-------------|
| Rudder | \[-1.0, 1.0\] | Yaw control |
| Planes | \[-1.0, 1.0\] | Pitch/depth control |
| Throttle | \[0.0, 1.0\] | Engine power |
| Active sonar ping | (action) | Triggers a ping (indistinguishable from submarine ping) |
| Detonate | (action) | Manual detonation command (armed torpedoes only) |

*(See `TorpedoOutput` in the Control Model section for the Java
interface.)*

**Explosion model:** Detonation applies damage and impulse to all
submarines within blast radius (configurable, default 50m). Damage
uses quadratic falloff: `(1 - d/r)²`, where d is distance and r is
blast radius. Impulse is physics-driven: the force vector goes from
the blast center to the submarine's center of mass, so the direction
of the blast relative to the submarine matters:
- Blast below pushes up (and possibly into the surface).
- Blast to port shoves starboard and induces roll.
- Blast behind accelerates forward.
- Off-center hits apply angular impulse (torque), disrupting heading
  and pitch. A near-miss can spin you into nearby terrain.

**Torpedo lifecycle:** active until detonation (proximity or manual),
fuel depletion, terrain collision, or loss of control (speed below
minimum, sinks to sea floor). If no new commands arrive, previous
actuator values remain in effect.

### Example: Submarine and Torpedo Controllers

**Submarine controller:**

```java
public final class MySubmarine implements SubmarineController {

    @Override
    public void onTick(SubmarineInput input, SubmarineOutput output) {
        // Check if any of our own torpedoes are heading back toward us
        for (var torp : input.ownTorpedoes()) {
            if (isHeadingTowardUs(torp, input.self())) {
                evade(input, output);
                return;
            }
        }

        // Normal hunting behaviour
        var contacts = input.sonar().contacts();
        if (!contacts.isEmpty()) {
            var nearest = closest(contacts);
            output.setRudder(steerToward(nearest, input.self()));
            output.setThrottle(0.8);
        }
    }
}
```

**Torpedo controller** (runs in its own process, autonomously):

```java
public final class MyTorpedo implements TorpedoController {

    @Override
    public void onTick(TorpedoInput input, TorpedoOutput output) {
        var contacts = input.sonar().contacts();
        if (contacts.isEmpty()) {
            // Search pattern: keep moving to maintain control authority
            output.setThrottle(0.7);
            output.setRudder(searchPattern(input));
        } else {
            // Chase nearest contact (no FFI, might be our own sub!)
            var target = closest(contacts);
            output.setRudder(steerToward(target, input.self()));
            output.setThrottle(1.0);
        }
    }
}
```

**Submission entry point:**

```java
public final class BotMain {
    public static void main(String[] args) {
        SeaRobots.run(new MySubmarine(), new MyTorpedo(), args);
    }
}
```

The submarine and torpedo controllers are **completely independent**:
they share no memory, no state, and no communication channel. The only
link is the torpedo position telemetry the engine provides to the
submarine.

---

## Communication Layer

### Two-Tier Transport

Communication uses a two-tier architecture with a **guest agent** inside
each participant's VM that bridges the engine and the controller
processes:

```
                             VM / container boundary
                                      │
Engine (host) ◄══ TCP ══► Guest Agent ◄══ Unix socket ══► Submarine process
                                      ◄══ Unix socket ══► Torpedo process 1
                                      ◄══ Unix socket ══► Torpedo process 2
                                      │
```

**Outer leg (Engine ↔ Guest Agent):** One TCP connection per participant.
The guest agent multiplexes all submarine and torpedo traffic over this
single connection.

**Inner leg (Guest Agent ↔ Controller processes):** Unix domain sockets.
The guest agent creates a socket for each controller process it spawns.
Low latency, no TCP overhead, trivial per-process setup.

**User code never sees TCP.** The SDK connects to a local Unix socket
provided by the guest agent. From the bot author's perspective, the
communication layer is invisible.

### Guest Agent

The guest agent is a small trusted process that runs inside each
participant's VM. It is provided by the arena infrastructure, not by
the participant.

Responsibilities:

- Holds the single TCP connection to the engine.
- Spawns the submarine controller process at match start.
- Spawns a new torpedo controller process each time the engine signals
  a torpedo launch.
- Routes per-tick messages between the engine and the appropriate
  controller process via Unix domain sockets.
- Destroys torpedo processes when their lifecycle ends.
- Enforces per-process resource limits within the VM.
- Captures stdout/stderr from controller processes for debugging.
- Reports process crashes back to the engine.
- Handles clean shutdown on match end.

### Wire Protocol

The same framed binary protocol is used across both transport tiers;
only the routing layer differs.

**Engine ↔ Guest Agent (TCP, multiplexed):**

| Message | Direction | Contents |
|---------|-----------|----------|
| `MATCH_START` | Engine → Agent | Match metadata, world parameters |
| `SUBMARINE_INPUT` | Engine → Agent | Tick, deadline, sub state, sonar, own torpedo positions |
| `SUBMARINE_OUTPUT` | Agent → Engine | Sub actuators, optional launch command |
| `SPAWN_TORPEDO` | Engine → Agent | Torpedo ID, launch parameters; agent spawns new process |
| `TORPEDO_INPUT` | Engine → Agent | Torpedo ID, tick, deadline, torpedo state, sonar |
| `TORPEDO_OUTPUT` | Agent → Engine | Torpedo ID, torpedo actuators |
| `DESTROY_TORPEDO` | Engine → Agent | Torpedo ID; agent kills the process |
| `MATCH_END` | Engine → Agent | Match result, final scores |

**Guest Agent ↔ Controller processes (Unix domain sockets):**

The agent strips the multiplexing envelope and forwards the payload
directly to the appropriate process. Controller processes see only
their own input/output messages. They are unaware of the agent or
other processes.

### Timeout Behaviour

| Entity | On missed deadline |
|--------|--------------------|
| Submarine | Previous actuator settings remain active |
| Each torpedo (independently) | Previous actuator settings remain active |
| New torpedo launches | Ignored: launch commands must arrive in time |

Because each torpedo is a separate process with its own Unix socket, a
slow torpedo does not block the submarine or other torpedoes; each
misses its own deadline independently.

---

## Java SDK

The SDK hides the entire communication layer. Bot authors never deal with
sockets, framing, serialization, or deadline tracking.

### Launcher

The SDK provides a single entry point that registers both controllers:

```java
public final class BotMain {
    public static void main(String[] args) {
        SeaRobots.run(new MySubmarine(), new MyTorpedo(), args);
    }
}
```

The SDK detects whether it is running as a submarine or torpedo process
(based on engine handshake) and dispatches to the appropriate controller.

### SDK Internals

Each process (submarine or torpedo) runs the same SDK structure:

| Thread | Responsibility |
|--------|---------------|
| Network reader | Reads framed TCP messages |
| Tick coordinator | Tracks current tick, deadline, staleness |
| Controller thread | Invokes `onTick()` on the appropriate controller |
| Network writer | Sends actuator packets |

The public API is **single-callback-at-a-time** by default. The SDK
manages concurrency internally.

### Test Kit

A testing harness for offline development of both controller types:

```java
@Test
void submarineShouldTurnTowardContact() {
    var controller = new MySubmarine();
    var input = SubmarineTestInputs.tick()
        .withSonarContact(bearing(35.0), range(1200.0))
        .build();
    var output = SubmarineTestOutputs.create();

    controller.onTick(input, output);

    assertTrue(output.rudder() > 0.0);
}

@Test
void torpedoShouldChaseContact() {
    var controller = new MyTorpedo();
    var input = TorpedoTestInputs.tick()
        .withSonarContact(bearing(10.0), range(200.0))
        .build();
    var output = TorpedoTestOutputs.create();

    controller.onTick(input, output);

    assertTrue(output.throttle() > 0.5);
}
```

---

## Physics Model

The simulation uses a **simplified 6-DOF rigid-body model** based on
established marine vehicle dynamics literature. The goal is physical
plausibility (not research-grade fidelity), so that submarines and
torpedoes behave in a way that feels right and rewards good control
strategies.

### Equations of Motion

The core model follows Fossen's compact matrix-vector formulation:

```
η̇ = J(η) · ν
M · ν̇ + C(ν) · ν + D(ν) · ν + g(η) = τ
```

Where:
- **η**: position and orientation in the world frame
- **ν**: linear and angular velocity in the body frame
- **M**: system inertia (rigid-body + added mass, diagonal approximation)
- **C(ν)**: Coriolis and centripetal forces
- **D(ν)**: hydrodynamic damping (linear + quadratic)
- **g(η)**: gravitational and buoyancy restoring forces
- **τ**: external forces: propulsion, control surfaces, currents, explosions

This is a system of 12 first-order ODEs integrated at each tick using
fixed-timestep **RK4** (4th-order Runge-Kutta) for determinism and
stability.

### Simplifications for the Game

| Component | Simplified model |
|-----------|-----------------|
| Added mass | Diagonal approximation (6 coefficients) |
| Damping | Linear + quadratic diagonal + key cross-terms |
| Control surfaces | Linear lift slope with stall limit |
| Propulsion | Thrust as function of throttle, first-order lag |
| Ballast | Net buoyancy variable with rate limit |
| Currents | Depth-varying 2D current field |

### Data-Driven Vehicle Physics (VehicleConfig)

All vehicle types (submarines, surface ships, torpedoes) use the same
physics engine with different parameters. Each entity carries a
`VehicleConfig` record containing all physics constants: mass, drag,
thrust, noise profile, hull dimensions, and vehicle type flags.

Preset configurations:

| Vehicle | Mass | Max Speed | Hull | Key traits |
|---------|------|-----------|------|------------|
| Submarine | 700 tons | ~15 m/s | 65m x 8m | Full depth control, ballast, quiet at low speed |
| Surface ship | 5000 tons | ~8 m/s | 150m x 30m | Surface-locked, very loud, no depth control |
| Torpedo | 300 kg | ~25 m/s | small | No ballast, minimum speed to maintain depth, 180s fuel |

**Surface-locked vehicles** stay at z=0. The physics skips pitch
updates, ballast processing, and vertical dynamics. Terrain collisions
only occur when the surface vessel passes over terrain above sea level
(islands, shoals). Surface ships are always loud: high base source
level (100 dB), surface noise penalty, and easy cavitation from
surface-running propellers.

**Engine clutch mechanic** applies to all vehicles: disengaging the
clutch lets the prop freewheel (no thrust, no engine braking, reduced
noise). Submarines use this for sprint-and-drift tactics.

Torpedoes use the same framework with different coefficients
(smaller body, higher thrust-to-drag ratio, no ballast). Torpedo control
surfaces are only effective when there is sufficient water flow, and control
authority is proportional to speed squared, so a slow or stalled torpedo
cannot steer.

### Numerical Integration

- Fixed-timestep RK4 at the simulation tick rate (e.g. 50 Hz).
- Substeps may be used for stability at lower tick rates.
- Deterministic: identical results given the same inputs and seed.

### Key References

- **Fossen, T.I.** *Handbook of Marine Craft Hydrodynamics and Motion
  Control*, 2nd ed., Wiley, 2021. The primary reference for the 6-DOF
  formulation and SNAME notation.
- **Prestero, T.J.** *Verification of a Six-Degree of Freedom Simulation
  Model for the REMUS AUV*, MIT/WHOI, 2001. Excellent practical derivation
  of all forces for a torpedo-shaped vehicle from geometry.
  ([MIT DSpace](https://dspace.mit.edu/handle/1721.1/65068))
- **Gertler, M. & Hagen, G.R.** *Standard Equations of Motion for
  Submarine Simulation*, NSRDC Report 2510, 1967. The original US Navy
  standard. ([DTIC PDF](https://apps.dtic.mil/sti/tr/pdf/AD0653861.pdf))
- **Feldman, J.** *DTNSRDC Revised Standard Submarine Equations of
  Motion*, 1979. Updated Navy standard.
  ([DTIC PDF](https://apps.dtic.mil/sti/tr/pdf/ADA071804.pdf))
- **Brutzman, D.P.** *Underwater Vehicle Dynamics Model* (dissertation
  Ch. VI), Naval Postgraduate School. Concise summary with real
  coefficient values.
  ([PDF](https://faculty.nps.edu/brutzman/dissertation/chapter6.pdf))
- **Fossen's Python Vehicle Simulator**: open-source implementations of
  marine vehicle models.
  ([GitHub](https://github.com/cybergalactic/PythonVehicleSimulator))
- **DARPA SUBOFF**: publicly available submarine hull geometry with
  published hydrodynamic coefficients. Useful as a reference data set for
  parameterisation.

---

## Sonar Model

Sonar is the primary detection system. Submarines and torpedoes cannot
see. They can only listen, and occasionally shout.

### Noise Emission

Every entity emits noise based on its current state:

| Source | Noise level | Notes |
|--------|-------------|-------|
| Engine / throttle | **Primary**, proportional to throttle | Cut engines = go quiet |
| Cavitation | High speed only, above a threshold | Speed and loud; unavoidable at high throttle |
| Hull flow | Low, proportional to speed² | Present even when gliding with engines off |
| Ballast changes | Transient burst | Blowing/flooding tanks is noisy |
| Torpedo launch | Loud transient | Reveals position momentarily |
| Active sonar ping | **Very loud**, long range | Heard by everyone, precise bearing |

**Key gameplay consequence:** noise comes primarily from the **engine**,
not from speed. A submarine that sprints to build speed, then cuts
engines and glides, is nearly silent while still moving. This enables
the classic "sprint and drift" tactic.

### Passive Sonar

Passive sonar listens for noise. Each tick, the engine computes what
each entity can hear using the sonar equation:

```
SE = SL_target - TL - NL_listener
TL = 10 * log10(distance)     (cylindrical spreading)
NL = max(55 dB ambient, SL_self - 35 dB)
Detection requires SE > 5 dB
```

Submarines have base SL = 90 dB + 2 dB per m/s of speed. Under ideal
conditions (no thermocline, no terrain), detection ranges are:

| Target | Listener | Max Range |
|--------|----------|-----------|
| Patrol (3 m/s, 96 dB) | Patrol (3 m/s) | ~1 km |
| Moderate (5 m/s, 100 dB) | Slow (1 m/s) | ~6 km |
| Sprinting (10 m/s, 110 dB) | Patrol (3 m/s) | ~25 km |
| Torpedo (25 m/s, 140 dB) | Anyone | Arena-wide |

Speed asymmetry is critical: a slow listener detects much further than
a fast one. Going slow is the primary tactical advantage for detection.

A passive sonar contact provides:

- **Bearing**: direction to the noise source (1-10 deg error depending
  on signal excess).
- **Signal excess**: a single number. Louder means closer *or* noisier
  engine, ambiguous by design.
- **Range** (from engine-side TMA): starts with large error, improves
  only through cross-track maneuvering. See
  [Passive Sonar / TMA Design](passive-sonar-tma-design.md).
- **Estimated heading** (from TMA): requires quality > 0.5.

A passive contact does **not** directly provide:

- Depth.
- Identity (no FFI).

This ambiguity is what makes the game interesting. A strong contact
could be a nearby quiet submarine or a distant one at full throttle. A
torpedo at full speed sounds the same as a submarine at full speed.

The engine runs automatic TMA (Target Motion Analysis) for each
contact, building range and heading estimates over time. The quality
of these estimates depends entirely on the listener's maneuvering:
deliberate zig-zag legs perpendicular to the bearing line are needed
to resolve range. Time alone gives almost nothing. See
[Passive Sonar / TMA Design](passive-sonar-tma-design.md) for details.

### Active Sonar

An active ping emits a loud pulse and listens for echoes. It is a
powerful but costly action:

- **Everyone hears the ping**: all entities get a precise bearing to
  the pinger at long range.
- **The pinger gets returns** with much better information:
  - Bearing (high accuracy).
  - Range (from time-of-flight, good accuracy).
  - Possibly a rough signature / size.
- Returns are still **no FFI**: you know something is there, not what
  it is.
- Terrain blocks active sonar: a ping won't find something behind a
  seamount.
- Thermocline degrades returns: pinging across a thermal layer gives
  poor or no returns.

Active pings from torpedoes are **indistinguishable** from submarine
pings, and this is what makes torpedo decoys work.

### Terrain Occlusion

Sound does not travel through solid rock. The engine performs an acoustic
line-of-sight check through the terrain heightmap between source and
listener.

- A submarine behind a seamount or ridge is in an **acoustic shadow**.
- Diffraction: sound bends somewhat around obstacles, so the shadow
  isn't absolute, so a contact behind terrain may appear at greatly
  reduced signal strength rather than disappearing entirely.
- **Both directions are blocked**: if you hide behind terrain, you also
  can't hear what's on the other side.

This makes terrain tactically valuable:

- Hide behind a ridge to mask engine noise during a sprint.
- Use a canyon to approach a target undetected.
- Set up an ambush at a chokepoint.
- Duck behind cover after an active ping.

### Thermal Layers (Thermoclines)

Temperature changes with depth create thermal boundaries. Sound
refracts at these boundaries, bending away from warmer water.

- A submarine below a thermocline is harder to detect from above (and
  vice versa).
- The engine applies a **detection penalty** when source and listener
  are on opposite sides of a thermocline.
- This makes depth control tactical: dive below the layer to hide, but
  your own sonar looking up is also degraded.
- Thermal layer depths are known from the terrain data (part of
  `MatchContext`).

### Tactical Patterns Enabled by This Model

**Sprint and drift:**
Full throttle → build speed → cut engines → glide silently while
decelerating. Enemy hears you during the sprint (bearing only), then
loses you during the drift. They don't know how far you've gone or
where you've turned.

**Terrain masking:**
Position behind a seamount. Engine noise is occluded. Pop out for an
active ping or torpedo launch, then duck back behind cover.

**Thermocline diving:**
Dive below the thermal layer. Enemy above gets severely degraded
detection. Your own upward sonar is also degraded, so you sacrifice
awareness for stealth.

**Decoy deployment:**
Launch a torpedo at submarine-like speed with active pinging. The
enemy sees a loud contact and can't tell if it's real. They waste
a torpedo on it, or manoeuvre away from a threat that isn't there.

---

## Damage Model

### Initial Model: Uniform Hit Points with Proximity Torpedoes

Each submarine starts with a fixed number of **hit points** (HP).
When HP reaches zero, the submarine is destroyed.

**Proximity torpedo detonation:**

Torpedoes use a **proximity fuse**: they detonate when they come within a
threshold distance of a target, not on contact. The explosion produces an
underwater shockwave that affects nearby submarines:

- **Damage** scales with proximity using quadratic falloff from the hull
  surface distance: `baseDamage * (1 - d/r)^2`. Base damage is 1200 HP
  (instant kill at point blank against 1000 HP subs). Blast radius 50m.
- **Impulse** pushes the target away from the blast, divided by the sub's
  mass and rotational inertia. The effect is a moderate shove and slight
  heading disruption, not a cartoon spin. Off-center hits add small yaw
  torque proportional to the cross product of the blast direction and the
  sub's forward axis.
- **Terrain detonation**: torpedoes detonate on terrain impact, which can
  damage nearby subs caught in the blast radius.
- **No FFI on the fuse**: the proximity fuse triggers on *any* submarine,
  including the owner (after a 5-second arming delay post-launch).

This creates interesting gameplay:

- Precise torpedo guidance is rewarded with more damage.
- Near-misses still matter: a close pass shoves the target off course.
- Terrain detonations near a sub deal splash damage.
- Evasive manoeuvres can reduce damage even when a torpedo gets close.

**Submarine death:**

When a submarine's HP reaches zero:

- Engine cuts, clutch disengages (propeller freewheels to a stop).
- Ballast tanks flood (maximum negative buoyancy).
- The sub sinks to the seabed, settles at the terrain slope angle.
- Terrain clearance drops to near-zero (hull rests on the bottom).
- Physics continues: the sub decelerates from drag and comes to rest.
- Viewer shows: decaying prop spin, bubble stream from the hull
  (diminishing as the sub settles), pitch adjusting to match the
  terrain slope beneath bow and stern.

**Collision damage:**

- Collision with terrain or other submarines deals damage proportional to
  impact velocity and applies a corresponding impulse.

**Crush depth:**

Submarines have a pressure hull with a **rated depth** and an
**absolute crush depth**. The zone between them is survivable but
increasingly dangerous: the longer and deeper you stay, the worse
it gets.

- **Rated depth** (e.g. −400 m): the submarine's design limit. Above
  this, no depth-related effects. Below this, things start going wrong.
- **Between rated depth and crush depth** (−400 m to −700 m):
  - **Hull stress damage:** the submarine takes HP damage each tick,
    proportional to how far past rated depth it is. Mild near the
    boundary, severe deeper down.
  - **Implosion risk:** each tick below rated depth has a probability
    of **catastrophic implosion** (instant destruction). The
    probability increases with both **excess depth** and
    **accumulated time** below rated depth. A quick dive to −450 m
    and back up is a calculated risk. Lingering at −600 m is playing
    Russian roulette.
  - The engine tracks accumulated hull stress per submarine. Stress
    builds while below rated depth and slowly recovers above it.
    Multiple deep dives without recovery between them are cumulative.
- **Crush depth** (e.g. −700 m): **instant destruction**, no chance of
  survival. The hull implodes regardless of HP or stress history.
- **Terrain goes deeper** than crush depth (e.g. −900 m). Deep
  trenches and canyons on the map are absolute no-go zones for
  submarines, lethal terrain features that constrain navigation.
- **Torpedoes are unaffected**: their smaller hulls are rated for
  the full ocean depth. A submarine cannot escape a torpedo by diving
  to its own death.
- All depth parameters are in `MatchConfig` and communicated to
  controllers at match start.

**Implosion model:**

```
excess = rated_depth - current_depth          (positive when below rated)
ratio = excess / (rated_depth - crush_depth)  (0.0 at rated, 1.0 at crush)

hull_stress_damage_per_tick = max_stress_dmg × ratio²
accumulated_stress += ratio × dt              (builds while below rated)
accumulated_stress -= recovery_rate × dt      (decays while above rated)
accumulated_stress = clamp(0, max_stress)

implosion_probability = base_rate × ratio² × (1 + accumulated_stress)
if random() < implosion_probability:
    submarine is destroyed (implosion)

if current_depth <= crush_depth:
    submarine is destroyed (implosion)
```

This creates layered tactical choices:

- **Brief deep dives** are viable: duck below rated depth to dodge a
  torpedo or cross below a thermocline, then climb back quickly.
  Costs some HP and stress but manageable.
- **Extended deep operations** are a gamble. You might survive, you
  might implode. The accumulated stress mechanic means you can't
  repeatedly yo-yo below rated depth without consequence.
- **Deep canyons are death traps**: a submarine chased into a canyon
  that descends past crush depth has nowhere to go.
- **Damaged submarines are fragile**: with low HP, even mild hull
  stress damage becomes critical. Depth management gets harder as
  the match progresses.
- **Depth as a match parameter**: a shallow rated depth (e.g. −250 m)
  compresses the vertical play space, making sonar more effective.
  A deep rated depth (e.g. −500 m) opens up more evasion room but
  pushes crush depth closer to the deepest terrain.

**Explosion model (simplified):**

```
distance = ||detonation_point - target_position||
if distance < blast_radius:
    damage = max_damage × (1 - distance / blast_radius)²
    impulse_magnitude = max_impulse × (1 - distance / blast_radius)²
    impulse_direction = normalize(target_position - detonation_point)
    apply impulse_magnitude × impulse_direction to target's velocity
    subtract damage from target's HP
```

The quadratic falloff gives a sharp damage curve: devastating up close,
minor at the edge of the blast radius. The exact parameters (blast radius,
max damage, max impulse) are tunable for gameplay balance.

### Future Refinements

The damage model is designed to be replaceable. Potential future
enhancements:

- **Positional damage:** hits on different sections (bow, engine, sail)
  affect different subsystems.
- **Subsystem degradation:** damaged sonar reduces detection range, damaged
  engines reduce max thrust, damaged control surfaces reduce
  manoeuvrability.
- **Flooding and buoyancy effects:** hull breaches cause progressive
  flooding that affects trim and depth control.
- **Damage control:** participants could allocate crew/resources to repair
  efforts as part of their actuator output.
- **Shaped charges:** directional warheads where detonation angle matters.

---

## Determinism & Replay

### Deterministic Core

Given:

- World generation seed
- Accepted actuator outputs per tick

The simulation engine is **deterministic**: the same inputs always
produce the same physics outcome. However, overall match results may
vary between runs if submarine implementations use their own random
number generators.

### Replay Format

Replay files contain:

- Match metadata (participants, engine version, physics version).
- World generation seed.
- Per-tick accepted actuator outputs for all participants.
- Key world events (launches, impacts, destructions, damage).
- Per-tick contact estimates published by submarines.
- Optional periodic state snapshots for fast seeking.

Replay enables:

- Full match reconstruction without re-running bot code.
- Tournament playback and broadcast.
- Deterministic debugging.
- Regression testing of engine changes.
- Dispute resolution.

---

## Safety Model

SeaRobots assumes participant code may be **actively malicious**.

### Threat Model

| Threat | Mitigation |
|--------|-----------|
| Fork/thread bombs | Process/thread limits via cgroups |
| Memory exhaustion | Hard memory ceiling |
| Busy loops | Per-tick wall-clock deadline; CPU quota |
| stdout/stderr spam | Output size limits |
| File probing | Read-only root FS, no access beyond scratch |
| Network callbacks / exfiltration | No outbound network (only engine socket) |
| JNI / native library escapes | Disallowed in submissions |
| Deserialization attacks | Engine never deserializes untrusted Java objects |
| Host kernel exploits | VM-backed isolation for adversarial tournaments |

### Submission Restrictions

- Source or bytecode only, no native libraries.
- No JNI, no Java agents, no Attach API.
- No subprocess spawning.
- Runtime image is built by the arena, not by the participant.

The real security boundary is always the **OS/VM sandbox**, not the JVM
type system.

---

## Design Principles

1. **Engine is authoritative.** Controllers propose, the engine decides.
2. **One participant = one sandbox.** VM-level isolation, guest agent inside.
3. **Torpedoes are autonomous.** Separate process, separate controller, no
   post-launch guidance, only position telemetry back to the sub.
4. **Shared compute budget.** Every process in the VM (sub + torpedoes)
   competes for the same CPU quota. More torpedoes = less compute each.
5. **No friend-or-foe identification.** Sonar and proximity fuses treat
   all contacts equally. Your own torpedoes can hunt you.
6. **Deterministic simulation.** Same seed + same inputs = same match.
7. **Decoupled rendering.** The viewer is a consumer, never a participant.
8. **Two-tier transport.** TCP from engine to guest agent, Unix domain
   sockets from agent to controller processes.
9. **Replay is first-class.** Recorded inputs enable full match replay.
10. **Safety by isolation.** Trust the sandbox, not the language runtime.

---

## Future Extensions

- **Advanced controller API:** Allow multiple improved responses per tick
  (overwrite-until-deadline semantics) for bots that want to send a fast
  initial response and refine.
- **Multi-language support:** The message protocol is language-agnostic.
  SDKs for Kotlin, Rust, Python, or other languages can wrap the same
  wire format.
- **WebAssembly bots:** Compile participant code to Wasm for an even
  narrower execution model.
- **Tournament system:** Rankings, matchmaking, league tables, scheduled
  events.
- **Spectator mode:** Live 3D viewing with commentary overlays.
- **Surface operations:** Nuclear submarines are faster submerged than
  surfaced, but diesel-electric boats are the reverse (~17 knots
  surface vs ~8 knots submerged for a Type VIIC). Surface speed,
  snorkeling, and battery management could add a diesel-electric
  submarine class with different tactical tradeoffs.
- **Advanced environment:** Dynamic ocean currents, weather effects,
  biological noise sources.
