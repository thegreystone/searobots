# SeaRobots Replay Format (`.srl`)

The replay log is the **canonical, lossless record of a match**. It captures
the full per-tick simulation state so a match can be replayed directly from the
log with no re-simulation, and so it can be analyzed offline. It is the only
artifact ever conserved; analysis tables (see [Two tiers](#two-tiers)) are
derived from it and regenerable.

Implementation: `se.hirt.searobots.engine.replay`
(`ReplayFormat`, `ReplayCodec`, `ReplayWriter`, `ReplayReader`, `ReplayHeader`,
`ReplayFrame`, `ReplayPlayer`). Playback shares the `SimClock` control surface with the
live `SimulationLoop`.

## Why this format

- **State capture, not input replay.** Controllers may use randomness, so
  re-running them under a fixed seed is not guaranteed to reproduce a match.
  Recording full state reproduces exactly what happened, regardless of
  controller determinism.
- **Tab-delimited lines, not JSON.** A 50 Hz, 40-minute match is ~120k frames;
  repeating field names per line wastes bytes and parse time. Every line is
  `split("\t")` in Java and JavaScript alike, so no JSON parser is needed.
  (Java object serialization was also rejected: platform churn, Java-shape
  lock-in, and browser-viewer hostility.)
- **Self-describing.** The header declares the format version, the match
  parameters, and — per record type — the ordered column list **with units**.
  Any reader, including an LLM, can interpret the file from the header alone.

## Layout

```
SRREPLAY <version>                                  magic + format version (line 1)
H    seed=.. tickRate=.. durationTicks=.. ...        match parameters (key=value)
COLS s  id: x:m y:m z:m heading:rad ... status:text  column schema per record tag
COLS c  ...
COLS w  ...
COLS p  ...
COLS S  ...
S    <id> <name> <colorArgb> <spawnX> <spawnY> <spawnZ>   sub definitions (once)
T    <tick>                                          begins a frame
s    <sub state...>                                  one per submarine per frame
c    <contact estimate...>                           child of the preceding s line
w    <waypoint...>                                   child of the preceding s line
f    <firing solution...>                            child of the preceding s line (0 or 1)
p    <torpedo state...>                              one per torpedo per frame
T    <tick>                                          next frame
...
E                                                    end of match
```

- Line 1 is the magic token and format version; the reader dispatches on it.
- The first column of every other line is a one-character **record tag**.
- Numbers are compact: integers print without a decimal point, others keep up
  to four decimals (trailing zeros stripped). `NaN` is written literally.
- Free-text fields (`status`, `label`, `diagPhase`) are sanitized of tabs and
  newlines so they can never break the line structure.

## Record tags

| Tag    | Record               | Cardinality                 |
|--------|----------------------|-----------------------------|
| `H`    | Match header         | once                        |
| `COLS` | Column schema        | once per data tag           |
| `S`    | Submarine definition | once per submarine          |
| `T`    | Frame marker         | once per recorded tick      |
| `s`    | Submarine state      | per submarine per frame     |
| `c`    | Contact estimate     | per estimate (child of `s`) |
| `w`    | Waypoint             | per waypoint (child of `s`) |
| `f`    | Firing solution      | 0 or 1 (child of `s`)       |
| `p`    | Torpedo state        | per torpedo per frame       |
| `E`    | End of match         | once                        |

The canonical column lists (names + units) live in `ReplayCodec`. Because the
writer emits them as `COLS` lines and the reader resolves columns **by name**,
a reader can still read the common columns of a file written by a later version
that appended columns. Incompatible changes bump `ReplayFormat.VERSION`.

Conceptually each data tag is its own rectangular table; the file interleaves
them in tick order for streaming replay.

## Fidelity

The format is lossless to its fixed numeric precision (4 decimals: sub-millimeter
on positions, ~6e-5 rad on angles — far below any physically meaningful
threshold). `ReplayRoundTripTest` asserts the stronger **lossless-against-live**
property: a match recorded through `SimulationListeners.composite` replays to a
stream equal to the one the simulation emitted.

**Not yet captured:** strategic-waypoint visualization; it reconstructs as empty.
Adding it is an additive change plus a version bump, exactly as firing solutions
were added in v2 (the `f` child line). Firing solutions **are** captured, so a
replay can pause on the first firing solution just like a live match.

## Reading and playback

Because the viewers are `SimulationListener`s, `ReplayReader.replay(listener)`
drives them directly:

```java
ReplayReader reader = new ReplayReader(Path.of("match-42.srl"));
ReplayHeader header = reader.header();       // version, seed, params, sub defs
reader.replay(viewer);                         // re-emits onTick/onMatchEnd
```

To record a live match while rendering it (the loop takes a single listener):

```java
try (var writer = new ReplayWriter(config, world.spawnPoints(), file)) {
    sim.run(world, controllers, vehicles,
            SimulationListeners.composite(viewer, writer));
}
```

In the viewer, `SimulationManager` records every live match automatically to
`replays/<timestamp>-<seedHex>.srl`, so any battle you watch can be replayed later.

### Playback with live-sim controls

`ReplayReader.replay(listener)` streams frames as fast as it can read them, which is
what the lossless-against-live test wants but not what a viewer wants. For interactive
playback, `ReplayReader.readAll()` buffers the whole match as a `List<ReplayFrame>`, and
`ReplayPlayer` walks it on its own thread:

```java
var reader = new ReplayReader(file);
var frames = reader.readAll();
var player = new ReplayPlayer(frames, reader.header().tickRateHz(), viewer);
new Thread(player::run).start();   // player.setPaused(false) to begin
```

`ReplayPlayer` implements `SimClock` — the same pause / single-step / speed-multiplier /
stop control surface as the live `SimulationLoop` — so the viewer drives a replay with the
exact controls it uses for a live match (`SimulationManager.startReplay(path)` wires this
up, regenerating the world from the header seed). Because playback flows through the same
event-pausing fan-out, the **pause-on-event** toggles apply to replays too, and
"fast-forward to next event" is simply *max speed until the fan-out pauses us* — no
separate seek machinery.

- Pause-on-death, pause-on-launch, and pause-on-torpedo-solution all work in replay:
  each derives from captured state (firing solutions are recorded as of format v2).
- The world is reconstructed from the header **seed**, so hand-built worlds
  (`GeneratedWorld.deepFlat()` / `lIslandRecovery()`) won't reproduce their terrain.

## Two tiers

- **Tier 1 — this replay log (`.srl`).** Canonical, lossless, streaming.
- **Tier 2 — analysis tables (planned, derived).** A Java pass splits the
  interleaved log into per-tag, strictly rectangular, **denormalized** TSVs
  (every torpedo/contact row carries `seed` and `tick`) for DuckDB/Polars
  SQL-style physics analysis. Tier 2 is a regenerable cache: evolve its schema
  freely and re-derive from Tier 1. Parquet or direct-to-DuckDB is a later
  drop-in if read performance demands it.
