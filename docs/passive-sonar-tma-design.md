# Passive Sonar and TMA Design

Engine-side Target Motion Analysis that provides submarine controllers
with progressively accurate contact data, without requiring controllers
to implement their own TMA.

## Design Philosophy

Real passive sonar gives bearing only. Range, heading, and speed must
be derived through Target Motion Analysis (TMA), which requires time,
maneuvering, and signal processing. In realistic submarine simulators
like Dangerous Waters, the player performs TMA manually. In more
accessible games like Cold Waters, the crew does it automatically and
presents a "solution percentage" that improves over time.

SeaRobots follows the Cold Waters approach: the engine runs TMA
automatically for each contact and provides the results on every
`SonarContact`. This means:

- Simple bots can read `contact.range()` and `contact.estimatedHeading()`
  directly, no TMA implementation needed.
- The quality of these estimates depends on the bot's behavior (maneuvering
  improves the solution, sitting still keeps it uncertain).
- Advanced bots can still use raw bearing data for custom analysis.
- The engine handles all the physics-accurate signal processing internally.

## What the Controller Receives

Each `SonarContact` provides:

| Field | Source | Accuracy |
|-------|--------|----------|
| bearing | Direct measurement | Good (1-10 deg depending on SE) |
| bearingUncertainty | From SE | Quantified |
| range | Engine TMA | Starts with large bias, improves only with maneuvering |
| rangeUncertainty | Engine TMA | Reflects solution quality |
| estimatedSpeed | Blade-rate analysis | Good at high SE (close range) |
| estimatedHeading | Engine TMA | Requires quality > 0.5 (deliberate maneuvering) |
| estimatedSourceLevel | Signal analysis | For classification |
| estimatedDepth | Active sonar only | From vertical angle + range (5% noise, min 5m). NaN for passive. |
| signalExcess | Direct measurement | Instantaneous |
| solutionQuality | Engine TMA | 0.0 (no data) to 1.0 (fire-quality solution) |

The `solutionQuality` field is the equivalent of Cold Waters' SOL%.
It reflects how much geometric data the TMA has accumulated. At 0.05,
range is a wild guess. At 0.4, range is order-of-magnitude accurate.
At 0.85+, the solution is good enough for a torpedo shot.

## Engine-Side Contact Tracker

The `SonarModel` maintains a `ContactTracker` for each listener-source
pair. The tracker accumulates bearing observations along with the
listener's own-ship motion, and computes a TMA solution.

### ContactTracker State

```
- Bearing history: list of (tick, bearing, ownShipX, ownShipY, SE)
- Accumulated cross-track displacement (meters)
- Estimated range (meters) with persistent bias and uncertainty
- Estimated target heading (radians) with quality-dependent noise
- Solution quality (0.0 to 1.0)
- Leg count (accumulated heading changes > 15 degrees)
- Last active ping range (if any, for calibration)
```

### Solution Quality Computation

Solution quality increases based on actual geometric information,
not time:

1. **Cross-track displacement** (primary driver): each meter of
   own-ship movement perpendicular to the contact bearing adds to
   the accumulated displacement. Quality scales with displacement
   relative to estimated range. At 2000m range, you need ~2000m
   of accumulated cross-track for maximum quality from this component.
   Capped at 0.5.

2. **Leg count**: each own-ship course change > 15 degrees counts as
   a new leg and adds 0.12 quality (capped at 0.35). Two deliberate
   legs provide significant improvement; three legs give a solid
   solution.

3. **Minimal time bonus**: just having observations in the history
   adds a tiny amount, capped at 0.05. This prevents time alone from
   resolving range: you must maneuver.

4. **Active ping**: immediately sets quality to 0.95 (known range).
   Calibrates the SL for subsequent passive ranging.

Quality floor is 0.05 (essentially "bearing only, no range info").

Solution quality decays when contact is lost:
- Decays at ~0.01 per second without contact.
- Range uncertainty grows at maxSubSpeed per second (target could be
  moving in any direction).

### Range Estimation Method

The tracker models the output of a Kalman filter / batch least-squares
TMA system using ground-truth distance with quality-dependent noise
and a persistent systematic bias.

**Initial bias (large):**
When first detected, the range estimate has a random bias of up to
1.5x the actual distance in either direction. This models the
fundamental ambiguity of bearing-only detection: without geometry, the
target could be at any distance along the bearing line.

**Bias decay (geometry-driven only):**
The bias decays proportional to the square of geometric information
(solutionQuality). Without cross-track maneuvering, the bias does
not decay at all:
- At quality 0.05 (no maneuvers): bias is essentially permanent
- At quality 0.3 (some maneuvering): half-life ~27 seconds
- At quality 0.5 (good geometry): half-life ~10 seconds
- At quality 0.8 (excellent): half-life ~4 seconds

**Random noise:**
On top of the biased estimate, each tick adds random noise:
- At low quality: +/- 25% of actual distance
- At high quality: +/- 5% of actual distance

**Smoothing:**
The range estimate is exponentially smoothed. The blend rate scales
with geometric information, preventing wild jumps.

### Heading Estimation

Target heading requires quality > 0.5 (at least two good legs of
maneuvering):
- Uses ground-truth target displacement with quality-dependent noise
- At quality 0.5: heading noise is ~25 degrees
- At quality 0.9: heading noise is ~5 degrees
- Requires at least 5 seconds between samples, and the target must
  have moved meaningfully (>10m)

Speed from blade-rate analysis is available independently and earlier
than TMA-derived heading.

### Convergence Timeline (typical)

Using a flat deep ocean at 2000m range, submarine at 3 m/s doing
north/south zig-zag legs (30s per leg), targeting a moderately
noisy contact:

| Time | Error | Quality | Notes |
|------|-------|---------|-------|
| 30s  | ~135% | 0.05    | First contact, huge bias, no geometry |
| 70s  | ~100% | 0.20    | First leg change, quality jumps |
| 130s | ~30%  | 0.27    | Second leg, bias decaying |
| 190s | ~6%   | 0.33    | Three legs, usable range estimate |
| 250s | <5%   | 0.45+   | Good solution, approaching firing quality |

A patient submarine with deliberate zig-zag maneuvering can build
a firing solution in 3-5 minutes without ever pinging.

## Sonar Parameters

The sonar equation determines passive detection range:

```
SE = SL_target - TL - NL_listener
TL = 10 * log10(R)          (cylindrical spreading)
NL = max(ambient, SL_self - selfNoiseOffset)
Detection requires SE > 5 dB
```

Current parameters:

| Parameter | Value | Effect |
|-----------|-------|--------|
| Base SL (submarine) | 90 dB | Machinery noise at zero speed |
| Speed noise | 2.0 dB/m/s | Each m/s adds 2 dB |
| Ambient noise | 55 dB | Deep ocean floor |
| Self-noise offset | 35 dB | Hydrophone isolation from own machinery |
| Detection threshold | 5 dB | Minimum SE to detect |
| Spreading coefficient | 10 | Cylindrical (shallow water waveguide) |
| Baffle penalty | 20 dB | Stern arc blind zone |

### Detection Ranges (ideal conditions)

These are theoretical maximums with no thermocline, no terrain
occlusion, and no baffles:

| Scenario | Max Range |
|----------|-----------|
| Patrol (3 m/s) heard by patrol (3 m/s) | ~1 km |
| Moderate (5 m/s) heard by patrol (3 m/s) | ~2.5 km |
| Moderate (5 m/s) heard by slow (1 m/s) | ~6.3 km |
| Moderate (5 m/s) heard by stopped listener | ~10 km |
| Fast (8 m/s) heard by patrol (3 m/s) | ~10 km |
| Sprinting (10 m/s) heard by patrol (3 m/s) | ~25 km (arena-wide) |
| Torpedo (25 m/s, 140 dB) heard by anyone | arena-wide |
| Stopped sub heard by patrol (3 m/s) | ~250m |

Key tactical implications:
- **Speed asymmetry matters:** a slow listener detects much further
  than a fast one (6.3 km vs 398m for the same target)
- **Going slow is a real tactical advantage** for listening
- **A stopped sub is nearly invisible** beyond 250m
- **Torpedoes are always detectable** from anywhere in the arena

### Thermocline, Terrain, and Baffle Effects

Under realistic conditions, detection ranges are significantly
shorter:
- Thermocline crossing: 5-15 dB additional loss
- Terrain occlusion: 15-30 dB per obstructing cell
- Baffles: 20 dB penalty in the stern arc

A sprinting sub that is arena-wide detectable in open water may be
undetectable at 5 km through a thermocline, or completely hidden
behind a seamount.

## Environmental Effects on TMA

### Thermocline

Sound crossing a thermocline boundary suffers additional
transmission loss. This affects:
- **Detection range**: contacts across the thermocline are harder to
  detect (lower SE at same distance).
- **TMA convergence**: lower SE means noisier bearing measurements,
  which slows the quality build.

### Terrain Occlusion

Ridges and islands between the listener and source add transmission
loss. This:
- **Blocks or weakens contacts** behind terrain features.
- **Disrupts TMA continuity**: if a contact moves behind a ridge,
  the tracker loses observations and quality decays.

### Baffles

The listener's stern arc has a 20 dB penalty, creating a near-blind
zone. The tracker does not update the TMA solution from
baffle-degraded observations.

## Interaction with Active Sonar

When the listener pings:
1. Active returns give precise range (2% RMS noise) and estimated
   depth (from vertical bearing angle, 5% RMS noise, min 5m).
2. The tracker calibrates the source level: `SL = SE + TL + NL`
   where TL uses the known range.
3. Subsequent passive observations use the calibrated SL for more
   accurate SE-based ranging.
4. Solution quality jumps to 0.95.
5. Everyone on the map hears the ping (tactical cost).
6. Active sonar cooldown: 250 ticks (5 seconds) between pings.

The ping is the "cheat code" for instant TMA, but it reveals your
position. The game rewards patience (building a passive solution
through maneuvering) vs impatience (pinging for instant data).

## Impact on Controller Design

### Simple Bot (Minimum Viable)
```java
public void onTick(SubmarineInput input, SubmarineOutput output) {
    for (var contact : input.sonarContacts()) {
        if (contact.solutionQuality() > 0.85) {
            // Good enough for a torpedo shot
            double targetRange = contact.range();
            double targetHeading = contact.estimatedHeading();
            // Set up torpedo shot...
        }
    }
}
```

### Advanced Controller Pattern
```java
// Zig-zag to build TMA solution
if (contact.solutionQuality() < 0.5) {
    // Maneuver: alternate legs perpendicular to bearing
    double perpBearing = contact.bearing() + Math.PI / 2;
    // Navigate to build cross-track geometry...
}

// When solution quality is high enough, fire without pinging
if (contact.solutionQuality() > 0.7 && contact.range() < 3000) {
    // Fire torpedo with TMA-derived position
    output.launchTorpedo(new TorpedoLaunchCommand(...));
}
```

### Design Considerations

**Multiple Contacts:**
The engine maintains a separate ContactTracker per listener-source
pair. The SonarContact list returned each tick includes all detected
contacts with their individual TMA solutions. The controller decides
which to prioritize.

**Contact Association:**
The engine associates contacts across ticks by bearing proximity
(within ~10 degrees of previous bearing, accounting for own-ship
motion). New contacts that don't match existing tracks create new
trackers.

**TMA Reset:**
When a contact is lost for more than ~30 seconds, the tracker is
discarded. A new detection starts fresh. However, if a ping was
previously used to calibrate SL for a contact in a similar bearing,
the calibration data can be reused.

## References

- Cold Waters: automatic TMA with SOL%, crew builds solution in
  background, quality depends on signal strength and maneuvering.
- Dangerous Waters: fully manual TMA, bearing-only from passive,
  player must triangulate.
- Ekelund Ranging (1958): range = delta(crossTrackSpeed) /
  delta(bearingRate), the foundational passive ranging technique.
