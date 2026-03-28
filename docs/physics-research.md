# Submarine Physics Research

Research notes on submarine physics from games, simulations, and
academic papers. Used to inform the SeaRobots physics model.

---

## Academic References

### Fossen: Handbook of Marine Craft Hydrodynamics (2nd ed, 2021)

The canonical reference for marine vehicle dynamics. Defines the
standard 6-DOF matrix-vector equation of motion:

```
M * v_dot + C(v) * v + D(v) * v + g(eta) = tau
```

Where:
- **M**: inertia matrix (rigid body + added mass)
- **C(v)**: Coriolis-centripetal matrix
- **D(v)**: damping matrix (linear + quadratic terms)
- **g(eta)**: restoring forces (gravity + buoyancy)
- **tau**: control forces (propeller + control surfaces)

Key chapters: 6-8 cover vehicle modeling, integrator guidance, and
control. The full formulation requires 100+ hydrodynamic coefficients
(Gertler & Hagen), but diagonal approximations work well for games.

Python Vehicle Simulator: `cybergalactic/PythonVehicleSimulator`
on GitHub. Contains working implementations of DSRV and REMUS 100
AUV models.

### Prestero: REMUS AUV 6-DOF Model (MIT Thesis, 2001)

Complete coefficient derivation for a torpedo-shaped hull (REMUS,
0.19m diameter, 1.9m length, 31.9 kg, max 2.5 m/s at 1525 RPM).
Provides tables of added mass coefficients (X_u_dot, Y_v_dot,
Z_w_dot, etc.) and working Matlab code.

Key data for scaling to game-sized submarine (30m hull, 6m beam):
- Added mass scales roughly with diameter squared and length
- X_u_dot (surge added mass) ~ 0.5x dry mass for slender body
- Y_v_dot (sway added mass) ~ 1.5x dry mass (blunt cross-section)
- Z_w_dot (heave added mass) ~ 1.25x dry mass

### Gertler & Hagen: Standard Equations of Motion (NSRDC 2510, 1967)

Classic coefficient dataset for submarine simulation. Free PDF from
DTIC. Defines the 100+ hydrodynamic coefficients that the Fossen
formulation uses.

### Feldman: DTNSRDC Revised Standard (1979)

Revised standard submarine equations of motion. Free PDF from DTIC.

### Brutzman: Underwater Vehicle Dynamics (NPS Dissertation)

Chapter VI covers practical implementation. Free PDF.

### DARPA SUBOFF

Publicly available benchmark submarine hull geometry with
experimentally validated coefficients. Used for verification of
simulation implementations (Groves et al. 1989, DTRC Report
DTRC/SHD-1298-01).

---

## Game Physics Analysis

### Cold Waters (Killerfish Games, 2017)

The most transparent game for studying submarine physics: almost
everything is controlled by editable text files.

**Movement model:**
- `AccelerationRate` and `DecelerationRate` per vessel
- Key insight: **deceleration is 4-10x slower than acceleration**
  - Alfa-class: AccelerationRate=0.35, DecelerationRate=0.08
  - Stingray: AccelerationRate=0.34, DecelerationRate=0.08
- This asymmetry creates the "heavy" feeling. A sub takes minutes to
  reach flank speed, but coasting from flank to stop takes
  substantially longer.
- Operations manual: "changes in speed require some time to take
  effect"

**Turning:**
- `RudderTurnRate` and `TurnRate` per vessel
- Alfa-class: RudderTurnRate=1, TurnRate=2.8
- `PivotPointTurning=0.256` appears consistently
- Rudder damage significantly increases turn radius

**Torpedoes:**
- Wire-guided with real-time control while wire intact
- Launch speed limit: above ~20-25 knots causes tube malfunction
- Settings: Sensor (Passive/Active), Search (Straight/Snake),
  Depth (Run Level/Shallow/Deep)
- A 50-knot torpedo needs ~213 seconds to travel 6000 yards

**Cavitation:**
- Formula: `cavitation_depth = 20 * speed_knots - 100`
- Every 5 knots extra = 100 feet less depth margin

**Overall feel:** Described as "an underwater fighter jet sim" with
WASD flight controls. Sonar model praised as "on par with 688(i)."
Sweet spot between accessibility and simulation.

### Silent Hunter III (Ubisoft, 2005)

**Movement:**
- "The friction of moving through the water will eventually slow you
  down, but your U-boat will continue to travel for a while even after
  ordering a full stop."
- Max speeds: ~17 knots surface, ~8 knots submerged (Type VIIC)
- Speed parameters in config: `eng_power` and `max_speed` sections

**What it gets right:**
- Adjustable realism (auto-targeting to full manual TDC)
- Damaged ship physics are impressive
- The manual torpedo data computer creates deep engagement
- Finding and stalking takes hours to days of game time

### UBOAT (Deep Water Studio, 2019)

**Turning circle data** (Type VIIC from uboatarchive.net):
- Surface: 270-230m depending on speed
- Submerged: 280-180m depending on speed
- At 32-degree rudder: ~250m turning radius
- At slowest speed: 180m (tighter because less momentum)

**Ballast tank simulation:**
- Flooding opens vents on top, air escapes, water rushes in
- "Adjust Ballast" command trims to neutral buoyancy
- Compressed air effectiveness decreases with depth: at 100m (10
  atm), a standard blow expels only ~1/10th the water vs surface
- Emergency blow: expels ALL compressed air, surfaces in under a
  minute from any depth, but leaves you unable to trim (no air left)
- Crash dive: 30-40 seconds (historical was 25-30 seconds)

**Community feedback:**
- Submarines should not stop "dead still": waves, currents, and wind
  cause drift. Static stop feels wrong.
- "Turning quickly requires a lot of water flowing over the rudders":
  at slow ahead, it takes forever to turn.

### Barotrauma (Undertow Games/FakeFish, 2019)

**Unique depth model:**
- Horizontal movement via engines, vertical via ballast pumps
- Most subs: ~40 km/h horizontal, rarely >20 km/h vertical
- Terminal upward velocity: ~15 m/s for ALL submarines (can't fill
  more than 100% volume with air)
- Terminal downward velocity: varies by vessel ballast volume ratio
- Pump speed affects transition rate, not maximum rates
- Optimal neutral ballast level: 0.5

**What it teaches:** Inertia management is genuinely dangerous at
high speeds. The modular submarine design creates emergent physics
where hull shape, compartments, and component placement all affect
handling.

### From the Depths (Brilliant Skies, 2020)

"Hefty physics model that includes buoyancy, drag, sealed
compartments, and inertia, so your hulls actually float, submerge,
and handle based on how you build them." Construction-based approach
where physics emerges from the design.

### Dangerous Waters / Sub Command (Sonalysts, 2005/2000)

"The greatest modern submarine simulator ever." 12+ detailed stations
per platform. Breathtaking attention to detail. Full simulation with
realistic sonar, weapons, and maneuvering. Hardcore simulation, not
accessible to casual players.

---

## What Makes Submarines Feel Heavy

Synthesis of reviews, discussions, and technical analysis across all
games:

### 1. Asymmetric acceleration/deceleration

The single most important factor. Cold Waters uses deceleration rates
4-10x lower than acceleration rates. Real physics: drag force is
proportional to v^2, so deceleration is fast at high speed but
asymptotically slow approaching zero. The sub should coast for a long
time after cutting engines.

**Current SeaRobots problem:** With MASS=8000 and DRAG=530, coasting
from 10 m/s reaches 6 m/s in 1 second and 3.3 m/s in 3 seconds.
This feels like pulling a handbrake, not an 8-ton vessel in water.
With added mass (MASS_SURGE=12000), coasting to 6.9 m/s after 1s
and 4.3 m/s after 3s. Better but could be tuned further.

### 2. Speed-dependent control authority

Rudders and planes do nothing at low speed. This is physically
accurate (lift ~ v^2) and creates consequential decisions. UBOAT
community notes: "at slow ahead, it takes forever to turn." Real
submarines have a concept of "steerage way" (~2-4 knots minimum to
steer). Below this, you drift.

### 3. Depth as a resource

Ballast blow effectiveness decreasing with depth (UBOAT: 1/10th at
100m), emergency blow leaving you unable to trim, cavitation
increasing with speed/depth. Each depth choice is a commitment.

### 4. Sound as the main sensor

Speed creates flow noise that blinds your sonar. You trade awareness
for speed. Sprint-and-drift emerges naturally from this tradeoff.
Cold Waters community confirms this is "an operational procedure
intended to allow maintenance of passive sonar situational awareness
while covering longer distances."

### 5. Consequences for velocity

- Launching torpedoes above 20-25 knots can fail (Cold Waters)
- Cavitation reveals you
- High-speed collisions with terrain are lethal
- Can't stop quickly, so every speed decision commits you

### 6. Planning ahead

Because you can't stop quickly, every speed/depth decision commits
you. This is the core of what makes submarine games feel different
from surface ship or aircraft games.

---

## Real-World Reference Numbers

**Stopping distance:** Roughly 10-50% of speed in knots expressed
as nautical miles. A submarine at 30 knots needs ~3+ nautical miles
to coast to a stop.

**Power/speed:** Power required scales with the cube of velocity.
Doubling speed requires 8x the power.

**Turning circle:** Type VIIC at moderate speed: ~250m radius.
Modern nuclear subs: larger (~400-600m) due to higher speeds.

**Maximum pitch angle:** ~20 degrees normally, 5-10 degrees at high
speed, up to 30 degrees tested (but equipment breaks/spills).

**Real rudder limits:** Maximum rudder angle ~35 degrees because
rudders stall at greater angles.

**Trim pump:** ~200 gallons per minute, noisy, results in ~10 ft
depth oscillation. Hover systems: +/- 1 foot oscillation.

**Torpedo speeds:** Mk 48 ADCAP: 55 knots (high), 40 knots
(search). Range: 35,000 yards at 55 knots. Wire-guided.

**Torpedo kill box:** Modern torpedoes have a digital boundary. If
the torpedo leaves this box, it inerts its warhead, shuts down its
engine, and sinks. This is equivalent to SeaRobots' fuel depletion
and minimum speed mechanics.

---

## Gravity-Assisted Coasting

One tactic not commonly modeled in games but physically real: a
submarine can maintain speed with engines off by pitching downward
and using gravity. The force component along the body axis is
`m * g * sin(pitch)`. At 20 degrees pitch-down, this provides
~4.9 m/s^2 equivalent thrust per unit mass, enough to maintain
~7 m/s with engines completely off (using only gravity and control
surfaces). This is a significant stealth tactic: completely silent
propulsion at the cost of depth.

Current SeaRobots physics does not model this gravity component
along the body axis. Adding it would make diving-while-coasting a
viable silent movement tactic.

---

## SeaRobots Submarine Scaling Decision

After analyzing WW2 and modern submarine displacements, the original
SeaRobots submarine (30m, 8 tons) was far too small and light. This
produced unrealistic "handbrake" deceleration and no sense of inertia.

**Chosen scale: WW2-era attack submarine**

| Parameter | Old | New | Reference |
|-----------|-----|-----|-----------|
| Hull length | 30m | 65m | Type VIIC: 67m |
| Hull beam | 6m | 8m | Type VIIC: 6.2m (slightly wider for gameplay) |
| Dry mass | 8,000 kg | 700,000 kg | Type VIIC: 770 tons surfaced |
| Added mass (surge) | n/a | 350,000 kg | ~50% of dry mass for slender hull |
| Effective mass | 8,000 kg | 1,050,000 kg | Dry + added mass |
| Drag coefficient | 530 | ~3,864 | 0.5 * rho * Cd * frontalArea |
| Max thrust | 120,000 N | ~870,000 N | For ~15 m/s top speed |
| Arena radius | 5,000m | 7,000m | Room for long-range torpedo combat |

**Resulting behavior:**

| Maneuver | Time |
|----------|------|
| 0 to cruise (8 m/s) | ~11 seconds |
| 0 to flank (15 m/s) | ~37 seconds |
| Cruise coast (neutral) to half speed | ~20 seconds |
| Flank coast (neutral) to half speed | ~14 seconds |
| Full coast to near-stop | ~60+ seconds |
| Turn radius at cruise | ~224m |
| Turn radius at flank | ~306m |

**Engine clutch mechanic:**

Three propulsion states:
- Engine engaged + throttle: normal powered operation, machinery noise
- Engine engaged + throttle zero: engine braking, prop acts as brake,
  faster deceleration, some machinery noise
- Engine disengaged (clutch out): freewheeling prop, hull drag only,
  longest coast, quietest state while moving (no machinery noise,
  just flow noise)

## Implications for SeaRobots Physics (Phase 4)

Based on this research, the Phase 4 physics upgrades should focus on:

1. **Submarine scaling:** 65m hull, 700 tons, with matching drag
   and thrust for realistic acceleration/coasting feel.

2. **Added mass:** Effective surge mass = dry mass + 50% added mass
   = 1,050,000 kg. This makes sprint-and-drift viable: coasting
   from cruise speed takes 20+ seconds to lose half speed.

3. **Engine clutch:** Separate clutch control for freewheeling vs
   engine braking. Freewheeling = longest coast, quietest. Engine
   braking = faster stop, some noise.

4. **Thrust lag:** 4-second spool-up creates commitment in speed
   decisions. Can't instantly sprint or brake.

5. **Control surface lift with stall:** v^2 authority scaling (not
   linear with speed) and stall at extreme deflections. Creates
   steerage way threshold.

6. **Arena and terrain scaling:** 7km radius arena, larger terrain
   features proportional to the bigger hull.

7. **Consider:** Gravity component along body axis for silent diving
   glide (needs careful tuning to avoid excessive terminal velocity).
   Current-drift at zero speed (subs should never be truly
   stationary).

---

## TMA in Submarine Games

Research on how submarine games present passive sonar and Target Motion
Analysis to the player.

### Cold Waters (most relevant to SeaRobots)

- TMA is fully automatic. Crew builds solution in background.
- Solution quality expressed as SOL% (0-100%).
- Bearing is accurate almost immediately.
- Range "pinballs all over" at low SOL, stabilizes at 85%+.
- At 85% SOL, you have a firing solution.
- Quality improves with: signal strength, correct classification,
  bearing rate (maneuvering), and time.
- Players spend 10-20 minutes at 5 knots building solutions.
- Design is accessible but somewhat opaque (no feedback on WHY
  solution improves or degrades).

### Dangerous Waters / Sub Command (most realistic)

- TMA is fully manual. Player must create tracks, assign trackers,
  and build solutions using a TMA ruler.
- Passive sonar gives bearing only, never range.
- Minimum 6 minutes for a decent solution.
- Requires multiple "legs" (own-ship course changes).
- 570-page manual. Very steep learning curve.

### Silent Hunter (WWII era)

- Hydrophone gives bearing only, no range.
- Player estimates range from bearing rate changes.
- "Fast 90" technique: attack from perpendicular using bearings alone.
- Speed estimated qualitatively from propeller sound.

### UBOAT

- Rough range estimates from hydrophone operator (patched to be less
  accurate after community feedback that it was unrealistically good).
- Full realistic mode requires manual triangulation over 30+ minutes.

### Key Design Decision for SeaRobots

Follow Cold Waters: engine-side automatic TMA with solution quality
that depends on the controller's behavior. Controllers read the results
directly. See `docs/passive-sonar-tma-design.md` for full specification.

### Real-World TMA Techniques

- **Ekelund Ranging (1958):** range = delta(crossTrackSpeed) /
  delta(bearingRate). Requires two legs with different headings.
  The foundational passive ranging technique.
- **Spiess Plot (1953):** Three bearings at known times define target
  motion. First complete bearings-only TMA.
- **Dot Stacking:** Plotting successive lines of bearing on a
  maneuvering board until they converge.
- **Bearing Rate Analysis:** Using maximum bearing rate at CPA.
- **Maximum Likelihood Estimator:** Computer algorithm fitting
  bearings to a linear motion model.

### Environmental Effects on TMA

- **Thermocline:** 5-10 dB cross-layer penalty. Contacts across the
  thermocline appear farther than they are.
- **Convergence Zones:** Sound refracted back to surface at ~30 nm
  intervals in deep water.
- **Shadow Zone:** Below thermocline, surface sounds blocked.
- **Terrain:** Ridges and islands block or weaken sound, disrupting
  TMA continuity and creating ranging errors.
- **Baffles:** Stern blind zone (130 deg from bow). No useful TMA
  data from contacts in baffles.
