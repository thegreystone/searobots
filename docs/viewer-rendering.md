# Viewer rendering

Island scenery, rendering settings, and tools for inspecting and regenerating visual assets.
See the [README](../README.md#running-the-viewer) for building and running the main viewer.
Run the commands below from the repository root after building the project.

## Rendering settings

The viewer uses 4x multisampling where the renderer supports it and a final antialiasing
pass, finer terrain textures, angle-dependent water reflections, and soft vegetation
contact shadows. F3 opens the render settings, including fine control over water highlights.

## Terrain and materials

Generated islands have seeded, gently rising beach shelves along suitable coast sections,
with rocky headlands and inland hills retained. The shaping preserves underwater terrain
and the land/water boundary used for navigation. The game shares the shoreline showcase's
PBR dry sand, wet sand, forest floor, and cliff rock, with metre-scale texture detail and
environment lighting. Submerged rock blends into finer gravel and irregular sediment deposits,
with a modest reflectance lift to keep nearby bottom detail visible through the water.
The terrain grid retains approximately eight-metre sampling in a
default world. Terrain retains that geometry at every distance so small islands and hills
remain beneath their trees; ordinary patch culling still limits offscreen work.

## Forests and scenery

Nearby island forests use modeled trunks, irregular branches, and individual foliage.
Spatial batches switch to simpler modeled crowns and then tree cutouts as the camera moves
away. Detailed batches are built gradually and cached within fixed geometry budgets, so a
large forest does not require every tree to have a detailed mesh resident at once. Nearby
modeled trees cast sun shadows; distant cutouts retain inexpensive contact shading.
The distant cutouts are baked from the same modeled tree variants, including their branches,
foliage colors, and leaf normals. Four side views and an overhead view use PBR lighting at
runtime, with the same per-tree variant, orientation, and tint. Their mipmaps retain each
view's silhouette without mixing neighboring atlas tiles. Distant trees fade before the
camera's clipping limit so their ground remains visible throughout the fade.
Nearby grass, low shrubs, and shoreline boulders are streamed in small batches and anchored
to the rendered terrain.
Scenery and tree batches are prepared in the background, with fixed active and resident
budgets. Distant tree regions are culled beyond six kilometres; the full terrain extent
remains visible at every camera height.

## Atmosphere and visibility

Atmosphere is enabled by default, with water transparency 0.10 and
underwater fog distance 700 to retain depth cues while keeping nearby hull markings clear.
F3's Atmosphere toggle disables air and underwater fog, daylight shading, local sun shadows, and nearby
ambient occlusion; the water and clouded HDR sky remain visible in either mode.


## Sonar and underwater effects

**S** toggles sonar pings in both the map and the 3D view. A faint spherical grid expands
from the submarine's position when it pinged, with a brighter contour where the wave reaches
the seabed. The bottom stays unlit until the sphere reaches it; terrain blocks the visual
wave from its source, including underwater ridges. The pulse uses simulation time, so pause,
step, replay, and playback speed apply to it. The animation depicts the outgoing ping;
the engine's sonar contacts still resolve immediately and use acoustic attenuation.

Bubble wakes render after the water surface so they remain visible when looking upward.
Their own material fades each particle with distance and the length of the underwater
viewing path, including crossings of the surface. Atmosphere and water settings also apply
to this attenuation.

The HUD uses bounded panels that adapt to the window width. **H** shows the full control
reference; the compact control strip stays visible during normal viewing.
**I** hides or shows the telemetry and tactical panels together with available competition
scores and details. These panels are hidden by default. The top-center playback panel stays
visible when toggling this information.
## Shoreline showcase

The standalone shoreline showcase is a small coastal cove for evaluating detailed scenery.
It uses original modeled trees with irregular branches and crowns, dense woodland pockets,
a half-metre terrain grid, PBR sand and rock, environment lighting, ambient occlusion,
and sun shadows. Run it after building the project:

```bash
mvn exec:exec -pl searobots-viewer -Dexec.mainClass=se.hirt.searobots.viewer.ShorelineScene
```

Use **1–5** for the coast, beach, forest, headland, and overview cameras, **WASD** to move,
**Q/Z** to move vertically, drag the mouse to look, **F1** to capture, and **H** to hide help.
For fixed-camera screenshots and a short rendering check, launch the packaged viewer:

```bash
# macOS; omit -XstartOnFirstThread on Linux/Windows
java -XstartOnFirstThread -XX:+UseZGC -Xmx2g \
  -cp searobots-viewer/target/searobots-viewer-0.1.2-SNAPSHOT.jar \
  se.hirt.searobots.viewer.ShorelineScene --capture /tmp/searobots-shoreline --seed 2c
```

The showcase is independent of match physics. Its original procedural assets use the project
license; the existing PBR ground textures bundled with jME are CC0 (see
`Textures/Terrain/PBR/license.txt` in `jme3-testdata`).

## Regenerating tree atlases

After changing the modeled tree archetypes, regenerate the distant-tree atlases from the
repository root (macOS; omit `-XstartOnFirstThread` on Linux/Windows):

```bash
java -XstartOnFirstThread -Djava.awt.headless=true \
  -cp searobots-viewer/target/searobots-viewer-0.1.2-SNAPSHOT.jar \
  se.hirt.searobots.viewer.tools.TreeImpostorBaker
```
