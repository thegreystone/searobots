These original tree atlases are generated from CoastalVegetation.TreeLibrary's NEAR meshes
by se.hirt.searobots.viewer.tools.TreeImpostorBaker. They use the project's license.

Each species has four side views and one overhead view of each modeled variant. Broadleaf
has three variants; conifer, palm and bush have two. Unused atlas columns are transparent.
The runtime and modeled trees share the same seeded variant selection.

Side atlas: 512 x 1024, four variant columns, four view rows, 128 x 256 texels per tile.
PNG rows from top: viewed from +Z, -X, -Z, +X. Top atlas: 512 x 128, 128 x 128 per tile.
The top camera views from +Y with -Z as its up vector.

Albedo is sRGB, with alpha coverage and the modeled vertex colors and bark/leaf textures.
Normals are linear tangent-space RGB, including the modeled bark normal map.
There is no baked sun, environment lighting, tone mapping or shadowing.
The loader builds alpha-preserving mipmaps for each view separately, without mixing tiles.

Regenerate from the repository root after rebuilding the viewer:
java -XstartOnFirstThread -Djava.awt.headless=true -cp searobots-viewer/target/searobots-viewer-0.1.2-SNAPSHOT.jar se.hirt.searobots.viewer.tools.TreeImpostorBaker
Omit -XstartOnFirstThread on Linux/Windows. An output-directory argument is optional.
