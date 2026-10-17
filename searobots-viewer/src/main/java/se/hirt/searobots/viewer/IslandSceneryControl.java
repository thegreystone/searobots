/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * This software is free:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package se.hirt.searobots.viewer;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingBox;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;

import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.viewer.IslandSceneryMeshes.Builder;
import se.hirt.searobots.viewer.shoreline.ShorelineTerrain;

/**
 * Streams nearby grass pockets, low shrubs and coastal stones in bounded material batches. One
 * background job constructs CPU meshes from an immutable copy of the rendered terrain; scene
 * attachment and material loading happen on the render thread. Call {@link #close()} on removal.
 */
public final class IslandSceneryControl extends AbstractControl implements AutoCloseable {

	public static final float CELL_SIZE = 32;
	/** Cell-centre range; the farthest blade stays within about 70 metres. */
	public static final float GRASS_DISTANCE = 48;
	public static final float SCENERY_DISTANCE = 180;
	public static final int MAX_ACTIVE_CELLS = 28;
	public static final int MAX_GRASS_CELLS = 8;
	public static final int MAX_ACTIVE_TRIANGLES = 180_000;
	public static final int MAX_ACTIVE_DRAWS = 96;
	public static final int MAX_RESIDENT_CELLS = 40;
	public static final int MAX_RESIDENT_TRIANGLES = 400_000;
	private static final int MAX_REQUESTED_CELLS = 32;
	private static final int MAX_GRASS_CLUMPS = 120;
	private static final int MAX_SHRUBS = 8;
	private static final int MAX_ROCKS = 3;

	private final Node scenery;
	private final TerrainMap terrain;
	private final IslandScenerySurface surface;
	private final AssetManager assets;
	private final Camera camera;
	private final long seed;
	private final LinkedHashMap<Key, CellData> cache = new LinkedHashMap<>(40, .75f, true);
	private final List<Request> requested = new ArrayList<>();
	private final Vector3f lastPosition = new Vector3f(Float.POSITIVE_INFINITY, 0, 0);
	private final Vector3f lastDirection = new Vector3f();
	private ExecutorService worker;
	private Future<CellData> pending;
	private Library library;
	private int residentTriangles, builds;
	private float elapsed = 1;
	private boolean closed;

	public static Node create(
		TerrainMap terrain, com.jme3.terrain.geomipmap.TerrainQuad quad, AssetManager assets, Camera camera,
		long seed) {
		Node node = new Node("Nearby island scenery");
		node.addControl(new IslandSceneryControl(node, terrain, new IslandScenerySurface(quad), assets, camera, seed));
		return node;
	}

	IslandSceneryControl(Node node, TerrainMap terrain, IslandScenerySurface surface, AssetManager assets,
			Camera camera, long seed) {
		scenery = node;
		this.terrain = terrain;
		this.surface = surface;
		this.assets = assets;
		this.camera = camera;
		this.seed = seed;
		publishStatistics();
	}

	@Override
	protected void controlUpdate(float tpf) {
		if (closed || camera == null)
			return;
		elapsed += tpf;
		if (elapsed >= .25f || camera.getLocation().distanceSquared(lastPosition) > 16
				|| camera.getDirection().dot(lastDirection) < .985f) {
			selectCells();
			lastPosition.set(camera.getLocation());
			lastDirection.set(camera.getDirection());
			elapsed = 0;
		}
		if (pending != null && pending.isDone()) {
			try {
				CellData ready = pending.get();
				makeRoom(ready.triangles);
				cache.put(ready.key, ready);
				residentTriangles += ready.triangles;
				builds++;
			} catch (Exception e) {
				close();
				throw new IllegalStateException("Cannot prepare island scenery", e);
			} finally {
				pending = null;
			}
		}
		activateRequested();
		if (pending == null && !requested.isEmpty()) {
			for (Request request : requested) {
				if (cache.containsKey(request.key))
					continue;
				if (library == null)
					library = new Library(assets);
				if (worker == null) {
					worker = Executors.newSingleThreadExecutor(task -> {
						Thread thread = new Thread(task, "Island scenery preparation");
						thread.setDaemon(true);
						return thread;
					});
				}
				pending = worker.submit(() -> buildCell(request.key, surface, terrain, library, seed));
				break;
			}
		}
		publishStatistics();
	}

	private void selectCells() {
		requested.clear();
		Vector3f position = camera.getLocation();
		int cx = (int) Math.floor(position.x / CELL_SIZE), cz = (int) Math.floor(position.z / CELL_SIZE);
		int radius = (int) Math.ceil(SCENERY_DISTANCE / CELL_SIZE);
		List<Request> candidates = new ArrayList<>();
		for (int z = cz - radius; z <= cz + radius; z++) {
			for (int x = cx - radius; x <= cx + radius; x++) {
				float wx = (x + .5f) * CELL_SIZE, wz = (z + .5f) * CELL_SIZE;
				float height = surface.height(wx, wz);
				if (!Float.isFinite(height))
					continue;
				float minimum = height, maximum = height;
				for (int dx : new int[] {-1, 1}) {
					for (int dz : new int[] {-1, 1}) {
						float corner = surface.height(wx + dx * 15, wz + dz * 15);
						if (Float.isFinite(corner)) {
							minimum = Math.min(minimum, corner);
							maximum = Math.max(maximum, corner);
						}
					}
				}
				if (maximum < -1)
					continue;
				float distance = position.distance(new Vector3f(wx, height + 1, wz));
				if (distance > SCENERY_DISTANCE)
					continue;
				BoundingBox box = new BoundingBox(new Vector3f(wx, (minimum + maximum) * .5f + 2, wz), 18,
						(maximum - minimum) * .5f + 4, 18);
				int state = camera.getPlaneState();
				boolean visible;
				try {
					camera.setPlaneState(0);
					visible = camera.contains(box) != Camera.FrustumIntersect.Outside;
				} finally {
					camera.setPlaneState(state);
				}
				if (visible)
					candidates.add(new Request(new Key(x, z), distance));
			}
		}
		candidates.sort(Comparator.comparingDouble(Request::distance).thenComparingInt(r -> r.key.x)
				.thenComparingInt(r -> r.key.z));
		requested.addAll(candidates.subList(0, Math.min(candidates.size(), MAX_REQUESTED_CELLS)));
	}

	private void activateRequested() {
		for (CellData cell : cache.values())
			cell.selected = false;
		int cells = 0, grassCells = 0, triangles = 0, draws = 0;
		for (Request request : requested) {
			CellData cell = cache.get(request.key);
			if (cell == null || cell.triangles == 0 || cells == MAX_ACTIVE_CELLS)
				continue;
			boolean grass = cell.grass != null && request.distance <= GRASS_DISTANCE && grassCells < MAX_GRASS_CELLS;
			int cost = cell.triangles - (grass ? 0 : cell.grassTriangles);
			int drawCost = cell.draws - (!grass && cell.grass != null ? 1 : 0);
			if (triangles + cost > MAX_ACTIVE_TRIANGLES && grass) {
				grass = false;
				cost -= cell.grassTriangles;
				drawCost--;
			}
			if (cost == 0 || triangles + cost > MAX_ACTIVE_TRIANGLES || draws + drawCost > MAX_ACTIVE_DRAWS)
				continue;
			if (cell.grass != null) {
				Spatial.CullHint hint = grass ? Spatial.CullHint.Inherit : Spatial.CullHint.Always;
				if (cell.grass.getCullHint() != hint)
					cell.grass.setCullHint(hint);
			}
			cell.grassActive = grass;
			cell.selected = true;
			cells++;
			triangles += cost;
			draws += drawCost;
			if (grass)
				grassCells++;
		}
		for (CellData cell : cache.values()) {
			if (cell.active == cell.selected)
				continue;
			if (cell.selected)
				scenery.attachChild(cell.node);
			else
				cell.node.removeFromParent();
			cell.active = cell.selected;
		}
	}

	private void makeRoom(int triangles) {
		while (cache.size() >= MAX_RESIDENT_CELLS || residentTriangles + triangles > MAX_RESIDENT_TRIANGLES) {
			Iterator<Map.Entry<Key, CellData>> iterator = cache.entrySet().iterator();
			boolean removed = false;
			while (iterator.hasNext()) {
				CellData cell = iterator.next().getValue();
				if (cell.active)
					continue;
				residentTriangles -= cell.triangles;
				iterator.remove();
				removed = true;
				break;
			}
			if (!removed)
				throw new IllegalStateException("Active island scenery exceeds the resident budget");
		}
	}

	static CellData buildCell(Key key, IslandScenerySurface surface, TerrainMap map, Library library, long seed) {
		library.prepareMeshes();
		Random random = new Random(seed ^ ((long) key.x << 32) ^ (key.z & 0xffffffffL) ^ 0x51cea43de12L);
		Builder grass = new Builder(false), shrubs = new Builder(false), dry = new Builder(true),
				wet = new Builder(true);
		int grassCount = 0, shrubCount = 0, rockCount = 0;
		float baseX = key.x * CELL_SIZE, baseZ = key.z * CELL_SIZE;
		int[] slots = new int[19 * 19];
		for (int index = 0; index < slots.length; index++)
			slots[index] = index;
		for (int index = slots.length - 1; index > 0; index--) {
			int other = random.nextInt(index + 1), swap = slots[index];
			slots[index] = slots[other];
			slots[other] = swap;
		}
		for (int slot : slots) {
			if (grassCount == MAX_GRASS_CLUMPS)
				break;
			int row = slot / 19, col = slot % 19;
			float x = baseX + (col + random.nextFloat()) * CELL_SIZE / 19,
					z = baseZ + (row + random.nextFloat()) * CELL_SIZE / 19;
			float height = surface.height(x, z), slope = surface.slope(x, z);
			float pocket = pocket(x, z, seed);
			if (!plantable(map, height, slope, x, z) || pocket < .28f || random.nextFloat() > pocket * .65f)
				continue;
			grass.append(library.grass[random.nextInt(3)], surface, x, height, z, 1, .8f + random.nextFloat() * .5f, 1,
					random.nextFloat() * 6.283185f, .8f + random.nextFloat() * .3f, true);
			grassCount++;
		}
		for (int attempt = 0; attempt < 22 && shrubCount < MAX_SHRUBS; attempt++) {
			float x = baseX + 1 + random.nextFloat() * 30, z = baseZ + 1 + random.nextFloat() * 30;
			float height = surface.height(x, z), slope = surface.slope(x, z);
			if (!plantable(map, height, slope, x, z) || height < 13 || pocket(x, z, seed) < .43f)
				continue;
			float scale = .75f + random.nextFloat() * 1.1f;
			shrubs.append(library.shrubs[random.nextInt(3)], surface, x, height, z, scale, scale, scale,
					random.nextFloat() * 6.283185f, .82f + random.nextFloat() * .25f, false);
			shrubCount++;
		}
		for (int attempt = 0; attempt < 12 && rockCount < MAX_ROCKS; attempt++) {
			float x = baseX + 2 + random.nextFloat() * 28, z = baseZ + 2 + random.nextFloat() * 28;
			float height = surface.height(x, z), slope = surface.slope(x, z);
			if (!Float.isFinite(height) || !Float.isFinite(slope) || height < -.9f || height > 65
					|| height > 1.2f && slope < .38f || height <= 1.2f && slope < .16f && random.nextFloat() > .04f)
				continue;
			float radius = .35f + random.nextFloat() * (height < 1.2f ? 1.1f : 1.9f);
			Builder stones = height < 1.2f ? wet : dry;
			stones.append(library.rocks[random.nextInt(4)], surface, x, height + radius * .25f, z,
					radius * (.8f + random.nextFloat() * .45f), radius * (.5f + random.nextFloat() * .4f), radius,
					random.nextFloat() * 6.283185f, 1, false);
			rockCount++;
		}
		Node node = new Node("Island scenery " + key.x + "," + key.z);
		Geometry grassGeometry = attach(node, "grass pockets", grass, library.foliage, RenderQueue.ShadowMode.Receive);
		attach(node, "low shrubs", shrubs, library.foliage, RenderQueue.ShadowMode.CastAndReceive);
		attach(node, "dry coastal stones", dry, library.dryRock, RenderQueue.ShadowMode.CastAndReceive);
		attach(node, "wet tide stones", wet, library.wetRock, RenderQueue.ShadowMode.CastAndReceive);
		node.updateGeometricState();
		int triangles = 0;
		for (Spatial child : node.getChildren())
			triangles += ((Geometry) child).getMesh().getTriangleCount();
		return new CellData(key, node, grassGeometry, triangles,
				grassGeometry == null ? 0 : grassGeometry.getMesh().getTriangleCount(), node.getQuantity(), grassCount,
				shrubCount, rockCount);
	}

	private static boolean plantable(TerrainMap map, float height, float slope, float x, float z) {
		return Float.isFinite(height) && Float.isFinite(slope) && height >= 11.5f && height < 140 && slope < .65f
				&& map.elevationAt(x, -z) > 5;
	}

	private static float pocket(float x, float z, long seed) {
		float phase = (seed & 0xffff) * .0017f;
		return (float) (.5 + .24 * Math.sin(x * .059 + phase + Math.sin(z * .031))
				+ .24 * Math.cos(z * .073 - phase + Math.sin(x * .043)));
	}

	private static Geometry attach(
		Node node, String name, Builder builder, Material material, RenderQueue.ShadowMode shadow) {
		if (builder.isEmpty())
			return null;
		Geometry geometry = new Geometry(name, builder.mesh());
		geometry.setMaterial(material);
		geometry.setShadowMode(shadow);
		node.attachChild(geometry);
		return geometry;
	}

	static final class Library {
		final Material foliage, dryRock, wetRock;
		Mesh[] grass, shrubs, rocks;

		Library(AssetManager assets) {
			foliage = new Material(assets, "Common/MatDefs/Light/PBRLighting.j3md");
			foliage.setColor("BaseColor", ColorRGBA.White);
			foliage.setFloat("Metallic", 0);
			foliage.setFloat("Roughness", .92f);
			foliage.setBoolean("UseVertexColor", true);
			foliage.setBoolean("BackfaceShadows", true);
			foliage.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
			dryRock = ShorelineTerrain.rockMaterial(assets, false);
			wetRock = ShorelineTerrain.rockMaterial(assets, true);
		}

		void prepareMeshes() {
			if (grass != null)
				return;
			grass = new Mesh[3];
			shrubs = new Mesh[3];
			rocks = new Mesh[4];
			for (int i = 0; i < 3; i++) {
				grass[i] = IslandSceneryMeshes.grass(0x63a550L + i);
				shrubs[i] = IslandSceneryMeshes.shrub(0xb055e5L + i);
			}
			for (int i = 0; i < 4; i++)
				rocks[i] = ShorelineTerrain.boulderMesh(i * 1.7f);
		}
	}

	public record Statistics(int activeCells, int grassClumps, int shrubs, int rocks, int activeTriangles,
			int activeDraws, int residentCells, int residentTriangles, int pendingJobs, int completedCells) {
	}

	public Statistics statistics() {
		int cells = 0, grass = 0, shrubs = 0, rocks = 0, triangles = 0, draws = 0;
		for (CellData cell : cache.values()) {
			if (!cell.active)
				continue;
			cells++;
			grass += cell.grassActive ? cell.grassCount : 0;
			shrubs += cell.shrubCount;
			rocks += cell.rockCount;
			triangles += cell.triangles - (cell.grassActive ? 0 : cell.grassTriangles);
			draws += cell.draws - (!cell.grassActive && cell.grass != null ? 1 : 0);
		}
		return new Statistics(cells, grass, shrubs, rocks, triangles, draws, cache.size(), residentTriangles,
				pending == null ? 0 : 1, builds);
	}

	private void publishStatistics() {
		Statistics stats = statistics();
		scenery.setUserData("sceneryTriangles", stats.activeTriangles());
		scenery.setUserData("sceneryDraws", stats.activeDraws());
		scenery.setUserData("grassClumps", stats.grassClumps());
		scenery.setUserData("shrubs", stats.shrubs());
		scenery.setUserData("rocks", stats.rocks());
		scenery.setUserData("residentSceneryCells", stats.residentCells());
	}

	@Override
	public void close() {
		closed = true;
		if (pending != null)
			pending.cancel(true);
		pending = null;
		if (worker != null)
			worker.shutdownNow();
		scenery.detachAllChildren();
		cache.clear();
		requested.clear();
		residentTriangles = 0;
		publishStatistics();
	}

	@Override
	public void setSpatial(Spatial spatial) {
		super.setSpatial(spatial);
		if (spatial == null)
			close();
	}

	@Override
	protected void controlRender(RenderManager manager, ViewPort viewPort) {
	}

	static record Key(int x, int z) {
	}

	private record Request(Key key, float distance) {
	}

	static final class CellData {
		final Key key;
		final Node node;
		final Geometry grass;
		final int triangles, grassTriangles, draws, grassCount, shrubCount, rockCount;
		boolean active, grassActive, selected;

		CellData(Key key, Node node, Geometry grass, int triangles, int grassTriangles, int draws, int grassCount,
				int shrubCount, int rockCount) {
			this.key = key;
			this.node = node;
			this.grass = grass;
			this.triangles = triangles;
			this.grassTriangles = grassTriangles;
			this.draws = draws;
			this.grassCount = grassCount;
			this.shrubCount = shrubCount;
			this.rockCount = rockCount;
		}
	}
}
