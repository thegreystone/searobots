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

import com.jme3.asset.AssetManager;
import com.jme3.bounding.BoundingBox;
import com.jme3.material.Material;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.control.AbstractControl;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.ModeledTree;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeDetail;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeKind;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeLibrary;

import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * Batches distant cards into large regions and replaces local cells with modeled trees. Detail is
 * selected from the main view camera, so water reflections and shadow passes share its selection.
 * One bounded background job prepares meshes at a time; updates only select cells and attach
 * completed batches atomically. A bounded cache retains recently visited cells.
 */
public final class IslandForestControl extends AbstractControl implements AutoCloseable {

	public static final float CELL_SIZE = 64f;
	public static final float CARD_REGION_SIZE = 512f;
	public static final float CARD_DISTANCE = 6000f;
	public static final float NEAR_DISTANCE = 95f;
	public static final float MIDDLE_DISTANCE = 350f;
	public static final int MAX_ACTIVE_CELLS = 20;
	public static final int MAX_ACTIVE_TREES = 2400;
	public static final int MAX_ACTIVE_TRIANGLES = 4_000_000;
	public static final int MAX_NEAR_TRIANGLES = 2_400_000;
	public static final int MAX_RESIDENT_CELLS = 32;
	public static final int MAX_RESIDENT_TRIANGLES = 6_000_000;

	private final AssetManager assets;
	private final Camera camera;
	private final long seed;
	private final List<Cell> cells;
	private final List<CardRegion> regions;
	private final Node forest;
	private final LinkedHashMap<CacheKey, DetailCell> cache = new LinkedHashMap<>(16, .75f, true);
	private final List<Cell> requested = new ArrayList<>();
	private final Vector3f lastPosition = new Vector3f(Float.POSITIVE_INFINITY, 0f, 0f);
	private final Vector3f lastDirection = new Vector3f();
	private TreeLibrary library;
	private float elapsed = 1f;
	private int residentTriangles, builds;
	private final int treeCount;
	private final ExecutorService worker = Executors.newSingleThreadExecutor(task -> {
		Thread thread = new Thread(task, "SeaRobots forest preparation");
		thread.setDaemon(true);
		thread.setPriority(Thread.NORM_PRIORITY - 1);
		return thread;
	});
	private Future<Prepared> pending;
	private Cell pendingCell;
	private int pendingTriangles;
	private boolean closed;
	private long lastPreparationNanos;

	IslandForestControl(Node forest, List<List<TreeScatter.TreeInstance>> perType, AssetManager assets, Camera camera,
			long seed) {
		this.forest = forest;
		this.assets = assets;
		this.camera = camera;
		this.seed = seed;
		Map<Long, Cell> grouped = new LinkedHashMap<>();
		int total = 0;
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			for (var tree : perType.get(type.ordinal())) {
				int x = (int) Math.floor(tree.x() / CELL_SIZE), z = (int) Math.floor(tree.z() / CELL_SIZE);
				long key = ((long) x << 32) | (z & 0xffffffffL);
				Cell cell = grouped.computeIfAbsent(key, k -> new Cell(k, x, z));
				cell.trees.get(type.ordinal()).add(tree);
				cell.treeCount++;
				cell.minimumY = Math.min(cell.minimumY, tree.y());
				cell.maximumY = Math.max(cell.maximumY, tree.y() + tree.h());
				cell.maximumWidth = Math.max(cell.maximumWidth, tree.w());
				total++;
			}
		}
		treeCount = total;
		cells = new ArrayList<>(grouped.values());
		Material[] sides = new Material[TreeScatter.Type.values().length], tops = new Material[sides.length];
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			if (perType.get(type.ordinal()).isEmpty())
				continue;
			sides[type.ordinal()] = TreeImpostors.material(assets, type, false);
			tops[type.ordinal()] = TreeImpostors.material(assets, type, true);
		}
		Map<Long, CardRegion> groupedRegions = new LinkedHashMap<>();
		int cellsPerRegion = (int) (CARD_REGION_SIZE / CELL_SIZE);
		for (Cell cell : cells) {
			cell.bounds = new BoundingBox(
					new Vector3f((cell.x + .5f) * CELL_SIZE, (cell.minimumY + cell.maximumY) * .5f,
							(cell.z + .5f) * CELL_SIZE),
					CELL_SIZE * .5f + cell.maximumWidth, (cell.maximumY - cell.minimumY) * .5f + 1f,
					CELL_SIZE * .5f + cell.maximumWidth);
			int x = Math.floorDiv(cell.x, cellsPerRegion), z = Math.floorDiv(cell.z, cellsPerRegion);
			long key = ((long) x << 32) | (z & 0xffffffffL);
			groupedRegions.computeIfAbsent(key, k -> new CardRegion(x, z)).cells.add(cell);
		}
		regions = new ArrayList<>(groupedRegions.values());
		for (CardRegion region : regions) {
			for (Cell cell : region.cells) {
				if (region.bounds == null)
					region.bounds = (BoundingBox) cell.bounds.clone();
				else
					region.bounds.mergeLocal(cell.bounds);
			}
			for (TreeScatter.Type type : TreeScatter.Type.values()) {
				List<TreeScatter.TreeInstance> trees = new ArrayList<>();
				for (Cell cell : region.cells)
					trees.addAll(cell.trees.get(type.ordinal()));
				if (trees.isEmpty())
					continue;
				Geometry side = card("distant " + type.name(), TreeScatter.buildImpostorCrossMesh(trees, type, seed),
						sides[type.ordinal()]);
				Geometry top = card("distant crowns " + type.name(),
						TreeScatter.buildImpostorTopMesh(trees, type, seed), tops[type.ordinal()]);
				region.node.attachChild(side);
				region.node.attachChild(top);
				CardIndices sideIndices = new CardIndices(side), topIndices = new CardIndices(top);
				int firstTree = 0;
				for (Cell cell : region.cells) {
					int count = cell.trees.get(type.ordinal()).size();
					if (count > 0) {
						cell.cardRanges.add(new CardRange(sideIndices, firstTree * 12, count * 12));
						cell.cardRanges.add(new CardRange(topIndices, firstTree * 24, count * 24));
					}
					firstTree += count;
				}
			}
			region.node.setShadowMode(RenderQueue.ShadowMode.Off);
			forest.attachChild(region.node);
		}
		selectCardRegions();
		if (camera != null && treeCount > 0)
			prepareLibrary();
		publishStatistics();
	}

	@Override
	protected void controlUpdate(float tpf) {
		if (closed || camera == null || cells.isEmpty())
			return;
		elapsed += tpf;
		selectCardRegions();
		collectPrepared();
		if (library != null && (elapsed >= .3f || camera.getLocation().distanceSquared(lastPosition) > 16f
				|| camera.getDirection().dot(lastDirection) < .985f)) {
			selectCells();
			lastPosition.set(camera.getLocation());
			lastDirection.set(camera.getDirection());
			elapsed = 0f;
		}
		if (library != null && pending == null) {
			for (Cell cell : requested) {
				CacheKey key = new CacheKey(cell.key, cell.wanted);
				if (cache.containsKey(key))
					continue;
				int triangles = estimate(cell, cell.wanted);
				makeRoom(triangles, cell);
				List<ModeledTree> trees = modeledTrees(cell);
				TreeLibrary archetypes = library;
				pendingCell = cell;
				pendingTriangles = triangles;
				pending = worker.submit(() -> {
					long start = System.nanoTime();
					Node model = archetypes.batch(trees, key.detail);
					return new Prepared(key, model, archetypes, triangles, System.nanoTime() - start);
				});
				break;
			}
		}
		publishStatistics();
	}

	private void prepareLibrary() {
		pending = worker.submit(() -> {
			long start = System.nanoTime();
			TreeLibrary archetypes = CoastalVegetation.treeLibrary(assets);
			return new Prepared(null, null, archetypes, 0, System.nanoTime() - start);
		});
	}

	private void collectPrepared() {
		if (pending == null || !pending.isDone())
			return;
		try {
			Prepared result = pending.get();
			library = result.library;
			lastPreparationNanos = result.elapsedNanos;
			if (result.key != null) {
				DetailCell detail = new DetailCell(result.key, result.node, pendingCell, result.triangles);
				cache.put(result.key, detail);
				residentTriangles += result.triangles;
				builds++;
			}
			// Re-select from the current camera before attaching a batch. A job may finish after
			// the camera has left its original view; it then remains only a bounded cache entry.
			elapsed = 1f;
		} catch (CancellationException ignored) {
			// A removed world cancels its single outstanding task; it must never reattach detail.
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			close();
		} catch (ExecutionException e) {
			close();
			throw new IllegalStateException("Cannot prepare local forest detail", e.getCause());
		} finally {
			pending = null;
			pendingCell = null;
			pendingTriangles = 0;
		}
	}

	private void selectCells() {
		requested.clear();
		List<Cell> visible = new ArrayList<>();
		Vector3f position = camera.getLocation();
		for (Cell cell : cells) {
			cell.wanted = null;
			cell.distance = distanceToBounds(position, cell.bounds);
			if (cell.distance <= MIDDLE_DISTANCE && inView(cell.bounds))
				visible.add(cell);
		}
		visible.sort(Comparator.comparingDouble((Cell cell) -> cell.distance).thenComparingLong(cell -> cell.key));
		int triangles = 0, nearTriangles = 0, trees = 0;
		for (Cell cell : visible) {
			if (requested.size() == MAX_ACTIVE_CELLS)
				break;
			TreeDetail detail = cell.distance <= NEAR_DISTANCE ? TreeDetail.NEAR : TreeDetail.MIDDLE;
			int cost = estimate(cell, detail);
			if (detail == TreeDetail.NEAR
					&& (triangles + cost > MAX_ACTIVE_TRIANGLES || nearTriangles + cost > MAX_NEAR_TRIANGLES)) {
				detail = TreeDetail.MIDDLE;
				cost = estimate(cell, detail);
			}
			if (triangles + cost > MAX_ACTIVE_TRIANGLES || trees + cell.treeCount > MAX_ACTIVE_TREES)
				continue;
			cell.wanted = detail;
			requested.add(cell);
			triangles += cost;
			if (detail == TreeDetail.NEAR)
				nearTriangles += cost;
			trees += cell.treeCount;
		}
		for (Cell cell : cells) {
			if (cell.wanted == null) {
				activate(cell, null);
				continue;
			}
			DetailCell ready = cache.get(new CacheKey(cell.key, cell.wanted));
			if (ready != null) {
				activate(cell, ready);
			} else if (cell.active != null && cell.wanted == TreeDetail.MIDDLE) {
				// Drop a more expensive tier immediately; the requested budget must hold while warming.
				activate(cell, null);
			}
		}
	}

	private int estimate(Cell cell, TreeDetail detail) {
		int ordinal = detail.ordinal();
		if (cell.triangles[ordinal] < 0) {
			int total = 0;
			for (TreeScatter.Type type : TreeScatter.Type.values()) {
				TreeKind kind = TreeKind.valueOf(type.name());
				for (var tree : cell.trees.get(type.ordinal()))
					total += library.triangleCount(kind, detail, variant(tree, type));
			}
			cell.triangles[ordinal] = total;
		}
		return cell.triangles[ordinal];
	}

	private List<ModeledTree> modeledTrees(Cell cell) {
		List<ModeledTree> result = new ArrayList<>(cell.treeCount);
		for (TreeScatter.Type type : TreeScatter.Type.values()) {
			TreeKind kind = TreeKind.valueOf(type.name());
			for (var tree : cell.trees.get(type.ordinal())) {
				result.add(new ModeledTree(kind, variant(tree, type), tree.x(), tree.y(), tree.z(), tree.w(), tree.h(),
						tree.rot(), tree.r(), tree.g(), tree.b()));
			}
		}
		return result;
	}

	private int variant(TreeScatter.TreeInstance tree, TreeScatter.Type type) {
		return TreeImpostors.variant(seed, tree, type);
	}

	private void makeRoom(int triangles, Cell replacement) {
		while (cache.size() >= MAX_RESIDENT_CELLS || residentTriangles + triangles > MAX_RESIDENT_TRIANGLES) {
			boolean removed = false;
			Iterator<Map.Entry<CacheKey, DetailCell>> iterator = cache.entrySet().iterator();
			while (iterator.hasNext()) {
				DetailCell entry = iterator.next().getValue();
				if (entry.cell.active == entry)
					continue;
				residentTriangles -= entry.triangles;
				iterator.remove();
				removed = true;
				break;
			}
			if (!removed && replacement.active != null) {
				// Usually the old middle tier can stay visible while its upgrade prepares. If its
				// allocation would exceed the hard resident limit, cards bridge this short interval.
				activate(replacement, null);
				continue;
			}
			if (!removed)
				throw new IllegalStateException("Active forest detail exceeds its resident budget");
		}
	}

	private void activate(Cell cell, DetailCell detail) {
		if (cell.active == detail)
			return;
		if (cell.active != null)
			cell.active.node.removeFromParent();
		boolean cardsEnabled = detail == null;
		if (cell.cardsEnabled != cardsEnabled) {
			// A cell owns contiguous ranges in up to eight shared region index buffers. Only
			// those indices change; trees never enter render queues as individual cell cards.
			for (CardRange range : cell.cardRanges)
				range.indices.enable(range.first, range.count, cardsEnabled);
			cell.cardsEnabled = cardsEnabled;
		}
		cell.active = detail;
		if (detail == null) {
			cell.node.removeFromParent();
		} else {
			cell.node.attachChild(detail.node);
			forest.attachChild(cell.node);
		}
	}

	private void selectCardRegions() {
		if (camera == null)
			return;
		for (CardRegion region : regions) {
			// Beyond a few kilometres individual trees are only pixels. Retain the landscape
			// silhouette while bounding card submissions even with the game's long far plane.
			boolean visible = distanceToBounds(camera.getLocation(), region.bounds) <= CARD_DISTANCE;
			region.node.setCullHint(visible ? Spatial.CullHint.Inherit : Spatial.CullHint.Always);
		}
	}

	private void publishStatistics() {
		Statistics statistics = statistics();
		forest.setUserData("treeCount", statistics.totalTrees());
		forest.setUserData("forestCellCount", statistics.cells());
		forest.setUserData("modeledTreeCount", statistics.modeledTrees());
		forest.setUserData("modeledTriangleCount", statistics.modeledTriangles());
		forest.setUserData("nearForestCells", statistics.nearCells());
		forest.setUserData("middleForestCells", statistics.middleCells());
		forest.setUserData("residentForestCells", statistics.residentCells());
		forest.setUserData("residentForestTriangles", statistics.residentTriangles());
		forest.setUserData("pendingForestPreparations", statistics.pendingJobs());
		forest.setUserData("distantForestRegions", statistics.cardRegions());
		forest.setUserData("visibleDistantForestRegions", statistics.visibleCardRegions());
		forest.setUserData("distantForestDrawObjects", statistics.cardDrawObjects());
	}

	public Statistics statistics() {
		int trees = 0, triangles = 0, near = 0, middle = 0;
		for (Cell cell : cells) {
			if (cell.active == null)
				continue;
			trees += cell.treeCount;
			triangles += cell.active.triangles;
			if (cell.active.key.detail == TreeDetail.NEAR)
				near++;
			else
				middle++;
		}
		int visibleRegions = 0, cardDraws = 0;
		for (CardRegion region : regions) {
			if (region.node.getCullHint() != Spatial.CullHint.Always) {
				visibleRegions++;
				cardDraws += region.node.getQuantity();
			}
		}
		return new Statistics(treeCount, cells.size(), trees, near, middle, triangles, cache.size(), residentTriangles,
				builds, pending == null ? 0 : 1, pendingTriangles, lastPreparationNanos / 1_000_000.0, regions.size(),
				visibleRegions, cardDraws);
	}

	public record Statistics(int totalTrees, int cells, int modeledTrees, int nearCells, int middleCells,
			int modeledTriangles, int residentCells, int residentTriangles, int cellBuilds, int pendingJobs,
			int pendingTriangles, double lastPreparationMillis, int cardRegions, int visibleCardRegions,
			int cardDrawObjects) {
	}

	/**
	 * Cancels preparation and releases this world's resident batches without blocking rendering.
	 */
	@Override
	public void close() {
		if (closed)
			return;
		closed = true;
		setEnabled(false);
		if (pending != null)
			pending.cancel(true);
		pending = null;
		pendingCell = null;
		pendingTriangles = 0;
		worker.shutdownNow();
		for (Cell cell : cells)
			activate(cell, null);
		cache.clear();
		requested.clear();
		residentTriangles = 0;
		publishStatistics();
	}

	public void dispose() {
		close();
	}

	public boolean isClosed() {
		return closed;
	}

	@Override
	public void setSpatial(Spatial spatial) {
		if (spatial == null)
			close();
		super.setSpatial(spatial);
	}

	// Package-visible waits make completion/cancellation tests deterministic. Neither wait is
	// used by the application: production updates only collect an already completed Future.
	boolean awaitPreparation(long timeoutMillis) throws InterruptedException {
		Future<Prepared> future = pending;
		if (future == null)
			return true;
		try {
			future.get(timeoutMillis, TimeUnit.MILLISECONDS);
			return true;
		} catch (TimeoutException e) {
			return false;
		} catch (CancellationException e) {
			return true;
		} catch (ExecutionException e) {
			throw new IllegalStateException("Cannot prepare forest detail", e.getCause());
		}
	}

	boolean awaitShutdown(long timeoutMillis) throws InterruptedException {
		return worker.awaitTermination(timeoutMillis, TimeUnit.MILLISECONDS);
	}

	private boolean inView(BoundingBox bounds) {
		int previous = camera.getPlaneState();
		camera.setPlaneState(0);
		boolean visible = camera.contains(bounds) != Camera.FrustumIntersect.Outside;
		camera.setPlaneState(previous);
		return visible;
	}

	private static float distanceToBounds(Vector3f point, BoundingBox box) {
		Vector3f center = box.getCenter();
		float dx = Math.max(0f, Math.abs(point.x - center.x) - box.getXExtent());
		float dy = Math.max(0f, Math.abs(point.y - center.y) - box.getYExtent());
		float dz = Math.max(0f, Math.abs(point.z - center.z) - box.getZExtent());
		return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	private static Geometry card(String name, com.jme3.scene.Mesh mesh, Material material) {
		mesh.setStatic();
		Geometry geometry = new Geometry(name, mesh);
		geometry.setMaterial(material);
		geometry.setShadowMode(RenderQueue.ShadowMode.Off);
		return geometry;
	}

	@Override
	protected void controlRender(RenderManager manager, ViewPort viewport) {
		// Selection belongs to the explicit main camera, never the reflection or sun camera.
	}

	private record Prepared(CacheKey key, Node node, TreeLibrary library, int triangles, long elapsedNanos) {
	}

	private record CacheKey(long cell, TreeDetail detail) {
	}

	private record DetailCell(CacheKey key, Node node, Cell cell, int triangles) {
	}

	private static final class CardRegion {
		final Node node;
		final List<Cell> cells = new ArrayList<>();
		BoundingBox bounds;

		CardRegion(int x, int z) {
			node = new Node("distant forest region " + x + ":" + z);
		}
	}

	private record CardRange(CardIndices indices, int first, int count) {
	}

	private static final class CardIndices {
		final VertexBuffer buffer;
		final IntBuffer indices;
		final int[] original;

		CardIndices(Geometry geometry) {
			buffer = geometry.getMesh().getBuffer(VertexBuffer.Type.Index);
			buffer.setUsage(VertexBuffer.Usage.Dynamic);
			indices = (IntBuffer) buffer.getData();
			original = new int[indices.limit()];
			for (int i = 0; i < original.length; i++)
				original[i] = indices.get(i);
		}

		void enable(int first, int count, boolean enabled) {
			// Disabled triangles collapse to a single existing vertex and rasterize no pixels.
			// Restoring the original range preserves topology, bounds and every other cell.
			for (int i = first; i < first + count; i++)
				indices.put(i, enabled ? original[i] : 0);
			buffer.updateData(indices);
		}
	}

	private static final class Cell {
		final long key;
		final int x, z;
		final Node node;
		final List<CardRange> cardRanges = new ArrayList<>();
		boolean cardsEnabled = true;
		final List<List<TreeScatter.TreeInstance>> trees = new ArrayList<>();
		final int[] triangles = {-1, -1};
		int treeCount;
		float minimumY = Float.POSITIVE_INFINITY, maximumY = Float.NEGATIVE_INFINITY, maximumWidth, distance;
		BoundingBox bounds;
		TreeDetail wanted;
		DetailCell active;

		Cell(long key, int x, int z) {
			this.key = key;
			this.x = x;
			this.z = z;
			node = new Node("forest cell " + x + ":" + z);
			for (TreeScatter.Type ignored : TreeScatter.Type.values())
				trees.add(new ArrayList<>());
		}
	}
}
