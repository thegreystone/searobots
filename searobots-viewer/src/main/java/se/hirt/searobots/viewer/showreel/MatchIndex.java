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
package se.hirt.searobots.viewer.showreel;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import se.hirt.searobots.engine.SimulationListener;
import se.hirt.searobots.engine.SubmarineSnapshot;
import se.hirt.searobots.engine.TorpedoSnapshot;

/**
 * A timeline of one match: when the dramatic things happen (launches, sonar pings, terminal
 * approaches, hits, kills) and how good the scenery around each submarine is along the way. Built
 * as a {@link SimulationListener}, either while a match runs or by replaying a recording through
 * it. The showreel plans its shots from this, so the camera can be in place before an event.
 */
public final class MatchIndex implements SimulationListener {
	/** Ticks between scenery samples (0.5 s at 50 Hz). */
	static final int SAMPLE_TICKS = 25;
	/** A torpedo closer than this to an enemy is on its terminal approach (as in the Director). */
	static final double TERMINAL_RANGE = 600;

	public enum Kind {
		/** A torpedo left its tube: subject torpedo, other launcher. */
		LAUNCH,
		/** A submarine pinged with active sonar: subject submarine. */
		PING,
		/** A submarine gained a firing solution: subject submarine, value quality. */
		SOLUTION,
		/**
		 * A torpedo closed within {@value #TERMINAL_RANGE} m of an enemy: subject torpedo, other
		 * target.
		 */
		TERMINAL,
		/** A torpedo's closest pass of an enemy: subject torpedo, other target, value distance. */
		CLOSEST,
		/** A torpedo went off: subject torpedo, other nearest submarine, value its distance. */
		DETONATION,
		/** A submarine lost hit points: subject submarine, value the loss. */
		HIT,
		/** A submarine was destroyed: subject submarine. */
		DEATH,
		/** The last recorded tick. */
		END
	}

	public record Event(long tick, Kind kind, int subject, int other, double x, double y, double z, double value) {
	}

	/** A submarine's position and surroundings at one sample tick. */
	public record Sample(long tick, int subId, double x, double y, double z, double heading, double speed,
			boolean alive, TerrainSurvey.Scenery scenery) {
	}

	/** A stretch of a submarine's track with good scenery; {@code peakTick} is the best moment. */
	public record ScenicWindow(int subId, long startTick, long endTick, long peakTick, double peakScore,
			double featureBearing) {
		public long duration() {
			return endTick - startTick;
		}
	}

	private final TerrainSurvey survey;
	private final int tickRateHz;
	private final List<Event> events = new ArrayList<>();
	private final Map<Integer, List<Sample>> samples = new HashMap<>();
	private final Map<Integer, String> names = new HashMap<>();

	// Edge detection
	private final Set<Integer> torpedoesSeen = new HashSet<>();
	private final Set<Integer> torpedoesDone = new HashSet<>();
	private final Set<Integer> terminal = new HashSet<>();
	private final Set<Integer> pinging = new HashSet<>();
	private final Set<Integer> solutions = new HashSet<>();
	private final Map<Integer, Integer> hp = new HashMap<>();
	private final Map<Integer, double[]> closest = new HashMap<>(); // torpedo -> {distance, tick, target}
	private long lastTick = -1;

	public MatchIndex(TerrainSurvey survey, int tickRateHz) {
		this.survey = survey;
		this.tickRateHz = tickRateHz;
	}

	@Override
	public void onTick(long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torpedoes) {
		lastTick = tick;
		for (var s : subs) {
			names.putIfAbsent(s.id(), s.name());
			var p = s.pose().position();
			int before = hp.getOrDefault(s.id(), s.hp());
			if (s.hp() < before)
				add(tick, Kind.HIT, s.id(), -1, p.x(), p.y(), p.z(), before - s.hp());
			if (s.hp() <= 0 && before > 0)
				add(tick, Kind.DEATH, s.id(), -1, p.x(), p.y(), p.z(), 0);
			hp.put(s.id(), s.hp());
			if (s.hp() > 0 && s.pingRequested() && pinging.add(s.id()))
				add(tick, Kind.PING, s.id(), -1, p.x(), p.y(), p.z(), 0);
			else if (!s.pingRequested())
				pinging.remove(s.id());
			if (s.hp() > 0 && s.firingSolution() != null && solutions.add(s.id()))
				add(tick, Kind.SOLUTION, s.id(), -1, p.x(), p.y(), p.z(), s.firingSolution().quality());
			else if (s.firingSolution() == null)
				solutions.remove(s.id());
			if (tick % SAMPLE_TICKS == 0)
				samples.computeIfAbsent(s.id(), id -> new ArrayList<>()).add(new Sample(tick, s.id(), p.x(), p.y(),
						p.z(), s.pose().heading(), s.speed(), s.hp() > 0, survey.sceneryAt(p.x(), p.y(), p.z())));
		}

		Set<Integer> present = new HashSet<>();
		for (var t : torpedoes) {
			present.add(t.id());
			var p = t.pose().position();
			if (t.alive() && torpedoesSeen.add(t.id()))
				add(tick, Kind.LAUNCH, t.id(), t.ownerId(), p.x(), p.y(), p.z(), 0);
			if (!t.alive() && !t.detonated())
				continue;
			for (var s : subs) {
				if (s.id() == t.ownerId() || s.hp() <= 0)
					continue;
				double d = p.distanceTo(s.pose().position());
				double[] best = closest.computeIfAbsent(t.id(), id -> new double[] {Double.MAX_VALUE, 0, -1});
				if (d < best[0]) {
					best[0] = d;
					best[1] = tick;
					best[2] = s.id();
				}
				if (t.alive() && d < TERMINAL_RANGE && terminal.add(t.id()))
					add(tick, Kind.TERMINAL, t.id(), s.id(), p.x(), p.y(), p.z(), d);
			}
			if (t.detonated() && torpedoesDone.add(t.id())) {
				int nearest = -1;
				double nearestDistance = Double.MAX_VALUE;
				for (var s : subs) {
					double d = p.distanceTo(s.pose().position());
					if (d < nearestDistance) {
						nearestDistance = d;
						nearest = s.id();
					}
				}
				add(tick, Kind.DETONATION, t.id(), nearest, p.x(), p.y(), p.z(), nearestDistance);
				finishTorpedo(t.id());
			}
		}
		for (int id : torpedoesSeen)
			if (!present.contains(id) && torpedoesDone.add(id))
				finishTorpedo(id);
	}

	private void finishTorpedo(int id) {
		double[] best = closest.get(id);
		if (best != null && best[2] >= 0)
			add((long) best[1], Kind.CLOSEST, id, (int) best[2], Double.NaN, Double.NaN, Double.NaN, best[0]);
	}

	@Override
	public void onMatchEnd() {
		for (int id : torpedoesSeen)
			if (torpedoesDone.add(id))
				finishTorpedo(id);
		if (lastTick >= 0)
			add(lastTick, Kind.END, -1, -1, Double.NaN, Double.NaN, Double.NaN, 0);
		events.sort((a, b) -> Long.compare(a.tick(), b.tick()));
	}

	private void add(long tick, Kind kind, int subject, int other, double x, double y, double z, double value) {
		events.add(new Event(tick, kind, subject, other, x, y, z, value));
	}

	public int tickRateHz() {
		return tickRateHz;
	}

	public TerrainSurvey survey() {
		return survey;
	}

	public List<Event> events() {
		return Collections.unmodifiableList(events);
	}

	public List<Event> events(Kind kind) {
		return events.stream().filter(e -> e.kind() == kind).toList();
	}

	public long endTick() {
		return lastTick;
	}

	public String name(int subId) {
		return names.getOrDefault(subId, "#" + subId);
	}

	public List<Integer> submarineIds() {
		return samples.keySet().stream().sorted().toList();
	}

	public List<Sample> samples(int subId) {
		return samples.getOrDefault(subId, List.of());
	}

	/** The sample of {@code subId} nearest {@code tick}, or null. */
	public Sample sampleAt(int subId, long tick) {
		var list = samples(subId);
		if (list.isEmpty())
			return null;
		int i = (int) Math.max(0, Math.min(list.size() - 1, Math.round((double) tick / SAMPLE_TICKS)));
		return list.get(i);
	}

	/**
	 * Stretches of at least {@code minTicks} where a living submarine's scenery, averaged over two
	 * seconds, stays at or above {@code threshold}. Best first.
	 */
	public List<ScenicWindow> scenicWindows(double threshold, long minTicks) {
		var windows = new ArrayList<ScenicWindow>();
		for (var entry : samples.entrySet()) {
			var list = entry.getValue();
			int n = list.size();
			double[] smooth = new double[n];
			for (int i = 0; i < n; i++) {
				double sum = 0;
				int count = 0;
				for (int k = Math.max(0, i - 2); k <= Math.min(n - 1, i + 2); k++) {
					sum += list.get(k).alive() ? list.get(k).scenery().score() : 0;
					count++;
				}
				smooth[i] = sum / count;
			}
			int start = -1;
			for (int i = 0; i <= n; i++) {
				boolean good = i < n && smooth[i] >= threshold && list.get(i).alive();
				if (good && start < 0)
					start = i;
				if (!good && start >= 0) {
					int peak = start;
					for (int k = start; k < i; k++)
						if (smooth[k] > smooth[peak])
							peak = k;
					long from = list.get(start).tick(), to = list.get(i - 1).tick();
					if (to - from >= minTicks)
						windows.add(new ScenicWindow(entry.getKey(), from, to, list.get(peak).tick(), smooth[peak],
								list.get(peak).scenery().featureBearing()));
					start = -1;
				}
			}
		}
		windows.sort((a, b) -> Double.compare(b.peakScore(), a.peakScore()));
		return windows;
	}

	/** Whether a torpedo's detonation or closest pass actually damaged its target. */
	public boolean hit(int torpedoId) {
		for (var d : events(Kind.DETONATION)) {
			if (d.subject() != torpedoId)
				continue;
			for (var h : events(Kind.HIT))
				if (h.subject() == d.other() && Math.abs(h.tick() - d.tick()) <= tickRateHz)
					return true;
		}
		return false;
	}

	/**
	 * How exciting the match is to watch, 0..100: kills, hits, near misses and salvoes, early
	 * enough to tell in a short film, and with scenery to show while the submarines hunt.
	 */
	public double excitement() {
		// Pings are routine (every few seconds), so they don't count
		var deaths = events(Kind.DEATH);
		double score = deaths.isEmpty() ? 0 : 30;
		if (!deaths.isEmpty() && deaths.getFirst().tick() < 25L * 60 * tickRateHz)
			score += 5;
		score += 8 * Math.min(3, events(Kind.HIT).size());
		var launches = events(Kind.LAUNCH);
		score += 2 * Math.min(8, launches.size());
		if (launches.stream().map(Event::other).distinct().count() > 1)
			score += 5; // both sides fired
		long nearMisses = events(Kind.CLOSEST).stream().filter(c -> c.value() < 80 && !hit(c.subject())).count();
		score += 4 * Math.min(3, nearMisses);
		score += 1.6 * Math.min(5, scenicWindows(0.6, 6L * tickRateHz).size());
		if (!launches.isEmpty() && launches.getFirst().tick() > 40L * 60 * tickRateHz)
			score -= 10;
		return Math.max(0, Math.min(100, score));
	}

	/**
	 * The share of the fighting (terminal approaches and detonations) within 800 m of an island's
	 * shore, 0..1: torpedoes running past a coastline make the best pictures.
	 */
	public double actionNearLand() {
		var action = events.stream().filter(e -> e.kind() == Kind.TERMINAL || e.kind() == Kind.DETONATION).toList();
		if (action.isEmpty())
			return 0;
		return action.stream().filter(e -> survey.sceneryAt(e.x(), e.y(), e.z()).shoreDistance() <= 800).count()
				/ (double) action.size();
	}

	/** Writes the timeline as readable text, for picking shots by hand or checking the plan. */
	public void write(Path file, String heading) throws IOException {
		try (var out = new PrintWriter(Files.newBufferedWriter(file))) {
			out.println("# " + heading);
			out.println("# " + survey);
			out.printf(Locale.ROOT, "# excitement %.0f, %d ticks (%s)%n", excitement(), lastTick, time(lastTick));
			for (var island : survey.islands())
				out.printf(Locale.ROOT, "island\tpeak %.0f m\t%.2f km2\tat %.0f, %.0f%n", island.peak(),
						island.areaKm2(), island.x(), island.y());
			out.println();
			out.println("# time\ttick\tevent\tsubject\tother\tvalue\tposition");
			for (var e : events)
				out.printf(Locale.ROOT, "%s\t%d\t%s\t%s\t%s\t%.1f\t%s%n", time(e.tick()), e.tick(), e.kind(),
						label(e.subject()), label(e.other()), e.value(),
						Double.isNaN(e.x()) ? "" : String.format(Locale.ROOT, "%.0f, %.0f, %.0f", e.x(), e.y(), e.z()));
			out.println();
			out.println("# scenic windows (sub, from, to, peak, score, feature bearing)");
			for (var w : scenicWindows(0.5, 4L * tickRateHz))
				out.printf(Locale.ROOT, "%s\t%s\t%s\t%s\t%.2f\t%.0f deg%n", label(w.subId()), time(w.startTick()),
						time(w.endTick()), time(w.peakTick()), w.peakScore(), Math.toDegrees(w.featureBearing()));
		}
	}

	private String label(int id) {
		if (id < 0)
			return "-";
		return id >= 1000 ? "torpedo " + id : name(id);
	}

	String time(long tick) {
		double seconds = tick / (double) tickRateHz;
		return String.format(Locale.ROOT, "%02d:%04.1f", (int) (seconds / 60), seconds % 60);
	}
}
