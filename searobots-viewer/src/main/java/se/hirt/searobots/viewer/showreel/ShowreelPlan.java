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

import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

import se.hirt.searobots.viewer.showreel.MatchIndex.Event;
import se.hirt.searobots.viewer.showreel.MatchIndex.Kind;
import se.hirt.searobots.viewer.showreel.MatchIndex.ScenicWindow;

/**
 * The edit of a showreel: a sequence of beats, each a camera shot over a stretch of the match.
 * Simulation time only moves forward, but at a speed that varies per beat: time-lapse while the
 * submarines hunt, real time for launches, slow motion into a detonation. A hard cut may skip
 * ahead. Built from a {@link MatchIndex}, so every beat is placed where the action will be.
 * <p>
 * Positions in beats are simulation coordinates (x east, y north, z up).
 */
public final class ShowreelPlan {

	/** The camera work of a beat. */
	public enum Shot {
		/**
		 * Low flight over the water towards an island, ending in a dive (focus, backdrop island).
		 */
		AERIAL,
		/** Bow-quarter pass along a submarine. */
		HERO,
		/** A submarine sliding past a static camera. */
		FLY_BY,
		/** A submarine in front of a cliff or ridge (backdrop: the feature). */
		SCENIC,
		/** North-up overview of the whole fight, seabed visible, with tracks. */
		TACTICAL,
		/** Well off a submarine as it pings, the pulse swelling out from the hull. */
		SONAR,
		/**
		 * The launcher's bow as its tube doors open and the torpedo leaves; as the weapon glides
		 * past, the camera takes up its speed and swings round it.
		 */
		LAUNCH,
		/** Riding behind a torpedo, looking towards its target. */
		TORPEDO_CHASE,
		/** Torpedo and target framed together as they close. */
		TERMINAL,
		/** Over a torpedo's shoulder, looking at the target it is about to hit. */
		IMPACT,
		/** Around a detonation (focus: where it goes off). */
		DETONATION,
		/** Around a dying submarine as it sinks. */
		SINKING,
		/** Rising from the water towards an island (focus, backdrop island). */
		OUTRO
	}

	/** How a beat begins. */
	public enum Entry {
		/** Hard cut. */
		CUT,
		/** The camera glides from the previous shot. */
		SMOOTH,
		/** The camera dives from the air into the water. */
		DESCENT,
		/** Hard cut, fading in from black. */
		FADE_IN,
		/** Hard cut on a white flash. */
		FLASH,
		/** No cut: the previous beat's shot carries on, at this beat's speed. */
		CONTINUE
	}

	/** A full-screen title shown over a beat. */
	public enum Card {
		NONE, TITLE, END
	}

	/**
	 * One beat. It lasts {@code seconds} of video, starting at {@code fromTick}, with the
	 * simulation running at {@code startSpeed} times real time, changing linearly to
	 * {@code endSpeed}. {@code caption} is "TITLE|subtitle" or null.
	 */
	public record Beat(String label, Shot shot, int subject, int secondary, double seconds, long fromTick,
			double startSpeed, double endSpeed, Entry entry, Card card, String caption, boolean trails, boolean pings,
			double[] focus, double[] backdrop) {

		/** Simulation seconds elapsed {@code t} video seconds into the beat. */
		public double simSecondsAt(double t) {
			t = Math.max(0, Math.min(seconds, t));
			return startSpeed * t + (endSpeed - startSpeed) * t * t / (2 * seconds);
		}

		/** Playback speed {@code t} video seconds into the beat. */
		public double speedAt(double t) {
			return startSpeed + (endSpeed - startSpeed) * Math.max(0, Math.min(1, t / seconds));
		}

		public long toTick(int tickRateHz) {
			return fromTick + Math.round(simSecondsAt(seconds) * tickRateHz);
		}

		Beat withEntry(Entry e) {
			return new Beat(label, shot, subject, secondary, seconds, fromTick, startSpeed, endSpeed, e, card, caption,
					trails, pings, focus, backdrop);
		}
	}

	private final List<Beat> beats;
	private final int tickRateHz;
	private final long seed;

	ShowreelPlan(List<Beat> beats, int tickRateHz, long seed) {
		this.beats = List.copyOf(beats);
		this.tickRateHz = tickRateHz;
		this.seed = seed;
	}

	public List<Beat> beats() {
		return beats;
	}

	public int tickRateHz() {
		return tickRateHz;
	}

	public long seed() {
		return seed;
	}

	/** Length of the film in seconds. */
	public double length() {
		return beats.stream().mapToDouble(Beat::seconds).sum();
	}

	/** Seconds over which the film fades to black at the end. */
	public double fadeOutSeconds() {
		return 1.6;
	}

	public String describe() {
		var sb = new StringBuilder();
		sb.append(String.format(Locale.ROOT, "Showreel for seed %s: %d beats, %.1f s%n", Long.toHexString(seed),
				beats.size(), length()));
		double at = 0;
		for (var b : beats) {
			sb.append(String.format(Locale.ROOT, "  %5.1fs  %-14s %-13s %5.1fs  sim %s-%s  speed %5.2f-%-6.2f %s%s%n",
					at, b.label(), b.shot(), b.seconds(), clock(b.fromTick()), clock(b.toTick(tickRateHz)),
					b.startSpeed(), b.endSpeed(), b.entry(), b.caption() == null ? "" : "  \"" + b.caption() + "\""));
			at += b.seconds();
		}
		return sb.toString();
	}

	private String clock(long tick) {
		double s = tick / (double) tickRateHz;
		return String.format(Locale.ROOT, "%02d:%04.1f", (int) (s / 60), s % 60);
	}

	// ── Planning ──

	/** Fastest time-lapse: beyond this, a beat skips ahead rather than racing. */
	private static final double MAX_SPEED = 160;
	/** The run into a blast: video seconds, and the speed it slows to from real time. */
	private static final double IMPACT_KILL = 4, IMPACT_KILL_END = 0.25, IMPACT_HIT = 3.5, IMPACT_HIT_END = 0.3;

	/**
	 * Plans a film of about {@code targetSeconds} from a match: an aerial title over the islands,
	 * the two submarines introduced where the scenery is best, the hunt in time-lapse, then the
	 * launches, hits and kill as they happen, and an outro.
	 */
	public static ShowreelPlan create(MatchIndex index, long seed, double targetSeconds, LocalTime timeOfDay) {
		// Optional beats go one at a time when the film runs long: the scenic pass first, then the
		// counter-fire
		Plan plan = null;
		for (boolean[] optional : new boolean[][] {{true, true}, {false, true}, {true, false}, {false, false}}) {
			plan = new Planner(index, timeOfDay).plan(optional[0], optional[1]);
			if (plan.length() <= targetSeconds + 3)
				break;
		}
		return new ShowreelPlan(plan.beats, index.tickRateHz(), seed);
	}

	private static final class Planner {
		final MatchIndex index;
		final int rate;
		final List<Beat> beats = new ArrayList<>();
		final double sunEast, sunNorth; // horizontal direction towards the sun
		long cursor; // simulation tick where the next beat starts, at the earliest

		Planner(MatchIndex index, LocalTime timeOfDay) {
			this.index = index;
			this.rate = index.tickRateHz();
			// As the viewer lights it: the sun rises in the east at 6:00 and sets in the west at 18:00
			double hour = timeOfDay.toSecondOfDay() / 3600.0;
			double arc = Math.max(0, Math.min(1, (hour - 6) / 12)) * Math.PI;
			double east = Math.cos(arc), north = 0.15, length = Math.hypot(east, north);
			sunEast = east / length;
			sunNorth = north / length;
		}

		double length() {
			return beats.stream().mapToDouble(Beat::seconds).sum();
		}

		long ticks(double seconds) {
			return Math.round(seconds * rate);
		}

		void add(Beat beat) {
			if (beat.fromTick() < cursor)
				throw new IllegalStateException("Beat " + beat.label() + " starts before the previous one ends");
			beats.add(beat);
			cursor = beat.toTick(rate);
		}

		/** A beat at constant speed. */
		Beat beat(
			String label, Shot shot, int subject, int secondary, double seconds, long from, double speed, Entry entry,
			String caption) {
			return new Beat(label, shot, subject, secondary, seconds, Math.max(from, cursor), speed, speed, entry,
					Card.NONE, caption, false, false, null, null);
		}

		/**
		 * Time-lapse in the tactical view from the cursor to {@code until}, taking about
		 * {@code seconds}, slowing to a sixth of its speed at the end if {@code rampDown}.
		 */
		void timelapse(String label, long until, double seconds, boolean rampDown, String caption) {
			long span = until - cursor;
			if (span < ticks(2.5))
				return;
			double k = rampDown ? 1 / 6.0 : 1;
			double start = 2 * span / (double) rate / (seconds * (1 + k));
			long from = cursor;
			if (start > MAX_SPEED) {
				start = MAX_SPEED;
				from = until - Math.round(start * (1 + k) / 2 * seconds * rate); // jump the rest
			}
			add(new Beat(label, Shot.TACTICAL, -1, -1, seconds, from, start, start * k, Entry.CUT, Card.NONE, caption,
					true, start * k <= 2, null, null));
		}

		Plan plan(boolean scenicPass, boolean counterFire) {
			var subs = index.submarineIds();
			int first = subs.isEmpty() ? 0 : subs.getFirst();
			var launches = index.events(Kind.LAUNCH);
			var hits = index.events(Kind.DETONATION).stream().filter(d -> index.hit(d.subject())).toList();
			var deaths = index.events(Kind.DEATH);
			long firstLaunch = launches.isEmpty() ? index.endTick() - ticks(30) : launches.getFirst().tick();

			// Climax: the detonation that kills, else the last hit, else the nearest miss
			Event kill = null, climax = null;
			if (!deaths.isEmpty()) {
				var death = deaths.getFirst();
				kill = death;
				climax = blowAt(hits, death);
			}
			if (climax == null && !hits.isEmpty())
				climax = hits.getLast();
			if (climax == null)
				climax = index.events(Kind.CLOSEST).stream().min(Comparator.comparingDouble(Event::value))
						.map(c -> index.events(Kind.DETONATION).stream().filter(d -> d.subject() == c.subject())
								.findFirst().orElse(null))
						.orElse(null);
			long climaxTick = climax != null ? climax.tick() : index.endTick() - ticks(10);
			final long climaxAt = climaxTick;
			Event firstHit = hits.stream().filter(h -> h.tick() < climaxAt - ticks(14)).findFirst().orElse(null);

			// ── Introductions: each submarine where its scenery is best, before the shooting starts ──
			var windows = index.scenicWindows(0.55, ticks(4));
			long introCutoff = firstLaunch - ticks(50);
			ScenicWindow winA = null, winB = null;
			for (var w : windows) { // best first
				if (w.peakTick() > introCutoff || w.peakTick() < ticks(12))
					continue;
				if (winA == null)
					winA = w;
				else if (winB == null && w.subId() != winA.subId())
					winB = w;
			}
			if (winA != null && winB != null && winB.peakTick() < winA.peakTick()) {
				var t = winA;
				winA = winB;
				winB = t;
			}
			int subA = winA != null ? winA.subId() : first, subB = -1;
			for (int id : subs)
				if (id != subA && subB < 0)
					subB = id;
			long introA = winA != null ? Math.max(ticks(12), winA.peakTick() - ticks(2)) : ticks(15);

			// Opening: aerial towards the island nearest the first submarine, title over it; the
			// simulation runs on unseen until the dive reaches the submarine
			var sampleA = index.sampleAt(subA, introA);
			double[] focusA = sampleA != null ? new double[] {sampleA.x(), sampleA.y(), 0} : new double[] {0, 0, 0};
			double aerialSeconds = 8.5;
			beats.add(new Beat("opening", Shot.AERIAL, subA, -1, aerialSeconds, 0,
					introA / (double) rate / aerialSeconds, introA / (double) rate / aerialSeconds, Entry.FADE_IN,
					Card.TITLE, null, false, false, focusA, island(focusA)));
			cursor = beats.getLast().toTick(rate);
			add(beat("intro " + index.name(subA), Shot.HERO, subA, -1, 5, cursor, 1, Entry.DESCENT,
					introCaption(subA)));

			if (subB >= 0) {
				long introB = cursor + ticks(1);
				Shot shotB = Shot.HERO;
				double[] featureB = null;
				if (winB != null && winB.peakTick() - ticks(2) > cursor) {
					introB = winB.peakTick() - ticks(2);
					shotB = Shot.SCENIC;
					featureB = feature(subB, introB, winB.featureBearing());
				}
				var b = beat("intro " + index.name(subB), shotB, subB, -1, 4.5, introB, 1, Entry.CUT,
						introCaption(subB));
				add(new Beat(b.label(), b.shot(), b.subject(), b.secondary(), b.seconds(), b.fromTick(), 1, 1,
						b.entry(), b.card(), b.caption(), false, false, null, featureB));
			}

			// ── The hunt: time-lapse, with a scenic pass and a sonar ping in real time ──
			// A launch shot opens just before the tube door does; the weapon clears the bow about three
			// seconds later (door, then sliding out of the tube) and its motor starts
			long launchShot = firstLaunch - ticks(0.4);
			String huntCaption = "THE HUNT|passive sonar · target motion analysis";
			ScenicWindow mid = null;
			if (scenicPass)
				for (var w : windows)
					if (w.startTick() > cursor + ticks(25) && w.endTick() < launchShot - ticks(25)
							&& w.peakScore() >= 0.58) {
						mid = w;
						break;
					}
			// A ping: the time-lapse slows as one goes out, and the pulse is followed in slow
			// motion
			final long pingFrom = cursor + ticks(40), pingTo = launchShot - ticks(70);
			final ScenicWindow scenic = mid;
			var ping = index.events(Kind.PING).stream()
					.filter(p -> p.tick() >= pingFrom && p.tick() <= pingTo && (scenic == null
							|| p.tick() < scenic.startTick() - ticks(20) || p.tick() > scenic.endTick() + ticks(20)))
					.findFirst().orElse(null);
			if (ping != null && (mid == null || ping.tick() < mid.startTick())) {
				sonar(ping, huntCaption);
				huntCaption = null;
			}
			if (mid != null) {
				long scenicStart = Math.max(mid.startTick(), mid.peakTick() - ticks(2));
				timelapse("hunt", scenicStart, clampSeconds(scenicStart - cursor, 45, 3, 6), false, huntCaption);
				add(new Beat("scenic " + index.name(mid.subId()), Shot.SCENIC, mid.subId(), -1, 4.5, scenicStart, 1, 1,
						Entry.CUT, Card.NONE, null, false, false, null,
						feature(mid.subId(), scenicStart, mid.featureBearing())));
				huntCaption = null;
				if (ping != null && ping.tick() > mid.endTick()) {
					sonar(ping, null);
				}
			}
			timelapse("closing in", launchShot, clampSeconds(launchShot - cursor, 40, 2.5, 6), true, huntCaption);

			if (launches.isEmpty()) {
				outro(surfaceAbove(subA, cursor));
				return new Plan(beats);
			}

			// ── First salvo ──
			var opening = launches.getFirst();
			int shooter = opening.other();
			long salvo = launches.stream().filter(l -> l.other() == shooter && l.tick() - opening.tick() < ticks(15))
					.count();
			add(beat("launch", Shot.LAUNCH, shooter, opening.subject(), 5.2, launchShot, 1, Entry.CUT,
					"TORPEDOES AWAY|" + index.name(shooter) + " fires " + count(salvo)));
			// The same shot carries on, slowed, as the camera picks up the weapon gliding past and
			// swings round it
			add(beat("torpedo", Shot.LAUNCH, shooter, opening.subject(), 5, cursor, 0.5, Entry.CONTINUE, null));

			// ── Counter-fire ──
			long nextAction = firstHit != null ? firstHit.tick() : climaxTick;
			if (counterFire) {
				var counter = launches.stream().filter(l -> l.other() != shooter && l.tick() > opening.tick() + ticks(8)
						&& l.tick() < nextAction - ticks(12)).findFirst();
				if (counter.isPresent()) {
					var c = counter.get();
					long at = c.tick() - ticks(0.4);
					timelapse("torpedoes running", at, clampSeconds(at - cursor, 25, 2, 4), true, null);
					if (at >= cursor)
						add(beat("counter-fire", Shot.LAUNCH, c.other(), c.subject(), 4.6, at, 1, Entry.CUT,
								index.name(c.other()).toUpperCase(Locale.ROOT) + " RETURNS FIRE|"));
				}
			}

			// ── First hit, short of the kill ──
			if (firstHit != null)
				strike(firstHit, false, null, Long.MAX_VALUE);

			// ── Climax ──
			if (climax != null) {
				String caption = null;
				boolean killing = kill != null && climax.other() == kill.subject();
				if (killing) {
					int victim = kill.subject();
					int winner = subs.stream().filter(id -> id != victim).findFirst().orElse(-1);
					boolean mutual = deaths.size() > 1;
					caption = index.name(kill.subject()).toUpperCase(Locale.ROOT) + " DESTROYED|"
							+ (winner >= 0 && !mutual ? index.name(winner) + " wins" : "");
				}
				// When both die, the second kill follows the first: with its run-in if there is time
				// (the first sinking cut short to make room), else its blast, else its wreck
				Event second = killing && deaths.size() > 1 && deaths.get(1).tick() - kill.tick() < ticks(150)
						? deaths.get(1) : null;
				Event finalBlow = second != null ? blowAt(hits, second) : null;
				long holdUntil = Long.MAX_VALUE;
				if (finalBlow != null) {
					long firstBlastEnd = blastBeat(climax, true, null).toTick(rate);
					long run = impactFrom(finalBlow.tick(), true);
					holdUntil = run >= firstBlastEnd ? run : Math.max(firstBlastEnd, finalBlow.tick());
				}
				strike(climax, killing, caption, holdUntil);
				if (second != null) {
					String last = index.name(second.subject()).toUpperCase(Locale.ROOT) + " DESTROYED|no survivors";
					if (finalBlow != null && impactFrom(finalBlow.tick(), true) >= cursor) {
						strike(finalBlow, true, last, Long.MAX_VALUE);
					} else {
						if (finalBlow != null && finalBlow.tick() >= cursor)
							add(blastBeat(finalBlow, true, null));
						sinking(second.subject(), last, 5);
					}
				}
			}
			var lastBlast = beats.reversed().stream().filter(b -> b.shot() == Shot.DETONATION).findFirst();
			outro(lastBlast.map(b -> new double[] {b.focus()[0], b.focus()[1], 0})
					.orElseGet(() -> surfaceAbove(shooter, cursor)));
			return new Plan(beats);
		}

		double[] surfaceAbove(int sub, long tick) {
			var s = index.sampleAt(sub, tick);
			return s != null ? new double[] {s.x(), s.y(), 0} : new double[] {0, 0, 0};
		}

		/**
		 * The time-lapse slowing towards a ping, then the pulse leaving the submarine in slow
		 * motion: sound crosses a kilometre a second here, too fast to follow at real time.
		 */
		void sonar(Event ping, String huntCaption) {
			long at = ping.tick() - ticks(0.05);
			timelapse("hunt", at, clampSeconds(at - cursor, 40, 3.5, 6), true, huntCaption);
			if (at < cursor)
				return;
			add(new Beat("sonar", Shot.SONAR, ping.subject(), -1, 4.5, at, 0.1, 0.1, Entry.CUT, Card.NONE,
					"ACTIVE SONAR|a ping reaches out for the enemy", false, true, null, null));
		}

		/**
		 * A torpedo strike: time-lapse up to it, the run in, the approach slowing to slow motion,
		 * the blast on a flash, and, if it kills, the sinking, ending by {@code holdUntil}.
		 */
		void strike(Event detonation, boolean kill, String sinkCaption, long holdUntil) {
			int torpedo = detonation.subject(), target = detonation.other();
			long blast = detonation.tick();
			// Over the weapon's shoulder into slow motion, landing on the detonation tick
			double impactSeconds = kill ? IMPACT_KILL : IMPACT_HIT, impactEnd = kill ? IMPACT_KILL_END : IMPACT_HIT_END;
			long impactFrom = impactFrom(blast, kill);
			double leadSeconds = kill ? 3.5 : 2.5;
			long leadFrom = impactFrom - ticks(leadSeconds);
			boolean lead = leadFrom > cursor + ticks(1);
			long approach = lead ? leadFrom : impactFrom;
			timelapse(kill ? "final approach" : "torpedoes running", approach,
					clampSeconds(approach - cursor, 30, 2.5, 5), true, null);
			if (impactFrom < cursor)
				return; // already past it
			if (lead && leadFrom >= cursor)
				add(beat(kill ? "chase" : "closing", kill ? Shot.TORPEDO_CHASE : Shot.TERMINAL, torpedo, target,
						leadSeconds, leadFrom, 1, Entry.CUT, null));
			add(new Beat("impact", Shot.IMPACT, torpedo, target, impactSeconds, impactFrom, 1, impactEnd, Entry.CUT,
					Card.NONE, null, false, false, null, null));
			// Land exactly on the detonation tick
			cursor = Math.min(cursor, blast);
			if (kill) {
				var boom = blastBeat(detonation, true, null);
				double sink = Math.min(5, (holdUntil - boom.toTick(rate)) / (double) rate);
				if (sink < 1.5) {
					add(blastBeat(detonation, true, sinkCaption)); // no time to watch it sink
				} else {
					add(boom);
					sinking(target, sinkCaption, sink);
				}
				return;
			}
			// Count every hit landing while the blast is on screen (salvoes often arrive in pairs)
			long blastEnd = blast + ticks(2.5);
			long hitsInBeat = index.events(Kind.HIT).stream()
					.filter(h -> h.subject() == target && h.tick() >= blast && h.tick() <= blastEnd).count();
			String name = index.name(target);
			add(blastBeat(detonation, false,
					hitsInBeat == 0 ? "NEAR MISS|" + name + " survives"
							: (hitsInBeat > 1 ? "DOUBLE HIT|" : "DIRECT HIT|") + name + " hull at "
									+ hullAfter(target, blastEnd) + "%"));
		}

		/** Where the over-the-shoulder run into a blast at {@code blast} starts. */
		long impactFrom(long blast, boolean kill) {
			double seconds = kill ? IMPACT_KILL : IMPACT_HIT, end = kill ? IMPACT_KILL_END : IMPACT_HIT_END;
			return blast - Math.round(seconds * (1 + end) / 2 * rate);
		}

		/** The blast, on a flash, in slow motion. */
		Beat blastBeat(Event detonation, boolean kill, String caption) {
			return new Beat(kill ? "kill" : "hit", Shot.DETONATION, detonation.subject(), detonation.other(),
					kill ? 3.2 : 3.5, detonation.tick(), kill ? 0.2 : 0.25, kill ? 0.8 : 1.0, Entry.FLASH, Card.NONE,
					caption, false, false, new double[] {detonation.x(), detonation.y(), detonation.z()}, null);
		}

		void sinking(int target, String caption, double seconds) {
			add(beat("sinking", Shot.SINKING, target, -1, seconds, cursor, 1, Entry.CUT, caption));
		}

		/**
		 * The damaging detonation on {@code death}'s submarine nearest the moment it died, or null.
		 */
		Event blowAt(List<Event> hits, Event death) {
			return hits.stream().filter(h -> h.other() == death.subject() && Math.abs(h.tick() - death.tick()) <= rate)
					.min(Comparator.comparingLong(h -> Math.abs(h.tick() - death.tick()))).orElse(null);
		}

		void outro(double[] focus) {
			add(new Beat("outro", Shot.OUTRO, -1, -1, 6.5, cursor, 1, 1, Entry.CUT, Card.END, null, false, false, focus,
					island(focus)));
		}

		/** The island to frame behind {@code focus}: big and tall, preferably near. */
		double[] island(double[] focus) {
			var islands = index.survey().islands();
			if (islands.isEmpty())
				return new double[] {focus[0] + 2000, focus[1], 0};
			var best = islands.stream().max(Comparator.comparingDouble(i -> {
				double dx = i.x() - focus[0], dy = i.y() - focus[1], d = Math.max(1, Math.hypot(dx, dy));
				// Looking towards the island should not mean looking into the sun and its glitter
				double glare = Math.max(0, (dx * sunEast + dy * sunNorth) / d);
				double clear = Math.max(0.05, 1 - 1.6 * Math.max(0, glare - 0.3));
				return i.peak() * Math.pow(Math.max(0.05, i.areaKm2()), 0.3) / (1 + d / 1500) * clear;
			})).orElseThrow();
			return new double[] {best.x(), best.y(), best.peak()};
		}

		/** A point on the terrain feature beside a submarine, from its position at {@code tick}. */
		double[] feature(int sub, long tick, double bearing) {
			var s = index.sampleAt(sub, tick);
			if (s == null)
				return null;
			return new double[] {s.x() + 300 * Math.sin(bearing), s.y() + 300 * Math.cos(bearing), s.z()};
		}

		int hullAfter(int sub, long tick) {
			int hp = 1000;
			for (var h : index.events(Kind.HIT))
				if (h.subject() == sub && h.tick() <= tick)
					hp -= (int) h.value();
			return Math.max(0, hp / 10);
		}

		String introCaption(int sub) {
			String name = index.name(sub);
			String lower = name.toLowerCase(Locale.ROOT);
			String what = lower.startsWith("claude") ? "Claude-authored attack submarine"
					: lower.startsWith("codex") ? "Codex-authored attack submarine" : "autonomous submarine";
			return name.toUpperCase(Locale.ROOT) + "|" + what;
		}

		double clampSeconds(long span, double speed, double min, double max) {
			return Math.max(min, Math.min(max, span / (double) rate / speed));
		}

		static String count(long n) {
			return n == 1 ? "a torpedo" : n == 2 ? "a pair" : "a salvo of " + n;
		}
	}

	private record Plan(List<Beat> beats) {
		double length() {
			return beats.stream().mapToDouble(Beat::seconds).sum();
		}
	}
}
