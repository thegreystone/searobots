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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.LocalTime;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import se.hirt.searobots.viewer.showreel.MatchIndex.Kind;
import se.hirt.searobots.viewer.showreel.ShowreelPlan.Beat;
import se.hirt.searobots.viewer.showreel.ShowreelPlan.Card;
import se.hirt.searobots.viewer.showreel.ShowreelPlan.Entry;
import se.hirt.searobots.viewer.showreel.ShowreelPlan.Shot;

class ShowreelPlanTest {
	private static TerrainSurvey survey;
	private static MatchIndex index;

	@BeforeAll
	static void play() {
		survey = new TerrainSurvey(SyntheticMatch.terrain());
		index = new MatchIndex(survey, SyntheticMatch.RATE);
		SyntheticMatch.play(index);
	}

	@Test
	void surveyFindsTheIslandAndRatesTheRidgeAboveOpenWater() {
		assertEquals(1, survey.islands().size());
		assertEquals(150, survey.islands().getFirst().peak(), 1);
		double ridge = survey.sceneryAt(0, 150, -150).score();
		double open = survey.sceneryAt(-5000, -5000, -150).score();
		assertTrue(ridge > open + 0.2, "The ridge (" + ridge + ") should rate well above open water (" + open + ")");
	}

	@Test
	void indexRecordsTheStrikeInOrder() {
		var launch = index.events(Kind.LAUNCH);
		assertEquals(1, launch.size());
		assertEquals(SyntheticMatch.LAUNCH, launch.getFirst().tick());
		assertEquals(0, launch.getFirst().other(), "The launcher");
		var terminal = index.events(Kind.TERMINAL).getFirst();
		var blast = index.events(Kind.DETONATION).getFirst();
		var death = index.events(Kind.DEATH).getFirst();
		assertTrue(terminal.tick() > SyntheticMatch.LAUNCH && terminal.tick() < SyntheticMatch.BLAST);
		assertEquals(SyntheticMatch.BLAST, blast.tick());
		assertEquals(1, blast.other(), "The nearest submarine is the target");
		assertEquals(SyntheticMatch.BLAST, death.tick());
		assertEquals(1, death.subject());
		assertTrue(index.hit(SyntheticMatch.TORPEDO));
		assertEquals(SyntheticMatch.END, index.endTick());
		assertFalse(index.events(Kind.PING).isEmpty());
	}

	@Test
	void scenicWindowsFollowTheRidge() {
		var windows = index.scenicWindows(0.5, 4L * SyntheticMatch.RATE);
		assertFalse(windows.isEmpty(), "Cruising along the ridge should be scenic");
		assertEquals(0, windows.getFirst().subId(), "Claude cruises along the ridge; Codex waits in open water");
	}

	@Test
	void speedRampsIntegrateToTheirTicks() {
		var beat = new Beat("ramp", Shot.TACTICAL, -1, -1, 4, 1000, 30, 5, Entry.CUT, Card.NONE, null, true, false,
				null, null);
		assertEquals(30 * 4 - (30 - 5) * 4 / 2.0, beat.simSecondsAt(4), 1e-9);
		assertEquals(1000 + Math.round(70.0 * 50), beat.toTick(50));
		assertEquals(17.5, beat.speedAt(2), 1e-9);
		assertEquals(0, beat.simSecondsAt(-1), 1e-9);
	}

	@Test
	void planTellsTheStoryInOrder() {
		var plan = ShowreelPlan.create(index, 42, 85, LocalTime.of(15, 0));
		List<Beat> beats = plan.beats();
		assertEquals(Shot.AERIAL, beats.getFirst().shot());
		assertEquals(Card.TITLE, beats.getFirst().card());
		assertEquals(Entry.FADE_IN, beats.getFirst().entry());
		assertEquals(Shot.OUTRO, beats.getLast().shot());
		assertEquals(Card.END, beats.getLast().card());
		assertTrue(plan.length() > 40 && plan.length() < 90, "Length " + plan.length());

		// Simulation time never runs backwards
		for (int i = 1; i < beats.size(); i++)
			assertTrue(beats.get(i).fromTick() >= beats.get(i - 1).toTick(plan.tickRateHz()) - 1,
					"Beat " + beats.get(i).label() + " starts before " + beats.get(i - 1).label() + " ends");

		Beat launch = find(beats, Shot.LAUNCH);
		assertTrue(launch.fromTick() <= SyntheticMatch.LAUNCH, "The launch shot opens before the launch");
		assertNotNull(launch.caption());
		Beat impact = find(beats, Shot.IMPACT);
		assertTrue(impact.endSpeed() < 1, "The approach slows into the hit");
		Beat blast = find(beats, Shot.DETONATION);
		assertEquals(SyntheticMatch.BLAST, blast.fromTick(), "The blast beat starts on the detonation");
		assertEquals(Entry.FLASH, blast.entry());
		Beat sinking = find(beats, Shot.SINKING);
		assertEquals(1, sinking.subject());
		assertTrue(sinking.caption().startsWith("CODEX SUB DESTROYED"), sinking.caption());
		assertTrue(beats.stream().anyMatch(b -> b.shot() == Shot.TACTICAL && b.startSpeed() > 4),
				"The hunt is shown in time-lapse");
	}

	@Test
	void openingFlightHeadsForTheIsland() {
		var aerial = ShowreelPlan.create(index, 42, 85, LocalTime.of(9, 0)).beats().getFirst();
		assertEquals(SyntheticMatch.ISLAND_X, aerial.backdrop()[0], 1);
		assertEquals(SyntheticMatch.ISLAND_Y, aerial.backdrop()[1], 1);
	}

	private static Beat find(List<Beat> beats, Shot shot) {
		return beats.stream().filter(b -> b.shot() == shot).findFirst()
				.orElseThrow(() -> new AssertionError("No " + shot + " beat"));
	}
}
