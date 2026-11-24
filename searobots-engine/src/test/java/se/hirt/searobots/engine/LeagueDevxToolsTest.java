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
package se.hirt.searobots.engine;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;
import se.hirt.searobots.engine.LeagueSparringRunner.Options;
import se.hirt.searobots.engine.LeagueSparringRunner.RecordMode;
import se.hirt.searobots.engine.replay.ReplayReader;
import se.hirt.searobots.api.MatchContext;
import dev.searobots.preflight.PreflightFixture;

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class LeagueDevxToolsTest {

	public static final class EarlyFailure implements SubmarineController {
		@Override
		public void onMatchStart(MatchContext context) {
			throw new IllegalStateException("immediate forfeit");
		}

		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
		}
	}

	public static final class LateFailure implements SubmarineController {
		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			if (input.tick() >= 10) {
				throw new IllegalStateException("failure during aftermath");
			}
		}
	}

	@Test
	void recordingDoesNotChangeScoreOrScoredTelemetry(@TempDir
	Path replayDir) throws Exception {
		var plain = LeagueSparringRunner.runAll(new Options(LateFailure.class.getName(), EarlyFailure.class.getName(),
				2, false, 0x2cL, 1, 2, true, RecordMode.NONE, replayDir, null)).getFirst();
		for (var mode : new RecordMode[] {RecordMode.ALL, RecordMode.LOSSES}) {
			var recorded = LeagueSparringRunner.runAll(new Options(LateFailure.class.getName(),
					EarlyFailure.class.getName(), 2, false, 0x2cL, 1, 2, true, mode, replayDir, null)).getFirst();
			assertEquals(plain.wins(), recorded.wins());
			assertEquals(0, recorded.forfeits());
			for (int i = 0; i < plain.games().size(); i++) {
				var expected = plain.games().get(i);
				var actual = recorded.games().get(i);
				assertEquals(expected.winnerSide(), actual.winnerSide());
				assertEquals(expected.reason(), actual.reason());
				assertEquals(expected.ticks(), actual.ticks());
				assertEquals(expected.hpSub(), actual.hpSub());
				assertEquals(expected.hpOpp(), actual.hpOpp());
				assertEquals(expected.torpedoesSub(), actual.torpedoesSub());
				assertNull(actual.forfeitSub());
				if (mode == RecordMode.ALL) {
					var reader = new ReplayReader(actual.savedReplay());
					var last = reader.readAll().getLast();
					assertEquals(actual.ticks() - 1 + LeagueSparringRunner.AFTERMATH_TICKS, last.tick());
					assertTrue(last.submarines().stream().allMatch(SubmarineSnapshot::forfeited),
							"Aftermath is still recorded even though its forfeits do not change the score");
				} else {
					assertNull(actual.savedReplay(), "A scored win must not be kept as a loss replay");
				}
			}
		}
	}

	@Test
	void preflightRunsFreshlyCompiledControllerAndHelpersInsteadOfClasspathCopies(@TempDir
	Path sourceDir) throws Exception {
		assertEquals("Old Compiled", new PreflightFixture().name());
		Path source = sourceDir.resolve("dev/searobots/preflight/PreflightFixture.java");
		Files.createDirectories(source.getParent());
		Files.writeString(source, """
				package dev.searobots.preflight;
				import se.hirt.searobots.api.*;
				public final class PreflightFixture implements SubmarineController {
					public String name() { return Identity.name(); }
					public String shortName() { return "FS"; }
					public void onTick(SubmarineInput in, SubmarineOutput out) {
						throw new IllegalStateException("fresh source smoke failure");
					}
					static final class Identity {
						static String name() { return "Fresh Source"; }
					}
				}
				""");
		var report = LeaguePreflight.check(sourceDir, PreflightFixture.class.getName(), true);
		assertEquals("Fresh Source", report.displayName());
		assertEquals("FS", report.shortName());
		assertFalse(report.passed());
		assertTrue(report.errors().stream().anyMatch(e -> e.contains("fresh source smoke failure")),
				"Smoke matches must execute the newly compiled onTick");
	}

	@Test
	void preflightRejectsEntrypointsAbsentFromSubmittedSources(@TempDir
	Path sourceDir) throws Exception {
		Files.writeString(sourceDir.resolve("Other.java"), "public class Other {}");
		var report = LeaguePreflight.check(sourceDir, PreflightFixture.class.getName(), false);
		assertFalse(report.passed());
		assertTrue(report.errors().stream().anyMatch(e -> e.contains("not compiled from the submitted sources")));
	}

	public static final class BoundaryRunner implements SubmarineController {
		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			// Head straight north at flank speed to cross the battle area boundary.
			output.setThrottle(1.0);
			output.setBallast(0.5);
			double heading = input.self().pose().heading();
			double err = Math.atan2(-Math.sin(heading), Math.cos(heading));
			output.setRudder(Math.clamp(err * 2.0, -1.0, 1.0));
			double zErr = -150.0 - input.self().pose().position().z();
			output.setSternPlanes(Math.clamp(zErr / 80.0, -1.0, 1.0));
		}
	}

	@Test
	void boundaryForfeitRecordsReasonAndCountsInSparringSummary(@TempDir
	Path replayDir) throws Exception {
		var summary = LeagueSparringRunner.runAll(new Options(BoundaryRunner.class.getName(),
				DummyTestSub.class.getName(), 1, false, 0x2cL, 30, 1, true, RecordMode.NONE, replayDir, null))
				.getFirst();
		assertEquals(1, summary.losses());
		assertEquals(1, summary.forfeits(), "Boundary forfeit must be counted in summary forfeits");
		var game = summary.games().getFirst();
		assertEquals("FORFEIT", game.reason());
		assertEquals("Left battle area", game.forfeitSub());
	}

	public static final class DummyTestSub implements SubmarineController {
		@Override
		public String name() {
			return "Dummy Test";
		}

		@Override
		public String shortName() {
			return "DT";
		}

		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			output.setThrottle(0.3);
			output.setBallast(0.5);
		}
	}

	@Test
	void controllerLoaderResolvesBuiltInAliasesAndExternalFqcn() {
		assertEquals("Claude", ControllerLoader.resolve("claude").displayName());
		assertEquals("Codex", ControllerLoader.resolve("Codex").displayName());
		assertEquals("Gemini", ControllerLoader.resolve("gemini").displayName());
		assertEquals("Default", ControllerLoader.resolve("default").displayName());
		assertEquals(4, ControllerLoader.allCombatBuiltIns().size());

		var custom = ControllerLoader.resolve(DummyTestSub.class.getName());
		assertEquals("Dummy Test", custom.displayName());
		assertEquals("DT", custom.shortName());
		assertInstanceOf(DummyTestSub.class, custom.factory().get());

		assertThrows(IllegalArgumentException.class, () -> ControllerLoader.resolve("com.example.DoesNotExist"));
	}

	@Test
	void sparringRunnerAlternatesSpawnsAndRecordsReplays(@TempDir
	Path replayDir) throws Exception {
		var opts = new Options(DummyTestSub.class.getName(), "default", 2, false, 0x2cL, 2, 2, true, RecordMode.ALL,
				replayDir, null);
		var summaries = LeagueSparringRunner.runAll(opts);
		assertEquals(1, summaries.size());

		var s = summaries.getFirst();
		assertEquals(2, s.games().size());
		assertFalse(s.games().get(0).swappedSpawns(), "Game 1 should place challenger in Slot 0");
		assertTrue(s.games().get(1).swappedSpawns(), "Game 2 should swap challenger to Slot 1");
		for (var g : s.games()) {
			assertNotNull(g.savedReplay());
			assertTrue(Files.isRegularFile(g.savedReplay()), "Expected replay file " + g.savedReplay());
		}
	}

	@Test
	void preflightAcceptsCleanSubmarineAndRejectsForbiddenApis(@TempDir
	Path tempDir) throws Exception {
		Path cleanSrc = tempDir.resolve("clean/com/example/CleanDevSub.java");
		Files.createDirectories(cleanSrc.getParent());
		Files.writeString(cleanSrc, """
				package com.example;
				import se.hirt.searobots.api.*;
				public final class CleanDevSub implements SubmarineController {
					@Override public String name() { return "Clean Dev"; }
					@Override public String shortName() { return "CD"; }
					@Override public void onTick(SubmarineInput in, SubmarineOutput out) {
						out.setThrottle(0.4);
						out.setBallast(0.5);
						out.setStatus("PATROL");
					}
				}
				""");
		var cleanReport = LeaguePreflight.check(tempDir.resolve("clean"), "com.example.CleanDevSub", false);
		assertTrue(cleanReport.passed(), "Expected clean sub to pass preflight: " + cleanReport.errors());
		assertEquals("Clean Dev", cleanReport.displayName());
		assertEquals("CD", cleanReport.shortName());

		// Hostile API reference must fail bytecode scan
		Path badSrc = tempDir.resolve("bad/com/example/BadDevSub.java");
		Files.createDirectories(badSrc.getParent());
		Files.writeString(badSrc, """
				package com.example;
				import se.hirt.searobots.api.*;
				public final class BadDevSub implements SubmarineController {
					@Override public void onTick(SubmarineInput in, SubmarineOutput out) {
						System.out.println("forbidden stdout");
					}
				}
				""");
		var badReport = LeaguePreflight.check(tempDir.resolve("bad"), "com.example.BadDevSub", false);
		assertFalse(badReport.passed(), "Expected System.out usage to fail preflight");

		// GitHub remote normalization helper
		assertEquals("https://github.com/octocat/my-sub",
				LeaguePreflight.normalizeGithubHttps("git@github.com:octocat/my-sub.git"));
		assertEquals("octocat", LeaguePreflight.extractGithubUser("https://github.com/octocat/my-sub"));
	}
}
