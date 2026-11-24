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

import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.*;

class LeagueDevxToolsTest {

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
