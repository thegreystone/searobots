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

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;

import java.nio.file.Files;
import java.nio.file.Path;
import javax.tools.ToolProvider;

import static org.junit.jupiter.api.Assertions.*;

class ViewerLaunchArgsTest {

	@Test
	void extraClasspathFlagDiscoversServiceProvidersWithoutExplicitShipSelectors(@TempDir
	Path tempDir) throws Exception {
		String controllerClass = "com.example.external.ServiceViewerSub";
		Path source = tempDir.resolve("ServiceViewerSub.java");
		Files.writeString(source, """
				package com.example.external;
				import se.hirt.searobots.api.*;
				public final class ServiceViewerSub implements SubmarineController {
					public String name() { return "Service Viewer"; }
					public void onTick(SubmarineInput in, SubmarineOutput out) {}
				}
				""");
		Path classes = tempDir.resolve("classes");
		Files.createDirectories(classes);
		Path api = Path.of(SubmarineController.class.getProtectionDomain().getCodeSource().getLocation().toURI());
		assertEquals(0, ToolProvider.getSystemJavaCompiler().run(null, null, null, "--release", "25", "-proc:none",
				"-cp", api.toString(), "-d", classes.toString(), source.toString()));
		Path service = classes.resolve("META-INF/services/" + SubmarineController.class.getName());
		Files.createDirectories(service.getParent());
		Files.writeString(service, controllerClass);
		assertFalse(SimConfigState.shipOptions().stream().anyMatch(o -> controllerClass.equals(o.className())));

		SimConfigState.applyCliOverrides(null, null, classes.toString(), null);
		var option = SimConfigState.shipOptions().stream().filter(o -> controllerClass.equals(o.className()))
				.findFirst().orElseThrow();
		assertEquals("Service Viewer", option.displayName());
		assertEquals(controllerClass, option.factory().get().getClass().getName());
		SimConfigState.applyCliOverrides(null, null, classes.toString(), null);
		assertEquals(1,
				SimConfigState.shipOptions().stream().filter(o -> controllerClass.equals(o.className())).count());
	}

	public static final class CustomViewerSub implements SubmarineController {
		@Override
		public String name() {
			return "Custom Viewer Sub";
		}

		@Override
		public String shortName() {
			return "CV";
		}

		@Override
		public void onTick(SubmarineInput input, SubmarineOutput output) {
			output.setThrottle(0.5);
		}
	}

	@Test
	void parsesNamedFlagsAndLegacyPositionalArgs() {
		var named = SubmarineScene3D.parseLaunchArgs(new String[] {"--ship1", CustomViewerSub.class.getName(),
				"--ship2", "codex", "--seed", "2c", "--speed", "4"});
		assertEquals(CustomViewerSub.class.getName(), named.ship1());
		assertEquals("codex", named.ship2());
		assertEquals(0x2cL, named.seed());
		assertEquals(4, named.speedMultiplier());
		assertNull(named.replayPath());

		var legacyReplay = SubmarineScene3D.parseLaunchArgs(new String[] {"target/replays/match.srl"});
		assertEquals("target/replays/match.srl", legacyReplay.replayPath());

		var legacySeed = SubmarineScene3D.parseLaunchArgs(new String[] {"deadbeef"});
		assertEquals(0xdeadbeefL, legacySeed.seed());
		assertNull(legacySeed.replayPath());

		var prefixedNamed = SubmarineScene3D.parseLaunchArgs(new String[] {"--seed", "0x2c"});
		assertEquals(0x2cL, prefixedNamed.seed());

		var prefixedPositional = SubmarineScene3D.parseLaunchArgs(new String[] {"0XDEADBEEF"});
		assertEquals(0xdeadbeefL, prefixedPositional.seed());
	}

	@Test
	void parseSeedAcceptsOptionalHexPrefixAndSignedDecimal() {
		assertEquals(0x2cL, SimConfigState.parseSeed("0x2c"));
		assertEquals(0xdeadbeefL, SimConfigState.parseSeed("  0XDEADBEEF  "));
		assertEquals(-1L, SimConfigState.parseSeed("0xffffffffffffffff"));
		assertEquals(-451325398205793679L, SimConfigState.parseSeed("-451325398205793679"));
		assertThrows(NumberFormatException.class, () -> SimConfigState.parseSeed("0x"));
	}

	@Test
	void simConfigDynamicallyRegistersExternalSubmarineFqcnWithoutDuplicating() {
		ClassLoader cl = ViewerLaunchArgsTest.class.getClassLoader();
		int idx1 = SimConfigState.resolveOrRegisterShip(CustomViewerSub.class.getName(), cl);
		assertTrue(idx1 > 0, "External SubmarineController FQCN should be registered dynamically");
		assertEquals("Custom Viewer Sub", SimConfigState.shipOptions().get(idx1).displayName());

		int idx2 = SimConfigState.resolveOrRegisterShip(CustomViewerSub.class.getName(), cl);
		assertEquals(idx1, idx2, "Repeated registration of same FQCN must reuse existing option");

		int claudeIdx = SimConfigState.resolveOrRegisterShip("claude", cl);
		assertTrue(claudeIdx > 0);
		assertEquals("Claude", SimConfigState.shipOptions().get(claudeIdx).displayName());
	}
}
