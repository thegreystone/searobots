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
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.SubmarineInput;
import se.hirt.searobots.api.SubmarineOutput;

import static org.junit.jupiter.api.Assertions.*;

class ViewerLaunchArgsTest {

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
