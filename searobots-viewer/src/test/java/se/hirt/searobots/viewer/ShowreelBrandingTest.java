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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.Test;

import se.hirt.searobots.viewer.showreel.ShowreelOverlay;

class ShowreelBrandingTest {

	@Test
	void defaultsMatchOriginalShowreelBranding() {
		var options = Showreel.Options.parse(new String[0]);
		assertNotNull(options);
		assertEquals(ShowreelOverlay.DEFAULT_TITLE, options.title);
		assertEquals(ShowreelOverlay.DEFAULT_TAGLINE, options.tagline);
		assertEquals("github.com/thegreystone/searobots", options.url);
	}

	@Test
	void customLeagueBrandingFlagsParseCleanly() {
		var options = Showreel.Options.parse(new String[] {"--replay", "match.srl", "--title", "SEAROBOTS LEAGUE",
				"--tagline", "2026-12 QUARTERFINAL  ·  RED OCTOBER DEF. GEMINI (17-11)", "--url", "searobots.hirt.se"});
		assertNotNull(options);
		assertEquals("SEAROBOTS LEAGUE", options.title);
		assertEquals("2026-12 QUARTERFINAL  ·  RED OCTOBER DEF. GEMINI (17-11)", options.tagline);
		assertEquals("searobots.hirt.se", options.url);
	}
}
