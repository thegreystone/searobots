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
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Font;
import java.awt.image.BufferedImage;

import org.junit.jupiter.api.Test;

class ShowreelOverlayCardTest {

	@Test
	void rendersLeagueTitleAndLongTaglineCardWithoutError() {
		BufferedImage card = ShowreelOverlay.renderCard(1280, 640, Font.SANS_SERIF, Font.SANS_SERIF, "SEAROBOTS LEAGUE",
				"SEASON 2026-12 GRAND FINAL  ·  RED OCTOBER DEFEATS GEMINI ATTACK SUB (17-14)", "searobots.hirt.se");
		assertEquals(1280, card.getWidth());
		assertEquals(640, card.getHeight());
		// Verify visible non-transparent pixels were drawn
		int opaquePixels = 0;
		for (int y = 0; y < card.getHeight(); y += 4) {
			for (int x = 0; x < card.getWidth(); x += 4) {
				if ((card.getRGB(x, y) >>> 24) > 20) {
					opaquePixels++;
				}
			}
		}
		assertTrue(opaquePixels > 100, "Rendered card should contain drawn title pixels");
	}

	@Test
	void fallsBackToDefaultsWhenTitleOrTaglineBlank() {
		BufferedImage card = ShowreelOverlay.renderCard(960, 480, Font.SANS_SERIF, Font.SANS_SERIF, " ", null, null);
		assertEquals(960, card.getWidth());
		assertEquals(480, card.getHeight());
	}
}
