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

import com.jme3.asset.DesktopAssetManager;
import com.jme3.font.BitmapText;
import com.jme3.scene.Spatial;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class HudPanelTest {
	@Test
	void wrapsLongNamesWithoutLosingWordsWhenSpaceIsAvailable() {
		String source = "VeryLongControllerName speed 12.5 m/s\nContact 340 degrees";
		var lines = HudPanel.fit(source, 12, 20, String::length);
		assertTrue(lines.stream().allMatch(line -> line.length() <= 12));
		assertEquals(source.replaceAll("\\s", ""), String.join("", lines).replaceAll("\\s", ""));
	}

	@Test
	void overflowIsExplicitAndNeverExceedsTheAvailableRows() {
		var lines = HudPanel.fit("First\nSecond\nThird\nFourth\nFifth", 20, 3, String::length);
		assertEquals(3, lines.size());
		assertEquals("First", lines.getFirst());
		assertEquals("... 3 more lines", lines.getLast());
		assertTrue(HudPanel.fit("Anything", 100, 0, String::length).isEmpty());
	}

	@Test
	void resizingRestoresFullContentAndEmptyTextRemovesTheBackdrop() {
		var assets = new DesktopAssetManager(true);
		var text = new BitmapText(assets.loadFont("Interface/Fonts/Default.fnt"));
		text.setSize(14);
		String source = "VeryLongControllerName  /  ACTIVE\nSpeed 12.5 m/s   Depth 180 m\nHP 1000   Torpedoes 8";
		text.setText(source);
		var panel = new HudPanel(assets, text);
		float smallHeight = panel.layout(12, 400, 140, 70, 1);
		assertTrue(smallHeight <= 70);
		assertNotEquals(source, text.getText());
		assertTrue(text.getText().contains("more"));
		panel.layout(12, 400, 600, 240, 1);
		assertEquals(source, text.getText());
		text.setText("");
		assertEquals(0, panel.layout(12, 400, 600, 240, 1));
		assertEquals(Spatial.CullHint.Always, panel.getCullHint());
	}
}
