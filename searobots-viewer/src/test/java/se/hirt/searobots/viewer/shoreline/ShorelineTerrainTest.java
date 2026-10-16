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
package se.hirt.searobots.viewer.shoreline;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.math.Vector2f;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.terrain.geomipmap.TerrainQuad;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class ShorelineTerrainTest {

	@Test
	void publicGroundHeightMatchesScaledTerrainForObjectPlacement() {
		var assets = new DesktopAssetManager(true);
		var landscape = ShorelineTerrain.create(assets, 417L);
		landscape.updateGeometricState();
		var terrain = (TerrainQuad) landscape.getChild("Sandy cove and headland");
		for (float x : new float[] {-127.5f, -103f, -31.5f, 0f, 74.5f, 107f, 127.5f}) {
			for (float z : new float[] {-81f, -23f, 2.5f, 49f, 103f, 127.5f}) {
				assertEquals(ShorelineTerrain.heightAt(x, z), terrain.getHeight(new Vector2f(x, z)), 0.0001f);
			}
		}
		assertEquals(ShadowMode.CastAndReceive, terrain.getShadowMode());
	}

	@Test
	void coveOpensToWaterAndRisesGentlyIntoForest() {
		assertTrue(ShorelineTerrain.heightAt(0f, -55f) < -3f);
		assertEquals(0f, ShorelineTerrain.heightAt(0f, -25f), 0.0001f);
		assertTrue(ShorelineTerrain.heightAt(0f, -15f) > 0f);
		assertTrue(ShorelineTerrain.heightAt(0f, -15f) < 1.5f);
		assertTrue(ShorelineTerrain.heightAt(0f, 65f) > 6f);
	}

	@Test
	void meshPerimeterIsSubmergedBeyondTheForestDomain() {
		for (float coordinate : new float[] {-128f, -64f, 0f, 64f, 128f}) {
			assertTrue(ShorelineTerrain.heightAt(-128f, coordinate) <= -64f);
			assertTrue(ShorelineTerrain.heightAt(128f, coordinate) <= -64f);
			assertTrue(ShorelineTerrain.heightAt(coordinate, 128f) <= -64f);
		}
		assertTrue(ShorelineTerrain.heightAt(109f, 80f) > 0f);
		assertTrue(ShorelineTerrain.heightAt(0f, 106f) > 0f);
	}
}
