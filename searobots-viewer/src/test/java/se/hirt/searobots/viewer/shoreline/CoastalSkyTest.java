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

import java.nio.FloatBuffer;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.texture.Image;
import com.jme3.texture.image.ColorSpace;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class CoastalSkyTest {

	@Test
	void cachedSkyIsLinearAndDaylightChangesDoNotRewriteItsCubemapOrOtherSkies() {
		var assets = new DesktopAssetManager(true);
		Geometry sky = (Geometry) CoastalSky.create(assets), probeSky = (Geometry) CoastalSky.create(assets);
		Image image = sky.getMaterial().getTextureParam("Texture").getTextureValue().getImage();
		assertSame(image, probeSky.getMaterial().getTextureParam("Texture").getTextureValue().getImage());
		assertEquals(Image.Format.RGB32F, image.getFormat());
		assertEquals(ColorSpace.Linear, image.getColorSpace());
		assertEquals(6, image.getData().size());
		assertEquals(new Vector3f(-1, -1, -1), sky.getMaterial().getParam("NormalScale").getValue());
		assertEquals(RenderQueue.Bucket.Sky, sky.getQueueBucket());
		assertEquals(RenderQueue.ShadowMode.Off, sky.getShadowMode());
		float firstPixel = image.getData(0).getFloat(0);
		CoastalSky.updateDaylight(sky, 0);
		assertEquals(0f, sky.getMaterial().getParam("Daylight").getValue());
		assertEquals(1f, probeSky.getMaterial().getParam("Daylight").getValue());
		CoastalSky.updateDaylight(sky, 2);
		assertEquals(1f, sky.getMaterial().getParam("Daylight").getValue());
		CoastalSky.updateDaylight(sky, -1);
		assertEquals(0f, sky.getMaterial().getParam("Daylight").getValue());
		assertEquals(firstPixel, image.getData(0).getFloat(0));
	}

	@Test
	void skyContainsBlueAtmosphereAndCloudVariationWithoutAFixedBrightSun() {
		Geometry sky = (Geometry) CoastalSky.create(new DesktopAssetManager(true));
		Image image = sky.getMaterial().getTextureParam("Texture").getTextureValue().getImage();
		float maximum = 0, minimum = Float.POSITIVE_INFINITY;
		int neutralCloudPixels = 0, bluePixels = 0;
		for (var face : image.getData()) {
			FloatBuffer pixels = face.asFloatBuffer();
			for (int pixel = 0; pixel < pixels.limit(); pixel += 3) {
				float r = pixels.get(pixel), g = pixels.get(pixel + 1), b = pixels.get(pixel + 2);
				assertTrue(Float.isFinite(r) && Float.isFinite(g) && Float.isFinite(b));
				minimum = Math.min(minimum, Math.min(r, Math.min(g, b)));
				maximum = Math.max(maximum, Math.max(r, Math.max(g, b)));
				if (Math.abs(r - b) < .15f && r > .55f)
					neutralCloudPixels++;
				if (b > r + .25f)
					bluePixels++;
			}
		}
		assertTrue(minimum >= 0);
		assertTrue(maximum < 1, "The moving game sun supplies the disc; sky pixels contain no fixed bright sun");
		assertTrue(neutralCloudPixels > 1000, "Clouds should have visible structure");
		assertTrue(bluePixels > 1000, "Sky retains the showcase's blue atmosphere");
	}
}
