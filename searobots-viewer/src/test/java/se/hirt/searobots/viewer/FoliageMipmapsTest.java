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

import com.jme3.texture.Image;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import org.junit.jupiter.api.Test;

import java.nio.ByteBuffer;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class FoliageMipmapsTest {

	@Test
	void filtersPremultipliedColorWithoutTransparentColorBleeding() {
		Image source = image(2, 2, new int[] {255, 0, 0, 255, 0, 0, 255, 0, 255, 0, 0, 255, 0, 0, 255, 0});
		Image mips = FoliageMipmaps.generate(source);
		ByteBuffer pixels = mips.getData(0);
		int last = mips.getMipMapSizes()[0];
		assertEquals(Image.Format.RGBA8, mips.getFormat());
		assertEquals(ColorSpace.sRGB, mips.getColorSpace());
		assertEquals(255, pixels.get(last) & 255);
		assertEquals(0, pixels.get(last + 1) & 255);
		assertEquals(0, pixels.get(last + 2) & 255);
		assertEquals(128, pixels.get(last + 3) & 255);
		// Edge dilation changes only colour, leaving fully transparent pixels transparent.
		assertEquals(255, pixels.get(4) & 255);
		assertEquals(0, pixels.get(7) & 255);
	}

	@Test
	void averagesSrgbInLinearLight() {
		Image mips = FoliageMipmaps
				.generate(image(2, 2, new int[] {255, 255, 255, 255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255, 255}));
		int offset = mips.getMipMapSizes()[0];
		for (int c = 0; c < 3; c++)
			assertEquals(188, mips.getData(0).get(offset + c) & 255, 1);
		assertEquals(255, mips.getData(0).get(offset + 3) & 255);
	}

	@Test
	void retainsCutoutCoverageAcrossUsefulMipLevels() {
		int size = 128;
		int[] pixels = new int[size * size * 4];
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				float dx = (2f * x + 1f) / size - 1f, dy = (2f * y + 1f) / size - 1f;
				float radius = (float) Math.sqrt(dx * dx + dy * dy);
				int p = (y * size + x) * 4;
				pixels[p] = 80;
				pixels[p + 1] = 150;
				pixels[p + 2] = 50;
				pixels[p + 3] = Math.round(Math.max(0f, Math.min(1f, (0.65f - radius) * 20f)) * 255f);
			}
		}
		Image mips = FoliageMipmaps.generate(image(size, size, pixels));
		float target = coverage(mips.getData(0), 0, size * size);
		int offset = 0, w = size;
		for (int levelSize : mips.getMipMapSizes()) {
			if (w >= 16)
				assertEquals(target, coverage(mips.getData(0), offset, w * w), 0.025, "Coverage drift at " + w + "px");
			offset += levelSize;
			w = Math.max(1, w / 2);
		}
		assertEquals(offset, mips.getData(0).remaining());
	}

	@Test
	void preservesEveryOpaqueSrgbChannelAtFullResolution() {
		int[] pixels = new int[256 * 4];
		for (int i = 0; i < 256; i++) {
			pixels[i * 4] = i;
			pixels[i * 4 + 1] = 255 - i;
			pixels[i * 4 + 2] = i;
			pixels[i * 4 + 3] = 255;
		}
		Image mips = FoliageMipmaps.generate(image(256, 1, pixels));
		for (int p = 0; p < pixels.length; p++)
			assertEquals(pixels[p], mips.getData(0).get(p) & 255);
	}

	@Test
	void includesEntireNpotImageAndStopsAtOnePixel() {
		int[] pixels = new int[7 * 5 * 4];
		Arrays.fill(pixels, 255);
		Image mips = FoliageMipmaps.generate(image(7, 5, pixels));
		assertArrayEquals(new int[] {7 * 5 * 4, 3 * 2 * 4, 4}, mips.getMipMapSizes());
		for (int p = 0; p < mips.getData(0).remaining(); p++)
			assertEquals(255, mips.getData(0).get(p) & 255);
	}

	private static Image image(int width, int height, int[] channels) {
		ByteBuffer data = BufferUtils.createByteBuffer(channels.length);
		for (int channel : channels)
			data.put((byte) channel);
		data.flip();
		return new Image(Image.Format.RGBA8, width, height, data, ColorSpace.sRGB);
	}

	private static float coverage(ByteBuffer pixels, int offset, int count) {
		int covered = 0;
		for (int p = offset + 3; p < offset + count * 4; p += 4) {
			if ((pixels.get(p) & 255) / 255f >= FoliageMipmaps.ALPHA_DISCARD)
				covered++;
		}
		return covered / (float) count;
	}
}
