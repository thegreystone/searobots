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
import com.jme3.asset.TextureKey;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import org.junit.jupiter.api.Test;

import java.nio.ByteBuffer;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class FoliageMipmapsTest {

	@Test
	void atlasMipsKeepAdjacentViewsSeparateAndPreserveTheBakedBaseIncludingTransparentRgb() {
		int width = 8, height = 4;
		int[] pixels = new int[width * height * 4];
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				int p = (y * width + x) * 4;
				pixels[p + (x < 4 ? 0 : 2)] = 255;
				pixels[p + 3] = 255;
			}
		}
		pixels[0] = 31;
		pixels[1] = 203;
		pixels[2] = 17;
		pixels[3] = 0;
		Image mips = FoliageMipmaps.generateAtlas(image(width, height, pixels), 4, 4, false);
		assertArrayEquals(new int[] {128, 32, 8}, mips.getMipMapSizes());
		for (int p = 0; p < pixels.length; p++)
			assertEquals(pixels[p], mips.getData(0).get(p) & 255, "Keep the baker's full-resolution gutter unchanged");
		int offset = mips.getMipMapSizes()[0], tileSize = 2;
		for (int level = 1; level < mips.getMipMapSizes().length; level++) {
			for (int y = 0; y < tileSize; y++) {
				for (int x = 0; x < tileSize * 2; x++) {
					int p = offset + (y * tileSize * 2 + x) * 4;
					assertEquals(x < tileSize ? 255 : 0, mips.getData(0).get(p) & 255);
					assertEquals(0, mips.getData(0).get(p + 1) & 255);
					assertEquals(x < tileSize ? 0 : 255, mips.getData(0).get(p + 2) & 255);
				}
			}
			offset += mips.getMipMapSizes()[level];
			tileSize /= 2;
		}
		Image linearAverage = FoliageMipmaps.generateAtlas(image(4, 2, new int[] {255, 255, 255, 255, 0, 0, 0, 255, 255,
				0, 0, 255, 255, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255, 255, 255, 0, 0, 255, 255, 0, 0, 255}), 2, 2,
				false);
		for (int c = 0; c < 3; c++)
			assertEquals(188, linearAverage.getData(0).get(32 + c) & 255, 1);
	}

	@Test
	void atlasCoverageIsPreservedPerViewAndNormalAlphaMatchesAlbedoExactly() {
		int tileSize = 64, width = tileSize * 2;
		int[] colors = new int[width * tileSize * 4], normals = new int[colors.length];
		for (int y = 0; y < tileSize; y++) {
			for (int x = 0; x < width; x++) {
				float dx = (2f * (x % tileSize) + 1f) / tileSize - 1f;
				float dy = (2f * y + 1f) / tileSize - 1f;
				float radius = (float) Math.sqrt(dx * dx + dy * dy), silhouette = x < tileSize ? .40f : .87f;
				int alpha = Math.round(Math.max(0f, Math.min(1f, (silhouette - radius) * 20f)) * 255f);
				int p = (y * width + x) * 4;
				colors[p] = 70;
				colors[p + 1] = 140;
				colors[p + 2] = 40;
				normals[p] = x < tileSize ? 255 : 128;
				normals[p + 1] = 128;
				normals[p + 2] = x < tileSize ? 128 : 255;
				colors[p + 3] = normals[p + 3] = alpha;
			}
		}
		Image albedo = FoliageMipmaps.generateAtlas(image(width, tileSize, colors), tileSize, tileSize, false);
		Image normal = FoliageMipmaps.generateAtlas(image(width, tileSize, normals), tileSize, tileSize, true);
		assertEquals(ColorSpace.sRGB, albedo.getColorSpace());
		assertEquals(ColorSpace.Linear, normal.getColorSpace());
		assertArrayEquals(albedo.getMipMapSizes(), normal.getMipMapSizes());
		for (int p = 3; p < albedo.getData(0).limit(); p += 4)
			assertEquals(albedo.getData(0).get(p), normal.getData(0).get(p),
					"The two maps must cut out the same pixels");
		float[] target = {tileCoverage(albedo, 0, tileSize, 0), tileCoverage(albedo, 0, tileSize, 1)};
		assertTrue(target[1] - target[0] > .4f, "Exercise very different silhouette coverage in adjacent views");
		int offset = 0, size = tileSize;
		for (int level = 0; level < albedo.getMipMapSizes().length; level++) {
			for (int tile = 0; tile < 2; tile++) {
				if (size >= 16)
					assertEquals(target[tile], tileCoverage(albedo, offset, size, tile), .025);
				if (level > 0) {
					for (int y = 0; y < size; y++) {
						for (int x = 0; x < size; x++) {
							int p = offset + (y * size * 2 + tile * size + x) * 4;
							float nx = normalChannel(normal, p), ny = normalChannel(normal, p + 1),
									nz = normalChannel(normal, p + 2);
							assertEquals(1f, Math.sqrt(nx * nx + ny * ny + nz * nz), .012,
									"Filtered and gutter normals stay unit length");
							if ((normal.getData(0).get(p + 3) & 255) > 0)
								assertTrue(tile == 0 ? nx > .995f : nz > .995f, "Adjacent view directions cannot mix");
						}
					}
				}
			}
			offset += albedo.getMipMapSizes()[level];
			size /= 2;
		}
	}

	@Test
	void normalMipsAverageDirectionsWithAlphaAndRenormalizeWithoutSrgbConversion() {
		Image mips = FoliageMipmaps
				.generateAtlas(
						image(4, 2,
								new int[] {255, 128, 128, 255, 128, 255, 128, 255, 128, 128, 255, 255, 128, 128, 255,
										255, 128, 128, 0, 0, 128, 128, 255, 0, 128, 128, 255, 255, 128, 128, 255, 255}),
						2, 2, true);
		int p = mips.getMipMapSizes()[0];
		assertEquals(ColorSpace.Linear, mips.getColorSpace());
		assertEquals(Math.sqrt(.5), normalChannel(mips, p), .01);
		assertEquals(Math.sqrt(.5), normalChannel(mips, p + 1), .01);
		assertEquals(0, normalChannel(mips, p + 2), .01);
		assertEquals(128, mips.getData(0).get(p + 3) & 255);
		assertEquals(1f, normalChannel(mips, p + 6), .01);
	}

	@Test
	void rectangularAtlasStopsBeforeGlMipDimensionsWouldMergeTiles() {
		int[] pixels = new int[8 * 16 * 4];
		Arrays.fill(pixels, 255);
		Image mips = FoliageMipmaps.generateAtlas(image(8, 16, pixels), 2, 4, false);
		assertArrayEquals(new int[] {8 * 16 * 4, 4 * 8 * 4}, mips.getMipMapSizes());
		assertThrows(IllegalArgumentException.class,
				() -> FoliageMipmaps.generateAtlas(image(8, 16, pixels), 3, 4, false));
		assertThrows(IllegalArgumentException.class,
				() -> FoliageMipmaps.generateAtlas(image(8, 16, pixels), 16, 4, false));
	}

	@Test
	void atlasCacheSeparatesLayoutsAndNormalMapsWithoutMutatingTheSourceImage() {
		var assets = new DesktopAssetManager(true);
		TextureKey sourceKey = new TextureKey("test/impostor.png", true);
		sourceKey.setGenerateMips(true);
		int[] pixels = new int[4 * 2 * 4];
		Arrays.fill(pixels, 255);
		Image source = image(4, 2, pixels);
		Texture original = new Texture2D(source);
		original.setKey(sourceKey);
		assets.addToCache(sourceKey, original);
		Texture first = FoliageMipmaps.loadAtlas(assets, sourceKey.getName(), 2, 2, false);
		Texture again = FoliageMipmaps.loadAtlas(assets, sourceKey.getName(), 2, 2, false);
		Texture normal = FoliageMipmaps.loadAtlas(assets, sourceKey.getName(), 2, 2, true);
		Texture differentLayout = FoliageMipmaps.loadAtlas(assets, sourceKey.getName(), 4, 2, false);
		assertSame(first.getImage(), again.getImage());
		assertNotSame(first.getImage(), normal.getImage());
		assertNotSame(first.getImage(), differentLayout.getImage());
		assertSame(source, original.getImage());
		assertNull(source.getMipMapSizes());
		assertEquals(ColorSpace.sRGB, source.getColorSpace());
		assertEquals(ColorSpace.Linear, normal.getImage().getColorSpace());
		assertEquals(Texture.WrapMode.EdgeClamp, first.getWrap(Texture.WrapAxis.S));
		assertEquals(Texture.WrapMode.EdgeClamp, first.getWrap(Texture.WrapAxis.T));
		assertEquals(Texture.MinFilter.Trilinear, first.getMinFilter());
		assertEquals(1, first.getAnisotropicFilter());
	}

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

	private static float tileCoverage(Image image, int offset, int size, int tile) {
		int covered = 0;
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				int p = offset + (y * size * 2 + tile * size + x) * 4 + 3;
				if ((image.getData(0).get(p) & 255) / 255f >= FoliageMipmaps.ALPHA_DISCARD)
					covered++;
			}
		}
		return covered / (float) (size * size);
	}

	private static float normalChannel(Image image, int p) {
		return (image.getData(0).get(p) & 255) / 127.5f - 1f;
	}
}
