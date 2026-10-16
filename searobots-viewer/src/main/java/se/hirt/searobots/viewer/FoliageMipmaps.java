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

import com.jme3.asset.AssetKey;
import com.jme3.asset.AssetManager;
import com.jme3.math.ColorRGBA;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.image.ColorSpace;
import com.jme3.texture.image.ImageRaster;
import com.jme3.util.BufferUtils;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

/** Mipmaps for alpha-tested leaves: filter colour in linear space and retain their silhouette. */
final class FoliageMipmaps {

	static final float ALPHA_DISCARD = 0.45f;
	private static final float MIN_ALPHA_SCALE = 0.5f;
	private static final float MAX_ALPHA_SCALE = 2f;

	private FoliageMipmaps() {
	}

	static Texture load(AssetManager assetManager, String path) {
		Texture texture = assetManager.loadTexture(path);
		// Keep the generated image with its asset manager, so repeated world builds reuse it and
		// neither the manager nor its native buffers are retained by a static cache.
		AssetKey<Image> key = new AssetKey<>(path + ".foliage-mips-v1");
		Image image = assetManager.getFromCache(key);
		if (image == null) {
			image = generate(texture.getImage());
			assetManager.addToCache(key, image);
		}
		texture.setImage(image);
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		texture.setAnisotropicFilter(8);
		texture.setWrap(Texture.WrapMode.EdgeClamp);
		return texture;
	}

	/** Filters each impostor view independently, preserving its own alpha-test coverage. */
	static Texture loadAtlas(AssetManager assetManager, String path, int tileWidth, int tileHeight, boolean normalMap) {
		Texture texture = assetManager.loadTexture(path);
		AssetKey<Image> key = new AssetKey<>(
				path + ".foliage-atlas-mips-v1-" + tileWidth + "x" + tileHeight + (normalMap ? "-normal" : "-albedo"));
		Image image = assetManager.getFromCache(key);
		if (image == null) {
			image = generateAtlas(texture.getImage(), tileWidth, tileHeight, normalMap);
			assetManager.addToCache(key, image);
		}
		texture.setImage(image);
		texture.setMinFilter(Texture.MinFilter.Trilinear);
		texture.setMagFilter(Texture.MagFilter.Bilinear);
		// Anisotropic footprints can cross neighbouring view tiles even with a colour gutter.
		texture.setAnisotropicFilter(1);
		texture.setWrap(Texture.WrapMode.EdgeClamp);
		return texture;
	}

	static Image generateAtlas(Image source, int tileWidth, int tileHeight, boolean normalMap) {
		int width = source.getWidth(), height = source.getHeight();
		if (tileWidth <= 0 || tileHeight <= 0 || (tileWidth & (tileWidth - 1)) != 0
				|| (tileHeight & (tileHeight - 1)) != 0 || width % tileWidth != 0 || height % tileHeight != 0)
			throw new IllegalArgumentException(
					"Atlas tiles must have power-of-two dimensions and exactly fill the image");
		int columns = width / tileWidth, rows = height / tileHeight;
		float[][] tiles = new float[columns * rows][tileWidth * tileHeight * 4];
		float[] targetCoverage = new float[tiles.length];
		byte[] base = new byte[width * height * 4];
		ImageRaster raster = ImageRaster.create(source, 0, 0, false);
		boolean srgb = source.getColorSpace() == ColorSpace.sRGB;
		ColorRGBA color = new ColorRGBA();
		for (int row = 0; row < rows; row++) {
			for (int column = 0; column < columns; column++) {
				float[] pixels = tiles[row * columns + column];
				for (int y = 0; y < tileHeight; y++) {
					for (int x = 0; x < tileWidth; x++) {
						int wx = column * tileWidth + x, wy = row * tileHeight + y;
						raster.getPixel(wx, wy, color);
						int p = (y * tileWidth + x) * 4, dest = (wy * width + wx) * 4;
						for (int c = 0; c < 3; c++) {
							float channel = c == 0 ? color.r : c == 1 ? color.g : color.b;
							pixels[p + c] = (normalMap ? channel * 2f - 1f : srgb ? srgbToLinear(channel) : channel)
									* color.a;
							// Keep the baked full-resolution colour gutter, including transparent RGB.
							base[dest + c] = (byte) toByte(normalMap || srgb ? channel : linearToSrgb(channel));
						}
						pixels[p + 3] = color.a;
						base[dest + 3] = (byte) toByte(color.a);
					}
				}
				targetCoverage[row * columns + column] = coverage(pixels, 1f);
			}
		}
		List<byte[]> levels = new ArrayList<>();
		levels.add(base);
		int w = tileWidth, h = tileHeight;
		// A rectangular tile must stop at 1x2 (or 2x1): another GL mip would merge columns
		// or rows. Square tiles reach 1x1. Every supplied level retains the original layout.
		while (w > 1 && h > 1) {
			int nextWidth = w / 2, nextHeight = h / 2;
			byte[] level = new byte[nextWidth * columns * nextHeight * rows * 4];
			for (int tile = 0; tile < tiles.length; tile++) {
				// Alpha corrections never feed back into filtering, and identical source alpha
				// gives the albedo and normal atlases exactly the same corrected silhouette.
				tiles[tile] = downsample(tiles[tile], w, h, nextWidth, nextHeight);
				float scale = coverageScale(tiles[tile], targetCoverage[tile]);
				byte[] pixels = normalMap ? encodeNormals(tiles[tile], nextWidth, nextHeight, scale)
						: encode(tiles[tile], nextWidth, nextHeight, scale);
				int column = tile % columns, row = tile / columns;
				for (int y = 0; y < nextHeight; y++)
					System.arraycopy(pixels, y * nextWidth * 4, level,
							((row * nextHeight + y) * nextWidth * columns + column * nextWidth) * 4, nextWidth * 4);
			}
			levels.add(level);
			w = nextWidth;
			h = nextHeight;
		}
		return mipImage(width, height, levels, normalMap ? ColorSpace.Linear : ColorSpace.sRGB);
	}

	static Image generate(Image source) {
		int width = source.getWidth(), height = source.getHeight();
		// ImageRaster's optional colour conversion uses gamma 2.2. Match the GPU's actual sRGB
		// transfer instead, including its linear segment, so the full-resolution colour is exact.
		ImageRaster raster = ImageRaster.create(source, 0, 0, false);
		boolean srgb = source.getColorSpace() == ColorSpace.sRGB;
		float[] pixels = new float[width * height * 4];
		ColorRGBA color = new ColorRGBA();
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				raster.getPixel(x, y, color);
				int p = (y * width + x) * 4;
				pixels[p] = (srgb ? srgbToLinear(color.r) : color.r) * color.a;
				pixels[p + 1] = (srgb ? srgbToLinear(color.g) : color.g) * color.a;
				pixels[p + 2] = (srgb ? srgbToLinear(color.b) : color.b) * color.a;
				pixels[p + 3] = color.a;
			}
		}

		float targetCoverage = coverage(pixels, 1f);
		List<byte[]> levels = new ArrayList<>();
		int w = width, h = height;
		while (true) {
			float scale = levels.isEmpty() ? 1f : coverageScale(pixels, targetCoverage);
			levels.add(encode(pixels, w, h, scale));
			if (w == 1 && h == 1)
				break;
			int nextWidth = Math.max(1, w / 2), nextHeight = Math.max(1, h / 2);
			// Use the uncorrected premultiplied pixels for every next level. Feeding scaled alpha
			// back into the pyramid would accumulate the correction and inflate distant crowns.
			pixels = downsample(pixels, w, h, nextWidth, nextHeight);
			w = nextWidth;
			h = nextHeight;
		}

		return mipImage(width, height, levels, ColorSpace.sRGB);
	}

	private static Image mipImage(int width, int height, List<byte[]> levels, ColorSpace colorSpace) {
		int[] sizes = levels.stream().mapToInt(a -> a.length).toArray();
		int totalSize = levels.stream().mapToInt(a -> a.length).sum();
		ByteBuffer data = BufferUtils.createByteBuffer(totalSize);
		for (byte[] level : levels)
			data.put(level);
		data.flip();
		return new Image(Image.Format.RGBA8, width, height, data, sizes, colorSpace);
	}

	private static byte[] encodeNormals(float[] pixels, int width, int height, float alphaScale) {
		byte[] encoded = new byte[pixels.length];
		boolean[] filled = new boolean[width * height];
		for (int p = 0; p < pixels.length; p += 4) {
			writeNormal(encoded, p, pixels[p], pixels[p + 1], pixels[p + 2]);
			encoded[p + 3] = (byte) toByte(pixels[p + 3] * alphaScale);
			filled[p / 4] = pixels[p + 3] > 0f;
		}
		// Extend directions into transparent edge texels within this view tile only. Averaged
		// gutter directions also need normalization; RGB averaging alone would shorten them.
		for (int pass = 0; pass < 2; pass++) {
			byte[] previous = encoded.clone();
			boolean[] previousFilled = filled.clone();
			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {
					int p = (y * width + x) * 4;
					if (previousFilled[p / 4])
						continue;
					float nx = 0, ny = 0, nz = 0;
					int count = 0;
					for (int dy = -1; dy <= 1; dy++) {
						for (int dx = -1; dx <= 1; dx++) {
							int sx = x + dx, sy = y + dy;
							if (sx < 0 || sx >= width || sy < 0 || sy >= height || !previousFilled[sy * width + sx])
								continue;
							int neighbor = (sy * width + sx) * 4;
							nx += (previous[neighbor] & 0xff) / 127.5f - 1f;
							ny += (previous[neighbor + 1] & 0xff) / 127.5f - 1f;
							nz += (previous[neighbor + 2] & 0xff) / 127.5f - 1f;
							count++;
						}
					}
					if (count > 0) {
						writeNormal(encoded, p, nx, ny, nz);
						filled[p / 4] = true;
					}
				}
			}
		}
		return encoded;
	}

	private static void writeNormal(byte[] encoded, int p, float x, float y, float z) {
		float length = (float) Math.sqrt(x * x + y * y + z * z);
		if (length < .000001f) {
			x = y = 0f;
			z = length = 1f;
		}
		encoded[p] = (byte) toByte(x / length * .5f + .5f);
		encoded[p + 1] = (byte) toByte(y / length * .5f + .5f);
		encoded[p + 2] = (byte) toByte(z / length * .5f + .5f);
	}

	private static float[] downsample(float[] source, int width, int height, int nextWidth, int nextHeight) {
		float[] result = new float[nextWidth * nextHeight * 4];
		for (int y = 0; y < nextHeight; y++) {
			float startY = y * (float) height / nextHeight, endY = (y + 1) * (float) height / nextHeight;
			for (int x = 0; x < nextWidth; x++) {
				float startX = x * (float) width / nextWidth, endX = (x + 1) * (float) width / nextWidth;
				int dest = (y * nextWidth + x) * 4;
				float area = (endX - startX) * (endY - startY);
				for (int sy = (int) startY; sy < Math.ceil(endY); sy++) {
					float yWeight = Math.min(endY, sy + 1f) - Math.max(startY, sy);
					for (int sx = (int) startX; sx < Math.ceil(endX); sx++) {
						float xWeight = Math.min(endX, sx + 1f) - Math.max(startX, sx);
						float weight = xWeight * yWeight / area;
						int p = (sy * width + sx) * 4;
						for (int c = 0; c < 4; c++)
							result[dest + c] += source[p + c] * weight;
					}
				}
			}
		}
		return result;
	}

	private static float coverageScale(float[] pixels, float targetCoverage) {
		// Tiny mip levels cannot express a fractional silhouette. Prefer the scale closest to
		// one when multiple scales are equally accurate instead of growing a one-pixel halo.
		float best = 1f, bestError = Math.abs(coverage(pixels, best) - targetCoverage);
		float low = MIN_ALPHA_SCALE, high = MAX_ALPHA_SCALE;
		for (int i = 0; i < 20; i++) {
			float scale = (low + high) * 0.5f;
			float actual = coverage(pixels, scale), error = Math.abs(actual - targetCoverage);
			if (error < bestError || (error == bestError && Math.abs(scale - 1f) < Math.abs(best - 1f))) {
				best = scale;
				bestError = error;
			}
			if (actual < targetCoverage)
				low = scale;
			else
				high = scale;
		}
		return best;
	}

	private static float coverage(float[] pixels, float scale) {
		int covered = 0;
		for (int p = 3; p < pixels.length; p += 4) {
			if (toByte(pixels[p] * scale) / 255f >= ALPHA_DISCARD)
				covered++;
		}
		return covered / (float) (pixels.length / 4);
	}

	private static byte[] encode(float[] pixels, int width, int height, float alphaScale) {
		byte[] encoded = new byte[pixels.length];
		for (int p = 0; p < pixels.length; p += 4) {
			float alpha = pixels[p + 3];
			if (alpha > 0f) {
				for (int c = 0; c < 3; c++)
					encoded[p + c] = (byte) toByte(linearToSrgb(pixels[p + c] / alpha));
			}
			encoded[p + 3] = (byte) toByte(alpha * alphaScale);
		}
		// Trilinear sampling also sees fully transparent neighbours. Extend nearby leaf colour
		// into those texels without changing alpha to avoid black fringes around the cutout.
		for (int pass = 0; pass < 2; pass++) {
			byte[] previous = encoded.clone();
			for (int y = 0; y < height; y++) {
				for (int x = 0; x < width; x++) {
					int p = (y * width + x) * 4;
					if (pixels[p + 3] > 0f || hasColor(previous, p))
						continue;
					int count = 0, red = 0, green = 0, blue = 0;
					for (int dy = -1; dy <= 1; dy++) {
						for (int dx = -1; dx <= 1; dx++) {
							int sx = x + dx, sy = y + dy;
							if (sx < 0 || sx >= width || sy < 0 || sy >= height)
								continue;
							int neighbor = (sy * width + sx) * 4;
							if (pixels[neighbor + 3] <= 0f && !hasColor(previous, neighbor))
								continue;
							red += previous[neighbor] & 0xff;
							green += previous[neighbor + 1] & 0xff;
							blue += previous[neighbor + 2] & 0xff;
							count++;
						}
					}
					if (count > 0) {
						encoded[p] = (byte) (red / count);
						encoded[p + 1] = (byte) (green / count);
						encoded[p + 2] = (byte) (blue / count);
					}
				}
			}
		}
		return encoded;
	}

	private static boolean hasColor(byte[] pixels, int p) {
		return pixels[p] != 0 || pixels[p + 1] != 0 || pixels[p + 2] != 0;
	}

	private static float linearToSrgb(float value) {
		return value <= 0.0031308f ? value * 12.92f : 1.055f * (float) Math.pow(value, 1.0 / 2.4) - 0.055f;
	}

	private static float srgbToLinear(float value) {
		return value <= 0.04045f ? value / 12.92f : (float) Math.pow((value + 0.055f) / 1.055f, 2.4);
	}

	private static int toByte(float value) {
		return Math.round(Math.max(0f, Math.min(1f, value)) * 255f);
	}
}
