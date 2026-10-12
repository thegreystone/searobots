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

import java.nio.ByteBuffer;

import com.jme3.math.Vector3f;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.MipMapGenerator;
import com.jme3.water.WaterFilter;

/**
 * Higher-resolution reflections and softer, broken shoreline foam using the standard water shader.
 */
final class WaterQualitySettings {

	private static final int FOAM_TEXTURE_SIZE = 512;
	private static Texture2D shorelineFoam;

	private WaterQualitySettings() {
	}

	static void configure(WaterFilter water) {
		water.setReflectionMapSize(1024);
		// Keep overhead views transparent while retaining strong reflections at grazing angles.
		water.setRefractionConstant(0.035f);
		water.setShininess(0.85f);
		water.setUseHQShoreline(true);
		water.setFoamTexture(foamTexture());
		water.setFoamIntensity(0.45f);
		// Vertical water depth: fade after 20 cm, disappear by 2.8 m. Wave foam starts at 1.1 m.
		water.setFoamExistence(new Vector3f(0.2f, 2.8f, 1.1f));
		// The shader multiplies ray depth by this value: 1.5 blends over about 67 cm,
		// instead of the abrupt 2 cm transition produced by the old value of 50.
		water.setFoamHardness(1.5f);
	}

	/**
	 * Sparse foam filaments and flecks, generated once. Periodic noise makes every scale tile
	 * seamlessly; the water shader's two drifting samples break up the repeating pattern. Dark gaps
	 * represent clear water, instead of adding a grey veil across the whole surf band.
	 */
	private static synchronized Texture2D foamTexture() {
		if (shorelineFoam != null)
			return shorelineFoam;
		ByteBuffer pixels = BufferUtils.createByteBuffer(FOAM_TEXTURE_SIZE * FOAM_TEXTURE_SIZE * 3);
		for (int y = 0; y < FOAM_TEXTURE_SIZE; y++) {
			float v = (float) y / FOAM_TEXTURE_SIZE;
			for (int x = 0; x < FOAM_TEXTURE_SIZE; x++) {
				float u = (float) x / FOAM_TEXTURE_SIZE;
				float warpX = 2f * (noise(u * 4f, v * 4f, 4, 7531) - 0.5f);
				float warpY = 2f * (noise(u * 4f, v * 4f, 4, 1289) - 0.5f);
				float detail = 0.65f * noise(u * 16f + warpX, v * 16f + warpY, 16, 8721)
						+ 0.35f * noise(u * 32f + 2f * warpX, v * 32f + 2f * warpY, 32, 2183);
				float filaments = 1f - smooth((Math.abs(detail - 0.5f) - 0.025f) / 0.07f);
				float patches = smooth((noise(u * 4f, v * 4f, 4, 9349) - 0.28f) / 0.4f);
				float flecks = smooth((noise(u * 64f, v * 64f, 64, 5879) - 0.62f) / 0.3f);
				byte value = (byte) Math.round(255f * patches * (0.88f * filaments + 0.12f * flecks));
				pixels.put(value).put(value).put(value);
			}
		}
		pixels.flip();
		Image image = new Image(Image.Format.RGB8, FOAM_TEXTURE_SIZE, FOAM_TEXTURE_SIZE, pixels, ColorSpace.Linear);
		MipMapGenerator.generateMipMaps(image);
		encodeFoamColor(image);
		shorelineFoam = new Texture2D(image);
		shorelineFoam.setName("ShorelineFoam");
		shorelineFoam.setWrap(Texture.WrapMode.Repeat);
		shorelineFoam.setMagFilter(Texture.MagFilter.Bilinear);
		shorelineFoam.setMinFilter(Texture.MinFilter.Trilinear);
		shorelineFoam.setAnisotropicFilter(4);
		return shorelineFoam;
	}

	static void encodeFoamColor(Image image) {
		// FoamMap is a colour texture in the standard water material. Encode after filtering
		// in linear light so sRGB sampling preserves the generated foam intensity at every mip.
		byte[] encoded = new byte[256];
		for (int i = 0; i < encoded.length; i++) {
			float linear = i / 255f;
			float srgb = linear <= 0.0031308f ? 12.92f * linear : 1.055f * (float) Math.pow(linear, 1.0 / 2.4) - 0.055f;
			encoded[i] = (byte) Math.round(srgb * 255f);
		}
		ByteBuffer pixels = image.getData(0);
		for (int i = 0; i < pixels.limit(); i++)
			pixels.put(i, encoded[pixels.get(i) & 255]);
		image.setColorSpace(ColorSpace.sRGB);
	}

	private static float noise(float x, float y, int period, int seed) {
		int ix = (int) Math.floor(x), iy = (int) Math.floor(y);
		float fx = smooth(x - ix), fy = smooth(y - iy);
		float a = hash(Math.floorMod(ix, period), Math.floorMod(iy, period), seed);
		float b = hash(Math.floorMod(ix + 1, period), Math.floorMod(iy, period), seed);
		float c = hash(Math.floorMod(ix, period), Math.floorMod(iy + 1, period), seed);
		float d = hash(Math.floorMod(ix + 1, period), Math.floorMod(iy + 1, period), seed);
		return (a + (b - a) * fx) * (1f - fy) + (c + (d - c) * fx) * fy;
	}

	private static float hash(int x, int y, int seed) {
		int h = x * 0x1f1f1f1f ^ y * 0x5f356495 ^ seed;
		h = (h ^ (h >>> 16)) * 0x7feb352d;
		h = (h ^ (h >>> 15)) * 0x846ca68b;
		h ^= h >>> 16;
		return (h & 0xffffff) / (float) 0xffffff;
	}

	private static float smooth(float value) {
		float t = Math.max(0f, Math.min(1f, value));
		return t * t * (3f - 2f * t);
	}
}
