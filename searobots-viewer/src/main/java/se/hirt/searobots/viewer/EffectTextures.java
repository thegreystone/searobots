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

import com.jme3.math.FastMath;
import com.jme3.texture.Image;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;

/**
 * Procedural textures for the underwater effects (explosions and wrecks): bubbles, big, in swarms
 * and smoky, the mottled skin of a gas bubble, froth and jets of spray, an oil slick and debris.
 * Each is generated once and shared.
 */
final class EffectTextures {
	private static Texture2D bigBubbles, bubbleSwarm, smokySkin, puffs, froth, oilSlick, debris, streak;

	private EffectTextures() {
	}

	/**
	 * Big air bubbles, four in a row, as they really rise: not round but wobbly, flattened
	 * ellipsoids and spherical caps (domed on top, flat or dished underneath), clear inside, with a
	 * thin silvery edge, brightest on top where it mirrors the light from the surface, and a darker
	 * band just inside it where the light bends. White, to be tinted by the particle colour; keep
	 * them upright (no random angle).
	 */
	static synchronized Texture2D bigBubbles() {
		if (bigBubbles == null) {
			// Per bubble: half width, height above the centre, depth below it (small: a cap), wobble
			float[][] shapes = {{0.92f, 0.62f, 0.18f, 0.06f}, {0.8f, 0.75f, 0.45f, 0.09f}, {0.95f, 0.5f, 0.1f, 0.05f},
					{0.85f, 0.7f, 0.3f, 0.12f}};
			bigBubbles = atlas(128, shapes.length, (cell, dx, dy) -> {
				float[] s = shapes[cell];
				// Shift down so the dome sits in the middle of the cell
				float y = dy + 0.5f * (s[1] - s[2]);
				float a = FastMath.atan2(y, dx);
				float c = FastMath.cos(a), sn = FastMath.sin(a);
				float h = sn > 0f ? s[1] : s[2];
				float edge = 1f / FastMath.sqrt((c / s[0]) * (c / s[0]) + (sn / h) * (sn / h));
				// Wobble: the outline ripples round the bubble
				float u = (a + FastMath.PI) / FastMath.TWO_PI * 8f;
				edge *= 1f + s[3] * (2f * fbm(u, cell * 5f, 8, 3) - 1f);
				float e = FastMath.sqrt(dx * dx + y * y) / edge;
				if (e >= 1f)
					return new float[] {1f, 1f, 1f, 0f};
				float aa = smooth((1f - e) / 0.035f);
				float top = 0.3f + 0.7f * smooth(0.5f + 0.6f * sn); // the edge mirrors the bright surface above
				float rim = FastMath.pow(e, 14f) * top;
				float band = smooth((e - 0.72f) / 0.16f) * (1f - smooth((e - 0.93f) / 0.05f)); // light bent away
				float ripple = fbm(dx * 3f + 2f + cell * 3f, y * 3f + 2f, 16, 3);
				float grey = Math.min(1f, 0.55f + 0.45f * rim / Math.max(0.01f, rim + 0.15f * band));
				float alpha = 0.04f + 0.05f * ripple + 0.12f * band + 0.9f * rim;
				return new float[] {grey, grey, Math.min(1f, grey * 1.03f), Math.min(1f, alpha) * aa};
			});
		}
		return bigBubbles;
	}

	/**
	 * Swarms of small bubbles, four in a row: dozens of tiny ones of mixed sizes, thickest in the
	 * middle, each a clear disc with a bright top edge. White, to be tinted by the particle colour.
	 */
	static synchronized Texture2D bubbleSwarm() {
		if (bubbleSwarm == null) {
			int cells = 4, per = 46;
			float[][][] swarm = new float[cells][per][];
			var r = new java.util.Random(0x5EA_B0B);
			for (int c = 0; c < cells; c++)
				for (int i = 0; i < per; i++) {
					// Gaussian spread, so they crowd towards the middle; mostly tiny, a few larger
					float x = (float) r.nextGaussian() * 0.33f, y = (float) r.nextGaussian() * 0.38f;
					float size = 0.025f + 0.09f * (float) Math.pow(r.nextFloat(), 3);
					swarm[c][i] = new float[] {x, y, size, 0.75f + 0.5f * r.nextFloat()};
				}
			bubbleSwarm = atlas(128, cells, (cell, dx, dy) -> {
				float alpha = 0f, grey = 0.7f;
				for (float[] b : swarm[cell]) {
					float bx = dx - b[0], by = dy - b[1];
					float e = FastMath.sqrt(bx * bx + by * by) / b[2];
					if (e >= 1f)
						continue;
					float top = 0.35f + 0.65f * smooth(0.5f + 0.6f * by / (e * b[2] + 1e-4f));
					float rim = FastMath.pow(e, 6f) * top * b[3];
					float a = (0.12f + 0.85f * rim) * smooth((1f - e) / 0.15f);
					if (a > alpha) {
						alpha = a;
						grey = 0.6f + 0.4f * Math.min(1f, rim * 1.5f);
					}
				}
				// Fade out towards the cell's edge, so no swarm shows the square it came in
				float d = FastMath.sqrt(dx * dx + dy * dy);
				return new float[] {grey, grey, Math.min(1f, grey * 1.03f), alpha * smooth((1f - d) / 0.25f)};
			});
		}
		return bubbleSwarm;
	}

	/**
	 * The skin of a gas bubble, to wrap round a sphere: mottled grey with uneven transparency, so
	 * the bubble looks lumpy and full of smoke. Repeats left to right.
	 */
	static synchronized Texture2D smokySkin() {
		if (smokySkin == null) {
			int w = 256, h = 128;
			ByteBuffer buf = BufferUtils.createByteBuffer(w * h * 4);
			for (int y = 0; y < h; y++) {
				for (int x = 0; x < w; x++) {
					float n = fbm(x / 32f, y / 32f, 8, 4);
					float detail = fbm(x / 8f, y / 8f, 32, 2);
					float grey = 0.45f + 0.5f * n + 0.15f * (detail - 0.5f);
					float a = FastMath.clamp(0.25f + 0.9f * (n - 0.35f) + 0.2f * (detail - 0.5f), 0.1f, 1f);
					put(buf, grey, grey, grey, a);
				}
			}
			buf.flip();
			smokySkin = texture(w, h, buf, Texture.WrapMode.Repeat);
		}
		return smokySkin;
	}

	/**
	 * Puffs of spray, mist or murk, eight in a row: soft, cloudy and ragged at the edge, white
	 * throughout (even where clear), to be tinted by the particle colour. Stands in for jME's
	 * Smoke.png, whose clear edges are green and tint whatever is built up out of many puffs.
	 */
	static synchronized Texture2D puffs() {
		if (puffs == null) {
			int cell = 64, cells = 8;
			ByteBuffer buf = BufferUtils.createByteBuffer(cell * cells * cell * 4);
			for (int y = 0; y < cell; y++) {
				for (int x = 0; x < cell * cells; x++) {
					float dx = ((x % cell) + 0.5f - cell / 2f) / (cell / 2f), dy = (y + 0.5f - cell / 2f) / (cell / 2f);
					float d = FastMath.sqrt(dx * dx + dy * dy);
					float n = fbm(x / 16f, y / 16f + 7f * (x / cell), 32, 4);
					float a = smooth((1f - d) / 0.5f + 0.9f * (n - 0.5f)) * (0.45f + 0.55f * n);
					put(buf, 1f, 1f, 1f, a);
				}
			}
			buf.flip();
			puffs = texture(cell * cells, cell, buf, Texture.WrapMode.EdgeClamp);
		}
		return puffs;
	}

	/**
	 * Churned white water, to wrap round a dome of spray: dense froth, mottled with greyer, thinner
	 * patches. Repeats left to right.
	 */
	static synchronized Texture2D froth() {
		if (froth == null) {
			int w = 256, h = 128;
			ByteBuffer buf = BufferUtils.createByteBuffer(w * h * 4);
			for (int y = 0; y < h; y++) {
				for (int x = 0; x < w; x++) {
					float n = fbm(x / 32f, y / 32f, 8, 4);
					float detail = fbm(x / 8f, y / 8f, 32, 2);
					float grey = 0.78f + 0.22f * n + 0.1f * (detail - 0.5f);
					put(buf, grey, grey, grey, FastMath.clamp(0.6f + 0.5f * n + 0.15f * (detail - 0.5f), 0f, 1f));
				}
			}
			buf.flip();
			froth = texture(w, h, buf, Texture.WrapMode.Repeat);
		}
		return froth;
	}

	/**
	 * A patch of oil on the water: dark, with a ragged edge and a faint rainbow sheen, for a quad
	 * lying on the surface.
	 */
	static synchronized Texture2D oilSlick() {
		if (oilSlick == null)
			oilSlick = disc(256, (dx, dy, d, n) -> {
				float edge = smooth((1f - d + 0.45f * (n - 0.5f)) * 3f);
				float sheen = fbm((dx + 1f) * 2f, (dy + 1f) * 2f, 4, 3);
				float hue = sheen * 4f;
				float k = 0.09f * smooth((sheen - 0.4f) * 2.5f);
				float r = 0.24f + k * (0.5f + 0.5f * FastMath.sin(hue * FastMath.TWO_PI));
				float g = 0.18f + k * (0.5f + 0.5f * FastMath.sin(hue * FastMath.TWO_PI + 2.1f));
				float b = 0.1f + k * (0.5f + 0.5f * FastMath.sin(hue * FastMath.TWO_PI + 4.2f));
				return new float[] {r, g, b, 0.85f * edge * (0.75f + 0.25f * n)};
			});
		return oilSlick;
	}

	/**
	 * A jet of spray: a narrow streak of white water running up the middle, soft and broken at the
	 * edges and ends, for particles that face their velocity.
	 */
	static synchronized Texture2D streak() {
		if (streak == null)
			streak = disc(64, (dx, dy, d, n) -> {
				float width = 0.22f + 0.12f * n;
				float across = Math.max(0f, 1f - Math.abs(dx) / width);
				float along = smooth((1f - Math.abs(dy)) / 0.6f);
				float a = across * across * along * (0.55f + 0.45f * n);
				return new float[] {1f, 1f, 1f, Math.min(1f, 1.4f * a)};
			});
		return streak;
	}

	/**
	 * Torn hull fragments, two by two: ragged shards of grey metal, lighter at the torn edges.
	 */
	static synchronized Texture2D debris() {
		if (debris == null) {
			int cell = 64, size = cell * 2, corners = 5;
			ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
			float[][] radii = new float[4][corners];
			for (int c = 0; c < 4; c++)
				for (int k = 0; k < corners; k++)
					radii[c][k] = 0.15f + 0.85f * lattice(c * 31 + k, 7, 1 << 16);
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					int c = (x / cell) + 2 * (y / cell);
					float dx = ((x % cell) + 0.5f - cell / 2f) / (cell / 2f),
							dy = ((y % cell) + 0.5f - cell / 2f) / (cell / 2f);
					float d = FastMath.sqrt(dx * dx + dy * dy);
					// The shard's outline: radius interpolated round its corners, roughened by noise
					float a = (FastMath.atan2(dy, dx) + FastMath.PI) / FastMath.TWO_PI * corners;
					int k0 = (int) a % corners;
					float r = FastMath.interpolateLinear(a - (int) a, radii[c][k0], radii[c][(k0 + 1) % corners])
							+ 0.08f * (fbm(x / 6f, y / 6f, 64, 2) - 0.5f);
					float alpha = FastMath.clamp((r - d) * 20f, 0f, 1f);
					float n = fbm(x / 10f, y / 10f, 32, 3);
					float grey = 0.1f + 0.2f * n + 0.18f * FastMath.clamp(1f - (r - d) * 8f, 0f, 1f);
					put(buf, grey, grey * 0.97f, grey * 0.93f, alpha);
				}
			}
			buf.flip();
			debris = texture(size, size, buf, Texture.WrapMode.EdgeClamp);
		}
		return debris;
	}

	private interface CellPixel {
		/** RGBA in image {@code cell} at offset ({@code dx}, {@code dy}) from its centre, -1..1. */
		float[] at(int cell, float dx, float dy);
	}

	/** {@code cells} square images of {@code size} pixels in a row. */
	private static Texture2D atlas(int size, int cells, CellPixel pixel) {
		ByteBuffer buf = BufferUtils.createByteBuffer(size * cells * size * 4);
		float c = size / 2f;
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size * cells; x++) {
				float[] p = pixel.at(x / size, ((x % size) + 0.5f - c) / c, (y + 0.5f - c) / c);
				put(buf, p[0], p[1], p[2], p[3]);
			}
		}
		buf.flip();
		return texture(size * cells, size, buf, Texture.WrapMode.EdgeClamp);
	}

	private interface Pixel {
		/**
		 * RGBA at offset ({@code dx}, {@code dy}) from the centre, {@code d} from it, with noise
		 * {@code n}.
		 */
		float[] at(float dx, float dy, float d, float n);
	}

	/** A round texture, transparent outside the unit circle. */
	private static Texture2D disc(int size, Pixel pixel) {
		ByteBuffer buf = BufferUtils.createByteBuffer(size * size * 4);
		float c = size / 2f;
		for (int y = 0; y < size; y++) {
			for (int x = 0; x < size; x++) {
				float dx = (x + 0.5f - c) / c, dy = (y + 0.5f - c) / c;
				float d = FastMath.sqrt(dx * dx + dy * dy);
				if (d > 1f) {
					put(buf, 1f, 1f, 1f, 0f);
					continue;
				}
				float n = fbm((dx + 1f) * 3f, (dy + 1f) * 3f, 6, 4);
				float[] p = pixel.at(dx, dy, d, n);
				put(buf, p[0], p[1], p[2], p[3]);
			}
		}
		buf.flip();
		return texture(size, size, buf, Texture.WrapMode.EdgeClamp);
	}

	private static void put(ByteBuffer buf, float r, float g, float b, float a) {
		buf.put(toByte(r)).put(toByte(g)).put(toByte(b)).put(toByte(a));
	}

	private static byte toByte(float v) {
		return (byte) Math.round(255 * FastMath.clamp(v, 0f, 1f));
	}

	private static Texture2D texture(int w, int h, ByteBuffer buf, Texture.WrapMode wrap) {
		var t = new Texture2D(new Image(Image.Format.RGBA8, w, h, buf, ColorSpace.sRGB));
		t.setWrap(wrap);
		return t;
	}

	static float smooth(float x) {
		x = FastMath.clamp(x, 0f, 1f);
		return x * x * (3f - 2f * x);
	}

	/** Fractal value noise in 0..1, repeating every {@code period} units, over {@code octaves}. */
	private static float fbm(float x, float y, int period, int octaves) {
		float sum = 0, amp = 0.5f, total = 0;
		for (int o = 0; o < octaves; o++) {
			sum += amp * valueNoise(x, y, period);
			total += amp;
			x *= 2;
			y *= 2;
			period *= 2;
			amp *= 0.5f;
		}
		return sum / total;
	}

	private static float valueNoise(float x, float y, int period) {
		int x0 = (int) Math.floor(x), y0 = (int) Math.floor(y);
		float fx = smooth(x - x0), fy = smooth(y - y0);
		float a = lattice(x0, y0, period), b = lattice(x0 + 1, y0, period);
		float c = lattice(x0, y0 + 1, period), d = lattice(x0 + 1, y0 + 1, period);
		return FastMath.interpolateLinear(fy, FastMath.interpolateLinear(fx, a, b),
				FastMath.interpolateLinear(fx, c, d));
	}

	private static float lattice(int x, int y, int period) {
		x = Math.floorMod(x, period);
		y = Math.floorMod(y, period);
		int h = x * 374761393 + y * 668265263;
		h = (h ^ (h >>> 13)) * 1274126177;
		return ((h ^ (h >>> 16)) & 0xFFFF) / 65535f;
	}
}
