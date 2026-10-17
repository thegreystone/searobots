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
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.material.Material;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.texture.Image;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.MipMapGenerator;
import com.jme3.water.WaterFilter;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class WaterQualitySettingsTest {

	@Test
	void preservesSampledFoamIntensityAcrossAllMipLevels() {
		ByteBuffer pixels = BufferUtils.createByteBuffer(16 * 16 * 3);
		for (int i = 0; i < 256; i++)
			pixels.put((byte) i).put((byte) i).put((byte) i);
		pixels.flip();
		Image image = new Image(Image.Format.RGB8, 16, 16, pixels, ColorSpace.Linear);
		MipMapGenerator.generateMipMaps(image);
		byte[] linear = new byte[image.getData(0).remaining()];
		image.getData(0).get(linear);
		image.getData(0).rewind();

		WaterQualitySettings.encodeFoamColor(image);

		assertEquals(ColorSpace.sRGB, image.getColorSpace());
		assertEquals(188, image.getData(0).get(128 * 3) & 255);
		for (int i = 0; i < linear.length; i++) {
			double srgb = (image.getData(0).get(i) & 255) / 255.0;
			double sampled = srgb <= 0.04045 ? srgb / 12.92 : Math.pow((srgb + 0.055) / 1.055, 2.4);
			// Eight-bit sRGB quantization costs at most about one linear byte of precision.
			assertEquals((linear[i] & 255) / 255.0, sampled, 0.0041, "Sample " + i);
		}
	}

	@Test
	void standardWaterMaterialAcceptsNamedFoamWithoutColorSpaceWarning() {
		var assets = new DesktopAssetManager(false);
		assets.registerLoader(J3MLoader.class, "j3m", "j3md");
		assets.registerLocator("/", ClasspathLocator.class);
		var material = new Material(assets, "Common/MatDefs/Water/Water.j3md");
		var water = new WaterFilter();
		WaterQualitySettings.configure(water);
		assertEquals("ShorelineFoam", water.getFoamTexture().getName());
		assertTrue(water.getFoamTexture().getImage().hasMipmaps());

		List<LogRecord> warnings = new ArrayList<>();
		Handler handler = new Handler() {
			@Override
			public void publish(LogRecord record) {
				if (record.getLevel().intValue() >= Level.WARNING.intValue())
					warnings.add(record);
			}

			@Override
			public void flush() {
			}

			@Override
			public void close() {
			}
		};
		Logger logger = Logger.getLogger(Material.class.getName());
		logger.addHandler(handler);
		try {
			material.setTexture("FoamMap", water.getFoamTexture());
			assertTrue(warnings.isEmpty(), "Binding foam emitted a material warning");
		} finally {
			logger.removeHandler(handler);
		}
	}
}
