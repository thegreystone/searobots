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

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import com.jme3.asset.AssetKey;
import com.jme3.asset.AssetManager;
import com.jme3.environment.util.EnvMapUtils;
import com.jme3.material.Material;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.texture.Image;
import com.jme3.texture.TextureCubeMap;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.BufferUtils;
import com.jme3.util.SkyFactory;

/** Original clouded coastal HDR sky, with inexpensive daylight modulation and no fixed sun. */
public final class CoastalSky {

	private static final String MATERIAL = "Shaders/Shoreline/CoastalSky.j3md";
	private static final AssetKey<TextureCubeMap> CUBE_KEY = new AssetKey<>("procedural/coastal-clouded-hdr-sky-v1");

	private CoastalSky() {
	}

	/**
	 * Creates a sky at full daylight. An independently cloned material can remain at daylight for
	 * probe capture while the visible sky follows the match time; both share the same cached map.
	 */
	public static Spatial create(AssetManager assets) {
		TextureCubeMap cube = assets.getFromCache(CUBE_KEY);
		if (cube == null) {
			cube = cubeMap();
			assets.addToCache(CUBE_KEY, cube);
		}
		// The SkyFactory sphere normals point inward; the map stores outward world directions.
		Geometry sky = (Geometry) SkyFactory.createSky(assets, cube, new Vector3f(-1, -1, -1),
				SkyFactory.EnvMapType.CubeMap);
		Material material = new Material(assets, MATERIAL);
		material.setTexture("Texture", cube);
		material.setVector3("NormalScale", new Vector3f(-1, -1, -1));
		material.setFloat("Daylight", 1);
		sky.setMaterial(material);
		sky.setName("Clouded coastal sky");
		return sky;
	}

	/** Updates only the material uniform; zero is a dark blue night and one is full daylight. */
	public static void updateDaylight(Spatial sky, float daylight) {
		((Geometry) sky).getMaterial().setFloat("Daylight", FastMath.clamp(daylight, 0, 1));
	}

	private static TextureCubeMap cubeMap() {
		int size = 256;
		ArrayList<ByteBuffer> faces = new ArrayList<>();
		Vector3f direction = new Vector3f();
		for (int face = 0; face < 6; face++) {
			ByteBuffer pixels = BufferUtils.createByteBuffer(size * size * 3 * 4).order(ByteOrder.nativeOrder());
			for (int y = 0; y < size; y++) {
				for (int x = 0; x < size; x++) {
					EnvMapUtils.getVectorFromCubemapFaceTexCoord(x, y, size, face, direction,
							EnvMapUtils.FixSeamsMethod.Stretch);
					float height = Math.max(0, direction.y), zenith = (float) Math.pow(height, .45),
							haze = (float) Math.exp(-height * 6);
					float cloud = 0;
					if (direction.y > .07f) {
						float cx = direction.x / (direction.y + .15f), cz = direction.z / (direction.y + .15f);
						float noise = (float) (.52 + .20 * Math.sin(cx * 2.1 + Math.sin(cz * 1.3))
								+ .15 * Math.sin(cz * 3.7 - cx * .8) + .08 * Math.sin(cx * 8.1 + cz * 6.5));
						cloud = FastMath.clamp((noise - .66f) * 3, 0, .75f)
								* FastMath.clamp((direction.y - .07f) * 5, 0, 1);
					}
					float red = .075f * zenith + .40f * (1 - zenith) + haze * .06f,
							green = .23f * zenith + .60f * (1 - zenith), blue = .62f * zenith + .82f * (1 - zenith);
					if (direction.y < 0) {
						// Continue the visible horizon into a muted ocean hemisphere for indirect light.
						float below = (float) Math.sqrt(-direction.y);
						red = .46f * (1 - below) + .08f * below;
						green = .60f * (1 - below) + .17f * below;
						blue = .82f * (1 - below) + .28f * below;
					}
					pixels.putFloat(red * (1 - cloud) + .93f * cloud);
					pixels.putFloat(green * (1 - cloud) + .91f * cloud);
					pixels.putFloat(blue * (1 - cloud) + .86f * cloud);
				}
			}
			pixels.flip();
			faces.add(pixels);
		}
		return new TextureCubeMap(new Image(Image.Format.RGB32F, size, size, 1, faces, ColorSpace.Linear));
	}
}
