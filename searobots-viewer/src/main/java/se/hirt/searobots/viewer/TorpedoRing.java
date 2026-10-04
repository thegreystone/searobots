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

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.util.BufferUtils;

/**
 * A band of light round a torpedo's body in its submarine's team colour, so a torpedo shows whose
 * it is. Built in the coordinates of the torpedo model (models/torpedo.obj), whose body is a
 * 16-sided cylinder of radius {@link #BODY_R} along +Y from the nose (y = -11.6) to the stern (y =
 * 14.7); the band follows the same 16 sides, standing a little proud of them.
 */
public final class TorpedoRing {
	private static final int SIDES = 16;
	private static final float BODY_R = 1.41421f, PHASE = 3.5f * FastMath.DEG_TO_RAD, PROUD = 1.03f;

	private TorpedoRing() {
	}

	/**
	 * Adds a band from station {@code y0} to {@code y1} (model units along the body) to the torpedo
	 * model {@code torpedo}, lit in {@code color} with {@code level} of it as colour and
	 * {@code glow} as glow for the bloom filter.
	 */
	public static Geometry attach(
		AssetManager assets, Node torpedo, java.awt.Color color, float y0, float y1, float level, float glow) {
		float rIn = BODY_R * 0.98f, rOut = BODY_R * PROUD;
		// Per side: the outer face and the two end faces, from the body out to the band
		float[] pos = new float[SIDES * 3 * 4 * 3];
		int[] idx = new int[SIDES * 3 * 6];
		int p = 0, k = 0, base = 0;
		for (int i = 0; i < SIDES; i++) {
			float a0 = PHASE + FastMath.TWO_PI * i / SIDES, a1 = PHASE + FastMath.TWO_PI * (i + 1) / SIDES;
			float[][] quads = {{rOut, y0, rOut, y1}, {rIn, y0, rOut, y0}, {rIn, y1, rOut, y1}};
			for (float[] q : quads) {
				float[][] corners = {{q[0], q[1], a0}, {q[2], q[3], a0}, {q[2], q[3], a1}, {q[0], q[1], a1}};
				for (float[] c : corners) {
					pos[p++] = c[0] * FastMath.cos(c[2]);
					pos[p++] = c[1];
					pos[p++] = c[0] * FastMath.sin(c[2]);
				}
				for (int t : new int[] {0, 1, 2, 0, 2, 3})
					idx[k++] = base + t;
				base += 4;
			}
		}
		Mesh mesh = new Mesh();
		mesh.setBuffer(VertexBuffer.Type.Position, 3, BufferUtils.createFloatBuffer(pos));
		mesh.setBuffer(VertexBuffer.Type.Index, 3, BufferUtils.createIntBuffer(idx));
		mesh.updateBound();
		Geometry ring = new Geometry("TeamRing", mesh);
		ColorRGBA c = new ColorRGBA(color.getRed() / 255f, color.getGreen() / 255f, color.getBlue() / 255f, 1f);
		Material m = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		m.setColor("Color", c.mult(level));
		m.setColor("GlowColor", c.mult(glow));
		m.getAdditionalRenderState().setFaceCullMode(com.jme3.material.RenderState.FaceCullMode.Off);
		ring.setMaterial(m);
		torpedo.attachChild(ring);
		return ring;
	}
}
