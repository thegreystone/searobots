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
import com.jme3.font.BitmapFont;
import com.jme3.font.BitmapText;
import com.jme3.font.LineWrapMode;
import com.jme3.font.Rectangle;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.shape.Quad;
import com.jme3.util.BufferUtils;

import java.nio.FloatBuffer;
import java.util.Objects;

/**
 * Render-thread loading screen. Progress is supplied by the setup work; only its presentation is
	 * interpolated. Instrument rings and a quiet scanning sweep stay active during long tasks.
 */
final class LoadingOverlay extends Node {

	private static final ColorRGBA CYAN = new ColorRGBA(0.31f, 0.88f, 0.96f, 1f);
	private final Geometry backing;
	private final Node ring = new Node("LoadingRing");
	private final Node outerDial = new Node("LoadingOuterDial");
	private final Node innerDial = new Node("LoadingInnerDial");
	private final Node scanner = new Node("LoadingScanner");
	private final Arc progressArc = new Arc(160, 0.030f);
	private final Arc progressGlow = new Arc(160, 0.11f);
	private final Geometry progressTip;
	private final Geometry gleam;
	private final Quaternion gleamRotation = new Quaternion();
	private final Quaternion outerRotation = new Quaternion();
	private final Quaternion innerRotation = new Quaternion();
	private final Quaternion scanRotation = new Quaternion();
	private final BitmapText title;
	private final BitmapText subtitle;
	private final BitmapText percentage;
	private final BitmapText status;
	private int width = -1, height = -1, shownPercentage = -1;
	private float displayedProgress, phase, centerX, centerY, scale;
	private boolean visible = true;
	private String currentMessage = "";

	LoadingOverlay(AssetManager assets, BitmapFont font) {
		super("LoadingOverlay");
		// The default GUI camera spans z=-1000..1000; normal HUD and Lemur use z=0..10.
		setLocalTranslation(0, 0, 100);
		setQueueBucket(RenderQueue.Bucket.Gui);
		setCullHint(CullHint.Never);
		backing = new Geometry("LoadingBackdrop", new Quad(1, 1));
		backing.setMaterial(material(assets, new ColorRGBA(0.014f, 0.034f, 0.055f, 1f)));
		attachChild(backing);
		attachChild(ring);
		ring.setLocalTranslation(0, 0, 1);

		addArc(ring, assets, "InnerGuide", .64f, .006f, 0, 360, new ColorRGBA(.26f, .68f, .78f, .18f));
		addArc(ring, assets, "OuterGuide", 1.38f, .005f, 0, 360, new ColorRGBA(.26f, .68f, .78f, .12f));
		addArc(ring, assets, "ProgressTrack", 1f, .030f, 0, 360, new ColorRGBA(.10f, .22f, .28f, 1f));
		ring.attachChild(outerDial);
		ring.attachChild(innerDial);
		ring.attachChild(scanner);
		outerDial.setLocalTranslation(0, 0, .01f);
		innerDial.setLocalTranslation(0, 0, .01f);
		scanner.setLocalTranslation(0, 0, .01f);
		addArc(outerDial, assets, "OuterSegmentA", 1.16f, .014f, 8, 82, new ColorRGBA(.30f, .78f, .88f, .42f));
		addArc(outerDial, assets, "OuterSegmentB", 1.16f, .014f, 104, 36, new ColorRGBA(.30f, .78f, .88f, .26f));
		addArc(outerDial, assets, "OuterSegmentC", 1.16f, .014f, 164, 126, new ColorRGBA(.30f, .78f, .88f, .32f));
		addArc(outerDial, assets, "OuterSegmentD", 1.16f, .035f, 312, 20, new ColorRGBA(.42f, .89f, .96f, .65f));
		addArc(innerDial, assets, "InnerSegmentA", .80f, .009f, 24, 102, new ColorRGBA(.28f, .76f, .85f, .32f));
		addArc(innerDial, assets, "InnerSegmentB", .80f, .009f, 190, 134, new ColorRGBA(.28f, .76f, .85f, .22f));
		addTicks(assets);
		addReticle(assets);
		addScan(assets);

		progressArc.mesh.setDynamic();
		progressGlow.mesh.setDynamic();
		Geometry glow = new Geometry("LoadingProgressGlow", progressGlow.mesh);
		glow.setMaterial(material(assets, new ColorRGBA(.20f, .78f, .94f, .12f)));
		glow.setLocalTranslation(0, 0, .02f);
		ring.attachChild(glow);
		Geometry progressRing = new Geometry("LoadingRingProgress", progressArc.mesh);
		progressRing.setMaterial(material(assets, CYAN));
		progressRing.setLocalTranslation(0, 0, 0.03f);
		ring.attachChild(progressRing);
		progressTip = new Geometry("LoadingProgressTip", new Quad(.035f, .035f));
		progressTip.setMaterial(material(assets, new ColorRGBA(.79f, .98f, 1f, 1f)));
		ring.attachChild(progressTip);

		Arc gleamArc = new Arc(24, 0.030f);
		gleamArc.setSweep(FastMath.TWO_PI * 0.10f);
		float[] colors = new float[25 * 2 * 4];
		for (int i = 0; i <= 24; i++) {
			float alpha = FastMath.sin(FastMath.PI * i / 24f) * 0.7f;
			for (int side = 0; side < 2; side++) {
				int offset = (i * 2 + side) * 4;
				colors[offset] = 0.68f;
				colors[offset + 1] = 0.96f;
				colors[offset + 2] = 1f;
				colors[offset + 3] = alpha;
			}
		}
		gleamArc.mesh.setBuffer(VertexBuffer.Type.Color, 4, colors);
		gleam = new Geometry("LoadingRingGleam", gleamArc.mesh);
		Material gleamMaterial = material(assets, ColorRGBA.White);
		gleamMaterial.setBoolean("VertexColor", true);
		gleam.setMaterial(gleamMaterial);
		gleam.setLocalTranslation(0, 0, 0.04f);
		ring.attachChild(gleam);

		title = text(font, "SeaRobots", new ColorRGBA(0.83f, 0.91f, 0.95f, 1f));
		subtitle = text(font, "P R E P A R I N G   W O R L D", new ColorRGBA(0.42f, 0.64f, 0.72f, 1f));
		percentage = text(font, "0%", new ColorRGBA(0.90f, 0.97f, 0.99f, 1f));
		status = text(font, "", new ColorRGBA(0.63f, 0.76f, 0.82f, 1f));
		status.setBox(new Rectangle(0, 0, 1, 1));
		status.setAlignment(BitmapFont.Align.Center);
		status.setVerticalAlignment(BitmapFont.VAlign.Center);
		status.setLineWrapMode(LineWrapMode.Word);
	}

	void update(float tpf, int width, int height, float progress, String message) {
		float elapsed = Float.isFinite(tpf) ? FastMath.clamp(tpf, 0, 0.1f) : 0;
		float target = Float.isFinite(progress) ? FastMath.clamp(progress, 0, 1) : 0;
		// A lower value means a new preparation cycle; never animate a completed ring backwards.
		if (target == 1f || target < displayedProgress) {
			displayedProgress = target;
		} else {
			displayedProgress += (target - displayedProgress) * (1f - (float) Math.exp(-elapsed * 8f));
			if (target - displayedProgress < 0.0005f)
				displayedProgress = target;
		}
		progressArc.setSweep(displayedProgress * FastMath.TWO_PI);
		progressGlow.setSweep(displayedProgress * FastMath.TWO_PI);
		float tipAngle = FastMath.HALF_PI - displayedProgress * FastMath.TWO_PI;
		progressTip.setLocalTranslation(FastMath.cos(tipAngle) - .0175f, FastMath.sin(tipAngle) - .0175f, .05f);
		progressTip.setCullHint(displayedProgress > 0 ? CullHint.Never : CullHint.Always);
		phase = (phase + elapsed) % 504f; // Common period of the decorative rotations.
		gleam.setLocalRotation(gleamRotation.fromAngleAxis(-phase * FastMath.TWO_PI / 7f, Vector3f.UNIT_Z));
		outerDial.setLocalRotation(outerRotation.fromAngleAxis(phase * FastMath.TWO_PI / 28f, Vector3f.UNIT_Z));
		innerDial.setLocalRotation(innerRotation.fromAngleAxis(-phase * FastMath.TWO_PI / 18f, Vector3f.UNIT_Z));
		scanner.setLocalRotation(scanRotation.fromAngleAxis(-phase * FastMath.TWO_PI / 9f, Vector3f.UNIT_Z));

		boolean resized = this.width != Math.max(1, width) || this.height != Math.max(1, height);
		if (resized)
			layout(Math.max(1, width), Math.max(1, height));
		int percent = (int) (displayedProgress * 100f);
		if (percent != shownPercentage || resized) {
			shownPercentage = percent;
			percentage.setText(percent + "%");
			center(percentage, centerY + percentage.getLineHeight() * 0.5f);
		}
		String task = Objects.requireNonNullElse(message, "Preparing your world");
		if (!task.equals(currentMessage)) {
			currentMessage = task;
			status.setText(task);
		}
	}

	void setVisible(boolean visible) {
		if (visible && !this.visible) {
			displayedProgress = 0;
			shownPercentage = -1;
		}
		this.visible = visible;
		setCullHint(visible ? CullHint.Never : CullHint.Always);
	}

	private void layout(int width, int height) {
		this.width = width;
		this.height = height;
		scale = Math.min(1.2f, Math.min(width / 800f, height / 650f));
		centerX = width * 0.5f;
		centerY = height * 0.53f;
		float radius = 96f * scale;
		backing.setLocalScale(width, height, 1);
		ring.setLocalScale(radius, radius, 1);
		ring.setLocalTranslation(centerX, centerY, 1);
		title.setSize(24f * scale);
		subtitle.setSize(12f * scale);
		percentage.setSize(32f * scale);
		status.setSize(16f * scale);
		center(title, centerY + radius * 1.48f + 46f * scale);
		center(subtitle, centerY - radius * 1.48f - 28f * scale);
		float padding = 32f * scale;
		status.setBox(new Rectangle(0, 0, width - padding * 2f, 54f * scale));
		status.setLocalTranslation(padding, Math.max(66f * scale, height * 0.13f), 3);
	}

	private BitmapText text(BitmapFont font, String value, ColorRGBA color) {
		BitmapText text = new BitmapText(font);
		text.setText(value);
		text.setColor(color);
		attachChild(text);
		return text;
	}

	private void center(BitmapText text, float y) {
		text.setLocalTranslation(centerX - text.getLineWidth() * 0.5f, y, 3);
	}

	private static Material material(AssetManager assets, ColorRGBA color) {
		Material material = new Material(assets, "Common/MatDefs/Misc/Unshaded.j3md");
		material.setColor("Color", color);
		RenderState state = material.getAdditionalRenderState();
		state.setDepthTest(false);
		state.setDepthWrite(false);
		state.setFaceCullMode(RenderState.FaceCullMode.Off);
		state.setBlendMode(RenderState.BlendMode.Alpha);
		return material;
	}

	private static void addArc(Node parent, AssetManager assets, String name, float radius, float thickness,
			float start, float degrees, ColorRGBA color) {
		Arc arc = new Arc(Math.max(12, (int) (degrees / 2)), radius, thickness);
		arc.setSweep(degrees * FastMath.DEG_TO_RAD);
		Geometry geometry = new Geometry(name, arc.mesh);
		geometry.setMaterial(material(assets, color));
		geometry.rotate(0, 0, -start * FastMath.DEG_TO_RAD);
		parent.attachChild(geometry);
	}

	/** All seventy-two instrument ticks share a single static mesh. */
	private void addTicks(AssetManager assets) {
		float[] positions = new float[72 * 12];
		int[] indices = new int[72 * 6];
		for (int i = 0; i < 72; i++) {
			float angle = FastMath.HALF_PI - i * FastMath.TWO_PI / 72;
			float x = FastMath.cos(angle), y = FastMath.sin(angle);
			float inner = i % 6 == 0 ? 1.26f : 1.30f;
			float halfWidth = i % 6 == 0 ? .007f : .004f;
			int p = i * 12, vertex = i * 4, index = i * 6;
			float[] radii = {inner, inner, 1.34f, 1.34f};
			for (int corner = 0; corner < 4; corner++) {
				float side = corner == 0 || corner == 3 ? -halfWidth : halfWidth;
				positions[p + corner * 3] = x * radii[corner] - y * side;
				positions[p + corner * 3 + 1] = y * radii[corner] + x * side;
			}
			indices[index] = vertex;
			indices[index + 1] = vertex + 1;
			indices[index + 2] = vertex + 2;
			indices[index + 3] = vertex;
			indices[index + 4] = vertex + 2;
			indices[index + 5] = vertex + 3;
		}
		Mesh mesh = new Mesh();
		mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
		mesh.setBuffer(VertexBuffer.Type.Index, 3, indices);
		mesh.updateBound();
		Geometry ticks = new Geometry("LoadingInstrumentTicks", mesh);
		ticks.setMaterial(material(assets, new ColorRGBA(.35f, .75f, .84f, .35f)));
		ring.attachChild(ticks);
	}

	private void addReticle(AssetManager assets) {
		Material ink = material(assets, new ColorRGBA(.40f, .83f, .91f, .40f));
		for (int i = 0; i < 4; i++) {
			Node marker = new Node("LoadingCardinalMarker");
			Geometry stem = new Geometry("Stem", new Quad(.008f, .11f));
			stem.setLocalTranslation(-.004f, 1.40f, 0);
			stem.setMaterial(ink);
			Geometry cap = new Geometry("Cap", new Quad(.12f, .008f));
			cap.setLocalTranslation(-.06f, 1.51f, 0);
			cap.setMaterial(ink);
			marker.attachChild(stem);
			marker.attachChild(cap);
			marker.rotate(0, 0, i * FastMath.HALF_PI);
			ring.attachChild(marker);
		}
	}

	/** A translucent annular sweep leaves the central percentage unobstructed. */
	private void addScan(AssetManager assets) {
		Arc scan = new Arc(48, .87f, .16f);
		scan.setSweep(FastMath.HALF_PI);
		float[] colors = new float[49 * 2 * 4];
		for (int i = 0; i <= 48; i++) {
			for (int side = 0; side < 2; side++) {
				int index = (i * 2 + side) * 4;
				colors[index] = .22f;
				colors[index + 1] = .75f;
				colors[index + 2] = .90f;
				colors[index + 3] = (float) Math.pow(1f - i / 48f, 2) * .13f;
			}
		}
		scan.mesh.setBuffer(VertexBuffer.Type.Color, 4, colors);
		Geometry sweep = new Geometry("LoadingScanSweep", scan.mesh);
		Material scanMaterial = material(assets, ColorRGBA.White);
		scanMaterial.setBoolean("VertexColor", true);
		sweep.setMaterial(scanMaterial);
		scanner.attachChild(sweep);
		addArc(scanner, assets, "ScanLeadingEdge", .87f, .16f, 0, .5f, new ColorRGBA(.38f, .87f, .96f, .26f));
	}

	/** Clockwise annular strip in normalized GUI coordinates, starting at twelve o'clock. */
	private static final class Arc {
		private final Mesh mesh = new Mesh();
		private final FloatBuffer positions;
		private final int segments;
		private final float innerRadius, outerRadius;

		private Arc(int segments, float thickness) {
			this(segments, 1f, thickness);
		}

		private Arc(int segments, float radius, float thickness) {
			this.segments = segments;
			innerRadius = radius - thickness * 0.5f;
			outerRadius = radius + thickness * 0.5f;
			positions = BufferUtils.createFloatBuffer((segments + 1) * 2 * 3);
			mesh.setBuffer(VertexBuffer.Type.Position, 3, positions);
			int[] indices = new int[segments * 6];
			for (int i = 0; i < segments; i++) {
				int vertex = i * 2, offset = i * 6;
				indices[offset] = vertex;
				indices[offset + 1] = vertex + 1;
				indices[offset + 2] = vertex + 2;
				indices[offset + 3] = vertex + 1;
				indices[offset + 4] = vertex + 3;
				indices[offset + 5] = vertex + 2;
			}
			mesh.setBuffer(VertexBuffer.Type.Index, 3, indices);
			setSweep(0);
		}

		private void setSweep(float sweep) {
			positions.clear();
			for (int i = 0; i <= segments; i++) {
				float angle = FastMath.HALF_PI - sweep * i / segments;
				float x = FastMath.cos(angle), y = FastMath.sin(angle);
				positions.put(x * innerRadius).put(y * innerRadius).put(0);
				positions.put(x * outerRadius).put(y * outerRadius).put(0);
			}
			positions.flip();
			mesh.getBuffer(VertexBuffer.Type.Position).updateData(positions);
			mesh.updateBound();
		}
	}
}
