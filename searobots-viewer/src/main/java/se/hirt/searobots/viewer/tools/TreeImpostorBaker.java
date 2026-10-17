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
package se.hirt.searobots.viewer.tools;

import com.jme3.app.SimpleApplication;
import com.jme3.material.MatParam;
import com.jme3.material.MatParamTexture;
import com.jme3.material.Material;
import com.jme3.material.MaterialDef;
import com.jme3.material.TechniqueDef;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.shader.VarType;
import com.jme3.system.AppSettings;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image;
import com.jme3.util.BufferUtils;
import se.hirt.searobots.viewer.TreeScatter;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.ModeledTree;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeDetail;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeKind;
import se.hirt.searobots.viewer.shoreline.CoastalVegetation.TreeLibrary;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Offline orthographic surface-data bake of the same seeded archetypes used by the game. Run with a
 * native OpenGL context; no sun, environment light, tone mapping or shadows enter the textures.
 * Runtime startup only loads the resulting atlases.
 */
public final class TreeImpostorBaker extends SimpleApplication {

	private static final int TILE_WIDTH = 128, SIDE_HEIGHT = 256, SUPER_SAMPLE = 2;
	private static final float WIDTH_PADDING = 1.12f, HEIGHT_PADDING = 1.04f;
	private final Path output;
	private final Node subject = new Node("impostor bake subject");
	private final Map<Material, Material> bakeMaterials = new IdentityHashMap<>();
	private TreeLibrary library;
	private FrameBuffer target;
	private BufferedImage sideColor, sideNormal, topColor, topNormal;
	private int typeIndex, variant, view, pass, frame;
	private ByteBuffer pixels;
	private boolean finished;

	private TreeImpostorBaker(Path output) {
		this.output = output;
	}

	public static void main(String[] args) throws IOException {
		Path output = args.length == 0 ? Path.of("searobots-viewer/src/main/resources/Textures/Terrain/impostors")
				: Path.of(args[0]);
		Files.createDirectories(output);
		TreeImpostorBaker app = new TreeImpostorBaker(output);
		AppSettings settings = new AppSettings(true);
		settings.setTitle("SeaRobots tree surface bake");
		settings.setResolution(256, 512);
		settings.setAudioRenderer(null);
		settings.setGammaCorrection(false);
		settings.setSamples(0);
		settings.setVSync(false);
		settings.setFrameRate(-1);
		app.setSettings(settings);
		app.setShowSettings(false);
		app.setPauseOnLostFocus(false);
		app.start();
	}

	@Override
	public void simpleInitApp() {
		setDisplayFps(false);
		setDisplayStatView(false);
		flyCam.setEnabled(false);
		guiNode.setCullHint(Spatial.CullHint.Always);
		renderer.setLinearizeSrgbImages(true);
		renderer.setMainFrameBufferSrgb(false);
		viewPort.setBackgroundColor(new ColorRGBA(0, 0, 0, 0));
		viewPort.setClearFlags(true, true, true);
		renderManager.setPreferredLightMode(TechniqueDef.LightMode.SinglePass);
		rootNode.attachChild(subject);
		library = CoastalVegetation.treeLibrary(assetManager);
		beginType();
		selectFrame();
	}

	private void beginType() {
		sideColor = new BufferedImage(TILE_WIDTH * 4, SIDE_HEIGHT * 4, BufferedImage.TYPE_INT_ARGB);
		sideNormal = new BufferedImage(TILE_WIDTH * 4, SIDE_HEIGHT * 4, BufferedImage.TYPE_INT_ARGB);
		topColor = new BufferedImage(TILE_WIDTH * 4, TILE_WIDTH, BufferedImage.TYPE_INT_ARGB);
		topNormal = new BufferedImage(TILE_WIDTH * 4, TILE_WIDTH, BufferedImage.TYPE_INT_ARGB);
	}

	private void selectFrame() {
		TreeScatter.Type type = TreeScatter.Type.values()[typeIndex];
		TreeKind kind = TreeKind.valueOf(type.name());
		// Match the modeled tree's usual aspect ratio, retaining its exact normalized silhouette.
		Node model = library.batch(List.of(new ModeledTree(kind, variant, 0, 0, 0, typeWidth(type), 1, 0, 1, 1, 1)),
				TreeDetail.NEAR);
		for (Spatial child : model.getChildren()) {
			Geometry geometry = (Geometry) child;
			Material material = bakeMaterials.computeIfAbsent(geometry.getMaterial(), this::bakeMaterial);
			material.setBoolean("BakeNormals", pass == 1);
			geometry.setMaterial(material);
		}
		subject.detachAllChildren();
		subject.attachChild(model);
		int height = view == 4 ? TILE_WIDTH : SIDE_HEIGHT;
		int width = TILE_WIDTH * SUPER_SAMPLE, renderHeight = height * SUPER_SAMPLE;
		if (target == null || target.getHeight() != renderHeight) {
			if (target != null)
				renderer.deleteFrameBuffer(target);
			target = new FrameBuffer(width, renderHeight, 1);
			target.addColorTarget(FrameBuffer.FrameBufferTarget.newTarget(Image.Format.RGBA8));
			target.setDepthTarget(FrameBuffer.FrameBufferTarget.newTarget(Image.Format.Depth));
			viewPort.setOutputFrameBuffer(target);
			pixels = BufferUtils.createByteBuffer(width * renderHeight * 4);
		}
		cam.resize(width, renderHeight, true);
		cam.setParallelProjection(true);
		float halfWidth = typeWidth(type) * WIDTH_PADDING * .5f;
		float halfHeight = view == 4 ? halfWidth : HEIGHT_PADDING * .5f;
		cam.setFrustum(.1f, 30, -halfWidth, halfWidth, halfHeight, -halfHeight);
		Vector3f center = view == 4 ? Vector3f.ZERO : new Vector3f(0, halfHeight, 0);
		Vector3f direction = switch (view) {
		case 0 -> Vector3f.UNIT_Z;
		case 1 -> new Vector3f(-1, 0, 0);
		case 2 -> new Vector3f(0, 0, -1);
		case 3 -> Vector3f.UNIT_X;
		default -> Vector3f.UNIT_Y;
		};
		cam.setLocation(center.add(direction.mult(10)));
		cam.lookAt(center, view == 4 ? new Vector3f(0, 0, -1) : Vector3f.UNIT_Y);
		frame = 0;
	}

	private static float typeWidth(TreeScatter.Type type) {
		return type.widthRatio();
	}

	private Material bakeMaterial(Material source) {
		MaterialDef original = source.getMaterialDef();
		MaterialDef definition = new MaterialDef(assetManager, "Tree surface-data bake");
		for (MatParam parameter : original.getMaterialParams()) {
			if (parameter instanceof MatParamTexture texture)
				definition.addMaterialParamTexture(texture.getVarType(), texture.getName(), texture.getColorSpace(),
						texture.getTextureValue());
			else
				definition.addMaterialParam(parameter.getVarType(), parameter.getName(), parameter.getValue());
		}
		definition.addMaterialParam(VarType.Boolean, "BakeNormals", false);
		for (TechniqueDef originalTechnique : original.getTechniqueDefs(TechniqueDef.DEFAULT_TECHNIQUE_NAME)) {
			try {
				TechniqueDef technique = originalTechnique.clone();
				technique.setShaderFile(originalTechnique.getVertexShaderName(),
						"Shaders/Shoreline/TreeImpostorBake.frag", originalTechnique.getVertexShaderLanguage(),
						originalTechnique.getFragmentShaderLanguage());
				definition.addTechniqueDef(technique);
			} catch (CloneNotSupportedException e) {
				throw new IllegalStateException(e);
			}
		}
		Material result = new Material(definition);
		for (MatParam parameter : source.getParams())
			result.setParam(parameter.getName(), parameter.getVarType(), parameter.getValue());
		result.getAdditionalRenderState().set(source.getAdditionalRenderState());
		return result;
	}

	@Override
	public void simpleRender(RenderManager manager) {
		if (finished || frame++ < 3)
			return;
		renderer.readFrameBufferWithFormat(target, pixels, Image.Format.RGBA8);
		BufferedImage atlas = view == 4 ? (pass == 0 ? topColor : topNormal) : (pass == 0 ? sideColor : sideNormal);
		copyTile(atlas);
		pass++;
		if (pass == 2) {
			pass = 0;
			view++;
		}
		if (view == 5) {
			view = 0;
			variant++;
		}
		TreeKind kind = TreeKind.values()[typeIndex];
		if (variant == library.variantCount(kind)) {
			try {
				writeType(kind);
			} catch (IOException e) {
				throw new IllegalStateException(e);
			}
			variant = 0;
			typeIndex++;
			if (typeIndex == TreeKind.values().length) {
				finished = true;
				System.out.println("Tree impostor surface bake complete: " + output.toAbsolutePath());
				stop();
				return;
			}
			beginType();
		}
		selectFrame();
	}

	private void copyTile(BufferedImage atlas) {
		int height = view == 4 ? TILE_WIDTH : SIDE_HEIGHT;
		int renderWidth = TILE_WIDTH * SUPER_SAMPLE, renderHeight = height * SUPER_SAMPLE;
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < TILE_WIDTH; x++) {
				float red = 0, green = 0, blue = 0, alpha = 0;
				for (int sy = 0; sy < SUPER_SAMPLE; sy++) {
					for (int sx = 0; sx < SUPER_SAMPLE; sx++) {
						int index = ((renderHeight - 1 - (y * SUPER_SAMPLE + sy)) * renderWidth + x * SUPER_SAMPLE + sx)
								* 4;
						float a = (pixels.get(index + 3) & 255) / 255f;
						red += (pixels.get(index) & 255) / 255f * a;
						green += (pixels.get(index + 1) & 255) / 255f * a;
						blue += (pixels.get(index + 2) & 255) / 255f * a;
						alpha += a;
					}
				}
				if (alpha == 0)
					continue;
				red /= alpha;
				green /= alpha;
				blue /= alpha;
				if (pass == 0) {
					red = linearToSrgb(red);
					green = linearToSrgb(green);
					blue = linearToSrgb(blue);
				} else {
					Vector3f normal = new Vector3f(red * 2 - 1, green * 2 - 1, blue * 2 - 1).normalizeLocal();
					Vector3f tangentNormal = switch (view) {
					case 0 -> normal;
					case 1 -> new Vector3f(normal.z, normal.y, -normal.x);
					case 2 -> new Vector3f(-normal.x, normal.y, -normal.z);
					case 3 -> new Vector3f(-normal.z, normal.y, normal.x);
					default -> new Vector3f(normal.x, -normal.z, normal.y);
					};
					red = tangentNormal.x * .5f + .5f;
					green = tangentNormal.y * .5f + .5f;
					blue = tangentNormal.z * .5f + .5f;
				}
				int color = channel(alpha / (SUPER_SAMPLE * SUPER_SAMPLE)) << 24 | channel(red) << 16
						| channel(green) << 8 | channel(blue);
				atlas.setRGB(variant * TILE_WIDTH + x, (view == 4 ? 0 : view * height) + y, color);
			}
		}
	}

	private void writeType(TreeKind kind) throws IOException {
		String prefix = kind.name().toLowerCase(Locale.ROOT);
		write(sideColor, prefix + "-side-albedo.png", TILE_WIDTH, SIDE_HEIGHT);
		write(sideNormal, prefix + "-side-normal.png", TILE_WIDTH, SIDE_HEIGHT);
		write(topColor, prefix + "-top-albedo.png", TILE_WIDTH, TILE_WIDTH);
		write(topNormal, prefix + "-top-normal.png", TILE_WIDTH, TILE_WIDTH);
		System.out.println("Baked " + kind + ": " + library.variantCount(kind) + " model variants, four sides and top");
	}

	private void write(BufferedImage atlas, String name, int tileWidth, int tileHeight) throws IOException {
		// Extend edge colours into transparent texels within each tile, without changing alpha.
		for (int dilation = 0; dilation < 3; dilation++) {
			int[] source = atlas.getRGB(0, 0, atlas.getWidth(), atlas.getHeight(), null, 0, atlas.getWidth());
			for (int y = 0; y < atlas.getHeight(); y++) {
				for (int x = 0; x < atlas.getWidth(); x++) {
					int index = y * atlas.getWidth() + x;
					if (source[index] != 0)
						continue;
					for (int offsetY = -1; offsetY <= 1; offsetY++) {
						for (int offsetX = -1; offsetX <= 1; offsetX++) {
							int nx = x + offsetX, ny = y + offsetY;
							if (nx < 0 || ny < 0 || nx >= atlas.getWidth() || ny >= atlas.getHeight()
									|| nx / tileWidth != x / tileWidth || ny / tileHeight != y / tileHeight)
								continue;
							int color = source[ny * atlas.getWidth() + nx];
							if (color != 0)
								atlas.setRGB(x, y, color & 0xffffff);
						}
					}
				}
			}
		}
		if (!ImageIO.write(atlas, "png", output.resolve(name).toFile()))
			throw new IOException("PNG writer not available");
	}

	private static float linearToSrgb(float linear) {
		return linear <= .0031308f ? linear * 12.92f : 1.055f * (float) Math.pow(linear, 1 / 2.4) - .055f;
	}

	private static int channel(float value) {
		return Math.max(0, Math.min(255, Math.round(value * 255)));
	}

	@Override
	public void destroy() {
		if (target != null)
			renderer.deleteFrameBuffer(target);
		super.destroy();
	}
}
