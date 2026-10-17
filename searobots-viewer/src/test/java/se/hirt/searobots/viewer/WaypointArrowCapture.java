package se.hirt.searobots.viewer;

import com.jme3.app.SimpleApplication;
import com.jme3.font.BitmapText;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.TranslucentBucketFilter;
import com.jme3.renderer.RenderManager;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext;
import com.jme3.util.Screenshots;
import java.awt.image.BufferedImage;
import java.io.File;
import java.nio.ByteBuffer;
import javax.imageio.ImageIO;
import se.hirt.searobots.engine.HullGeometry;

/** Temporary real-OpenGL capture harness; removed after inspecting the output. */
public class WaypointArrowCapture extends SimpleApplication {
	private static final int WIDTH = 1000;
	private static final int HEIGHT = 700;
	private static final String[] NAMES = {"forward", "aft", "starboard", "above", "below-forward", "below-aft",
			"tilted-above", "tactical-far", "tactical-near-aft", "port-occlusion"};
	private final ByteBuffer pixels = ByteBuffer.allocateDirect(WIDTH * HEIGHT * 4);
	private final BufferedImage output = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_4BYTE_ABGR);
	private final Vector3f position = new Vector3f();
	private final Vector3f target = new Vector3f();
	private final Quaternion rotation = new Quaternion();
	private Spatial hull;
	private WaypointDirectionIndicator direction;
	private BitmapText label;
	private int shot;
	private int frames;

	public static void main(String[] args) {
		WaypointArrowCapture app = new WaypointArrowCapture();
		AppSettings settings = new AppSettings(true);
		settings.setResolution(WIDTH, HEIGHT);
		settings.setAudioRenderer(null);
		settings.setRenderer(AppSettings.LWJGL_OPENGL32);
		settings.setSamples(0);
		settings.setFrameRate(30);
		app.setSettings(settings);
		app.setShowSettings(false);
		app.start(JmeContext.Type.OffscreenSurface);
	}

	@Override
	public void simpleInitApp() {
		setDisplayStatView(false);
		setDisplayFps(false);
		flyCam.setEnabled(false);
		viewPort.setBackgroundColor(new ColorRGBA(0.025f, 0.055f, 0.085f, 1));
		AmbientLight ambient = new AmbientLight();
		ambient.setColor(ColorRGBA.White.mult(0.45f));
		rootNode.addLight(ambient);
		DirectionalLight light = new DirectionalLight();
		light.setDirection(new Vector3f(-0.7f, -1, -0.3f).normalizeLocal());
		light.setColor(ColorRGBA.White.mult(0.8f));
		rootNode.addLight(light);
		hull = assetManager.loadModel("models/submarine-hybrid.obj");
		hull.depthFirstTraversal(spatial -> {
			if (spatial instanceof Geometry geometry) {
				Material material = new Material(assetManager, "Common/MatDefs/Light/Lighting.j3md");
				material.setBoolean("UseMaterialColors", true);
				material.setColor("Diffuse", new ColorRGBA(0.30f, 0.38f, 0.43f, 1));
				material.setColor("Ambient", new ColorRGBA(0.30f, 0.38f, 0.43f, 1));
				material.setColor("Specular", new ColorRGBA(0.15f, 0.15f, 0.15f, 1));
				material.setFloat("Shininess", 16);
				material.getAdditionalRenderState().setFaceCullMode(RenderState.FaceCullMode.Off);
				geometry.setMaterial(material);
			}
		});
		rootNode.attachChild(hull);
		direction = new WaypointDirectionIndicator(assetManager, "wpDirection",
				new HullGeometry.Envelope(38, 5.5, 4.5, 0, 0.11), new ColorRGBA(0.45f, 0.93f, 1, 1));
		rootNode.attachChild(direction);
		FilterPostProcessor processor = new FilterPostProcessor(assetManager);
		processor.addFilter(new TranslucentBucketFilter());
		viewPort.addProcessor(processor);
		label = new BitmapText(assetManager.loadFont("Interface/Fonts/Default.fnt"));
		label.setSize(20);
		label.setLocalTranslation(20, HEIGHT - 20, 0);
		guiNode.attachChild(label);
		configureShot();
	}

	private void configureShot() {
		boolean tactical = shot == 7 || shot == 8;
		rotation.fromAngles(-FastMath.HALF_PI, 0, 0);
		if (shot == 6) {
			rotation.fromAngles(-1.2f, 0.4f, 0.2f);
		}
		hull.setLocalRotation(rotation);
		Vector3f localTarget = switch (shot) {
		case 1, 5, 7 -> new Vector3f(0, 1000, 0.11f);
		case 8 -> new Vector3f(0, 100, 0.11f);
		case 9 -> new Vector3f(-1000, 0, 0.11f);
		case 2 -> new Vector3f(1000, 0, 0.11f);
		case 3, 6 -> new Vector3f(0, 0, 1000);
		default -> new Vector3f(0, -1000, 0.11f);
		};
		rotation.mult(localTarget, target).addLocal(position);
		cam.setFrustumPerspective(45, (float) WIDTH / HEIGHT, 0.5f, 30000);
		cam.setLocation(
				tactical ? new Vector3f(0, 7000, 5000) : new Vector3f(85, shot >= 4 && shot <= 5 ? -65 : 70, 95));
		cam.lookAt(position, Vector3f.UNIT_Y);
		cam.update();
		direction.update(position, rotation, target, cam, tactical);
		if (tactical) {
			Spatial arrow = direction.getChild("arrow");
			rootNode.updateGeometricState();
			Vector3f base = arrow.localToWorld(Vector3f.ZERO, new Vector3f());
			Vector3f tip = arrow.localToWorld(Vector3f.UNIT_Z, new Vector3f());
			System.out.println("ARROW_STATE camera=" + cam.getLocation() + " dir=" + cam.getDirection() + " target="
					+ target + " rotation=" + rotation + " cull=" + arrow.getCullHint() + " base="
					+ arrow.getLocalTranslation() + " scale=" + arrow.getLocalScale() + " worldBound="
					+ arrow.getWorldBound() + " screenBase=" + cam.getScreenCoordinates(base) + " screenTip="
					+ cam.getScreenCoordinates(tip));
		}
		label.setText(NAMES[shot] + " - actual submarine mesh, " + (tactical ? "tactical" : "normal depth-tested"));
		frames = 0;
	}

	@Override
	public void simpleRender(RenderManager manager) {
		if (++frames != 12) {
			return;
		}
		try {
			pixels.clear();
			renderer.readFrameBufferWithFormat(null, pixels, com.jme3.texture.Image.Format.BGRA8);
			Screenshots.convertScreenShot(pixels, output);
			File file = new File("searobots-viewer/target/waypoint-arrow-" + NAMES[shot] + ".png");
			ImageIO.write(output, "png", file);
			System.out.println("ARROW_CAPTURE " + file.getAbsolutePath());
			if (++shot == NAMES.length) {
				stop();
			} else {
				configureShot();
			}
		} catch (Exception failure) {
			failure.printStackTrace();
			stop();
		}
	}
}
