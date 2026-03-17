package se.hirt.searobots.experiments;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.effect.ParticleEmitter;
import com.jme3.effect.ParticleMesh;
import com.jme3.material.RenderState;
import com.jme3.texture.Image;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;
import com.jme3.util.TangentBinormalGenerator;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeCanvasContext;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

/**
 * Visualises the legacy submarine .obj model in a jMonkeyEngine canvas
 * embedded in a Swing JFrame.
 *
 * Run with: mvn compile exec:java
 * Or just run this main class from your IDE.
 */
public class SubmarineViewer extends SimpleApplication {

    private Node modelNode;
    private Spatial propeller;
    private Spatial torpedoPropeller;
    // Control surfaces: pivoted at their bounding center
    private Spatial elevatorL, elevatorR, rudderU, rudderL;

    public static void main(String[] args) {
        SubmarineViewer app = new SubmarineViewer();

        AppSettings settings = new AppSettings(true);
        settings.setWidth(1280);
        settings.setHeight(800);
        settings.setTitle("SeaRobots - Submarine Model Viewer");
        settings.setFrameRate(60);
        app.setSettings(settings);
        app.setShowSettings(false);
        app.setPauseOnLostFocus(false);

        // Create the jME canvas for Swing embedding
        app.createCanvas();
        JmeCanvasContext ctx = (JmeCanvasContext) app.getContext();
        Canvas canvas = ctx.getCanvas();
        canvas.setPreferredSize(new Dimension(1280, 800));

        // Build Swing frame
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("SeaRobots - Submarine Model Viewer");
            frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
            frame.addWindowListener(new WindowAdapter() {
                @Override
                public void windowClosed(WindowEvent e) {
                    app.stop();
                    System.exit(0);
                }
            });

            JPanel mainPanel = new JPanel(new BorderLayout());
            mainPanel.add(canvas, BorderLayout.CENTER);

            // Info panel at the bottom
            JPanel infoPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
            infoPanel.add(new JLabel(
                    "Models: submarine.obj | test-sub-reduced.obj | torpedo.obj  " +
                    "   [Drag to rotate, scroll to zoom, right-drag to pan]"));
            mainPanel.add(infoPanel, BorderLayout.SOUTH);

            frame.setContentPane(mainPanel);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });

        app.startCanvas();
    }

    @Override
    public void simpleInitApp() {
        // Enable the fly camera for orbit-style navigation
        flyCam.setDragToRotate(true);
        flyCam.setMoveSpeed(50f);

        // Set a dark blue background (underwater feel)
        viewPort.setBackgroundColor(new ColorRGBA(0.02f, 0.04f, 0.12f, 1f));

        modelNode = new Node("models");
        // Convert from 3ds Max Z-up to jME Y-up
        modelNode.rotate(-FastMath.HALF_PI, 0, 0);
        rootNode.attachChild(modelNode);

        // Load the submarine models (spread along the Y axis, which is
        // the model's long axis before the Z-up to Y-up correction)
        Spatial hybrid = loadModel("Models/submarine-hybrid.obj", 0, 0, 0);
        if (hybrid instanceof Node hybridNode) {
            Spatial prop = findChild(hybridNode, "Propeller");
            if (prop != null) {
                propeller = setupPivot(hybridNode, prop, Vector3f.ZERO);
            }
            // Set up control surface pivots
            // Rudder pivot shifted slightly towards bow (-Y in model space)
            Vector3f rudderPivotOffset = new Vector3f(0, -0.3f, 0);
            for (String name : new String[]{"elevatorl", "elevatorr", "rudderu", "rudderl"}) {
                Spatial surface = findChild(hybridNode, name);
                if (surface != null) {
                    boolean isRudder = name.startsWith("rudder");
                    Spatial pivot = setupPivot(hybridNode, surface,
                            isRudder ? rudderPivotOffset : Vector3f.ZERO);
                    switch (name) {
                        case "elevatorl" -> elevatorL = pivot;
                        case "elevatorr" -> elevatorR = pivot;
                        case "rudderu" -> rudderU = pivot;
                        case "rudderl" -> rudderL = pivot;
                    }
                }
            }
        }
        Spatial torp = loadModel("Models/torpedo.obj", 30, 0, 0);
        if (torp instanceof Node torpNode) {
            Spatial prop = findChild(torpNode, "g_Propeller_Plane");
            if (prop != null) {
                torpedoPropeller = setupPivot(torpNode, prop, Vector3f.ZERO);
            }
        }

        // Bubble particle emitter behind the propeller
        ParticleEmitter bubbles = new ParticleEmitter("bubbles", ParticleMesh.Type.Triangle, 300);
        Material bubbleMat = new Material(assetManager, "Common/MatDefs/Misc/Particle.j3md");
        // Create a simple white circle texture programmatically
        bubbleMat.setTexture("Texture", createBubbleTexture());
        bubbleMat.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        bubbles.setMaterial(bubbleMat);
        bubbles.setImagesX(1); bubbles.setImagesY(1);
        bubbles.setStartColor(new ColorRGBA(0.6f, 0.8f, 1.0f, 0.6f));
        bubbles.setEndColor(new ColorRGBA(0.4f, 0.6f, 1.0f, 0.0f));
        bubbles.setStartSize(0.3f);
        bubbles.setEndSize(0.8f);
        bubbles.setLowLife(1.5f);
        bubbles.setHighLife(3.0f);
        bubbles.setParticlesPerSec(60);
        // Emit from just behind the prop, gravity pulls bubbles upward (+Z in model space)
        bubbles.setLocalTranslation(0, 70, 3 - 2.5f);  // behind prop, Z shifted like high-poly parts
        bubbles.getParticleInfluencer().setInitialVelocity(new Vector3f(0, 3f, 0));
        bubbles.getParticleInfluencer().setVelocityVariation(0.4f);
        bubbles.setGravity(0, 0, -4f);  // upward in model space (Z-up)
        bubbles.setInWorldSpace(false);  // particles rotate with the model
        modelNode.attachChild(bubbles);

        // Add grid floor for reference
        addReferenceGrid();

        // Lighting
        DirectionalLight sun = new DirectionalLight();
        sun.setDirection(new Vector3f(-0.5f, -1f, -0.5f).normalizeLocal());
        sun.setColor(ColorRGBA.White.mult(0.8f));
        rootNode.addLight(sun);

        DirectionalLight fill = new DirectionalLight();
        fill.setDirection(new Vector3f(0.5f, 0.3f, 0.5f).normalizeLocal());
        fill.setColor(new ColorRGBA(0.4f, 0.5f, 0.6f, 1f));
        rootNode.addLight(fill);

        AmbientLight ambient = new AmbientLight();
        ambient.setColor(new ColorRGBA(0.2f, 0.25f, 0.3f, 1f));
        rootNode.addLight(ambient);

        // Position camera: elevated, zoomed out to see all three models
        cam.setLocation(new Vector3f(120, 50, 120));
        cam.lookAt(Vector3f.ZERO, Vector3f.UNIT_Y);
    }

    private Spatial loadModel(String path, float x, float y, float z) {
        try {
            Spatial model = assetManager.loadModel(path);
            generateSmoothNormals(model);
            model.setLocalTranslation(x, y, z);
            modelNode.attachChild(model);
            System.out.println("Loaded: " + path + " (" + countGeometries(model) + " geometries)");
            return model;
        } catch (Exception e) {
            System.err.println("Failed to load " + path + ": " + e.getMessage());
            Box box = new Box(5, 2, 10);
            Geometry placeholder = new Geometry(path + "_placeholder", box);
            Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
            mat.setColor("Color", ColorRGBA.Red);
            placeholder.setMaterial(mat);
            placeholder.setLocalTranslation(x, y, z);
            modelNode.attachChild(placeholder);
            return placeholder;
        }
    }

    private Texture2D createBubbleTexture() {
        int size = 32;
        java.nio.ByteBuffer buf = com.jme3.util.BufferUtils.createByteBuffer(size * size * 4);
        float center = size / 2f;
        float radius = size / 2f;
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float dx = x - center, dy = y - center;
                float dist = (float) Math.sqrt(dx * dx + dy * dy);
                float alpha = Math.max(0, 1f - dist / radius);
                alpha = alpha * alpha; // softer falloff
                buf.put((byte) 255).put((byte) 255).put((byte) 255).put((byte) (alpha * 255));
            }
        }
        buf.flip();
        Image img = new Image(Image.Format.RGBA8, size, size, buf, ColorSpace.sRGB);
        return new Texture2D(img);
    }

    private Spatial setupPivot(Node parent, Spatial part, Vector3f pivotOffset) {
        part.getParent().detachChild(part);
        Node pivot = new Node(part.getName() + "Pivot");
        var bound = part.getWorldBound();
        Vector3f center = ((com.jme3.bounding.BoundingBox) bound).getCenter().add(pivotOffset);
        pivot.setLocalTranslation(center);
        part.setLocalTranslation(part.getLocalTranslation().subtract(center));
        pivot.attachChild(part);
        parent.attachChild(pivot);
        System.out.println("Pivot for " + part.getName() + " at: " + center);
        return pivot;
    }

    private Spatial findChild(Node node, String name) {
        for (Spatial child : node.getChildren()) {
            if (name.equals(child.getName())) return child;
            if (child instanceof Node n) {
                Spatial found = findChild(n, name);
                if (found != null) return found;
            }
        }
        return null;
    }

    private void addReferenceGrid() {
        // Simple axis markers
        addAxisLine(Vector3f.UNIT_X.mult(100), ColorRGBA.Red);
        addAxisLine(Vector3f.UNIT_Y.mult(100), ColorRGBA.Green);
        addAxisLine(Vector3f.UNIT_Z.mult(100), ColorRGBA.Blue);
    }

    private void addAxisLine(Vector3f extent, ColorRGBA color) {
        Box box;
        if (extent.x > 0) box = new Box(extent.x, 0.1f, 0.1f);
        else if (extent.y > 0) box = new Box(0.1f, extent.y, 0.1f);
        else box = new Box(0.1f, 0.1f, extent.z);

        Geometry geom = new Geometry("axis", box);
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", color);
        geom.setMaterial(mat);
        rootNode.attachChild(geom);
    }

    private void generateSmoothNormals(Spatial spatial) {
        if (spatial instanceof Geometry geom) {
            Mesh mesh = geom.getMesh();
            if (mesh.getBuffer(com.jme3.scene.VertexBuffer.Type.Normal) == null) {
                var posBuf = mesh.getFloatBuffer(com.jme3.scene.VertexBuffer.Type.Position);
                int vertCount = posBuf.limit() / 3;
                float[] normals = new float[vertCount * 3];

                // Accumulate face normals at each vertex
                var idxBuf = mesh.getIndexBuffer();
                int triCount = idxBuf.size() / 3;
                Vector3f v0 = new Vector3f(), v1 = new Vector3f(), v2 = new Vector3f();
                Vector3f edge1 = new Vector3f(), edge2 = new Vector3f(), faceNormal = new Vector3f();

                for (int t = 0; t < triCount; t++) {
                    int i0 = idxBuf.get(t * 3), i1 = idxBuf.get(t * 3 + 1), i2 = idxBuf.get(t * 3 + 2);
                    v0.set(posBuf.get(i0 * 3), posBuf.get(i0 * 3 + 1), posBuf.get(i0 * 3 + 2));
                    v1.set(posBuf.get(i1 * 3), posBuf.get(i1 * 3 + 1), posBuf.get(i1 * 3 + 2));
                    v2.set(posBuf.get(i2 * 3), posBuf.get(i2 * 3 + 1), posBuf.get(i2 * 3 + 2));
                    v1.subtract(v0, edge1);
                    v2.subtract(v0, edge2);
                    edge1.cross(edge2, faceNormal);
                    // Weight by face area (unnormalized cross product magnitude)
                    for (int idx : new int[]{i0, i1, i2}) {
                        normals[idx * 3]     += faceNormal.x;
                        normals[idx * 3 + 1] += faceNormal.y;
                        normals[idx * 3 + 2] += faceNormal.z;
                    }
                }
                // Normalize
                for (int i = 0; i < vertCount; i++) {
                    float nx = normals[i * 3], ny = normals[i * 3 + 1], nz = normals[i * 3 + 2];
                    float len = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
                    if (len > 0) { normals[i * 3] /= len; normals[i * 3 + 1] /= len; normals[i * 3 + 2] /= len; }
                }
                mesh.setBuffer(com.jme3.scene.VertexBuffer.Type.Normal, 3,
                        com.jme3.util.BufferUtils.createFloatBuffer(normals));
                mesh.updateBound();
            }
        } else if (spatial instanceof Node node) {
            for (Spatial child : node.getChildren()) {
                generateSmoothNormals(child);
            }
        }
    }

    private int countGeometries(Spatial spatial) {
        if (spatial instanceof Geometry) return 1;
        if (spatial instanceof Node) {
            int count = 0;
            for (Spatial child : ((Node) spatial).getChildren()) {
                count += countGeometries(child);
            }
            return count;
        }
        return 0;
    }

    private float angle = 0;

    @Override
    public void simpleUpdate(float tpf) {
        // Turntable rotation around world Y (vertical) axis.
        // Since modelNode has a -90deg X rotation for Z-up conversion,
        // we apply the turntable via a parent node to keep it in world space.
        if (modelNode != null) {
            angle += tpf * 0.3f;
            modelNode.setLocalRotation(
                    new com.jme3.math.Quaternion().fromAngles(-FastMath.HALF_PI, angle, 0));
        }
        // Spin propellers around the sub's long axis (Y in model space)
        if (propeller != null) {
            propeller.rotate(0, tpf * 3f, 0);
        }
        if (torpedoPropeller != null) {
            torpedoPropeller.rotate(0, tpf * 5f, 0);
        }
        // Oscillate control surfaces
        // Elevators: pitch around X axis (horizontal fins deflect up/down)
        float elevatorAngle = FastMath.sin(angle * 2f) * 0.3f;
        if (elevatorL != null) {
            elevatorL.setLocalRotation(
                    new com.jme3.math.Quaternion().fromAngles(elevatorAngle, 0, 0));
        }
        if (elevatorR != null) {
            elevatorR.setLocalRotation(
                    new com.jme3.math.Quaternion().fromAngles(elevatorAngle, 0, 0));
        }
        // Rudders: vertical fins, rotate around Z (vertical axis in model space)
        float rudderAngle = FastMath.sin(angle * 1.3f) * 0.5f;
        if (rudderU != null) {
            rudderU.setLocalRotation(
                    new com.jme3.math.Quaternion().fromAngles(0, 0, rudderAngle));
        }
        if (rudderL != null) {
            rudderL.setLocalRotation(
                    new com.jme3.math.Quaternion().fromAngles(0, 0, rudderAngle));
        }
    }
}
