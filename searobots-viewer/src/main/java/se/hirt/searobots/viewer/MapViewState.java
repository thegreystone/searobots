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

import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.BaseAppState;
import com.jme3.material.Material;
import com.jme3.material.RenderState;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Quad;
import com.jme3.texture.Image;
import com.jme3.texture.Texture2D;
import com.jme3.texture.image.ColorSpace;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.nio.ByteBuffer;

/**
 * AppState that renders the 2D tactical map as a fullscreen textured quad
 * overlaid on the 3D scene. Toggled with M key, with a cross-fade transition.
 */
final class MapViewState extends BaseAppState {

    private static final float FADE_DURATION = 0.4f;

    private final MapRenderer mapRenderer;
    private Geometry mapQuad;
    private Material mapMaterial;
    private Texture2D mapTexture;

    // Fade state
    private float fadeProgress = 0f;  // 0 = invisible, 1 = fully opaque
    private boolean fadingIn = false;
    private boolean fadingOut = false;
    private boolean mapVisible = false;

    // Throttle: render map at ~30fps, not every frame
    private float renderTimer = 0f;
    private static final float RENDER_INTERVAL = 1f / 30f;

    // Mouse pan state
    private boolean dragging = false;
    private float dragStartX, dragStartY;
    private double dragViewX, dragViewY;

    // Animated zoom: lerp from current to target
    private double targetPixelsPerMeter;
    private double targetViewX, targetViewY;
    private boolean zooming = false;
    private static final float ZOOM_LERP_SPEED = 12f; // higher = snappier

    MapViewState(MapRenderer mapRenderer) {
        this.mapRenderer = mapRenderer;
    }

    @Override
    protected void initialize(Application app) {
        var sa = (SimpleApplication) app;
        int w = app.getCamera().getWidth();
        int h = app.getCamera().getHeight();

        // Create fullscreen quad in the Gui bucket
        mapQuad = new Geometry("MapQuad", new Quad(w, h));
        mapMaterial = new Material(app.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        mapMaterial.getAdditionalRenderState().setBlendMode(RenderState.BlendMode.Alpha);
        mapMaterial.setColor("Color", new ColorRGBA(1, 1, 1, 0)); // start invisible
        mapQuad.setMaterial(mapMaterial);
        mapQuad.setQueueBucket(RenderQueue.Bucket.Gui);
        mapQuad.setLocalTranslation(0, 0, 5); // above 3D, below Lemur GUI (z=10)

        // Create texture (will be updated each render)
        mapTexture = new Texture2D();
        mapTexture.setMinFilter(com.jme3.texture.Texture.MinFilter.BilinearNoMipMaps);
        mapTexture.setMagFilter(com.jme3.texture.Texture.MagFilter.Bilinear);
    }

    @Override
    protected void onEnable() {
        var sa = (SimpleApplication) getApplication();
        sa.getGuiNode().attachChild(mapQuad);
    }

    @Override
    protected void onDisable() {
        mapQuad.removeFromParent();
        mapVisible = false;
        fadeProgress = 0f;
        fadingIn = false;
        fadingOut = false;
        mapMaterial.setColor("Color", new ColorRGBA(1, 1, 1, 0));
    }

    /** Toggle the map view with cross-fade. */
    void toggle() {
        if (fadingIn) {
            // Reverse: start fading out
            fadingIn = false;
            fadingOut = true;
        } else if (fadingOut) {
            // Reverse: start fading in
            fadingOut = false;
            fadingIn = true;
        } else if (mapVisible) {
            fadingOut = true;
        } else {
            fadingIn = true;
            mapVisible = true;
        }
    }

    boolean isMapVisible() {
        return mapVisible || fadingIn;
    }

    @Override
    public void update(float tpf) {
        // Handle fade transitions
        if (fadingIn) {
            fadeProgress = Math.min(1f, fadeProgress + tpf / FADE_DURATION);
            if (fadeProgress >= 1f) {
                fadingIn = false;
                mapVisible = true;
            }
        } else if (fadingOut) {
            fadeProgress = Math.max(0f, fadeProgress - tpf / FADE_DURATION);
            if (fadeProgress <= 0f) {
                fadingOut = false;
                mapVisible = false;
            }
        }

        mapMaterial.setColor("Color", new ColorRGBA(1, 1, 1, fadeProgress));

        // Animate zoom: exponential lerp toward target
        if (zooming) {
            double t = 1.0 - Math.exp(-ZOOM_LERP_SPEED * tpf);
            double oldPpm = mapRenderer.pixelsPerMeter;
            mapRenderer.pixelsPerMeter += (targetPixelsPerMeter - mapRenderer.pixelsPerMeter) * t;
            mapRenderer.viewX += (targetViewX - mapRenderer.viewX) * t;
            mapRenderer.viewY += (targetViewY - mapRenderer.viewY) * t;
            // Stop when close enough
            if (Math.abs(mapRenderer.pixelsPerMeter - targetPixelsPerMeter) < 0.0001) {
                mapRenderer.pixelsPerMeter = targetPixelsPerMeter;
                mapRenderer.viewX = targetViewX;
                mapRenderer.viewY = targetViewY;
                zooming = false;
            }
        }

        // Only render map when visible or transitioning
        if (fadeProgress <= 0f) return;

        // Throttle rendering
        renderTimer += tpf;
        if (renderTimer < RENDER_INTERVAL) return;
        renderTimer = 0f;

        int w = getApplication().getCamera().getWidth();
        int h = getApplication().getCamera().getHeight();

        // Resize quad if window size changed
        var quadMesh = (Quad) mapQuad.getMesh();
        // Quad doesn't expose width/height, so just recreate if needed
        mapQuad.setMesh(new Quad(w, h));

        // Render the 2D map to a BufferedImage
        BufferedImage img = mapRenderer.render(w, h);
        if (img == null) return;

        // Upload pixel data to JME texture
        uploadTexture(img, w, h);
        mapMaterial.setTexture("ColorMap", mapTexture);
    }

    private void uploadTexture(BufferedImage img, int w, int h) {
        // Get pixel data from BufferedImage (ARGB int array)
        int[] pixels = ((DataBufferInt) img.getRaster().getDataBuffer()).getData();

        // Convert ARGB to RGBA byte buffer
        ByteBuffer buf = ByteBuffer.allocateDirect(w * h * 4);
        for (int y = 0; y < h; y++) {
            // Flip vertically: JME textures have origin at bottom-left
            int srcRow = (h - 1 - y) * w;
            for (int x = 0; x < w; x++) {
                int argb = pixels[srcRow + x];
                buf.put((byte) ((argb >> 16) & 0xFF)); // R
                buf.put((byte) ((argb >> 8) & 0xFF));  // G
                buf.put((byte) (argb & 0xFF));          // B
                buf.put((byte) ((argb >> 24) & 0xFF)); // A
            }
        }
        buf.flip();

        Image jmeImage = new Image(Image.Format.RGBA8, w, h, buf, ColorSpace.sRGB);
        mapTexture.setImage(jmeImage);
    }

    /** Pan the map view by screen-space pixels. */
    void pan(float dx, float dy) {
        if (mapVisible || fadingIn) {
            mapRenderer.viewX += dx / mapRenderer.pixelsPerMeter;
            mapRenderer.viewY -= dy / mapRenderer.pixelsPerMeter;
        }
    }

    /** Zoom the map view centered on the given screen position, with smooth animation. */
    void zoomAt(double factor, float screenX, float screenY) {
        if (!(mapVisible || fadingIn)) return;
        int w = getApplication().getCamera().getWidth();
        int h = getApplication().getCamera().getHeight();

        // Use current target if already animating, otherwise start from current
        double basePpm = zooming ? targetPixelsPerMeter : mapRenderer.pixelsPerMeter;
        double baseVX = zooming ? targetViewX : mapRenderer.viewX;
        double baseVY = zooming ? targetViewY : mapRenderer.viewY;

        // Convert screen position to world coordinates at the base zoom
        double worldX = (screenX - w / 2.0) / basePpm + baseVX;
        double worldY = (screenY - h / 2.0) / basePpm + baseVY;

        // Compute new zoom level
        double newPpm = basePpm * factor;
        newPpm = Math.max(0.01, Math.min(10.0, newPpm));

        // Compute new view center so the world point under cursor stays put
        targetPixelsPerMeter = newPpm;
        targetViewX = worldX - (screenX - w / 2.0) / newPpm;
        targetViewY = worldY - (screenY - h / 2.0) / newPpm;
        zooming = true;
    }

    @Override
    protected void cleanup(Application app) {
    }
}
