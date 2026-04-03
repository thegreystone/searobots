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
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.water.WaterFilter;
import com.simsilica.lemur.*;
import com.simsilica.lemur.component.SpringGridLayout;

import java.util.function.IntConsumer;

/**
 * Lemur-based render settings panel. Includes atmosphere/effects toggles
 * and water filter parameter sliders.
 */
final class WaterSettingsState extends BaseAppState {

    private Container window;
    private WaterFilter waterFilter;
    private SubmarineScene3D scene;

    // Track slider refs for polling in update()
    private record SliderBinding(com.simsilica.lemur.core.VersionedReference<Double> ref,
                                 Slider slider, Label label, String name, IntConsumer onChange) {}
    private final java.util.List<SliderBinding> sliderBindings = new java.util.ArrayList<>();

    void setWaterFilter(WaterFilter wf) {
        this.waterFilter = wf;
    }

    void setScene(SubmarineScene3D scene) {
        this.scene = scene;
    }

    @Override
    protected void initialize(Application app) {
    }

    @Override
    protected void onEnable() {
        if (waterFilter == null) {
            setEnabled(false);
            return;
        }
        var app = (SimpleApplication) getApplication();

        window = new Container(new SpringGridLayout(Axis.Y, Axis.X, FillMode.Even, FillMode.Last));
        window.addChild(new Label("Render Settings"));
        window.addChild(new Label("")); // spacer

        // Render toggles
        window.addChild(new Label("-- Effects --"));
        if (scene != null) {
            addToggle(window, "Atmosphere", scene.isAtmosphereEnabled(),
                    v -> scene.setAtmosphereEnabled(v));
            addToggle(window, "God Rays", scene.isGodRaysEnabled(),
                    v -> scene.setGodRaysEnabled(v));
            addToggle(window, "Debug Mode (no effects)", scene.isDebugMode(),
                    v -> scene.setDebugMode(v));
        }

        window.addChild(new Label("")); // spacer

        // Water Color
        window.addChild(new Label("-- Water Color --"));
        addSlider(window, "Red", 0, 100, (int) (waterFilter.getWaterColor().r * 100), v -> {
            var c = waterFilter.getWaterColor(); waterFilter.setWaterColor(new ColorRGBA(v / 100f, c.g, c.b, 1f));
        });
        addSlider(window, "Green", 0, 100, (int) (waterFilter.getWaterColor().g * 100), v -> {
            var c = waterFilter.getWaterColor(); waterFilter.setWaterColor(new ColorRGBA(c.r, v / 100f, c.b, 1f));
        });
        addSlider(window, "Blue", 0, 100, (int) (waterFilter.getWaterColor().b * 100), v -> {
            var c = waterFilter.getWaterColor(); waterFilter.setWaterColor(new ColorRGBA(c.r, c.g, v / 100f, 1f));
        });

        // Deep Water Color
        window.addChild(new Label("-- Deep Water Color --"));
        addSlider(window, "Red", 0, 100, (int) (waterFilter.getDeepWaterColor().r * 100), v -> {
            var c = waterFilter.getDeepWaterColor(); waterFilter.setDeepWaterColor(new ColorRGBA(v / 100f, c.g, c.b, 1f));
        });
        addSlider(window, "Green", 0, 100, (int) (waterFilter.getDeepWaterColor().g * 100), v -> {
            var c = waterFilter.getDeepWaterColor(); waterFilter.setDeepWaterColor(new ColorRGBA(c.r, v / 100f, c.b, 1f));
        });
        addSlider(window, "Blue", 0, 100, (int) (waterFilter.getDeepWaterColor().b * 100), v -> {
            var c = waterFilter.getDeepWaterColor(); waterFilter.setDeepWaterColor(new ColorRGBA(c.r, c.g, v / 100f, 1f));
        });

        // Water properties
        window.addChild(new Label("-- Water Properties --"));
        addSlider(window, "Transparency", 1, 50, (int) (waterFilter.getWaterTransparency() * 100), v ->
                waterFilter.setWaterTransparency(v / 100f));
        addSlider(window, "Wave Scale", 1, 100, (int) (waterFilter.getWaveScale() * 10000), v ->
                waterFilter.setWaveScale(v / 10000f));
        addSlider(window, "Normal Scale", 1, 300, (int) (waterFilter.getNormalScale() * 10), v ->
                waterFilter.setNormalScale(v / 10f));
        addSlider(window, "Speed", 1, 30, (int) (waterFilter.getSpeed() * 10), v ->
                waterFilter.setSpeed(v / 10f));

        // Sun/Light
        window.addChild(new Label("-- Sun/Light --"));
        addSlider(window, "Sun Scale", 1, 100, (int) (waterFilter.getSunScale() * 10), v ->
                waterFilter.setSunScale(v / 10f));
        addSlider(window, "Shininess", 1, 200, (int) waterFilter.getShininess(), v ->
                waterFilter.setShininess(v));

        // Caustics
        window.addChild(new Label("-- Caustics --"));
        addSlider(window, "Intensity", 0, 100, (int) (waterFilter.getCausticsIntensity() * 100), v ->
                waterFilter.setCausticsIntensity(v / 100f));

        // Underwater
        window.addChild(new Label("-- Underwater --"));
        addSlider(window, "Fog Distance", 50, 2000, (int) waterFilter.getUnderWaterFogDistance(), v ->
                waterFilter.setUnderWaterFogDistance(v));

        // Color Extinction
        window.addChild(new Label("-- Color Extinction --"));
        var ce = waterFilter.getColorExtinction();
        addSlider(window, "Red depth", 1, 100, (int) ce.x, v -> {
            var cur = waterFilter.getColorExtinction(); waterFilter.setColorExtinction(new Vector3f(v, cur.y, cur.z));
        });
        addSlider(window, "Green depth", 1, 100, (int) ce.y, v -> {
            var cur = waterFilter.getColorExtinction(); waterFilter.setColorExtinction(new Vector3f(cur.x, v, cur.z));
        });
        addSlider(window, "Blue depth", 1, 100, (int) ce.z, v -> {
            var cur = waterFilter.getColorExtinction(); waterFilter.setColorExtinction(new Vector3f(cur.x, cur.y, v));
        });

        // Print button
        window.addChild(new Label("")); // spacer
        var printBtn = window.addChild(new Button("Print values to console"));
        printBtn.addClickCommands(b -> {
            var wc = waterFilter.getWaterColor();
            var dw = waterFilter.getDeepWaterColor();
            var ext = waterFilter.getColorExtinction();
            System.out.println("=== Water Settings ===");
            System.out.printf("waterColor: (%.3f, %.3f, %.3f)%n", wc.r, wc.g, wc.b);
            System.out.printf("deepWaterColor: (%.3f, %.3f, %.3f)%n", dw.r, dw.g, dw.b);
            System.out.printf("waterTransparency: %.3f%n", waterFilter.getWaterTransparency());
            System.out.printf("waveScale: %.4f%n", waterFilter.getWaveScale());
            System.out.printf("normalScale: %.1f%n", waterFilter.getNormalScale());
            System.out.printf("speed: %.1f%n", waterFilter.getSpeed());
            System.out.printf("sunScale: %.1f%n", waterFilter.getSunScale());
            System.out.printf("shininess: %.0f%n", waterFilter.getShininess());
            System.out.printf("causticsIntensity: %.2f%n", waterFilter.getCausticsIntensity());
            System.out.printf("underWaterFogDistance: %.0f%n", waterFilter.getUnderWaterFogDistance());
            System.out.printf("colorExtinction: (%.0f, %.0f, %.0f)%n", ext.x, ext.y, ext.z);
        });

        // Close button
        var closeBtn = window.addChild(new Button("Close"));
        closeBtn.addClickCommands(b -> setEnabled(false));

        // Center on screen
        window.setLocalTranslation(
                app.getCamera().getWidth() * 0.3f,
                app.getCamera().getHeight() * 0.85f,
                10);

        app.getGuiNode().attachChild(window);
    }

    @Override
    public void update(float tpf) {
        // Poll slider references for changes
        for (var binding : sliderBindings) {
            if (binding.ref.update()) {
                int v = (int) binding.slider.getModel().getValue();
                binding.label.setText(String.format("%-16s %d", binding.name, v));
                binding.onChange.accept(v);
            }
        }
    }

    private void addToggle(Container parent, String label, boolean initial,
                           java.util.function.Consumer<Boolean> onChange) {
        var btn = parent.addChild(new Button((initial ? "[x] " : "[ ] ") + label));
        final boolean[] state = {initial};
        btn.addClickCommands(b -> {
            state[0] = !state[0];
            btn.setText((state[0] ? "[x] " : "[ ] ") + label);
            onChange.accept(state[0]);
        });
    }

    private void addSlider(Container parent, String name, int min, int max, int initial, IntConsumer onChange) {
        var row = parent.addChild(new Container(new SpringGridLayout(Axis.X, Axis.Y, FillMode.None, FillMode.Last)));
        var label = row.addChild(new Label(String.format("%-16s %d", name, initial)));
        label.setPreferredSize(new com.jme3.math.Vector3f(160, 20, 0));

        var slider = row.addChild(new Slider(Axis.X));
        slider.setPreferredSize(new com.jme3.math.Vector3f(300, 20, 0));
        slider.getModel().setMinimum(min);
        slider.getModel().setMaximum(max);
        slider.getModel().setValue(Math.max(min, Math.min(max, initial)));
        var ref = slider.getModel().createReference();
        sliderBindings.add(new SliderBinding(ref, slider, label, name, onChange));
    }

    @Override
    protected void onDisable() {
        sliderBindings.clear();
        if (window != null) {
            window.removeFromParent();
            window = null;
        }
    }

    @Override
    protected void cleanup(Application app) {
    }
}
