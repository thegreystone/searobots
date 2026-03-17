/*
 * Copyright (C) 2026 Marcus Hirt
 *                    (see TerrainViewer.java for full license text)
 */
package se.hirt.searobots.viewer;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.water.WaterFilter;

import javax.swing.*;
import javax.swing.event.ChangeListener;
import java.awt.*;

/**
 * Non-modal dialog for tuning WaterFilter parameters in real time.
 */
final class WaterSettingsDialog extends JDialog {

    WaterSettingsDialog(Frame owner, WaterFilter wf) {
        super(owner, "Water Settings", false);
        setLayout(new BorderLayout());

        var panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        // Water Color
        panel.add(label("-- Water Color --"));
        var wcR = slider("Red", 0, 100, (int) (wf.getWaterColor().r * 100), v -> {
            var c = wf.getWaterColor(); wf.setWaterColor(new ColorRGBA(v / 100f, c.g, c.b, 1f));
        });
        var wcG = slider("Green", 0, 100, (int) (wf.getWaterColor().g * 100), v -> {
            var c = wf.getWaterColor(); wf.setWaterColor(new ColorRGBA(c.r, v / 100f, c.b, 1f));
        });
        var wcB = slider("Blue", 0, 100, (int) (wf.getWaterColor().b * 100), v -> {
            var c = wf.getWaterColor(); wf.setWaterColor(new ColorRGBA(c.r, c.g, v / 100f, 1f));
        });
        panel.add(wcR); panel.add(wcG); panel.add(wcB);

        // Deep Water Color
        panel.add(label("-- Deep Water Color --"));
        var dwR = slider("Red", 0, 100, (int) (wf.getDeepWaterColor().r * 100), v -> {
            var c = wf.getDeepWaterColor(); wf.setDeepWaterColor(new ColorRGBA(v / 100f, c.g, c.b, 1f));
        });
        var dwG = slider("Green", 0, 100, (int) (wf.getDeepWaterColor().g * 100), v -> {
            var c = wf.getDeepWaterColor(); wf.setDeepWaterColor(new ColorRGBA(c.r, v / 100f, c.b, 1f));
        });
        var dwB = slider("Blue", 0, 100, (int) (wf.getDeepWaterColor().b * 100), v -> {
            var c = wf.getDeepWaterColor(); wf.setDeepWaterColor(new ColorRGBA(c.r, c.g, v / 100f, 1f));
        });
        panel.add(dwR); panel.add(dwG); panel.add(dwB);

        // Key parameters
        panel.add(label("-- Water Properties --"));
        panel.add(slider("Transparency", 1, 50, (int) (wf.getWaterTransparency() * 100), v ->
                wf.setWaterTransparency(v / 100f)));
        panel.add(slider("Wave Scale", 1, 100, (int) (wf.getWaveScale() * 10000), v ->
                wf.setWaveScale(v / 10000f)));
        panel.add(slider("Normal Scale", 1, 300, (int) (wf.getNormalScale() * 10), v ->
                wf.setNormalScale(v / 10f)));
        panel.add(slider("Speed", 1, 30, (int) (wf.getSpeed() * 10), v ->
                wf.setSpeed(v / 10f)));

        // Sun/Light
        panel.add(label("-- Sun/Light --"));
        panel.add(slider("Sun Scale", 1, 100, (int) (wf.getSunScale() * 10), v ->
                wf.setSunScale(v / 10f)));
        panel.add(slider("Shininess", 1, 200, (int) wf.getShininess(), v ->
                wf.setShininess(v)));

        // Caustics
        panel.add(label("-- Caustics --"));
        panel.add(slider("Intensity", 0, 100, (int) (wf.getCausticsIntensity() * 100), v ->
                wf.setCausticsIntensity(v / 100f)));

        // Underwater
        panel.add(label("-- Underwater --"));
        panel.add(slider("Fog Distance", 50, 2000, (int) wf.getUnderWaterFogDistance(), v ->
                wf.setUnderWaterFogDistance(v)));

        // Color Extinction
        panel.add(label("-- Color Extinction (R/G/B depth) --"));
        var ce = wf.getColorExtinction();
        panel.add(slider("Red extinction", 1, 100, (int) ce.x, v -> {
            var cur = wf.getColorExtinction(); wf.setColorExtinction(new Vector3f(v, cur.y, cur.z));
        }));
        panel.add(slider("Green extinction", 1, 100, (int) ce.y, v -> {
            var cur = wf.getColorExtinction(); wf.setColorExtinction(new Vector3f(cur.x, v, cur.z));
        }));
        panel.add(slider("Blue extinction", 1, 100, (int) ce.z, v -> {
            var cur = wf.getColorExtinction(); wf.setColorExtinction(new Vector3f(cur.x, cur.y, v));
        }));

        // Print button
        var printBtn = new JButton("Print current values");
        printBtn.setAlignmentX(Component.LEFT_ALIGNMENT);
        printBtn.addActionListener(e -> {
            var wc = wf.getWaterColor();
            var dw = wf.getDeepWaterColor();
            var ext = wf.getColorExtinction();
            System.out.println("=== Water Settings ===");
            System.out.printf("waterColor: (%.3f, %.3f, %.3f)%n", wc.r, wc.g, wc.b);
            System.out.printf("deepWaterColor: (%.3f, %.3f, %.3f)%n", dw.r, dw.g, dw.b);
            System.out.printf("waterTransparency: %.3f%n", wf.getWaterTransparency());
            System.out.printf("waveScale: %.4f%n", wf.getWaveScale());
            System.out.printf("normalScale: %.1f%n", wf.getNormalScale());
            System.out.printf("speed: %.1f%n", wf.getSpeed());
            System.out.printf("sunScale: %.1f%n", wf.getSunScale());
            System.out.printf("shininess: %.0f%n", wf.getShininess());
            System.out.printf("causticsIntensity: %.2f%n", wf.getCausticsIntensity());
            System.out.printf("underWaterFogDistance: %.0f%n", wf.getUnderWaterFogDistance());
            System.out.printf("colorExtinction: (%.0f, %.0f, %.0f)%n", ext.x, ext.y, ext.z);
        });
        panel.add(Box.createVerticalStrut(10));
        panel.add(printBtn);

        var scroll = new JScrollPane(panel);
        scroll.setPreferredSize(new Dimension(380, 600));
        add(scroll, BorderLayout.CENTER);

        pack();
        setLocationRelativeTo(owner);
    }

    private static JLabel label(String text) {
        var l = new JLabel(text);
        l.setAlignmentX(Component.LEFT_ALIGNMENT);
        l.setBorder(BorderFactory.createEmptyBorder(8, 0, 2, 0));
        return l;
    }

    private static JPanel slider(String name, int min, int max, int initial,
                                  java.util.function.IntConsumer onChange) {
        var p = new JPanel(new BorderLayout(5, 0));
        p.setAlignmentX(Component.LEFT_ALIGNMENT);
        p.setMaximumSize(new Dimension(Integer.MAX_VALUE, 40));
        var lbl = new JLabel(String.format("%-20s %d", name, initial));
        lbl.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        lbl.setPreferredSize(new Dimension(180, 20));
        var s = new JSlider(min, max, Math.max(min, Math.min(max, initial)));
        s.addChangeListener(e -> {
            int v = s.getValue();
            lbl.setText(String.format("%-20s %d", name, v));
            onChange.accept(v);
        });
        p.add(lbl, BorderLayout.WEST);
        p.add(s, BorderLayout.CENTER);
        return p;
    }
}
