/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * Generates all custom textures for the SeaRobots viewer:
 * - Tree billboard textures (tree1..tree4.png)
 * - Vegetation ground texture (vegetation.png + normal map)
 * - Deep ocean sediment texture (deep_sediment.png)
 *
 * Run directly to regenerate:
 *   java TextureGenerator.java
 *
 * Output directory: searobots-viewer/src/main/resources/Textures/Terrain/custom/
 */
package se.hirt.searobots.viewer.tools;

import java.awt.*;
import java.awt.image.*;
import javax.imageio.*;
import java.io.*;
import java.util.*;

public class TextureGenerator {
    static final String DIR = "searobots-viewer/src/main/resources/Textures/Terrain/custom/";

    public static void main(String[] args) throws Exception {
        new File(DIR).mkdirs();
        genBroadleaf("tree1.png", 256, 360, new Random(101), 16);
        genBroadleaf("tree2.png", 256, 320, new Random(505), 14);
        genPalm();
        genCompact();
        genVegetation();
        genDeepSediment();
        System.out.println("All custom textures generated in " + DIR);
    }

    // ── Trees ──────────────────────────────────────────────────────────

    static void genBroadleaf(String file, int w, int h, Random rng, int trunkBase) throws Exception {
        var img = clear(w, h); var g = g(img);
        int cx = w / 2, baseY = h;
        // Trunk
        for (int i = 0; i < (int)(h * 0.4); i++) {
            float t = (float) i / (h * 0.4f);
            int tw = (int)(trunkBase - t * (trunkBase - 5));
            int x = cx + (int)(Math.sin(t * 1.5 + (file.contains("2") ? 0.5 : 0)) * 3);
            g.setColor(new Color(50 + rng.nextInt(12), 36 + rng.nextInt(8), 20 + rng.nextInt(5)));
            g.fillRect(x - tw / 2, baseY - i, tw, 2);
        }
        // Branch forks
        int forkY = baseY - (int)(h * 0.38);
        drawBranch(g, rng, cx, forkY, cx - 40, forkY - 60, 6, 2);
        drawBranch(g, rng, cx, forkY, cx + 35, forkY - 70, 6, 2);
        drawBranch(g, rng, cx, forkY, cx - 10, forkY - 80, 5, 2);
        // Dense canopy
        int ccy = baseY - (int)(h * 0.58);
        int crx = (int)(w * 0.38), cry = (int)(h * 0.24);
        leaves(g, rng, cx, ccy + 12, crx, cry, 1200, 12, 30, 255);
        leaves(g, rng, cx, ccy, crx, cry, 1600, 25, 55, 255);
        leaves(g, rng, cx, ccy - 10, (int)(crx * 0.9), (int)(cry * 0.85), 800, 42, 90, 255);
        binarizeAlpha(img);
        g.dispose();
        ImageIO.write(img, "PNG", new File(DIR + file));
        System.out.println("  " + file);
    }

    static void genPalm() throws Exception {
        int w = 256, h = 400;
        var img = clear(w, h); var g = g(img); var rng = new Random(303);
        int cx = w / 2;
        // Trunk
        int segs = 30; float sway = 16;
        int[] tx = new int[segs + 1], ty = new int[segs + 1];
        for (int i = 0; i <= segs; i++) {
            float t = (float) i / segs;
            tx[i] = cx + (int)(Math.sin(t * 1.3) * sway * t);
            ty[i] = h - (int)(t * 275);
        }
        for (int i = 0; i < segs; i++) {
            float t = (float) i / segs;
            int thick = (int)(16 - t * 9);
            g.setColor(new Color(78 + (i % 3) * 10 + (int)(t * 15),
                    55 + (i % 3) * 7 + (int)(t * 10), 30 + (int)(t * 8)));
            g.setStroke(new BasicStroke(thick, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g.drawLine(tx[i], ty[i], tx[i + 1], ty[i + 1]);
        }
        int crX = tx[segs], crY = ty[segs];
        // Dense fronds with leaflets
        for (int f = 0; f < 18; f++) {
            double ba = f * Math.PI * 2 / 18 + (rng.nextDouble() - 0.5) * 0.2;
            int fLen = 55 + rng.nextInt(30);
            int steps = 18;
            int[] fx = new int[steps + 1], fy = new int[steps + 1];
            for (int s = 0; s <= steps; s++) {
                float st = (float) s / steps;
                fx[s] = crX + (int)(Math.cos(ba) * fLen * st);
                fy[s] = crY + (int)(Math.sin(ba) * fLen * st * 0.22) + (int)(st * st * 50);
            }
            g.setColor(new Color(45, 72, 22)); g.setStroke(new BasicStroke(2.5f));
            for (int s = 0; s < steps; s++) g.drawLine(fx[s], fy[s], fx[s + 1], fy[s + 1]);
            for (int s = 1; s <= steps; s++) {
                float st = (float) s / steps;
                double dx = fx[s] - fx[Math.max(0, s - 1)], dy = fy[s] - fy[Math.max(0, s - 1)];
                double len = Math.sqrt(dx * dx + dy * dy); if (len < 0.5) continue;
                double px = -dy / len, py = dx / len;
                int leafLen = (int)(22 * (1 - st * 0.3));
                for (int side = -1; side <= 1; side += 2) {
                    int tipX = fx[s] + (int)(px * leafLen * side);
                    int tipY = fy[s] + (int)(py * leafLen * side) + (int)(st * 4);
                    g.setColor(new Color(8 + rng.nextInt(15), 55 + rng.nextInt(65), 4 + rng.nextInt(8)));
                    int midX = (fx[s] + tipX) / 2, midY = (fy[s] + tipY) / 2;
                    int lw = 3;
                    g.fillPolygon(
                        new int[]{fx[s], midX + (int)(py * lw * side), tipX, midX - (int)(py * lw * side)},
                        new int[]{fy[s], midY - (int)(px * lw), tipY, midY + (int)(px * lw)}, 4);
                }
            }
        }
        for (int i = 0; i < 5; i++) {
            g.setColor(new Color(80 + rng.nextInt(18), 60 + rng.nextInt(12), 28 + rng.nextInt(10)));
            g.fillOval(crX - 5 + rng.nextInt(10), crY + 1 + rng.nextInt(10), 9, 10);
        }
        binarizeAlpha(img);
        g.dispose();
        ImageIO.write(img, "PNG", new File(DIR + "tree3.png"));
        System.out.println("  tree3.png (palm)");
    }

    static void genCompact() throws Exception {
        int w = 200, h = 220;
        var img = clear(w, h); var g = g(img); var rng = new Random(404);
        int cx = w / 2, baseY = h;
        drawBranch(g, rng, cx - 10, baseY, cx - 18, baseY - 70, 9, 4);
        drawBranch(g, rng, cx + 6, baseY, cx + 12, baseY - 65, 8, 4);
        drawBranch(g, rng, cx - 18, baseY - 70, cx - 35, baseY - 110, 4, 2);
        drawBranch(g, rng, cx + 12, baseY - 65, cx + 38, baseY - 105, 4, 2);
        int ccy = baseY - 95;
        leaves(g, rng, cx, ccy + 8, 72, 46, 900, 14, 32, 255);
        leaves(g, rng, cx, ccy, 78, 52, 1100, 24, 52, 255);
        leaves(g, rng, cx, ccy - 8, 62, 38, 500, 40, 85, 255);
        binarizeAlpha(img);
        g.dispose();
        ImageIO.write(img, "PNG", new File(DIR + "tree4.png"));
        System.out.println("  tree4.png (compact)");
    }

    // ── Terrain textures ───────────────────────────────────────────────

    static void genVegetation() throws Exception {
        int size = 512;
        var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
        float[][] n1 = valueNoise(size, 8);
        float[][] n2 = valueNoise(size, 32);
        float[][] n3 = valueNoise(size, 64);
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float n = n1[y][x] * 0.5f + n2[y][x] * 0.3f + n3[y][x] * 0.2f;
                int r = clamp((int)((0.08f + n * 0.12f) * 255));
                int g = clamp((int)((0.25f + n * 0.25f) * 255));
                int b = clamp((int)((0.05f + n * 0.08f) * 255));
                img.setRGB(x, y, (r << 16) | (g << 8) | b);
            }
        }
        ImageIO.write(img, "PNG", new File(DIR + "vegetation.png"));

        // Normal map from bump
        var nimg = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
        float[][] bump = valueNoise(size, 48);
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                int xp = (x + 1) % size, xm = (x - 1 + size) % size;
                int yp = (y + 1) % size, ym = (y - 1 + size) % size;
                float dx = (bump[y][xp] - bump[y][xm]) * 2.0f;
                float dy = (bump[yp][x] - bump[ym][x]) * 2.0f;
                int nr = clamp((int)((0.5f - dx * 0.5f) * 255));
                int ng = clamp((int)((0.5f - dy * 0.5f) * 255));
                nimg.setRGB(x, y, (nr << 16) | (ng << 8) | 255);
            }
        }
        ImageIO.write(nimg, "PNG", new File(DIR + "vegetation_normal.png"));
        System.out.println("  vegetation.png + normal");
    }

    static void genDeepSediment() throws Exception {
        int size = 512;
        float[][] n1 = valueNoise(size, 10);
        float[][] n2 = valueNoise(size, 40);
        float[][] n3 = valueNoise(size, 80);
        var img = new BufferedImage(size, size, BufferedImage.TYPE_INT_RGB);
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float v = n1[y][x] * 0.5f + n2[y][x] * 0.3f + n3[y][x] * 0.2f;
                int r = clamp((int)((0.10f + v * 0.06f) * 255));
                int g = clamp((int)((0.11f + v * 0.06f) * 255));
                int b = clamp((int)((0.14f + v * 0.07f) * 255));
                img.setRGB(x, y, (r << 16) | (g << 8) | b);
            }
        }
        ImageIO.write(img, "PNG", new File(DIR + "deep_sediment.png"));
        System.out.println("  deep_sediment.png");
    }

    // ── Helpers ─────────────────────────────────────────────────────────

    static void drawBranch(Graphics2D g, Random rng, int x0, int y0, int x1, int y1, int bw, int tw) {
        int steps = Math.max(5, (int)(Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) / 3));
        for (int i = 0; i < steps; i++) {
            float t = (float) i / steps;
            int x = x0 + (int)((x1 - x0) * t), y = y0 + (int)((y1 - y0) * t);
            int w = (int)(bw + (tw - bw) * t);
            int c = 45 + rng.nextInt(15);
            g.setColor(new Color(c, c - 10, c - 20));
            g.fillOval(x - w / 2, y - 1, w, 3);
        }
    }

    static void leaves(Graphics2D g, Random rng, int cx, int cy, int rx, int ry,
                        int count, int gBase, int gMid, int alpha) {
        for (int i = 0; i < count; i++) {
            double ax = rng.nextGaussian() * 0.55;
            double ay = rng.nextGaussian() * 0.55;
            if (ax * ax + ay * ay > 1.0) continue;
            int x = cx + (int)(ax * rx), y = cy + (int)(ay * ry);
            double angle = rng.nextDouble() * Math.PI;
            int lw = 2 + rng.nextInt(4), lh = 1 + rng.nextInt(3);
            var old = g.getTransform();
            g.translate(x, y); g.rotate(angle);
            g.setColor(new Color(gBase + rng.nextInt(15), gMid + rng.nextInt(35),
                    5 + rng.nextInt(12), alpha));
            g.fillOval(-lw, -lh, lw * 2, lh * 2);
            g.setTransform(old);
        }
    }

    static void binarizeAlpha(BufferedImage img) {
        for (int y = 0; y < img.getHeight(); y++)
            for (int x = 0; x < img.getWidth(); x++) {
                int px = img.getRGB(x, y);
                int a = (px >> 24) & 0xFF;
                img.setRGB(x, y, a >= 128 ? (px | 0xFF000000) : 0);
            }
    }

    static float[][] valueNoise(int size, int freq) {
        float[][] grid = new float[freq + 1][freq + 1];
        var r = new Random(freq * 31 + 7);
        for (int y = 0; y <= freq; y++)
            for (int x = 0; x <= freq; x++) grid[y][x] = r.nextFloat();
        for (int y = 0; y <= freq; y++) grid[y][freq] = grid[y][0];
        for (int x = 0; x <= freq; x++) grid[freq][x] = grid[0][x];
        float[][] result = new float[size][size];
        for (int y = 0; y < size; y++) {
            float gy = (float) y / size * freq;
            int iy = (int) gy; float fy = gy - iy; fy = fy * fy * (3 - 2 * fy);
            for (int x = 0; x < size; x++) {
                float gx = (float) x / size * freq;
                int ix = (int) gx; float fx = gx - ix; fx = fx * fx * (3 - 2 * fx);
                float top = grid[iy][ix] + (grid[iy][ix + 1] - grid[iy][ix]) * fx;
                float bot = grid[iy + 1][ix] + (grid[iy + 1][ix + 1] - grid[iy + 1][ix]) * fx;
                result[y][x] = top + (bot - top) * fy;
            }
        }
        return result;
    }

    static BufferedImage clear(int w, int h) {
        var i = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
        var g = i.createGraphics(); g.setComposite(AlphaComposite.Clear);
        g.fillRect(0, 0, w, h); g.dispose(); return i;
    }

    static Graphics2D g(BufferedImage i) {
        var g = i.createGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        return g;
    }

    static int clamp(int v) { return Math.max(0, Math.min(255, v)); }
}
