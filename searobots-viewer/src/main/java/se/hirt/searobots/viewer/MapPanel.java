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

import se.hirt.searobots.api.*;
import se.hirt.searobots.engine.GeneratedWorld;
import se.hirt.searobots.engine.SubmarineSnapshot;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Top-down 2D rendering of the underwater terrain and overlays.
 */
public class MapPanel extends JPanel {

    private GeneratedWorld world;
    private BufferedImage terrainImage;

    // view state
    private double viewX, viewY;           // world centre of the view
    private double pixelsPerMeter = 0.3;   // zoom level
    private boolean showContours = true;
    private boolean showCurrents = false;
    private boolean showTrails = true;
    private boolean showRoute = false;
    private boolean showContactEstimates = true;
    private boolean showWaypoints = true;

    // mouse interaction
    private Point dragStart;
    private double dragViewX, dragViewY;

    // status line
    private String statusText = "";

    private boolean needsInitialFit = true;

    // submarine rendering
    private static final int MAX_TRAIL_LENGTH = 500;
    private volatile List<SubmarineSnapshot> submarines = List.of();
    @SuppressWarnings("unchecked")
    private Deque<Vec3>[] trails = new Deque[0];
    @SuppressWarnings("unchecked")
    private List<Vec3>[] routes = new List[0];
    private static final int ROUTE_SAMPLE_INTERVAL = 25; // record route every N ticks
    private volatile long simTick;
    private volatile boolean simPaused;
    private volatile double simSpeed = 1.0;

    // Ping animations
    private record PingAnimation(double x, double y, long startTick, Color color, int sourceId) {}
    private final java.util.ArrayList<PingAnimation> pingAnimations = new java.util.ArrayList<>();
    private static final double PING_VISUAL_SPEED = 1500.0; // m/s — realistic sound speed in water
    private static final double PING_MAX_RADIUS = 10000.0; // covers full battle area diameter
    private static final double PING_FLASH_DURATION = 1.2;  // seconds — bright origin burst
    private static final double PING_FLASH_RADIUS = 150.0;  // meters

    // Detection highlights: when a ping ring sweeps over another sub
    private record DetectionHighlight(double x, double y, long startTick, Color color) {}
    private final java.util.ArrayList<DetectionHighlight> detectionHighlights = new java.util.ArrayList<>();
    private static final double DETECTION_HIGHLIGHT_DURATION = 1.5; // seconds
    private static final double DETECTION_HIGHLIGHT_RADIUS = 80.0;  // meters

    // Ping fix trace records for contact tracking visualization
    private record PingFixRecord(double x, double y, long tick, Color color, int subId) {}
    private final java.util.ArrayList<PingFixRecord> pingFixRecords = new java.util.ArrayList<>();
    private static final long PING_FIX_EXPIRE_TICKS = 3000; // ~60 seconds at 50Hz

    public MapPanel(GeneratedWorld world) {
        setBackground(new Color(5, 10, 30));
        setWorld(world);
        setupInputHandlers();
    }

    public void setWorld(GeneratedWorld world) {
        this.world = world;
        this.viewX = 0;
        this.viewY = 0;
        fitBattleArea();
        this.terrainImage = renderTerrainImage();
        this.submarines = List.of();
        this.trails = new Deque[0];
        this.routes = new List[0];
        this.simTick = 0;
        repaint();
    }

    public void updateSubmarines(long tick, List<SubmarineSnapshot> subs) {
        this.simTick = tick;
        this.submarines = subs;

        // Grow trail/route arrays if needed
        if (trails.length < subs.size()) {
            @SuppressWarnings("unchecked")
            Deque<Vec3>[] newTrails = new Deque[subs.size()];
            System.arraycopy(trails, 0, newTrails, 0, trails.length);
            for (int i = trails.length; i < newTrails.length; i++) {
                newTrails[i] = new ArrayDeque<>();
            }
            trails = newTrails;
        }
        if (routes.length < subs.size()) {
            @SuppressWarnings("unchecked")
            List<Vec3>[] newRoutes = new List[subs.size()];
            System.arraycopy(routes, 0, newRoutes, 0, routes.length);
            for (int i = routes.length; i < newRoutes.length; i++) {
                newRoutes[i] = new java.util.ArrayList<>();
            }
            routes = newRoutes;
        }

        for (int i = 0; i < subs.size(); i++) {
            var pos = subs.get(i).pose().position();

            // Trail: recent breadcrumb (every 5 ticks, capped)
            if (tick % 5 == 0) {
                trails[i].addLast(pos);
                while (trails[i].size() > MAX_TRAIL_LENGTH) {
                    trails[i].removeFirst();
                }
            }

            // Route: full history (less frequent sampling)
            if (tick % ROUTE_SAMPLE_INTERVAL == 0) {
                routes[i].add(pos);
            }

            // Detect ping
            if (subs.get(i).pingRequested()) {
                pingAnimations.add(new PingAnimation(pos.x(), pos.y(), tick,
                        subs.get(i).color(), subs.get(i).id()));
            }
        }

        // Check if any ping ring just swept over a sub this tick
        for (var ping : pingAnimations) {
            double elapsed = (tick - ping.startTick) / 50.0;
            double prevElapsed = (tick - 1 - ping.startTick) / 50.0;
            if (prevElapsed < 0) continue;
            double ringNow = elapsed * PING_VISUAL_SPEED;
            double ringPrev = prevElapsed * PING_VISUAL_SPEED;

            for (var sub : subs) {
                if (sub.id() == ping.sourceId) continue; // don't highlight the pinger
                var sp = sub.pose().position();
                double dx = sp.x() - ping.x();
                double dy = sp.y() - ping.y();
                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist >= ringPrev && dist <= ringNow
                        && !isTerrainBlocked(ping.x(), ping.y(), sp.x(), sp.y())) {
                    detectionHighlights.add(new DetectionHighlight(
                            sp.x(), sp.y(), tick, ping.color()));
                }
            }
        }

        // Detect ping fixes from contact estimates for trace lines
        for (var sub : subs) {
            var estimates = sub.contactEstimates();
            if (estimates == null || estimates.isEmpty()) continue;
            for (var est : estimates) {
                if (est.confidence() > 0.7 && "ping".equals(est.label())
                        && est.uncertaintyRadius() < 100) {
                    // Avoid duplicates: check distance from last fix for this sub
                    boolean isDuplicate = false;
                    for (int j = pingFixRecords.size() - 1; j >= 0; j--) {
                        var prev = pingFixRecords.get(j);
                        if (prev.subId == sub.id()) {
                            double ddx = est.x() - prev.x;
                            double ddy = est.y() - prev.y;
                            if (ddx * ddx + ddy * ddy < 100) { // within 10m
                                isDuplicate = true;
                            }
                            break;
                        }
                    }
                    if (!isDuplicate) {
                        pingFixRecords.add(new PingFixRecord(
                                est.x(), est.y(), tick, sub.color(), sub.id()));
                    }
                }
            }
        }

        // Expire finished animations and old ping fix records
        double maxTime = PING_MAX_RADIUS / PING_VISUAL_SPEED + 1.0;
        pingAnimations.removeIf(p -> (tick - p.startTick) / 50.0 > maxTime);
        detectionHighlights.removeIf(h -> (tick - h.startTick) / 50.0 > DETECTION_HIGHLIGHT_DURATION);
        pingFixRecords.removeIf(r -> tick - r.tick > PING_FIX_EXPIRE_TICKS);

        repaint();
    }

    public void setSimPaused(boolean p) { this.simPaused = p; }
    public void setSimSpeed(double s) { this.simSpeed = s; }

    private void fitBattleArea() {
        int w = getWidth();
        int h = getHeight();
        if (w <= 0 || h <= 0) {
            // Not yet laid out — use a reasonable default, will refit on first paint
            pixelsPerMeter = 0.18;
            return;
        }
        double diameter = world.config().battleArea().extent() * 2;
        pixelsPerMeter = Math.min(w, h) * 0.8 / diameter;
    }

    public void zoom(double factor) {
        pixelsPerMeter *= factor;
        pixelsPerMeter = Math.max(0.01, Math.min(10.0, pixelsPerMeter));
        repaint();
    }

    public void toggleContours() {
        showContours = !showContours;
        repaint();
    }

    public void toggleCurrents() {
        showCurrents = !showCurrents;
        repaint();
    }

    public void toggleTrails() {
        showTrails = !showTrails;
        repaint();
    }

    public void toggleRoute() {
        showRoute = !showRoute;
        repaint();
    }

    public void toggleContactEstimates() {
        showContactEstimates = !showContactEstimates;
        repaint();
    }

    public void toggleWaypoints() {
        showWaypoints = !showWaypoints;
        repaint();
    }

    // ── rendering ──────────────────────────────────────────────────────

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        if (world == null) return;

        if (needsInitialFit) {
            needsInitialFit = false;
            fitBattleArea();
        }

        var g2 = (Graphics2D) g.create();
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Save the system's base transform (includes DPI scaling on Windows)
        var baseTransform = g2.getTransform();

        // Compose world-to-screen on top of the base transform
        var worldTransform = new AffineTransform(baseTransform);
        worldTransform.translate(getWidth() / 2.0, getHeight() / 2.0);
        worldTransform.scale(pixelsPerMeter, -pixelsPerMeter);
        worldTransform.translate(-viewX, -viewY);

        // Set world-coordinate transform for all world-space drawing.
        // drawImage composes its AffineTransform arg with g2's current
        // transform, so every draw method sees the same pipeline.
        g2.setTransform(worldTransform);

        drawTerrain(g2);
        drawBattleArea(g2);
        if (submarines.isEmpty()) drawSpawnPoints(g2);
        if (showCurrents) drawCurrentArrows(g2);
        if (showRoute) drawSubmarineRoutes(g2);
        if (showTrails) drawSubmarineTrails(g2);
        drawSubmarines(g2);
        if (showContactEstimates) drawContactEstimates(g2);
        if (showWaypoints) drawWaypoints(g2);
        drawPingAnimations(g2);
        drawDetectionHighlights(g2);

        g2.setTransform(baseTransform);
        g2.setStroke(new BasicStroke(1.0f));
        drawInfoOverlay(g2);
        if (!submarines.isEmpty()) drawSubmarineHud(g2);

        g2.dispose();
    }

    private void drawTerrain(Graphics2D g2) {
        // g2 is in world-coordinate space; drawImage composes imgToWorld
        // on top of the current transform, giving base * worldToScreen * imgToWorld
        var terrain = world.terrain();
        double cellSize = terrain.getCellSize();

        var imgToWorld = new AffineTransform();
        imgToWorld.translate(terrain.getOriginX(),
                terrain.getOriginY() + (terrain.getRows() - 1) * cellSize);
        imgToWorld.scale(cellSize, -cellSize);

        g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g2.drawImage(terrainImage, imgToWorld, null);
        g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION,
                RenderingHints.VALUE_INTERPOLATION_NEAREST_NEIGHBOR);

        if (showContours) drawContourLines(g2);
    }

    private static final double BASE_CONTOUR_INTERVAL = 25.0;
    private static final double MIN_CONTOUR_SPACING_PX = 4.0;

    private double effectiveContourInterval() {
        // Double the interval until contour lines are at least MIN_CONTOUR_SPACING_PX apart
        double interval = BASE_CONTOUR_INTERVAL;
        while (interval * pixelsPerMeter < MIN_CONTOUR_SPACING_PX) {
            interval *= 2;
        }
        return interval;
    }

    private void drawContourLines(Graphics2D g2) {
        // Draw contour lines as vector geometry in world coordinates
        var terrain = world.terrain();
        int cols = terrain.getCols();
        int rows = terrain.getRows();
        double cell = terrain.getCellSize();
        double ox = terrain.getOriginX();
        double oy = terrain.getOriginY();

        double interval = effectiveContourInterval();

        g2.setColor(new Color(0, 0, 0, 60));
        g2.setStroke(new BasicStroke((float) (1.0 / pixelsPerMeter)));

        for (int row = 0; row < rows - 1; row++) {
            for (int col = 0; col < cols - 1; col++) {
                double e00 = terrain.elevationAtGrid(col, row);
                double e10 = terrain.elevationAtGrid(col + 1, row);
                double e01 = terrain.elevationAtGrid(col, row + 1);

                // check horizontal edge (col,row) -> (col+1,row)
                long c0 = (long) Math.floor(e00 / interval);
                long c1 = (long) Math.floor(e10 / interval);
                if (c0 != c1) {
                    double contourElev = Math.max(c0, c1) * interval;
                    double t = (contourElev - e00) / (e10 - e00);
                    double cx = ox + (col + t) * cell;
                    double cy = oy + row * cell;
                    drawContourSegmentH(g2, terrain, col, row, cx, cy, cell, ox, oy, interval);
                }

                // check vertical edge (col,row) -> (col,row+1)
                long c2 = (long) Math.floor(e01 / interval);
                if (c0 != c2) {
                    double contourElev = Math.max(c0, c2) * interval;
                    double t = (contourElev - e00) / (e01 - e00);
                    double cx = ox + col * cell;
                    double cy = oy + (row + t) * cell;
                    drawContourSegmentV(g2, terrain, col, row, cx, cy, cell, ox, oy, interval);
                }
            }
        }
    }

    private void drawContourSegmentH(Graphics2D g2, TerrainMap terrain,
                                      int col, int row, double cx, double cy,
                                      double cell, double ox, double oy, double interval) {
        if (row + 1 >= terrain.getRows()) return;
        double e00 = terrain.elevationAtGrid(col, row);
        double e10 = terrain.elevationAtGrid(col + 1, row);
        double e01 = terrain.elevationAtGrid(col, row + 1);
        double e11 = terrain.elevationAtGrid(col + 1, row + 1);

        double contourElev = Math.max(Math.floor(e00 / interval),
                Math.floor(e10 / interval)) * interval;

        if (crossesContour(e01, e11, interval)) {
            double t = (contourElev - e01) / (e11 - e01);
            if (t >= 0 && t <= 1) {
                g2.draw(new Line2D.Double(cx, cy, ox + (col + t) * cell, oy + (row + 1) * cell));
                return;
            }
        }
        if (crossesContour(e00, e01, interval)) {
            double t = (contourElev - e00) / (e01 - e00);
            if (t >= 0 && t <= 1) {
                g2.draw(new Line2D.Double(cx, cy, ox + col * cell, oy + (row + t) * cell));
                return;
            }
        }
        if (crossesContour(e10, e11, interval)) {
            double t = (contourElev - e10) / (e11 - e10);
            if (t >= 0 && t <= 1) {
                g2.draw(new Line2D.Double(cx, cy, ox + (col + 1) * cell, oy + (row + t) * cell));
            }
        }
    }

    private void drawContourSegmentV(Graphics2D g2, TerrainMap terrain,
                                      int col, int row, double cx, double cy,
                                      double cell, double ox, double oy, double interval) {
        if (col + 1 >= terrain.getCols()) return;
        double e00 = terrain.elevationAtGrid(col, row);
        double e10 = terrain.elevationAtGrid(col + 1, row);
        double e01 = terrain.elevationAtGrid(col, row + 1);
        double e11 = terrain.elevationAtGrid(col + 1, row + 1);

        double contourElev = Math.max(Math.floor(e00 / interval),
                Math.floor(e01 / interval)) * interval;

        if (crossesContour(e10, e11, interval)) {
            double t = (contourElev - e10) / (e11 - e10);
            if (t >= 0 && t <= 1) {
                g2.draw(new Line2D.Double(cx, cy, ox + (col + 1) * cell, oy + (row + t) * cell));
                return;
            }
        }
        if (crossesContour(e01, e11, interval)) {
            double t = (contourElev - e01) / (e11 - e01);
            if (t >= 0 && t <= 1) {
                g2.draw(new Line2D.Double(cx, cy, ox + (col + t) * cell, oy + (row + 1) * cell));
                return;
            }
        }
        if (crossesContour(e00, e10, interval)) {
            double t = (contourElev - e00) / (e10 - e00);
            if (t >= 0 && t <= 1) {
                g2.draw(new Line2D.Double(cx, cy, ox + (col + t) * cell, oy + row * cell));
            }
        }
    }

    private void drawBattleArea(Graphics2D g2) {
        // g2 is already in world-coordinate transform
        g2.setColor(new Color(255, 200, 50, 160));
        g2.setStroke(new BasicStroke((float) (2.0 / pixelsPerMeter),
                BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL,
                0, new float[]{(float) (20 / pixelsPerMeter)}, 0));

        switch (world.config().battleArea()) {
            case BattleArea.Circular(var r) ->
                    g2.draw(new Ellipse2D.Double(-r, -r, 2 * r, 2 * r));
            case BattleArea.Rectangular(var hw, var hh) ->
                    g2.draw(new Rectangle2D.Double(-hw, -hh, 2 * hw, 2 * hh));
        }
    }

    private void drawSpawnPoints(Graphics2D g2) {
        // g2 is already in world-coordinate transform
        double r = 8 / pixelsPerMeter;
        g2.setStroke(new BasicStroke((float) (2.0 / pixelsPerMeter)));

        var colors = new Color[]{
                new Color(60, 220, 120), new Color(220, 80, 80),
                new Color(80, 140, 255), new Color(255, 180, 40),
                new Color(200, 80, 220), new Color(80, 220, 220)
        };

        var spawns = world.spawnPoints();
        for (int i = 0; i < spawns.size(); i++) {
            var sp = spawns.get(i);
            g2.setColor(colors[i % colors.length]);
            g2.fill(new Ellipse2D.Double(sp.x() - r, sp.y() - r, 2 * r, 2 * r));
            g2.setColor(Color.WHITE);
            g2.draw(new Ellipse2D.Double(sp.x() - r, sp.y() - r, 2 * r, 2 * r));
        }
    }

    private void drawCurrentArrows(Graphics2D g2) {
        // g2 is already in world-coordinate transform
        double extent = world.config().battleArea().extent();
        double spacing = extent / 8;
        double arrowLen = spacing * 0.4;

        var bands = world.currentField().bands();
        if (bands.isEmpty()) return;
        var midBand = bands.get(bands.size() / 2);
        var current = midBand.current();
        if (current.length() < 0.01) return;

        g2.setColor(new Color(180, 220, 255, 120));
        g2.setStroke(new BasicStroke((float) (1.5 / pixelsPerMeter)));
        var dir = current.normalize();

        for (double x = -extent; x <= extent; x += spacing) {
            for (double y = -extent; y <= extent; y += spacing) {
                double ex = x + dir.x() * arrowLen;
                double ey = y + dir.y() * arrowLen;
                g2.draw(new Line2D.Double(x, y, ex, ey));
                double ax = -dir.x() * arrowLen * 0.25 + dir.y() * arrowLen * 0.15;
                double ay = -dir.y() * arrowLen * 0.25 - dir.x() * arrowLen * 0.15;
                g2.draw(new Line2D.Double(ex, ey, ex + ax, ey + ay));
                ax = -dir.x() * arrowLen * 0.25 - dir.y() * arrowLen * 0.15;
                ay = -dir.y() * arrowLen * 0.25 + dir.x() * arrowLen * 0.15;
                g2.draw(new Line2D.Double(ex, ey, ex + ax, ey + ay));
            }
        }
    }

    private void drawSubmarines(Graphics2D g2) {
        // g2 is in world-coordinate transform
        var subs = submarines;
        double size = 30 / pixelsPerMeter; // triangle size in world coords

        for (var sub : subs) {
            var pos = sub.pose().position();
            double heading = sub.pose().heading();

            var saved = g2.getTransform();
            g2.translate(pos.x(), pos.y());
            // Heading is clockwise from +Y; in our world-coord system
            // (Y-up, screen Y-flipped), rotate by -heading
            g2.rotate(-heading);

            var triangle = new Path2D.Double();
            triangle.moveTo(0, size * 0.6);        // nose (forward = +Y in body)
            triangle.lineTo(-size * 0.35, -size * 0.4);
            triangle.lineTo(size * 0.35, -size * 0.4);
            triangle.closePath();

            Color c = sub.forfeited() ? Color.DARK_GRAY : sub.color();
            g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 180));
            g2.fill(triangle);
            g2.setColor(Color.WHITE);
            g2.setStroke(new BasicStroke((float) (1.5 / pixelsPerMeter)));
            g2.draw(triangle);

            g2.setTransform(saved);
        }
    }

    private void drawContactEstimates(Graphics2D g2) {
        // g2 is in world-coordinate transform
        var subs = submarines;
        double markerSize = 20 / pixelsPerMeter;
        long tick = simTick;

        for (var sub : subs) {
            var estimates = sub.contactEstimates();
            if (estimates == null || estimates.isEmpty()) continue;

            Color base = sub.color();
            for (var est : estimates) {
                double ur = est.uncertaintyRadius();
                double alive = est.contactAlive();
                int alpha = (int) (60 + 180 * alive);

                // Uncertainty circle (translucent fill)
                double minDrawRadius = 10; // minimum world-coord radius to draw
                if (ur > minDrawRadius) {
                    int circleAlpha = (int) (alive * 30);
                    if (circleAlpha > 0) {
                        g2.setColor(new Color(base.getRed(), base.getGreen(), base.getBlue(),
                                Math.min(circleAlpha, 255)));
                        g2.fill(new Ellipse2D.Double(
                                est.x() - ur, est.y() - ur,
                                ur * 2, ur * 2));
                    }
                    // Circle outline
                    int outlineAlpha = (int) (alive * 120);
                    if (outlineAlpha > 0) {
                        g2.setColor(new Color(base.getRed(), base.getGreen(), base.getBlue(),
                                Math.min(outlineAlpha, 255)));
                        g2.setStroke(new BasicStroke((float) (1.5 / pixelsPerMeter)));
                        g2.draw(new Ellipse2D.Double(
                                est.x() - ur, est.y() - ur,
                                ur * 2, ur * 2));
                    }
                }

                // Center diamond marker
                g2.setColor(new Color(base.getRed(), base.getGreen(), base.getBlue(), alpha));
                double s = markerSize * (0.5 + 0.5 * alive);
                var diamond = new Path2D.Double();
                diamond.moveTo(est.x(), est.y() + s);
                diamond.lineTo(est.x() + s * 0.6, est.y());
                diamond.lineTo(est.x(), est.y() - s);
                diamond.lineTo(est.x() - s * 0.6, est.y());
                diamond.closePath();
                g2.fill(diamond);

                // Diamond outline
                g2.setColor(new Color(255, 255, 255, alpha / 2));
                g2.setStroke(new BasicStroke((float) (1.0 / pixelsPerMeter)));
                g2.draw(diamond);

                // Draw heading/speed arrow if known
                if (!Double.isNaN(est.estimatedHeading()) && est.estimatedSpeed() > 0) {
                    double arrowLen = est.estimatedSpeed() * 20 / pixelsPerMeter; // scale: 20px per m/s
                    double arrowX = est.x() + arrowLen * Math.sin(est.estimatedHeading());
                    double arrowY = est.y() + arrowLen * Math.cos(est.estimatedHeading());
                    // Use sub color with heading arrow
                    g2.setColor(new Color(base.getRed(), base.getGreen(), base.getBlue(), alpha));
                    g2.setStroke(new BasicStroke((float) (2.0 / pixelsPerMeter),
                            BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                    g2.draw(new Line2D.Double(est.x(), est.y(), arrowX, arrowY));
                    // Arrowhead
                    double headLen = arrowLen * 0.2;
                    double headAngle = Math.toRadians(25);
                    double hx1 = arrowX - headLen * Math.sin(est.estimatedHeading() + headAngle);
                    double hy1 = arrowY - headLen * Math.cos(est.estimatedHeading() + headAngle);
                    double hx2 = arrowX - headLen * Math.sin(est.estimatedHeading() - headAngle);
                    double hy2 = arrowY - headLen * Math.cos(est.estimatedHeading() - headAngle);
                    g2.draw(new Line2D.Double(arrowX, arrowY, hx1, hy1));
                    g2.draw(new Line2D.Double(arrowX, arrowY, hx2, hy2));
                }
            }

            // Ping trace lines for this sub
            g2.setStroke(new BasicStroke((float) (1.5 / pixelsPerMeter)));
            PingFixRecord prev = null;
            for (var rec : pingFixRecords) {
                if (rec.subId != sub.id()) continue;
                if (prev != null) {
                    long age = tick - rec.tick;
                    float ageFraction = Math.min((float) age / PING_FIX_EXPIRE_TICKS, 1.0f);
                    int lineAlpha = (int) ((1.0f - ageFraction * 0.8f) * 180);
                    if (lineAlpha > 0) {
                        g2.setColor(new Color(rec.color.getRed(), rec.color.getGreen(),
                                rec.color.getBlue(), lineAlpha));
                        g2.draw(new Line2D.Double(prev.x, prev.y, rec.x, rec.y));
                    }
                }
                prev = rec;
            }
        }
    }

    private void drawWaypoints(Graphics2D g2) {
        // g2 is in world-coordinate transform
        var subs = submarines;
        double markerRadius = 8 / pixelsPerMeter; // ~8 screen pixels

        for (var sub : subs) {
            var waypoints = sub.waypoints();
            if (waypoints == null || waypoints.isEmpty()) continue;

            Color subColor = sub.color();

            // 1. Draw Catmull-Rom spline through waypoints (darker shade to avoid trail conflict)
            if (waypoints.size() >= 2) {
                g2.setColor(new Color(subColor.getRed() / 2, subColor.getGreen() / 2,
                        subColor.getBlue() / 2, 160));
                g2.setStroke(new BasicStroke((float) (2.0 / pixelsPerMeter),
                        BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));

                for (int i = 0; i < waypoints.size() - 1; i++) {
                    // Control points for Catmull-Rom: p0, p1, p2, p3
                    var p0 = waypoints.get(Math.max(0, i - 1));
                    var p1 = waypoints.get(i);
                    var p2 = waypoints.get(i + 1);
                    var p3 = waypoints.get(Math.min(waypoints.size() - 1, i + 2));

                    int segments = 20;
                    double prevX = p1.x();
                    double prevY = p1.y();

                    for (int s = 1; s <= segments; s++) {
                        double t = (double) s / segments;
                        double t2 = t * t;
                        double t3 = t2 * t;

                        // Catmull-Rom formula
                        double cx = 0.5 * ((2 * p1.x())
                                + (-p0.x() + p2.x()) * t
                                + (2 * p0.x() - 5 * p1.x() + 4 * p2.x() - p3.x()) * t2
                                + (-p0.x() + 3 * p1.x() - 3 * p2.x() + p3.x()) * t3);
                        double cy = 0.5 * ((2 * p1.y())
                                + (-p0.y() + p2.y()) * t
                                + (2 * p0.y() - 5 * p1.y() + 4 * p2.y() - p3.y()) * t2
                                + (-p0.y() + 3 * p1.y() - 3 * p2.y() + p3.y()) * t3);

                        g2.draw(new Line2D.Double(prevX, prevY, cx, cy));
                        prevX = cx;
                        prevY = cy;
                    }
                }
            }

            // 2. Draw waypoint markers
            for (int i = 0; i < waypoints.size(); i++) {
                var wp = waypoints.get(i);
                boolean isActive = wp.active();

                double r = isActive ? markerRadius * 1.4 : markerRadius;

                // Depth-based coloring
                Color depthColor = waypointDepthColor(wp.z());

                // Inner fill
                g2.setColor(depthColor);
                g2.fill(new Ellipse2D.Double(wp.x() - r, wp.y() - r, 2 * r, 2 * r));

                // White outline (thicker for active)
                float strokeWidth = isActive ? (float) (2.5 / pixelsPerMeter) : (float) (1.0 / pixelsPerMeter);
                g2.setStroke(new BasicStroke(strokeWidth));
                g2.setColor(isActive ? Color.WHITE : new Color(255, 255, 255, 160));
                g2.draw(new Ellipse2D.Double(wp.x() - r, wp.y() - r, 2 * r, 2 * r));

                // Active waypoint gets an extra bright ring
                if (isActive) {
                    double outerR = r * 1.5;
                    g2.setColor(new Color(255, 255, 255, 80));
                    g2.setStroke(new BasicStroke((float) (1.0 / pixelsPerMeter)));
                    g2.draw(new Ellipse2D.Double(wp.x() - outerR, wp.y() - outerR,
                            2 * outerR, 2 * outerR));
                }
            }
        }
    }

    /**
     * Returns a depth-based color for waypoint visualization.
     * Shallow (-20m): light cyan (150, 220, 255)
     * Moderate (-200m): medium blue (50, 100, 220)
     * Deep (-500m): dark blue/purple (80, 40, 180)
     */
    private static Color waypointDepthColor(double z) {
        // z is negative (depth below sea level)
        double depth = -z; // positive depth value

        int r, g, b;
        if (depth <= 20) {
            r = 150; g = 220; b = 255;
        } else if (depth <= 200) {
            double t = (depth - 20) / 180.0;
            r = (int) (150 + t * (50 - 150));
            g = (int) (220 + t * (100 - 220));
            b = (int) (255 + t * (220 - 255));
        } else if (depth <= 500) {
            double t = (depth - 200) / 300.0;
            r = (int) (50 + t * (80 - 50));
            g = (int) (100 + t * (40 - 100));
            b = (int) (220 + t * (180 - 220));
        } else {
            r = 80; g = 40; b = 180;
        }
        return new Color(r, g, b, 200);
    }

    private void drawPingAnimations(Graphics2D g2) {
        // g2 is in world-coordinate transform
        long tick = simTick;

        for (var ping : pingAnimations) {
            double elapsed = (tick - ping.startTick) / 50.0; // seconds
            double leadingRadius = elapsed * PING_VISUAL_SPEED;
            Color c = ping.color();
            float progress = (float) Math.min(leadingRadius / PING_MAX_RADIUS, 1.0);

            // 1. Origin flash — bright burst that fades over PING_FLASH_DURATION
            if (elapsed < PING_FLASH_DURATION) {
                float t = (float) (elapsed / PING_FLASH_DURATION);
                float flashFade = (1.0f - t) * (1.0f - t); // quadratic — fast initial fade
                double flashR = PING_FLASH_RADIUS * (0.3 + 0.7 * t); // grows outward

                // Bright white core
                int coreAlpha = (int) (flashFade * 160);
                if (coreAlpha > 0) {
                    g2.setColor(new Color(255, 255, 255, coreAlpha));
                    g2.fill(new Ellipse2D.Double(
                            ping.x() - flashR, ping.y() - flashR, flashR * 2, flashR * 2));
                }

                // Colored outer glow
                double glowR = flashR * 1.6;
                int glowAlpha = (int) (flashFade * 60);
                if (glowAlpha > 0) {
                    g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), glowAlpha));
                    g2.fill(new Ellipse2D.Double(
                            ping.x() - glowR, ping.y() - glowR, glowR * 2, glowR * 2));
                }
            }

            if (leadingRadius <= 0) continue;

            // 2. Sonar wash — very subtle filled disc behind the leading edge
            int washAlpha = (int) ((1.0f - progress * 0.8f) * 15);
            if (washAlpha > 0) {
                g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), washAlpha));
                g2.fill(new Ellipse2D.Double(
                        ping.x() - leadingRadius, ping.y() - leadingRadius,
                        leadingRadius * 2, leadingRadius * 2));
            }

            // 3. Leading ring — bright white edge with colored glow rings trailing behind
            float distanceFade = 1.0f - progress * 0.6f; // fades to 40% at max range

            // Bright leading edge (white-shifted)
            int brightR = Math.min(255, c.getRed() + 140);
            int brightG = Math.min(255, c.getGreen() + 140);
            int brightB = Math.min(255, c.getBlue() + 140);
            int leadAlpha = (int) (distanceFade * 220);
            g2.setColor(new Color(brightR, brightG, brightB, Math.min(255, leadAlpha)));
            g2.setStroke(new BasicStroke((float) (3.0 / pixelsPerMeter),
                    BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.draw(new Ellipse2D.Double(
                    ping.x() - leadingRadius, ping.y() - leadingRadius,
                    leadingRadius * 2, leadingRadius * 2));

            // 4. Trailing glow rings — 4 rings closely spaced behind the leading edge
            for (int i = 1; i <= 4; i++) {
                double r = leadingRadius - i * 200.0; // 200m spacing
                if (r <= 0) break;
                int a = (int) (distanceFade * (140 - i * 30));
                if (a <= 0) continue;
                g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), a));
                g2.setStroke(new BasicStroke((float) ((2.5 - i * 0.4) / pixelsPerMeter),
                        BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.draw(new Ellipse2D.Double(ping.x() - r, ping.y() - r, r * 2, r * 2));
            }
        }
    }

    private void drawDetectionHighlights(Graphics2D g2) {
        // g2 is in world-coordinate transform
        long tick = simTick;

        for (var h : detectionHighlights) {
            double elapsed = (tick - h.startTick) / 50.0;
            float t = (float) (elapsed / DETECTION_HIGHLIGHT_DURATION);
            if (t > 1.0f) continue;

            // Pulsing diamond that fades out
            float fade = (1.0f - t) * (1.0f - t); // quadratic fade
            double pulseRadius = DETECTION_HIGHLIGHT_RADIUS * (0.5 + 0.5 * t); // grows slightly

            // Bright filled glow
            int glowAlpha = (int) (fade * 100);
            if (glowAlpha > 0) {
                Color c = h.color();
                g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), glowAlpha));
                g2.fill(new Ellipse2D.Double(
                        h.x() - pulseRadius, h.y() - pulseRadius,
                        pulseRadius * 2, pulseRadius * 2));
            }

            // Bright white ring
            int ringAlpha = (int) (fade * 220);
            if (ringAlpha > 0) {
                g2.setColor(new Color(255, 255, 255, ringAlpha));
                g2.setStroke(new BasicStroke((float) (2.5 / pixelsPerMeter),
                        BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.draw(new Ellipse2D.Double(
                        h.x() - pulseRadius, h.y() - pulseRadius,
                        pulseRadius * 2, pulseRadius * 2));
            }
        }
    }

    private void drawSubmarineRoutes(Graphics2D g2) {
        // Full path history — thin solid line at low alpha
        var subs = submarines;
        g2.setStroke(new BasicStroke((float) (1.0 / pixelsPerMeter)));

        for (int i = 0; i < subs.size() && i < routes.length; i++) {
            var route = routes[i];
            if (route == null || route.size() < 2) continue;

            Color c = subs.get(i).color();
            g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 80));
            for (int j = 1; j < route.size(); j++) {
                var p0 = route.get(j - 1);
                var p1 = route.get(j);
                g2.draw(new Line2D.Double(p0.x(), p0.y(), p1.x(), p1.y()));
            }
        }
    }

    private void drawSubmarineTrails(Graphics2D g2) {
        // g2 is in world-coordinate transform
        var subs = submarines;
        g2.setStroke(new BasicStroke((float) (1.5 / pixelsPerMeter)));

        for (int i = 0; i < subs.size() && i < trails.length; i++) {
            var trail = trails[i];
            if (trail == null || trail.size() < 2) continue;

            Color c = subs.get(i).color();
            Vec3[] points;
            try { points = trail.toArray(new Vec3[0]); }
            catch (Exception e) { continue; } // concurrent modification
            for (int j = 1; j < points.length; j++) {
                if (points[j - 1] == null || points[j] == null) continue;
                float alpha = (float) j / points.length * 0.6f;
                g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(),
                        (int) (alpha * 255)));
                g2.draw(new Line2D.Double(
                        points[j - 1].x(), points[j - 1].y(),
                        points[j].x(), points[j].y()));
            }
        }
    }

    private void drawSubmarineHud(Graphics2D g2) {
        // g2 is in screen coordinates
        g2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        int lineH = 16;
        int padding = 8;
        int boxW = 310;

        // Calculate box height: 1 line for tick + 4px gap + 4 lines per sub
        var subs = submarines;
        int boxH = lineH + 4 + subs.size() * 4 * lineH + padding;

        int boxX = getWidth() - boxW - padding;
        int boxY = padding;

        // Backdrop
        g2.setColor(new Color(0, 0, 0, 128));
        g2.fillRoundRect(boxX, boxY, boxW, boxH, 8, 8);

        int x = boxX + padding;
        int y = boxY + padding + 12;

        // Sim status
        g2.setColor(new Color(200, 220, 255));
        String status = simPaused ? "PAUSED" : String.format("%.0fx", simSpeed);
        g2.drawString(String.format("Tick: %d  [%s]", simTick, status), x, y);
        y += lineH + 4;

        for (var sub : subs) {
            var pos = sub.pose().position();
            Color c = sub.forfeited() ? Color.DARK_GRAY : sub.color();
            g2.setColor(c);

            String label = sub.forfeited() ? "FORFEIT"
                    : sub.hp() <= 0 ? "DEAD" : "";
            g2.drawString(String.format("%s (#%d)  HP %d  %s",
                    sub.name(), sub.id(), sub.hp(), label), x, y);
            y += lineH;

            g2.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 200));
            g2.drawString(String.format("  %5.1f m/s  depth %4.0fm  hdg %5.1f\u00b0",
                    sub.speed(), -pos.z(),
                    Math.toDegrees(sub.pose().heading())), x, y);
            y += lineH;

            // Noise + throttle
            double noise = sub.noiseLevel();
            double throttle = sub.throttle();
            // Convert linear noise back to dB for display (80 dB = 1.0 linear)
            double noiseDb = 80 + 20 * Math.log10(Math.max(noise, 0.001));
            int noiseR = (int) Math.clamp((noiseDb - 70) * 25.5, 0, 255);
            g2.setColor(new Color(Math.max(noiseR, 80), 200 - noiseR / 2, 50, 200));
            g2.drawString(String.format("  noise:%3.0f dB  throttle:%+4.0f%%",
                    noiseDb, throttle * 100), x, y);
            y += lineH;

            if (sub.status() != null && !sub.status().isEmpty()) {
                g2.setColor(new Color(180, 200, 230));
                g2.drawString("  " + sub.status(), x, y);
            }
            y += lineH;
        }
    }

    private void drawInfoOverlay(Graphics2D g2) {
        // g2 is in screen coordinates (base transform)
        g2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        int lineH = 16;
        int padding = 6;

        // Top-left info panel: seed, thermoclines, spawns
        int infoLines = 2 + world.thermalLayers().size(); // seed + spawns + thermoclines
        int infoW = 320;
        int infoH = infoLines * lineH + padding * 2;
        g2.setColor(new Color(0, 0, 0, 128));
        g2.fillRoundRect(padding, padding, infoW, infoH, 8, 8);

        g2.setColor(new Color(200, 220, 255));
        int y = padding + padding + 12;
        int x = padding + padding;

        g2.drawString("Seed: " + world.config().worldSeed(), x, y);
        y += lineH;

        for (int i = 0; i < world.thermalLayers().size(); i++) {
            var tl = world.thermalLayers().get(i);
            g2.drawString(String.format("Thermocline %d: %.0fm  %.1f\u00b0C / %.1f\u00b0C",
                    i + 1, -tl.depth(), tl.temperatureAbove(), tl.temperatureBelow()), x, y);
            y += lineH;
        }

        g2.drawString(String.format("Spawns: %d", world.spawnPoints().size()), x, y);

        // depth / altitude colour key
        drawColorKey(g2);

        drawScaleBar(g2);

        if (!statusText.isEmpty()) {
            g2.setColor(new Color(200, 220, 255));
            g2.drawString(statusText, padding + padding, getHeight() - 10);
        }

        // Bottom-right help panel
        String[] help = {
                "SPACE  new map",
                "+/-    zoom",
                "C      contours " + (showContours ? "ON" : "OFF"),
                "F      currents " + (showCurrents ? "ON" : "OFF"),
                "T      trails " + (showTrails ? "ON" : "OFF"),
                "R      route " + (showRoute ? "ON" : "OFF"),
                "E      contacts " + (showContactEstimates ? "ON" : "OFF"),
                "W      waypoints " + (showWaypoints ? "ON" : "OFF"),
                "P      pause/resume",
                "N      single step",
                "1-5    speed 1x/2x/5x/10x/25x",
                "drag   pan"
        };
        int helpW = 220;
        int helpH = help.length * lineH + padding * 2;
        int helpX = getWidth() - helpW - padding;
        int helpY = getHeight() - helpH - padding;
        g2.setColor(new Color(0, 0, 0, 128));
        g2.fillRoundRect(helpX, helpY, helpW, helpH, 8, 8);

        g2.setColor(new Color(150, 170, 200));
        int hy = helpY + padding + 12;
        for (String h : help) {
            g2.drawString(h, helpX + padding, hy);
            hy += lineH;
        }
    }

    private void drawScaleBar(Graphics2D g2) {
        // pick a nice round distance that fits roughly 1/4 of the panel width
        double targetPixels = getWidth() * 0.25;
        double targetMeters = targetPixels / pixelsPerMeter;

        // snap to a nice round number
        double[] niceValues = {10, 25, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
        double scaleMeters = niceValues[niceValues.length - 1];
        for (double v : niceValues) {
            if (v * pixelsPerMeter >= targetPixels * 0.4) {
                scaleMeters = v;
                break;
            }
        }

        int barPx = (int) (scaleMeters * pixelsPerMeter);
        int bx = (getWidth() - barPx) / 2;
        int by = getHeight() - 30;
        int barH = 6;

        // backdrop
        g2.setColor(new Color(5, 10, 30, 140));
        g2.fillRoundRect(bx - 6, by - 16, barPx + 12, barH + 22, 4, 4);

        // bar
        g2.setColor(new Color(200, 220, 255));
        g2.fillRect(bx, by, barPx, barH);

        // end ticks
        g2.fillRect(bx, by - 3, 1, barH + 6);
        g2.fillRect(bx + barPx, by - 3, 1, barH + 6);

        // label
        g2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        String label = scaleMeters >= 1000
                ? String.format("%.0f km", scaleMeters / 1000)
                : String.format("%.0f m", scaleMeters);
        var fm = g2.getFontMetrics();
        g2.drawString(label, bx + (barPx - fm.stringWidth(label)) / 2, by - 4);
    }

    private static final int COLOR_KEY_BAR_WIDTH = 30;
    private static final int COLOR_KEY_BORDER = 3;       // grey padding around the colored bar
    private static final int COLOR_KEY_MARGIN_LEFT = 6;   // gap from panel edge to backdrop
    private static final int COLOR_KEY_MARGIN_TOP = 250;  // gap from panel top to backdrop
    private static final int COLOR_KEY_MARGIN_BOTTOM = 250; // gap from panel bottom to backdrop
    private static final int TICK_LENGTH = 4;
    private static final int LABEL_OFFSET = 12;  // gap from bar edge to label text

    private void drawColorKey(Graphics2D g2) {
        var terrain = world.terrain();
        double minE = terrain.getMinElevation();
        double maxE = terrain.getMaxElevation();
        double rangeE = maxE - minE;
        if (rangeE == 0) return;

        int backdropX = COLOR_KEY_MARGIN_LEFT;
        int barX = backdropX + COLOR_KEY_BORDER;
        int barTop = COLOR_KEY_MARGIN_TOP + COLOR_KEY_BORDER;
        int barBot = getHeight() - COLOR_KEY_MARGIN_BOTTOM - COLOR_KEY_BORDER;
        int barH = barBot - barTop;
        if (barH < 100) return;

        int labelX = barX + COLOR_KEY_BAR_WIDTH + LABEL_OFFSET;
        int backdropW = COLOR_KEY_BORDER * 2 + COLOR_KEY_BAR_WIDTH;
        int backdropH = COLOR_KEY_BORDER * 2 + barH;

        // semi-transparent backdrop snug around bar + labels
        g2.setColor(new Color(5, 10, 30, 140));
        g2.fillRoundRect(backdropX, barTop - COLOR_KEY_BORDER,
                backdropW, backdropH, 4, 4);

        // draw the gradient bar
        for (int py = 0; py < barH; py++) {
            double t = 1.0 - (double) py / barH;
            double elev = minE + t * rangeE;
            g2.setColor(elevationColor(elev, minE, maxE, rangeE));
            g2.fillRect(barX, barTop + py, COLOR_KEY_BAR_WIDTH, 1);
        }


        // tick marks and labels — 100m intervals
        g2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 14));

        double tickInterval = 100;
        double firstTick = Math.ceil(-maxE / tickInterval) * tickInterval;
        double lastTick = Math.floor(-minE / tickInterval) * tickInterval;

        boolean hasSeaLevel = minE < 0 && maxE > 0;
        for (double depth = firstTick; depth <= lastTick; depth += tickInterval) {
            if (depth == 0 && hasSeaLevel) continue; // SEA LEVEL marker covers this
            double elev = -depth;
            double t = (elev - minE) / rangeE;
            int sy = barBot - (int) (t * barH);
            if (sy < barTop + 6 || sy > barBot - 6) continue;

            g2.setColor(new Color(150, 170, 200));
            g2.drawLine(barX + COLOR_KEY_BAR_WIDTH, sy, barX + COLOR_KEY_BAR_WIDTH + TICK_LENGTH, sy);
            g2.setColor(new Color(220, 230, 255));
            String label = depth <= 0
                    ? String.format("+%.0f m", -depth)
                    : String.format("%.0f m", depth);
            g2.drawString(label, labelX, sy + 5);
        }

        // water line marker
        if (minE < 0 && maxE > 0) {
            double t = (0 - minE) / rangeE;
            int wy = barBot - (int) (t * barH);
            g2.setColor(new Color(255, 255, 255, 200));
            g2.drawLine(barX - 2, wy, barX + COLOR_KEY_BAR_WIDTH + TICK_LENGTH, wy);
            g2.setFont(new Font(Font.MONOSPACED, Font.BOLD, 11));
            g2.drawString("SEA LEVEL", labelX, wy + 4);
            g2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        }
    }

    private static Color elevationColor(double elev, double minE, double maxE, double rangeE) {
        int r, g, b;
        if (elev >= 0) {
            double t = Math.min(elev / Math.max(1, maxE), 1.0);
            r = (int) (50 + t * 90);
            g = (int) (100 + t * 155);
            b = (int) (30 + t * 40);
        } else {
            double t = (elev - minE) / rangeE;
            r = (int) (15 + t * 70);
            g = (int) (25 + t * 155);
            b = (int) (70 + t * 130);
        }
        return new Color(r, g, b);
    }

    // ── terrain image ──────────────────────────────────────────────────

    private BufferedImage renderTerrainImage() {
        if (world == null) return null;
        var terrain = world.terrain();
        int w = terrain.getCols();
        int h = terrain.getRows();
        var img = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);

        double minE = terrain.getMinElevation();
        double maxE = terrain.getMaxElevation();
        double rangeE = maxE - minE;
        if (rangeE == 0) rangeE = 1;

        for (int py = 0; py < h; py++) {
            int row = h - 1 - py;
            for (int px = 0; px < w; px++) {
                double elev = terrain.elevationAtGrid(px, row);
                Color c = elevationColor(elev, minE, maxE, rangeE);
                img.setRGB(px, py, (c.getRed() << 16) | (c.getGreen() << 8) | c.getBlue());
            }
        }
        return img;
    }

    private static boolean crossesContour(double a, double b, double interval) {
        long la = (long) Math.floor(a / interval);
        long lb = (long) Math.floor(b / interval);
        return la != lb;
    }

    // ── input handling ─────────────────────────────────────────────────

    private void setupInputHandlers() {
        setFocusable(true);

        addMouseWheelListener(e -> {
            double rotation = e.getPreciseWheelRotation();
            if (rotation == 0) return;
            double factor = Math.pow(1.15, -rotation);
            zoom(factor);
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                if (dragStart != null) {
                    double dx = (e.getX() - dragStart.x) / pixelsPerMeter;
                    double dy = -(e.getY() - dragStart.y) / pixelsPerMeter;
                    viewX = dragViewX - dx;
                    viewY = dragViewY - dy;
                    repaint();
                }
            }

            @Override
            public void mouseMoved(MouseEvent e) {
                double wx = screenToWorldX(e.getX());
                double wy = screenToWorldY(e.getY());
                double elev = world.terrain().elevationAt(wx, wy);
                statusText = String.format("(%.0f, %.0f)  depth: %.0fm", wx, wy, -elev);
                repaint();
            }
        });

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                dragStart = e.getPoint();
                dragViewX = viewX;
                dragViewY = viewY;
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                dragStart = null;
            }
        });
    }

    private double screenToWorldX(int sx) {
        return (sx - getWidth() / 2.0) / pixelsPerMeter + viewX;
    }

    private double screenToWorldY(int sy) {
        return -(sy - getHeight() / 2.0) / pixelsPerMeter + viewY;
    }

    /**
     * Check if an island blocks line-of-sight between two surface points.
     * Steps at half-cell resolution so no terrain cell is skipped, regardless of distance.
     */
    private boolean isTerrainBlocked(double x1, double y1, double x2, double y2) {
        if (world == null) return false;
        var terrain = world.terrain();
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 1.0) return false;
        double step = terrain.getCellSize() * 0.5;
        int samples = Math.max(2, (int) Math.ceil(dist / step));
        for (int i = 1; i < samples; i++) {
            double t = (double) i / samples;
            if (terrain.elevationAt(x1 + dx * t, y1 + dy * t) >= 0) {
                return true;
            }
        }
        return false;
    }
}
