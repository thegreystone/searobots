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
package se.hirt.searobots.engine;

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.Vec3;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Locale;

/**
 * Records match data to a JSONL file for post-match analysis.
 * <p>
 * Logs every N ticks (default 10 → 5 Hz at 50 Hz tick rate), plus
 * every tick where damage occurs or a sub dies/forfeits.
 * <p>
 * Format: first line is match header, subsequent lines are tick data.
 */
public final class MatchRecorder implements SimulationListener {

    private static final int LOG_INTERVAL = 10;  // log every 10 ticks (5 Hz)
    private static final DateTimeFormatter TS_FMT =
            DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");

    private final BufferedWriter writer;
    private final Path logFile;
    private int[] lastHp;

    public MatchRecorder(MatchConfig config, List<Vec3> spawnPoints, Path logDir)
            throws IOException {
        Files.createDirectories(logDir);
        String timestamp = LocalDateTime.now().format(TS_FMT);
        this.logFile = logDir.resolve(
                "match-" + config.worldSeed() + "-" + timestamp + ".jsonl");
        this.writer = Files.newBufferedWriter(logFile);

        writeHeader(config, spawnPoints);
    }

    public Path logFile() { return logFile; }

    private void writeHeader(MatchConfig config, List<Vec3> spawnPoints) throws IOException {
        var sb = new StringBuilder(512);
        sb.append("{\"type\":\"header\"");
        sb.append(",\"seed\":").append(config.worldSeed());
        sb.append(",\"tickRate\":").append(config.tickRateHz());
        sb.append(",\"durationTicks\":").append(config.matchDurationTicks());
        sb.append(",\"startingHp\":").append(config.startingHp());
        sb.append(",\"crushDepth\":").append(config.crushDepth());
        sb.append(",\"ratedDepth\":").append(config.ratedDepth());
        var arena = config.battleArea();
        switch (arena) {
            case se.hirt.searobots.api.BattleArea.Circular c ->
                sb.append(",\"arena\":{\"type\":\"circle\",\"radius\":").append(c.radius()).append("}");
            case se.hirt.searobots.api.BattleArea.Rectangular r ->
                sb.append(",\"arena\":{\"type\":\"rect\",\"halfWidth\":").append(r.halfWidth())
                  .append(",\"halfHeight\":").append(r.halfHeight()).append("}");
        }
        sb.append(",\"spawns\":[");
        for (int i = 0; i < spawnPoints.size(); i++) {
            if (i > 0) sb.append(",");
            appendVec3(sb, spawnPoints.get(i));
        }
        sb.append("]}\n");
        writer.write(sb.toString());
    }

    @Override
    public void onTick(long tick, List<SubmarineSnapshot> submarines) {
        // Initialize HP tracking on first tick
        if (lastHp == null) {
            lastHp = new int[submarines.size()];
            for (int i = 0; i < submarines.size(); i++) {
                lastHp[i] = submarines.get(i).hp();
            }
        }

        // Check if any sub took damage or died/forfeited this tick
        boolean eventTick = false;
        for (int i = 0; i < submarines.size(); i++) {
            var sub = submarines.get(i);
            if (sub.hp() < lastHp[i] || sub.hp() <= 0 || sub.forfeited()) {
                eventTick = true;
            }
            lastHp[i] = sub.hp();
        }

        // Log at regular interval or on events
        if (tick % LOG_INTERVAL != 0 && !eventTick) return;

        try {
            var sb = new StringBuilder(256);
            sb.append("{\"type\":\"tick\"");
            sb.append(",\"t\":").append(tick);
            if (eventTick) sb.append(",\"event\":true");
            sb.append(",\"subs\":[");

            for (int i = 0; i < submarines.size(); i++) {
                if (i > 0) sb.append(",");
                appendSub(sb, submarines.get(i));
            }

            sb.append("]}\n");
            writer.write(sb.toString());
        } catch (IOException e) {
            // Logging failure should not crash the simulation
            System.err.println("MatchRecorder write failed: " + e.getMessage());
        }
    }

    @Override
    public void onMatchEnd() {
        try {
            writer.write("{\"type\":\"end\"}\n");
            writer.flush();
            writer.close();
        } catch (IOException e) {
            System.err.println("MatchRecorder close failed: " + e.getMessage());
        }
    }

    private static void appendSub(StringBuilder sb, SubmarineSnapshot sub) {
        var pos = sub.pose().position();
        sb.append("{\"id\":").append(sub.id());
        sb.append(",\"x\":").append(fmt(pos.x()));
        sb.append(",\"y\":").append(fmt(pos.y()));
        sb.append(",\"z\":").append(fmt(pos.z()));
        sb.append(",\"hdg\":").append(fmt(Math.toDegrees(sub.pose().heading())));
        sb.append(",\"pitch\":").append(fmt(Math.toDegrees(sub.pose().pitch())));
        sb.append(",\"spd\":").append(fmt(sub.speed()));
        sb.append(",\"vz\":").append(fmt(sub.velocity().linear().z()));
        sb.append(",\"hp\":").append(sub.hp());
        sb.append(",\"noise\":").append(fmt(sub.noiseLevel()));
        if (sub.forfeited()) sb.append(",\"forfeited\":true");
        var status = sub.status();
        if (status != null && !status.isEmpty()) {
            sb.append(",\"status\":\"").append(escapeJson(status)).append("\"");
        }
        var estimates = sub.contactEstimates();
        if (estimates != null && !estimates.isEmpty()) {
            sb.append(",\"contacts\":[");
            for (int i = 0; i < estimates.size(); i++) {
                if (i > 0) sb.append(",");
                var ce = estimates.get(i);
                sb.append("{\"x\":").append(fmt(ce.x()));
                sb.append(",\"y\":").append(fmt(ce.y()));
                sb.append(",\"conf\":").append(fmt(ce.confidence()));
                sb.append(",\"alive\":").append(fmt(ce.contactAlive()));
                sb.append(",\"ur\":").append(fmt(ce.uncertaintyRadius()));
                if (!ce.label().isEmpty()) {
                    sb.append(",\"lbl\":\"").append(escapeJson(ce.label())).append("\"");
                }
                sb.append("}");
            }
            sb.append("]");
        }
        sb.append("}");
    }

    private static void appendVec3(StringBuilder sb, Vec3 v) {
        sb.append("[").append(fmt(v.x())).append(",")
          .append(fmt(v.y())).append(",").append(fmt(v.z())).append("]");
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.1f", v);
    }

    private static String escapeJson(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
