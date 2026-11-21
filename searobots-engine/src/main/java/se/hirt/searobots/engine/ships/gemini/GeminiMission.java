/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine.ships.gemini;

import java.util.Locale;

/**
 * Compact mission briefing passed from {@link GeminiAttackSub} to {@link GeminiTorpedoController}
 * at launch time via {@code TorpedoLaunchCommand.missionData()}. After launch there is no further
 * communication between submarine and torpedo.
 */
public record GeminiMission(Mode mode, double targetX, double targetY, double targetZ, boolean depthKnown,
		double targetHeading, double targetSpeed, double uncertaintyRadius, double launcherX, double launcherY,
		double launcherZ, double launcherEscapeBearing, double waypointX, double waypointY, long launchTick) {

	public enum Mode {
		KILL, FINISHER, SNAP
	}

	public boolean hasWaypoint() {
		return Double.isFinite(waypointX) && Double.isFinite(waypointY);
	}

	public String encode() {
		return String.format(Locale.US, "v1;%s;%.1f;%.1f;%.1f;%d;%.4f;%.2f;%.1f;%.1f;%.1f;%.1f;%.4f;%.1f;%.1f;%d",
				mode.name(), targetX, targetY, targetZ, depthKnown ? 1 : 0,
				Double.isFinite(targetHeading) ? targetHeading : -999.0,
				Double.isFinite(targetSpeed) ? targetSpeed : -1.0, uncertaintyRadius, launcherX, launcherY, launcherZ,
				launcherEscapeBearing, Double.isFinite(waypointX) ? waypointX : 1e9,
				Double.isFinite(waypointY) ? waypointY : 1e9, launchTick);
	}

	public static GeminiMission decode(
		String raw, double fallbackX, double fallbackY, double fallbackZ, double fallbackHeading) {
		if (raw == null || !raw.startsWith("v1;")) {
			return fallback(fallbackX, fallbackY, fallbackZ, fallbackHeading);
		}
		try {
			String[] p = raw.split(";");
			if (p.length < 16)
				return fallback(fallbackX, fallbackY, fallbackZ, fallbackHeading);
			Mode mode = Mode.valueOf(p[1]);
			double tx = Double.parseDouble(p[2]);
			double ty = Double.parseDouble(p[3]);
			double tz = Double.parseDouble(p[4]);
			boolean depthKnown = Integer.parseInt(p[5]) != 0;
			double thRaw = Double.parseDouble(p[6]);
			double th = thRaw < -100 ? Double.NaN : thRaw;
			double ts = Double.parseDouble(p[7]);
			double unc = Double.parseDouble(p[8]);
			double lx = Double.parseDouble(p[9]);
			double ly = Double.parseDouble(p[10]);
			double lz = Double.parseDouble(p[11]);
			double esc = Double.parseDouble(p[12]);
			double wxRaw = Double.parseDouble(p[13]);
			double wyRaw = Double.parseDouble(p[14]);
			double wx = Math.abs(wxRaw) > 1e8 ? Double.NaN : wxRaw;
			double wy = Math.abs(wyRaw) > 1e8 ? Double.NaN : wyRaw;
			long tick = Long.parseLong(p[15]);
			return new GeminiMission(mode, tx, ty, tz, depthKnown, th, ts, unc, lx, ly, lz, esc, wx, wy, tick);
		} catch (RuntimeException e) {
			return fallback(fallbackX, fallbackY, fallbackZ, fallbackHeading);
		}
	}

	private static GeminiMission fallback(double x, double y, double z, double heading) {
		double tx = x + Math.sin(heading) * 1500.0;
		double ty = y + Math.cos(heading) * 1500.0;
		return new GeminiMission(Mode.SNAP, tx, ty, z, false, Double.NaN, -1.0, 500.0, x, y, z,
				GeminiMath.normalizeBearing(heading + Math.PI), Double.NaN, Double.NaN, 0L);
	}
}
