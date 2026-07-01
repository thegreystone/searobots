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
package se.hirt.searobots.engine.replay;

import java.util.Locale;

/**
 * Shared constants and field encoding for the SeaRobots replay format.
 * <p>
 * The format is line-oriented and tab-delimited, deliberately not JSON: a 50 Hz, 40-minute match produces ~120k frames,
 * and repeating field names on every line (as JSON does) would multiply file size and parse cost. Every line is parsed
 * with {@code line.split("\t")} in any language (Java for the existing viewers and tests, JavaScript for a future
 * browser visualizer), so no JSON parser is required on either side.
 * <p>
 * Each line begins with a single-character tag identifying the record type. Fixed-arity records put any free-text field
 * (status, label, phase) last; those strings are sanitized of tabs and newlines on write, so the split is always
 * unambiguous. Variable-length data (a sub's contact estimates and waypoints) is emitted as additional tagged child
 * lines following the sub.
 * <p>
 * <b>Layout</b>
 * <pre>
 *   SRREPLAY  &lt;version&gt;                 magic + format version (line 1)
 *   H   seed tickRate durationTicks ...   match header
 *   S   id name colorRGB spawnX spawnY spawnZ   one per submarine (definitions)
 *   T   &lt;tick&gt;                         begins a frame
 *   s   id ...fixed... status             submarine state (one per sub per frame)
 *   c   ...contact estimate...            child of the preceding s line
 *   w   ...waypoint...                    child of the preceding s line
 *   p   id owner colorRGB ... phase       torpedo state (one per torpedo per frame)
 *   E   reason                            end of match
 * </pre>
 * The reader dispatches on the version on line 1, so multiple format versions can be supported side by side.
 */
public final class ReplayFormat {

	private ReplayFormat() {
	}

	public static final String MAGIC = "SRREPLAY";

	/** Current format version written by {@link ReplayWriter}. */
	public static final int VERSION = 1;

	public static final String DELIM = "\t";

	// Record tags.
	public static final String TAG_HEADER = "H";
	public static final String TAG_SUBDEF = "S";
	public static final String TAG_TICK = "T";
	public static final String TAG_SUB = "s";
	public static final String TAG_CONTACT = "c";
	public static final String TAG_WAYPOINT = "w";
	public static final String TAG_TORPEDO = "p";
	public static final String TAG_END = "E";

	/**
	 * Formats a double compactly: integers print without a decimal point, other values keep up to four decimals with
	 * trailing zeros stripped. {@code NaN} round-trips as the literal {@code NaN}.
	 */
	public static String num(double v) {
		if (Double.isNaN(v)) {
			return "NaN";
		}
		if (Double.isInfinite(v)) {
			return v > 0 ? "Inf" : "-Inf";
		}
		if (v == Math.rint(v) && Math.abs(v) < 1e15) {
			return Long.toString((long) v);
		}
		String s = String.format(Locale.US, "%.4f", v);
		int end = s.length();
		while (s.charAt(end - 1) == '0') {
			end--;
		}
		if (s.charAt(end - 1) == '.') {
			end--;
		}
		return s.substring(0, end);
	}

	public static double parseD(String s) {
		return switch (s) {
			case "NaN" -> Double.NaN;
			case "Inf" -> Double.POSITIVE_INFINITY;
			case "-Inf" -> Double.NEGATIVE_INFINITY;
			default -> Double.parseDouble(s);
		};
	}

	public static String bool(boolean b) {
		return b ? "1" : "0";
	}

	public static boolean parseBool(String s) {
		return "1".equals(s);
	}

	/**
	 * Removes tabs and newlines from a free-text field so it cannot break the line structure. Such fields (status,
	 * label, phase) are always written last on their line.
	 */
	public static String sanitize(String s) {
		if (s == null || s.isEmpty()) {
			return "";
		}
		return s.replace('\t', ' ').replace('\n', ' ').replace('\r', ' ');
	}
}
