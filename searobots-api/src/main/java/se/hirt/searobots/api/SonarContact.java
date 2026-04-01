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
package se.hirt.searobots.api;

/**
 * A sonar contact: what a submarine or torpedo knows about a detected entity.
 *
 * @param bearing              absolute bearing in radians [0, 2pi)
 * @param signalExcess         dB above detection threshold (higher = stronger/closer)
 * @param range                meters (active sonar returns only, 0 for passive)
 * @param isActive             true if this contact came from an active sonar return
 * @param estimatedSpeed       target speed estimate from blade-rate tonals (m/s, -1 if unavailable)
 * @param bearingUncertainty   1-sigma bearing error in radians, computed by the sonar model
 * @param rangeUncertainty     1-sigma range error in meters (active returns only, 0 for passive)
 * @param estimatedSourceLevel estimated source level in dB, derived from the acoustic
 *                             signature. Enables classification (surface ships are much
 *                             louder than submarines) and more accurate SE-based ranging.
 * @param solutionQuality      TMA solution quality (0.0 to 1.0), equivalent to Cold Waters SOL%
 * @param estimatedHeading     estimated target heading in radians [0, 2pi), or NaN if quality too low
 * @param estimatedDepth       estimated target depth in meters (negative = below surface),
 *                             derived from vertical bearing angle and range for active returns.
 *                             NaN if unavailable (passive contacts).
 * @param classification       acoustic classification of the contact, based on blade-rate
 *                             tonals, broadband signature, and speed. Requires sufficient
 *                             signal excess to classify; faint contacts are UNKNOWN.
 */
public record SonarContact(double bearing, double signalExcess, double range, boolean isActive,
                           double estimatedSpeed, double bearingUncertainty,
                           double rangeUncertainty, double estimatedSourceLevel,
                           double solutionQuality, double estimatedHeading,
                           double estimatedDepth, Classification classification) {

    /**
     * Acoustic classification of a sonar contact. Determined by the sonar
     * model from the target's noise signature (blade rate, broadband level,
     * cavitation pattern). Classification improves with signal excess:
     * faint contacts can only be classified as UNKNOWN.
     */
    public enum Classification {
        /** Too faint to classify (low signal excess). */
        UNKNOWN,
        /** Submarine: moderate noise, moderate speed, submerged. */
        SUBMARINE,
        /** Surface ship: high broadband noise, slow large prop, at surface depth. */
        SURFACE_SHIP,
        /** Torpedo: very high blade rate, high speed, small signature. */
        TORPEDO
    }

    /** Constructor without classification (defaults to UNKNOWN). */
    public SonarContact(double bearing, double signalExcess, double range, boolean isActive,
                        double estimatedSpeed, double bearingUncertainty,
                        double rangeUncertainty, double estimatedSourceLevel,
                        double solutionQuality, double estimatedHeading,
                        double estimatedDepth) {
        this(bearing, signalExcess, range, isActive, estimatedSpeed, bearingUncertainty,
             rangeUncertainty, estimatedSourceLevel, solutionQuality, estimatedHeading,
             estimatedDepth, Classification.UNKNOWN);
    }

    /** Constructor without depth or classification. */
    public SonarContact(double bearing, double signalExcess, double range, boolean isActive,
                        double estimatedSpeed, double bearingUncertainty,
                        double rangeUncertainty, double estimatedSourceLevel,
                        double solutionQuality, double estimatedHeading) {
        this(bearing, signalExcess, range, isActive, estimatedSpeed, bearingUncertainty,
             rangeUncertainty, estimatedSourceLevel, solutionQuality, estimatedHeading,
             Double.NaN, Classification.UNKNOWN);
    }
}
