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
 * A sonar contact: what a submarine knows about a detected entity.
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
 */
public record SonarContact(double bearing, double signalExcess, double range, boolean isActive,
                           double estimatedSpeed, double bearingUncertainty,
                           double rangeUncertainty, double estimatedSourceLevel,
                           double solutionQuality, double estimatedHeading) {}
