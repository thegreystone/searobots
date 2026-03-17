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
 * A submarine's declaration that it has a viable torpedo firing solution
 * on a target. Published via {@link SubmarineOutput#publishFiringSolution}
 * for viewer visualization and match recording.
 *
 * <p>The controller decides when it has a solution based on its own
 * criteria (range, uncertainty, environment, target geometry, etc.).
 * Different controllers may use different criteria.
 *
 * @param targetX         estimated target world X coordinate (meters)
 * @param targetY         estimated target world Y coordinate (meters)
 * @param targetHeading   estimated target heading in radians [0, 2pi), or NaN if unknown
 * @param targetSpeed     estimated target speed in m/s, or -1 if unknown
 * @param quality         solution quality 0.0 (marginal) to 1.0 (excellent)
 */
public record FiringSolution(double targetX, double targetY,
                              double targetHeading, double targetSpeed,
                              double quality) {
    public FiringSolution {
        quality = Math.clamp(quality, 0.0, 1.0);
    }
}
