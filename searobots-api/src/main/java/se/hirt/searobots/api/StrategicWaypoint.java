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
 * A high-level mission waypoint set by the tactical AI. Specifies
 * where and why the submarine should go, without dictating how.
 *
 * @param x              world X coordinate (meters)
 * @param y              world Y coordinate (meters)
 * @param preferredDepth preferred operating depth (negative, meters below sea level)
 * @param purpose        tactical reason for heading to this position
 * @param noise          noise discipline policy
 * @param pattern        movement pattern to use en route
 * @param arrivalRadius  distance in meters at which the waypoint is considered reached
 * @param targetSpeed    desired speed in m/s, or -1 if the autopilot should decide
 */
public record StrategicWaypoint(
        double x,
        double y,
        double preferredDepth,
        Purpose purpose,
        NoisePolicy noise,
        MovementPattern pattern,
        double arrivalRadius,
        double targetSpeed
) {
    public StrategicWaypoint {
        if (arrivalRadius <= 0) arrivalRadius = 200;
        if (noise == null) noise = NoisePolicy.NORMAL;
        if (pattern == null) pattern = MovementPattern.DIRECT;
        if (purpose == null) purpose = Purpose.PATROL;
    }
}
