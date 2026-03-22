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
 * A thermocline layer: a gradient band where temperature (and hence sound speed)
 * changes rapidly with depth. The layer extends from {@code top()} to {@code bottom()},
 * with the strongest gradient at {@code depth} (the core).
 *
 * @param depth      Z coordinate of the gradient core (negative, e.g. -120.0)
 * @param thickness  total thickness of the gradient band in metres (e.g. 50.0)
 * @param temperatureAbove water temperature above the layer (C)
 * @param temperatureBelow water temperature below the layer (C)
 */
public record ThermalLayer(double depth, double thickness,
                           double temperatureAbove, double temperatureBelow) {

    /** Compact constructor for backwards compatibility (zero thickness = sharp boundary). */
    public ThermalLayer(double depth, double temperatureAbove, double temperatureBelow) {
        this(depth, 0.0, temperatureAbove, temperatureBelow);
    }

    /** Top of the gradient band (shallowest, closest to surface). */
    public double top() { return depth + thickness / 2; }

    /** Bottom of the gradient band (deepest). */
    public double bottom() { return depth - thickness / 2; }

    /** Temperature difference across the layer. */
    public double gradient() { return Math.abs(temperatureAbove - temperatureBelow); }
}
