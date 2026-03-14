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

public sealed interface BattleArea {

    record Circular(double radius) implements BattleArea {}

    record Rectangular(double halfWidth, double halfHeight) implements BattleArea {}

    default boolean contains(double x, double y) {
        return switch (this) {
            case Circular(var r) -> x * x + y * y <= r * r;
            case Rectangular(var hw, var hh) -> Math.abs(x) <= hw && Math.abs(y) <= hh;
        };
    }

    default double distanceToBoundary(double x, double y) {
        return switch (this) {
            case Circular(var r) -> r - Math.sqrt(x * x + y * y);
            case Rectangular(var hw, var hh) -> Math.min(hw - Math.abs(x), hh - Math.abs(y));
        };
    }

    default double extent() {
        return switch (this) {
            case Circular(var r) -> r;
            case Rectangular(var hw, var hh) -> Math.sqrt(hw * hw + hh * hh);
        };
    }
}
