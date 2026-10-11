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

import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec3;

import java.util.ArrayList;
import java.util.List;

/** Frictionless terrain impulses; position projection is deliberately a separate operation. */
final class TerrainContactResponse {

	private static final double RESTITUTION = 0.05;
	private static final double CONTACT_SLOP = 1e-6;
	private static final double SPEED_TOLERANCE = 1e-8;
	private static final int MAX_ITERATIONS = 96;

	private TerrainContactResponse() {
	}

	record Impact(double closingSpeed, double energyJoules) {
	}

	private static final class Contact {
		final Vec3 offset, normal;
		final double inverseMass;
		double impulse;

		Contact(SubmarineEntity sub, Vec3 offset, Vec3 normal) {
			this.offset = offset;
			this.normal = normal;
			inverseMass = SubCollisionResponse.inverseContactMass(sub, offset, normal);
		}
	}

	/**
	 * Measures incoming energy once, then removes normal motion without snapping the hull's pitch.
	 */
	static Impact apply(
		SubmarineEntity sub, Vec3 position, Vec3[] localPoints, Vec3 incomingLinear, double incomingYawRate,
		double incomingPitchRate, TerrainMap terrain, CurrentField currents) {
		var contacts = contacts(sub, position, localPoints, terrain);
		var right = new Vec3(Math.cos(sub.heading()), -Math.sin(sub.heading()), 0);
		var angular = right.scale(incomingPitchRate).add(new Vec3(0, 0, -incomingYawRate));
		double closing = 0, energy = 0;
		for (var contact : contacts) {
			double inward = -incomingLinear.add(angular.cross(contact.offset)).dot(contact.normal);
			closing = Math.max(closing, inward);
			energy = Math.max(energy, TerrainImpactEnergy.energyJoules(sub.vehicleConfig(), contact.offset,
					contact.normal, sub.heading(), sub.pitch(), inward));
		}
		if (closing > SPEED_TOLERANCE || sub.hp() <= 0) {
			compress(sub, contacts, currents);
			if (sub.hp() > 0) {
				// Poisson restitution returns only a small fraction of the compression impulse.
				// A slow contact therefore has a proportionally small rebound.
				for (var contact : contacts) {
					SubCollisionResponse.applyImpulse(sub, contact.offset,
							contact.normal.scale(RESTITUTION * contact.impulse));
					contact.impulse = 0;
				}
				compress(sub, contacts, currents);
			}
		}
		return new Impact(closing, energy);
	}

	/**
	 * Zero-restitution support used in the coupled hull/terrain velocity solver; never charges
	 * damage.
	 */
	static double projectVelocity(SubmarineEntity sub, TerrainMap terrain, CurrentField currents) {
		var position = sub.pose().position();
		var points = HullGeometry.terrainContactPoints(position.add(new Vec3(0, 0, -CONTACT_SLOP)), sub.heading(),
				sub.pitch(), sub.vehicleConfig(), terrain);
		return compress(sub, contacts(sub, position, points, terrain), currents);
	}

	private static List<Contact> contacts(SubmarineEntity sub, Vec3 position, Vec3[] points, TerrainMap terrain) {
		var contacts = new ArrayList<Contact>();
		for (var local : points) {
			var world = HullTerrainGeometry.worldPoint(local, position, sub.heading(), sub.pitch());
			if (world.z() - terrain.elevationAt(world.x(), world.y()) > CONTACT_SLOP) {
				continue;
			}
			for (var normal : normals(world, terrain)) {
				var contact = new Contact(sub, world.subtract(position), normal);
				if (contact.inverseMass > 0) {
					contacts.add(contact);
				}
			}
		}
		return contacts;
	}

	private static List<Vec3> normals(Vec3 point, TerrainMap terrain) {
		double cell = terrain.getCellSize();
		double gx = (point.x() - terrain.getOriginX()) / cell;
		double gy = (point.y() - terrain.getOriginY()) / cell;
		int col = (int) Math.floor(gx), row = (int) Math.floor(gy);
		int otherCol = gx - col < 1e-9 ? col - 1 : col;
		int otherRow = gy - row < 1e-9 ? row - 1 : row;
		var normals = new ArrayList<Vec3>(4);
		// Bilinear cells have exact gradients. At a grid edge both adjacent faces can
		// support the hull; averaging across a sharp crest would erase its steep flank.
		for (int c = otherCol; c <= col; c++) {
			for (int r = otherRow; r <= row; r++) {
				double fx = Math.clamp(gx - c, 0, 1), fy = Math.clamp(gy - r, 0, 1);
				double e00 = terrain.elevationAtGrid(c, r), e10 = terrain.elevationAtGrid(c + 1, r);
				double e01 = terrain.elevationAtGrid(c, r + 1), e11 = terrain.elevationAtGrid(c + 1, r + 1);
				double slopeX = ((e10 - e00) * (1 - fy) + (e11 - e01) * fy) / cell;
				double slopeY = ((e01 - e00) * (1 - fx) + (e11 - e10) * fx) / cell;
				var normal = new Vec3(-slopeX, -slopeY, 1).normalize();
				if (normals.stream().noneMatch(existing -> existing.subtract(normal).length() < 1e-12)) {
					normals.add(normal);
				}
			}
		}
		return normals;
	}

	private static double compress(SubmarineEntity sub, List<Contact> contacts, CurrentField currents) {
		double maximumAppliedSpeed = 0;
		for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
			double maximum = 0;
			for (var contact : contacts) {
				double inward = -SubCollisionResponse.contactVelocity(sub, contact.offset, currents)
						.dot(contact.normal);
				double next = Math.max(0, contact.impulse + inward / contact.inverseMass);
				if (contact.impulse > 0 || next > 0) {
					maximum = Math.max(maximum, Math.abs(inward));
				}
				if (next != contact.impulse) {
					SubCollisionResponse.applyImpulse(sub, contact.offset,
							contact.normal.scale(next - contact.impulse));
				}
				contact.impulse = next;
			}
			maximumAppliedSpeed = Math.max(maximumAppliedSpeed, maximum);
			if (maximum <= SPEED_TOLERANCE) {
				break;
			}
		}
		return maximumAppliedSpeed;
	}
}
