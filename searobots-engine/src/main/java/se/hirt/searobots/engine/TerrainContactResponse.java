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
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/** Frictionless terrain impulses; position projection is deliberately a separate operation. */
final class TerrainContactResponse {

	private static final double RESTITUTION = 0.05;
	private static final double CONTACT_SLOP = 1e-6;
	private static final double SPEED_TOLERANCE = 1e-8;
	private static final int MAX_ITERATIONS = 96;
	private static final int MAX_SUPPORT_ITERATIONS = 8;

	private TerrainContactResponse() {
	}

	record Impact(double closingSpeed, double energyJoules) {
	}

	private static final class Contact {
		final Vec3 offset, normal;
		final double inverseMass;
		double targetSpeed;
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

	/**
	 * Supports an already-contacting wreck over a finite interval. A tangent velocity at the
	 * current pose can still intersect the seabed as its curved hull rotates or the terrain normal
	 * changes. Speculative endpoint contacts supply only the separating impulse needed by that
	 * interval within terrain cells already supporting the hull. The caller still sweeps the
	 * resulting path for newly reached terrain and intervening obstacles.
	 */
	static int projectSupportVelocity(
		SubmarineEntity sub, Vec3[] supportedPoints, double seconds, TerrainMap terrain, CurrentField currents) {
		var position = sub.pose().position();
		double heading = sub.heading(), pitch = sub.pitch();
		var current = currents.currentAt(position.z());
		var supportedCells = new HashSet<TerrainCell>();
		for (var local : supportedPoints) {
			var world = HullTerrainGeometry.worldPoint(local, position, heading, pitch);
			if (world.z() - terrain.elevationAt(world.x(), world.y()) <= CONTACT_SLOP) {
				var cells = terrainCells(world, terrain);
				// An edge or corner already supports the hull on every adjacent face.
				for (int c = cells.firstColumn(); c <= cells.lastColumn(); c++) {
					for (int r = cells.firstRow(); r <= cells.lastRow(); r++) {
						supportedCells.add(new TerrainCell(c, r));
					}
				}
			}
		}
		int iterations = 0;
		for (int iteration = 0; iteration < MAX_SUPPORT_ITERATIONS; iteration++) {
			iterations++;
			var linear = sub.velocity().linear().add(new Vec3(current.x(), current.y(), 0));
			var end = position.add(linear.scale(seconds));
			double endHeading = heading + sub.yawRate() * seconds;
			double endPitch = Math.clamp(pitch + sub.pitchRate() * seconds, -Math.PI / 4, Math.PI / 4);
			var points = HullGeometry.terrainContactPoints(end, endHeading, endPitch, sub.vehicleConfig(), terrain);
			var contacts = new ArrayList<Contact>();
			double maximumPenetration = 0;
			for (var local : points) {
				var world = HullTerrainGeometry.worldPoint(local, end, endHeading, endPitch);
				var normals = normals(world, terrain, supportedCells);
				if (normals.isEmpty()) {
					continue;
				}
				double penetration = terrain.elevationAt(world.x(), world.y()) - world.z();
				maximumPenetration = Math.max(maximumPenetration, penetration);
				if (penetration < -CONTACT_SLOP) {
					continue;
				}
				var offset = HullTerrainGeometry.worldPoint(local, position, heading, pitch).subtract(position);
				for (var normal : normals) {
					var contact = new Contact(sub, offset, normal);
					if (contact.inverseMass > 0) {
						// The slop only widens contact detection. A late swept contact can leave an
						// interval of nanoseconds; dividing a position margin by it would turn the
						// margin into an unbounded impulse. The predicted penetration itself is
						// bounded by the closing speed over the interval.
						contact.targetSpeed = SubCollisionResponse.contactVelocity(sub, offset, currents).dot(normal)
								+ Math.max(0, penetration) * normal.z() / seconds;
						contacts.add(contact);
					}
				}
			}
			if (maximumPenetration <= 0) {
				break;
			}
			compress(sub, contacts, currents);
		}
		return iterations;
	}

	private record TerrainCell(int column, int row) {
	}

	private record TerrainCells(int firstColumn, int lastColumn, int firstRow, int lastRow) {
	}

	private static TerrainCells terrainCells(Vec3 point, TerrainMap terrain) {
		double cell = terrain.getCellSize();
		double gx = (point.x() - terrain.getOriginX()) / cell;
		double gy = (point.y() - terrain.getOriginY()) / cell;
		int col = (int) Math.floor(gx), row = (int) Math.floor(gy);
		int edgeCol = (int) Math.rint(gx), edgeRow = (int) Math.rint(gy);
		boolean onColumnEdge = Math.abs(gx - edgeCol) < 1e-9;
		boolean onRowEdge = Math.abs(gy - edgeRow) < 1e-9;
		// Shape transforms can round an exact grid edge to either side of its integer.
		return new TerrainCells(onColumnEdge ? edgeCol - 1 : col, onColumnEdge ? edgeCol : col,
				onRowEdge ? edgeRow - 1 : row, onRowEdge ? edgeRow : row);
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
		return normals(point, terrain, null);
	}

	private static List<Vec3> normals(Vec3 point, TerrainMap terrain, Set<TerrainCell> supportedCells) {
		double cell = terrain.getCellSize();
		double gx = (point.x() - terrain.getOriginX()) / cell;
		double gy = (point.y() - terrain.getOriginY()) / cell;
		var cells = terrainCells(point, terrain);
		var normals = new ArrayList<Vec3>(4);
		// Bilinear cells have exact gradients. At a grid edge both adjacent faces can
		// support the hull; averaging across a sharp crest would erase its steep flank.
		for (int c = cells.firstColumn(); c <= cells.lastColumn(); c++) {
			for (int r = cells.firstRow(); r <= cells.lastRow(); r++) {
				if (supportedCells != null && !supportedCells.contains(new TerrainCell(c, r))) {
					continue;
				}
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
				double inward = contact.targetSpeed
						- SubCollisionResponse.contactVelocity(sub, contact.offset, currents).dot(contact.normal);
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
