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

import java.util.Objects;
import java.util.Optional;

/**
 * Optional controller-side memory and prediction for one associated sonar contact. The engine has
 * already done the acoustic analysis and TMA; this helper only consumes the public estimates. The
 * controller selects observations, decides whether they belong to this track, and calls
 * {@link #reset()} when switching targets or starting a match. No enemy identity or world state is
 * used.
 * <p>
 * Advance the clock with {@link #update(Vec3, SonarContact, double)} once per controller tick,
 * passing {@code null} during contact loss. Additional observations at that same time may use zero
 * elapsed seconds. A usable ranged observation replaces the position solution and its reported
 * uncertainty; correlated observations are never treated as independent samples. Between fixes the
 * mean follows the last reported heading and speed. A fresh report with unresolved motion clears
 * that prediction, whereas an absent report preserves it.
 * <p>
 * Uncertainty is a conservative display envelope: an ellipse enclosing twice the reported range and
 * bearing errors, expanded by a bounded motion model. It is not a calibrated probability or an
 * exact reachable set. Unknown depth allows any horizontal projection of the slant range. Known
 * depth uses a two-sigma allowance of {@code max(10 m, 10% of range)}, matching the submarine
 * sonar's depth-error model. The motion limits and initial speed/heading error allowances are
 * controller assumptions, not information about the target. They should include allowances for
 * currents and any faster vehicle types being tracked. Reported tonal speed is relative to the
 * water, while TMA heading follows ground motion: their product is an approximate horizontal
 * velocity, with current and sway error covered by the configured motion allowances.
 * <p>
 * This class is mutable and intended for a single controller thread. It never expires a track or
 * decides whether a contact is still alive; those are controller policies.
 */
public final class SonarTrack {

	/**
	 * Horizontal motion assumptions: speed in m/s, acceleration in m/s squared, turn rate in rad/s,
	 * and initial speed and heading error allowances in m/s and radians. All values must be finite
	 * and nonnegative; maximum speed must be positive.
	 */
	public record MotionModel(double maxSpeed, double maxAcceleration, double maxTurnRate, double speedUncertainty,
			double headingUncertainty) {
		public MotionModel {
			if (!Double.isFinite(maxSpeed) || maxSpeed <= 0)
				throw new IllegalArgumentException("Maximum speed must be finite and positive");
			requireNonnegative(maxAcceleration, "Maximum acceleration");
			requireNonnegative(maxTurnRate, "Maximum turn rate");
			requireNonnegative(speedUncertainty, "Speed uncertainty");
			requireNonnegative(headingUncertainty, "Heading uncertainty");
		}

		/**
		 * Conservative starter assumptions, adjustable by controllers: 1 m/s squared acceleration,
		 * 15 degrees/s turn rate, speed error max(2 m/s, 20% of maximum speed), and 30 degrees
		 * heading error. These are tracking assumptions, not hard limits imposed by the physics
		 * engine.
		 */
		public static MotionModel submarine(double maxSpeed) {
			return new MotionModel(maxSpeed, 1, Math.toRadians(15), Math.max(2, maxSpeed * 0.2), Math.toRadians(30));
		}
	}

	/**
	 * Last position fix, in world coordinates, unchanged during coasting or motion-only updates.
	 * Observation is null for explicit world-position evidence supplied via
	 * {@link #observePosition(Vec2, Ellipse, double, double, double)}.
	 */
	public record Fix(double x, double y, double timeSeconds, SonarContact observation) {
	}

	/**
	 * Ellipse centred on the estimate, with radii in metres and major-axis heading measured from
	 * north (positive Y) toward east (positive X). At long gaps the motion allowance dominates and
	 * the ellipse approaches a circle.
	 */
	public record Ellipse(double majorRadius, double minorRadius, double heading) {
		/** Radius of a circle enclosing the ellipse, suitable for the existing viewer. */
		public double uncertaintyRadius() {
			return majorRadius;
		}
	}

	/**
	 * Current predicted world position and uncertainty. Negative speed or NaN heading means
	 * unresolved motion. Quality is the latest engine solution quality, reduced as the envelope
	 * grows without a position fix; it is a heuristic score, not a probability. Contact age
	 * measures any sonar evidence, whereas fix age measures usable ranging evidence.
	 */
	public record Estimate(double x, double y, double estimatedHeading, double estimatedSpeed,
			double secondsSinceContact, double secondsSinceFix, double solutionQuality, Ellipse uncertainty) {
		/**
		 * Prepare a viewer estimate using the controller's separate belief that the contact is
		 * alive. Display confidence combines that belief with the track's quality score.
		 */
		public ContactEstimate toContactEstimate(double contactAlive, String label) {
			if (!Double.isFinite(contactAlive))
				throw new IllegalArgumentException("Contact-alive belief must be finite");
			double alive = Math.clamp(contactAlive, 0, 1);
			return new ContactEstimate(x, y, alive * solutionQuality, alive, uncertainty.uncertaintyRadius(),
					estimatedHeading, estimatedSpeed, label);
		}
	}

	private final MotionModel model;
	private double timeSeconds;
	private double lastContactTime;
	private Fix lastFix;
	private double anchorX, anchorY, anchorTime;
	private double heading = Double.NaN, speed = -1;
	private double quality;
	private Ellipse anchorUncertainty;

	public SonarTrack(double maxSpeed) {
		this(MotionModel.submarine(maxSpeed));
	}

	public SonarTrack(MotionModel model) {
		this.model = Objects.requireNonNull(model, "model");
	}

	/**
	 * Advance the clock by elapsed simulation seconds, then ingest the selected observation at the
	 * supplied observer position. Delta time may be zero (e.g. an initial fix) and must be finite
	 * and nonnegative. Pass null for an absent observation. Bearing-only or unbounded-range
	 * readings refresh contact age but cannot establish or tighten a Cartesian fix. The returned
	 * snapshot is empty until a usable ranged observation arrives. Position must be finite.
	 */
	public Optional<Estimate> update(Vec3 observerPosition, SonarContact observation, double deltaTimeSeconds) {
		requireNonnegative(deltaTimeSeconds, "Delta time");
		Objects.requireNonNull(observerPosition, "observerPosition");
		if (!Double.isFinite(observerPosition.x()) || !Double.isFinite(observerPosition.y())
				|| !Double.isFinite(observerPosition.z()) || !Double.isFinite(timeSeconds + deltaTimeSeconds))
			throw new IllegalArgumentException("Position and simulation time must be finite");
		validatePrediction(timeSeconds + deltaTimeSeconds);
		timeSeconds += deltaTimeSeconds;
		if (observation != null) {
			lastContactTime = timeSeconds;
			var prior = estimate();
			boolean ranged = usableRange(observation);
			if (ranged) {
				double depth = usableDepth(observation, observerPosition.z());
				double distance = horizontalRange(observation.range(), depth, observerPosition.z());
				double x = observerPosition.x() + distance * Math.sin(observation.bearing());
				double y = observerPosition.y() + distance * Math.cos(observation.bearing());
				ranged = Double.isFinite(x) && Double.isFinite(y);
				if (ranged) {
					lastFix = new Fix(x, y, timeSeconds, observation);
					anchorX = x;
					anchorY = y;
					anchorUncertainty = observationUncertainty(observation, observerPosition.z(), distance);
					quality = reportedQuality(observation);
				}
			}
			if (!ranged && prior.isPresent()) {
				// Preserve the already-grown envelope; an unranged report supplies no position fix.
				var old = prior.get();
				anchorX = old.x();
				anchorY = old.y();
				anchorUncertainty = old.uncertainty();
				quality = Math.min(old.solutionQuality(), reportedQuality(observation));
			}
			if (lastFix != null) {
				anchorTime = timeSeconds;
				setMotion(observation.estimatedHeading(), observation.estimatedSpeed());
			}
		}
		return estimate();
	}

	public Optional<Fix> lastFix() {
		return Optional.ofNullable(lastFix);
	}

	/** Obtain a snapshot without advancing simulation time. */
	public Optional<Estimate> estimate() {
		return estimateAt(timeSeconds);
	}

	/**
	 * Forecast from the current evidence without advancing the live clock or replacing its fix. The
	 * horizon must be finite and nonnegative. The returned ages and uncertainty describe the
	 * forecast time, so controllers can distinguish a future aim point from a current sensor fix.
	 */
	public Optional<Estimate> predict(double secondsAhead) {
		requireNonnegative(secondsAhead, "Prediction horizon");
		double forecastTime = timeSeconds + secondsAhead;
		if (!Double.isFinite(forecastTime))
			throw new IllegalArgumentException("Prediction time must be finite");
		validatePrediction(forecastTime);
		return estimateAt(forecastTime);
	}

	/**
	 * Ingest a world-position estimate derived by the controller from public evidence, for example
	 * an explosion bearing and range. The caller supplies its uncertainty envelope and motion
	 * assumptions. This neither labels the evidence as active sonar nor associates it with a
	 * target. Call after advancing to the observation time with
	 * {@link #update(Vec3, SonarContact, double)}. Position, ellipse axes/orientation, and quality
	 * must be finite; quality is clamped to [0,1]. Unresolved heading or speed has the same meaning
	 * as on a sonar report.
	 */
	public Optional<Estimate> observePosition(
		Vec2 position, Ellipse uncertainty, double estimatedHeading, double estimatedSpeed, double solutionQuality) {
		Objects.requireNonNull(position, "position");
		Objects.requireNonNull(uncertainty, "uncertainty");
		if (!Double.isFinite(position.x()) || !Double.isFinite(position.y()) || !Double.isFinite(solutionQuality)
				|| !Double.isFinite(uncertainty.heading()))
			throw new IllegalArgumentException("Position, ellipse heading, and quality must be finite");
		requireNonnegative(uncertainty.majorRadius(), "Major radius");
		requireNonnegative(uncertainty.minorRadius(), "Minor radius");
		if (uncertainty.minorRadius() > uncertainty.majorRadius())
			throw new IllegalArgumentException("Major radius must be at least the minor radius");
		lastFix = new Fix(position.x(), position.y(), timeSeconds, null);
		anchorX = position.x();
		anchorY = position.y();
		anchorTime = lastContactTime = timeSeconds;
		anchorUncertainty = new Ellipse(uncertainty.majorRadius(), uncertainty.minorRadius(),
				normalize(uncertainty.heading()));
		quality = Math.clamp(solutionQuality, 0, 1);
		setMotion(estimatedHeading, estimatedSpeed);
		return estimate();
	}

	/**
	 * Apply associated motion evidence without manufacturing a position fix. Retains the current
	 * predicted position and its already-grown uncertainty, and preserves lastFix and its age.
	 * Refreshes contact age, since a new motion report is evidence of continued contact. Repeated
	 * reports do not reduce the accumulated position envelope. Empty until a position fix exists.
	 */
	public Optional<Estimate> observeMotion(double estimatedHeading, double estimatedSpeed) {
		var current = estimate();
		if (current.isEmpty())
			return current;
		var estimate = current.get();
		anchorX = estimate.x();
		anchorY = estimate.y();
		anchorUncertainty = estimate.uncertainty();
		quality = estimate.solutionQuality();
		anchorTime = lastContactTime = timeSeconds;
		setMotion(estimatedHeading, estimatedSpeed);
		return estimate();
	}

	private Optional<Estimate> estimateAt(double estimateTime) {
		if (lastFix == null)
			return Optional.empty();
		double age = estimateTime - anchorTime;
		boolean resolved = speed >= 0 && (speed == 0 || Double.isFinite(heading));
		double vx = resolved && speed > 0 ? speed * Math.sin(heading) : 0;
		double vy = resolved && speed > 0 ? speed * Math.cos(heading) : 0;
		double expansion = motionExpansion(age, resolved);
		var ellipse = expand(anchorUncertainty, expansion);
		double scale = Math.max(1, anchorUncertainty.majorRadius());
		double decayedQuality = quality * scale / Math.max(scale, ellipse.majorRadius());
		return Optional.of(new Estimate(anchorX + vx * age, anchorY + vy * age, heading, speed,
				estimateTime - lastContactTime, estimateTime - lastFix.timeSeconds(), decayedQuality, ellipse));
	}

	private void validatePrediction(double predictionTime) {
		if (lastFix != null && speed > 0 && Double.isFinite(heading)) {
			double age = predictionTime - anchorTime;
			if (!Double.isFinite(anchorX + speed * Math.sin(heading) * age)
					|| !Double.isFinite(anchorY + speed * Math.cos(heading) * age))
				throw new IllegalArgumentException("Prediction time exceeds finite track coordinates");
		}
	}

	private void setMotion(double estimatedHeading, double estimatedSpeed) {
		heading = Double.isFinite(estimatedHeading) ? normalize(estimatedHeading) : Double.NaN;
		speed = Double.isFinite(estimatedSpeed) && estimatedSpeed >= 0 ? Math.min(estimatedSpeed, model.maxSpeed())
				: -1;
	}

	/** Clear all evidence and restart the helper's simulation clock. */
	public void reset() {
		timeSeconds = lastContactTime = anchorTime = 0;
		lastFix = null;
		anchorX = anchorY = quality = 0;
		anchorUncertainty = null;
		heading = Double.NaN;
		speed = -1;
	}

	private static boolean usableRange(SonarContact observation) {
		return Double.isFinite(observation.bearing()) && Double.isFinite(observation.range()) && observation.range() > 0
				&& Double.isFinite(observation.rangeUncertainty()) && observation.rangeUncertainty() >= 0
				&& observation.rangeUncertainty() < Double.MAX_VALUE
				&& Double.isFinite(observation.bearingUncertainty()) && observation.bearingUncertainty() >= 0;
	}

	private static double horizontalRange(double range, double depth, double ownDepth) {
		if (!Double.isFinite(depth))
			return range;
		// Factored form avoids squaring the slant range and catastrophic cancellation near vertical.
		double ratio = Math.min(1, Math.abs(depth - ownDepth) / range);
		return range * Math.sqrt((1 - ratio) * (1 + ratio));
	}

	private static double usableDepth(SonarContact c, double ownDepth) {
		// Conflicting range/depth intervals cannot establish a precise horizontal projection.
		double separation = Math.abs(c.estimatedDepth() - ownDepth);
		double maximum = bounded(c.range() + bounded(2 * c.rangeUncertainty()) + Math.max(10, c.range() * 0.1));
		return Double.isFinite(separation) && separation <= maximum ? c.estimatedDepth() : Double.NaN;
	}

	private static Ellipse observationUncertainty(SonarContact c, double ownDepth, double horizontal) {
		double rangeError = bounded(2 * c.rangeUncertainty());
		double upperRange = bounded(c.range() + rangeError);
		double lowerRange = Math.max(0, c.range() - rangeError);
		double lowerHorizontal = 0, upperHorizontal = upperRange;
		double depth = usableDepth(c, ownDepth);
		if (Double.isFinite(depth)) {
			double depthError = Math.max(10, c.range() * 0.1);
			double separation = Math.abs(depth - ownDepth);
			lowerHorizontal = horizontalRange(lowerRange, separation + depthError, 0);
			upperHorizontal = horizontalRange(upperRange, Math.max(0, separation - depthError), 0);
		}
		double radialError = Math.max(horizontal - lowerHorizontal, upperHorizontal - horizontal);
		double angle = Math.min(Math.PI, 2 * c.bearingUncertainty());
		double along = bounded(radialError + upperHorizontal * (1 - Math.cos(angle)));
		double across = upperHorizontal * Math.sin(Math.min(Math.PI / 2, angle));
		// Enclose the polar-error sector with a rectangle, then enclose that rectangle by an ellipse.
		double a = bounded(Math.sqrt(2) * along), b = bounded(Math.sqrt(2) * across);
		double circle = bounded(radialError + upperHorizontal * (2 * Math.sin(angle / 2)));
		if (Math.max(a, b) > circle)
			return new Ellipse(circle, circle, 0);
		return a >= b ? new Ellipse(a, b, normalize(c.bearing()))
				: new Ellipse(b, a, normalize(c.bearing() + Math.PI / 2));
	}

	private double motionExpansion(double age, boolean resolved) {
		if (!resolved)
			return bounded(model.maxSpeed() * age);
		// |v_actual - v_nominal| <= speed error + heading error + bounded acceleration/turning,
		// capped by maxSpeed + nominalSpeed. Integrate this bound analytically, independent of dt.
		double cap = bounded(model.maxSpeed() + speed);
		double initial = Math.min(cap,
				model.speedUncertainty() + speed * (2 * Math.sin(Math.min(Math.PI, model.headingUncertainty()) / 2)));
		double growth = bounded(model.maxAcceleration() + speed * model.maxTurnRate());
		if (growth == 0)
			return bounded(initial * age);
		double saturatedAt = (cap - initial) / growth;
		if (age <= saturatedAt)
			return bounded(initial * age + 0.5 * growth * age * age);
		return bounded((0.5 * initial + 0.5 * cap) * saturatedAt + cap * (age - saturatedAt));
	}

	private static Ellipse expand(Ellipse ellipse, double radius) {
		if (radius == 0)
			return ellipse;
		double a = ellipse.majorRadius(), b = ellipse.minorRadius();
		if (a == 0)
			return new Ellipse(radius, radius, 0);
		// Outer ellipsoid of the Minkowski sum of the old ellipse and a motion-error disk.
		// Simply adding radius to both axes would fail to enclose the sum near the diagonals.
		double reference = Math.hypot(a / Math.sqrt(2), b / Math.sqrt(2));
		if (reference == 0)
			return new Ellipse(bounded(a + radius), bounded(a + radius), 0);
		double cross = Math.sqrt(radius) * Math.sqrt(reference);
		return new Ellipse(bounded(Math.hypot(Math.hypot(a, radius), bounded(cross * Math.hypot(1, a / reference)))),
				bounded(Math.hypot(Math.hypot(b, radius), bounded(cross * Math.hypot(1, b / reference)))),
				ellipse.heading());
	}

	private static double reportedQuality(SonarContact observation) {
		return Double.isFinite(observation.solutionQuality()) ? Math.clamp(observation.solutionQuality(), 0, 1) : 0;
	}

	private static double bounded(double value) {
		return Math.min(Double.MAX_VALUE, value);
	}

	private static double normalize(double angle) {
		double result = angle % (2 * Math.PI);
		return result < 0 ? result + 2 * Math.PI : result;
	}

	private static void requireNonnegative(double value, String name) {
		if (!Double.isFinite(value) || value < 0)
			throw new IllegalArgumentException(name + " must be finite and nonnegative");
	}
}
