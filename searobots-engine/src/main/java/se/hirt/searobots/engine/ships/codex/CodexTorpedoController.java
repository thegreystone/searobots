package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoLaunchContext;
import se.hirt.searobots.api.TorpedoOutput;

public final class CodexTorpedoController implements TorpedoController {
    private static final double MIN_TERRAIN_CLEARANCE = 35.0;
    private static final double MAX_CRUISE_DEPTH = -30.0;
    private static final double MAX_TARGET_DEPTH = -150.0;
    private static final long STRAIGHT_RUN_TICKS = 300L;
    private static final double STRAIGHT_RUN_DISTANCE = 450.0;
    private static final double STRAIGHT_RUN_RUDDER_LIMIT = 0.22;
    private static final double ACTIVE_SONAR_ARM_DISTANCE = 850.0;
    private static final double OWNER_AVOIDANCE_RADIUS = 850.0;
    private static final double OWNER_REJECTION_RUN_DISTANCE = 1_200.0;
    private static final double TARGET_GATE_RADIUS = 1_400.0;

    private TerrainMap terrain;
    private boolean hasTarget;
    private double targetX = Double.NaN;
    private double targetY = Double.NaN;
    private double targetZ = -100.0;
    private double estVelX = 0.0;
    private double estVelY = 0.0;
    private double prevFixX = Double.NaN;
    private double prevFixY = Double.NaN;
    private long prevFixTick = Long.MIN_VALUE / 4;
    private double passiveRangeEstimate = Double.NaN;
    private double launchX = Double.NaN;
    private double launchY = Double.NaN;
    private double launchHeading = Double.NaN;
    private long launchTick = Long.MIN_VALUE / 4;

    @Override
    public void onLaunch(TorpedoLaunchContext context) {
        this.terrain = context.terrain();
        this.launchX = context.launchPosition().x();
        this.launchY = context.launchPosition().y();
        this.launchHeading = context.launchHeading();
        String missionData = context.missionData();
        if (missionData == null || missionData.isBlank()) {
            return;
        }

        try {
            var parts = missionData.contains(";")
                    ? missionData.split(";")
                    : missionData.split(",");
            if (parts.length >= 2) {
                targetX = Double.parseDouble(parts[0].trim());
                targetY = Double.parseDouble(parts[1].trim());
                targetZ = parts.length >= 3 ? Double.parseDouble(parts[2].trim()) : -100.0;
                if (parts.length >= 5) {
                    double heading = Double.parseDouble(parts[3].trim());
                    double speed = Double.parseDouble(parts[4].trim());
                    if (!Double.isNaN(heading) && speed > 0.0) {
                        estVelX = Math.sin(heading) * speed;
                        estVelY = Math.cos(heading) * speed;
                    }
                }
                hasTarget = true;
            }
        } catch (NumberFormatException ignored) {
            hasTarget = false;
        }
    }

    @Override
    public void onTick(TorpedoInput input, TorpedoOutput output) {
        output.setThrottle(1.0);

        var pos = input.self().position();
        double heading = input.self().heading();
        if (launchTick <= Long.MIN_VALUE / 8) {
            launchTick = input.tick();
        }
        double runDistance = Math.hypot(pos.x() - launchX, pos.y() - launchY);
        boolean straightRun = inStraightRun(runDistance, input.tick());
        if (input.activeSonarCooldownTicks() == 0
                && !straightRun
                && runDistance >= ACTIVE_SONAR_ARM_DISTANCE) {
            output.activeSonarPing();
        }

        SonarContact fix = pickForwardContact(input.activeSonarReturns(), pos, heading);
        if (fix != null) {
            double fixX = pos.x() + Math.sin(fix.bearing()) * fix.range();
            double fixY = pos.y() + Math.cos(fix.bearing()) * fix.range();
            updateVelocityEstimate(input.tick(), fixX, fixY);
            targetX = fixX;
            targetY = fixY;
            passiveRangeEstimate = fix.range();
            if (!Double.isNaN(fix.estimatedDepth())) {
                targetZ = fix.estimatedDepth();
            }
            hasTarget = true;
        } else if (hasTarget) {
            SonarContact passive = pickForwardContact(input.sonarContacts(), pos, heading);
            if (passive != null) {
                double rangeGuess = Double.isNaN(passiveRangeEstimate)
                        ? 450.0
                        : Math.max(passiveRangeEstimate * 0.9, 250.0);
                targetX = pos.x() + Math.sin(passive.bearing()) * rangeGuess;
                targetY = pos.y() + Math.cos(passive.bearing()) * rangeGuess;
                if (!Double.isNaN(passive.estimatedHeading()) && passive.estimatedSpeed() > 0.0) {
                    estVelX = Math.sin(passive.estimatedHeading()) * passive.estimatedSpeed();
                    estVelY = Math.cos(passive.estimatedHeading()) * passive.estimatedSpeed();
                }
            }
        }

        if (!hasTarget) {
            output.setRudder(0.0);
            output.setSternPlanes(0.0);
            return;
        }

        double speed = Math.max(input.speed(), 8.0);
        double[] intercept = predictedIntercept(pos.x(), pos.y(), speed);
        double interceptX = intercept[0];
        double interceptY = intercept[1];
        double guidanceX = straightRun
                ? launchX + Math.sin(launchHeading) * STRAIGHT_RUN_DISTANCE
                : interceptX;
        double guidanceY = straightRun
                ? launchY + Math.cos(launchHeading) * STRAIGHT_RUN_DISTANCE
                : interceptY;
        double dx = guidanceX - pos.x();
        double dy = guidanceY - pos.y();
        double dist = Math.hypot(dx, dy);
        double bearingToIntercept = normalize(Math.atan2(dx, dy));

        double headingError = bearingToIntercept - heading;
        while (headingError > Math.PI) {
            headingError -= 2.0 * Math.PI;
        }
        while (headingError < -Math.PI) {
            headingError += 2.0 * Math.PI;
        }
        double rudderGain = straightRun ? 1.2 : dist < 250.0 ? 2.0 : 1.5;
        double rudderLimit = straightRun ? STRAIGHT_RUN_RUDDER_LIMIT : 1.0;
        output.setRudder(Math.clamp(headingError * rudderGain, -rudderLimit, rudderLimit));

        double desiredZ;
        if (fix != null && dist < 180.0) {
            desiredZ = pos.z();
        } else if (fix != null && dist < 450.0 && prevFixTick > Long.MIN_VALUE / 8) {
            desiredZ = pos.z() + (targetZ - pos.z()) * 0.05;
        } else {
            double cruiseTarget = Math.max(targetZ, MAX_TARGET_DEPTH);
            desiredZ = pos.z() + (cruiseTarget - pos.z()) * 0.015;
        }

        if (terrain != null && dist > 120.0) {
            double floor = terrain.elevationAt(pos.x(), pos.y());
            double safeZ = floor + MIN_TERRAIN_CLEARANCE;
            double lookAhead = Math.max(input.speed(), 5.0) * 4.0;
            double aheadFloor = terrain.elevationAt(
                    pos.x() + Math.sin(bearingToIntercept) * lookAhead,
                    pos.y() + Math.cos(bearingToIntercept) * lookAhead);
            safeZ = Math.max(safeZ, aheadFloor + MIN_TERRAIN_CLEARANCE + 10.0);
            if (desiredZ < safeZ) {
                desiredZ = safeZ;
            }
        }

        desiredZ = Math.min(desiredZ, MAX_CRUISE_DEPTH);
        double dz = desiredZ - pos.z();
        double pitchGain = dist < 250.0 ? 0.06 : 0.035;
        double pitchMax = dist < 250.0 ? 0.6 : 0.45;
        output.setSternPlanes(Math.clamp(dz * pitchGain, -pitchMax, pitchMax));

        if (fix != null && fix.range() < 12.0) {
            output.detonate();
        }
        output.publishTarget(guidanceX, guidanceY, targetZ);
    }

    private void updateVelocityEstimate(long tick, double fixX, double fixY) {
        if (!Double.isNaN(prevFixX) && prevFixTick > Long.MIN_VALUE / 8) {
            double dt = (tick - prevFixTick) / 50.0;
            if (dt > 0.25 && dt < 15.0) {
                double rawVX = (fixX - prevFixX) / dt;
                double rawVY = (fixY - prevFixY) / dt;
                estVelX = estVelX * 0.45 + rawVX * 0.55;
                estVelY = estVelY * 0.45 + rawVY * 0.55;
            }
        }
        prevFixX = fixX;
        prevFixY = fixY;
        prevFixTick = tick;
    }

    private SonarContact pickForwardContact(java.util.List<SonarContact> contacts,
                                            se.hirt.searobots.api.Vec3 pos,
                                            double heading) {
        SonarContact best = null;
        double bestScore = Double.NEGATIVE_INFINITY;
        for (SonarContact contact : contacts) {
            if (contact.isActive() && contact.range() <= 0.0) {
                continue;
            }
            double angleDiff = contact.bearing() - heading;
            while (angleDiff > Math.PI) {
                angleDiff -= 2.0 * Math.PI;
            }
            while (angleDiff < -Math.PI) {
                angleDiff += 2.0 * Math.PI;
            }
            double forwardness = Math.cos(angleDiff);
            double rangePenalty = contact.range() > 0.0 ? contact.range() * 0.01 : 0.0;
            if (!isPlausibleContact(contact, pos, heading)) {
                continue;
            }
            double score = forwardness * 100.0 + contact.signalExcess() - rangePenalty;
            if (hasTarget) {
                double contactRange = contact.range() > 0.0 ? contact.range() : Math.max(passiveRangeEstimate, 450.0);
                double cx = targetX;
                double cy = targetY;
                if (!Double.isNaN(contactRange)) {
                    cx = pos.x() + Math.sin(contact.bearing()) * contactRange;
                    cy = pos.y() + Math.cos(contact.bearing()) * contactRange;
                }
                double predictedError = Math.hypot(cx - targetX, cy - targetY);
                score -= predictedError * 0.03;
            }
            if (score > bestScore) {
                bestScore = score;
                best = contact;
            }
        }
        return best;
    }

    private double[] predictedIntercept(double ownX, double ownY, double ownSpeed) {
        double dist = Math.hypot(targetX - ownX, targetY - ownY);
        double timeToIntercept = Math.clamp(dist / ownSpeed, 1.0, 35.0);
        double leadScale = dist < 220.0 ? 0.35 : 1.0;
        return new double[]{
                targetX + estVelX * timeToIntercept * leadScale,
                targetY + estVelY * timeToIntercept * leadScale
        };
    }

    private boolean isPlausibleContact(SonarContact contact,
                                       se.hirt.searobots.api.Vec3 pos,
                                       double heading) {
        if (!hasTarget) {
            return true;
        }
        double rangeGuess = contact.range() > 0.0
                ? contact.range()
                : Double.isNaN(passiveRangeEstimate)
                ? 500.0
                : Math.max(passiveRangeEstimate, 300.0);
        double cx = pos.x() + Math.sin(contact.bearing()) * rangeGuess;
        double cy = pos.y() + Math.cos(contact.bearing()) * rangeGuess;
        double fromLaunch = Math.hypot(cx - launchX, cy - launchY);
        double fromTarget = Math.hypot(cx - targetX, cy - targetY);
        double runDistance = Math.hypot(pos.x() - launchX, pos.y() - launchY);
        double headingDiff = heading - launchHeading;
        while (headingDiff > Math.PI) headingDiff -= 2.0 * Math.PI;
        while (headingDiff < -Math.PI) headingDiff += 2.0 * Math.PI;
        boolean stillOutbound = Math.abs(headingDiff) < Math.toRadians(70.0);
        boolean ownerDangerZone = fromLaunch < OWNER_AVOIDANCE_RADIUS
                && runDistance < OWNER_REJECTION_RUN_DISTANCE;
        if ((stillOutbound || ownerDangerZone) && fromTarget > TARGET_GATE_RADIUS) {
            return false;
        }
        return fromTarget <= TARGET_GATE_RADIUS || contact.signalExcess() > 18.0;
    }

    private boolean inStraightRun(double runDistance, long tick) {
        return tick - launchTick < STRAIGHT_RUN_TICKS || runDistance < STRAIGHT_RUN_DISTANCE;
    }

    private double normalize(double bearing) {
        double result = bearing % (2.0 * Math.PI);
        if (result < 0.0) {
            result += 2.0 * Math.PI;
        }
        return result;
    }
}
