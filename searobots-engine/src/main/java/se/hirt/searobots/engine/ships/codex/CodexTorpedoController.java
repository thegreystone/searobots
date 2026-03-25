package se.hirt.searobots.engine.ships.codex;

import se.hirt.searobots.api.SonarContact;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.TorpedoController;
import se.hirt.searobots.api.TorpedoInput;
import se.hirt.searobots.api.TorpedoLaunchContext;
import se.hirt.searobots.api.TorpedoOutput;

public final class CodexTorpedoController implements TorpedoController {
    private static final double MIN_TERRAIN_CLEARANCE = 30.0;
    private static final double MAX_CRUISE_DEPTH = -30.0;
    private static final double MAX_TARGET_DEPTH = -150.0;

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

    @Override
    public void onLaunch(TorpedoLaunchContext context) {
        this.terrain = context.terrain();
        String missionData = context.missionData();
        if (missionData == null || missionData.isBlank()) {
            return;
        }

        try {
            var parts = missionData.split(",");
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

        if (input.activeSonarCooldownTicks() == 0) {
            output.activeSonarPing();
        }

        SonarContact fix = pickForwardContact(input.activeSonarReturns(), heading);
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
            SonarContact passive = pickForwardContact(input.sonarContacts(), heading);
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

        double dx = targetX - pos.x();
        double dy = targetY - pos.y();
        double dist = Math.hypot(dx, dy);
        double speed = Math.max(input.speed(), 8.0);
        double timeToIntercept = Math.clamp(dist / speed, 1.0, 35.0);
        double leadScale = dist < 220.0 ? 0.35 : 1.0;
        double interceptX = targetX + estVelX * timeToIntercept * leadScale;
        double interceptY = targetY + estVelY * timeToIntercept * leadScale;

        double bearingToIntercept = Math.atan2(interceptX - pos.x(), interceptY - pos.y());
        if (bearingToIntercept < 0.0) {
            bearingToIntercept += 2.0 * Math.PI;
        }

        double headingError = bearingToIntercept - heading;
        while (headingError > Math.PI) {
            headingError -= 2.0 * Math.PI;
        }
        while (headingError < -Math.PI) {
            headingError += 2.0 * Math.PI;
        }
        double rudderGain = dist < 250.0 ? 2.0 : 1.5;
        output.setRudder(Math.clamp(headingError * rudderGain, -1.0, 1.0));

        double desiredZ;
        if (dist < 180.0) {
            desiredZ = pos.z();
        } else if (dist < 450.0 && prevFixTick > Long.MIN_VALUE / 8) {
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
                    pos.x() + Math.sin(heading) * lookAhead,
                    pos.y() + Math.cos(heading) * lookAhead);
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
        output.publishTarget(interceptX, interceptY, targetZ);
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

    private SonarContact pickForwardContact(java.util.List<SonarContact> contacts, double heading) {
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
            double score = forwardness * 100.0 + contact.signalExcess() - rangePenalty;
            if (score > bestScore) {
                bestScore = score;
                best = contact;
            }
        }
        return best;
    }
}
