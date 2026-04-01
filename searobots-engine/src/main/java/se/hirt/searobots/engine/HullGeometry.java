/*
 * Copyright (C) 2026 Marcus Hirt
 */
package se.hirt.searobots.engine;

/**
 * Hull collision geometry: ellipsoid parameters and distance calculations
 * shared between the simulation loop, physics, and tests.
 */
public final class HullGeometry {

    private HullGeometry() {}

    // Submarine hull ellipsoid (collision + fuse check)
    public static final double SEMI_LENGTH = 38.0;   // bow-to-stern half-length
    public static final double SEMI_BEAM = 5.5;      // port-to-starboard half-width
    public static final double SEMI_HEIGHT = 4.5;    // keel-to-deck half-height
    public static final double AFT_OFFSET = -2.0;    // center shifted slightly aft

    /**
     * Distance from a point (px, py, pz) to the nearest point on a submarine's
     * hull ellipsoid. Returns 0 if the point is inside the ellipsoid.
     *
     * @param px point x (e.g. torpedo position)
     * @param py point y
     * @param pz point z
     * @param subX sub center x
     * @param subY sub center y
     * @param subZ sub center z
     * @param subHeading sub heading in radians
     * @param subPitch sub pitch in radians
     * @return distance to hull surface, or 0 if inside
     */
    public static double distanceToHull(double px, double py, double pz,
                                         double subX, double subY, double subZ,
                                         double subHeading, double subPitch) {
        double sinH = Math.sin(subHeading), cosH = Math.cos(subHeading);
        double sinP = Math.sin(subPitch), cosP = Math.cos(subPitch);
        double fwdX = sinH * cosP, fwdY = cosH * cosP, fwdZ = sinP;
        double rightX = cosH, rightY = -sinH;
        double upX = -sinH * sinP, upY = -cosH * sinP, upZ = cosP;

        // Offset sub center aft
        double cx = subX + fwdX * AFT_OFFSET;
        double cy = subY + fwdY * AFT_OFFSET;
        double cz = subZ + fwdZ * AFT_OFFSET;

        // Delta from sub center to point
        double dx = px - cx, dy = py - cy, dz = pz - cz;

        // Project into sub's local frame
        double localFwd = dx * fwdX + dy * fwdY + dz * fwdZ;
        double localRight = dx * rightX + dy * rightY;
        double localUp = dx * upX + dy * upY + dz * upZ;

        // Normalize by ellipsoid semi-axes
        double normFwd = localFwd / SEMI_LENGTH;
        double normRight = localRight / SEMI_BEAM;
        double normUp = localUp / SEMI_HEIGHT;
        double normDist = Math.sqrt(normFwd * normFwd + normRight * normRight + normUp * normUp);

        if (normDist <= 1.0) return 0; // inside ellipsoid

        // Distance from point to ellipsoid surface along the line from center
        double centerDist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        double ellipsoidRadius = centerDist / normDist;
        return centerDist - ellipsoidRadius;
    }

    /**
     * Convenience: distance from a torpedo entity to a submarine entity's hull.
     */
    public static double distanceToHull(TorpedoEntity torp, SubmarineEntity sub) {
        return distanceToHull(torp.x(), torp.y(), torp.z(),
                sub.x(), sub.y(), sub.z(), sub.heading(), sub.pitch());
    }

    /**
     * Distance from a torpedo's BOW (nose tip) to a submarine's hull surface.
     * The bow is the forward-most point of the torpedo cylinder, which is the
     * closest part of the torpedo to the target during approach.
     *
     * @param torpX torpedo center x
     * @param torpY torpedo center y
     * @param torpZ torpedo center z
     * @param torpHeading torpedo heading (radians)
     * @param torpPitch torpedo pitch (radians)
     * @param torpHalfLength torpedo half-length (from VehicleConfig.hullHalfLength)
     */
    public static double bowDistanceToHull(double torpX, double torpY, double torpZ,
                                            double torpHeading, double torpPitch,
                                            double torpHalfLength,
                                            double subX, double subY, double subZ,
                                            double subHeading, double subPitch) {
        // Torpedo bow = center + forward * halfLength
        double cosP = Math.cos(torpPitch), sinP = Math.sin(torpPitch);
        double bowX = torpX + Math.sin(torpHeading) * cosP * torpHalfLength;
        double bowY = torpY + Math.cos(torpHeading) * cosP * torpHalfLength;
        double bowZ = torpZ + sinP * torpHalfLength;
        return distanceToHull(bowX, bowY, bowZ, subX, subY, subZ, subHeading, subPitch);
    }

    /**
     * Convenience: bow distance from torpedo snapshot to submarine snapshot.
     */
    public static double bowDistanceToHull(TorpedoSnapshot torp, SubmarineSnapshot sub) {
        var tp = torp.pose();
        var sp = sub.pose();
        double halfLen = se.hirt.searobots.api.VehicleConfig.torpedo().hullHalfLength(); // 2.5m
        return bowDistanceToHull(
                tp.position().x(), tp.position().y(), tp.position().z(),
                tp.heading(), tp.pitch(), halfLen,
                sp.position().x(), sp.position().y(), sp.position().z(),
                sp.heading(), sp.pitch());
    }
}
