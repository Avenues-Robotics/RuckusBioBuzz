package org.firstinspires.ftc.teamcode.auto.modules;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.pipelines.PollenDetectionPipeline;

import java.util.List;
import java.util.Locale;

/**
 * Picks the pollen clump to drive into along a wall, from ONE camera frame's clumps (PollenDetectionPipeline).
 *
 * <p>Each clump is assumed to lie against the wall, its centre on the field line y = {@code lineY}. Its bearing from
 * the camera comes from its horizontal position in the upright frame (steeringError, -1 left .. +1 right) and the
 * frame's horizontal field of view (SystemConstants.Camera.HFOV_DEG); the ray from the camera (CAMERA_OFFSET on the
 * robot) is intersected with that line. Clumps whose wall x falls outside [xMin, xMax] (plus a ball's width) are
 * skipped. "Biggest" = pixel area x distance², so a far clump is not undercounted for looking smaller.
 *
 * <p>Pure math on its inputs; nothing here moves anything.
 */
public final class WallClumpPicker {
    private WallClumpPicker() {}

    public static final class Pick {
        /** Field x (in) to drive to along the wall, clamped to [xMin, xMax]. */
        public final double x;
        /** Clump's own field x before the clamp. */
        public final double clumpX;
        public final int balls;
        public final int candidates;

        Pick(double x, double clumpX, int balls, int candidates) {
            this.x = x;
            this.clumpX = clumpX;
            this.balls = balls;
            this.candidates = candidates;
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "x %.1f (clump %.1f, ~%d balls, %d candidates)", x, clumpX, balls, candidates);
        }
    }

    /** Null when no clump projects onto the wall inside the allowed x range. */
    public static Pick pick(Pose robot, List<PollenDetectionPipeline.Clump> clumps, double lineY, double xMin, double xMax) {
        if (robot == null || clumps == null || clumps.isEmpty()) return null;
        double[] off = SystemConstants.Camera.CAMERA_OFFSET;
        double c = Math.cos(robot.heading()), s = Math.sin(robot.heading());
        double camX = robot.x() + off[0] * c - off[1] * s;
        double camY = robot.y() + off[0] * s + off[1] * c;
        double halfFov = Math.toRadians(Math.max(1, Math.min(170, SystemConstants.Camera.HFOV_DEG)) / 2.0);

        PollenDetectionPipeline.Clump best = null;
        double bestScore = 0, bestX = 0;
        int candidates = 0;
        for (PollenDetectionPipeline.Clump k : clumps) {
            // steeringError +1 = right edge of the frame = clockwise of the camera axis.
            double bearing = robot.heading() - Math.atan(k.steeringError * Math.tan(halfFov));
            double dy = Math.sin(bearing);
            if (Math.abs(dy) < 1e-3) continue;              // parallel to the wall
            double dist = (lineY - camY) / dy;
            if (dist <= 0) continue;                        // the wall is behind this ray
            double x = camX + dist * Math.cos(bearing);
            if (x < xMin - 3 || x > xMax + 3) continue;     // off the allowed stretch (3 in ~ one ball's slack)
            candidates++;
            double score = k.areaPx * dist * dist;
            if (best == null || score > bestScore) {
                best = k;
                bestScore = score;
                bestX = x;
            }
        }
        if (best == null) return null;
        return new Pick(Math.max(xMin, Math.min(xMax, bestX)), bestX, best.estimatedBallCount, candidates);
    }
}
