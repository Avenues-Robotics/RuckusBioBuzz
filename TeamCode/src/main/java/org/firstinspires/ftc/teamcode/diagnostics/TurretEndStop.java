package org.firstinspires.ftc.teamcode.diagnostics;

import org.firstinspires.ftc.teamcode.SystemConstants;

/**
 * DIAGNOSTIC ONLY. Keeps raw servo test OpModes off the turret servos' travel ends.
 *
 * <p>2026-11-14 (operator): the turret mechanism's limits are the Axon MAX+'s own travel ends, and
 * on the full 500-2500 us range the servo can overshoot an end, cross its encoder wrap and drive
 * through the mechanism (it has broken it). The shipped turret code clamps to
 * {@code SystemConstants.Turret.*_TURRET_MIN/MAX}; the bench OpModes that drive a servo by name
 * did not, and two of them defaulted to position 0.0. This applies the same limits to them.
 */
public final class TurretEndStop {
    /**
     * Closest any free / limit-finding command may get to a servo end. 0.003 (1 deg) still skipped
     * on 2026-11-14; 0.018 is the shipped pollen stop (see SystemConstants.Turret).
     */
    public static final double END = 0.018;

    private TurretEndStop() {
    }

    public static boolean isTurret(String servoName) {
        return "pollenTurretServo".equals(servoName) || "nectarTurretServo".equals(servoName);
    }

    /** {@code p} for any other servo; for a turret servo, clamped to its shipped limits. */
    public static double clamp(String servoName, double p) {
        if ("pollenTurretServo".equals(servoName)) {
            return within(p, SystemConstants.Turret.POLLEN_TURRET_MIN, SystemConstants.Turret.POLLEN_TURRET_MAX);
        }
        if ("nectarTurretServo".equals(servoName)) {
            return within(p, SystemConstants.Turret.NECTAR_TURRET_MIN, SystemConstants.Turret.NECTAR_TURRET_MAX);
        }
        return p;
    }

    /** For limit finding: anywhere on the servo's travel except the last {@link #END} at each end. */
    public static double free(double p) {
        return Math.max(END, Math.min(1 - END, p));
    }

    private static double within(double p, double a, double b) {
        double lo = Math.max(END, Math.min(a, b)), hi = Math.min(1 - END, Math.max(a, b));
        return Math.max(lo, Math.min(hi, p));
    }
}
