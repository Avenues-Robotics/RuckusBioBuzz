package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.systems.Turret;

/**
 * Flywheel target velocity from the turret-to-hive distance:
 * {@code ticks/s = c[0] + c[1]·d + c[2]·d² + ...}, d in inches (the distance {@link Turret} aims with).
 *
 * <p>The coefficients and ranges are fitted on the Mechanism Tuner's <b>Regression</b> tab
 * ({@code /tune}) and spliced in by {@code tools/mechtune/mechtune.py export --only regression --write}.
 * The splicer replaces each right-hand side by field name, so keep the declarations one per line.
 *
 * <p>The distance is clamped to the range the fit was recorded over before it is evaluated: a
 * polynomial extrapolated past its data can run away in either direction.
 */
public class ShootingRegression {

    // ---- fitted values (written by mechtune.py export --only regression --write) ----
    public static double[] POLLEN_COEFFS = {0, 1};
    public static double[] NECTAR_COEFFS = {0, 1};
    /** {min, max} distance (inches) the fit was recorded over. */
    public static double[] POLLEN_RANGE = {0, 1000};
    public static double[] NECTAR_RANGE = {0, 1000};

    private final Turret aiming;

    public ShootingRegression(Turret aiming) {
        this.aiming = aiming;
    }

    public double shooterspeedpollen() {
        return evaluate(POLLEN_COEFFS, POLLEN_RANGE, pollenDistance());
    }

    public double shooterspeednectar() {
        return evaluate(NECTAR_COEFFS, NECTAR_RANGE, nectarDistance());
    }

    public double pollenDistance() {
        return aiming.aimPollen().distance;
    }

    public double nectarDistance() {
        return aiming.aimNectar().distance;
    }

    /** Polynomial in d (Horner), d clamped to range; never negative. Shared with the Mechanism Tuner. */
    public static double evaluate(double[] coeffs, double[] range, double distance) {
        double d = distance;
        if (range != null && range.length == 2) {
            d = Math.max(Math.min(range[0], range[1]), Math.min(Math.max(range[0], range[1]), d));
        }
        double v = 0;
        for (int i = coeffs.length - 1; i >= 0; i--) {
            v = v * d + coeffs[i];
        }
        return Math.max(0, v);
    }
}
