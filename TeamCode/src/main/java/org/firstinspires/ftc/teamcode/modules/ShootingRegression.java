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
    // STOPGAP 2026-11-15, not a fit: a flat 2400 ticks/s (~90% of the 2650-2675 measured at full power, 12.6 V;
    // operator: the hive needs ~100% power from a normal distance, and 2400 leaves headroom to recover after a
    // shot on a sagging battery). Replace with a Regression-tab fit. Was {0, 1}: ticks/s = inches, ~4% power.
    // 2026-11-16: pollen flywheel re-geared 24:20 (wheel 1.2x the motor) -> 35:21 (1.667x). The target is MOTOR
    // encoder ticks/s, so the same wheel speed is 2400 x 1.2 / (35/21) = 1728. Motor then at ~65% of its loaded max
    // (2650) instead of ~91% - room to recover after a shot and to hold speed on a sagging battery (model estimate,
    // FINDINGS 2026-11-15). Still a stopgap: re-fit on the Regression tab, and re-check the PIDF, at the new gearing.
    // 35:21 installed 2026-11-16 (operator; no flywheel weight yet): 2400 x 1.2 / (35/21) = 1728 - same wheel speed
    // as 2400 at 24:20. Re-fit on the Regression tab.
    public static double[] POLLEN_COEFFS = {1257.16, -2.78486, 0.0324061};

    public static double[] NECTAR_COEFFS = {0, 1};
    /** {min, max} distance (inches) the fit was recorded over. */
    public static double[] POLLEN_RANGE = {50.2, 103.6};
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
