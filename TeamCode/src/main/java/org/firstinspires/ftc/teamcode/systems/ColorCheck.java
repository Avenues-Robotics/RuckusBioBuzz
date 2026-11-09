package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

/**
 * Wrong-alliance piece detector for the intake.
 *
 * Rule: a piece is RED when red > RED_OVER_BLUE x blue, BLUE when blue > BLUE_OVER_RED x red, and
 * either call also needs the sensor distance under MAX_DIST_CM (a piece is actually there). Green is
 * ignored on purpose: this sensor reads green highest on everything, so the old "channel > 1.5 x
 * both others" rule could never call a red piece (red is only 0.68-1.30 x green on red pieces).
 *
 * TUNED 2026-11-03 from diagnostics/color (1).csv (Mechanism Tuner samples; surface/volts not
 * recorded): RED n=120, BLUE n=120, EMPTY n=210. Per label, min-max:
 *   red/blue  RED 1.096-2.150   BLUE 0.149-0.580   EMPTY 0.663-0.780
 *   blue/red  RED 0.465-0.913   BLUE 1.725-6.704   EMPTY 1.282-1.509
 *   dist cm   RED 2.41-5.88     BLUE 1.40-6.14     EMPTY 7.24-8.26
 * Each threshold sits about midway in its gap (geometric mean for the ratios). 0 of 450 samples
 * misclassified, and the ratio test and the distance gate each separate the data on their own.
 * Thinnest margin: BLUE_OVER_RED, ~7% to EMPTY and ~6% to BLUE; the distance gate covers EMPTY there.
 * Not yet sampled: a half-in piece, field tiles, a hand.
 */
public class ColorCheck {
    // Thresholds (RED_OVER_BLUE, BLUE_OVER_RED, MAX_DIST_CM): SystemConstants.ColorCheck.

    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private Alliance alliance;

    public ColorCheck(ColorSensor colorSensor, Alliance alliance) {
        this.colorSensor = colorSensor;
        this.distanceSensor = colorSensor instanceof DistanceSensor ? (DistanceSensor) colorSensor : null;  // This could use a comment
        this.alliance = alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }  // Not terrible to have setters, but in what context does alliance change after the color check object is created?

    /** True when the piece in the intake belongs to the other alliance. No alliance set: never. */
    public boolean reject() {
        if (alliance == null) return false;
        int r = colorSensor.red();
        int b = colorSensor.blue();
        boolean wrongColor = alliance == Alliance.BLUE ? isRed(r, b) : isBlue(r, b);
        // Distance is only read once the color already says reject, to keep I2C reads off the loop.
        return wrongColor && present(distanceCm());
    }

    private double distanceCm() {
        return distanceSensor == null ? Double.NaN : distanceSensor.getDistance(DistanceUnit.CM);
    }

    // Pure rule, shared with diagnostics/tuning/MechanismTuner so the tool cannot drift from it.
    public static boolean isRed(int r, int b) { return r > SystemConstants.ColorCheck.RED_OVER_BLUE * b; } // All of these methods are hard to read squeezed onto one line with no javadocs, comments, or spacing.
    public static boolean isBlue(int r, int b) { return b > SystemConstants.ColorCheck.BLUE_OVER_RED * r; }
    public static boolean present(double distCm) { return !(distCm >= SystemConstants.ColorCheck.MAX_DIST_CM); }
    public static boolean calledRed(int r, int b, double distCm) { return present(distCm) && isRed(r, b); }
    public static boolean calledBlue(int r, int b, double distCm) { return present(distCm) && isBlue(r, b); }
}
