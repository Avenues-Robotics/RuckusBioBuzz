package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing constants - the one file, as in the Pedro Quickstart. Swerve only: the mecanum
 * constants and the config.jsonc drivetrain switch were removed 2026-10-01 (they are in git
 * history before that date).
 *
 * <p>Layout, top to bottom: TUNED VALUES (the block a Swerve Bring-Up export rewrites), then
 * follower constants (Foresight), localizer constants (Pinpoint), drivetrain constants (swerve +
 * pods), and the factories. Every OpMode builds its follower through
 * {@link #createFollower(HardwareMap)}.
 *
 * <p>The tuned values are one declaration each so {@code tools/swervetune/robot.py constants}
 * can replace them by name, leaving every provenance comment in this file where it is. Keep
 * them that way: an initialiser the splicer cannot find is reported, not guessed at.
 */
public class Constants {
    // =======================================================================================
    // TUNED VALUES. Rewritten by name from a Swerve Bring-Up export (robot.py constants).
    // Provenance for each lives with the section that uses it, below.
    // =======================================================================================

    // Follower (Foresight). The same set, and the same names, as Pedro's Foresight Tuner prints
    // (robot.py foresight imports its output). An initialiser naming PLACEHOLDER_MAX_VELOCITY is
    // NOT a measurement - the visualizer refuses to import it, and so should you.
    private static final double PLACEHOLDER_MAX_VELOCITY = 73.9;
    public static final double maxAchievableForwardVelocity = PLACEHOLDER_MAX_VELOCITY;
    public static final double maxAchievableStrafeVelocity = PLACEHOLDER_MAX_VELOCITY;
    public static final double naturalForwardDeceleration = 40.0;
    public static final double naturalStrafeDeceleration = 40.0;
    public static final double headingKP = 7.7173;
    public static final double headingBrakeLinear = 0.050501;
    public static final double headingBrakeQuadratic = 0.019853;
    public static final double forwardBrakeLinear = 0.0;
    public static final double strafeBrakeLinear = 0.0;
    public static final double forwardBrakeQuadratic = 1.0 / (2 * naturalForwardDeceleration);
    public static final double strafeBrakeQuadratic = 1.0 / (2 * naturalStrafeDeceleration);
    public static final double coastKV = 1.0 / PLACEHOLDER_MAX_VELOCITY;
    public static final double brakeKV = 1.0 / PLACEHOLDER_MAX_VELOCITY;
    // Translational: primary kP above translationalSwitchIn of error, secondary below it (the
    // tuner's piecewise shape). Equal primary/secondary builds ONE PID, exactly as before.
    public static final double forwardTranslationalPrimaryKP = 0.26;
    public static final double forwardTranslationalSecondaryKP = 0.26;
    public static final double strafeTranslationalPrimaryKP = 0.26;
    public static final double strafeTranslationalSecondaryKP = 0.26;
    public static final double translationalSwitchIn = 2.5;
    public static final double translationalKI = 0.0;
    // 0.025 -> 0 (2026-11-15): the D term acts on Foresight's PROJECTED error (pose + a v^2 brake displacement),
    // which jumps with velocity noise every loop; holds commanded 4-8x full scale and reversed the drive motors at
    // full voltage (the operator's 'violent jolts'). Hold A/B, n=10/arm randomised interleaved: drive-power
    // reversals per hold pair 77.7 (SD 15.6) -> 2.3 (SD 3.0), diff 75.4 [64.8, 86.0], Welch t=15.0; peak command
    // 5.1 -> 1.28; hold end error 0.22 -> 0.34 in. P only, as the Foresight Tuner identifies it.
    public static final double translationalKD = 0.0;

    // Localizer (Pinpoint), inches
    public static final double pinpointXPodOffset = -2.84;
    public static final double pinpointYPodOffset = -5.988;
    public static final boolean pinpointXPodReversed = false;
    public static final boolean pinpointYPodReversed = false;

    // Pods, indexed by servo number ss0..ss3 (0=RB, 1=RF, 2=LF, 3=LB)
    // Pods REBUILT 2026-11-14 (~530 deg/s): kD/kS/kS band and the pulse settings are the floor
    // candidate "F", accepted 2026-11-15 (FINDINGS). Gain history further down is the old plant.
    public static final double[] turnKPPerPod = {0.380, 0.380, 0.380, 0.380};
    public static final double[] turnKDPerPod = {0.015, 0.015, 0.015, 0.015};
    public static final double[] turnKSPerPod = {0.080, 0.080, 0.080, 0.080};
    public static final double[] turnKSBandDegPerPod = {6.0, 6.0, 6.0, 6.0};
    public static final double[] podZeroDeg = {210.108701, 246.728678, 153.233917, 327.317849};
    public static final double[] podMinV = {0.083, 0.073, 0.145, 0.164};
    public static final double[] podMaxV = {3.301, 3.302, 3.341, 3.391};
    public static final boolean[] podDriveReversed = {true, false, true, true};
    public static final String[] podEncoderNames = {"se0", "se2", "se3", "se1"};
    public static final double turnServoCaching = 0.01;

    // Pulsed final approach, shared by all four pods (see buildPod)
    public static final double pulseBandDeg = 3.5;
    public static final double pulseToleranceDeg = 3.0;
    public static final double pulsePower = 0.045;
    public static final double pulseSeconds = 0.020;
    public static final double pulseStationaryDegPerSec = 20.0;
    public static final double pulseCoastSeconds = 0.10;

    // Steering response while driving - robot-wide statics, applied by createSwerve (applySteeringStatics), so
    // TeleOp and the autos both run them. 2026-11-15, 2x2 tiles, heading hold off (TeleOp has none):
    // stick circle (circle.py) + stick-flick square (square.py), FINDINGS "TeleOp steering".
    public static final double turnScheduleFloor = 0.30;
    public static final double turnDemandRateKV = 0.045;
    public static final double turnDemandRateTauS = 0.015;
    public static final double turnDemandRateMaxDegS = 240.0;
    public static final double demandSlewDegPerSec = 300.0;

    // =======================================================================================
    // FOLLOWER CONSTANTS
    // =======================================================================================

    // ---------------------------------------------------------------------------------------
    // PATH FOLLOWING (Pedro 3 "Foresight"). NOT MEASURED ON THIS ROBOT - see FORESIGHT_MEASURED.
    // ---------------------------------------------------------------------------------------
    //
    // Pedro 3.0.0 replaced the 2.x PIDF follower with Foresight, which plans against a braking
    // model and needs constants the 2.x follower never had. Their status, field by field:
    //
    //   naturalForward/StrafeDeceleration  40 in/s^2   MEASURED 2026-08-14, the coast-down below
    //                                                  (same quantity the v3 tuner measures).
    //   maxAchievableForward/StrafeVelocity 73.9 in/s  UNMEASURED. Carried from the 2.x
    //                                                  SwerveConstants.velocity(73.9), which has
    //                                                  been there since the 2026-05-14 skeleton,
    //                                                  before the pods existed.
    //   quadraticBrakeCoefficients 1/(2*40)            DERIVED, not measured: stopping distance
    //   linearBrakeCoefficients 0                      under pure coast, v^2/(2a). Foresight also
    //                                                  brakes actively, so this over-predicts the
    //                                                  stopping distance - it brakes early, the
    //                                                  safe direction.
    //   headingBrakeCoefficients 0.0505 / 0.0199       MEASURED 2026-11-15, Foresight box tuner heading
    //                                                  braking (12 trials, 0.2-1.0 power, tiles); was 0,
    //                                                  which disabled heading lookahead. Validated with
    //                                                  headingKP below (pedrocheck pc2x2_candB).
    //   coast / brake kV 1/73.9                        UNMEASURED. Power = v / vmax, the naive
    //                                                  feed-forward.
    //   headingFeedback kP 7.7173                      MEASURED 2026-11-15, Foresight box tuner heading ID
    //                                                  (n=3: 6.53 / 7.89 / 8.73; K 6.45-6.61 rad/s/power,
    //                                                  tau 0.127-0.173 s - a later run gave 13.8, the plant
    //                                                  is not stationary). P only, as before. Was 1.20,
    //                                                  carried from 2.x: turns overshot 8-9.5 deg and sat
    //                                                  3.7-4.5 deg off. pedrocheck shipped vs this (+ the
    //                                                  heading braking above), 5 blocks interleaved, tiles:
    //                                                  turn overshoot 9.5/8.0 -> 0.50/0.67 deg, residual
    //                                                  3.7/4.5 -> 0.16/0.20 deg, settle 1.04 s; hold
    //                                                  position residual unchanged (0.32-0.34 vs 0.34-0.38).
    //   forward/strafeTranslational P 0.26             CARRIED from the 2.x translational PIDF,
    //                                                  2026-08-14. Same units (power per inch). Its kD
    //                                                  0.025 was removed 2026-11-15 (see translationalKD).
    //                                                  Holds stall 0.3 in short under the ~0.19 drive
    //                                                  breakaway. 2x2 hold A/B 2026-11-15 (FINDINGS): a
    //                                                  secondary kP of 0.40 / 0.55 cuts that to 0.20 / 0.11
    //                                                  in but adds ~4 pod flips per stop. Kept at 0.26: the
    //                                                  secondary is the path cross-track gain, so it waits
    //                                                  for path validation on the long strip.
    //
    // Measure all of it with the Foresight Tuner (Tuning.java - REMOVED 2026-11-01 with Sloth; it
    // survives in git history at cf5dcee9^ - no branch kept it), import it into TUNED VALUES with
    // robot.py foresight, VALIDATE IT THROUGH PEDRO (TUNING_TASK.md stage 6), and only then set
    // FORESIGHT_MEASURED by hand. No tool sets it.
    public static final boolean FORESIGHT_MEASURED = false;

    /**
     * Call before following a path or holding a point. Refuses to let Foresight drive on the
     * placeholder constants above - an auto on invented numbers is a robot at speed on a guess.
     */
    public static void requireForesightMeasured() {
        if (!FORESIGHT_MEASURED && dryRunLocalizer == null) {   // dry run: simulated pose, robot on blocks
            throw new IllegalStateException("Foresight constants in Constants are "
                    + "placeholders (Pedro 3 migration). Run the Foresight Tuner, paste its "
                    + "values, then set FORESIGHT_MEASURED = true.");
        }
    }

    /**
     * The translational controller for one axis: Pedro's Foresight Tuner shape - primary kP at or
     * above {@code switchIn} of error, secondary below - or a single PID when the two kP are
     * equal, which is what shipped before the tuner was run (one controller, one derivative
     * history). Public so the bring-up tool's follower bench builds exactly this.
     */
    public static Controller translationalController(double primaryKP, double secondaryKP,
            double kI, double kD, double switchIn) {
        if (primaryKP == secondaryKP) {
            return Controller.pid(primaryKP, kI, kD);
        }
        return Controller.piecewise(Controller.pid(secondaryKP, kI, kD))
                .put(switchIn, Controller.pid(primaryKP, kI, kD));
    }

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.proportional(headingKP));
        c.forwardTranslational.set(translationalController(forwardTranslationalPrimaryKP,
                forwardTranslationalSecondaryKP, translationalKI, translationalKD,
                translationalSwitchIn));
        c.strafeTranslational.set(translationalController(strafeTranslationalPrimaryKP,
                strafeTranslationalSecondaryKP, translationalKI, translationalKD,
                translationalSwitchIn));

        c.coast.set(Controller.proportionalFeedforward(coastKV));
        c.brake.set(Controller.proportionalFeedforward(brakeKV));

        c.maxAchievableForwardVelocity.set(maxAchievableForwardVelocity);
        c.maxAchievableStrafeVelocity.set(maxAchievableStrafeVelocity);
        c.naturalForwardDeceleration.set(naturalForwardDeceleration);
        c.naturalStrafeDeceleration.set(naturalStrafeDeceleration);

        c.linearBrakeCoefficients.set(Matrix.diag(forwardBrakeLinear, strafeBrakeLinear));
        c.quadraticBrakeCoefficients.set(Matrix.diag(forwardBrakeQuadratic, strafeBrakeQuadratic));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(headingBrakeLinear,
                headingBrakeQuadratic));
    });

    // 2.x FOLLOWER HISTORY. The Pedro 2.1.2 follower these were tuned on no longer exists; kept
    // because the measurements are real and the carried values above point here.
    //
    // Measured 2026-08-14 by coast-down on the tiles (accelerate, cut power, fit the velocity
    // decay): -40 in/s^2, against the -197.1 that had been configured - the follower was assuming
    // 5x more passive braking than the drivetrain has, which is why every path arrival overshot
    // by 4-7 inches regardless of drive PIDF. Swerve is isotropic (the pods point wherever the
    // motion goes), so forward and lateral share the value.
    //
    // Translational retuned 2026-08-14 through the bring-up tool's follower bench (pedrotune.py),
    // odometry pods live, 8 in lateral hold-point steps on the tiles: 0.125/0.008 settled in
    // 1.39 s; 0.26/0.025 settles in 0.68 s with ZERO overshoot over n=8 and 0.32 in residual.
    // 0.34/0.035 was no faster - the step is velocity-limited - so the lower gain wins on
    // robustness.
    //
    // Heading measured 2026-08-11 on the Pinpoint IMU (no odometry pods needed), across commanded
    // rotations of 15/45/90/135/180 degrees in both directions.
    //
    // kP: below ~1.0 the robot never settles (0.80 left 3-4 degrees standing); 1.40 hits a
    // stability edge, overshooting 22 then 51 degrees. 1.20 settles in ~0.65s.
    // kD: 0.003 overshot 8-12 degrees on anything past 45 degrees, because the output
    // saturates on large errors and the robot arrives carrying momentum. 0.030 cuts that to
    // 3-5 degrees for ~0.5 degrees more residual, and leaves small rotations clean
    // (15 degrees: no overshoot, settles in 0.22s).
    // kF stays 0: like the pod turn PIDF it is applied as a sign-only relay, and 0.06
    // produced 23-26 oscillations per step.
    //
    // Verified holding heading to +/-3 degrees while translating through two full circles.
    //
    // 2026-10-01: Foresight's heading controller is P only and is now tuned THROUGH the follower
    // (bring-up pedroPidf hp=..., validated with pedrocheck.py). The tool's heading-hold PIDF
    // below is its own control law (kD, sign feed-forward) and does not validate Foresight.
    //
    // If you change these, change SwerveBringUp's headingKp/headingKd to match. The
    // bring-up tool keeps its own copy for live experimentation and does not read this
    // file; it sat at the pre-tuning 1.75/0.003 for a week after this line moved to
    // 1.20/0.030, so anyone opening the heading routine was measuring against a
    // baseline the robot had stopped using.
    //
    // kD 0.030 -> 0.080 on 2026-08-13 late: the loop now runs ~2-3x faster and the pod
    // turn loop underneath is much snappier (per-pod kP 0.38-0.44 vs 0.20), so 0.030
    // overshot a 90 deg heading step by 11-15 deg. At 0.080 the same step settles in
    // 0.56 s with 2.3 deg worst overshoot and 20 deg steps do not overshoot at all.
    // Measured through the bring-up tool's headingGoto harness (tools/swervetune/
    // headtune.py, results in current_runs/headtune.jsonl); path-following has not yet
    // been re-validated at this value.
    //
    // Drive PIDF retuned 2026-08-14 with the honest zero-power acceleration in place (the two
    // interact: at ZPA -197 every candidate overshot 4-7 in because the follower braked late;
    // at the measured -40, 24 in lines at 0.7 power arrive in 1.56 s with 0.40 in worst
    // overshoot and zero oscillation): FilteredPIDF 0.008 / 0 / 0.0001 / 0.6 / 0.10. Foresight
    // has no drive PIDF; its coast/brake controllers replace it.
    //
    // Centripetal was 0.0005: swept 0.0-0.003 on 12 in curves at 0.7 power on 2026-08-14 and
    // the tracking error would not separate - the tuning box is too small to sustain the
    // v^2 * curvature regime the term corrects. Foresight has no centripetal term. Mass was
    // 13.732 kg.

    // =======================================================================================
    // LOCALIZER CONSTANTS
    // =======================================================================================

    // Calibrated 2026-08-14 against reality, the day the pods were physically attached. The
    // values that had been sitting here were never measured on this robot: both encoder
    // directions were backwards (forward drove x negative, strafe-left drove y negative) and
    // the two offsets appear to have been SWAPPED - rotating 90 degrees in place orbited the
    // pose by 18 inches. Directions fixed by nudge tests; offsets solved by paired +/-45 deg
    // rotations (slip bias cancels in the pairing) and a linear fit to the fake-translation
    // rate. Verified residual: ~0.6 in of orbit per 45 degrees of rotation, which is the
    // measurement noise floor of the method.
    //
    // Pedro 3 mapping, checked against both PinpointLocalizer sources: 2.x forwardPodY and
    // strafePodX went to GoBildaPinpointDriver.setOffsets(x, y) in that order, exactly as
    // xPodOffset / yPodOffset do now, and forward/strafe directions are x/y directions.
    //
    // resetMode NONE: the 2.x localizer never recalibrated the IMU at construction (it only
    // wrote the pose to 0,0,0 - see DriveTeleOp.init). v3's default RECALIBRATE_IMU would add
    // an IMU recalibration to every OpMode init.
    public static PinpointConfig pinpointConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(pinpointXPodOffset);
        c.yPodOffset.set(pinpointYPodOffset);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(pinpointXPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(pinpointYPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });

    // =======================================================================================
    // DRIVETRAIN CONSTANTS
    // =======================================================================================

    // Pods snap to an X with no input, so the robot resists being pushed. Note this makes any
    // momentary drop to zero input visible as all four pods yanking to the X - which is correct
    // behaviour, but during bring-up it hides what the pods are doing. Swap to
    // IGNORE_ANGLE_CHANGES while diagnosing, then put this back.
    //
    // manualBrakeMode is the 2.x useBrakeModeInTeleOp(true). Voltage compensation stays off, as
    // it was (and in 2.x it was a silent no-op even when on).
    public static SwerveConfig swerveConfig = new SwerveConfig(c -> {
        c.voltageCompensation.set(false);
        c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.X_LOCK);
        c.manualBrakeMode.set(true);
    });

    // Zero offsets, analog ranges, encoder pairings and drive directions below come from a
    // Swerve Bring-Up session on 2026-08-10, verified driving.
    //
    // ---------------------------------------------------------------------------------------
    // TURN GAINS: retuned 2026-08-13 on the FTC game tiles after a pod repair and lubrication
    // of all four steering paths. Still NOT validated on a drained battery (criterion 12).
    // ---------------------------------------------------------------------------------------
    //
    // Pedro 3 migration: the pod control loop is carried over unchanged (same PIDFController
    // arithmetic, same kS/pulse/schedule/flip/slew code). Checked on the host by driving the
    // 2.1.2 and 3.0.0 CoaxialPod + Swerve side by side through identical inputs and simulated
    // plants: servo commands agree to 1e-14 over 1200 steps at 8, 10 and 25 ms loops. Every
    // number below still describes the code that runs. Since 2026-10-10 the mixer/pod differ
    // from 2.1.2 in release, flip-skip and stop() handling (RUCKUS_PATCHES.md); turn-gain step
    // tests drive pod.move() directly and are unaffected, driving-session numbers predate them.
    //
    // Everything below the 2026-08-12 block was measured before the mechanical work and no
    // longer describes this robot's friction. Kept because the negative results are still
    // valid and re-running them would cost days.
    //
    // Post-lube retune, all on the tiles at 12.9-13.6 V, randomised and interleaved:
    //
    //   kD: 0.022 confirmed. At kS 0.050, raising kD made things worse, not better -
    //       4/31 loose at 0.022, 7/30 at 0.030, 24/32 at 0.040. Damping does not buy back
    //       the wide mode.
    //   kS: 0.035 confirmed against 0.050 at n=60/53, Fisher exact p=0.025 - 1.7% loose
    //       against 13.2%, worst peak-to-peak 7.98 deg against 47.67. This REVERSES the
    //       pre-lube result where 0.045 beat 0.035 on the same tiles. The story is
    //       consistent: lubrication cut kinetic friction, so a kS sized to static breakaway
    //       (measured 0.050 on tiles) now overdrives the pod once it is moving. kS wants to
    //       track the kinetic regime, which is why the measured breakaway is the wrong target.
    //   kP: 0.200 -> 0.320. Residual falls monotonically and unambiguously across 534
    //       pod-runs: 3.24 / 2.64 / 1.86 / 1.09 deg mean at kP 0.200 / 0.230 / 0.260 / 0.320.
    //
    // The reason kP went UP rather than down, which is the opposite of the usual instinct:
    // large excursions (>15 deg post-settle peak-to-peak) occur at EVERY kP tested - 8 out of
    // 534 pod-runs overall, including one 30.7 deg event at kP 0.200 and a 43.4 deg event at
    // 0.230. Their rate does not separate by kP (0.5 / 1.0 / 2.4 / 3.3 percent, all four 95%
    // intervals overlapping). Backing kP off therefore buys no measurable protection while
    // costing the one criterion still being missed. The point estimates do trend upward, so
    // if more data ever separates them this decision should be revisited.
    //
    // Also measured post-lube and worth keeping:
    //   - Breakaway is 0.050 on tiles, 0.030 off the ground, uniform across all four pods
    //     (it was 0.05/0.07/0.06/0.05 before the mechanical work). The pod-to-pod spread is
    //     gone; the surface difference is not.
    //   - No backlash after the repair. Directional dead zone 0.05-0.45 deg, nothing
    //     significant on any pod, so criteria 5 and 6 remain mechanically unblocked.
    //   - Servo current at breakaway is under ~1% of the 3300 mA stall figure (Axon MINI+,
    //     interpolated to the 5 V rail). Torque headroom is large and is not the constraint.
    //
    // Committed because they are a large, measured improvement on what shipped, not because they
    // are finished. Randomised interleaved A/B against the previous gains, same battery, 40
    // pod-runs each, 90 degree steps on blocks at 13.1 V:
    //
    //     steady-state error   3.23 -> 0.96 deg mean,  7.24 -> 2.52 deg max
    //     error sign changes   2.35 -> 0.85 per step
    //     post-settle p-p      0.53 -> 0.21 deg mean, 0.45 max
    //
    // kD chosen at n=60 pod-runs against three rivals, randomised, 12.87-12.95 V. Selection was
    // lexicographic: zero loose-mode runs first, then lowest residual. kD 0.022 was the only one
    // of four to hold 0/60 (true rate bounded below 6%); kD 0.020, 0.014 and 0.010 drew 3, 1 and 1.
    // At kD 0.022: residual at t=3 s 0.77 deg mean / 2.36 max, settle to +/-2.0 deg 0.647 s with
    // 97% of pods settling, post-settle p-p 1.24 max. Per pod the residual is 0.76 / 0.73 / 1.06 /
    // 0.52 deg, so it is a broad distribution rather than one bad pod - 35% of pod-runs exceed
    // 1.0 deg and 12% exceed 1.5 deg.
    //
    // What was actually wrong, in order of size:
    //
    // 1. Loop rate. The turn PID runs at the OpMode loop rate, which was 33 Hz under a four-pod
    //    hold - slower than the servos' own 20 ms PWM frame. Raising it to ~100 Hz cut error sign
    //    changes from 1-4 to 0-1 with the gains untouched. DriveTeleOp was 30.7 Hz and is now
    //    75.8 Hz; see that file. This was the single biggest lever and it is not a gain.
    //
    // 2. kF is a relay, not a feed-forward. CoaxialPod feeds the PIDF a sign, so the F term is
    //    +/-kF. Measured breakaway is 0.025-0.050, so the shipped kF of 0.005 was 5-10x too small
    //    to do anything, and the pod parked where kP*e fell under breakaway: 0.035/0.300 = 6.7 deg.
    //    Setting kF to breakaway does fix the residual (3.52 -> 1.25 deg) and limit-cycles at
    //    42.5 deg peak-to-peak. Replaced by kS, a continuous tanh static-friction term with the
    //    same authority away from the target and no switching discontinuity at it. kF is now 0.
    //
    // 3. servoCachingThreshold 0.05 was wider than the entire breakaway range, so a settled pod's
    //    written power was frozen across a ~19 degree error window. Now 0.01; below that the extra
    //    bus traffic costs loop rate (96 -> 77 Hz) for no measurable gain.
    //
    // Negative results, all measured, so nobody repeats them:
    //
    //   - Encoder noise is NOT why kD misbehaves. Noise floor is sigma 0.042-0.054 deg against a
    //     0.1125 deg read granularity, which pushes 0.0006 of output through kD 0.010. The old
    //     "kD above 0.010 adds ringing" note was a loop-rate artefact.
    //   - Derivative-on-measurement does not permit more damping. Implemented and available
    //     (default off); kD 0.040 rings 16x, 0.070 and 0.110 are unusable.
    //   - Widening the servo PWM pulse range 600-2400 -> 500-2500 us does nothing. The servo
    //     already reaches ~90% of top speed at 0.3 power, so the extra range is never used.
    //   - No measurable backlash. Approaching one heading from both sides 8 times each, the
    //     directional gap is 0.01-0.60 deg and no pod's gap exceeds twice its standard error.
    //   - No mechanical outlier pod. Loose-mode rate is 12.8/17.4/18.2/20.7 percent across the
    //     four, statistically one band.
    //   - kI is still 0, but the old "30-45 degrees of hunting" result is not why. That predates
    //     both the caching fix and PIDFController's integral band and reset threshold. Re-testable.
    //   - Loop rate ABOVE ~48 Hz buys nothing. Randomised interleaved A/B on 2026-08-13, 40
    //     pod-runs per arm, 47.8 Hz against 92.1 Hz with only the dashboard publish rate changed:
    //     steady state 2.65 -> 2.77 deg (95% CI -0.51 to +0.71, p 0.70), err at 3 s 2.80 -> 2.93
    //     (p 0.69), rings 3.95 -> 6.22 (p 0.39), post-settle p-p 0.84 -> 1.02 (p 0.46), rise
    //     10-90% unchanged at 0.37 s. Nothing moved. The servos' own 20 ms PWM frame is ~50 Hz, so
    //     updating faster than that cannot reach them. This does NOT contradict the loop-rate win
    //     recorded above, which was climbing out of ~18 Hz; it bounds where that win stops. It
    //     also kills the tempting story that the residual is "one control period of pod travel" -
    //     per-update travel went 4.5 -> 2.3 deg and the residual did not follow.

    // IMPORTANT, loop rate figures: every rate quoted in this file before 2026-08-13 was computed
    // as the mean of an instantaneous 1/dt column, which overweights short loops and reads ~1.8x
    // optimistic on every trace in tools/swervetune/runs. True throughput is 1/mean(dt). The
    // "33 Hz" and "~100 Hz" above are that inflated statistic; the honest pair is roughly 18 Hz
    // and 50 Hz. So are DriveTeleOp's "30.7 Hz" and "75.8 Hz"; its first honest measurement was
    // 99.3 Hz true (2026-08-16, CLAUDE.md section 6). The direction and the ranking of the fixes are unaffected - the absolute numbers
    // were wrong. The scorer now records loop_hz_true alongside the old figure.
    //
    // Known not met, off the ground: 90 degree settle to +/-2.0 deg is ~0.5 s against a 350 ms
    // target; residual at t=3 s is 0.54-1.0 deg mean but up to 2.5 deg worst case; pod-to-pod
    // spread is 24-84% against 15%. Roughly 17% of step responses land in a wide mode with 25 deg
    // of peak-to-peak rather than the usual 0.23 deg - that bimodality is gain-controllable and
    // is the open problem. Full trial log and traces in tools/swervetune.

    // Public so the bring-up tool can read them and report any divergence from its own working
    // copy. The tool is supposed to be able to hold different gains - that is what it is for - but
    // it should never be possible to measure at gains nobody intended, which is what happened when
    // its calibration file sat at kD 0.010 while this file said 0.022.
    //
    // PER-POD as of 2026-08-13 evening, indexed by servo number ss0..ss3 like podZeroDeg
    // (0=RB, 1=RF, 2=LF, 3=LB). The pods are not interchangeable plants: re-scoring the day's
    // 534-run kP sweep per pod shows pod 0 carrying the highest residual at every kP while almost
    // never drawing the loose mode (1/159), and pod 3 the lowest residual while drawing it in a
    // quarter of runs at kP 0.320. One shared gain set cannot serve both, which is why criterion
    // 10 (pod-to-pod settle spread) never passed. The scalar turnKP/turnKD/turnKS survive as the
    // pooled-tune values; the factories read the arrays.
    //
    // How the per-pod values were chosen (game tiles, 12.28-12.40 V, randomised interleaved
    // 90-degree steps, n=15-25 per cell per pod, 210 sweep trials total, tools in
    // tools/swervetune/current_runs/perpod_*.jsonl):
    //
    //   The controlling discovery: the wide mode (>=10 deg post-settle excursions) is fuelled by
    //   kS, not tamed by kD. At kS 0.035, kP 0.44 drew 5/25 wide on pod 0; at kS 0.028 the same
    //   kP drew 0/20 wide on pods 0 and 2 and 1/20 on pod 1 - and with the wide mode defuelled,
    //   the extra P authority is free residual: pod 0 |e3| 1.31 -> 0.92, pod 1 0.57 -> 0.49,
    //   pod 2 1.40 -> 0.57 versus their kP 0.320 / kS 0.035 cells. kS below the kinetic need
    //   parks the pod short at low kP (pod 0 at kP .32 / kS .022: 2.84 deg) - the two must move
    //   together.
    //
    //   Pod 3 is the outlier: it draws wide events at EVERY cell tested (kS 0.022-0.050, kP
    //   0.32-0.44), best rate ~7% (1/15) at kS 0.022. Its assignment favours the smallest events
    //   and residual among the tame cells, not zero events, because zero was not on the menu
    //   today. If pod 3's wide mode matters at competition, the fix is mechanical (or the shelved
    //   pulsed/dom experiments), not more gain search on these axes.
    //
    //   Second discovery, and the reason the final values lean conservative: the plant is not
    //   stationary. Pod 0's kS 0.028 cell measured 0/20 wide, then 6/25 wide THIRTY MINUTES
    //   later at the same gains - kinetic friction keeps falling as the pods warm through
    //   sustained testing, so a kS fitted to the friction of the moment goes unstable as the
    //   session runs on. The final choices reduce near-target feed-forward (lower kS, or the
    //   same kS with a wider tanh band) because that direction FAILS SAFE: too little
    //   feed-forward parks a cold pod slightly short; too much feeds the wide mode. Pod 0 takes
    //   the wider band (0/15 wide, 0 wobbles, |e3| 0.79 on the hot plant), pods 1-2 take kS
    //   0.024 (pod 2: 0/15 wide, 0 wobbles), measured per pod, not assumed transferable.
    //   SUPERSEDED: that per-pod assignment is history. The arrays in TUNED VALUES at the top are
    //   uniform again - kS 0.022 and band 2.0 deg on every pod (CLAUDE.md section 5).
    public static final double turnKP = 0.320;
    public static final double turnKD = 0.022;

    // 2026-08-14 late, the rolling-plant discovery (see CoaxialPod.TURN_GAIN_SCHEDULING): a
    // rolling wheel steers nearly friction-free, so gains sized against parked stiction were
    // several times over-gained the moment the robot moved - a 3-4 Hz, 25 degree limit cycle
    // on every pod that no static tuning could see, because every static test ran at zero
    // drive power. With the schedule scaling turn effort down against drive power AND the
    // pod's own steering speed (the at-rest hunt kept its own contact patch rolling - it
    // sustained the regime that fed it), one gentle recipe now wins on every pod: steady-drive
    // pod error fell from 33-89 to 3-8 deg p-p and the X-lock hold went from hunting
    // indefinitely to actuation-silent. The per-pod kP 0.44 aggression that won the static
    // shootout is exactly what the schedule now provides only when parked.
    // turnKPPerPod / turnKDPerPod / turnKSPerPod / turnKSBandDegPerPod: TUNED VALUES, top.

    /**
     * Static-friction feed-forward, set to the measured breakaway. Replaces the kF relay.
     *
     * 0.035 is the OFF-GROUND breakaway. A carpet re-fit on the competition surface, at
     * competition weight, produced a rival value of 0.045 - measured, not estimated, and the
     * two INVERT between surfaces. Randomised, 10 repeats, n=40 pod-runs per cell, kP 0.200 /
     * kD 0.022 / cache 0.01, 12.5 V:
     *
     *                        kS 0.035                      kS 0.045
     *   game surface   |ss| 2.92 / 7.37 deg           |ss| 1.48 / 4.59 deg
     *                  rings 1.57, p-p max 1.80       rings 2.38, p-p max 3.04
     *   off ground     |ss| 0.56 / 2.22 deg           |ss| 1.23 / 7.26 deg
     *                  rings 1.40, p-p max 9.00       rings 11.43, p-p max 49.31
     *                  1/40 loose                     13/40 loose
     *
     * So 0.045 roughly halves the residual on the surface we actually compete on, and falls
     * apart off the ground. Note the two failures are not the same size: on carpet 0.045's
     * worst peak-to-peak is 3.04 deg, a wiggle; off the ground it is 49 deg.
     *
     * Staying at 0.035 for now, deliberately. A mechanical pass (clean and lubricate the
     * steering path) is planned, and reducing friction moves the plant TOWARD the off-ground
     * condition, which is exactly where 0.045 breaks. 0.035 is the value that holds across the
     * whole friction range measured so far; 0.045 is fitted to the friction the pods have
     * today. Re-fit kS after the mechanical pass and revisit this - if friction ends up low and
     * stable, 0.045 may no longer be the right rival either.
     *
     * Criterion 7 (surface robustness) was written to guard against venue-to-venue variation.
     * That rationale is weaker than assumed: the carpet tested on is the official FTC game
     * surface, the same tiles used at competition. Off-ground robustness is a bench convenience,
     * not an operating condition - it is being weighted here only because of the pending
     * mechanical change.
     */
    public static final double turnKS = 0.035;
    public static final double turnKSBandDeg = 2.0;

    // turnServoCaching (servo output caching threshold, every pod): TUNED VALUES, top.

    private static double dtLength = 146.420; //distance from robot center to front/back pod center
    private static double dtWidth = 154.240; // distance from robot center to left/right pod center

    /**
     * Per-pod zero offset and analog range, indexed by servo number: ss0, ss1, ss2, ss3.
     *
     * <p>These were literals inside the pod factories, which meant nothing could compare them
     * against the bring-up tool's live calibration. The tool's divergence guard covered gains only,
     * so on 2026-08-13, after a pod repair and a re-zero of all four, this file was carrying zeros
     * 30-175 degrees away from the calibrated hardware and absolutely nothing said so. Gains being
     * wrong costs tuning; zeros being wrong points the wheels the wrong way.
     *
     * <p>Keep these as the single source of truth - the factories below read them, and
     * SwerveBringUp compares against them.
     */
    // Re-captured 2026-08-15 after the operator's full re-zero (which also flipped drive
    // direction on ss0 and ss3 - a re-zero near +-180 and a direction flip are two halves of
    // the same physical statement). Source of truth: /sdcard/FIRST/swerve_bringup_cal.txt, the
    // stored radians converted at full precision - NOT the /state "zeroDeg" field, which is a
    // live telemetry value that moves with wheel position and briefly ended up in this array
    // (sync from the cal file or the export, never from telemetry). History that still
    // matters: the 2026-08-14 zeros carried a +13.13 deg common trim solved from the odometry
    // crab angle (all four wheels parallel but 13 deg left of true chassis-forward - invisible
    // to every pod-relative measurement). Whether THIS by-eye re-zero has a similar collective
    // bias is unverified: redo the crawl-speed crab check before trusting field-frame autos.
    // podZeroDeg: TUNED VALUES, top.

    // podMinV / podMaxV (analog encoder low/high volts, ss0..ss3): TUNED VALUES, top.

    /**
     * Drive-motor reversal per pod, ss0..ss3, same indexing. The other half of "which way does
     * this corner push": a wrong zero and a wrong drive direction produce the same haywire
     * symptom, and until 2026-08-13 late the divergence guard compared neither directions nor
     * zeros-at-drive-time - the factories hardcoded these and nothing could disagree out loud.
     * Servo direction is REVERSE on all four (Axon MINI+ orientation) and the encoders read
     * un-reversed; those stay literals in the builder until one actually varies per pod.
     */
    // ss0 and ss3 flipped with the 2026-08-15 re-zero, paired with their ~180 zero moves.
    // podDriveReversed: TUNED VALUES, top.

    // podEncoderNames: TUNED VALUES, top. Motor and servo names are sm#/ss# by servo number.
    //
    // The encoder names do NOT follow the sm#/ss#/se# numbering. The Axon feedback wires are
    // spliced two per analog port, and the wiring scan found every pod landing on a different
    // channel than its own number. Do not "tidy" these to match.
    /**
     * The pods from the most recent {@link #createSwerve} call, indexed by ss number.
     *
     * <p>Diagnostics only. The Follower owns the pods and exposes no accessor, so without this
     * there is no way to instrument the COMPETITION control path - and every steering number this
     * project has ever published came from the bring-up tool instead, driving its own copy of the
     * drivetrain. Nothing in the control path reads this array; it is a handle, not state.
     */
    public static final CoaxialPod[] builtPods = new CoaxialPod[4];

    /** Shared body so a per-pod gain can never be wired to the wrong corner by hand. */
    private static CoaxialPod buildPod(HardwareMap hardwareMap, int ss, Vector2D offset) {
        String motor = "sm" + ss;
        String servo = "ss" + ss;
        String encoder = podEncoderNames[ss];
        CoaxialPod pod = new CoaxialPod(hardwareMap, new CoaxialPodConfig(c -> {
            c.name.set(servo);
            c.motorName.set(motor);
            c.servoName.set(servo);
            c.servoEncoderName.set(encoder);
            // A fresh controller per pod: it carries integral, error and timing state.
            c.turnController.set(new PIDFController(
                    new PIDFCoefficients(turnKPPerPod[ss], 0, turnKDPerPod[ss], 0)));
            c.driveDirection.set(podDriveReversed[ss] ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
            c.angleOffsetRad.set(Math.toRadians(podZeroDeg[ss]));
            c.podOffset.set(offset);
            c.analogMinVoltage.set(podMinV[ss]);
            c.analogMaxVoltage.set(podMaxV[ss]);
            c.encoderReversed.set(false);
            c.motorCachingThreshold.set(0.05);
            c.servoCachingThreshold.set(turnServoCaching);
        }));
        pod.setStaticFriction(turnKSPerPod[ss], Math.toRadians(turnKSBandDegPerPod[ss]));
        // Pulsed final approach with the stall-escalation ladder (see CoaxialPod). Inside 6 deg
        // the servo is silent except for discrete stationary pulses; base pulse 0.035/20 ms is
        // measured travel on the lubricated plant (pulsecal2, 2026-08-15), tolerance 0.6 deg.
        // Bench A/B: parked pods go actuation-silent instantly (continuous never fully quiets),
        // steps land 0.35 mean / 0.81 max deg, moving regimes measure identical to continuous.
        pod.setPulsedApproach(true, Math.toRadians(pulseBandDeg),
                Math.toRadians(pulseToleranceDeg), pulsePower, pulseSeconds,
                Math.toRadians(pulseStationaryDegPerSec), pulseCoastSeconds);
        builtPods[ss] = pod;
        return pod;
    }

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        return buildPod(hardwareMap, 2, Vector2D.cartesian(dtLength, dtWidth));
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        return buildPod(hardwareMap, 1, Vector2D.cartesian(dtLength, -dtWidth));
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        return buildPod(hardwareMap, 3, Vector2D.cartesian(-dtLength, dtWidth));
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        return buildPod(hardwareMap, 0, Vector2D.cartesian(-dtLength, -dtWidth));
    }

    // =======================================================================================
    // FACTORIES
    // =======================================================================================

    /**
     * The steering statics (turn gain schedule floor, demand-rate feed-forward, Swerve's demand slew limit). They
     * are static in the vendored classes and survive across OpModes in one app process, so every drivetrain build
     * re-applies them: before 2026-11-15 a bench session's live values leaked into the next TeleOp.
     *
     * <p>Values and evidence (2x2 tiles, 12.2-12.7 V, heading hold off; stick circle 110 deg/s at 0.25 drive,
     * n=4-5 per arm; stick-flick square, 90 deg flicks at 0.25 drive, n=4 per arm; randomised, interleaved):
     * <ul>
     * <li>floor 0.24 -> 0.30: 0.24 (sized 2026-08-15 on the OLD 214 deg/s pods) left a rolling pod below
     *     breakaway inside the 20 deg gate - a stall-and-jump staircase, ~25 wheel reversals per lap;</li>
     * <li>demand-rate feed-forward kV 0.045 power per rad/s (vendored, CoaxialPod): pods no longer trail a
     *     turning stick by the error that makes the push - circle error 14.0 -> 6.2 deg mean, 28 -> ~13 p95,
     *     lag ~100 -> ~20 ms, wheel reversals 24 -> 9.6 per lap; tau 15 ms;</li>
     * <li>feed-forward only below 240 deg/s, slew 214 -> 300 deg/s: Swerve ramps every flick at exactly the slew
     *     rate, and fed forward that ramp overshot 27-41 deg; excluded, flicks align in 373 vs 563 ms (-190
     *     [-229, -151]) with 8.6 vs 16.6 deg overshoot.</li>
     * </ul>
     * Pod kD stays 0.015: 0.03 / 0.045 / 0.06 damp flicks but push the rolling wobble to ~4.6 Hz on the left pods
     * (circle reversals 16 / 25 / 28 per lap vs 9.6).
     */
    public static void applySteeringStatics() {
        Swerve.setDemandSlewDegPerSec(demandSlewDegPerSec);
        CoaxialPod.setScheduleTuning(turnScheduleFloor, Double.NaN, Double.NaN);
        CoaxialPod.setDemandRateFeedforward(turnDemandRateKV, turnDemandRateTauS,
                Math.toRadians(turnDemandRateMaxDegS));
    }

    /** The competition drivetrain on its own - also what the tuners drive. */
    public static Swerve createSwerve(HardwareMap hardwareMap) {
        applySteeringStatics();
        return new Swerve(hardwareMap, swerveConfig, leftFront(hardwareMap),
                rightFront(hardwareMap), leftBack(hardwareMap), rightBack(hardwareMap));
    }

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, pinpointConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        Localizer localizer = dryRunLocalizer != null ? dryRunLocalizer : createLocalizer(hardwareMap);
        Drivetrain drivetrain = createSwerve(hardwareMap);
        if (dryRunDriveWrap != null) {
            drivetrain = dryRunDriveWrap.apply(drivetrain);
        }
        return new Follower(localizer, drivetrain, new Foresight(foresightConfig));
    }

    // Auto Dry Run hook (diagnostics/autosim/AutoDryRun - never set by competition code). Set ONLY for the
    // duration of the wrapped auto's init() (try/finally), so a stale static cannot reach a real auto: while
    // set, createFollower builds the follower on the simulated localizer and the wrapped (recording) swerve,
    // and requireForesightMeasured() lets the simulated follower through. null = normal behaviour.
    public static volatile Localizer dryRunLocalizer;
    public static volatile java.util.function.UnaryOperator<Drivetrain> dryRunDriveWrap;
}
