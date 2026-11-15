// Swerve Bring-Up export.
// Each line replaces the declaration of the same name in Constants.java's
// TUNED VALUES block: python3 tools/swervetune/robot.py constants --write

// Follower (Foresight) - as the follower bench last ran it (pedroPidf)
public static final double naturalForwardDeceleration = 40.0;
public static final double naturalStrafeDeceleration = 40.0;
public static final double headingKP = 1.20;
public static final double forwardTranslationalPrimaryKP = 0.26;
public static final double forwardTranslationalSecondaryKP = 0.26;
public static final double strafeTranslationalPrimaryKP = 0.26;
public static final double strafeTranslationalSecondaryKP = 0.26;
public static final double translationalKI = 0.0;
public static final double translationalKD = 0.025;

// Localizer (Pinpoint), inches
public static final double pinpointXPodOffset = -2.840;
public static final double pinpointYPodOffset = -5.988;
public static final boolean pinpointXPodReversed = false;
public static final boolean pinpointYPodReversed = false;

// Pods, indexed by servo number ss0..ss3 (0=RB, 1=RF, 2=LF, 3=LB)
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
