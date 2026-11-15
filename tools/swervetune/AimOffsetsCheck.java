// HOST CHECK for helpers/AimOffsets.java (no robot): the coach-perspective geometry of driver 2's d-pad.
// From the repo root (zsh):
//   O=$(mktemp -d); P=$O/src/org/firstinspires/ftc/teamcode; mkdir -p $P/helpers
//   cp tools/swervetune/AimOffsetsCheck.java $P/ && cp TeamCode/src/main/java/org/firstinspires/ftc/teamcode/helpers/{AimOffsets,Alliance,HivePosition}.java $P/helpers/
//   printf 'package org.firstinspires.ftc.teamcode;\npublic final class SystemConstants {\n public static class TeleOp { public static double AIM_STEP_IN = 1.0; public static double SPEED_STEP = 25; }\n public static class HivePosition { public static double xred = 58, xblue = 84, UP_POS = 90, DOWN_POS = 52, HALF_Y = 71; }\n}\n' > $P/SystemConstants.java
//   J="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin"
//   "$J/javac" -nowarn -d $O/out -sourcepath "$O/src:$PWD/third_party/PedroPathing/core/src/main/java" $P/AimOffsetsCheck.java && "$J/java" -cp $O/out org.firstinspires.ftc.teamcode.AimOffsetsCheck
// (The stub SystemConstants carries only the fields AimOffsets / HivePosition read - the real one needs the
// dashboard jars. Keep its HivePosition values equal to SystemConstants.HivePosition.)
package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.helpers.AimOffsets;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

public final class AimOffsetsCheck {
    static int fails;

    static void check(boolean ok, String what) {
        if (!ok) {
            fails++;
            System.out.println("FAIL " + what);
        }
    }

    public static void main(String[] a) {
        // 1. Every robot position, both alliances: a RIGHT press is perpendicular to the shot line and never has a
        //    component toward the coach's LEFT (red's right is -y: drivers at x = 0 facing +x; blue's is +y).
        int n = 0, degenerate = 0;
        for (Alliance al : Alliance.values()) {
            double[] coach = al == Alliance.RED ? new double[] {0, -1} : new double[] {0, 1};
            for (double x = 2; x < 141; x += 3) {
                for (double y = 2; y < 141; y += 3) {
                    Pose robot = new Pose(x, y, 0);
                    Pose hive = HivePosition.target(robot, al);
                    double[] r = AimOffsets.coachRight(robot, hive, al);
                    double dx = hive.x() - x, dy = hive.y() - y, d = Math.hypot(dx, dy);
                    check(Math.abs(Math.hypot(r[0], r[1]) - 1) < 1e-9, "unit " + al + " " + x + "," + y);
                    check(d < 1e-9 || Math.abs((r[0] * dx + r[1] * dy) / d) < 1e-9, "perpendicular " + al + " " + x + "," + y);
                    double toward = r[0] * coach[0] + r[1] * coach[1];
                    check(toward > -1e-9, "toward coach's right " + al + " " + x + "," + y + " -> " + toward);
                    if (toward < 0.2) degenerate++;
                    n++;
                }
            }
        }
        System.out.println(n + " positions (both alliances): perpendicular + toward the coach's right; " + degenerate
                + " nearly level with the hive in x (the shot runs along y, so the coach sees a sideways miss as near/far)");

        // 2. Worked examples, by hand. Red hives x 58, y 52 / 90.
        double[] r = AimOffsets.coachRight(new Pose(30, 52, 0), new Pose(58, 52), Alliance.RED);   // red, own side, shooting +x
        check(Math.abs(r[0]) < 1e-9 && Math.abs(r[1] + 1) < 1e-9, "red near side: right = -y, got " + r[0] + "," + r[1]);
        r = AimOffsets.coachRight(new Pose(100, 52, 0), new Pose(58, 52), Alliance.RED);           // red, far side, shooting -x
        check(Math.abs(r[0]) < 1e-9 && Math.abs(r[1] + 1) < 1e-9, "red far side: still -y (shot's LEFT), got " + r[0] + "," + r[1]);
        r = AimOffsets.coachRight(new Pose(120, 90, 0), new Pose(84, 90), Alliance.BLUE);          // blue, own side, shooting -x
        check(Math.abs(r[0]) < 1e-9 && Math.abs(r[1] - 1) < 1e-9, "blue near side: right = +y, got " + r[0] + "," + r[1]);

        // 3. Accumulation is a fixed field vector judged from the LAST SHOT: two RIGHT presses from (30, 52) red.
        AimOffsets o = new AimOffsets();
        o.nudgeLateral(1, new Pose(30, 52, 0), new Pose(100, 100, 0), Alliance.RED);
        o.nudgeLateral(1, new Pose(30, 52, 0), new Pose(100, 100, 0), Alliance.RED);
        check(Math.abs(o.goalDx()) < 1e-9 && Math.abs(o.goalDy() + 2.0) < 1e-9 && o.lateralClicks() == 2,
                "2 RIGHT from the last shot = (0, -2) in, got " + o.goalDx() + "," + o.goalDy());
        o.nudgeLateral(-1, null, new Pose(30, 52, 0), Alliance.RED);   // no shot yet: judged from where the robot is
        check(Math.abs(o.goalDy() + 1.0) < 1e-9 && o.lateralClicks() == 1, "1 LEFT back: (0, -1), got " + o.goalDy());
        o.nudgeSpeed(1);
        o.nudgeSpeed(1);
        o.nudgeSpeed(-1);
        check(Math.abs(o.speedOffset() - 25) < 1e-9 && o.speedClicks() == 1, "speed +2 -1 steps = +25, got " + o.speedOffset());
        o.reset();
        check(o.goalDx() == 0 && o.goalDy() == 0 && o.speedOffset() == 0, "reset");

        System.out.println(fails == 0 ? "PASS" : fails + " FAILED");
        System.exit(fails == 0 ? 0 : 1);
    }
}
