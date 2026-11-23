package org.firstinspires.ftc.teamcode.fieldview;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.ZoneCheck;

public final class FieldView {
    private static volatile String snapshot;
    private static volatile long publishedAtNs;
    private static long lastPublishNs;

    private FieldView() {}

    /** No zone (AutoDryRun): the page draws no zone and shows no verdict. */
    public static void publish(Pose robot, Pose predicted) {
        publish(robot, predicted, null);
    }

    /**
     * With {@code alliance}: also the pollen shooting zone exactly as Shooter evaluates it - ZoneCheck on
     * SystemConstants.Zone.mapPollen, the predicted pose for the verdict - plus the map itself (red frame; the
     * page rotates it 180 deg for blue, as ZoneCheck does). Until 2026-11-23 the page still read the pre-map
     * fields (zoneY, inZone, predInZone) that this stopped sending on 2026-11-05, so it always said OUT OF ZONE.
     */
    public static void publish(Pose robot, Pose predicted, Alliance alliance) {
        long now = System.nanoTime();
        if (now - lastPublishNs < SystemConstants.FieldView.PUBLISH_INTERVAL_MS * 1e6) {
            return;
        }
        lastPublishNs = now;

        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"x\":").append(robot.x())
            .append(",\"y\":").append(robot.y())
            .append(",\"h\":").append(robot.heading())
            .append(",\"px\":").append(predicted.x())
            .append(",\"py\":").append(predicted.y())
            .append(",\"ph\":").append(predicted.heading())
            .append(",\"predictMs\":").append(SystemConstants.PredictiveAiming.PREDICT_MS)
            .append(",\"field\":").append(SystemConstants.Zone.FIELD_SIZE)
            .append(",\"robotL\":").append(SystemConstants.Zone.ROBOT_LENGTH)
            .append(",\"robotW\":").append(SystemConstants.Zone.ROBOT_WIDTH);
        if (alliance != null) {
            int[][] map = SystemConstants.Zone.mapPollen;
            ZoneCheck zone = new ZoneCheck(alliance, SystemConstants.Zone.yOffsetPollen, map);
            boolean nowIn = zone.inZone(robot);
            boolean predIn = zone.inZone(predicted);
            int[] cell = zone.getZone();
            sb.append(",\"alliance\":\"").append(alliance.name()).append('"')
                .append(",\"inZone\":").append(nowIn)
                .append(",\"predInZone\":").append(predIn)
                .append(",\"predCell\":[").append(cell[0]).append(',').append(cell[1]).append(']')
                .append(",\"zoneNx\":").append(SystemConstants.Zone.xQuality)
                .append(",\"zoneNy\":").append(SystemConstants.Zone.yQuality)
                .append(",\"zoneMap\":[");
            for (int y = 0; map != null && y < map.length; y++) {
                if (y > 0) sb.append(',');
                sb.append('[');
                for (int x = 0; map[y] != null && x < map[y].length; x++) {
                    if (x > 0) sb.append(',');
                    sb.append(map[y][x]);
                }
                sb.append(']');
            }
            sb.append(']');
        }
        snapshot = sb.toString();
        publishedAtNs = now;
    }

    static String state() {
        String s = snapshot;
        if (s == null) {
            return "{\"live\":false}";
        }
        long ageMs = (System.nanoTime() - publishedAtNs) / 1_000_000L;
        return s + ",\"live\":" + (ageMs < 1000) + ",\"ageMs\":" + ageMs + "}";
    }
}
