package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class ZoneCheck {
    private static Alliance alliance;
    private static ShooterType shooterType;
    public enum ShooterType{  // I gave this feedback to Ryder but this is not system/game agnostic.  It's only used to find the offsets, so just pass the offsets as constants or an object to the constructor.
        POLLEN, NECTAR
    }

    public ZoneCheck(Alliance alliance, ShooterType shooterType) {
        ZoneCheck.shooterType = shooterType;
        ZoneCheck.alliance = alliance;
    }

    public void setAlliance(Alliance alliance) {
        ZoneCheck.alliance = alliance;
    }

    public static boolean inZone(Pose pose) {
        boolean pollen = shooterType == ShooterType.POLLEN;
        double offset = pollen ? SystemConstants.Zone.yOffsetPollen : SystemConstants.Zone.yOffsetNectar;  // The ternary operator is super unclear to read.  Just use if...else.  Only use in very simple cases or as a method parameter where an if else doesn't fit.
        double x = pose.x() - offset * Math.sin(pose.heading());
        double y = pose.y() + offset * Math.cos(pose.heading());
        if (alliance == Alliance.BLUE) {
            x = SystemConstants.Zone.FIELD_SIZE - x;
            y = SystemConstants.Zone.FIELD_SIZE - y;
        }
        int[][] map = pollen ? SystemConstants.Zone.mapPollen : SystemConstants.Zone.mapNectar;
        int xQuad = (int) Math.floor(x / (SystemConstants.Zone.FIELD_SIZE / SystemConstants.Zone.xQuality));
        int yQuad = (int) Math.floor(y / (SystemConstants.Zone.FIELD_SIZE / SystemConstants.Zone.yQuality));
        if (map == null || yQuad < 0 || yQuad >= map.length || map[yQuad] == null
                || xQuad < 0 || xQuad >= map[yQuad].length) {
            return false;
        }
        return map[yQuad][xQuad] == 1;
    }
}
