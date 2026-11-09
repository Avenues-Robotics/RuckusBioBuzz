package org.firstinspires.ftc.teamcode.modules;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.SystemConstants;
import org.firstinspires.ftc.teamcode.helpers.Alliance;

public class ZoneCheck {
    private final Alliance alliance;
    private final int offset;
    private final int[][] map;
    private int xQuadrant;
    private int yQuadrant;


    public ZoneCheck(Alliance alliance, int offset, int[][] map) {
        this.offset = offset;
        this.map = map;
        this.alliance = alliance;
    }

    public boolean inZone(Pose pose) {
        double x;
        double y;
        if (alliance == Alliance.BLUE) {
            x = SystemConstants.Zone.FIELD_SIZE - (pose.x() - offset * Math.sin(pose.heading()));
            y = SystemConstants.Zone.FIELD_SIZE - (pose.y() + offset * Math.cos(pose.heading()));
        } else {
            x = pose.x() - offset * Math.sin(pose.heading());
            y = pose.y() + offset * Math.cos(pose.heading());
        }
        xQuadrant = (int)Math.floor(x / (SystemConstants.Zone.FIELD_SIZE / SystemConstants.Zone.xQuality));
        yQuadrant = (int)Math.floor(y / (SystemConstants.Zone.FIELD_SIZE / SystemConstants.Zone.yQuality));
        if (map == null || yQuadrant < 0 || yQuadrant >= map.length || map[yQuadrant] == null
                || xQuadrant < 0 || xQuadrant >= map[yQuadrant].length) {
            return false;
        }
        return map[yQuadrant][xQuadrant] == 1;
    }
    public int[] getZone(){
        return new int[]{xQuadrant, yQuadrant};
    }
}
