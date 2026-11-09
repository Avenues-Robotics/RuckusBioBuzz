package org.firstinspires.ftc.teamcode.systems;

import org.firstinspires.ftc.teamcode.SystemConstants;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.Alliance;
import org.firstinspires.ftc.teamcode.modules.PredictiveAiming;
import org.firstinspires.ftc.teamcode.modules.ZoneCheck;
import org.firstinspires.ftc.teamcode.helpers.HivePosition;

public class Shooter {
    // Tunables (MIN_GATE_OPEN_S, HOLD_SPEED_FRACTION): SystemConstants.Shooter.

    private final ElapsedTime shootTimer = new ElapsedTime();
    private final ZoneCheck zone;
    private final Turret aiming;
    private final Flywheel flywheel;
    private final HivePosition hivePosition;
    private Gate shooterGate;

    private boolean inZone;
    private boolean forceZone;
    private final PredictiveAiming predictiveAiming;

    private enum states {
            RESET, TRACK, SHOOT
    }
    states state = states.TRACK;

    public Shooter(HardwareMap hardwareMap, PredictiveAiming predictor, Alliance alliance) {
        hivePosition = new HivePosition();
        zone = new ZoneCheck(alliance, SystemConstants.Zone.yOffsetPollen, SystemConstants.Zone.mapPollen);
        aiming = new Turret(hardwareMap, predictor, new HivePosition(), alliance);
        flywheel = new Flywheel(hardwareMap, aiming);
        shooterGate = new Gate(hardwareMap);
        this.predictiveAiming = predictor;

    }
    public void setForceZone(boolean force) {
        forceZone = force;
    }

    public boolean shooting(){
        return state == states.SHOOT;
    }

    public void update() {
        inZone = forceZone || zone.inZone(predictiveAiming.predictPose());
        flywheel.update(inZone);

        switch (state) {
            case RESET:
                shooterGate.closeGate();
                if (!aiming.unwinding()) state = states.TRACK;
                break;
            case TRACK:
            case SHOOT:
                aiming.update();
                boolean aimed = aiming.pollenAimed() && aiming.nectarAimed();
                if (aiming.unwinding()) {
                    state = states.RESET;
                } else if (state == states.SHOOT) {
                    // A ball hitting the wheel dips its speed past the 5% entry band. Re-checking
                    // the entry condition here closed the gate on that ball and handed the intake
                    // back to the driver mid-shot, so hold SHOOT for a minimum open time and then
                    // on a wider speed band. Leaving the zone still ends it at once.
                    boolean hold = shootTimer.seconds() < SystemConstants.Shooter.MIN_GATE_OPEN_S
                            || (aimed && flywheel.atSpeedPollen(SystemConstants.Shooter.HOLD_SPEED_FRACTION)
                                && flywheel.atSpeedNectar(SystemConstants.Shooter.HOLD_SPEED_FRACTION));
                    if (!inZone || !hold) state = states.TRACK;
                } else if (inZone && aimed && flywheel.atSpeedPollen() && flywheel.atSpeedNectar()) {
                    state = states.SHOOT;
                    shootTimer.reset();
                }
                if (state == states.SHOOT) {
                    shooterGate.openGate();

                } else {
                    shooterGate.closeGate();
                }
                break;
        }
    }

    public boolean ready() {
        return state == states.SHOOT;
    }
    public states getState(){
        return state;
    }

    public boolean inZone() { return zone.inZone(predictiveAiming.predictPose()); }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    public boolean nectarAimed() { return aiming.nectarAimed(); }
}
