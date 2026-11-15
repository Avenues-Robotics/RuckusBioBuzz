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
    /** Driver override (TeleOp right trigger): shoot now, whatever zone / aim / wheel speed say. */
    private boolean manualShoot;
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
    /** An auto's Shoot event is active (AutoMechanisms only): treat the robot as in the zone, and with the nectar
     *  side unmounted let the pollen side fire alone (nectarReady). */
    public void setForceZone(boolean force) {
        forceZone = force;
    }

    /**
     * While true the gate is open (state SHOOT, so TeleOp feeds the intake) and the flywheel runs at its
     * full target, bypassing the zone, aim and at-speed checks - except while the turret is unwinding.
     * The automatic path is for autos while the nectar side is unmounted (NECTAR_MOUNTED, nectarReady).
     */
    public void setManualShoot(boolean shoot) {
        manualShoot = shoot;
    }

    public boolean shooting(){
        return state == states.SHOOT;
    }

    public void update() {
        inZone = forceZone || manualShoot || zone.inZone(predictiveAiming.predictPose());
        flywheel.update(inZone);

        switch (state) {
            case RESET:
                shooterGate.closeGate();
                // Keep aiming through the unwind settle. Freezing the aim for UNWIND_SETTLE_S left the turret
                // where the hive WAS while the robot kept turning (median 102 deg/s while turning, TeleOp
                // 2026-11-15), so it landed 100+ deg past and swung back. Tracking resumes from the unwound
                // copy (track() follows the copy nearest the last command); the gate stays closed until the
                // settle is over.
                aiming.update();
                if (!aiming.unwinding()) state = states.TRACK;
                break;
            case TRACK:
            case SHOOT:
                aiming.update();
                if (aiming.unwinding()) {
                    state = states.RESET;
                } else if (manualShoot) {
                    if (state != states.SHOOT) shootTimer.reset();
                    state = states.SHOOT;
                } else if (state == states.SHOOT) {
                    // A ball hitting the wheel dips its speed past the 5% entry band. Re-checking
                    // the entry condition here closed the gate on that ball and handed the intake
                    // back to the driver mid-shot, so hold SHOOT for a minimum open time and then
                    // on a wider speed band. Leaving the zone still ends it at once.
                    boolean hold = shootTimer.seconds() < SystemConstants.Shooter.MIN_GATE_OPEN_S
                            || (pollenReady(SystemConstants.Shooter.HOLD_SPEED_FRACTION)
                                && nectarReady(SystemConstants.Shooter.HOLD_SPEED_FRACTION));
                    if (!inZone || !hold) state = states.TRACK;
                } else if (inZone && pollenReady(SystemConstants.Flywheel.AT_SPEED_FRACTION)
                        && nectarReady(SystemConstants.Flywheel.AT_SPEED_FRACTION)) {
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

    private boolean pollenReady(double speedFraction) {
        return aiming.pollenAimed() && flywheel.atSpeedPollen(speedFraction);
    }

    /**
     * Nectar side aimed and at speed. With SystemConstants.Shooter.NECTAR_MOUNTED false the nectar side is not
     * there, so it counts as ready ONLY during an auto's Shoot event (forceZone - set by AutoMechanisms alone):
     * autos fire on the pollen side, and TeleOp's automatic path stays shut (it shoots on the right trigger).
     */
    private boolean nectarReady(double speedFraction) {
        if (!SystemConstants.Shooter.NECTAR_MOUNTED) {
            return forceZone;
        }
        return aiming.nectarAimed() && flywheel.atSpeedNectar(speedFraction);
    }

    public boolean ready() {
        return state == states.SHOOT;
    }
    public states getState(){
        return state;
    }

    /** RESET (unwinding, gate shut), TRACK (aiming, gate shut) or SHOOT (gate open) - for telemetry. */
    public String stateName() {
        return state.name();
    }

    public boolean inZone() { return zone.inZone(predictiveAiming.predictPose()); }
    /** Driver telemetry: pollen wheel velocity, its current commanded target (ticks/s), and the F voltage scale. */
    public double pollenVelocity() { return flywheel.pollenVelocity(); }
    public double pollenCommand() { return flywheel.pollenCommand(); }
    public double flywheelFScale() { return flywheel.fScale(); }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    public boolean nectarAimed() { return aiming.nectarAimed(); }
}
