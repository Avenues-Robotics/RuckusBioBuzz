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

    /** Shoot by itself this loop: an auto's Shoot event, or TeleOp in the zone (zoneAutoShoot, not locked). */
    private boolean autoFire;
    private boolean forceZone;
    /** TeleOp only: fire by itself when the robot is in the zone map. Autos never set it - they fire on events. */
    private boolean zoneAutoShoot;
    /** TeleOp shooter lock: turrets straight ahead, no zone logic, the flywheel always at full target. */
    private boolean lockForward;
    /** Robot pose when the gate last opened - driver 2's aim corrections are judged from it (AimOffsets). */
    private Pose lastShotPose;
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
    /** Autos only: pre-aim the turret from a path's end pose (Turret.setAimPose); null = live tracking. */
    public void setAimPose(com.pedropathing.math.Pose robotPose) {
        aiming.setAimPose(robotPose);
    }

    public void setManualShoot(boolean shoot) {
        manualShoot = shoot;
    }

    /** TeleOp: fire automatically in the zone (the autos leave this false and fire on their Shoot events). */
    public void setZoneAutoShoot(boolean on) {
        zoneAutoShoot = on;
    }

    /** TeleOp shooter lock: turrets straight ahead, zone logic off, flywheel always at full target. */
    public void setLockForward(boolean lock) {
        lockForward = lock;
        aiming.setLockForward(lock);
    }

    public boolean lockedForward() {
        return lockForward;
    }

    /** TeleOp driver 2: aim-point shift (field frame, in) and flywheel y-intercept (ticks/s). */
    public void setAimOffsets(double goalDx, double goalDy, double speedOffset) {
        aiming.setGoalOffset(goalDx, goalDy);
        flywheel.setSpeedOffset(speedOffset);
    }

    /** The point the shooter aims at for a robot at {@code robot} (hive + driver 2's offset). */
    public Pose goal(Pose robot) {
        return aiming.goal(robot);
    }

    /** Robot pose when the gate last opened, or null before the first shot. */
    public Pose lastShotPose() {
        return lastShotPose;
    }

    public boolean shooting(){
        return state == states.SHOOT;
    }

    public void update() {
        boolean zoneIn = zone.inZone(predictiveAiming.predictPose());
        autoFire = forceZone || (zoneAutoShoot && !lockForward && zoneIn);
        // Full speed whenever a shot may come: automatic, the driver's trigger, the zone (pre-spin, unchanged for
        // the autos), and all of lock mode - there is no zone there, the driver fires when the robot faces the goal.
        flywheel.update(autoFire || manualShoot || zoneIn || lockForward);
        states before = state;

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
                    if (!autoFire || !hold) state = states.TRACK;
                } else if (autoFire && pollenReady(SystemConstants.Flywheel.AT_SPEED_FRACTION)
                        && nectarReady(SystemConstants.Flywheel.AT_SPEED_FRACTION)) {
                    state = states.SHOOT;
                    shootTimer.reset();
                }
                if (state == states.SHOOT) {
                    shooterGate.openGate();
                    if (before != states.SHOOT) {
                        lastShotPose = predictiveAiming.predictPose(0);
                    }

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
     * there and counts as ready, so automatic shots go on the pollen side alone. WHEN a shot may be automatic is
     * autoFire: an auto's Shoot event (forceZone), or TeleOp in the zone (zoneAutoShoot, not locked). Until
     * 2026-11-15 this returned forceZone instead, which kept TeleOp's in-zone shot shut.
     */
    private boolean nectarReady(double speedFraction) {
        if (!SystemConstants.Shooter.NECTAR_MOUNTED) {
            return true;
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

    /** TeleOp relocalize: true parks the turrets at Turret.RESET_POSE (nothing past the back) and keeps the gate shut. */
    public void setTurretParked(boolean parked) {
        aiming.setParked(parked);
    }

    public boolean inZone() { return zone.inZone(predictiveAiming.predictPose()); }
    /** Driver telemetry: pollen wheel velocity, its current commanded target (ticks/s), and the F voltage scale. */
    public double pollenVelocity() { return flywheel.pollenVelocity(); }
    public double pollenCommand() { return flywheel.pollenCommand(); }
    public double flywheelFScale() { return flywheel.fScale(); }
    public boolean pollenAimed() { return aiming.pollenAimed(); }
    /** Shoot-on-the-move lead time last used by the turret, ms. */
    public double sotmLeadMs() { return aiming.sotmLeadSeconds() * 1000.0; }
    public boolean nectarAimed() { return aiming.nectarAimed(); }
}
