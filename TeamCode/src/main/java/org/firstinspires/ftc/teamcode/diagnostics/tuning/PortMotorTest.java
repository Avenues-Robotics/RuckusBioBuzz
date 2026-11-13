package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * DIAGNOSTIC ONLY. "Motor ports" tab of the Mechanism Tuner: run one motor by hub + port number,
 * raw power, ignoring every direction flag in SystemConstants.
 *
 * <p>Motors are found through the active configuration: a configured motor's controller reports the
 * same {@code getConnectionInfo()} as its {@link LynxModule} ("...; module N"), which pins it to a
 * hub. A port with no motor configured is driven through the controller another motor on that hub
 * already uses ({@code new DcMotorImplEx(controller, port)}), so a hub with no motor configured at
 * all cannot be reached. While a port runs, the motor is forced to FORWARD and RUN_WITHOUT_ENCODER;
 * both are restored on stop, so a flywheel or intake object holding the same motor is left as it was.
 */
final class PortMotorTest {

    static final class Hub {
        final LynxModule module;
        final String label;
        final int address;
        DcMotorController controller;
        final DcMotorEx[] motors = new DcMotorEx[4];
        final String[] names = {"", "", "", ""};

        Hub(LynxModule module, String label) {
            this.module = module;
            this.label = label;
            this.address = module.getModuleAddress();
        }
    }

    final List<Hub> hubs = new ArrayList<>();
    String error = "";

    // running test
    private Hub runHub;
    private int runPort = -1;
    private DcMotorEx runMotor;
    private DcMotor.RunMode savedMode;
    private DcMotorSimple.Direction savedDir;
    private double power;
    private double durationS;
    private long startNs;
    private int startTicks;
    double amps = Double.NaN, maxAmps;
    int ticks;
    double vel;
    String lastEnd = "";

    PortMotorTest(HardwareMap hw) {
        try {
            for (LynxModule m : hw.getAll(LynxModule.class)) {
                hubs.add(new Hub(m, m.isParent() ? "Control Hub" : "Expansion Hub " + m.getModuleAddress()));
            }
            // Control Hub first, then expansion hubs by address.
            java.util.Collections.sort(hubs, (a, b) -> a.module.isParent() != b.module.isParent()
                    ? (a.module.isParent() ? -1 : 1) : Integer.compare(a.address, b.address));
            for (DcMotor m : hw.getAll(DcMotor.class)) {
                DcMotorController c = m.getController();
                int port = m.getPortNumber();
                if (port < 0 || port > 3) continue;
                for (Hub h : hubs) {
                    if (!h.module.getConnectionInfo().equals(c.getConnectionInfo())) continue;
                    if (h.controller == null) h.controller = c;
                    if (h.motors[port] == null && m instanceof DcMotorEx) {
                        h.motors[port] = (DcMotorEx) m;
                        Set<String> n = hw.getNamesOf(m);
                        h.names[port] = n.isEmpty() ? "" : n.iterator().next();
                    }
                }
            }
        } catch (RuntimeException e) {
            error = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    boolean running() {
        return runMotor != null;
    }

    Hub hub(int address) {
        for (Hub h : hubs) if (h.address == address) return h;
        return null;
    }

    /** @return a message for the dashboard. */
    String start(int address, int port, double p, double seconds) {
        Hub h = hub(address);
        if (h == null) return "REFUSED motorPort: no hub at address " + address + ".";
        if (port < 0 || port > 3) return "REFUSED motorPort: port must be 0-3.";
        DcMotorEx m = h.motors[port];
        if (m == null) {
            if (h.controller == null) {
                return "REFUSED motorPort: " + h.label + " has no motor in the active configuration, so there is"
                        + " no controller to drive port " + port + " through. Configure any motor on it.";
            }
            m = new DcMotorImplEx(h.controller, port);
            h.motors[port] = m;
        }
        if (runMotor != null && runMotor != m) stop("switched to another port");
        if (runMotor != m) {
            savedMode = m.getMode();
            savedDir = m.getDirection();
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runHub = h;
            runPort = port;
            runMotor = m;
            startTicks = m.getCurrentPosition();
            maxAmps = 0;
            amps = Double.NaN;
        }
        power = Math.max(-1, Math.min(1, p));
        durationS = Math.max(0, seconds);
        startNs = System.nanoTime();
        lastEnd = "";
        m.setPower(power);
        return "Motor port: " + h.label + " port " + port + label(h, port) + " at raw power "
                + FlyRecorder.r(power, 2) + (durationS > 0 ? " for " + FlyRecorder.r(durationS, 1) + " s" : " until Stop")
                + " (direction flags ignored).";
    }

    void update(long now, boolean slow) {
        if (runMotor == null) return;
        try {
            ticks = runMotor.getCurrentPosition() - startTicks;
            vel = runMotor.getVelocity();
            if (slow) {
                amps = runMotor.getCurrent(CurrentUnit.AMPS);
                maxAmps = Math.max(maxAmps, amps);
            }
        } catch (RuntimeException ignored) {
            // a read failure is not a reason to keep a motor spinning or to crash the tuner
        }
        if (durationS > 0 && (now - startNs) / 1e9 >= durationS) {
            stop("timer (" + FlyRecorder.r(durationS, 1) + " s)");
        }
    }

    void stop(String why) {
        if (runMotor == null) return;
        try {
            runMotor.setPower(0);
            if (savedDir != null) runMotor.setDirection(savedDir);
            if (savedMode != null && savedMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) runMotor.setMode(savedMode);
        } finally {
            lastEnd = runHub.label + " port " + runPort + ": stopped (" + why + "), " + ticks + " ticks, peak "
                    + FlyRecorder.r(maxAmps, 2) + " A";
            runMotor = null;
            runHub = null;
            runPort = -1;
        }
    }

    private static String label(Hub h, int port) {
        return h.names[port].isEmpty() ? " (not in config)" : " (" + h.names[port] + ")";
    }

    String json() {
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\"error\":\"").append(TuningHub.esc(error)).append("\",\"hubs\":[");
        for (int i = 0; i < hubs.size(); i++) {
            Hub h = hubs.get(i);
            if (i > 0) sb.append(',');
            sb.append("{\"address\":").append(h.address)
                    .append(",\"label\":\"").append(TuningHub.esc(h.label))
                    .append("\",\"reachable\":").append(h.controller != null)
                    .append(",\"names\":[");
            for (int p = 0; p < 4; p++) {
                if (p > 0) sb.append(',');
                sb.append('"').append(TuningHub.esc(h.names[p])).append('"');
            }
            sb.append("]}");
        }
        sb.append("],\"running\":").append(runMotor != null);
        if (runMotor != null) {
            double el = (System.nanoTime() - startNs) / 1e9;
            sb.append(",\"address\":").append(runHub.address)
                    .append(",\"port\":").append(runPort)
                    .append(",\"power\":").append(FlyRecorder.r(power, 2))
                    .append(",\"durationS\":").append(FlyRecorder.r(durationS, 1))
                    .append(",\"elapsedS\":").append(FlyRecorder.r(el, 1))
                    .append(",\"ticks\":").append(ticks)
                    .append(",\"vel\":").append(FlyRecorder.r(vel, 0))
                    .append(",\"amps\":").append(Double.isNaN(amps) ? "null" : FlyRecorder.r(amps, 2))
                    .append(",\"maxAmps\":").append(FlyRecorder.r(maxAmps, 2));
        }
        sb.append(",\"lastEnd\":\"").append(TuningHub.esc(lastEnd)).append("\"}");
        return sb.toString();
    }
}
