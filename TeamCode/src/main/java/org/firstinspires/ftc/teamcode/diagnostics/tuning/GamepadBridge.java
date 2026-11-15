package org.firstinspires.ftc.teamcode.diagnostics.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.lang.reflect.Method;
import java.util.Locale;
import java.util.Map;

/**
 * DIAGNOSTIC ONLY - never ships. Feeds gamepad1 of whatever OpMode is running from the /tune
 * "Driver Station" tab, for driving with no Driver Station.
 *
 * <p>Why not FTC Dashboard's gamepad (2026-11-15): Dashboard 0.5.1 writes into
 * {@code opMode.gamepad1}, but on SDK 12 an iterative {@link OpMode} re-copies its
 * {@code latestGamepad1Data} into {@code gamepad1} before every {@code loop()}
 * ({@code OpMode.internalPreUserCode}), and that data only ever comes from a Driver Station. With
 * no DS the injected sticks are wiped before TeleOp reads them; only momentary presses slip
 * through. This class hands the state in through the same door the DS uses,
 * {@code OpModeInternal.newGamepadDataAvailable(Gamepad, Gamepad)} (package-private, so by
 * reflection). {@code OpModeManagerImpl.runActiveOpMode} only calls that when the DS data changes,
 * so with no DS nothing overwrites it.
 *
 * <p>Safety: if no update arrives for {@link #WATCHDOG_MS}, both gamepads are set to neutral. The
 * tab sends at 20 Hz while it is visible and enabled, and neutral when it stops.
 */
final class GamepadBridge {
    static final long WATCHDOG_MS = 400;

    private static volatile long lastUpdateMs;
    private static volatile boolean live;
    private static volatile OpMode target;
    private static Method newData;
    private static Thread watchdog;

    private GamepadBridge() {
    }

    /** Query params: lx ly rx ry lt rt (floats), btn = comma list of pressed button field names. */
    static synchronized String update(Map<String, String> p) {
        OpMode op = activeOpMode();
        if (op == null) {
            return "{\"ok\":false,\"message\":\"no OpMode running\"}";
        }
        Gamepad g1 = new Gamepad();
        g1.left_stick_x = f(p, "lx");
        g1.left_stick_y = f(p, "ly");
        g1.right_stick_x = f(p, "rx");
        g1.right_stick_y = f(p, "ry");
        g1.left_trigger = f(p, "lt");
        g1.right_trigger = f(p, "rt");
        String btn = p.get("btn");
        if (btn != null && !btn.isEmpty()) {
            for (String b : btn.split(",")) {
                press(g1, b.trim());
            }
        }
        try {
            inject(op, g1, new Gamepad());
        } catch (Exception e) {
            RobotLog.ee("GamepadBridge", e, "inject failed");
            return "{\"ok\":false,\"message\":\"" + e.getClass().getSimpleName() + "\"}";
        }
        target = op;
        lastUpdateMs = System.currentTimeMillis();
        live = true;
        startWatchdog();
        return String.format(Locale.US, "{\"ok\":true,\"opMode\":\"%s\"}", activeName());
    }

    /** Neutral sticks and buttons now (tab disabled / hidden / STOP). */
    static synchronized String neutral() {
        OpMode op = target;
        live = false;
        if (op != null) {
            try {
                inject(op, new Gamepad(), new Gamepad());
            } catch (Exception e) {
                RobotLog.ee("GamepadBridge", e, "neutral failed");
            }
        }
        return "{\"ok\":true}";
    }

    private static OpMode activeOpMode() {
        OpModeManagerImpl mgr = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        if (mgr == null || OpModeManager.DEFAULT_OP_MODE_NAME.equals(mgr.getActiveOpModeName())) {
            return null;
        }
        return mgr.getActiveOpMode();
    }

    private static String activeName() {
        OpModeManagerImpl mgr = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        return mgr == null ? "" : mgr.getActiveOpModeName().replace("\"", "'");
    }

    private static void inject(OpMode op, Gamepad g1, Gamepad g2) throws Exception {
        if (newData == null) {
            Class<?> internal = Class.forName("com.qualcomm.robotcore.eventloop.opmode.OpModeInternal");
            Method m = internal.getDeclaredMethod("newGamepadDataAvailable", Gamepad.class, Gamepad.class);
            m.setAccessible(true);
            newData = m;
        }
        newData.invoke(op, g1, g2);
    }

    private static void startWatchdog() {
        if (watchdog != null && watchdog.isAlive()) {
            return;
        }
        watchdog = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    return;
                }
                if (live && System.currentTimeMillis() - lastUpdateMs > WATCHDOG_MS) {
                    neutral();
                }
            }
        }, "GamepadBridge watchdog");
        watchdog.setDaemon(true);
        watchdog.start();
    }

    private static float f(Map<String, String> p, String k) {
        try {
            float v = Float.parseFloat(p.get(k));
            return Float.isNaN(v) ? 0f : Math.max(-1f, Math.min(1f, v));
        } catch (Exception e) {
            return 0f;
        }
    }

    private static void press(Gamepad g, String b) {
        switch (b) {
            case "a": g.a = true; break;
            case "b": g.b = true; break;
            case "x": g.x = true; break;
            case "y": g.y = true; break;
            case "left_bumper": g.left_bumper = true; break;
            case "right_bumper": g.right_bumper = true; break;
            case "dpad_up": g.dpad_up = true; break;
            case "dpad_down": g.dpad_down = true; break;
            case "dpad_left": g.dpad_left = true; break;
            case "dpad_right": g.dpad_right = true; break;
            case "start": g.start = true; break;
            case "back": g.back = true; break;
            case "guide": g.guide = true; break;
            case "left_stick_button": g.left_stick_button = true; break;
            case "right_stick_button": g.right_stick_button = true; break;
            default: break;
        }
    }
}
