package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;

public final class AutoConstants {
    private AutoConstants() {}

    @Config
    @Configurable
    public static class Autonomous {
        // A Shoot step ends once the gate has been open SHOOT_OPEN_TIME in total, or after SHOOT_TIME at most
        // (AutoMechanisms.eventDone). It was a fixed 1500 ms: a turret unwind (~1.0-1.35 s, gate shut) at the
        // start of the step ate the shot - Auto Dry Run 2026-11-15, 2/4 Shoot steps fired, the gate open
        // 1.18 / 1.23 s in the two that did (SHOOT_OPEN_TIME keeps that).
        public static double SHOOT_TIME = 3000; //ms, cap
        public static double SHOOT_OPEN_TIME = 1200; //ms of gate open
        // While a path is followed, aim the turret from the path's END pose (fixed robot-relative angle)
        // instead of tracking the live pose: no unwinds in transit, arrives aimed. Host model of the RED
        // auto (ideal following): 5 -> 3 unwinds. AutoMechanisms.update.
        public static boolean PRE_AIM = true;
        public static double SCOOPULA_TRANSITION_TIME = 300; //ms
    }
}
