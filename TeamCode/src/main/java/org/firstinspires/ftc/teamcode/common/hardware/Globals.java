package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KActuatorGroup;

public class Globals {

    /**
     * Match constants.
     */
    public static Side SIDE = Side.LEFT;
    public static boolean IS_AUTO = false;
    public static boolean IS_USING_IMU = true;

    /**
     * Robot state constants.
     */
    public static boolean SWERVE_X = false;
    public static boolean IS_ON_PATH = true;
    public static boolean IS_PATHFINDING = false;

    //TODO tune these values
    public static double INTAKE_CLAW_OPEN = 0.0;
    public static double INTAKE_CLAW_CLOSED = 0.0;

    public static class EXTENSION {
        public static double ARM_P = 1.3;
        public static double ARM_I = 0.0;
        public static double ARM_D = 0.035;
        public static double ARM_F_MIN = 0.05;
        public static double ARM_F_MAX = 0.13;
        public static KActuatorGroup.FeedforwardMode ARM_FF_MODE = KActuatorGroup.FeedforwardMode.ANGLE_BASED;

        public static double ARM_MP_V = 4.7;
        public static double ARM_MP_A = 20;
        public static double ARM_MP_D = 7.5;

        public static double EXTENSION_P = 0.016379;
        public static double EXTENSION_I = 0.0;
        public static double EXTENSION_D = 0.0;
        public static double EXTENSION_F = 0.0;
        public static KActuatorGroup.FeedforwardMode EXTENSION_FF_MODE = KActuatorGroup.FeedforwardMode.CONSTANT;

        public static double EXTENSION_MP_V = 0.0;
        public static double EXTENSION_MP_A = 0.0;
        public static double EXTENSION_MP_D = 0.0;
    }

    public static class INTAKE {
        public static double CLAW_A_CLOSED = 0.0;
        public static double CLAW_A_OPEN = 0.0;

        public static double CLAW_B_CLOSED = 0.0;
        public static double CLAW_B_OPEN = 0.0;

        public static double INTAKE_PIVOT_V = 0.0;
        public static double INTAKE_PIVOT_A = 0.0;
        public static double INTAKE_PIVOT_D = 0.0;
    }
}
