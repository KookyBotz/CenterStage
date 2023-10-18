package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;

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

    //TODO tune these values
    public static double INTAKE_PIVOT_FLAT = 0.0;
    public static double INTAKE_PIVOT_STORED = 0.0;
    public static double INTAKE_PIVOT_SCORING = 0.0;

    // TODO tune these values
    public static double EXTENSION_PITCH_P = 0.0;
    public static double EXTENSION_PITCH_I = 0.0;
    public static double EXTENSION_PITCH_D = 0.0;
    public static double EXTENSION_PITCH_F = 0.0;

    /**
     * Wait times for sequence tuning.
     */
    public static int WAIT_1 = 100;
}
