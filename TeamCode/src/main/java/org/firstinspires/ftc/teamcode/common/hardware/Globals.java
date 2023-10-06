package org.firstinspires.ftc.teamcode.common.hardware;


public class Globals {

    public enum Side {
        LEFT, RIGHT
    }

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

    /**
     * Wait times for sequence tuning.
     */
    public static int WAIT_1 = 100;
}
