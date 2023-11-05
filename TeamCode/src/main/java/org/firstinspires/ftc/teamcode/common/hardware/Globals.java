package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.centerstage.Side;

public class Globals {

    public static Side COLOR = Side.RED;
    /**
     * Match constants.
     */
    public static Side SIDE = Side.LEFT;
    public static boolean IS_AUTO = false;
    public static boolean IS_USING_IMU = true;
    public static boolean USING_DASHBOARD = false;

    /**
     * Robot state constants.
     */
    public static boolean SWERVE_X = false;
    public static boolean IS_ON_PATH = true;
    public static boolean IS_PATHFINDING = false;

    //TODO tune these values
    public static double INTAKE_CLAW_OPEN = 0.0;
    public static double INTAKE_CLAW_CLOSED = 0.0;
}
