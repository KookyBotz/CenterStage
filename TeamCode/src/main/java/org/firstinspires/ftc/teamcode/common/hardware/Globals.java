package org.firstinspires.ftc.teamcode.common.hardware;

import org.firstinspires.ftc.teamcode.common.vision.Location;

public class Globals {

    public static Location SIDE = Location.FAR;
    /**
     * Match constants.
     */
    public static Location ALLIANCE = Location.RED;
    public static Location RANDOMIZATION = Location.LEFT;
    public static Location PRELOAD = Location.LEFT;

    public static boolean IS_AUTO = false;

    /**
     * Robot State Constants
     */
    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;

    public static void startScoring() {
        IS_SCORING = true;
        IS_INTAKING = false;
    }

    public static void stopScoring(){
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void stopIntaking() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static int getTargetIndex() {
        int index = 0;

        index += (ALLIANCE == Location.BLUE ? 0 : 6);
        index += (PRELOAD == Location.RIGHT ? 0 : 1);
        if (RANDOMIZATION == Location.CENTER) index += 2;
        else if (RANDOMIZATION == Location.RIGHT) index += 4;

        return index;
    }
}
