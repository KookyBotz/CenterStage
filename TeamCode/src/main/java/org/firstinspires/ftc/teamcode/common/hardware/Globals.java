package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.vision.Location;

public class Globals {

    public static Location SIDE = Location.FAR;
    /**
     * Match constants.
     */
    public static Location ALLIANCE = Location.RED;
    public static Location RANDOMIZATION = Location.LEFT;
    public static Location PRELOAD = Location.LEFT;
    public static Location ROUTE = Location.STAGEDOOR;

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

        if (PRELOAD == Location.RIGHT) index += 0;
        else if (PRELOAD == Location.LEFT) index += 1;

        if (RANDOMIZATION == Location.CENTER) index += 2;
        else if (RANDOMIZATION == Location.RIGHT) index += 4;

        if (ALLIANCE == Location.RED) index += 6;

        System.out.println("CURRENT INDEX");
        System.out.println(index);
//        System.out.println(Range.clip(index, 0, 11));

//        return Range.clip(index, 0, 5);
        return index;
    }
}
