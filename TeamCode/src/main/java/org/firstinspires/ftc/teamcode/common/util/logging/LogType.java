package org.firstinspires.ftc.teamcode.common.util.logging;

public enum LogType {
    ARM_POSITION("Arm Position", "radians"),
    ARM_POWER_AVERAGE("Arm Power Average", "power"),
    ARM_POWER("Arm Power", "power"),
    ARM_MAX_POWER("Arm Max Power", "power"),
    ARM_TARGET_POSITION("Arm Target Position", "radians"),
    ARM_NOTES("Notes", "");


//    EXTENSION_POSITION("Extension Position", "ticks"),
//    CONTOUR_LENGTH("Contour Length", "px"),
//    CONTOUR_AREA("Contour Area", "px"),
//    CENTROID_X("Centroid X", "px"),
//    CENTROID_Y("Centroid Y", "px"),
//    TAPE_CONTOUR_LENGTH("Tape Contour Length", "px"),
//    TAPE_CONTOUR_AREA("Tape Contour Area", "px"),
//    TAPE_CENTROID_X("Tape Centroid X", "px"),
//    TAPE_CENTROID_Y("Tape Centroid Y", "px");


    private final String header;
    private final String unit;

    LogType(String header, String unit) {
        this.header = header;
        this.unit = unit;
    }

    public String getHeader() {
        return this.header;
    }

    public String getUnit() {
        return this.unit;
    }
}