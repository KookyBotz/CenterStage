package org.firstinspires.ftc.teamcode.common.util.logging;

public enum LogType {
    ARM_POSITION("Arm Position", "radians"),
    EXTENSION_POSITION("Extension Position", "ticks"),
    CONTOUR_LENGTH("Contour Length", "px"),
    CONTOUR_AREA("Contour Area", "px"),
    CENTROID_X("Centroid X", "px"),
    CENTROID_Y("Centroid Y", "px");

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