package org.firstinspires.ftc.teamcode.common.drive.localizer;


import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}