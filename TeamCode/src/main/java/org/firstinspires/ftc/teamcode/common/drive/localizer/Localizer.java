package org.firstinspires.ftc.teamcode.common.drive.localizer;


import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}