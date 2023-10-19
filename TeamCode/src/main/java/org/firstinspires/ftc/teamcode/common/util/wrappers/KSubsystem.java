package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class KSubsystem extends SubsystemBase {

    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
