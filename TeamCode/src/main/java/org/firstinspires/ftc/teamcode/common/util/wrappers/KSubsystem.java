package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class KSubsystem extends SubsystemBase {

    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
