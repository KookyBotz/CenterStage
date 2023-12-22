package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ArmLiftCommand extends InstantCommand {
    public ArmLiftCommand(double position){
        super(()-> RobotHardware.getInstance().armLiftServo.setPosition(position));
    }
}
