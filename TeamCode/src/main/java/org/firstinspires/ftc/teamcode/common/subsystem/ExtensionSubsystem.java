package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileState;

public class ExtensionSubsystem extends SubsystemBase {

    private final AsymmetricMotionProfile extensionProfile;
    private final AsymmetricMotionProfile pitchProfile;

    private ProfileState extensionState, angularState;

    private final PIDController extensionController, pitchController;

    public ExtensionSubsystem() {
        extensionProfile = null;
        pitchProfile = null;

        extensionController = null;
        pitchController = null;
    }

    @Override
    public void periodic() {

    }



}
