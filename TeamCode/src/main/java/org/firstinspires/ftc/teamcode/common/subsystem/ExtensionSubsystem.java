package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.common.util.wrappers.KSubsystem;

/**
 * Parts List:
 * <ul>
 *     <li>1x Motor Pitch</li>
 *     <li>1x Motor Extension</li>
 *     <li>1x Analog Encoder Pitch</li>
 * </ul>
 */
public class ExtensionSubsystem extends KSubsystem {

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

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }
}
