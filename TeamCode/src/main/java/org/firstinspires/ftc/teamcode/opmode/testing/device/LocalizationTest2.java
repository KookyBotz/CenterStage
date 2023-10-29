package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

public class LocalizationTest2 extends LinearOpMode {
    public WEncoder podLeft, podRight, podFront;

    @Override
    public void runOpMode() throws InterruptedException {
        this.podLeft = new WEncoder(new MotorEx(hardwareMap, "dtBackRightMotor").encoder);
        this.podFront = new WEncoder(new MotorEx(hardwareMap, "dtFrontRightMotor").encoder);
        this.podRight = new WEncoder(new MotorEx(hardwareMap, "dtBackLeftMotor").encoder);
    }
}
