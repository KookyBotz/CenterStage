package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp
@Disabled
public class ClimbTest extends OpMode {
    private CRServoImplEx left, right;

    @Override
    public void init() {
        left = hardwareMap.get(CRServoImplEx.class, "left");
        right = hardwareMap.get(CRServoImplEx.class, "right");

        left.setPwmRange(new PwmControl.PwmRange(500, 2500));
        right.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void loop() {
        left.setPower(gamepad1.left_stick_y);
        right.setPower(-gamepad1.left_stick_y);

    }
}
