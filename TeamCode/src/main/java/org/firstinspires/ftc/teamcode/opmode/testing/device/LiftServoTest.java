package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class LiftServoTest extends OpMode {
    private ServoImplEx lift;

    public static double pos = 0.5;

    @Override
    public void init() {
        lift = hardwareMap.get(ServoImplEx.class, "lift");
        lift.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void loop() {
        lift.setPosition(pos);
    }
}
