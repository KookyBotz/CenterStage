package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
@Disabled
public class DroneTest extends OpMode {
    private ServoImplEx drone;

    public static double pos = 0.5;

    @Override
    public void init() {
        drone = hardwareMap.get(ServoImplEx.class, "drone");
    }

    @Override
    public void loop() {
        drone.setPosition(pos);
    }
}
