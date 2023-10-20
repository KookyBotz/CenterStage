package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

@Config
@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {
    private WServo servo1;
    private WServo servo2;
    private WServo servo3;
    private WServo servo4;

    public static double targetPos1 = 0.0;
    public static double targetPos2 = 0.0;
    public static double targetPos3 = 0.0;
    public static double targetPos4 = 0.0;

    @Override
    public void init() {
        servo1 = (WServo) hardwareMap.get(Servo.class, "servo1");
        servo2 = (WServo) hardwareMap.get(Servo.class, "servo2");
        servo3 = (WServo) hardwareMap.get(Servo.class, "servo3");
        servo4 = (WServo) hardwareMap.get(Servo.class, "servo4");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo1.setPosition(targetPos1);
        }
        // claw right side of bot

        if (gamepad1.b) {
            servo2.setPosition(targetPos2);
        }
        // claw left side of bot

        if (gamepad1.x) {
            servo3.setPosition(targetPos3);
        }

        if (gamepad1.y) {
            servo4.setPosition(targetPos4);
        }
    }
}
