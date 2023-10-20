package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;
    private Servo servo4;

    public static double targetPos1 = 0.0;
    public static double targetPos2 = 0.0;
    public static double targetPos3 = 0.0;
    public static double targetPos4 = 0.0;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
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
