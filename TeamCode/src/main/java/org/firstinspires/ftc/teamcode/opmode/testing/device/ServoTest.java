package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

@Config
@TeleOp(name = "ServoTest")
@Disabled
public class ServoTest extends OpMode {
    private WServo servo1;
    private WServo servo2;
    private WServo servo3;
    private WServo servo4;

    public static double targetPos1 = 0.0;
    public static double targetPos2 = 0.0;
    public static double targetPos3 = 0.5;
    public static double targetPos4 = 0.5;

    @Override
    public void init() {
        servo1 = new WServo(hardwareMap.get(Servo.class, "servo1"));
        servo2 = new WServo(hardwareMap.get(Servo.class, "servo2"));
        servo3 = new WServo(hardwareMap.get(Servo.class, "servo3"));
        servo4 = new WServo(hardwareMap.get(Servo.class, "servo4"));
        servo4.setOffset(0.04);
        servo4.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // 0.435 intaking
        // 0.5 neutral
        // 0.75 scoring

        // LEFT SIDE CLAW, 0.09 CLOSE, 0.12 DEPOSIT OPEN, 0.4 OPEN

        // RIGHT SIDE CLAW, 0.51 CLOSE, 0.88 OPEN

        // 0.51, 0.53, 0.88

        // servo 3 = claw right
        // servo 4 - claw left

        // 0.475
        // 0.5

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
