package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.Actuator;

@Config
@TeleOp(name = "ActuationMotorTest")
public class ActuationMotorTest extends OpMode {
    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public AnalogInput extensionPitchEnc;

    public DcMotorEx liftMotor;
    public DcMotorEx armMotor;

    public Actuator pitchActuator;

    private double loopTime = 0.0;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 0.0;
    public static double targetPosition = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        liftMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "extensionPitchMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // a, lift, went up with 0.1
        // b, arm, went down with 0.1

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setNegArm(true);

        pitchActuator = new Actuator(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(0, 0, 0))
                .setFeedforward(Actuator.FeedforwardMode.ANGLE_BASED, 0);

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {

        pitchActuator.read();
        pitchActuator.setTargetPosition(targetPosition);

        pitchActuator.setPID(P, I, D);
        pitchActuator.setFeedforward(Actuator.FeedforwardMode.ANGLE_BASED, F);

        pitchActuator.periodic();

        pitchActuator.write();

//        if (gamepad1.a) {
//            liftMotor.setPower(0.1);
//        } else {
//            liftMotor.setPower(0.0);
//        }
//
//        if (gamepad1.b) {
//            armMotor.setPower(0.1);
//        } else {
//            armMotor.setPower(0.0);
//        }


//        telemetry.addData("radian reading", extensionPitchEncoder.getCurrentPosition());

        telemetry.addData("power", pitchActuator.getPower());
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("currentPosition", pitchActuator.getPosition());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
