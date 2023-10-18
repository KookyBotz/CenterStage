package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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

    public DcMotorEx extensionMotor;
    public DcMotorEx armMotor;

    public Motor.Encoder extensionEncoder;

    public Actuator pitchActuator;
    public Actuator extensionActuator;

    private double loopTime = 0.0;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public double p = 0.0;
    public double i = 0.0;
    public double d = 0.0;
    public static double F_MIN = 0.05;
    public static double F_MAX = 0.13;
    public static double targetPosition = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "extensionPitchMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionEncoder = new MotorEx(hardwareMap, "dtFrontLeftMotor").encoder;

        // a, lift, went up with 0.1
        // b, arm, went down with 0.1

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setNegArm(true);

        pitchActuator = new Actuator(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(0, 0, 0))
                .setFeedforward(Actuator.FeedforwardMode.ANGLE_BASED, F_MIN, F_MAX);

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {

        pitchActuator.read();
        pitchActuator.setTargetPosition(targetPosition);

        if (P != p || I != i || D != d) {
            pitchActuator.setPIDController(new PIDController(P, I, D));
            p = P;
            i = I;
            d = D;
        }

//        pitchActuator.setFeedforward(Actuator.FeedforwardMode.ANGLE_BASED, 0.05, 0.12);

        double liftTicks = extensionEncoder.getPosition();
        pitchActuator.updateFeedforward(liftTicks / 560.0);

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
        telemetry.addData("liftPosition", liftTicks);
        telemetry.addData("currentFF", pitchActuator.getCurrentFeedforward());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
