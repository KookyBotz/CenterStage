package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

@Disabled
@Config
@TeleOp(name = "ActuationMotorTest")
public class ActuationMotorTest extends OpMode {
    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public AnalogInput extensionPitchEnc;

    public DcMotorEx armMotor;


    public WActuatorGroup pitchActuator;
    public WActuatorGroup extensionActuator;

    private double loopTime = 0.0;

    public static double P = 1;
    public static double I = 0.0;
    public static double D = 0.045;
    public static double F = 0.07;

    public static double A = 10;
    public static double DA = 10;
    public static double V = 10;

    public static double armTargetPosition = 1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "extensionPitchMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // a, lift, went up with 0.1
        // b, arm, went down with 0.1

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.zero(2.086);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setWraparound(true);

        pitchActuator = new WActuatorGroup(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(1, 0, 0.045))
                .setMotionProfile(0, new ProfileConstraints(5, 100, 100))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.04)
                .setErrorTolerance(0.03);

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {
//        pitchActuator.setPID(P, I, D);
//        pitchActuator.setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, F);

        pitchActuator.read();


        if(gamepad1.a) pitchActuator.setMotionProfileTargetPosition(0);
        if(gamepad1.b) pitchActuator.setMotionProfileTargetPosition(3);
//        if(gamepad1.x) pitchActuator.setMotionProfile(0.2, new ProfileConstraints(V, A, DA));


        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

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

//        telemetry.addData("voltage", extensionEncoder.getVoltage());
        telemetry.addData("power", pitchActuator.getPower());
        telemetry.addData("targetPosition", armTargetPosition);
        telemetry.addData("targetPositionLift", pitchActuator.getState().x);
        telemetry.addData("currentPosition", pitchActuator.getPosition());
        telemetry.addData("reached", pitchActuator.hasReached());
//        ProfileState state = pitchActuator.getState();

//        telemetry.addData("v", state.v);
//        telemetry.addData("p", state.x);
//        telemetry.addData("a", state.a);
//        telemetry.addData("v", pitchActuator.getConstraints().velo);
//        telemetry.addData("time:", pitchActuator.timer.time());

//        telemetry.addData("liftPosition", liftTicks);
//        telemetry.addData("currentFF", pitchActuator.getCurrentFeedforward());

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
