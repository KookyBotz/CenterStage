package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

@Config
@TeleOp(name = "ActuationMotorTest")
public class ActuationMotorTest extends OpMode {
    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public AnalogInput extensionPitchEnc;

    public DcMotorEx extensionMotor;
    public DcMotorEx armMotor;

    public WEncoder extensionEncoder;

    public WActuatorGroup pitchActuator;
    public WActuatorGroup extensionActuator;

    private double loopTime = 0.0;

    public static double v = 0.0;
    public static double a1 = 0.0;
    public static double a2 = 0.0;

    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F_MIN = 0.0;
    public static double F_MAX = 0.0;
    public static double armTargetPosition = 1.57;
    public static double liftTargetPosition = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "extensionPitchMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionEncoder = new WEncoder(new MotorEx(hardwareMap, "dtFrontLeftMotor").encoder);

        // a, lift, went up with 0.1
        // b, arm, went down with 0.1

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.zero(2.086);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setWraparound(true);

        this.extensionActuator = new WActuatorGroup(extensionMotor, extensionEncoder)
                .setPIDController(new PIDController(0, 0, 0))
//                .setMotionProfile(new ProfileConstraints(0, 0, 0))
                .setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, 0.0);

        pitchActuator = new WActuatorGroup(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(1.3, 0, 0.035))
                .setMotionProfile(Math.PI / 2, new ProfileConstraints(4.7, 20, 7.5))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED, 0.05, 0.13);

        pitchActuator.setTargetPosition(Math.PI / 2);

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {

        pitchActuator.read();
        extensionActuator.read();

        if (gamepad1.a) {
            pitchActuator.setMotionProfileTargetPosition(armTargetPosition);
        }

        if (gamepad1.b) {
            extensionActuator.setTargetPosition(liftTargetPosition);
        }

        if (gamepad1.y) {
            extensionActuator.setPIDController(new PIDController(P, I, D));
            extensionActuator.setFeedforward(WActuatorGroup.FeedforwardMode.CONSTANT, F_MIN);
        }

        double liftTicks = extensionEncoder.getPosition();
        pitchActuator.updateFeedforward(liftTicks / 560.0);

        pitchActuator.periodic();
        extensionActuator.periodic();

        pitchActuator.write();
        extensionActuator.write();

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
        telemetry.addData("power", extensionActuator.getPower());
        telemetry.addData("targetPosition", liftTargetPosition);
        telemetry.addData("currentPosition", extensionActuator.getPosition());
        telemetry.addData("Current", extensionMotor.getCurrent(CurrentUnit.AMPS));
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
