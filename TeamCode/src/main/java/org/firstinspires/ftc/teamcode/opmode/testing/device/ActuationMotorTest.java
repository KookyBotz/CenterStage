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
    private double p = 0.0;
    private double i = 0.0;
    private double d = 0.0;
    public static double F_MIN = 0.0;
    public static double F_MAX = 0.0;
    public static double armTargetPosition = 1.57;
    public static double newTargetPos = 0;
    private double liftTargetPosition = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        pitchActuator = new WActuatorGroup(armMotor, extensionPitchEncoder)
                .setPIDController(new PIDController(1, 0.1, 0.05))
                .setFeedforward(WActuatorGroup.FeedforwardMode.ANGLE_BASED,  0.07)
                .setErrorTolerance(0.01);

        pitchActuator.setTargetPosition(Math.PI / 2);

        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {

        pitchActuator.read();
        pitchActuator.read();
        pitchActuator.read();
        pitchActuator.read();
        pitchActuator.read();
        pitchActuator.read();
        pitchActuator.read();



        if(gamepad1.a) pitchActuator.setTargetPosition(armTargetPosition);


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
        telemetry.addData("targetPositionLift", pitchActuator.getTargetPosition());
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
