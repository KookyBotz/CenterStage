package org.firstinspires.ftc.teamcode.opmode.testing.pathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

@Autonomous(name = "LocalizationTest")
@Disabled
public class LocalizationTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
//    TwoWheelLocalizer localizer;
    private double loopTime = 0.0;

    private boolean flag = true;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.init(hardwareMap);

        robot.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.read();

//        robot.startIMUThread(this);
        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));
//        robot.reset();
//        robot.setStartOffset(Math.PI / 2);

//        localizer = new TwoWheelLocalizer();
//        localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));

        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        robot.read();
        Pose drive = new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01));
        robot.drivetrain.set(drive, 0);

        super.run();
        robot.localizer.periodic();
//        localizer.periodic();
        robot.drivetrain.periodic();
        robot.extension.periodic();

        Pose currentPose = robot.localizer.getPose();
        Pose globalTagPosition = robot.getAprilTagPosition();

        if (globalTagPosition == null) globalTagPosition = new Pose();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.addData("tag", globalTagPosition.toString());
        telemetry.addData("three", currentPose.toString());
//        telemetry.addData("two", localizer.getPose().toString());

        telemetry.addData("left", robot.localizer.positionLeft.getAsDouble());
        telemetry.addData("right", robot.localizer.positionRight.getAsDouble());
        telemetry.addData("front", robot.localizer.positionFront.getAsDouble());
        telemetry.addData("arm", robot.extensionEncoder.getPosition());
        telemetry.update();

//        if (gamepad1.a && flag) {
//            CommandScheduler.getInstance().schedule(new RelocalizeCommand());
//            flag = false;
////            localizer.setPose(globalTagPosition);
//        }
//
//        robot.write();
        robot.clearBulkCache();

//        if (isStopRequested()) robot.closeCamera();
    }
}
