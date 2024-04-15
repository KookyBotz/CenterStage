package org.firstinspires.ftc.teamcode.opmode.testing.pathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

@Disabled
@Autonomous(name = "odo tuning")
public class tuning extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;

    private boolean flag = true;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.init(hardwareMap);

        robot.read();

        robot.localizer.setPose(new Pose());


        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        schedule(
                new SequentialCommandGroup(
                        new PositionCommand(new Pose(0, 0, -Math.PI)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PositionCommand(new Pose()),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose(0, 0, Math.PI)),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose()),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PositionCommand(new Pose(0, 0, -Math.PI)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PositionCommand(new Pose()),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose(0, 0, Math.PI)),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose()),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PositionCommand(new Pose(0, 0, -Math.PI)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PositionCommand(new Pose()),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose(0, 0, Math.PI)),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose()),
                        new WaitUntilCommand(() -> gamepad1.a)


                )
        );
    }

    @Override
    public void run() {
        robot.read();

        super.run();
        robot.localizer.periodic();
        robot.drivetrain.periodic();

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
        robot.write();
        robot.clearBulkCache();

//        if (isStopRequested()) robot.closeCamera();
    }
}
