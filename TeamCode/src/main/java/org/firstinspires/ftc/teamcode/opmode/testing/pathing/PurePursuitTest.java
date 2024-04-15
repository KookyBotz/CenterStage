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

import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;

@Disabled
@Autonomous(name = "Pure Pursuit Test")
public class PurePursuitTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;

    private Pose INTAKE = new Pose(11.25, 39.35, 0);
    private Pose DEPOSIT = new Pose(11.25, -35, 0);

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.init(hardwareMap);

        robot.read();

        robot.localizer.setPose(new Pose(16.85, 39.35, 0));

        PurePursuitPath depositPath = new PurePursuitPath(
                new Waypoint(INTAKE, 12),
//                new Waypoint(new Point(16, -18.5), 12),
                new Waypoint(DEPOSIT, 12)
        );

        PurePursuitPath intakePath = new PurePursuitPath(
                new Waypoint(DEPOSIT, 12),
//                new Waypoint(new Point(11.25, 27.35), 12),
                new Waypoint(INTAKE, 12)
        );

        PurePursuitPath depositPath2 = new PurePursuitPath(
                new Waypoint(INTAKE, 12),
//                new Waypoint(new Point(16, -18.5), 12),
                new Waypoint(DEPOSIT, 12)
        );

        PurePursuitPath intakePath2 = new PurePursuitPath(
                new Waypoint(DEPOSIT, 12),
//                new Waypoint(new Point(11.25, 27.35), 12),
                new Waypoint(INTAKE, 12)
        );

        PurePursuitPath depositPath3 = new PurePursuitPath(
                new Waypoint(INTAKE, 12),
//                new Waypoint(new Point(16, -18.5), 12),
                new Waypoint(DEPOSIT, 12)
        );

        PurePursuitPath intakePath3 = new PurePursuitPath(
                new Waypoint(DEPOSIT, 12),
//                new Waypoint(new Point(11.25, 27.35), 12),
                new Waypoint(INTAKE, 12)
        );



        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        schedule(
                new SequentialCommandGroup(
                        new PositionCommand(INTAKE),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PurePursuitCommand(depositPath),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PurePursuitCommand(intakePath),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PurePursuitCommand(depositPath2),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PurePursuitCommand(intakePath2),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PurePursuitCommand(depositPath3),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PurePursuitCommand(intakePath3)
                )
        );

    }

    @Override
    public void run() {
        super.run();
        robot.read();

        robot.clearBulkCache();

        robot.drivetrain.periodic();
        robot.localizer.periodic();


//        Logger.logData(LogType.L, String.valueOf(robot.localizer.positionLeft.getAsDouble()));
//        Logger.logData(LogType.R, String.valueOf(robot.localizer.positionRight.getAsDouble()));
//        Logger.logData(LogType.F, String.valueOf(robot.localizer.positionFront.getAsDouble()));
//        Logger.logData(LogType.X, String.valueOf(robot.localizer.getPose().x));
//        Logger.logData(LogType.Y, String.valueOf(robot.localizer.getPose().y));
//        Logger.logData(LogType.H, String.valueOf(robot.localizer.getPose().heading));

//        if (gamepad1.a) robot.localizer.setLateral(robot.getAprilTagPosition());

        if (gamepad1.b) robot.localizer.setLateral(robot.localizer.distanceMeasurement);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("pose: ", robot.localizer.getPose());
        telemetry.addData("atag: ", robot.getAprilTagPosition());
        telemetry.addData("x: ", robot.localizer.distanceMeasurement);
        telemetry.update();
        loopTime = loop;

        robot.write();

//        if (isStopRequested()) CSVInterface.log();
    }
}
