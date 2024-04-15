package org.firstinspires.ftc.teamcode.opmode.cycleauto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PurePursuitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.DepositExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.StackDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.StackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.StackSetupCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.YellowPixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.wallauto.YellowPixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Point;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.Waypoint;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;

@Disabled
@Autonomous(name = "🔵 Blue Close Wall Auto")
public class BlueCloseWallAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;

    private final Pose START = new Pose(62.9375, -16.35, Math.PI / 2);

    private final Pose FINAL_INTAKE = new Pose(44, 47, 0.275);

    private final Pose DEPOSIT = new Pose(44.75, -39.5, 0);

    private final Pose PURPLE = new Pose(47.5, -16.35, 1.925);
    private final Pose YELLOW = new Pose(42.5, -39.5, 0.03);

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.init(hardwareMap);

        robot.read();

        robot.setIMUStartOffset(START.heading);
        robot.localizer.setPose(START);


        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        PurePursuitPath intake = new PurePursuitPath(
                new Waypoint(YELLOW, 15),
                new Waypoint(new Point(58, -24), 15),
                new Waypoint(new Point(58, 36), 15),
                new Waypoint(FINAL_INTAKE, 15)
        );

        PurePursuitPath deposit = new PurePursuitPath(
                new Waypoint(FINAL_INTAKE, 15),
                new Waypoint(new Pose(60, 36, 0), 15),
                new Waypoint(new Pose(58, -24, 0), 15),
                new Waypoint(DEPOSIT, 15)
        );

        PurePursuitPath intake2 = new PurePursuitPath(
                new Waypoint(DEPOSIT, 15),
                new Waypoint(new Point(58, -24), 15),
                new Waypoint(new Point(58, 36), 15),
                new Waypoint(FINAL_INTAKE, 15)
        );

        PurePursuitPath deposit2 = new PurePursuitPath(
                new Waypoint(FINAL_INTAKE, 15),
                new Waypoint(new Pose(60, 36, 0), 15),
                new Waypoint(new Pose(58, -24, 0), 15),
                new Waypoint(DEPOSIT, 15)
        );

        PurePursuitPath intake3 = new PurePursuitPath(
                new Waypoint(DEPOSIT, 15),
                new Waypoint(new Point(58, -24), 15),
                new Waypoint(new Point(58, 36), 15),
                new Waypoint(FINAL_INTAKE, 15)
        );

        PurePursuitPath deposit3 = new PurePursuitPath(
                new Waypoint(FINAL_INTAKE, 15),
                new Waypoint(new Pose(60, 36, 0), 15),
                new Waypoint(new Pose(58, -24, 0), 15),
                new Waypoint(DEPOSIT, 15)
        );


        schedule(
                new SequentialCommandGroup(
                        //purple
                        new WaitUntilCommand(()->gamepad1.a),

                        new PositionCommand(PURPLE),
                        new WaitUntilCommand(() -> gamepad1.a),

                        //yellow
                        new PositionCommand(YELLOW),
                        new WaitUntilCommand(() -> gamepad1.a),


                        new PurePursuitCommand(intake),
                        new WaitUntilCommand(() -> gamepad1.a),


                        new PurePursuitCommand(deposit),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PurePursuitCommand(intake2),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PurePursuitCommand(deposit2),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PurePursuitCommand(intake3),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PurePursuitCommand(deposit3),
                        new WaitUntilCommand(() -> gamepad1.a)

                )
        );

    }

    @Override
    public void run() {
        super.run();
        robot.clearBulkCache();
        robot.read();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("pose: ", robot.localizer.getPose());
        telemetry.addData("atag: ", robot.getAprilTagPosition());
        telemetry.addData("x", robot.localizer.distanceMeasurement);
        telemetry.addData("h", robot.getAngle());
        telemetry.update();
        loopTime = loop;

        robot.write();

        if (gamepad1.b) robot.localizer.setLateral(robot.localizer.distanceMeasurement);
        if (gamepad1.x) robot.localizer.setPose(robot.getAprilTagPosition());
    }
}
