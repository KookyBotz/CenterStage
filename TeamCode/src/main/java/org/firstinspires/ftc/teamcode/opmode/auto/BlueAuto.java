package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstStackSetupCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.SecondDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.SecondStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.ThirdDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.ThirdStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Blue Auto")
public class BlueAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();


    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.read();

        robot.startIMUThread(this);
        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));
        robot.reset();
        robot.setStartOffset(Math.PI / 2);

        while (!isStarted()) {
            telemetry.addLine("in init");
            telemetry.update();
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),
                        // go to yellow pixel scoring pos
                        new PositionCommand(new Pose(37.75, 39.35, Math.PI / 2))
                                .alongWith(new PurplePixelExtendCommand()),

                        new PurplePixelDepositCommand(),


                        new PositionCommand(new Pose(38, 39.25, 0))
                                .alongWith(new FirstStackSetupCommand()),


                        new FirstStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -29, 0))
                                .andThen(new RelocalizeCommand())
                                .andThen(new PositionCommand(new Pose(35.75, -29, 0)))
                                .alongWith(new FirstDepositCommand()),


                        new PositionCommand(new Pose(38, 39, 0)),

                        new SecondStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -29, 0))
                                .andThen(new RelocalizeCommand())
                                .andThen(new PositionCommand(new Pose(35.75, -29, 0)))
                                .alongWith(new SecondDepositCommand()),


                        new PositionCommand(new Pose(38, 39, 0)),

                        new ThirdStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -29, 0))
                                .andThen(new RelocalizeCommand())
                                .andThen(new PositionCommand(new Pose(34, -29, 0)))
                                .alongWith(new ThirdDepositCommand()),

                        new InstantCommand(() -> endTime = timer.seconds())
                )
        );
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addLine(robot.localizer.getPose().toString());
        telemetry.addLine(String.valueOf(robot.localizer.positionFront.getAsDouble()));
        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
