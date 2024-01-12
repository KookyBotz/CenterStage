package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Config
@Autonomous(name = "Blue Auto")
public class BlueAuto extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.USING_DASHBOARD = false;
        Globals.COLOR = Side.BLUE;

        robot.init(hardwareMap);

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(robot::startCamera),
                        new WaitCommand(2000),

                        new InstantCommand(timer::reset),
                        // go to yellow pixel scoring pos
                        new PositionCommand(new Pose(37.75, 39.35, Math.PI / 2))
                                .alongWith(new PurplePixelExtendCommand()),

                        new PurplePixelDepositCommand(),


                        new PositionCommand(new Pose(38, 39.25, -0.02))
                                .alongWith(new FirstStackSetupCommand()),


                        new FirstStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -29.2, 0))
                                .andThen(new RelocalizeCommand())
                                .andThen(new PositionCommand(new Pose(35.75, -29, 0)))
                                .alongWith(new FirstDepositCommand()),


                        new PositionCommand(new Pose(38, 39, -0.02)),

                        new SecondStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -31.85, 0))
                                .andThen(new RelocalizeCommand())
                                .alongWith(new SecondDepositCommand()),


                        new PositionCommand(new Pose(38, 39, -0.02)),

                        new ThirdStackGrabCommand(),


                        new PositionCommand(new Pose(35.75, -31.85, 0))
                                .andThen(new RelocalizeCommand())
                                .alongWith(new ThirdDepositCommand()),

                        new InstantCommand(() -> endTime = timer.seconds())
                )
        );

        while (opModeIsActive()) {
            robot.read();
            CommandScheduler.getInstance().run();
            robot.periodic();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addLine(robot.localizer.getPose().toString());
            telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
            loopTime = loop;
            telemetry.update();

            robot.write();
            robot.clearBulkCache();
        }

        robot.closeCamera();
    }
}
