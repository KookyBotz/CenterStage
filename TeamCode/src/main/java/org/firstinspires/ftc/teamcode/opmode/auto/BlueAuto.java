package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.AutoDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.FirstStackSetupCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelDepositCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.SecondStackGrabCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

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
        Globals.IS_USING_IMU = false;
        Globals.USING_DASHBOARD = true;
        Globals.COLOR = Side.BLUE;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.updateState(IntakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        robot.localizer.setPoseEstimate(new Pose2d(0, 0, 0));


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),
                        // go to yellow pixel scoring pos
                        new PositionCommand(new Pose(25, 0, 0))
                                .alongWith(new PurplePixelExtendCommand()),

                        new PurplePixelDepositCommand(),

                        new PositionCommand(new Pose(25, -0.25, -Math.PI / 2))
                                .alongWith(new FirstStackSetupCommand()),

                        new FirstStackGrabCommand(),

                        new PositionCommand(new Pose(27, -68.25, -Math.PI / 2))
                                .alongWith(new AutoDepositCommand()),

                        new InstantCommand(() -> endTime = timer.seconds())

//                        new PositionCommand(new Pose(26.5, 0.6, -Math.PI/2)),
//
//                        new SecondStackGrabCommand(),
//
//                        new PositionCommand(new Pose(27, -68, -Math.PI/2))
//                                .alongWith(new AutoDepositCommand())
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
        telemetry.addLine(robot.localizer.getPos().toString());
        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
