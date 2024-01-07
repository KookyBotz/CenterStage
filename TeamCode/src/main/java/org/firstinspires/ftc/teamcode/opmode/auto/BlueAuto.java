package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

@Photon
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

        robot.localizer.setPose(new Pose(63.65, 39.35, Math.PI / 2));

        while (!isStarted()) {
            telemetry.addLine("in init");
            telemetry.update();
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
//                        new InstantCommand(timer::reset),
                        // go to yellow pixel scoring pos
                        new WaitUntilCommand(() -> gamepad1.a),
                        new PositionCommand(new Pose(37.75, 39.35, Math.PI / 2)),
                        new WaitUntilCommand(() -> gamepad1.a),
//                                .alongWith(new PurplePixelExtendCommand()),

//                        new PurplePixelDepositCommand(),

                        new PositionCommand(new Pose(37.75, 39, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),
//                                .alongWith(new FirstStackSetupCommand()),

//                        new FirstStackGrabCommand(),

                        new PositionCommand(new Pose(35.75, -27, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new RelocalizeCommand(),
                        new WaitCommand(1000),
                        new WaitUntilCommand(() -> gamepad1.a),
//                                .alongWith(new FirstDepositCommand()),

                        new PositionCommand(new Pose(37.75, 39, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),

//                        new SecondStackGrabCommand(),

                        new PositionCommand(new Pose(35.75, -27, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new RelocalizeCommand(),
                        new WaitCommand(1000),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose(37.75, 39, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),

//                        new SecondStackGrabCommand(),

                        new PositionCommand(new Pose(35.75, -27, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new RelocalizeCommand(),
                        new WaitCommand(1000),
                        new WaitUntilCommand(() -> gamepad1.a),

                        new PositionCommand(new Pose(37.75, 39, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),

//                        new SecondStackGrabCommand(),

                        new PositionCommand(new Pose(35.75, -27, 0)),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new RelocalizeCommand(),
                        new WaitCommand(1000),
                        new WaitUntilCommand(() -> gamepad1.a),
                        new InstantCommand(() -> {
                            robot.dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                            robot.dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                            robot.dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                            robot.dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        })

//                                .alongWith(new SecondDepositCommand()),

//                        new InstantCommand(() -> endTime = timer.seconds())

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
        telemetry.addLine(robot.localizer.getPose().toString());
        telemetry.addData("Runtime: ", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache();
    }
}
