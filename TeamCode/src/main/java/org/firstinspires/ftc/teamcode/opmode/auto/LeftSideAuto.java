package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.commandbase.auto.GVFCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.common.drive.pathing.path.GVFPathFollower;
import org.firstinspires.ftc.teamcode.common.drive.pathing.path.HermitePath;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

@Autonomous(name = "LeftSideAuto2")
public class LeftSideAuto extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private WSubsystem drivetrain;
    private ThreeWheelLocalizer localizer;

    // path that goes forward and to the left
    private HermitePath trajectory = new HermitePath()
            .addPose(0, 0, new Vector2D(0, 100))
            .addPose(0, 20, new Vector2D(0, 500))
            .addPose(20, 40, new Vector2D(500, 0))
            .flip()
            .construct();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = true;
        Globals.IS_USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrivetrain();
        localizer = new ThreeWheelLocalizer();

        robot.addSubsystem(drivetrain);

        robot.enabled = true;

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new GVFCommand((Drivetrain) drivetrain, localizer, trajectory)
        );

        while (opModeIsActive()) {
            robot.clearBulkCache();
            robot.read();

            robot.periodic();
            localizer.periodic();

            telemetry.addData("nearestT", GVFPathFollower.nearestT);
            telemetry.addData("targetVel", GVFCommand.gvf);
            telemetry.addData("currentVel", localizer.getNewPoseVelocity());
            telemetry.addData("currentPose", localizer.getPoseEstimate());
            telemetry.addData("HEADING", localizer.getPos().heading);
            telemetry.addData("powers2", GVFCommand.powers2);
            telemetry.addData("delta", GVFCommand.hahaFunnyDelta);


            telemetry.update();
            CommandScheduler.getInstance().run();
            robot.write();
        }
    }
}
