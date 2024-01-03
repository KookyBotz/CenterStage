package org.firstinspires.ftc.teamcode.common.commandbase.drivecommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
public class PositionCommand extends CommandBase {
    ThreeWheelLocalizer localizer;
    Drivetrain drivetrain;
    Pose targetPose;

    public static double xP = 0.105;
    public static double xD = 0.0175;

    public static double yP = 0.105;
    public static double yD = 0.0175;

    public static double hP = 1.5;
    public static double hD = 0.075;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 2;
    public static double ALLOWED_HEADING_ERROR = 0.03;

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;
    private ElapsedTime stable;

    public PositionCommand(Pose targetPose) {
        this.drivetrain = robot.drivetrain;
        this.localizer = robot.localizer;
        this.targetPose = targetPose;


        xController.reset();
        yController.reset();
        hController.reset();
    }

    /**
     *
     */
    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robotPose = localizer.getPose();

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        Pose robotPose = localizer.getPose();
        Pose delta = targetPose.subtract(robotPose);

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > 5000 || stable.milliseconds() > 250;
    }

    public Pose getPower(Pose robotPose) {
        if(targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if(targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double y_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
        double x_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

        hPower = Range.clip(hPower, -0.7, 0.7);

        double correction = robot.getVoltage() / 13.5;
        x_rotated /= correction;
        y_rotated /= correction;
        hPower /= correction;

        return new Pose(x_rotated * 1.41, y_rotated, hPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}
