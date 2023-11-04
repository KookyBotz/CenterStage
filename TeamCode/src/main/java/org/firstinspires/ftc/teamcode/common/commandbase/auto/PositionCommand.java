package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

public class PositionCommand extends CommandBase {
    Localizer localizer;
    Drivetrain drivetrain;
    Pose targetPose;

    public static double xP = 0.04;
    public static double xD = 0.0;
    public static double xF = 0.0;

    public static double yP = 0.04;
    public static double yD = 0.0;
    public static double yF = 0.0;

    public static double hP = 0.0;
    public static double hD = 0.0;
    public static double hF = 0;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 2; // inches
    public static double ALLOWED_HEADING_ERROR = 0.2; // radians

    public static double max_power = 0.5;
    public static double max_heading = 0.3;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
    }

    /**
     *
     */
    @Override
    public void execute() {
        Pose robotPose = localizer.getPos();

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    public Pose getPower(Pose robotPose) {
        Pose delta = targetPose.subt(robotPose);

        double angleDelta = MathUtils.getRotDist(-robotPose.heading, targetPose.heading);

        double xPower = xController.calculate(0, delta.x);
        double yPower = yController.calculate(0, delta.y);
        double hPower = hController.calculate(0, angleDelta) * Math.signum(angleDelta);

        double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
        double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

        double x_power = -x_rotated < -max_power ? -max_power :
                Math.min(-x_rotated, max_power);
        double y_power = -y_rotated < -max_power ? -max_power :
                Math.min(-y_rotated, max_power);
        double heading_power = MathUtils.clamp(hPower, -max_heading, max_heading);

        if (Math.abs(x_power) < 0.01) x_power = 0;
        if (Math.abs(y_power) < 0.01) y_power = 0;

        // todo replace first 12 with voltage
        return new Pose(xPower / 12 * 12, yPower / 12 * 12, hPower / 12 * 12);
    }
}
