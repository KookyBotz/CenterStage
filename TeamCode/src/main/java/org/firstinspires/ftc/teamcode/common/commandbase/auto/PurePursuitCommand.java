package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitPath;

@Config
public class PurePursuitCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;

    public static double xP = 0.05;
    public static double xD = 0.5;
    public static double xF = 0;

    public static double yP = 0.05;
    public static double yD = 0.5;
    public static double yF = 0.0;

    public static double hP = 0.75;
    public static double hD = 0.02;
    public static double hF = 0;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);

    public PurePursuitCommand(Drivetrain drivetrain, Localizer localizer, PurePursuitPath purePursuitPath) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
    }

    @Override
    public void execute() {
        if (purePursuitPath.isFinished()) PID = true;

        Pose robotPose = localizer.getPos();
        Pose targetPose = purePursuitPath.update(robotPose);

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConfig.ALLOWED_HEADING_ERROR) finished = true;

        Pose delta = targetPose.subtract(robotPose);
        double y_rotated = delta.x * Math.cos(robotPose.heading) - delta.y * Math.sin(robotPose.heading);
        double x_rotated = delta.x * Math.sin(robotPose.heading) + delta.y * Math.cos(robotPose.heading);

        double hPower = -hController.calculate(0, delta.heading);

        if (PID) {
            delta.x = Math.signum(delta.x) * Math.sqrt(Math.abs(delta.x));
            delta.y = Math.signum(delta.y) * Math.sqrt(Math.abs(delta.y));
            double xPower = xController.calculate(0, x_rotated);
            double yPower = yController.calculate(0, y_rotated);

            System.out.println(new Pose(xPower / (Math.sqrt(2) / 2), yPower, hPower));
            drivetrain.set(new Pose(xPower / (Math.sqrt(2) / 2), yPower, hPower));
        } else {
            double xPercentage = x_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
            double yPercentage = y_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;

            System.out.println(new Pose(xPercentage / (Math.sqrt(2) / 2), yPercentage, hPower));
            drivetrain.set(new Pose(xPercentage / (Math.sqrt(2) / 2), yPercentage, hPower));
        }
    }

    @Override
    public boolean isFinished() {
        return PID && finished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}
