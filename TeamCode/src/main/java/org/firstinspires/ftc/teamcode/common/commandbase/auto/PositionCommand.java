package org.firstinspires.ftc.teamcode.common.commandbase.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.Localizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

@Config
public class PositionCommand extends CommandBase {
    Localizer localizer;
    Drivetrain drivetrain;
    Pose targetPose;

    public static double xP = 0.025;
    public static double xD = 0.04;
    public static double xF = 0.0;

    public static double yP = 0.025;
    public static double yD = 0.04;
    public static double yF = 0.0;

    public static double hP = 0.75;
    public static double hD = 0.02;
    public static double hF = 0;

    public static double kStatic = 0.06;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1; // inches
    public static double ALLOWED_HEADING_ERROR = 0.04; // radians

    private RobotHardware robot = RobotHardware.getInstance();

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
        Pose robotPose = localizer.getPos();
        Pose delta = targetPose.subtract(robotPose);
        System.out.println(delta.toVec2D().magnitude() + " " + delta.heading);

        return delta.toVec2D().magnitude() < ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(delta.heading) < ALLOWED_HEADING_ERROR;
    }

    public Pose getPower(Pose robotPose) {
        Pose delta = targetPose.subtract(robotPose);

        double xPower = xController.calculate(0, delta.x);
        double yPower = yController.calculate(0, delta.y);
        double hPower = -hController.calculate(0, delta.heading);

        double x_rotated = xPower * Math.cos(robotPose.heading) - yPower * Math.sin(robotPose.heading);
        double y_rotated = xPower * Math.sin(robotPose.heading) + yPower * Math.cos(robotPose.heading);

        if (Math.abs(x_rotated) < 0.01) x_rotated = 0;
        else x_rotated += kStatic * Math.signum(x_rotated);
        if (Math.abs(y_rotated) < 0.01) y_rotated = 0;
        else y_rotated += kStatic * Math.signum(y_rotated);
        if (Math.abs(hPower) < 0.01) hPower = 0;
        else hPower += kStatic * Math.signum(hPower);

        x_rotated = Range.clip(x_rotated, -0.5, 0.5);
        y_rotated = Range.clip(y_rotated, -0.5, 0.5);

        // todo replace first 12 with voltage
        return new Pose((y_rotated / robot.getVoltage() * 12.5) / (Math.sqrt(2)/2), x_rotated / robot.getVoltage() * 12.5, hPower / robot.getVoltage() * 12.5);
    }

    @Override
    public void end (boolean interrupted){
        drivetrain.set(new Pose());
    }
}
