package org.firstinspires.ftc.teamcode.common.commandbase.drivecommand;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitConfig;
import org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

import static org.firstinspires.ftc.teamcode.common.drive.pathing.purepursuit.PurePursuitConfig.*;

@Config
public class PurePursuitCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final FusedLocalizer localizer;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;


    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime accelLimit;
    private final double ACCEL_LIMIT = 0.5;

    private ElapsedTime timer;

    public PurePursuitCommand(PurePursuitPath purePursuitPath) {
        this.drivetrain = robot.drivetrain;
        this.localizer = robot.localizer;
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
    }

    @Override
    public void execute() {
        if (accelLimit == null) accelLimit = new ElapsedTime();
        if (purePursuitPath.isFinished()) PID = true;

        Pose robotPose = localizer.getPose();
        Pose targetPose = purePursuitPath.update(robotPose);

        if (PID && timer == null) {
            timer = new ElapsedTime();
        }

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConfig.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConfig.ALLOWED_HEADING_ERROR) finished = true;


        if (targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = -hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        drivetrain.set(new Pose(x_rotated * X_GAIN, y_rotated, hPower).scale(Math.min(accelLimit.seconds() / ACCEL_LIMIT, 1)));


//        else {
//            Pose delta = targetPose.subtract(robotPose);
//            double y_rotated = delta.x * Math.cos(robotPose.heading) - delta.y * Math.sin(robotPose.heading);
//            double x_rotated = delta.x * Math.sin(robotPose.heading) + delta.y * Math.cos(robotPose.heading);
//
//            double xPercentage = x_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
//            double yPercentage = y_rotated / purePursuitPath.getRadius() * PurePursuitConfig.FOLLOW_SPEED;
//
//            if(targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
//            if(targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;
//            double hPower = -hController.calculate(robotPose.heading, targetPose.heading);
//
//            hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
//            xPercentage = Range.clip(xPercentage, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
//            yPercentage = Range.clip(yPercentage, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
//
//            drivetrain.set(new Pose(xPercentage * X_GAIN, yPercentage, hPower));
//        }
    }

    @Override
    public boolean isFinished() {
        return PID && finished || (timer != null && timer.milliseconds() > 2000);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}
