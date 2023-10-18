package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;

@TeleOp(name = "ReadingsTest")
@Photon(maximumParallelCommands = 8)
public class ReadingsTest extends CommandOpMode {

    private ElapsedTime timer;
    private double loopTime = 0.0;

    private final RobotHardware robot = RobotHardware.getInstance();
    private ExtensionSubsystem extension;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;
        Globals.IS_USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        extension = new ExtensionSubsystem();
        robot.addSubsystem(extension);
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.write();


        telemetry.addData("radian reading", Math.toDegrees(robot.extensionPitchActuator.getPosition()));

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }
}
