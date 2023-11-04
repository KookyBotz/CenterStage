package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

@Config
@TeleOp(name = "ReadingsTest2")
//@Photon(maximumParallelCommands = 8)
@Disabled
public class ReadingsTest extends OpMode {

    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public AnalogInput extensionPitchEnc;

//    private ElapsedTime timer;
    private double loopTime = 0.0;

//    private final RobotHardware robot = RobotHardware.getInstance();
//    private ExtensionSubsystem extension;

//    @Override
//    public void initialize() {
//        CommandScheduler.getInstance().reset();
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
//        Globals.IS_AUTO = false;
//        Globals.IS_USING_IMU = true;
//
//        robot.init(hardwareMap, telemetry);
////        extension = new ExtensionSubsystem();
////        robot.addSubsystem(extension);
//    }
//
//    @Override
//    public void run() {
//        super.run();
////        robot.read();
////        super.run();
////        robot.write();
//
//
//        telemetry.addData("degree reading", Math.toDegrees(robot.extensionPitchEncoder.getCurrentPosition()));
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        loopTime = loop;
//        telemetry.update();
//
////        robot.clearBulkCache();
//    }

    @Override
    public void init() {

//        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
//        Globals.IS_AUTO = false;
//        Globals.IS_USING_IMU = true;

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setWraparound(true);

//        robot.init(hardwareMap, telemetry);
        telemetry.addLine("here");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("radian reading", extensionPitchEncoder.getCurrentPosition());
//        telemetry.addData("extension reading", )

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
    }
}
