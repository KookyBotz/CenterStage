package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

@TeleOp(name = "IVKTest")
@Disabled
public class IVKTest extends OpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrivetrain drivetrain;
    private ExtensionSubsystem extension;
    private IntakeSubsystem intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Globals.IS_AUTO = false;

        robot.init(hardwareMap);
        drivetrain = new MecanumDrivetrain();
        extension = new ExtensionSubsystem();
        intake = new IntakeSubsystem();
        robot.addSubsystem(drivetrain, intake);

        robot.read();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        robot.read();
        extension.read();

        drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);

        // input
//        super.run();
        robot.periodic();

        telemetry.addData("armAngle", robot.armActuator.getPosition());
        telemetry.addData("extensionPosition", robot.extensionActuator.getPosition());

//        double targetAngle = (2 * Math.PI / 3) + (Math.PI - robot.pitchActuator.getPosition());
        double targetAngle = (robot.armActuator.getPosition() - ((2 * Math.PI) / 3));
//        telemetry.addData("targetAngle", targetAngle);
//        telemetry.addData("servo pos", MathUtils.clamp(MathUtils.map(targetAngle, 0, Math.PI / 2, 0.47, 0.96), 0.075, 0.96));
//        robot.intakePivotActuator.setTargetPosition(MathUtils.clamp(MathUtils.map(targetAngle, 0, Math.PI / 2, 0.47, 0.96), 0.075, 0.96));

        telemetry.update();

//        robot.write();
    }
}
