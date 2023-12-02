package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;


@TeleOp(name = "arm")
public class Arm extends OpMode {
    public AbsoluteAnalogEncoder extensionPitchEncoder;
    public AnalogInput extensionPitchEnc;

    public WEncoder extensionEncoder;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        extensionEncoder = new WEncoder(new MotorEx(hardwareMap, "dtFrontLeftMotor").encoder);

        this.extensionPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        this.extensionPitchEncoder = new AbsoluteAnalogEncoder(extensionPitchEnc);
        extensionPitchEncoder.zero(2.086);
        extensionPitchEncoder.setInverted(true);
        extensionPitchEncoder.setWraparound(true);
    }

    @Override
    public void loop() {
        telemetry.addData("position", extensionPitchEncoder.getCurrentPosition());
        telemetry.addData("position 2", extensionEncoder.getPosition());
        telemetry.update();
    }

}
