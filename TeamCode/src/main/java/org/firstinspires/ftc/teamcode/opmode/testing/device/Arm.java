package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

@TeleOp(name = "arm")
@Disabled
public class Arm extends OpMode {

    public WEncoder extensionEncoder;
    AnalogInput armPitchEnc;
    AbsoluteAnalogEncoder armPitchEncoder;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        armPitchEnc = hardwareMap.get(AnalogInput.class, "extensionPitchEncoder");
        armPitchEncoder = new AbsoluteAnalogEncoder(armPitchEnc);
        armPitchEncoder.zero(2.086);
        armPitchEncoder.setInverted(true);
        armPitchEncoder.setWraparound(true);
    }

    @Override
    public void loop() {
//        telemetry.addData("position", extensionPitchEncoder.getCurrentPosition());
        telemetry.addData("arm position", armPitchEncoder.getCurrentPosition());
        telemetry.addData("arm position", armPitchEncoder.getVoltage());
        telemetry.update();
    }

}
