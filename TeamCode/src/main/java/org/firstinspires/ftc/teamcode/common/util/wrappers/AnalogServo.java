package org.firstinspires.ftc.teamcode.common.util.wrappers;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonLynxServoController;
import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import javax.annotation.Nonnegative;

public class AnalogServo {
    private PhotonServo servo;
    private AbsoluteAnalogEncoder encoder;

    public AnalogServo(Servo servo, AnalogInput encoder) {
        this.servo = (PhotonServo) servo;
        this.encoder = new AbsoluteAnalogEncoder(encoder);
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void invert() {
        encoder.setInverted(true);
        servo.setDirection(Servo.Direction.REVERSE);
    }

    @Nonnegative
    public double getPosition() {
        return encoder.getCurrentPosition();
    }
}
