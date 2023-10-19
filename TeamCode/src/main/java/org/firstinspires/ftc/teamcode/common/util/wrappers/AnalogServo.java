package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.outoftheboxrobotics.photoncore.hardware.servo.PhotonServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import javax.annotation.Nonnegative;

public class AnalogServo {
    private PhotonServo servo;
    private AbsoluteAnalogEncoder encoder;

    public AnalogServo(Servo servo, AnalogInput encoder) {
        this.servo = (PhotonServo) servo;
        this.encoder = new AbsoluteAnalogEncoder(encoder);
    }

    public AnalogServo(Servo servo) {
        this.servo = (PhotonServo) servo;
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
        if (hasEncoder()) {
            return encoder.getCurrentPosition();
        } else {
            return servo.getPosition();
        }
    }

    public boolean hasEncoder() {
        return encoder != null;
    }
}
