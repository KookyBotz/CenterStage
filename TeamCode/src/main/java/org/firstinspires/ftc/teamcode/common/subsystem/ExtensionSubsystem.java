package org.firstinspires.ftc.teamcode.common.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

@Config
public class ExtensionSubsystem extends WSubsystem {

    private final RobotHardware robot = RobotHardware.getInstance();
    private int backdropHeight = 0;
    public IntSupplier liftTicks;
    public DoubleSupplier armAngle;

    public double feedforward = 0.0;

    private double power = 0.0;
    private double maxPower = 0.0;
    private double count = 0.0;

    public static boolean LOGGING = false;
    private boolean written = false;

    public ExtensionSubsystem() {
        this.liftTicks = () -> robot.intSubscriber(Sensors.SensorType.EXTENSION_ENCODER);
        this.armAngle = () -> robot.doubleSubscriber(Sensors.SensorType.ARM_ENCODER);
    }

    @Override
    public void periodic() {
        double kG = 0.18; // TODO empirically tune this
        double dist_cog_cor_max = 10.161;
        double arm_reference_angle = robot.armActuator.getTargetPosition();

        double offset = -(robot.armActuator.getPosition() / Math.PI) * 50;
        double extension = (liftTicks.getAsInt() + offset) / 26.0;

        double m_1 = 1.07;
        double m_2 = 0.36;
        double m_3 = 0.14;
        double m_4 = 0.44;

        double length_r = 1.3;
        double length_last = 6.0;

        double dist_cog = (m_2 * (extension + 1) + m_3 * (2 * extension - 1) + m_4 * (2 * extension + 0.5 * length_last)) / (m_1 + m_2 + m_3 + m_4);
        double dist_cog_cor = Math.sqrt(Math.pow(dist_cog, 2) + Math.pow(length_r, 2));

        double dist_ratio = dist_cog_cor / dist_cog_cor_max;

        feedforward = kG * Math.cos(arm_reference_angle) * dist_ratio;

        robot.armActuator.updateFeedforward(feedforward);
        robot.extensionActuator.setOffset(-(robot.armActuator.getPosition() / Math.PI) * 50);

        robot.armActuator.setCurrentPosition(armAngle.getAsDouble());
        robot.extensionActuator.setCurrentPosition(liftTicks.getAsInt());

        double error = robot.extensionActuator.getOverallTargetPosition() - robot.extensionActuator.getPosition();
        double feedforward = 0.1 * Math.abs(Math.cos(robot.armActuator.getPosition())) * Math.signum(error);

        if (LOGGING) {

            if (!RobotHardware.getInstance().armActuator.hasReached()) {
                Logger.logData(LogType.ARM_POSITION, String.valueOf(RobotHardware.getInstance().armActuator.getPosition()));
                Logger.logData(LogType.ARM_POWER, String.valueOf(RobotHardware.getInstance().armActuator.getPower()));
                Logger.logData(LogType.ARM_TARGET_POSITION, String.valueOf(RobotHardware.getInstance().armActuator.getTargetPosition()));

                double curPower = RobotHardware.getInstance().armActuator.getPower();
                power += curPower;
                count += 1.0;
                if (Math.abs(curPower) > Math.abs(maxPower)) {
                    maxPower = curPower;
                }
            }

            if (RobotHardware.getInstance().armActuator.hasReached() && !written) {
                System.out.println("ARM POWER " + (power / count));
                Logger.logData(LogType.ARM_POWER_AVERAGE, String.valueOf(power / count));
                Logger.logData(LogType.ARM_MAX_POWER, String.valueOf(maxPower));
                CSVInterface.log();
                written = true;
            }
        }


//        robot.extensionActuator.updateFeedforward(Math.abs(error) > 10 ? feedforward : 0);

        robot.armActuator.periodic();
        robot.extensionActuator.periodic();
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {
        robot.armActuator.write();
        robot.extensionActuator.write();
    }

    @Override
    public void reset() {

    }

    public int getBackdropHeight() {
        return backdropHeight;
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(getBackdropHeight() + amount, 0, 11);
    }

    public void setBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtils.clamp(amount, 0, 11);
    }
}
