package org.firstinspires.ftc.teamcode.opmode.testing.extra;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.StackRelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;
import org.firstinspires.ftc.teamcode.common.vision.StackPipeline;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "PixelTest")
public class PixelTest extends LinearOpMode {

    private StackPipeline stackPipeline;
    private VisionPortal portal;


    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        CommandScheduler.getInstance().reset();

//        telemetry = FtcDashboard.getInstance().getTelemetry();

        stackPipeline = new StackPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(stackPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

//        GamepadEx g1 = new GamepadEx(gamepad1);
//        g1.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new StackRelocalizeCommand(stackPipeline, new Pose(0, 0, 0)))

        while (opModeInInit()) {
            telemetry.addData("CORRECTION: ", -stackPipeline.getStrafeCorrection());
            telemetry.addLine("TAPE POSE (" + stackPipeline.getClosestTapeContour().x + " " + stackPipeline.getClosestTapeContour().y);
            telemetry.addLine("PIXEL POSE (" + stackPipeline.getClosestPixelContour().x + " " + stackPipeline.getClosestPixelContour().y);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            System.out.println("CORRECTION: " + -stackPipeline.getStrafeCorrection());
            telemetry.addData("CORRECTION: ", -stackPipeline.getStrafeCorrection());
            telemetry.addLine("TAPE POSE (" + stackPipeline.getClosestTapeContour().x + " " + stackPipeline.getClosestTapeContour().y);
            telemetry.addLine("PIXEL POSE (" + stackPipeline.getClosestPixelContour().x + " " + stackPipeline.getClosestPixelContour().y);
            telemetry.update();
        }

        CSVInterface.log();
    }
}
