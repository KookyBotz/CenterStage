package org.firstinspires.ftc.teamcode.opmode.testing.extra;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.cycleautocommand.StackRelocalizeCommand;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;
import org.firstinspires.ftc.teamcode.common.vision.StackPipeline;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

@Config
@Autonomous(name = "PixelTest")
@Disabled
public class PixelTest extends LinearOpMode {

    private StackPipeline stackPipeline;
    private VisionPortal portal;

    private boolean previousState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        CommandScheduler.getInstance().reset();


        stackPipeline = new StackPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(stackPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (opModeInInit()) {
            StackPipeline.ContourData data = stackPipeline.getClosestTapeContour();

            telemetry.addData("CENTX", data.x);
            telemetry.addData("CENTY", data.y);
            telemetry.addData("AREA", data.area);
            telemetry.addData("LENGTH", data.length);
            telemetry.addData("CORRECTION", stackPipeline.getStrafeCorrection());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            StackPipeline.ContourData data = stackPipeline.getClosestTapeContour();
            telemetry.addData("CENTX", data.x);
            telemetry.addData("CENTY", data.y);
            telemetry.addData("AREA", data.area);
            telemetry.addData("LENGTH", data.length);
            telemetry.update();

            boolean currentState = gamepad1.a;
            if (currentState && !previousState) {
                // log
//                StackPipeline.ContourData data = stackPipeline.getClosestPixelContour();
//
//                Logger.logData(LogType.CENTROID_X, String.valueOf(data.x));
//                Logger.logData(LogType.CENTROID_Y, String.valueOf(data.y));
//                Logger.logData(LogType.CONTOUR_AREA, String.valueOf(data.area));
//                Logger.logData(LogType.CONTOUR_LENGTH, String.valueOf(data.length));

//                data = stackPipeline.getClosestTapeContour();
                telemetry.addData("CENTX", data.x);
                telemetry.addData("CENTY", data.y);
                telemetry.addData("AREA", data.area);
                telemetry.addData("LENGTH", data.length);
                telemetry.addData("CORRECTION", stackPipeline.getStrafeCorrection());
                telemetry.update();

//                Logger.logData(LogType.TAPE_CENTROID_X, String.valueOf(data.x));
//                Logger.logData(LogType.TAPE_CENTROID_Y, String.valueOf(data.y));
//                Logger.logData(LogType.TAPE_CONTOUR_AREA, String.valueOf(data.area));
//                Logger.logData(LogType.TAPE_CONTOUR_LENGTH, String.valueOf(data.length));
            }

            previousState = currentState;
        }

        CSVInterface.log();
    }
}
