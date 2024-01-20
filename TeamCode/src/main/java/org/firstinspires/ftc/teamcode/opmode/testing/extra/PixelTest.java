package org.firstinspires.ftc.teamcode.opmode.testing.extra;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;
import org.firstinspires.ftc.teamcode.common.vision.PixelPipeline;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "PixelTest")
public class PixelTest extends LinearOpMode {

    private PixelPipeline pixelPipeline;
    private VisionPortal portal;

    private boolean previousState = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        pixelPipeline = new PixelPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(pixelPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (opModeInInit()) {
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            boolean currentState = gamepad1.a;
            if (currentState && !previousState) {
                // log
                PixelPipeline.ContourData data = pixelPipeline.getClosestContour();

                Logger.logData(LogType.CENTROID_X, String.valueOf(data.x));
                Logger.logData(LogType.CENTROID_Y, String.valueOf(data.y));
                Logger.logData(LogType.CONTOUR_AREA, String.valueOf(data.area));
                Logger.logData(LogType.CONTOUR_LENGTH, String.valueOf(data.length));
            }

            previousState = currentState;

            telemetry.update();
        }

        CSVInterface.log();
    }
}
