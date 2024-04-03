package org.firstinspires.ftc.teamcode.opmode.testing.extra;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "PipelineTest")
@Disabled
public class PipelineTest extends LinearOpMode {

    private PropPipeline propPipeline;
    private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1920, 1080))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (opModeInInit()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.addData("leftZone", propPipeline.left.toString());
            telemetry.addData("centerZone", propPipeline.center.toString());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Location", propPipeline.getLocation());
            telemetry.addData("leftZone", propPipeline.left.toString());
            telemetry.addData("centerZone", propPipeline.center.toString());
            telemetry.update();
        }
    }
}
