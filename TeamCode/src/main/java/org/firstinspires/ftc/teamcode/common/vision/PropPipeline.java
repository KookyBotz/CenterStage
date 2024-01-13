package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class PropPipeline implements VisionProcessor {
    private static final boolean DEBUG = false;

    private volatile Side location = Side.RIGHT;

    private final Mat hsv = new Mat();

    public static int redLeftX = 765;
    public static int redLeftY = 500;

    public static int redCenterX = 1335;
    public static int redCenterY = 450;

    public static int blueLeftX = 200;
    public static int blueLeftY = 475;

    public static int blueCenterX = 925;
    public static int blueCenterY = 440;

    public static int leftWidth = 175;
    public static int leftHeight = 125;

    public static int centerWidth = 125;
    public static int centerHeight = 125;

    public static double threshold = 180;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);

    Telemetry telemetry;

    Side ALLIANCE = Side.RED;

    public PropPipeline() {
        this(null);
    }

    public PropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Rect leftZoneArea;
        Rect centerZoneArea;

        if (ALLIANCE == Side.RED) {
            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
        } else {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, centerWidth, centerHeight);
        }

        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);


        if (DEBUG) {
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
            Imgproc.blur(frame, frame, new Size(5, 5));
            Core.inRange(frame, new Scalar(0, 150, 150), new Scalar(180, 255, 255), frame);
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);
        }

        Imgproc.cvtColor(leftZone, leftZone, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(centerZone, centerZone, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));
        Core.inRange(leftZone, new Scalar(0, 150, 150), new Scalar(180, 255, 255), leftZone);
        Core.inRange(centerZone, new Scalar(0, 150, 150), new Scalar(180, 255, 255), centerZone);

        left = Core.mean(leftZone);
        center = Core.mean(centerZone);

        leftColor = left.val[0];
        centerColor = center.val[0];

        if (telemetry != null) {
            telemetry.addData("leftColor", left.toString());
            telemetry.addData("centerColor", center.toString());
            telemetry.addData("analysis", location.toString());
            telemetry.update();
        }

        if (leftColor > threshold) {
            // left zone has it
            location = Side.LEFT;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 10);
        } else if (centerColor > threshold) {
            // center zone has it
            location = Side.CENTER;
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 10);
        } else {
            // right zone has it
            location = Side.RIGHT;
        }

        leftZone.release();
        centerZone.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Side getLocation() {
        return this.location;
    }
}
