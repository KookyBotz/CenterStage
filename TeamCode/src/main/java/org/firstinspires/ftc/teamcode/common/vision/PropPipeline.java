package org.firstinspires.ftc.teamcode.common.vision;

import static org.firstinspires.ftc.teamcode.common.hardware.Globals.ALLIANCE;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class PropPipeline implements VisionProcessor {
    private volatile Side location = Side.RIGHT;

    private Rect leftZoneArea;
    private Rect centerZoneArea;

    private Mat hsv = new Mat();

    public static int redLeftX = 765;
    public static int redLeftY = 575;

    public static int redCenterX = 1310;
    public static int redCenterY = 525;

    public static int blueLeftX = 200;
    public static int blueLeftY = 550;

    public static int blueCenterX = 900;
    public static int blueCenterY = 515;

    public static int width = 175;
    public static int height = 50;

    public static double redThreshold = 1;
    public static double blueThreshold = 0.6;
    public static double threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    Telemetry telemetry;

    public PropPipeline(){
        this(null);
    }

    public PropPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (ALLIANCE == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (ALLIANCE == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }

        leftZoneArea = new Rect(ALLIANCE == Side.RED? redLeftX : blueLeftX, ALLIANCE == Side.RED? redLeftY : blueLeftY, width, height);
        centerZoneArea = new Rect(ALLIANCE == Side.RED?redCenterX:blueCenterX, ALLIANCE == Side.RED?redCenterY:blueCenterY, width, height);

        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);

        Imgproc.cvtColor(leftZone, leftZone, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(centerZone, centerZone, Imgproc.COLOR_RGB2HSV);

        left = Core.sumElems(leftZone);
        center = Core.sumElems(centerZone);

        leftColor = left.val[0] / 1000000.0;
        centerColor = center.val[0] / 1000000.0;

        if(telemetry != null){
            telemetry.addData("leftColor", leftColor);
            telemetry.addData("centerColor", centerColor);
            telemetry.update();
        }

        Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);

        if(ALLIANCE == Side.BLUE){
            if (leftColor > threshold) {
                // left zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(0, 0, 255), 10);
            } else if (centerColor > threshold) {
                // center zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, centerZoneArea, new Scalar(0, 0, 255), 10);
            } else {
                // right zone has it
                location = Side.RIGHT;
            }
        }else{
            if (leftColor > threshold) {
                // left zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 0, 0), 10);
            } else if (centerColor > threshold) {
                // center zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 0, 0), 10);
            } else {
                // right zone has it
                location = Side.RIGHT;
            }
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
