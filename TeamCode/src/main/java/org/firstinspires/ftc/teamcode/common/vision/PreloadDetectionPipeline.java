package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;


import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class PreloadDetectionPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private int targetAprilTagID = 0;

    private Location preloadedZone = Location.CENTER;

//    private AprilTagProcessor aprilTag;
//
//    public PreloadDetectionPipeline(AprilTagProcessor aprilTag) {
//        this.aprilTag = aprilTag;
//    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        List<AprilTagDetection> currentDetections = RobotHardware.getInstance().getAprilTagDetections();
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == targetAprilTagID) {
                        int leftX = Integer.MAX_VALUE;
                        int rightX = Integer.MIN_VALUE;
                        int topY = Integer.MIN_VALUE;
                        int bottomY = Integer.MAX_VALUE;

                        for (Point point : detection.corners) {
                            if (point.x < leftX) leftX = (int) point.x;
                            if (point.x > rightX) rightX = (int) point.x;
                            if (point.y > topY) topY = (int) point.y;
                            if (point.y < bottomY) bottomY = (int) point.y;
                        }

                        int tagCenterX = (int) detection.center.x;
                        int tagCenterY = (int) detection.center.y;

                        int tagWidth = rightX - leftX;
                        int tagHeight = topY - bottomY;

                        int inclusionZoneWidth = (int) (tagWidth * 1.5);
                        int inclusionZoneHeight = (int) (tagHeight * 1.5);

                        int exclusionZoneWidth = (int) (tagWidth * 0.28);
                        int exclusionZoneHeight = (int) (tagHeight * 0.28);

                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 80, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 80, inclusionZoneWidth, inclusionZoneHeight);

                        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.64), tagCenterY - 60, exclusionZoneWidth, exclusionZoneHeight);
                        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.28), tagCenterY - 60, exclusionZoneWidth, exclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 7);
                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 7);

                        int leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
                        int rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);

                        preloadedZone = (leftZoneAverage > rightZoneAverage) ? Location.LEFT : Location.RIGHT;
                        System.out.println("PRELOADED ZONE: " + preloadedZone);
                        Globals.PRELOAD = preloadedZone;
                    }
                }
            }
        }

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public void setTargetAprilTagID(Location preloadLocation) {
        targetAprilTagID = 0;
        switch(preloadLocation) {
            case LEFT:
                targetAprilTagID = 1;
                break;
            case CENTER:
                targetAprilTagID = 2;
                break;
            case RIGHT:
                targetAprilTagID = 3;
                break;
            default:
                break;
        }

        if (Globals.ALLIANCE == Location.RED) targetAprilTagID += 3;
    }

    public Location getPreloadedZone() {
        return this.preloadedZone;
    }

    public int getTargetAprilTagID() {
        return this.targetAprilTagID;
    }

    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (!(x >= exclusionRect.x && x < exclusionRect.x + exclusionRect.width && y >= inclusionRect.y && y < inclusionRect.y + inclusionRect.height)) {
                    sum += frame.get(y, x)[0];
                    count++;
                }
            }
        }

        return sum / count;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
