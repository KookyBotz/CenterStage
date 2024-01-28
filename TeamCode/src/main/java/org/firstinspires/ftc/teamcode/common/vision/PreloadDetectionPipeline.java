package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class PreloadDetectionPipeline implements VisionProcessor {

    private int targetAprilTagID = 0;

    private Location preloadedZone = Location.CENTER;

//    private AprilTagProcessor aprilTag;
//
//    public PreloadDetectionPipeline(AprilTagProcessor aprilTag) {
//        this.aprilTag = aprilTag;
//    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

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

                        Rect leftInclusionZone = new Rect(tagCenterX - inclusionZoneWidth, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);
                        Rect rightInclusionZone = new Rect(tagCenterX, tagCenterY - 110, inclusionZoneWidth, inclusionZoneHeight);

                        Rect leftExclusionZone = new Rect(tagCenterX - (int) (inclusionZoneWidth * 0.64), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);
                        Rect rightExclusionZone = new Rect(tagCenterX + (int) (inclusionZoneWidth * 0.28), tagCenterY - 90, exclusionZoneWidth, exclusionZoneHeight);

                        Imgproc.rectangle(frame, leftInclusionZone, new Scalar(0, 255, 0), 7);
                        Imgproc.rectangle(frame, rightInclusionZone, new Scalar(0, 255, 0), 7);

                        int leftZoneAverage = meanColor(frame, leftInclusionZone, leftExclusionZone);
                        int rightZoneAverage = meanColor(frame, rightInclusionZone, rightExclusionZone);

//                        System.out.println("LEFTAVG " + leftZoneAverage);
//                        System.out.println("RIGHTAVG " + rightZoneAverage);

                        preloadedZone = (leftZoneAverage > rightZoneAverage) ? Location.LEFT : Location.RIGHT;
                        System.out.println("PRELOADED ZONE: " + preloadedZone);
                        Globals.PRELOAD = preloadedZone;
                    }
                }
            }
        }


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getPreloadedZone() {
        return this.preloadedZone;
    }

    public int getTargetAprilTagID() {
        return this.targetAprilTagID;
    }

    public void setTargetAprilTagID(Location preloadLocation) {
        targetAprilTagID = 0;
        switch (preloadLocation) {
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

    public int meanColor(Mat frame, Rect inclusionRect, Rect exclusionRect) {
        if (frame == null) {
            System.out.println("frame is bad");
            return 0;
        }

        int sum = 0;
        int count = 0;
        for (int y = inclusionRect.y; y < inclusionRect.y + inclusionRect.height; y++) {
            for (int x = inclusionRect.x; x < inclusionRect.x + inclusionRect.width; x++) {
                if (x < 0 || x >= frame.cols() || y < 0 || y >= frame.rows()) {
                    continue;
                }

                if (x >= exclusionRect.x && x < exclusionRect.x + exclusionRect.width && y >= exclusionRect.y && y < exclusionRect.y + exclusionRect.height) {
                    continue;
                }

                double[] data = frame.get(y, x);
                if (data != null && data.length > 0) {
                    sum += data[0];
                    count++;
                }
            }
        }

        return count > 0 ? sum / count : 0;
    }


}
