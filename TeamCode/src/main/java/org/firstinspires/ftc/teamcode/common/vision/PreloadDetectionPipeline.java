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

import java.util.List;

public class PreloadDetectionPipeline implements VisionProcessor {

    private int targetAprilTagID = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        List<AprilTagDetection> currentDetections = RobotHardware.getInstance().getAprilTagDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.id == targetAprilTagID) {
                    int leftX = Integer.MAX_VALUE;
                    int rightX = Integer.MIN_VALUE;
                    int topY = Integer.MIN_VALUE;
                    int bottomY = Integer.MAX_VALUE;

                    int tagCenterX = 0, tagCenterY = 0;

                    for (Point point : detection.corners) {
                        if (point.x < leftX) leftX = (int) point.x;
                        if (point.x > rightX) rightX = (int) point.x;
                        if (point.y > topY) topY = (int) point.y;
                        if (point.y < bottomY) bottomY = (int) point.y;

                        tagCenterX = (int) detection.center.x;
                        tagCenterY = (int) detection.center.y;
                    }

                    int tagWidth = rightX - leftX;
                    int tagHeight = topY - bottomY;

                    double exclusionAmount = 0.28; // % ignored in the middle
                    int exclusionWidth = (int) (tagWidth * exclusionAmount);
                    int exclusionHeight = (int) (tagHeight * exclusionAmount);

                    int exclusionTopLeftX = tagCenterX - exclusionWidth / 2;
                    int exclusionTopLeftY = tagCenterY - exclusionHeight / 2;

                    Rect inclusionRect = new Rect(tagCenterX - tagWidth / 2, tagCenterY - tagHeight / 2, tagWidth, tagHeight);
                    Rect exclusionRect = new Rect(exclusionTopLeftX, exclusionTopLeftY, exclusionWidth, exclusionHeight);

                    // From here, now that we have region size data, we can shift the regions n rows, (value can be grabbed
                    // from the simulation) and then call the method meanColor(frame, inclusionRect, exclusionRect)
                    // and thus determine if the mean value falls within our tolerance, of knowing if there is a pixel or not
                }
            }
        }

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
}
