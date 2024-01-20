package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class PixelPipeline implements VisionProcessor {

    public static int TOPLEFT_X = 200;
    public static int TOPLEFT_Y = 450;

    public static int WIDTH = 1520;
    public static int HEIGHT = 630;

    ContourData closestContour = null;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        frame = frame.submat(new Rect(TOPLEFT_X, TOPLEFT_Y, WIDTH, HEIGHT));

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HLS);

        Mat mask = new Mat();
        Core.inRange(frame, new Scalar(0, 185, 0), new Scalar(179, 255, 255), mask);

        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(frame, frame, element);
        Imgproc.dilate(frame, frame, element);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat(); // test if i cna remove this
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = Double.MAX_VALUE;

        for (MatOfPoint contour : contours) {
            double length = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double area = Imgproc.contourArea(contour);

            if (length >= 100 && area >= 3000) {
                Moments M = Imgproc.moments(contour);

                int cX = (int) (M.get_m10() / M.get_m00());
                int cY = (int) (M.get_m01() / M.get_m00());


                System.out.println("POSE: (" + cX + ", " + cY + ") - AREA: (" + area + ") - LENGTH: (" + length + ")");

                double distance = Math.sqrt(Math.pow(cX - 760, 2) + Math.pow(cY - 315, 2));

                if (distance < minDistance) {
                    minDistance = distance;
                    closestContour = new ContourData(cX, cY, area, length);
                    Imgproc.drawContours(frame, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 3);
                    Imgproc.circle(frame, new Point(cX, cY), 7, new Scalar(255, 0, 0), -1);
                }
            }
        }

        mask.release();
        hierarchy.release();
        element.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public ContourData getClosestContour() {
        return closestContour;
    }

    public class ContourData {
        public int x;
        public int y;
        public double area;
        public double length;

        public ContourData(int x, int y, double area, double length) {
            this.x = x;
            this.y = y;
            this.area = area;
            this.length = length;
        }
    }
}
