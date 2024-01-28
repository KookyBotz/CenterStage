package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
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
import java.util.concurrent.atomic.AtomicReference;

public class StackPipeline implements VisionProcessor, CameraStreamSource {

    public static int TOPLEFT_X = 600;
    public static int TOPLEFT_Y = 450;
    public static int WIDTH = 1120;
    public static int HEIGHT = 630;
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

//    public volatile double correctionAmt = 0.0;
    ContourData closestPixelContour = new ContourData(0, 0, 0, 0);
    ContourData closestTapeContour = new ContourData(0, 0, 0, 0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Mat frame2 = frame.submat(new Rect(TOPLEFT_X, TOPLEFT_Y, WIDTH, HEIGHT));

        Imgproc.cvtColor(frame2, frame2, Imgproc.COLOR_BGR2HLS);

        Mat mask = new Mat();
        Core.inRange(frame2, new Scalar(0, 200, 0), new Scalar(179, 255, 255), mask);

        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.erode(mask, mask, element);
        Imgproc.dilate(mask, mask, element);

        Mat pixel_mask = mask.submat(new Rect(0, 0, WIDTH, 240));
        Mat tape_mask = mask.submat(new Rect(0, 240, WIDTH, 390));

        List<MatOfPoint> pixelContours = new ArrayList<>();
        List<MatOfPoint> tapeContours = new ArrayList<>();
        Mat hierarchy = new Mat(); // test if i cna remove this
        Imgproc.findContours(pixel_mask, pixelContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(tape_mask, tapeContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Go through all pixel contours
        double minDistance = Double.MAX_VALUE;
        for (MatOfPoint contour : pixelContours) {
            double length = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double area = Imgproc.contourArea(contour);

            if (length >= 100 && area >= 3000) {
                Moments M = Imgproc.moments(contour);

                int cX = (int) (M.get_m10() / M.get_m00());
                int cY = (int) (M.get_m01() / M.get_m00());

                double distance = Math.sqrt(Math.pow(cX - 976, 2) + Math.pow(cY - 138, 2));

                if (distance < minDistance) {
                    minDistance = distance;
                    closestPixelContour = new ContourData(cX, cY, area, length);
                }
            }
        }

        Imgproc.circle(frame2, new Point(closestPixelContour.x, closestPixelContour.y), 7, new Scalar(0, 0, 255), -1);


        // Go through all tape contours
        minDistance = Double.MAX_VALUE;
        for (MatOfPoint contour : tapeContours) {
            double length = Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true);
            double area = Imgproc.contourArea(contour);

            if (length >= 70 && area >= 2500) {
                Moments M = Imgproc.moments(contour);

                int cX = (int) (M.get_m10() / M.get_m00());
                int cY = (int) (M.get_m01() / M.get_m00());

                double distance = Math.sqrt(Math.pow(cX - 976, 2) + Math.pow(cY - 138, 2));

//                System.out.println("TAPE POSE: (" + cX + ", " + cY + ")");

                if (distance < minDistance) {
                    minDistance = distance;
                    closestTapeContour = new ContourData(cX, cY, area, length);
                }
            }
        }

        Imgproc.circle(frame2, new Point(closestTapeContour.x, closestTapeContour.y), 7, new Scalar(0, 0, 255), -1);
//        frame2.copyTo(frame);
//        frame2.copyTo(frame);

//        System.out.println("TAPE POSE: (" + closestTapeContour.x + ", " + closestTapeContour.y + ")");

        mask.release();
        hierarchy.release();
        element.release();
        frame2.release();

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public ContourData getClosestPixelContour() {
        return closestPixelContour;
    }

    public ContourData getClosestTapeContour() {
        return closestTapeContour;
    }

    public double getStrafeCorrection() {
//        correctionAmt = -0.0120*closestTapeContour.x + 12.42;
        return Range.clip(-0.0120 * (closestTapeContour.x + 400) + 12.26, -3, 3);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
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
