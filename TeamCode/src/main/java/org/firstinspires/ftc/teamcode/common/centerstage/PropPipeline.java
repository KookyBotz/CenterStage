package org.firstinspires.ftc.teamcode.common.centerstage;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Side location = Side.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    private Rect leftZoneArea;
    private Rect centerZoneArea;

    private Mat finalMat = new Mat();

    public static int leftX = 775;
    public static int leftY = 525;

    public static int centerX = 1150;
    public static int centerY = 150;

    public static int width = 175;
    public static int height = 175;

    int targetIndex = 0;

    public static int redThreshold = 200;
    public static int blueThreshold = 200;
    public static int threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        if (Globals.COLOR == Side.BLUE) {
            targetIndex = 0;
            threshold = redThreshold;
        } else {
            targetIndex = 2;
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (Globals.COLOR == Side.RED) {
            targetIndex = 0;
            threshold = redThreshold;
        } else {
            targetIndex = 2;
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);

        leftZoneArea = new Rect(leftX, leftY, width, height);
        centerZoneArea = new Rect(centerX, centerY, width, height);

        Mat leftZone = finalMat.submat(leftZoneArea);
        Mat centerZone = finalMat.submat(centerZoneArea);

        Scalar sumLeft = Core.sumElems(leftZone);
        Scalar sumCenter = Core.sumElems(centerZone);

        leftColor = sumLeft.val[targetIndex];
        centerColor = sumCenter.val[targetIndex];

        if (leftColor > threshold) {
            // left zone has it
            location = Side.LEFT;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
        } else if (centerColor > threshold) {
            // center zone has it
            location = Side.CENTER;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
        } else {
            // right zone has it
            location = Side.RIGHT;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
        }

        Imgproc.rectangle(finalMat, leftZoneArea, new Scalar(255, 255, 255));
        Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

        Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(finalMat, b);
        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Side getLocation() {
        return this.location;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
