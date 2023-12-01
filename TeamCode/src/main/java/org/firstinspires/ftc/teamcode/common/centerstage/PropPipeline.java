package org.firstinspires.ftc.teamcode.common.centerstage;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class PropPipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Side location = Side.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    private Rect leftZoneArea;
    private Rect centerZoneArea;

    private Mat finalMat = new Mat();

    public static int blueLeftX = 800;
    public static int blueLeftY = 550;

    public static int blueCenterX = 1175;
    public static int blueCenterY = 175;

    public static int redLeftX = 900;
    public static int redLeftY = 525;

    public static int redCenterX = 1325;
    public static int redCenterY = 100;

    public static int width = 125;
    public static int height = 125;

    public static double redThreshold = 2.5;
    public static double blueThreshold = 0.2;
    public static double threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        if (Globals.COLOR == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (Globals.COLOR == Side.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);
        Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

        leftZoneArea = new Rect(Globals.COLOR == Side.RED? redLeftX : blueLeftX, Globals.COLOR == Side.RED? redLeftY : blueLeftY, width, height);
        centerZoneArea = new Rect(Globals.COLOR == Side.RED?redCenterX:blueCenterX, Globals.COLOR == Side.RED?redCenterY:blueCenterY, width, height);

        Mat leftZone = finalMat.submat(leftZoneArea);
        Mat centerZone = finalMat.submat(centerZoneArea);

        left = Core.sumElems(leftZone);
        center = Core.sumElems(centerZone);

        leftColor = left.val[0] / 1000000.0;
        centerColor = center.val[0] / 1000000.0;

        if(Globals.COLOR == Side.BLUE){
            if (leftColor < threshold) {
                // left zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor < threshold) {
                // center zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it
                location = Side.RIGHT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }else{
            if (leftColor > threshold) {
                // left zone has it
                location = Side.CENTER;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor > threshold) {
                // center zone has it
                location = Side.RIGHT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it
                location = Side.LEFT;
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }

        Imgproc.rectangle(finalMat, leftZoneArea, new Scalar(255, 255, 255));
        Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

        Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(finalMat, b);
        lastFrame.set(b);

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

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
