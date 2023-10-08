package org.firstinspires.ftc.teamcode.opmode.testing.extra;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

public class FunAlgo implements VisionProcessor {


    private Mat cameraMatrix = new Mat(3, 3, CvType.CV_32F);

    private final Mat intrinsics = new Mat(3, 3, CvType.CV_32F);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cameraMatrix.put(0, 0, 1, 1, 1);
        cameraMatrix.put(1, 0, 1, 1, 1);
        cameraMatrix.put(2, 2, 1);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        MatOfPoint2f pixelCords = new MatOfPoint2f(
                new Point(1, 1), // 2D pixel coordinates of the first tag
                new Point(1, 1),     // 2D pixel coordinates of the second tag
                new Point(1, 1)      // 2D pixel coordinates of the third tag
        );

        MatOfPoint3f poseCords = new MatOfPoint3f(
                new Point3(1, 1, 1), // 3D pose coordinates of the first tag
                new Point3(1, 1, 1), // 3D pose coordinates of the second tag
                new Point3(1, 1, 1) // 3D pose coordinates of the third tag
        );

        MatOfPoint3f arbitrary_3d_pose_value = new MatOfPoint3f(new Point3(1, 1, 1));

        MatOfDouble distCoeffs = new MatOfDouble(1, 1, 1, 1, 1);

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        Calib3d.solvePnP(poseCords, pixelCords, cameraMatrix, distCoeffs, rvec, tvec);

        MatOfPoint2f object2DLocation = new MatOfPoint2f();
        Calib3d.projectPoints(arbitrary_3d_pose_value, rvec, tvec, cameraMatrix, distCoeffs, object2DLocation);

        return object2DLocation;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
