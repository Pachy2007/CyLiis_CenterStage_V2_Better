package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Config
public class    PropDetectionBlueFar implements VisionProcessor {

    public static boolean masked=false;
    public int detection = 2;

    public static int rightRectX1 = 30, rightRectY1 = 225;
    public static int rightRectX2 = 110, rightRectY2 = 350;

    public static double rightThresh = 250000;
    public double rightSum = 0;

    public static int middleRectX1 = 360, middleRectY1 = 200;
    public static int middleRectX2 = 450, middleRectY2 = 310;

    public static double middleThresh = 300000;
    public double middleSum = 0;

    public static int blueLowH = 110, blueLowS = 70, blueLowV = 0;
    public static int blueHighH = 120, blueHighS = 110, blueHighV = 255;

    Mat workingMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, workingMat, Imgproc.COLOR_RGB2HSV);

        Rect rightRect = new Rect(new Point(rightRectX1, rightRectY1), new Point(rightRectX2, rightRectY2));
        Rect middleRect = new Rect(new Point(middleRectX1, middleRectY1), new Point(middleRectX2, middleRectY2));

        Scalar lowThresh = new Scalar(blueLowH, blueLowS, blueLowV);
        Scalar highThresh = new Scalar(blueHighH, blueHighS, blueHighV);

        Core.inRange(workingMat, lowThresh, highThresh, workingMat);

        rightSum = Core.sumElems(workingMat.submat(rightRect)).val[0];
        middleSum = Core.sumElems(workingMat.submat(middleRect)).val[0];

        Imgproc.rectangle(frame, rightRect, new Scalar(0,255,0), 5);
        Imgproc.rectangle(frame, middleRect, new Scalar(0,255,0), 5);

        if(rightSum > rightThresh)
            detection = 3;
        else if (middleSum > middleThresh)
            detection = 2;
        else detection = 1;

        if(masked)
        workingMat.copyTo(frame);

        workingMat.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}