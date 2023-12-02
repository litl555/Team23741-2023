package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.BoardDetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Map;

public class TeamPropDetectionPipeline extends OpenCvPipeline {
    private OpenCvCamera cam;
    private LoggerTool telemetry;
    private boolean isRed;
    private Mat dilateKernel;

    public TeamPropPosition propPos = TeamPropPosition.undefined;

    public TeamPropDetectionPipeline(OpenCvCamera cam, LoggerTool telemetry, boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;

        this.cam = cam;
        cam.setPipeline(this);

        dilateKernel = new Mat(6, 6, CvType.CV_32SC1);
        dilateKernel.put(0, 0,
                1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1); // lol

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                cam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.add("STATUS", "Pipeline has initialized");
            }
            @Override public void onError(int errorCode) {
                telemetry.add("PIPELINE ERROR", errorCode);
            }
        });
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat inputCopy = input.clone();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);

        if (isRed) Core.inRange(input, new Scalar(25, 160, 0), new Scalar(255, 255, 150), input);
        else Core.inRange(input, new Scalar(5, 0, 150), new Scalar(255, 125, 255), input);

        Imgproc.dilate(input, input, dilateKernel);

        ArrayList<MatOfPoint> cnts = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(input, cnts, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f curve = new MatOfPoint2f();
        double biggestArea = 0;
        MatOfPoint2f bestCurve = null;
        int bestCntInd = -1;
        for (int i = 0; i < cnts.size(); i++) {
            MatOfPoint cnt = cnts.get(i);
            cnt.convertTo(curve, CvType.CV_32FC2);

            double perimeter = Imgproc.arcLength(curve, true);
            Imgproc.approxPolyDP(curve, curve, perimeter * 0.04, true);

            //if (curve.total() != 4) continue;

            double area = Imgproc.contourArea(curve);
            if (area > biggestArea) {
                biggestArea = area;
                bestCurve = curve;
                bestCntInd = i;
            } else curve.release();
        }

        telemetry.add("Last run", System.currentTimeMillis());

        if (bestCntInd != -1) {
            ArrayList<MatOfPoint> toDraw = new ArrayList<>();

            toDraw.add(cnts.get(bestCntInd));
            Imgproc.drawContours(inputCopy, toDraw, -1, new Scalar(255, 0, 0));

            telemetry.add("Detected prop", "true");
        } else telemetry.add("Detected prop", "false");

        telemetry.add("Biggest area", biggestArea);

        if (biggestArea < 18 * 18) propPos = TeamPropPosition.left;
        else {
            Moments m = Imgproc.moments(bestCurve);
            Point p = new Point(m.m10 / (m.m00 + 1e-5), m.m01 / (m.m00 + 1e-5));

            if (p.x < 300) propPos = TeamPropPosition.middle;
            else propPos = TeamPropPosition.right;
            telemetry.add("x", p.x);
        }

        for (int i = 0; i < cnts.size(); i++) cnts.get(i).release();
        hierarchy.release();

        return input;
    }

    public void destroy() {
        cam.stopStreaming();
    }
}
