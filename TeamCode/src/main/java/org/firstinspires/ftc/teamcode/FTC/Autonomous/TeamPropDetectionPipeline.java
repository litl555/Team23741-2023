package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.BoardDetectionPipeline;
import org.opencv.android.Utils;
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

@Config
public class TeamPropDetectionPipeline extends OpenCvPipeline {
    private OpenCvCamera cam;
    private LoggerTool telemetry;
    private boolean isRed;
    private Mat dilateKernel;

    public static int blueY_lower = 5, blueCr_lower = 0, blueCb_lower = 140;
    public static int blueY_upper = 255, blueCr_upper = 125, blueCb_upper = 255;

    public static int redY_lower = 25, redCr_lower = 160, redCb_lower = 0;
    public static int redY_upper = 255, redCr_upper = 255, redCb_upper = 150;

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
                cam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry.add("STATUS", "Pipeline has initialized");
            }
            @Override public void onError(int errorCode) {
                telemetry.add("PIPELINE ERROR", errorCode);
            }
        });
    }

    @Override
    public Mat processFrame(Mat _input) {
        Mat input = new Mat();
        Imgproc.cvtColor(_input, input, Imgproc.COLOR_RGB2YCrCb);

        //if (isRed) Core.inRange(input, new Scalar(25, 160, 0), new Scalar(255, 255, 150), input);
        //else Core.inRange(input, new Scalar(5, 0, 150), new Scalar(255, 125, 255), input);

        if (isRed) Core.inRange(input, new Scalar(redY_lower, redCr_lower, redCb_lower), new Scalar(redY_upper, redCr_upper, redCb_upper), input);
        else Core.inRange(input, new Scalar(blueY_lower, blueCr_lower, blueCb_lower), new Scalar(blueY_upper, blueCr_upper, blueCb_upper), input);

        Imgproc.dilate(input, input, dilateKernel);

        ArrayList<MatOfPoint> cnts = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(input, cnts, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double biggestArea = 0;
        MatOfPoint bestCurve = null;
        int bestCntInd = -1;
        for (int i = 0; i < cnts.size(); i++) {
            MatOfPoint cnt = cnts.get(i);

            //if (curve.total() != 4) continue;

            double area = Imgproc.contourArea(cnt);
            if (area > biggestArea) {
                biggestArea = area;
                bestCurve = cnt;
                bestCntInd = i;
            }
        }

        if (bestCntInd != -1) {
            ArrayList<MatOfPoint> toDraw = new ArrayList<>();

            toDraw.add(cnts.get(bestCntInd));
            Imgproc.drawContours(_input, toDraw, -1, new Scalar(0, 0, 0), -1);

            telemetry.add("Detected prop", "true");
        } else telemetry.add("Detected prop", "false");

        telemetry.add("Biggest area", biggestArea);
        telemetry.add("Detecting", isRed ? "red" : "blue");

        int centerLine = 200;

        // so we dont detect lines
        if (biggestArea < 15_000) propPos = TeamPropPosition.left;
        else {
            Moments m = Imgproc.moments(bestCurve);
            Point p = new Point(m.m10 / (m.m00 + 1e-5), m.m01 / (m.m00 + 1e-5));

            Imgproc.circle(_input, p, 7, new Scalar(255, 255, 255), -1);

            if (p.y < centerLine) propPos = TeamPropPosition.middle;
            else propPos = TeamPropPosition.right;
            telemetry.add("y", p.y);
        }

        Imgproc.line(_input, new Point(0, centerLine), new Point(1280, centerLine), new Scalar(255, 255, 0));
        Imgproc.putText(_input, "middle", new Point(720, centerLine / 2), 1, 3, new Scalar(255, 255, 0));
        Imgproc.putText(_input, "right", new Point(720, centerLine + centerLine / 2), 1, 3, new Scalar(255, 255, 0));

        Imgproc.putText(_input, "Detected " + (isRed ? "red" : "blue") + " at " + propPos.toString(), new Point(50, 600), 1, 3, isRed ? new Scalar(255, 0, 0) : new Scalar(0, 0, 255), 7);

        Bitmap bitmap = Bitmap.createBitmap(_input.cols(), _input.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(_input, bitmap);
        FtcDashboard.getInstance().sendImage(bitmap);

        for (int i = 0; i < cnts.size(); i++) cnts.get(i).release();
        hierarchy.release();

        input.release();

        return _input;
    }

    public void destroy() {
        cam.stopStreaming();
    }
}
