package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FTC.Commands.TVec;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Pixels.BoardDetectionPipeline;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.AprilTag;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class BoardTagLocalizationPipeline extends OpenCvPipeline {
    public static float decimation = 1.5f;
    private long detectorPtr;

    public BoardTagLocalizationPipeline(OpenCvCamera cam) {
        detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, decimation, 2);
        cam.setPipeline(this);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                //cam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                //Robot.telemetry.addImportant("STATUS", "Pipeline has initialized");
            }
            @Override public void onError(int errorCode) {
                Robot.telemetry.addImportant("PIPELINE ERROR", errorCode);
            }
        });
    }

    @Override
    public Mat processFrame(Mat _input) {
        Mat processedTag = new Mat();
        Imgproc.cvtColor(_input, processedTag, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.threshold(processedTag, processedTag, 160, 255, Imgproc.THRESH_BINARY);

        ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(detectorPtr, processedTag,
            BoardConstants.tagSize, 572.42030608, 571.44741121, 248.86563431, 347.77866451);

        processedTag.release();

        double px = 0;
        double py = 0;
        double count = 0;
        // for each april tag that we see, get their est position relative to robot
        for (AprilTagDetection detection : detections) {
            Pose p = getCVPose(detection.pose);

            // m to mm
            double x = get1D(p.tvec, 0) * 1000.0;
            double y = get1D(p.tvec, 1) * 1000.0;
            double z = get1D(p.tvec, 2) * 1000.0;

            int offsetId = detection.id - 3;
            TVec t = new TVec();
            t.updateTvec(new double[] {-y, z, x}, detection.id - 3);

            Robot.telemetry.addImportant(new LoggerData(offsetId + " TVEC","(" + x + ", " + y + ", " + z + ")", "CAMERA"));
            Robot.telemetry.addImportant(new LoggerData(offsetId + " WORLD", "(" + TVec.worldPos.getX() + ", " + TVec.worldPos.getY() + ")", "CAMERA"));
            //Robot.telemetry.addImportant("tvec xyz " + offsetId, "(" + x + ", " + y + ", " + z + ")");
            //Robot.telemetry.addImportant("world xyz " + offsetId, "(" + TVec.worldPos.getX() + ", " + TVec.worldPos.getY() + ")");

            px += TVec.worldPos.getX();
            py += TVec.worldPos.getY();
            count++;
        }

        if (count != 0 && Constants.robotPose.getX() > 900) Constants.robotPose = new Pose2d(Constants.robotPose.getX(), -px / count, Constants.robotPose.getHeading());

        Robot.telemetry.addImportant(new LoggerData("Last Run", System.currentTimeMillis(), "CAMERA"));

        //Bitmap bitmap = Bitmap.createBitmap(_input.cols(), _input.rows(), Bitmap.Config.RGB_565);
        //Utils.matToBitmap(_input, bitmap);
        //FtcDashboard.getInstance().sendImage(bitmap);

        return _input;
    }

    private void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        Scalar blue = new Scalar(7,197,235,255);
        Scalar red = new Scalar(255,0,0,255);
        Scalar green = new Scalar(0,255,0,255);
        Scalar white = new Scalar(255,255,255,255);

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
            new Point3(0,0,0),
            new Point3(length,0,0),
            new Point3(0,length,0),
            new Point3(0,0,-length) // TODO: WHY IS THIS NEGATIVE
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness); // X
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness); // Y
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness); // Z

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    private double get1D(Mat m, int i) { return m.get(i, 0)[0]; }

    private Pose getCVPose(AprilTagPose p) {
        Pose pose = new Pose();
        pose.tvec.put(0,0, p.x);
        pose.tvec.put(1,0, p.y);
        pose.tvec.put(2,0, p.z);

        Mat R = new Mat(3, 3, CvType.CV_32F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R.put(i,j, p.R.get(i,j));
            }
        }

        Calib3d.Rodrigues(R, pose.rvec);

        return pose;
    }

    @Override
    protected void finalize() {
        if (detectorPtr != 0) AprilTagDetectorJNI.releaseApriltagDetector(detectorPtr);
    }
}
