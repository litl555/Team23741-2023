package org.firstinspires.ftc.teamcode.FTC.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerData;
import org.firstinspires.ftc.teamcode.FTC.Pixels.BoardDetectionPipeline;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.AprilTag;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
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
    private long detectorPtr;

    public BoardTagLocalizationPipeline(OpenCvCamera cam) {
        detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 1.5f, 2);
        cam.setPipeline(this);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                cam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                Robot.telemetry.addImportant("STATUS", "Pipeline has initialized");
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
            BoardConstants.tagSize,
            CameraIntrinsics.fx, CameraIntrinsics.fy, CameraIntrinsics.cx, CameraIntrinsics.cy);

        processedTag.release();

        // for each april tag that we see, get their est position relative to robot
        for (AprilTagDetection detection : detections) {
            if (detection.id != BoardConstants.tagID.red.center) continue;;
            Pose p = getCVPose(detection.pose);

            // m to mm
            double x = get1D(p.tvec, 0) * 1000.0;
            double y = get1D(p.tvec, 1) * 1000.0;
            double z = get1D(p.tvec, 2) * 1000.0;

            Robot.telemetry.addImportant("xyz", "(" + (int) x + ", " + (int) y + ", " + (int) z + ")");

            x += 36 * 25.4;
            z += 60.0 * 25.4;

            Robot.telemetry.addImportant("centered xz", "(" + (int) x + ", " + (int) z + ")");
        }

        return _input;
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
