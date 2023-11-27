package org.firstinspires.ftc.teamcode.FTC.Pixels;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.CameraIntrinsics;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.BoardConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Constants.PixelConstants;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.AprilTag;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Hex;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.PixelColor;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.AprilTagPose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Map;

public class BoardDetectionPipeline extends OpenCvPipeline {
    private static final double RVEC_ERROR_THRESHOLD = 0.05;
    private long detectorPtr;
    private Mat camMatrix, sharpenKernel, hexKernel;
    private MatOfDouble distCoeff;
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private LoggerTool telemetry;
    private pipelineStatus status = pipelineStatus.initializing;
    private boolean hasDetectedTags, hasDetectedBoard, shouldGetBoard;
    private MatOfPoint3f world = new MatOfPoint3f();
    private Map<Hex, PixelColor> detectedBoard = new Hashtable<>();
    private AprilTag detectedTag;
    private OpenCvCamera cam;

    public double brightness = 0;

    public boolean hasReadTags() { return hasDetectedTags; }
    public boolean hasReadBoard() { return hasDetectedBoard; }
    public boolean hasInitialized() { return status != pipelineStatus.initializing; }
    public AprilTag getTag() { return detectedTag; }
    public Map<Hex, PixelColor> getBoard() { return detectedBoard; }
    public boolean canRequestDetection() { return status == pipelineStatus.idle; }
    public boolean requestDetection(boolean shouldGetBoard) { // returns if it succeed in starting or not
        if (status != pipelineStatus.idle) {
            telemetry.add("ERROR", "Pipeline is not available to run.");
            telemetry.update();
            return false;
        }

        status = pipelineStatus.toStart;
        this.shouldGetBoard = shouldGetBoard;
        return true;
    }

    public BoardDetectionPipeline(OpenCvCamera cam, LoggerTool telemetry) {
        /*
         * we dont have access to HardwareMap (doesnt extend LinearOpMode) so you have to pass that in
         * everything else is done here though
         * OpenCvCamera cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "outtake_camera"));
         * BoardDetectionPipeline pipeline = new BoardDetectionPipeline(cam);
         */
        this.cam = cam;
        cam.setPipeline(this);

        // TODO: adjust decimation based upon distance to board
        detectorPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 1.5f, 3);

        camMatrix = new Mat(3, 3, CvType.CV_32FC1);
        camMatrix.put(0, 0,
                CameraIntrinsics.fx, 0, CameraIntrinsics.cx,
                0, CameraIntrinsics.fy, CameraIntrinsics.cy,
                0, 0, 1);

        distCoeff = new MatOfDouble();
        distCoeff.fromArray(CameraIntrinsics.dist);

        this.telemetry = telemetry;

        // pre generate board from offset coordinates
        ArrayList<Point3> worldBuffer = new ArrayList<>();
        for (int row = 0; row < 11; row++) {
            for (int col = 0; col < (row % 2 == 0 ? 6 : 7); col++) {
                Hex h = new Hex(col, row);
                detectedBoard.put(h, PixelColor.empty);

                worldBuffer.add(h.irl);
            }
        }

        world = new MatOfPoint3f();
        world.fromList(worldBuffer); // java opencv

        sharpenKernel = new Mat(3, 3, CvType.CV_32SC1);
        sharpenKernel.put(0, 0,
                0, -1, 0,
                -1, 5, -1,
                0, -1, 0);

        hexKernel = new Mat(3, 3, CvType.CV_32SC1);
        hexKernel.put(0, 0,
                1, 1, 1,
                1, 1, 1,
                1, 1, 1);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                cam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                status = BoardDetectionPipeline.pipelineStatus.idle;
                telemetry.add("STATUS", "Pipeline has initialized");
            }
            @Override public void onError(int errorCode) {
                telemetry.add("PIPELINE ERROR", errorCode);
            }
        });
    }

    @Override
    public Mat processFrame(Mat input) {
        //status = pipelineStatus.toStart; // testing
        if (status == pipelineStatus.idle || status == pipelineStatus.initializing) return input;
        if (status == pipelineStatus.running) {
            telemetry.add("ERROR", "Pipeline is already running. Aborting.");
            return input;
        }
        assert status == pipelineStatus.toStart;
        status = pipelineStatus.running;
        hasDetectedBoard = false;
        hasDetectedTags = false;

        for (Hex h : detectedBoard.keySet()) {
            detectedBoard.put(h, PixelColor.empty);
        }

        telemetry.add("STATUS", "Processing new frame");
        telemetry.add("LAST RUN", System.currentTimeMillis());

        // testing
        input.convertTo(input, -1, 1, brightness);

        // possible optimization: cull mat by half and only look at bottom half
        // maybe pre allocate these?
        Mat processedTag = new Mat();
        Imgproc.cvtColor(input, processedTag, Imgproc.COLOR_RGBA2GRAY);
        Imgproc.threshold(processedTag, processedTag, 160, 255, Imgproc.THRESH_BINARY);

        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(detectorPtr, processedTag,
                BoardConstants.tagSize,
                CameraIntrinsics.fx, CameraIntrinsics.fy, CameraIntrinsics.cx, CameraIntrinsics.cy);

        processedTag.release();

        if (detections.size() == 0) {
            telemetry.add("STATUS", "No tags detected");
            Bitmap bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(input, bitmap);
            FtcDashboard.getInstance().sendImage(bitmap);
            status = pipelineStatus.idle;
            return input;
        }

        detectedTag = detectAprilTags(detections);
        hasDetectedTags = true;

        //shouldGetBoard = true; // testing
        if (!shouldGetBoard) {
            status = pipelineStatus.idle;
            Bitmap bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(input, bitmap);
            FtcDashboard.getInstance().sendImage(bitmap);
            return input;
        }

        // now get where the pixels are
        Mat processedHex = new Mat();

        Imgproc.filter2D(input, processedHex, -1, sharpenKernel);
        Imgproc.cvtColor(processedHex, processedHex, Imgproc.COLOR_RGB2HSV);

        Map<PixelColor, ArrayList<Point>> hexCenters = new Hashtable<>();
        Mat[] masks = new Mat[4];
        for (int i = 0; i < 4; i++) masks[i] = new Mat(input.rows(), input.cols(), CvType.CV_8U, new Scalar(0));
        hexCenters.put(PixelColor.white, getHexCenters(BoardConstants.hsv.white.upper, BoardConstants.hsv.white.lower, processedHex, masks[0]));
        hexCenters.put(PixelColor.yellow, getHexCenters(BoardConstants.hsv.yellow.upper, BoardConstants.hsv.yellow.lower, processedHex, masks[1]));
        hexCenters.put(PixelColor.green, getHexCenters(BoardConstants.hsv.green.upper, BoardConstants.hsv.green.lower, processedHex, masks[2]));
        hexCenters.put(PixelColor.purple, getHexCenters(BoardConstants.hsv.purple.upper, BoardConstants.hsv.purple.lower, processedHex, masks[3]));
        PixelColor[] maskOverlapKey = new PixelColor[] {PixelColor.white, PixelColor.yellow, PixelColor.green, PixelColor.purple};

        processedHex.release();

        // iterate over every expected center and pull each point to the nearest one. start from the bottom
        // TODO: do we need distCoeff?
        MatOfPoint2f screen = new MatOfPoint2f();
        MatOfDouble _distCoeff = new MatOfDouble();
        Calib3d.projectPoints(world, detectedTag.pose.rvec, detectedTag.pose.tvec, camMatrix, _distCoeff, screen); // assumes that we have center tag
        Point[] screenPoints = screen.toArray();

        screen.release();
        _distCoeff.release();

        // world is saved in this order (bottom to top)
        int index = 0;
        for (int row = 0; row < 11; row++) {
            for (int col = 0; col < (row % 2 == 0 ? 6 : 7); col++) {
                if (index == screenPoints.length) continue;

                // currently, camera distortion is such that all the points are below where they should be
                // which means that they will always on a pixel, just maybe not the right one
                // hence use mask to filter out- if they are not on mask, dont bother checking them
                Point p = screenPoints[index];

                int maskOverlapId = -1; // 0 = white, 1 = yellow, 2 = green, 3 = purple
                for (int i = 0; i < 4; i++) {
                    double[] pBuffer = masks[i].get((int) p.y, (int) p.x);
                    if (pBuffer == null || pBuffer[0] == 0) continue;

                    maskOverlapId = i;
                    break;
                }

                // -1 is default value, != -1 means it must have been set- something must overlap
                if (maskOverlapId != -1) {
                    // loop through every hexCenter and find the closest
                    PixelColor closestColor = PixelColor.empty;
                    double bestDist = 1_000_000;
                    Point truePos = new Point();
                    int pixelIndex = -1;

                    for (PixelColor color : hexCenters.keySet()) {
                        ArrayList<Point> centers = hexCenters.get(color);
                        for (int i = 0; i < centers.size(); i++) {
                            Point center = centers.get(i);
                            double dist = Math.sqrt((p.y - center.y) * (p.y - center.y) + (p.x - center.x) * (p.x - center.x));

                            if (dist < bestDist) {
                                bestDist = dist;
                                closestColor = color;
                                pixelIndex = i;
                                truePos = center;
                            }
                        }
                    }

                    if (closestColor == PixelColor.empty) {
                        // sometimes we will fail to detect a hexagon, but the mask will usually still contain it
                        // use the mask then to try and figure out what the pixel is- this becomes more unreliable the higher up a pixel is
                        // (due to project points distorting)
                        //closestColor = maskOverlapKey[maskOverlapId];

                        telemetry.add("WARNING", "More points in mask than actually detected, most likely failed to detect some pixels");
                        index++;
                        continue;
                    } else {
                        // remove the closest center from being considered again
                        hexCenters.get(closestColor).remove(pixelIndex);
                    }

                    Scalar c = PixelConstants.colorToRgb.get(closestColor);
                    //Imgproc.circle(input, truePos, 8, c, -1);

                    detectedBoard.put(new Hex(col, row), closestColor);
                } else {
                    //Imgproc.circle(input, screenPoints[index], 4, new Scalar(255, 0, 0), -1);
                }

                index++;
            }
        }

        for (int i = 0; i < 4; i++) masks[i].release();

        // testing
        for (AprilTagDetection d : detections) {
            Pose p = getCVPose(d.pose);
            //drawAxisMarker(input, 0.05, 3, p.rvec, p.tvec, camMatrix);
        }

        hasDetectedBoard = true;

        status = pipelineStatus.idle;
        telemetry.add("STATUS", "Finished full detection");

        Bitmap bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(input, bitmap);
        FtcDashboard.getInstance().sendImage(bitmap);

        return input;
    }

    private ArrayList<Point> getHexCenters(Scalar upper, Scalar lower, Mat sharp, Mat outMask) {
        // note that this alg works better from far away (reduces distortion due to angle)
        // now for each of these segments get their contour
        // remove noise, then close holes
        Mat segment = new Mat();
        Core.inRange(sharp, lower, upper, segment);
        // erode to remove noise, dilate to try and force hexagons to join together
        // be very aggressive- we want to remove as much noise as possible
        //Imgproc.erode(segment, segment, hexKernel);
        //Imgproc.dilate(segment, segment, hexKernel);
        //Imgproc.dilate(segment, segment, hexKernel);
        Imgproc.morphologyEx(segment, segment, Imgproc.MORPH_OPEN, hexKernel);
        Imgproc.morphologyEx(segment, segment, Imgproc.MORPH_CLOSE, hexKernel);

        ArrayList<MatOfPoint> cnts = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(segment, cnts, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        // now go over each contour. if the area is too small, remove them
        MatOfPoint2f curve = new MatOfPoint2f();
        ArrayList<Point> valid = new ArrayList<>();
        ArrayList<MatOfPoint> maskBuffer = new ArrayList<>(); maskBuffer.add(new MatOfPoint());
        for (int i = 0; i < cnts.size(); i++) {
            MatOfPoint cnt = cnts.get(i);
            double parent = hierarchy.get(0, i)[3];

            cnt.convertTo(curve, CvType.CV_32FC2);

            double perimeter = Imgproc.arcLength(curve, true);
            Imgproc.approxPolyDP(curve, curve, perimeter * 0.04, true);
            double area = Imgproc.contourArea(curve);

            if (area < 10 * 10) {
                cnt.release();
                continue; // this should vary with distance- or calc it with pose estimate?
            }

            // draw this contour to mask
            // its fine if we have a noise, backboard will have very little noise
            // minor errors will then be filtered out when we get distance between centers and points
            maskBuffer.set(0, cnt);
            Imgproc.fillPoly(outMask, maskBuffer, new Scalar(255));
            cnt.release();

            // we only want the children (interior hexagons)
            // https://docs.opencv.org/4.x/d9/d8b/tutorial_py_contours_hierarchy.html
            if (parent == -1) continue;

            // this may be overkill, might only need to filter based on verts
            // ratio between area of hexagon and minEnclosingCircle should be ~0.827
            float[] radius = new float[1];
            Point center = new Point();
            Imgproc.minEnclosingCircle(curve, center, radius);
            double error = area / (radius[0] * radius[0] * Math.PI);

            if (error < 0.827 - 0.4 || error > 0.827 + 0.4) continue;

            long verts = curve.total();
            if (verts <= 4 || verts >= 9) continue;

            // get centroid
            Moments m = Imgproc.moments(curve);
            Point p = new Point(m.m10 / (m.m00 + 1e-5), m.m01 / (m.m00 + 1e-5));
            valid.add(p);
        }

        hierarchy.release();
        curve.release();
        segment.release();
        // mask buffer is alr released (see cnt)

        return valid;
    }

    private AprilTag detectAprilTags(ArrayList<AprilTagDetection> detections) {
        // average rvecs, get tvec of center
        ArrayList<Mat> rvecs = new ArrayList<>();
        Mat tvec = new Mat();

        int id = 0;
        for (AprilTagDetection detection : detections) {
            // the detections already solve pnp, so we don't need to do it
            Pose p = getCVPose(detection.pose);
            if (detection.id == BoardConstants.tagID.blue.center || detection.id == BoardConstants.tagID.red.center) {
                tvec = p.tvec;
                id = detection.id;
            }

            // sometimes the z vector (pointing towards camera) will be backwards
            // right now, hack is to just ignore these values
            // maybe need to recalibrate camera?
            // see discord for relevant github issues discussion, consider implementing that
            if (get1D(p.rvec, 1) < 0.4) {
                telemetry.add("WARNING 1", "RVEC likely is backwards, ignoring");
                continue;
            }

            rvecs.add(p.rvec);
        }

        if (tvec.empty()) {
            telemetry.add("WARNING", "Did not detect center tag!");
            tvec = getCVPose(detections.get(0).pose).tvec;
            id = detections.get(0).id;
        }

        if (rvecs.size() != 3) telemetry.add("WARNING", "Did not detect all 3 april tags");

        // now average rvec
        Mat rvec = new Mat(3, 1, CvType.CV_32F);
        set1D(rvec, 0, 0); set1D(rvec, 1, 0); set1D(rvec, 2, 0); // not sure if this is needed
        for (int i = 0; i < rvecs.size(); i++) {
            for (int j = i + 1; j < rvecs.size(); j++) {
                double error = nDistance(rvecs.get(i), rvecs.get(j));
                if (error > RVEC_ERROR_THRESHOLD) telemetry.add("WARNING", "RVEC (" + error + ") surpasses RVEC_ERROR_THRESHOLD");
            }

            // [i] += [i] / length
            for (int j = 0; j < 3; j++) { // iterate over each dimension
                double d = get1D(rvecs.get(i), j) / (double) rvecs.size();
                set1D(rvec, j, get1D(rvec, j) + d);
            }
        }

        return new AprilTag(new Pose(rvec, tvec), id);
    }

    private double nDistance(Mat v1, Mat v2) {
        Mat n1 = norm(v1);
        Mat n2 = norm(v2);

        return Math.sqrt(
            Math.pow(get1D(n1, 0) - get1D(n2, 0), 2) +
            Math.pow(get1D(n1, 1) - get1D(n2, 1), 2) +
            Math.pow(get1D(n1, 2) - get1D(n2, 2), 2)
        );
    }

    private Mat norm(Mat m) {
        double x = get1D(m, 0);
        double y = get1D(m, 1);
        double z = get1D(m, 2);
        double l = Math.sqrt(x * x + y * y + z * z);

        Mat out = new Mat(3, 1, CvType.CV_32F);
        set1D(out, 0, x / l);
        set1D(out, 1, y / l);
        set1D(out, 2, z / l);

        return out;
    }

    public void setStatus(pipelineStatus status) {
        this.status = status;
    }

    private double get1D(Mat m, int i) { return m.get(i, 0)[0]; }
    private void set1D(Mat m, int i, double d) { m.put(i, 0, d); }

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

    @Override
    protected void finalize() {
        if (detectorPtr != 0) AprilTagDetectorJNI.releaseApriltagDetector(detectorPtr);
    }

    enum pipelineStatus {
        running, idle, toStart, initializing
    }
}
