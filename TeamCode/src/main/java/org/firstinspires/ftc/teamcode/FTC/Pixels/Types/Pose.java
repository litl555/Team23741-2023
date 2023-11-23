package org.firstinspires.ftc.teamcode.FTC.Pixels.Types;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class Pose {
    public Mat rvec, tvec;

    public Pose() {
        rvec = new Mat(3, 1, CvType.CV_32F);
        tvec = new Mat(3, 1, CvType.CV_32F);
    }

    public Pose(Mat rvec, Mat tvec) {
        this.rvec = rvec;
        this.tvec = tvec;
    }
}
