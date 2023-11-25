package org.firstinspires.ftc.teamcode.FTC.Pixels.Constants;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.opencv.core.Scalar;

import java.util.Hashtable;
import java.util.Map;

public class BoardConstants {
    // all in meters
    public static final double inchToMeter = 1.0 / 39.37;
    public static final double tagSize = 2.0 * inchToMeter;
    public static final double tagCenterHeight = 4.116025 * inchToMeter;
    public static final double tagCenterToFirstRowCenter = (1.25 + 2.0) * inchToMeter;

    public static final double relativeSetLineOneHeight = 12.375 * inchToMeter - tagCenterHeight;
    public static final double relativeSetLineTwoHeight = 19.0 * inchToMeter - tagCenterHeight;
    public static final double relativeSetLineThreeHeight = 25.75 * inchToMeter - tagCenterHeight;

    public static class tagID {
        public static class blue {
            public static final int left = 1;
            public static final int center = 2;
            public static final int right = 3;
        }

        public static class red {
            public static final int left = 4;
            public static final int center = 5;
            public static final int right = 6;
        }

        // IN METERS
        public static Map<Integer, Pose2d> positionFromCenter = new Hashtable<Integer, Pose2d>() {{
            double x = (24 + 12) * inchToMeter;
            double y = (48 + 24 - 11.25) * inchToMeter;
            double t = 6 * inchToMeter; // distance from each tag center to each other
            Rotation2d rot = new Rotation2d();

            put(blue.left, new Pose2d(-x - t, y, rot));
            put(blue.center, new Pose2d(-x, y, rot));
            put(blue.right, new Pose2d(-x + t, y, rot));

            put(red.left, new Pose2d(x - t, y, rot));
            put(red.center, new Pose2d(x, y, rot));
            put(red.right, new Pose2d(x + t, y, rot));
        }};
    }

    public static class hsv {
        public static class white {
            public static final Scalar lower = new Scalar(0, 0, 150);
            public static final Scalar upper = new Scalar(200, 25, 255);
        }

        public static class purple {
            public static final Scalar lower = new Scalar(80, 25, 80);
            public static final Scalar upper = new Scalar(200, 255, 255);
        }

        public static class green {
            public static final Scalar lower = new Scalar(40, 100, 30);
            public static final Scalar upper = new Scalar(100, 255, 255);
        }

        public static class yellow {
            public static final Scalar lower = new Scalar(15, 50, 110);
            public static final Scalar upper = new Scalar(40, 255, 255);
        }
    }
}
