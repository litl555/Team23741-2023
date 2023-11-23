package org.firstinspires.ftc.teamcode.FTC.Pixels.Constants;

import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.PixelColor;
import org.opencv.core.Scalar;

import java.util.HashMap;
import java.util.Map;

public class PixelConstants { // all in inches
    public static final double width = 3.0 * BoardConstants.inchToMeter;
    public static final double size = width / Math.sqrt(3);
    public static final double height = 2.0 * size;
    public static final double horz = width; // horizontal spacing between center of pixels
    public static final double vert = 3.0 / 2.0 * size; // vertical spacing between center of pixels

    public static final Scalar rgbEmpty = new Scalar(0, 0, 0);
    public static final Scalar rgbWhite = new Scalar(255, 255, 255);
    public static final Scalar rgbPurple = new Scalar(255, 0, 255);
    public static final Scalar rgbGreen = new Scalar(0, 255, 0);
    public static final Scalar rgbYellow = new Scalar(255, 255, 0);
    public static final Map<PixelColor, Scalar> colorToRgb = new HashMap<PixelColor, Scalar>() {{ // java
        put(PixelColor.empty, rgbEmpty);
        put(PixelColor.white, rgbWhite);
        put(PixelColor.green, rgbGreen);
        put(PixelColor.yellow, rgbYellow);
        put(PixelColor.purple, rgbPurple);
    }};

    public static final double[][] neighborsCube = new double[][] {
        {1, 0, -1}, {1, -1, 0}, {0, -1, 1},
        {-1, 0, 1}, {-1, 1, 0}, {0, 1, -1}
    };

    public static final double[][] neighborsAxial = new double[][] {
        {1, 0}, {1, -1}, {0, -1},
        {-1, 0}, {-1, 1}, {0, 1}
    };
}
