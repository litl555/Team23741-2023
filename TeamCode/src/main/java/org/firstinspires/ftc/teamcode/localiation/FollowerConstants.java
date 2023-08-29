package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FollowerConstants {
    public static double kv = .00079;
    public static double xyTolerance = 1000;
    public static double aTolerance = Math.toRadians(90);

    public static double ka = .0001;
    public static double kpxy = .005;
    public static double ks = .0001;
    public static double kp = .001;
    public static double kpa = -1;
}
