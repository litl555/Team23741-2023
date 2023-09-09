package org.firstinspires.ftc.teamcode.localiation;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FollowerConstants {
    public static double kv = .0007;
    public static double xyTolerance = 20;
    public static double aTolerance = Math.toRadians(5);

    public static double ka = .0001;
    public static double kpxy = .002;
    public static double ks = .0001;
    public static double kp = .0003;
    public static double kpa = -1;
}
