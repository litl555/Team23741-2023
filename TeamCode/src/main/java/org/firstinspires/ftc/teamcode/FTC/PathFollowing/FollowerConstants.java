package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FollowerConstants {
    public static double angleSpeed = .02;
    public static double kv = .0007;
    public static double xyTolerance = 50;
    public static double aTolerance = Math.toRadians(5);
    public static double dxy = 0.005;
    public static double kvCorrect = 0.06;
    public static double kvFollow = 0.007;
    public static double ka = .006;
    public static double kpxy = .002;
    public static double ks = .0001;
    public static double kp = .0003;
    public static double kpa = -2.0;

    public static double trajRunnerSpeedMult = .011;
}
