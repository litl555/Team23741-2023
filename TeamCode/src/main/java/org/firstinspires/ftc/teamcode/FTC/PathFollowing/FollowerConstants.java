package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class FollowerConstants {
    public static double angleSpeed = .03;
    public static double kv = .0007;
    public static double xyTolerance = 20;
    public static double aTolerance = Math.toRadians(3);
    public static double kixy = 0.001;
    public static double dxy = 0.3;
    public static double kvCorrect = 0.08;
    public static double kvFollow = 20;
    public static double ka = .006;
    public static double kpxy = .003;
    public static double ks = .0001;
    public static double kp = .0003;
    public static double kpa = -5.0;

    public static double trajRunnerSpeedMult = .025;
}
