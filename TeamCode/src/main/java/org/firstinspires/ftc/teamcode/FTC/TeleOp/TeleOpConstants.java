package org.firstinspires.ftc.teamcode.FTC.TeleOp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TeleOpConstants {
    public static double liftUpSpeed = 0.7;
    public static double liftDownSpeed = 0.3;

    public static double wristIntake = 0.476667;
    public static double wristIntakeGrab = 0.4783333;
    public static double wristClearing = 0.5;
    public static double wristGround = 0.50833;
    public static double wristPlacing = 0.566666667;

    public static double armIntake = 0.483333;
    public static double armPlace1 = 0.5; // 1x , 2, 3 dont mean anything- just for testing
    public static double armPlace2 = 0.5;
    public static double armGround = 0.39888;


    // TODO: is the tick rate constant? ie do we have to do like .deltaTime shenanigans (ie Unity)
    public static double overrideArmSpeed = 0.003;
    public static double overrideWristSpeed = 0.003;

    /*
     * go down, then wrist fowards to 0.49
     * close
     * then wrist back to previous set (0.4677778)
     */

    public static double armAdjust1 = 0.018;
    public static double armAdjust2 = 0.015;
    public static int liftWait1 = 100;
    public static int armWait2 = 600;
    public static int armWait3 = 500;
}
