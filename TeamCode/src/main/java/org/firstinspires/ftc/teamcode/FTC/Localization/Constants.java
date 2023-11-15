package org.firstinspires.ftc.teamcode.FTC.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
@Config
public class Constants {
    public OdometryModule rightPod=new OdometryModule(hardwareMap.dcMotor.get("leftFront"));
    public OdometryModule leftPod=new OdometryModule(hardwareMap.dcMotor.get("leftRear"));
    public OdometryModule backPod = new OdometryModule(hardwareMap.dcMotor.get("rightFront"));
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
    public static double angle = 0;
    public static final float Max_velo = 1;
    public static double PERPENDICULAR_X = -203.08f - 2.5f;//172.475 //forward and back
    public static double timed = 0;
    public static double maxVelocty = 1000;
    public static double yes = 0;
    public static double maxAcceleration = 400;
    public static final double PERPENDICULAR_Y = 0; //left to right
    public static final float X_OFFSET = 40.608f; //Offset of parallel wheels forward
    public static double LATERAL_DISTANCE = 284.84; //Distance beween odometry wheels 151.425f*2f
    public static final float ODO_WHEEL_RADIUS = 17.5f;
    public static final float PPR = 8192f;
    public static final Pose2d startPose = new Pose2d(0, 0, 0);

    public static Pose2d robotPose = new Pose2d(0, 0, 0);

    public static long getTime() {
        return (System.nanoTime());
    }

    public static double ticksToMM(int ticks) {
        return (ticks / (double) 8192 * ODO_WHEEL_RADIUS * 2 * Math.PI);
    }

    public static double toSec(long nano) {
        return nano / 1000000000.0;
    }

}
