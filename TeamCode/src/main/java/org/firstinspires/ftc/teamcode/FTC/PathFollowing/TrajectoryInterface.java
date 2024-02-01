package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public interface TrajectoryInterface {
    double getTotalTime();

    Pose2d getEnd();

    Pose2d equation(double t);

    ArrayList<Double> getVelosSpaced();

    ArrayList<Double> getMp();

    Vector2d getCentripetalForceVector(double t);

    double getClosestTValue(Pose2d point);

    ArrayList<Double> getAmp();

    boolean getEndStopped();

    ArrayList<Double> getTimeValuesVar();

    Pose2d velocities(double t);

    Pose2d accelerrations(double t);

    Pose2d normalize(Pose2d pose);

    void forceSetP5(Pose2d p);
}
