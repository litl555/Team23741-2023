package org.firstinspires.ftc.teamcode.FTC.PathFollowing;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class SimpleTrajectory {
    private Pose2d start, end, velStart, velEnd;
    private double heading;
    public SimpleTrajectory(Pose2d start, Pose2d end, Pose2d velStart, Pose2d velEnd, double heading) {
        this.start = start;
        this.end = end;
        this.velStart = velStart;
        this.velEnd = velEnd;
        this.heading = heading;
    }

    public Trajectory getTrajectory(boolean startStopped, boolean endStopped) {
        return new Trajectory(start, end, velStart, velEnd, new Pose2d(0, 0), new Pose2d(0, 0), startStopped, endStopped);
    }

    public TrajectoryRunner getTrajectoryRunner(Trajectory traj) {
        return new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, traj, heading, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
    }

    public double getHeading() { return heading; }
}
