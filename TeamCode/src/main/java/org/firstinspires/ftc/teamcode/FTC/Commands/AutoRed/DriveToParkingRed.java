package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToParkingRed extends CommandBase {
    private boolean finished = false;
    private Trajectory traj = null;
    private TrajectoryRunner tr = null;

    public static double xOffset = 0, yOffset = -50;
    public static double vel_x = 500, vel_y = 2397;
    private double heading;
    public DriveToParkingRed(double heading) {
        this.heading = heading;
    }


    @Override
    public void initialize() {
        Pose2d initPos = new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX());

        traj = new Trajectory(initPos, new Pose2d(1500 + xOffset, 1500 + yOffset), new Pose2d(0, 0), new Pose2d(vel_x, vel_y), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        // try to prevent the robot from turning as much as possible, since depending of if were on truss side or not the robot could be at -180 or 180
        tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, traj, heading, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

        tr.start();
    }

    @Override
    public void execute() {
        if (tr.currentState != TrajectoryRunner.State.FINISHED) tr.update();
        else finished = true;
    }

    @Override
    public boolean isFinished() {
        return (finished);
    }
}
