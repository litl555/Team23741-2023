package org.firstinspires.ftc.teamcode.FTC.Commands.AutoBlue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class DriveToParkingBlue extends CommandBase {
    private boolean finished = false;
    private Trajectory traj = null;
    private TrajectoryRunner tr = null;

    public static double xOffset = 50, yOffset = -50;
    public static double vel_x = -500, vel_y = 2397;
    @Override
    public void initialize() {
        Pose2d initPos = new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX());

        // were moving to the same destination so just use the same start pos for all of them
        traj = new Trajectory(initPos, new Pose2d(-1500 + xOffset, 1500 + yOffset), new Pose2d(0, 0), new Pose2d(vel_x, vel_y), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, traj, -180, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

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
