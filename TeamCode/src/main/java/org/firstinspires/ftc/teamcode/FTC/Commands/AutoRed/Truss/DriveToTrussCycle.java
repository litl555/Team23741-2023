package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultiTrajEvent;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class DriveToTrussCycle extends CommandBase {
    ActualMultiTrajRunner mtr;
    private boolean finished = false;

    @Override
    public void initialize() {
        SimpleTrajectory trajectory = new SimpleTrajectory(new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX()), new Pose2d(200, 200), new Pose2d(0, -300), new Pose2d(0, 0), -180);
        mtr = new ActualMultiTrajRunner(new SimpleTrajectory[]{trajectory});
        mtr.start();
    }

    @Override
    public void execute() {
        mtr.update();
        if (mtr.hasFinished()) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (finished);
    }

}
