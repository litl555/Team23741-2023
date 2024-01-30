package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultiTrajEvent;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToTrussCycle extends CommandBase {
    ActualMultiTrajRunner mtr;
    public static double xoffright = 100;
    public static double xoffmiddle = 100;
    public static double xoffleft = 100;
    private boolean finished = false;
    private TeamPropPosition pos;

    public DriveToTrussCycle(TeamPropPosition propPosition) {
        pos = propPosition;
    }

    @Override
    public void initialize() {
        SimpleTrajectory trajectory = null;
        switch (pos) {
            case right:
                trajectory = new SimpleTrajectory(new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX()), new Pose2d(200 + xoffright, 200), new Pose2d(0, -300), new Pose2d(0, 0), 180.0);
                break;
            case middle:
                trajectory = new SimpleTrajectory(new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX()), new Pose2d(200 + xoffmiddle, 200), new Pose2d(0, -300), new Pose2d(0, 0), -180.0);
                break;
            case left:
                trajectory = new SimpleTrajectory(new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX()), new Pose2d(200 + xoffleft, 200), new Pose2d(0, -300), new Pose2d(0, 0), -180.0);
                break;

        }
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
