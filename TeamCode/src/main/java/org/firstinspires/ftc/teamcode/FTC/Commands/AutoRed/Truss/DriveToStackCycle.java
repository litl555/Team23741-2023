package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;

public class DriveToStackCycle extends CommandBase {
    ActualMultiTrajRunner mtr;
    private boolean finished = false;
    Pose2d base = new Pose2d(13.5 * 25.4 + 150, -58.5 * 25.4 - 40);

    @Override
    public void initialize() {
        SimpleTrajectory trajectory = new SimpleTrajectory(new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX()), new Pose2d(base.getX(), base.getY()), new Pose2d(0, 0), new Pose2d(300, -400), -180);
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
