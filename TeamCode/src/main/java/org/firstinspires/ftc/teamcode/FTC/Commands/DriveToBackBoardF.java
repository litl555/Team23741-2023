package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Pixels.Types.Pose;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class DriveToBackBoardF extends CommandBase {
    Trajectory trajectory;
    boolean finished = false;
    TrajectoryRunner tr;
    double leftPos = -30;
    double rightPos = 30;
    MultipleTrajectoryRunner mtr;
    public static Pose2d end = new Pose2d(900, 1200);
    int position; //1=left 2=center 3=right

    public DriveToBackBoardF(int position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        if (position == 1) {
            trajectory = new Trajectory(new Pose2d(1500, 300), new Pose2d(end.getX() + leftPos, end.getY()), new Pose2d(-1540, -10), new Pose2d(0, 600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        } else if (position == 2) {
            trajectory = new Trajectory(new Pose2d(1500, 300), new Pose2d(end.getX(), end.getY()), new Pose2d(-1540, -10), new Pose2d(0, 600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        } else {
            trajectory = new Trajectory(new Pose2d(1500, 300), new Pose2d(end.getX(), end.getY() + rightPos), new Pose2d(-1540, -10), new Pose2d(0, 600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);

        }
        tr = new TrajectoryRunner(Robot.hardwareMap, Robot.l, trajectory, 0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
        tr.start();
    }

    @Override
    public void execute() {
        if (tr.currentState != TrajectoryRunner.State.FINISHED) {
            tr.update();
        } else {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (finished);
    }
}
