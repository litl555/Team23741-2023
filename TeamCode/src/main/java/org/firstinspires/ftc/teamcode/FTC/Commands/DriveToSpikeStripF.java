package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToSpikeStripF extends CommandBase {
    TeamPropPosition pos;
    TrajectoryRunner tr = null;
    Trajectory trajectory;
    private boolean finished = false;
    public static Pose2d endPos3 = new Pose2d(934.858, 488.861, 90.0);
    public static Pose2d endPos2 = new Pose2d(818.479, 269.194, 90.0);
    public static Pose2d endPos1 = new Pose2d(774.279, 231.441, 0.0);

    public DriveToSpikeStripF(TeamPropPosition pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {

        Pose2d startPose = new Pose2d(Robot.l.getPoseEstimate().getY() * -1.0, Robot.l.getPoseEstimate().getX(), 0);
        switch (pos) {
            case right:
                trajectory = new Trajectory(startPose, endPos3, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.l, trajectory, endPos3.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case middle:
                trajectory = new Trajectory(startPose, endPos2, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.l, trajectory, endPos2.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case left:
                trajectory = new Trajectory(startPose, endPos1, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.l, trajectory, endPos1.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case undefined:
                trajectory = new Trajectory(startPose, endPos1, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.l, trajectory, endPos1.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
        }
        tr.start();

    }

    @Override
    public void execute() {
        tr.update();
        if (tr.currentState == TrajectoryRunner.State.FINISHED) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (finished);
    }
}
