package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToSpikeStripRed extends CommandBase {
    TeamPropPosition pos;
    TrajectoryRunner tr = null;
    Trajectory trajectory;
    private boolean finished = false;
    public static double leftXOffset = 20;
    public static double leftYOffset = 20.0;
    public static double middleXOffset = -40.0;
    public static double middleYOffset = 50.0;
    public static double rightXOffset = -30.0;
    public static double rightYOffset = -30.0;
    public Pose2d rightPos = new Pose2d(1106.858 + rightXOffset, 523.861 + rightYOffset, 90.0);
    public Pose2d middlePos = new Pose2d(840.479 + middleXOffset, 269.194 + middleYOffset, 90.0);
    public Pose2d leftPos = new Pose2d(772.279 + leftXOffset, 213.441 + leftYOffset, 2.0);

    //private final double inToMm = 25.4;
    //public static double _middle_x_offset = 20;
    //public static double _middle_y_offset = 60;
    //public Pose2d middlePos = new Pose2d(25 * inToMm + Robot.length / 2.0 + 40 + _middle_x_offset, 16 * inToMm + _middle_y_offset, 90);

    public DriveToSpikeStripRed(TeamPropPosition pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {
        Pose2d startPose = new Pose2d(Robot.customLocalization.getPoseEstimate().getY() * -1.0, Robot.customLocalization.getPoseEstimate().getX(), 0);

        switch (pos) {
            case right:
                trajectory = new Trajectory(startPose, rightPos, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, rightPos.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case undefined: // if undefined go to middle
            case middle:
                trajectory = new Trajectory(startPose, middlePos, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, middlePos.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case left:
                trajectory = new Trajectory(startPose, leftPos, new Pose2d(0.0, 400.0), new Pose2d(-400.0, -500.0), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, leftPos.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
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
