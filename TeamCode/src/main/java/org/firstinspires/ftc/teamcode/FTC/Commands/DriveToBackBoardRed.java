package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToBackBoardRed extends CommandBase {
    Trajectory trajectory;
    boolean finished = false;
    TrajectoryRunner tr;
    public static double leftXOffset = -230, leftYOffset = 0;
    public static double middleXOffset = 20, middleYOffset = 0;
    public static double rightXOffset = 120, rightYOffset = 0;
    // THESE ARE NOT OFFSETS!
    public static double vel_leftX = 0, vel_leftY = 600;
    public static double vel_middleX = 0, vel_middleY = 600;
    public static double vel_rightX = -1117, vel_rightY = 564;
    public static Pose2d end = new Pose2d(844, 1145);
    TeamPropPosition position; //1=left 2=center 3=right

    public DriveToBackBoardRed(TeamPropPosition position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        Pose2d initPos = new Pose2d(Constants.robotPose.getY() * -1.0, Constants.robotPose.getX());

        switch (position) {
            case left:
                trajectory = new Trajectory(initPos, new Pose2d(end.getX() + leftXOffset, end.getY() + leftYOffset), new Pose2d(0, 0), new Pose2d(vel_leftX, vel_leftY), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case undefined:
            case middle:
                trajectory = new Trajectory(initPos, new Pose2d(end.getX() + middleXOffset, end.getY() + middleYOffset), new Pose2d(500, 0), new Pose2d(vel_middleX, vel_middleY), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case right:
                trajectory = new Trajectory(initPos, new Pose2d(end.getX() + rightXOffset, end.getY() + rightYOffset), new Pose2d(0, 0), new Pose2d(vel_rightX, vel_rightY), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, 180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
        }

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
