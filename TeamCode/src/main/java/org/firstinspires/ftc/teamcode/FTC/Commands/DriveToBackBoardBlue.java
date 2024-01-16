package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultipleTrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToBackBoardBlue extends CommandBase {
    Trajectory trajectory;
    boolean finished = false;
    TrajectoryRunner tr;
    public static double leftPosBlue = 180.0;
    public static double leftCloseBlue = -30.0;
    public static double angleLeft = 8.0;
    public static double rightPosBlue = -180.0;
    public static double rightCloseBlue = -10.0;
    public static double angleRight=-8.0;
    MultipleTrajectoryRunner mtr;
    public static Pose2d end = new Pose2d(844, 1145);
    TeamPropPosition position; //1=left 2=center 3=right

    public DriveToBackBoardBlue(TeamPropPosition position) {
        this.position = position;
    }

    @Override
    public void initialize() {
        trajectory = new Trajectory(new Pose2d(-1500, 300), new Pose2d(end.getX(), end.getY()), new Pose2d(-1540, -10), new Pose2d(0, 600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
        tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, -180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

        switch (position) {
            case left:
                trajectory = new Trajectory(new Pose2d(-1500, 300), new Pose2d(-1.0 * (end.getX() + leftPosBlue), end.getY() + leftCloseBlue), new Pose2d(100, 100), new Pose2d(1900, 1600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, -(180.0 + angleLeft), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

                break;
            case middle:
                trajectory = new Trajectory(new Pose2d(-1500, 300), new Pose2d(-1.0 * end.getX(), end.getY()), new Pose2d(1540, -10), new Pose2d(0, 600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, -180.0, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

                break;
            case right:
                trajectory = new Trajectory(new Pose2d(-1500, 300), new Pose2d((end.getX() + rightPosBlue) * -1.0, end.getY()), new Pose2d(1540, -10), new Pose2d(0, 600), new Pose2d(0, 0), new Pose2d(0, 0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, trajectory, 180.0+angleRight, TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);

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
