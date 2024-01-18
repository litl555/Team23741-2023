package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToSpikeStripRedTruss extends CommandBase {
    private static final double inToMm = 25.4;

    private TeamPropPosition pos;
    private boolean finished = false;
    private Trajectory traj = null;
    private TrajectoryRunner tr = null;

    public static double leftXOffset = 0;
    public static double leftYOffset = 0;
    public static double middleXOffset = -60;
    public static double middleYOffset = -40;
    public static double rightXOffset = 0;
    public static double rightYOffset = 0;
    public Pose2d rightPos = new Pose2d(34 * inToMm + rightXOffset, -24 * inToMm - Robot.length / 2.0 - 40 + rightYOffset, 180);
    public Pose2d middlePos = new Pose2d(24 * inToMm - Robot.width / 2.0 - 10 + middleXOffset, -36 * inToMm + middleYOffset, -90.0);
    public Pose2d leftPos = new Pose2d(34 * inToMm + leftXOffset, -48 * inToMm + leftYOffset, 90);

    public DriveToSpikeStripRedTruss(TeamPropPosition pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {
        Pose2d startPose = new Pose2d(Robot.customLocalization.getPoseEstimate().getY() * -1.0, Robot.customLocalization.getPoseEstimate().getX(), 0);

        switch (pos) {
            case right:
                traj = new Trajectory(startPose, rightPos, new Pose2d(0.0, 0), new Pose2d(-1160, 1327), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, false);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, traj, rightPos.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case undefined: // if undefined go to middle
            case middle:
                traj = new Trajectory(startPose, middlePos, new Pose2d(0.0, 0), new Pose2d(-1300, 530), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, traj, middlePos.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
            case left:
                traj = new Trajectory(startPose, leftPos, new Pose2d(0, 0), new Pose2d(-870, 85), new Pose2d(0.0, 0.0), new Pose2d(0.0, 0.0), true, true);
                tr = new TrajectoryRunner(Robot.hardwareMap, Robot.customLocalization, traj, leftPos.getHeading(), TrajectoryRunner.HeadingType.ConstantHeadingVelo, Robot.telemetry);
                break;
        }
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
