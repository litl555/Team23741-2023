package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class DriveToBackBoardRedTruss extends CommandBase {
    private static final double inToMm = 25.4;

    private TeamPropPosition pos;
    private ActualMultiTrajRunner tr;

    public static double leftXOffset = -230;
    public static double leftYOffset = 0;
    public static double middleXOffset = -10;
    public static double middleYOffset = -10;
    public static double rightXOffset = 120;
    public static double rightYOffset = 0;


    private Pose2d backBoardCenter = new Pose2d(36 * inToMm, 46 * inToMm);
    private Pose2d middlePos = new Pose2d(backBoardCenter.getX() + middleXOffset, backBoardCenter.getY() + middleYOffset);
    private Pose2d rightPos = new Pose2d(backBoardCenter.getX() + rightXOffset, backBoardCenter.getY() + rightYOffset);
    private Pose2d leftPos = new Pose2d(backBoardCenter.getX() + leftXOffset, backBoardCenter.getY() + leftYOffset);

    public DriveToBackBoardRedTruss(TeamPropPosition pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {
        Pose2d startPose = new Pose2d(Robot.customLocalization.getPoseEstimate().getY() * -1.0, Robot.customLocalization.getPoseEstimate().getX(), 0);
        Pose2d base = new Pose2d(12 * inToMm, -48 * inToMm);
        Pose2d intermediary = new Pose2d(12 * inToMm, 36 * inToMm);

        SimpleTrajectory startToBase = null;
        SimpleTrajectory baseToInter = new SimpleTrajectory(base, intermediary, new Pose2d(0 , 0), new Pose2d(0, 0), -180);
        SimpleTrajectory interToEnd = null;

        switch (pos) {
            case right:
                startToBase = new SimpleTrajectory(startPose, base, new Pose2d(18, -1200), new Pose2d(0, 0), -180);
                interToEnd = new SimpleTrajectory(intermediary, rightPos, new Pose2d(0, 0), new Pose2d(210, 1078), -180);
                break;
            case left:
                startToBase = new SimpleTrajectory(startPose, base, new Pose2d(0, 0), new Pose2d(-2560, 990), -180);
                interToEnd = new SimpleTrajectory(intermediary, leftPos, new Pose2d(0, 0), new Pose2d(69, 978), -180);
                break;
            case undefined: // if undefined go to middle
            case middle:
                startToBase = new SimpleTrajectory(startPose, base, new Pose2d(0, 0), new Pose2d(349, -744), -180);
                interToEnd = new SimpleTrajectory(intermediary, middlePos, new Pose2d(0, 0), new Pose2d(913, -20), -180);
                break;
        }

        tr = new ActualMultiTrajRunner(new SimpleTrajectory[] {startToBase, baseToInter, interToEnd});
        tr.start();
    }

    @Override
    public void execute() {
        tr.update();
    }

    @Override
    public boolean isFinished() {
        return tr.hasFinished();
    }
}
