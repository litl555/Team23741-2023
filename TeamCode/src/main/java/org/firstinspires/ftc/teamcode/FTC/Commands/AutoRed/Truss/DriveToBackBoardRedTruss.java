package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.BoardTagLocalizationPipeline;
import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultiTrajEvent;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToBackBoardRedTruss extends CommandBase {
    private static final double inToMm = 25.4;

    private TeamPropPosition pos;
    private ActualMultiTrajRunner tr;

    public static double leftXOffset = -230;
    public static double leftYOffset = 0;
    public static double middleXOffset = -30;
    public static double middleYOffset = -20;
    public static double rightXOffset = 320;
    public static double iterOneOffY = 40;
    public static double rightYOffset = 0;
    public static double yOffsetGlobal = -60.0;

    private Pose2d backBoardCenter = new Pose2d(36 * inToMm, 46 * inToMm + yOffsetGlobal);
    private Pose2d middlePos, rightPos, leftPos;


    public DriveToBackBoardRedTruss(TeamPropPosition pos, int iter) {
        this.pos = pos;

        if (iter == 1) {
            if (pos == TeamPropPosition.left) this.pos = TeamPropPosition.middle;
            backBoardCenter = new Pose2d(36 * inToMm, 46 * inToMm + yOffsetGlobal - iterOneOffY);
        }
        middlePos = new Pose2d(backBoardCenter.getX() + middleXOffset, backBoardCenter.getY() + middleYOffset);
        rightPos = new Pose2d(backBoardCenter.getX() + rightXOffset, backBoardCenter.getY() + rightYOffset);
        leftPos = new Pose2d(backBoardCenter.getX() + leftXOffset, backBoardCenter.getY() + leftYOffset);
    }

    @Override
    public void initialize() {
        Pose2d startPose = new Pose2d(Robot.customLocalization.getPoseEstimate().getY() * -1.0, Robot.customLocalization.getPoseEstimate().getX(), 0);
        // shared position just before we start pixel placing movement
        //Pose2d intermediary = new Pose2d(10 * inToMm, 36 * inToMm);
        Pose2d intermediary = new Pose2d(24 * inToMm, 36 * inToMm);
        Pose2d camPos = new Pose2d(10 * inToMm, 0);
        SimpleTrajectory baseToInter = new SimpleTrajectory(startPose, intermediary, new Pose2d(-870, 1625), new Pose2d(1140, 460), pos == TeamPropPosition.right ? 180 : -180);
        //SimpleTrajectory baseToInter = new SimpleTrajectory(startPose, intermediary, new Pose2d(-870, 1625), new Pose2d(0, 0), pos == TeamPropPosition.right ? 180 : -180);
        //SimpleTrajectory camToInter = new SimpleTrajectory(camPos, intermediary, new Pose2d(0, 0), new Pose2d(0, 0), pos == TeamPropPosition.right ? 180 : -180);
        SimpleTrajectory interToEnd = null;

        Pose2d endPos = null;

        switch (pos) {
            case right:
                endPos = rightPos;
                interToEnd = new SimpleTrajectory(intermediary, rightPos, new Pose2d(0, 0), new Pose2d(210, 1078), 180);
                break;
            case left:
                endPos = leftPos;
                interToEnd = new SimpleTrajectory(intermediary, leftPos, new Pose2d(0, 0), new Pose2d(69, 978), -180);
                break;
            case undefined: // if undefined go to middle
            case middle:
                endPos = middlePos;
                interToEnd = new SimpleTrajectory(intermediary, middlePos, new Pose2d(0, 0), new Pose2d(913, -20), -180);
                break;
        }

        //BoardTagLocalizationPipeline.endTruth = new Pose2d(intermediary.getX(), intermediary.getY(), intermediary.getHeading());
        tr = new ActualMultiTrajRunner(
            new SimpleTrajectory[] {baseToInter, interToEnd},
            new MultiTrajEvent[] {
                new MultiTrajEvent(0,
                    new SequentialCommandGroup(
                        new InstantCommand(() -> BoardTagLocalizationPipeline.shouldGetPosition = true),
                        new WaitUntilCommand(() -> BoardTagLocalizationPipeline.finalDeltaX != 0),
                        new InstantCommand(() -> {
                            switch (pos) {
                                case middle:
                                    tr.trajectories[1].forceSetP5(new Pose2d(middlePos.getX() + BoardTagLocalizationPipeline.finalDeltaX, middlePos.getY(), middlePos.getHeading()));
                                    break;
                            }
                        })
                )), new MultiTrajEvent(1, new InstantCommand(() -> {
                    Constants.robotPose = new Pose2d(intermediary.getY(), -intermediary.getX());
                }))
            });
        /*
        tr = new ActualMultiTrajRunner(
            new SimpleTrajectory[] {baseToInter, interToEnd},
            new MultiTrajEvent[] {new MultiTrajEvent(0,
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        BoardTagLocalizationPipeline.shouldGetPosition = true;
                        BoardTagLocalizationPipeline.endTruth = new Pose2d(middlePos.getX(), middlePos.getY(), middlePos.getHeading());
                    }),
                    new WaitCommand(500)
                ))});
        */
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
