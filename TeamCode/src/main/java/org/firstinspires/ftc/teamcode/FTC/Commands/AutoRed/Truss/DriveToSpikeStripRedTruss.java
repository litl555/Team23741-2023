package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultiTrajEvent;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.Trajectory;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveToSpikeStripRedTruss extends CommandBase {
    private static final double inToMm = 25.4;

    private TeamPropPosition pos;
    private Command pixelPlaceCommand;
    private ActualMultiTrajRunner tr;

    public static double leftXOffset = 0;
    public static double leftYOffset = 0;
    public static double middleXOffset = -60;
    public static double middleYOffset = -40;
    public static double rightXOffset = 0;
    public static double rightYOffset = 0;
    public Pose2d rightPos = new Pose2d(34 * inToMm + rightXOffset, -24 * inToMm - Robot.length / 2.0 - 40 + rightYOffset, 180);
    public Pose2d middlePos = new Pose2d(24 * inToMm - Robot.width / 2.0 - 10 + middleXOffset, -36 * inToMm + middleYOffset, -90.0);
    public Pose2d leftPos = new Pose2d(34 * inToMm + leftXOffset, -48 * inToMm + leftYOffset, 90);

    public DriveToSpikeStripRedTruss(TeamPropPosition pos, Command pixelPlaceCommand) {
        this.pos = pos;
        this.pixelPlaceCommand = pixelPlaceCommand;
    }

    @Override
    public void initialize() {
        Pose2d startPose = new Pose2d(Robot.customLocalization.getPoseEstimate().getY() * -1.0, Robot.customLocalization.getPoseEstimate().getX(), 0);
        // shared position to prep for 2+1
        Pose2d base = new Pose2d(12 * inToMm, -48 * inToMm);

        SimpleTrajectory toStrip = null;
        SimpleTrajectory stripToBase = null;

        switch (pos) {
            case right:
                toStrip = new SimpleTrajectory(startPose, rightPos, new Pose2d(0.0, 0), new Pose2d(-1160, 1327), rightPos.getHeading());
                stripToBase = new SimpleTrajectory(rightPos, base, new Pose2d(18, -1200), new Pose2d(0, 0), -180);
                break;
            case undefined: // if undefined go to middle
            case middle:
                toStrip = new SimpleTrajectory(startPose, middlePos, new Pose2d(0.0, 0), new Pose2d(-1300, 530), middlePos.getHeading());
                // odo gets stuck in the "stays in the same place" thingy if it is -180, so have to give it a bit extra movement :/
                stripToBase = new SimpleTrajectory(middlePos, base, new Pose2d(0, 0), new Pose2d(0, 0), 180);
                break;
            case left:
                toStrip = new SimpleTrajectory(startPose, leftPos, new Pose2d(0.0, 0), new Pose2d(-870, 85), leftPos.getHeading());
                stripToBase = new SimpleTrajectory(leftPos, base, new Pose2d(0, 0), new Pose2d(-2560, 990), -180);
                break;
        }

        tr = new ActualMultiTrajRunner(
            new SimpleTrajectory[] {toStrip, stripToBase},
            new MultiTrajEvent[] {new MultiTrajEvent(0, pixelPlaceCommand)});

        tr.start();
    }

    @Override
    public void execute() { tr.update(); }
    @Override
    public boolean isFinished() {
        return tr.hasFinished();
    }
}
