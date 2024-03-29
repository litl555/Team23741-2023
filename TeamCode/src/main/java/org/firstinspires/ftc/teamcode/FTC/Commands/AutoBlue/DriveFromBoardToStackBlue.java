package org.firstinspires.ftc.teamcode.FTC.Commands.AutoBlue;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Commands.ReturnClawToTray;
import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.ActualMultiTrajRunner;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.MultiTrajEvent;
import org.firstinspires.ftc.teamcode.FTC.PathFollowing.SimpleTrajectory;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

@Config
public class DriveFromBoardToStackBlue extends CommandBase {
    private ActualMultiTrajRunner mtr;
    private TeamPropPosition pos;
    private long clawReturnDelay;

    public DriveFromBoardToStackBlue(TeamPropPosition pos, long clawReturnDelay) {
        this.pos = pos;
        this.clawReturnDelay = clawReturnDelay;
    }

    public static double yoffset = 0;
    public static double xBase = -240;
    public static double xStackOffset=-150;
    @Override
    public void initialize() {
        SimpleTrajectory boardToBase = null, baseToStack = null;

        Pose2d start = Constants.getCurrentFieldCoords();
        Pose2d base = new Pose2d(xBase, 200);
        Pose2d stack = new Pose2d(-12 * 25.4+xStackOffset, -58.5 * 25.4 + yoffset);

        switch (pos) {
            case right:
                boardToBase = new SimpleTrajectory(start, base, new Pose2d(320, -500), new Pose2d(0, -2400), 180);
                baseToStack = new SimpleTrajectory(base, stack, new Pose2d(0, 0), new Pose2d(-300, -400), 180);
                break;
            case left:
                boardToBase = new SimpleTrajectory(start, base, new Pose2d(320, -54), new Pose2d(0, -2400), -180);
                baseToStack = new SimpleTrajectory(base, stack, new Pose2d(0, 0), new Pose2d(-300, -400), -180);
                break;
            case undefined:
            case middle:
                boardToBase = new SimpleTrajectory(start, base, new Pose2d(320, -54), new Pose2d(0, -2400), 180);
                baseToStack = new SimpleTrajectory(base, stack, new Pose2d(0, 0), new Pose2d(-300, -400), 180);
                break;
        }

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new WaitCommand(clawReturnDelay), new ReturnClawToTray()));
        mtr = new ActualMultiTrajRunner(
            new SimpleTrajectory[]{boardToBase, baseToStack},
            new MultiTrajEvent[]{
                new MultiTrajEvent(0,
                    new WaitUntilCommand(() -> Robot.liftSubsystem.targetPos == Robot.liftSubsystem.rowHeights[0]))});

        mtr.start();
    }

    @Override
    public void execute() {
        mtr.update();
    }

    @Override
    public boolean isFinished() {
        return mtr.hasFinished();
    }
}
