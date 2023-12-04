package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.FTC.Localization.CustomLocalization;
import org.firstinspires.ftc.teamcode.FTC.Localization.LoggerTool;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

/**
 * Drive to backboard and places both pixels on backboard. Commands go: begin driving, lift lift and
 * set wrist and claw when past truss, align once finished with center spline, pid to position,
 * place pixels on board.
 */
public class PlacePixel extends SequentialCommandGroup {

    private Pose2d leftColumn = new Pose2d(0, 0, 0);

    public PlacePixel(ClawSubsystem claw, LiftSubsystem lift, DriveSubsystem drive, int row1, int column1, int row2, int column2, CustomLocalization l) {
        DriveToBackdrop driveToBackdrop = new DriveToBackdrop(drive, Robot.telemetry);
        addCommands(
                new ScheduleCommand(driveToBackdrop),
                new WaitUntilCommand(Robot::isPastTruss),
                new ParallelCommandGroup(
                        new GoToHeight(new LiftSubsystem(), new ClawSubsystem(), row1),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(driveToBackdrop::isFinished),
                                //Snap a pic and correct

                                new PIDToPose(leftColumn.plus(new Pose2d(column1, 0, 0)), l)
                        )
                ),
                new UpdateClaw(claw, ClawSubsystem.ClawState.OPENONE),
                new ParallelCommandGroup(
                        new PIDToPose(leftColumn.plus(new Pose2d(column2, 0, 0)), l),
                        new GoToHeight(lift, claw, row2)
                ),
                new UpdateClaw(claw, ClawSubsystem.ClawState.OPEN)

        );
    }
}
