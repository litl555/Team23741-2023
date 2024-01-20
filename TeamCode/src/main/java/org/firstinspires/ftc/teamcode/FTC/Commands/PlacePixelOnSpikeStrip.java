package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Board.DriveToSpikeStripRed;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class PlacePixelOnSpikeStrip extends SequentialCommandGroup {
    public PlacePixelOnSpikeStrip(TeamPropPosition pos) {
        addCommands(
                new ParallelCommandGroup(new DriveToSpikeStripRed(pos), new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, Robot.level)),
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE)
        );
    }
}
