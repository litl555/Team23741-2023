package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class PlacePixelOnBackboardF extends SequentialCommandGroup {
    public PlacePixelOnBackboardF(TeamPropPosition position) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveToBackBoardRed(position),
                        new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, Robot.level)
                ),
                new WaitCommand(2000),
                new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN)

        );
    }

    private int propToInt(TeamPropPosition prop) {
        switch (prop) {
            case left:
                return 1;

            case middle:
                return 2;

            case right:
                return 3;


        }
        return (1);
    }
}
