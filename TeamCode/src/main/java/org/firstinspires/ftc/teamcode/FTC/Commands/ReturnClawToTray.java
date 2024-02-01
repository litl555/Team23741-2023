package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class ReturnClawToTray extends SequentialCommandGroup {
    public ReturnClawToTray() {
        addCommands(
            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 1),
            new WaitCommand(250),
            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 0)
        );
    }

    public ReturnClawToTray(long delay) {
        addCommands(
            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 1),
            new WaitCommand(delay),
            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 0)
        );
    }
}
