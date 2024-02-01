package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class RamBoard extends SequentialCommandGroup {
    public RamBoard() {
        addCommands(
            new InstantCommand(() -> Robot.customLocalization.setWeightedDrivePowers(new Pose2d(0.0, 0.20, 0))),
            //new WaitCommand(750),
            new WaitCommand(10_000),
            new InstantCommand(() -> Robot.customLocalization.setWeightedDrivePowers(new Pose2d(0, 0, 0)))
        );
    }
}
