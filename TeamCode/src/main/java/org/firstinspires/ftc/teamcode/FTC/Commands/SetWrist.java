package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;

public class SetWrist extends InstantCommand {
    public SetWrist(ClawSubsystem claw, int row) {
        super(
                () -> claw.updateWristRow(row)
        );
    }
}
