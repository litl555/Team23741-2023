package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;

public class UpdateClaw extends InstantCommand {
    public UpdateClaw(ClawSubsystem claw, ClawSubsystem.ClawState state) {
        super(
                () -> claw.update(state)
        );
    }
}
