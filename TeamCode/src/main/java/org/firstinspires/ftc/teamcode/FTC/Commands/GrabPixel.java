package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;

public class GrabPixel extends InstantCommand {
    public GrabPixel(ClawSubsystem claw) {
        super(
                () -> claw.update(ClawSubsystem.ClawState.CLOSED)
        );
    }
}
