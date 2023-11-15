package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;

public class ResetPixels extends InstantCommand {
    public ResetPixels(IntakeSubsystem intake) {
        super(
                () -> intake.resetPixel()
        );
    }
}
