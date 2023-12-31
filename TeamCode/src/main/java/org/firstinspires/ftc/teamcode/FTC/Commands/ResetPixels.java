package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;

/**
 * Reset number of pixels in intake
 */
public class ResetPixels extends InstantCommand {
    public ResetPixels(IntakeSubsystem intake) {
        super(
                () -> intake.resetPixel()
        );
    }
}
