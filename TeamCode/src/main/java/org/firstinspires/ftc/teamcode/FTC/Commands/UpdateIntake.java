package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;

public class UpdateIntake extends InstantCommand {
    public UpdateIntake(IntakeSubsystem intake, IntakeSubsystem.IntakePowerSetting power) {
        super(
                () -> intake.update(power)
        );
        addRequirements(intake);
    }
}
