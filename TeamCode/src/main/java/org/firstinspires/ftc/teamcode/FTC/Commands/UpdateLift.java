package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;

public class UpdateLift extends InstantCommand {
    public UpdateLift(LiftSubsystem lift, int height) {
        super(
                () -> lift.updateRow(height)
        );
    }
}
