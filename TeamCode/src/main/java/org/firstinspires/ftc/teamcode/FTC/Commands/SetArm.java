package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;

public class SetArm extends InstantCommand {
    public SetArm(ClawSubsystem arm, int row) {
        super(() -> arm.updateArmRow(row));
    }
}
