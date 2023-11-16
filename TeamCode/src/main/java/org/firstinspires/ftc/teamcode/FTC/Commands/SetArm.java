package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;

public class SetArm extends InstantCommand {
    public SetArm(ClawSubsystem arm, double angle) {
        super(() -> arm.setArm(angle));
    }
}
