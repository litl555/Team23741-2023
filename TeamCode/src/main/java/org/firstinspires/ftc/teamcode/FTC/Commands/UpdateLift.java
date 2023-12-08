package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class UpdateLift extends InstantCommand {
    public UpdateLift(LiftSubsystem lift) {
        super(
                () -> {
                    lift.updateRow(Robot.level);
                    Robot.telemetry.add("height", Robot.level);
                }
        );
    }
}
