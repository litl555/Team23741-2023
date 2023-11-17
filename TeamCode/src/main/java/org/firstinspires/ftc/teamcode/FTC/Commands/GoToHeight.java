package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;

public class GoToHeight extends ParallelCommandGroup {
    public GoToHeight(LiftSubsystem lift, ClawSubsystem claw, int index) {
        addCommands(
                new UpdateLift(lift, index),
                new SetArm(claw, index),
                new SetWrist(claw, index)
        );
    }
}
