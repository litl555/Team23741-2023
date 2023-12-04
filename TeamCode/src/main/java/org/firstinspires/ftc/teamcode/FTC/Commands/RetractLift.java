package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class RetractLift extends ParallelCommandGroup {
    public RetractLift() {
        addCommands(
                new InstantCommand(() -> Robot.lift.setTargetPos(LiftSubsystem.safetyThreshold)),
                new SetArm(Robot.claw, 0),
                new SetWrist(Robot.claw, 0),
                new WaitCommand(1000),
                new InstantCommand(() -> Robot.lift.setTargetPos(0.0))

        );
    }
}
