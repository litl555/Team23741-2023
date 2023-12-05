package org.firstinspires.ftc.teamcode.FTC.Commands.timing;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class liftUpTime extends ParallelCommandGroup {
    public liftUpTime(LiftSubsystem lift, ClawSubsystem claw, long time, double power) {
        addCommands(
                new SequentialCommandGroup(
                        // close claw
                        new UpdateClaw(claw, ClawSubsystem.ClawState.CLOSED),
                        //new InstantCommand(() -> Robot.claw.setWrist(ClawSubsystem.wristClearing)),
                        new WaitCommand(2_000),
                        // move lift up
                        new InstantCommand(() -> lift.setPower(power)),
                        new WaitCommand(time),
                        new InstantCommand(() -> lift.setPower(0))
                )
        );
    }
}
