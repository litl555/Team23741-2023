package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants;

public class GoToHeight extends ParallelCommandGroup {
    public GoToHeight(LiftSubsystem lift, ClawSubsystem claw, int index) {
        addCommands(
                new ConditionalCommand(new SequentialCommandGroup(new UpdateClaw(claw, ClawSubsystem.ClawState.CLOSED), new InstantCommand(() -> Robot.claw.setWrist(TeleOpConstants.wristClearing)),
                        new WaitCommand(1000), new UpdateLift(lift, index)), new UpdateLift(lift, index), () -> {
                    if (LiftSubsystem.index1 == 0) {
                        return (true);
                    } else {
                        return (false);
                    }
                })

                ,
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> LiftSubsystem.safeRegion),
                        new SetArm(claw, index),
                        new SetWrist(claw, index)
                )
        );
    }
}
