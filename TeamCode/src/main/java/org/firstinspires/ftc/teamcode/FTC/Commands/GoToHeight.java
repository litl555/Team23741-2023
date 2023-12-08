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
                new ConditionalCommand(new SequentialCommandGroup(
                        new UpdateClaw(claw, ClawSubsystem.ClawState.CLOSED), //close claw
                        new WaitCommand(AutonomousConstants.beforeLiftClearWait),
                        new InstantCommand(() -> lift.setTargetPos(AutonomousConstants.liftClearTicks)),//raise lift so we can move claw back

                        new WaitCommand(AutonomousConstants.afterLiftClearWait),
                        new InstantCommand(() -> Robot.claw.setWrist(AutonomousConstants.wristIntake + AutonomousConstants.wristClearDelta)),//move the wrist back
                        new WaitCommand(AutonomousConstants.waitAfterWristClear),
                        new UpdateLift(lift)),//raise lift to the level

                        new UpdateLift(lift), () -> {//if we're not in the tray just go to the right level
                    if (Robot.level == 0) {
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
