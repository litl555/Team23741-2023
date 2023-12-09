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
    // this will automatically sync robot.level, so dont touch it!
    public GoToHeight(LiftSubsystem lift, ClawSubsystem claw, int newLevel) { // why are we passing these in? Robot.lift and Robot.claw?
        addCommands(
            new SequentialCommandGroup(
                // if our current level is 0, assume we have a pixel and try to grab it, then move to 1
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        //new UpdateClaw(Robot.claw, ClawSubsystem.ClawState.CLOSED), // close claw
                        //new WaitCommand(AutonomousConstants.beforeLiftClearWait),
                        //new InstantCommand(() -> lift.setTargetPos(AutonomousConstants.liftClearTicks)), // raise lift so we can move claw back
                        //new WaitCommand(AutonomousConstants.afterLiftClearWait),
                        //new InstantCommand(() -> Robot.claw.setWrist(AutonomousConstants.wristIntake + AutonomousConstants.wristClearDelta)),//move the wrist back
                        //new WaitCommand(AutonomousConstants.waitAfterWristClear),
                        new InstantCommand(() -> lift.updateRow(1))
                    ),
                    // if we are moving to 0, prepare to pick up a pixel
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            // do not assume there are no pixels, just in case
                            // close claw, move it until were in the tray, then open
                            new InstantCommand(() -> Robot.claw.update(ClawSubsystem.ClawState.CLOSED)),
                            new InstantCommand(() -> lift.updateRow(1)), // have lift be in tray, so we can open claw
                            new WaitCommand(1000),
                            //new WaitUntilCommand(() -> -lift.read() > LiftSubsystem.rowHeights[1] + 10), // hopefully lift is in tray -> 10 is the buffer since encoder != exactly, TODO: get a better threshold
                            new InstantCommand(() -> Robot.claw.update(ClawSubsystem.ClawState.OPEN)),
                            new WaitCommand(100), // let claw open
                            new InstantCommand(() -> lift.updateRow(newLevel))
                        ),
                        new InstantCommand(() -> lift.updateRow(newLevel)),
                        () -> newLevel == 0
                    ),
                    () -> Robot.level == 0
                ),
                new InstantCommand(() -> Robot.level = newLevel)

                // if we are trying any claw shenanigans wait until were in the safe region
                /*new SequentialCommandGroup(
                        new WaitUntilCommand(() -> LiftSubsystem.safeRegion),
                        new SetArm(claw, Robot.level),
                        new SetWrist(claw, Robot.level)
                ),*/
            )
        );
    }
}
