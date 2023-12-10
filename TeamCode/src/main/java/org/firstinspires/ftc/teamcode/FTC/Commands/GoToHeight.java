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
import org.firstinspires.ftc.teamcode.FTC.TeleOp.ArmWristPos;
import org.firstinspires.ftc.teamcode.FTC.TeleOp.TeleOpConstants;

public class GoToHeight extends ParallelCommandGroup {
    // this will automatically sync robot.level, so dont touch it!
    public GoToHeight(LiftSubsystem lift, ClawSubsystem claw, int newLevel) { // why are we passing these in? Robot.lift and Robot.claw?
        /*
         * general structure
         * (1)
         * if (leaving from position 0)
         *      assume we have pixel, and run the pixel pickup process
         *
         * (2)
         * if (going to 1 or 0)
         *      (3)
         *      if (going to 0)
         *          open claw
         *
         *      wait until claw finishes
         *
         * (4)
         * tilt arm and wrist for intake position
         * move lift
         */

        /* pixel grabbing motion
         * l
         */

        addCommands(
            new SequentialCommandGroup(
                // 1
                new ConditionalCommand(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> ClawSubsystem.zero.override(claw)),
                            new WaitCommand(500),
                            new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)),
                            new WaitCommand(500),
                            new InstantCommand(() -> {
                                lift.maxPower = 0.25;
                                ClawSubsystem.clearPixelIntake.apply(claw);
                                lift.setTargetPos(350);
                            }),
                            new WaitUntilCommand(lift.pid::atSetPoint),
                            new WaitCommand(200),
                            new InstantCommand(() -> lift.maxPower = 0.8)
                    ),
                    new InstantCommand(),
                    () -> Robot.level == 0
                ),

                // 2
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // if were going to 1, close
                        new ConditionalCommand(
                            new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)),
                            new InstantCommand(),
                            () -> newLevel == 1
                        ),

                        // 3
                        new ConditionalCommand( // were at one are were trying to go to 0, so open the claw since were now in the tray
                            new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPEN)),
                            new InstantCommand(),
                            () -> (newLevel == 0 && Robot.level == 1)
                        ),

                        new WaitCommand(400) // hopefully this allows claw to move
                    ),
                    new InstantCommand(),
                    () -> (newLevel == 0 || newLevel == 1)
                ),

                // 4
                new InstantCommand(() -> {
                    // hack, if were picking up pixel we dont want to stop at top of tray
                    if (Robot.level == 0 && newLevel == 1) claw.updateArmWristPos(2);
                    else claw.updateArmWristPos(newLevel);
                }),
                new ConditionalCommand(
                    new WaitCommand(400),
                    new InstantCommand(),
                    () -> Robot.level < 2 || newLevel < 2
                ),
                new InstantCommand(() -> {
                    if (Robot.level == 0 && newLevel == 1) {
                        lift.updateRow(2);
                        Robot.level = 2;
                    } else {
                        lift.updateRow(newLevel);
                        Robot.level = newLevel;
                    }
                })
            )
        );
    }
}
