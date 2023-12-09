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
        /*
         * general structure
         * (1)
         * if (leaving from position 0)
         *      assume we have pixel, and run the pixel pickup process
         *
         * (2)
         * if (going to 1 or 0)
         *      (3)
         *      if (going from not 0) // going down into the tray
         *          close claw
         *
         *      (4)
         *      if (going to 0)
         *          open claw
         *
         *      wait until claw finishes
         *
         * (5)
         * tilt arm and wrist for intake position
         * move lift
         */

        addCommands(
            new SequentialCommandGroup(
                // 1
                new ConditionalCommand(
                    new SequentialCommandGroup(),
                    new InstantCommand(), // TODO
                    () -> Robot.level == 0
                ),

                // 2
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // 3
                        new ConditionalCommand(
                            new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.CLOSED)),
                            new InstantCommand(),
                            () -> Robot.level != 0
                        ),

                        // 4
                        new ConditionalCommand(
                            new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPEN)),
                            new InstantCommand(),
                            () -> (newLevel == 0 && Robot.level == 1)
                        ),

                        new WaitCommand(500) // hopefully this allows claw to move
                    ),
                    new InstantCommand(),
                    () -> (newLevel == 0 || newLevel == 1)
                ),

                // 5
                new InstantCommand(() -> lift.updateRow(newLevel)),
                new InstantCommand(() -> claw.updateArmWristPos(newLevel)),
                new InstantCommand(() -> Robot.level = newLevel)
            )
        );
    }
}
