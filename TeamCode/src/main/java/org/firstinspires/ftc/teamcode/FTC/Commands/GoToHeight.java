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
    private ClawSubsystem.ClawState pickupStateOverride = ClawSubsystem.ClawState.CLOSED;
    public GoToHeight(LiftSubsystem lift, ClawSubsystem claw, int newLevel) {
        initialize(lift, claw, newLevel);
    }
    public GoToHeight(LiftSubsystem lift, ClawSubsystem claw, int newLevel, ClawSubsystem.ClawState pickupStateOverride) {
        // java doesnt have default parameters??????????????
        this.pickupStateOverride = pickupStateOverride;
        initialize(lift, claw, newLevel);
    }

    // this will automatically sync robot.level, so dont touch it!
    private void initialize(LiftSubsystem lift, ClawSubsystem claw, int newLevel) {
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
                        // grab pixel
                        new InstantCommand(() -> ClawSubsystem.pickZero.apply(claw)),
                        new WaitCommand(400),
                        new InstantCommand(() -> claw.update(pickupStateOverride)),
                        new WaitCommand(400),

                        // extract pixel
                        new InstantCommand(() -> {
                            lift.maxPower = 0.35;
                            lift.setTargetPos(350);
                        }),
                        new WaitCommand(50),
                        new InstantCommand(() -> ClawSubsystem.clearPixelIntake.apply(claw)),
                        new WaitUntilCommand(lift.pid::atSetPoint),

                        // reset lift power to allow next command
                        new InstantCommand(() -> lift.maxPower = 1)
                    ),
                    new InstantCommand(),
                    () -> Robot.level == 0
                ),
                // 2
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // if were going to 1, close
                        new ConditionalCommand(
                            new InstantCommand(() -> {
                                claw.update(ClawSubsystem.ClawState.CLOSED);
                                lift.maxPower = 0.7;
                            }),
                            new InstantCommand(),
                            () -> newLevel == 1
                        ),

                        // 3
                        new ConditionalCommand( // were at one are were trying to go to 0, so open the claw since were now in the tray
                            new SequentialCommandGroup(
                                new InstantCommand(() -> claw.update(ClawSubsystem.ClawState.OPEN)),
                                new WaitCommand(400)),
                            new InstantCommand(),
                            () -> (newLevel == 0 && Robot.level == 1)
                        )
                    ),
                    new InstantCommand(),
                    () -> (newLevel == 0 || newLevel == 1)
                ),
                // 4
                new InstantCommand(() -> {
                    // if were picking up pixel we dont want to stop at top of tray
                    if (Robot.level == 0 && newLevel == 1) claw.updateArmWristPos(2);
                    else claw.updateArmWristPos(newLevel);
                }),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        new WaitCommand(600),
                        new InstantCommand(() -> lift.maxPower = 1)),
                    new InstantCommand(),
                    () -> Robot.level >= 2 && newLevel < 2
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
