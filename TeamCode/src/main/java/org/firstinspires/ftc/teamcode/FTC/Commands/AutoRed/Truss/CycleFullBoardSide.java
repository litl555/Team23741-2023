package org.firstinspires.ftc.teamcode.FTC.Commands.AutoRed.Truss;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.FTC.Autonomous.TeamPropPosition;
import org.firstinspires.ftc.teamcode.FTC.Commands.GoToHeight;
import org.firstinspires.ftc.teamcode.FTC.Commands.IntakePixelFromStack;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamBoard;
import org.firstinspires.ftc.teamcode.FTC.Commands.RamIntake;
import org.firstinspires.ftc.teamcode.FTC.Commands.UpdateClaw;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class CycleFullBoardSide extends SequentialCommandGroup {
    public CycleFullBoardSide(TeamPropPosition last, int iter) {
        addCommands(
            new ParallelCommandGroup(
                // reset lift
                new SequentialCommandGroup(
                    new WaitCommand(1_000),
                    new SequentialCommandGroup(
                        new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 1),
                        new WaitCommand(200),
                        new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 0))),
                new DriveToTrussCycle(last).interruptOn(() -> Robot.liftSubsystem.targetPos == Robot.liftSubsystem.rowHeights[0])),
            new DriveToStackCycle(last),
            new ParallelCommandGroup(
                new RamIntake(),
                new IntakePixelFromStack(2, 2000, 4)
            ),
            new ParallelCommandGroup(
                // clean up intake
                new SequentialCommandGroup(
                    new InstantCommand(() -> Robot.intakeSubsystem.setPower(1)),
                    new WaitCommand(500),
                    new InstantCommand(() -> {
                        Robot.intakeSubsystem.setPower(0);
                        Robot.intakeSubsystem.setDroptakePosition(IntakeSubsystem.droptakeLevel[6]);
                    })
                ),
                new DriveToBackBoardRedTruss(last, iter),
                new SequentialCommandGroup(
                    new WaitCommand(2_500),
                    new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 2)
                )
            ),
            new GoToHeight(Robot.liftSubsystem, Robot.clawSubsystem, 3),
            new RamBoard(),

            new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPENONE),
            new WaitCommand(250),
            new InstantCommand(() -> Robot.clawSubsystem.setWrist(ClawSubsystem.zero.wrist + 0.083333 + 0.02)),
            new WaitCommand(500),
            new UpdateClaw(Robot.clawSubsystem, ClawSubsystem.ClawState.OPEN)
        );
    }
}
