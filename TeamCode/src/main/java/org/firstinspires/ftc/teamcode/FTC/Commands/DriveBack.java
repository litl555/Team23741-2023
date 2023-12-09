package org.firstinspires.ftc.teamcode.FTC.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.FTC.Localization.Constants;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.FTC.Subsystems.Robot;

public class DriveBack extends ParallelCommandGroup {
    public DriveBack(IntakeSubsystem intake, LiftSubsystem lift, ClawSubsystem claw) {
        addCommands(
                new WaitUntilCommand(() -> Constants.robotPose.getX() < 0),
                new ParallelCommandGroup(
                        new UpdateIntake(intake, IntakeSubsystem.IntakePowerSetting.INTAKE),
                        new GoToHeight(lift, claw, Robot.level)
                )
        );
    }
}
